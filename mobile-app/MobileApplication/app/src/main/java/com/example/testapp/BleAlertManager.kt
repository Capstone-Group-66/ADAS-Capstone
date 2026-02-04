package com.example.testapp

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import androidx.annotation.RequiresApi
import androidx.annotation.RequiresPermission
import com.example.testapp.model.GpsData
import com.example.testapp.model.SerializationDeserialization
import kotlinx.coroutines.channels.BufferOverflow
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.serialization.encodeToByteArray
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.time.LocalTime
import java.time.format.DateTimeFormatter
import java.util.UUID

/**
 * BLE Manager
 *
 * Handles:
 * - Scanning & connection to Jetson Nano BLE peripheral
 * - Receiving BLE alert fragments
 * - Reassembling fragments into TickPayload CBOR
 * - Managing tick order, TTL, MTU changes
 * - Buffering payloads for reconnect
 *
 * + TX:
 * - Sending GPS data as CBOR over BLE (chunked, MTU-safe)
 */
class BleManager(
    private val context: Context,
) {
    // BLE Constants - Official ADAS UUIDs (must match jetson-core/BleUuids.hpp)
    private val ADAS_SERVICE_UUID: UUID =
        UUID.fromString("0000ada5-0000-1000-8000-00805f9b34fb")

    // RX: Jetson -> Android notifications (your existing stream)
    private val ADAS_ALERT_STREAM_UUID: UUID =
        UUID.fromString("0000a1e7-0000-1000-8000-00805f9b34fb")

    // TX: Android -> Jetson writes (YOU MUST SET THIS TO THE REAL UUID ON JETSON)
    // If Jetson uses a separate characteristic for inbound commands/GPS, put it here.
    private val ADAS_GPS_TX_UUID: UUID =
        UUID.fromString("0000a2e7-0000-1000-8000-00805f9b34fb") // TODO: replace with real TX UUID

    // Jetson peripheral reference
    private var bluetoothGatt: BluetoothGatt? = null

    // Latest ATT MTU (default 23)
    private var currentMtu: Int = 23

    // Expose data stream
    private val _packetFlow =
        MutableSharedFlow<ByteArray>(
            replay = 0,
            extraBufferCapacity = 64,
            onBufferOverflow = BufferOverflow.DROP_OLDEST,
        )
    val packetFlow: Flow<ByteArray> = _packetFlow

    // Expose status logs
    private val _logFlow = MutableStateFlow<List<String>>(emptyList())
    val logFlow: StateFlow<List<String>> = _logFlow

    // Connection state
    private val _connectionState = MutableStateFlow("Disconnected")
    val connectionState: StateFlow<String> = _connectionState

    private var bluetoothAdapter: android.bluetooth.BluetoothAdapter? = null
    private var bluetoothLeScanner: android.bluetooth.le.BluetoothLeScanner? = null

    // Scan retry logic
    private val handler = Handler(Looper.getMainLooper())
    private var isScanning = false
    private val SCAN_RETRY_DELAY_MS = 2000L
    private val SCAN_TIMEOUT_MS = 5000L

    // CCCD for notifications
    private val CCCD_UUID: UUID =
        UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    // Cached characteristics
    private var alertStreamChar: BluetoothGattCharacteristic? = null
    private var gpsTxChar: BluetoothGattCharacteristic? = null

    private fun log(msg: String) {
        val list = _logFlow.value.toMutableList()
        val time = DateTimeFormatter.ofPattern("HH:mm:ss").format(LocalTime.now())
        list.add(0, "$time $msg")
        if (list.size > 100) list.removeAt(list.lastIndex)
        _logFlow.value = list
        android.util.Log.d("BleManager", msg)
    }

    // Computed slice_cap = MTU - 7 (used for RX sizing conceptually; Jetson controls RX frag size)
    private val sliceCap: Int
        get() = (currentMtu - 7).coerceAtLeast(1)

    // For TX: ATT payload usable is generally MTU - 3
    private fun attPayloadMax(): Int = (currentMtu - 3).coerceAtLeast(20)

    // Active tick currently being rebuilt
    private data class InProgressTick(
        val tickId: Int,
        var seqMax: Int,
        val fragments: MutableMap<Int, ByteArray> = mutableMapOf(),
        var receivedCount: Int = 0,
    )

    private var activeTick: InProgressTick? = null

    // Ring buffer for reconnect resend (placeholders)
    private val reconnectRing = ArrayDeque<ByteArray>()

    // BLE Header Definition (4 bytes)
    // Matches struct BLEHeader in Jetson code.
    data class BLEHeader(
        val tickId: Int,
        val seqNo: Int,
        val seqMax: Int,
    ) {
        companion object {
            private const val HEADER_SIZE = 4

            /**
             * Parse 4-byte header from BLE notification.
             * Byte layout:
             *   uint16 tick_id (LE)
             *   uint8  seq_no
             *   uint8  seq_max
             */
            fun fromBytes(bytes: ByteArray): BLEHeader? {
                if (bytes.size < HEADER_SIZE) return null
                val bb = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)
                val tickId = bb.short.toInt() and 0xFFFF
                val seqNo = bb.get().toInt() and 0xFF
                val seqMax = bb.get().toInt() and 0xFF
                return BLEHeader(tickId, seqNo, seqMax)
            }
        }
    }

    // Initialize BLE stack
    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun initialize() {
        val bluetoothManager =
            context.getSystemService(Context.BLUETOOTH_SERVICE) as android.bluetooth.BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner
        scanForJetson()
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    private fun scanForJetson() {
        if (bluetoothLeScanner == null) {
            log("Bluetooth not supported or disabled")
            scheduleRetry()
            return
        }

        if (isScanning) {
            log("Already scanning...")
            return
        }

        log("Starting scan for ADAS-Jetson...")
        _connectionState.value = "Scanning..."
        isScanning = true

        val filter =
            ScanFilter.Builder()
                .setServiceUuid(android.os.ParcelUuid(ADAS_SERVICE_UUID))
                .build()

        val settings =
            ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()

        bluetoothLeScanner?.startScan(listOf(filter), settings, scanCallback)

        // timeout -> retry
        handler.postDelayed({
            if (isScanning && bluetoothGatt == null) {
                log("Scan timeout, retrying...")
                stopScanAndRetry()
            }
        }, SCAN_TIMEOUT_MS)
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    private fun stopScanAndRetry() {
        try {
            bluetoothLeScanner?.stopScan(scanCallback)
        } catch (_: SecurityException) {
        }
        isScanning = false
        scheduleRetry()
    }

    private fun scheduleRetry() {
        if (bluetoothGatt != null) return
        _connectionState.value = "Retrying in ${SCAN_RETRY_DELAY_MS / 1000}s..."
        handler.postDelayed({
            if (bluetoothGatt == null) {
                try {
                    scanForJetson()
                } catch (e: SecurityException) {
                    log("Permission error: ${e.message}")
                }
            }
        }, SCAN_RETRY_DELAY_MS)
    }

    private val scanCallback =
        object : android.bluetooth.le.ScanCallback() {
            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onScanResult(
                callbackType: Int,
                result: android.bluetooth.le.ScanResult?,
            ) {
                result?.device?.let { device ->
                    log("Found device: ${device.name} (${device.address})")
                    if (bluetoothGatt == null) {
                        try {
                            bluetoothLeScanner?.stopScan(this)
                        } catch (_: SecurityException) {
                        }
                        isScanning = false
                        connectToJetson(device)
                    }
                }
            }

            override fun onScanFailed(errorCode: Int) {
                log("Scan failed: $errorCode")
                isScanning = false
                _connectionState.value = "Scan Failed: $errorCode"
                scheduleRetry()
            }
        }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun connectToJetson(device: BluetoothDevice) {
        log("Connecting to ${device.address}...")
        _connectionState.value = "Connecting..."
        bluetoothGatt =
            device.connectGatt(
                context,
                false,
                gattCallback,
                BluetoothDevice.TRANSPORT_LE,
            )
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        alertStreamChar = null
        gpsTxChar = null
        _connectionState.value = "Disconnected"
    }

    // BLE GATT Callback (core of receiving alerts)
    private val gattCallback =
        object : BluetoothGattCallback() {
            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onConnectionStateChange(
                gatt: BluetoothGatt?,
                status: Int,
                newState: Int,
            ) {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    log("Connected to GATT server.")
                    _connectionState.value = "Connected"
                    gatt?.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    log("Disconnected from GATT server.")
                    _connectionState.value = "Disconnected"
                    bluetoothGatt = null
                    alertStreamChar = null
                    gpsTxChar = null
                    // You can call scheduleRetry() here if you want auto-reconnect via scan
                }
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onServicesDiscovered(
                gatt: BluetoothGatt?,
                status: Int,
            ) {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    log("Service discovery failed: $status")
                    return
                }

                log("Services discovered.")
                val service = gatt?.getService(ADAS_SERVICE_UUID)
                if (service == null) {
                    log("ADAS Service not found!")
                    return
                }

                // RX characteristic
                val rx = service.getCharacteristic(ADAS_ALERT_STREAM_UUID)
                if (rx == null) {
                    log("AlertStream characteristic not found!")
                } else {
                    alertStreamChar = rx
                    gatt.setCharacteristicNotification(rx, true)

                    val descriptor = rx.getDescriptor(CCCD_UUID)
                    if (descriptor != null) {
                        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                        gatt.writeDescriptor(descriptor)
                        log("Subscribing to AlertStream...")
                    } else {
                        log("CCCD descriptor not found for AlertStream")
                    }
                }

                // TX characteristic
                val tx = service.getCharacteristic(ADAS_GPS_TX_UUID)
                if (tx != null) {
                    gpsTxChar = tx
                    log("GPS TX characteristic found.")
                } else {
                    log("GPS TX characteristic not found! (check ADAS_GPS_TX_UUID)")
                }

                // Request MTU (you can change to 247/185 depending on Jetson stack)
                gatt.requestMtu(185)
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onDescriptorWrite(
                gatt: BluetoothGatt?,
                descriptor: BluetoothGattDescriptor?,
                status: Int,
            ) {
                if (descriptor?.uuid == CCCD_UUID) {
                    if (status == BluetoothGatt.GATT_SUCCESS) {
                        log("Subscribed to AlertStream.")
                    } else {
                        log("Failed to subscribe (CCCD write): $status")
                    }
                }
            }

            // whenever the BLE packet size changes, update slice size
            override fun onMtuChanged(
                gatt: BluetoothGatt?,
                mtu: Int,
                status: Int,
            ) {
                if (status == BluetoothGatt.GATT_SUCCESS) {
                    currentMtu = mtu
                    log("MTU changed to $mtu (attPayloadMax=${attPayloadMax()}, sliceCap=$sliceCap)")
                } else {
                    log("MTU change failed: $status (mtu=$mtu)")
                }
            }

            // whenever a fragment arrives from Jetson, process here
            @RequiresApi(Build.VERSION_CODES.O)
            override fun onCharacteristicChanged(
                gatt: BluetoothGatt?,
                characteristic: BluetoothGattCharacteristic?,
            ) {
                if (characteristic?.uuid == ADAS_ALERT_STREAM_UUID) {
                    val raw = characteristic.value ?: return
                    handleIncomingFragment(raw)
                }
            }
        }

    // Fragment Handler — central logic

    /**
     * Handles:
     * 1. Read the header
     * 2. Determine which tick the fragment belongs to
     * 3. Store fragment in the right place
     * 4. When all fragments arrive, rebuild full payload
     * 5. Send complete CBOR TickPayload to the app/UI
     */
    @RequiresApi(Build.VERSION_CODES.O)
    private fun handleIncomingFragment(data: ByteArray) {
        val header = BLEHeader.fromBytes(data) ?: return
        if (data.size <= 4) return

        val slice = data.copyOfRange(4, data.size)

        // Ensure activeTick exists and matches tick_id, else reset
        if (activeTick == null || activeTick?.tickId != header.tickId) {
            activeTick =
                InProgressTick(
                    tickId = header.tickId,
                    seqMax = header.seqMax,
                )
        }

        val tick = activeTick ?: return

        if (!tick.fragments.containsKey(header.seqNo)) {
            tick.fragments[header.seqNo] = slice
            tick.receivedCount++
        }

        // If complete → reassemble full CBOR payload
        if (tick.receivedCount == (tick.seqMax + 1)) {
            val full = rebuildFullPayload(tick)
            if (full != null) {
                onFullTickPayloadReceived(tick.tickId, full)
            }
            activeTick = null
        }
    }

    // Rebuild CBOR TickPayload from fragments
    private fun rebuildFullPayload(tick: InProgressTick): ByteArray? {
        val size = tick.fragments.values.sumOf { it.size }
        val buffer = ByteBuffer.allocate(size)

        for (i in 0..tick.seqMax) {
            val frag = tick.fragments[i] ?: return null
            buffer.put(frag)
        }
        return buffer.array()
    }

    // Called when one full tick CBOR buffer has been reassembled
    private fun onFullTickPayloadReceived(
        tickId: Int,
        cborBuffer: ByteArray,
    ) {
        log("Received full tick payload: $tickId (${cborBuffer.size} bytes)")

        // Debug: Decode and log the payload contents
        try {
            val payload =
                com.example.testapp.model.TickDecoder.decode(cborBuffer)

            log("=== FCW DEBUG ===")
            log("  Tick ID: ${payload.tickId}")
            log("  Speed: ${payload.speed} km/h")
            log("  Health Mask: ${payload.healthMask}")
            log("  BSD Mask: ${payload.bsdMask}")
            log("  Alerts: ${payload.alerts.size}")

            payload.alerts.forEachIndexed { idx, alert ->
                val typeStr =
                    when (alert.type) {
                        0 -> "FCW"
                        1 -> "LDW"
                        2 -> "RCW"
                        3 -> "BSD"
                        else -> "UNKNOWN(${alert.type})"
                    }
                val sevStr =
                    when (alert.severity) {
                        0 -> "Info"
                        1 -> "Warning"
                        2 -> "Critical"
                        else -> "Unknown(${alert.severity})"
                    }
                log("  Alert[$idx]: type=$typeStr severity=$sevStr")
                log("    rationale: ${alert.rationale}")
            }
            log("=================")
        } catch (e: Exception) {
            log("DEBUG decode error: ${e.message}")
        }

        _packetFlow.tryEmit(cborBuffer)
    }

    // Resend logic after reconnect
    private fun resendRingBuffer() {
        // TODO: Iterate through reconnectRing and send CBOR if still valid (TTL)
    }

    // Placeholder for writing data to the Jetson (generic)
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun sendCommand(cmd: ByteArray) {
        val gatt =
            bluetoothGatt ?: run {
                log("sendCommand: not connected")
                return
            }
        val tx =
            gpsTxChar ?: run {
                log("sendCommand: gpsTxChar not ready (or wrong UUID)")
                return
            }

        tx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        tx.value = cmd

        val ok = gatt.writeCharacteristic(tx)
        if (!ok) log("sendCommand: writeCharacteristic returned false")
    }

    // TX: Send GPS data (CBOR) to Jetson (chunked, MTU-safe)
    private var gpsSeq: Int = 0

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun sendGpsData(data: GpsData) {
        val gatt =
            bluetoothGatt ?: run {
                log("sendGpsData: not connected")
                return
            }
        val tx =
            gpsTxChar ?: run {
                log("sendGpsData: gpsTxChar not ready (check ADAS_GPS_TX_UUID / discovery)")
                return
            }

        val payload: ByteArray =
            SerializationDeserialization.cbor.encodeToByteArray(data)

        gpsSeq = (gpsSeq + 1) and 0xFFFF
        val seq = gpsSeq

        // framing:
        // first chunk: type(1) + seq(2) + totalLen(2) + data...
        // next chunks: type(1) + seq(2) + data...
        val type: Byte = 0x01
        val maxPayload = attPayloadMax()

        var offset = 0
        var first = true

        while (offset < payload.size) {
            val header: ByteArray =
                if (first) {
                    byteArrayOf(
                        type,
                        (seq shr 8).toByte(),
                        seq.toByte(),
                        (payload.size shr 8).toByte(),
                        payload.size.toByte(),
                    )
                } else {
                    byteArrayOf(
                        type,
                        (seq shr 8).toByte(),
                        seq.toByte(),
                    )
                }

            val room = maxPayload - header.size
            if (room <= 0) {
                log("sendGpsData: MTU too small for header (mtu=$currentMtu)")
                return
            }

            val take = minOf(room, payload.size - offset)
            val packet = ByteArray(header.size + take)
            System.arraycopy(header, 0, packet, 0, header.size)
            System.arraycopy(payload, offset, packet, header.size, take)

            tx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
            tx.value = packet

            val ok = gatt.writeCharacteristic(tx)
            if (!ok) {
                log("sendGpsData: writeCharacteristic returned false")
                return
            }

            offset += take
            first = false
        }

        log("sendGpsData: sent ${payload.size}B CBOR (seq=$seq, mtu=$currentMtu)")
    }

    // Asks Android to negotiate a larger BLE MTU
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun requestMtu(mtu: Int) {
        bluetoothGatt?.requestMtu(mtu)
    }
}
