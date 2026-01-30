package com.example.testapp

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
<<<<<<< HEAD
import android.bluetooth.le.ScanFilter
=======
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
>>>>>>> origin/main
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.ParcelUuid
import androidx.annotation.RequiresPermission
import com.example.testapp.model.SerializationDeserialization
import com.example.testapp.model.TickStreamDecoder
import com.example.testapp.model.TickStreamPayload
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.asSharedFlow
import java.nio.ByteBuffer
import java.nio.ByteOrder
<<<<<<< HEAD
import java.time.LocalTime
import java.time.format.DateTimeFormatter
=======
import java.util.UUID
>>>>>>> origin/main

class BleManager(
    private val context: Context,
    private val serde: SerializationDeserialization,
) {
<<<<<<< HEAD
    // BLE Constants - Official ADAS UUIDs (must match jetson-core/BleUuids.hpp)
    private val ADAS_SERVICE_UUID = java.util.UUID.fromString("0000ada5-0000-1000-8000-00805f9b34fb")
    private val ADAS_ALERT_STREAM_UUID = java.util.UUID.fromString("0000a1e7-0000-1000-8000-00805f9b34fb")
=======
    private val ADAS_SERVICE_UUID: UUID =
        UUID.fromString("0000ada5-0000-1000-8000-00805f9b34fb")
>>>>>>> origin/main

    private val ADAS_ALERT_STREAM_UUID: UUID =
        UUID.fromString("0000a1e7-0000-1000-8000-00805f9b34fb")

    private val CCCD_UUID: UUID =
        UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    private val bluetoothAdapter: BluetoothAdapter by lazy {
        val mgr = context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        mgr.adapter
    }

    private val scanner: BluetoothLeScanner?
        get() = bluetoothAdapter.bluetoothLeScanner

    private var scanCallback: ScanCallback? = null
    private var bluetoothGatt: BluetoothGatt? = null

    private val _blePackets = MutableSharedFlow<ByteArray>(extraBufferCapacity = 128)
    val blePackets: SharedFlow<ByteArray> = _blePackets.asSharedFlow()

<<<<<<< HEAD
    // Expose data stream
    private val _packetFlow =
        kotlinx.coroutines.flow.MutableSharedFlow<ByteArray>(
            replay = 0,
            extraBufferCapacity = 64,
            onBufferOverflow = kotlinx.coroutines.channels.BufferOverflow.DROP_OLDEST,
        )
    val packetFlow: kotlinx.coroutines.flow.Flow<ByteArray> = _packetFlow

    // Expose status logs
    private val _logFlow = kotlinx.coroutines.flow.MutableStateFlow<List<String>>(emptyList())
    val logFlow: kotlinx.coroutines.flow.StateFlow<List<String>> = _logFlow

    // Connection state
    private val _connectionState = kotlinx.coroutines.flow.MutableStateFlow("Disconnected")
    val connectionState: kotlinx.coroutines.flow.StateFlow<String> = _connectionState

    private var bluetoothAdapter: android.bluetooth.BluetoothAdapter? = null
    private var bluetoothLeScanner: android.bluetooth.le.BluetoothLeScanner? = null

    // Scan retry logic
    private val handler = android.os.Handler(android.os.Looper.getMainLooper())
    private var isScanning = false
    private val SCAN_RETRY_DELAY_MS = 2000L
    private val SCAN_TIMEOUT_MS = 5000L

    private fun log(msg: String) {
        val list = _logFlow.value.toMutableList()
        // Add timestamped log
        val time = DateTimeFormatter.ofPattern("HH:mm:ss").format(LocalTime.now())
        list.add(0, "$time $msg")
        if (list.size > 100) list.removeAt(list.lastIndex)
        _logFlow.value = list
        android.util.Log.d("BleManager", msg)
    }

    // Computed slice_cap = MTU - 7
    private val sliceCap: Int
        get() = (currentMtu - 7).coerceAtLeast(1)
=======
    private val tickStreamDecoder = TickStreamDecoder(serde)
    private val _ticks = MutableSharedFlow<TickStreamPayload>(extraBufferCapacity = 128)
    val ticks: SharedFlow<TickStreamPayload> = _ticks.asSharedFlow()
>>>>>>> origin/main

    private data class InProgressTick(
        val tickId: Int,
        val seqMax: Int,
        val fragments: MutableMap<Int, ByteArray> = mutableMapOf(),
        var receivedCount: Int = 0,
    )

    private var activeTick: InProgressTick? = null

    data class BLEHeader(
        val tickId: Int,
        val seqNo: Int,
        val seqMax: Int,
    ) {
        companion object {
            private const val HEADER_SIZE = 4

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

<<<<<<< HEAD
    // Initialize BLE stack
    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun initialize() {
        val bluetoothManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as android.bluetooth.BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner
        scanForJetson()
=======
    @RequiresPermission(allOf = [Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT])
    fun startScan(onDeviceFound: (BluetoothDevice) -> Unit) {
        if (scanCallback != null) return

        val filter =
            ScanFilter
                .Builder()
                .setServiceUuid(ParcelUuid(ADAS_SERVICE_UUID))
                .build()

        val settings =
            ScanSettings
                .Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()

        scanCallback =
            object : ScanCallback() {
                @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
                override fun onScanResult(
                    callbackType: Int,
                    result: ScanResult,
                ) {
                    val device = result.device ?: return
                    stopScan()
                    onDeviceFound(device)
                }

                override fun onScanFailed(errorCode: Int) {
                    scanCallback = null
                }
            }

        scanner?.startScan(listOf(filter), settings, scanCallback)
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun stopScan() {
        val cb = scanCallback ?: return
        scanner?.stopScan(cb)
        scanCallback = null
>>>>>>> origin/main
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
            ScanFilter
                .Builder()
                .setServiceUuid(android.os.ParcelUuid(ADAS_SERVICE_UUID))
                .build()

        val settings =
            ScanSettings
                .Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()

        bluetoothLeScanner?.startScan(listOf(filter), settings, scanCallback)

        // Schedule timeout - if nothing found, stop and retry
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
        } catch (e: SecurityException) {
            // ignore
        }
        isScanning = false
        scheduleRetry()
    }

    private fun scheduleRetry() {
        if (bluetoothGatt != null) return // Already connected
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
                        } catch (e: SecurityException) {
                            // ignore
                        }
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
<<<<<<< HEAD
        log("Connecting to ${device.address}...")
        _connectionState.value = "Connecting..."
        bluetoothGatt =
            device.connectGatt(
                context,
                false,
                gattCallback,
                BluetoothDevice.TRANSPORT_LE,
            )
=======
        bluetoothGatt?.close()
        bluetoothGatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
>>>>>>> origin/main
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        activeTick = null
    }

<<<<<<< HEAD
    // BLE GATT Callback (core of receiving alerts)
    private val gattCallback =
        object : BluetoothGattCallback() {
            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onConnectionStateChange(
                gatt: BluetoothGatt?,
                status: Int,
                newState: Int,
            ) {
                if (newState == android.bluetooth.BluetoothProfile.STATE_CONNECTED) {
                    log("Connected to GATT server.")
                    _connectionState.value = "Connected"
                    gatt?.discoverServices()
                } else if (newState == android.bluetooth.BluetoothProfile.STATE_DISCONNECTED) {
                    log("Disconnected from GATT server.")
                    _connectionState.value = "Disconnected"
                    bluetoothGatt = null
                    // Retry scan after delay? For now just log
                }
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onServicesDiscovered(
                gatt: BluetoothGatt?,
                status: Int,
            ) {
                if (status == BluetoothGatt.GATT_SUCCESS) {
                    log("Services discovered.")
                    val service = gatt?.getService(ADAS_SERVICE_UUID)
                    if (service != null) {
                        val characteristic = service.getCharacteristic(ADAS_ALERT_STREAM_UUID)
                        if (characteristic != null) {
                            gatt.setCharacteristicNotification(characteristic, true)
                            val descriptor = characteristic.getDescriptor(java.util.UUID.fromString("00002902-0000-1000-8000-00805f9b34fb"))
                            if (descriptor != null) {
                                descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                                gatt.writeDescriptor(descriptor)
                                log("Subscribed to AlertStream.")
                            } else {
                                log("CCCD descriptor not found")
                            }

                            // Request MTU
                            gatt.requestMtu(185)
                        } else {
                            log("AlertStream characteristic not found!")
                        }
                    } else {
                        log("ADAS Service not found!")
                    }
                } else {
                    log("Service discovery failed: $status")
                }
            }

            // whenever the BLE packet size changes, update slice size
            override fun onMtuChanged(
                gatt: BluetoothGatt?,
                mtu: Int,
                status: Int,
            ) {
                log("MTU changed to $mtu")
                currentMtu = mtu
            }

            // whenever a fragment arrives from Jetson, process here
            @RequiresApi(Build.VERSION_CODES.O)
            override fun onCharacteristicChanged(
                gatt: BluetoothGatt?,
                characteristic: BluetoothGattCharacteristic?,
            ) {
                if (characteristic?.uuid == ADAS_ALERT_STREAM_UUID) {
                    val raw = characteristic.value ?: return
                    // log("RX ${raw.size} bytes")
                    handleIncomingFragment(raw)
                }
            }
        }

    // Fragment Handler — central logic

    /**
     * Fragment Handler
     *
     * Handles:
     * 1. Read the header
     * 2. Determine which tick the fragment belongs to
     * 3. Store fragment in the right place
     * 4. When all fragments arrive, rebuild full payload
     * 5. Send complete CBOR TickPayload to the app/UI
     *
     */
    @RequiresApi(Build.VERSION_CODES.O)
    private fun handleIncomingFragment(data: ByteArray) {
        // Extract header
        val header = BLEHeader.fromBytes(data) ?: return

        // Extract slice (payload fragment)
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

        // If complete → reassemble full CBOR payload B
        if (tick.receivedCount == (tick.seqMax + 1)) {
            val full = rebuildFullPayload(tick)
            if (full != null) {
                onFullTickPayloadReceived(tick.tickId, full)
            }
            // Reset buffer afterwards
            activeTick = null
        }
    }

    // Rebuild CBOR TickPayload from fragments
    private fun rebuildFullPayload(tick: InProgressTick): ByteArray? {
        val size = tick.fragments.values.sumOf { it.size }
        val buffer = ByteBuffer.allocate(size)

        for (i in 0..tick.seqMax) {
            val frag = tick.fragments[i] ?: return null // Missing fragment
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
                com.example.testapp.model.TickDecoder
                    .decode(cborBuffer)
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

        // Emit to flow
        _packetFlow.tryEmit(cborBuffer)
    }

    // Resend logic after reconnect
    private fun resendRingBuffer() {
        // TODO: Iterate through reconnectRing and send CBOR(B) if still valid (TTL)
    }

    // Placeholder for writing data to the Jetson
    fun sendCommand(cmd: ByteArray) {
        // TODO: write to characteristic
    }

    // Asks Andorid to negotiate a larger BLE MTU
=======
>>>>>>> origin/main
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun requestMtu(mtu: Int) {
        bluetoothGatt?.requestMtu(mtu)
    }

    private val gattCallback =
        object : BluetoothGattCallback() {
            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onConnectionStateChange(
                gatt: BluetoothGatt,
                status: Int,
                newState: Int,
            ) {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    gatt.close()
                    return
                }

                when (newState) {
                    BluetoothProfile.STATE_CONNECTED -> {
                        gatt.requestMtu(185)
                        gatt.discoverServices()
                    }
                    BluetoothProfile.STATE_DISCONNECTED -> {
                        gatt.close()
                    }
                }
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onMtuChanged(
                gatt: BluetoothGatt,
                mtu: Int,
                status: Int,
            ) {
                gatt.discoverServices()
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onServicesDiscovered(
                gatt: BluetoothGatt,
                status: Int,
            ) {
                if (status != BluetoothGatt.GATT_SUCCESS) return

                val svc = gatt.getService(ADAS_SERVICE_UUID) ?: return
                val ch = svc.getCharacteristic(ADAS_ALERT_STREAM_UUID) ?: return

                enableNotifications(gatt, ch)
            }

            override fun onCharacteristicChanged(
                gatt: BluetoothGatt,
                characteristic: BluetoothGattCharacteristic,
            ) {
                if (characteristic.uuid != ADAS_ALERT_STREAM_UUID) return
                val raw = characteristic.value ?: return
                handleIncomingFragment(raw)
            }
        }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private fun enableNotifications(
        gatt: BluetoothGatt,
        ch: BluetoothGattCharacteristic,
    ) {
        if (!gatt.setCharacteristicNotification(ch, true)) return
        val cccd = ch.getDescriptor(CCCD_UUID) ?: return
        cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        gatt.writeDescriptor(cccd)
    }

    private fun handleIncomingFragment(data: ByteArray) {
        val header = BLEHeader.fromBytes(data) ?: return
        val slice = data.copyOfRange(4, data.size)

        if (activeTick == null || activeTick?.tickId != header.tickId) {
            activeTick = InProgressTick(header.tickId, header.seqMax)
        }

        val tick = activeTick ?: return

        if (!tick.fragments.containsKey(header.seqNo)) {
            tick.fragments[header.seqNo] = slice
            tick.receivedCount += 1
        }

        if (tick.receivedCount == tick.seqMax + 1) {
            val frame =
                rebuildFullPayload(tick) ?: run {
                    activeTick = null
                    return
                }
            onFullTickPayloadReceived(frame)
            activeTick = null
        }
    }

    private fun rebuildFullPayload(tick: InProgressTick): ByteArray? {
        val parts = ArrayList<ByteArray>(tick.seqMax + 1)
        for (i in 0..tick.seqMax) {
            val p = tick.fragments[i] ?: return null
            parts += p
        }

        val total = parts.sumOf { it.size }
        val merged = ByteArray(total)
        var pos = 0
        for (p in parts) {
            p.copyInto(merged, destinationOffset = pos)
            pos += p.size
        }
        return merged
    }

    private fun onFullTickPayloadReceived(cborFrame: ByteArray) {
        val len = cborFrame.size
        val framed = ByteArray(2 + len)
        framed[0] = (len and 0xFF).toByte()
        framed[1] = ((len ushr 8) and 0xFF).toByte()
        cborFrame.copyInto(framed, destinationOffset = 2)

        _blePackets.tryEmit(framed)

        val decoded = tickStreamDecoder.onBytes(framed)
        decoded.forEach { _ticks.tryEmit(it) }
    }
}
