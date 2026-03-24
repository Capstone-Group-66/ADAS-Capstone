package com.example.testapp

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothProfile
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
import java.util.ArrayDeque
import java.util.UUID

/**
 * BleManager
 *
 * Purpose:
 * - Scan + connect to ADAS-Jetson peripheral
 * - Subscribe to AlertStream notifications (Jetson -> Phone)
 * - Reassemble fragmented CBOR TickPayload frames
 * - Provide packet + log + connection state flows for UI
 *
 * TX:
 * - Send speed-only GpsData as CBOR over Command characteristic (Phone -> Jetson)
 * - Uses a simple paced TX queue to avoid BLE write overflow
 */
class BleManager(
    private val context: Context,
    private val cfg: AdasBleConfig = AdasBleConfig.default(),
) {
    // =======================================================
    // SECTION: BLE CONFIG / INTERFACE CONTRACTS
    // =======================================================

    /**
     * Centralized BLE contract config (from ADAS Interface Contracts v1.0, Feb 12 2026)
     */
    data class AdasBleConfig(
        val serviceUuid: UUID,
        val alertStreamUuid: UUID,
        val statusUuid: UUID,
        val commandUuid: UUID,
        val pairUuid: UUID,
        val deviceName: String = "ADAS-Jetson",
        val negotiatedMtu: Int = 185,
    ) {
        companion object {
            fun default(): AdasBleConfig =
                AdasBleConfig(
                    serviceUuid = UUID.fromString("0000ada5-0000-1000-8000-00805f9b34fb"),
                    alertStreamUuid = UUID.fromString("0000a1e7-0000-1000-8000-00805f9b34fb"),
                    statusUuid = UUID.fromString("000057a7-0000-1000-8000-00805f9b34fb"),
                    commandUuid = UUID.fromString("0000c0ad-0000-1000-8000-00805f9b34fb"),
                    pairUuid = UUID.fromString("0000fa17-0000-1000-8000-00805f9b34fb"),
                    deviceName = "ADAS-Jetson",
                    negotiatedMtu = 185,
                )
        }
    }

    // Standard CCCD UUID to enable notifications on a characteristic
    private val CCCD_UUID: UUID =
        UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    // =======================================================
    // SECTION: PUBLIC FLOWS (UI OBSERVABILITY)
    // =======================================================

    private val _packetFlow =
        MutableSharedFlow<ByteArray>(
            replay = 0,
            extraBufferCapacity = 64,
            onBufferOverflow = BufferOverflow.DROP_OLDEST,
        )
    val packetFlow: Flow<ByteArray> = _packetFlow

    private val _logFlow = MutableStateFlow<List<String>>(emptyList())
    val logFlow: StateFlow<List<String>> = _logFlow

    private val _connectionStatus = MutableStateFlow(BleConnectionStatus.disconnected())
    val connectionStatus: StateFlow<BleConnectionStatus> = _connectionStatus

    // =======================================================
    // SECTION: ANDROID BLE STATE
    // =======================================================

    private var bluetoothGatt: BluetoothGatt? = null
    private var bluetoothAdapter: android.bluetooth.BluetoothAdapter? = null
    private var bluetoothLeScanner: android.bluetooth.le.BluetoothLeScanner? = null

    // Cached characteristics (discovered after service discovery)
    private var alertStreamChar: BluetoothGattCharacteristic? = null
    private var commandChar: BluetoothGattCharacteristic? = null

    // MTU (default 23 until negotiated)
    private var currentMtu: Int = 23

    // =======================================================
    // SECTION: SCAN / RETRY MECHANICS
    // =======================================================

    private val handler = Handler(Looper.getMainLooper())
    private var isScanning = false
    private val SCAN_RETRY_DELAY_MS = 2000L
    private val SCAN_TIMEOUT_MS = 5000L
    private val ASSEMBLY_TIMEOUT_MS = 1500L
    private val MAX_FRAGMENTS_PER_TICK = 64
    private val seenScanDevices = mutableSetOf<String>()

    @Volatile
    private var activeSession = false

    // =======================================================
    // SECTION: LOGGING HELPERS
    // =======================================================

    private fun log(msg: String) {
        val list = _logFlow.value.toMutableList()
        val time = DateTimeFormatter.ofPattern("HH:mm:ss").format(LocalTime.now())
        list.add(0, "$time $msg")
        if (list.size > 100) list.removeAt(list.lastIndex)
        _logFlow.value = list
        android.util.Log.d("BleManager", msg)
    }

    // =======================================================
    // SECTION: MTU / PAYLOAD SIZING HELPERS
    // =======================================================

    // Jetson RX fragmentation uses: slice_capacity = MTU - 3 (ATT overhead) - 4 header
    // We track sliceCap for debugging/visibility (Jetson controls actual RX frag size).
    private val sliceCap: Int
        get() = (currentMtu - 7).coerceAtLeast(1)

    // For TX: typical max payload per write is MTU - 3
    private fun attPayloadMax(): Int = (currentMtu - 3).coerceAtLeast(20)

    // =======================================================
    // SECTION: RX REASSEMBLY (JETSON -> PHONE)
    // =======================================================

    /**
     * Matches BLE Wire Protocol header:
     *  - tick_id : uint16 LE
     *  - seq_no  : uint8
     *  - seq_max : uint8
     */
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

    private data class InProgressTick(
        val tickId: Int,
        var seqMax: Int,
        val fragments: MutableMap<Int, ByteArray> = mutableMapOf(),
        var receivedCount: Int = 0,
    )

    private var activeTick: InProgressTick? = null
    private val assemblyTimeoutRunnable =
        Runnable {
            clearActiveTick("Dropping stale fragment assembly due to timeout")
        }

    @RequiresApi(Build.VERSION_CODES.O)
    private fun handleIncomingFragment(data: ByteArray) {
        val header = BLEHeader.fromBytes(data) ?: return
        if (data.size <= 4) return
        if (header.seqNo > header.seqMax) {
            log("Dropping malformed fragment: seqNo=${header.seqNo} > seqMax=${header.seqMax}")
            clearActiveTick("Clearing fragment assembly after malformed sequence")
            return
        }
        if (header.seqMax + 1 > MAX_FRAGMENTS_PER_TICK) {
            log("Dropping malformed fragment: seqMax=${header.seqMax} exceeds supported limit")
            clearActiveTick("Clearing oversized fragment assembly")
            return
        }

        val slice = data.copyOfRange(4, data.size)

        // Reset assembly when tick changes (simple strategy)
        if (activeTick != null && activeTick?.tickId != header.tickId) {
            clearActiveTick("Dropping incomplete tick ${activeTick?.tickId} due to new tick ${header.tickId}")
        }

        if (activeTick == null) {
            activeTick =
                InProgressTick(
                    tickId = header.tickId,
                    seqMax = header.seqMax,
                )
        }

        val currentTick = activeTick ?: return
        if (currentTick.seqMax != header.seqMax) {
            clearActiveTick("Dropping tick ${currentTick.tickId} due to inconsistent seqMax")
            activeTick =
                InProgressTick(
                    tickId = header.tickId,
                    seqMax = header.seqMax,
                )
        }

        val tick = activeTick ?: return
        resetAssemblyTimeout()

        if (!tick.fragments.containsKey(header.seqNo)) {
            tick.fragments[header.seqNo] = slice
            tick.receivedCount++
        }

        if (tick.receivedCount == (tick.seqMax + 1)) {
            val full = rebuildFullPayload(tick)
            if (full != null) {
                onFullTickPayloadReceived(tick.tickId, full)
            }
            clearActiveTick()
        }
    }

    private fun resetAssemblyTimeout() {
        handler.removeCallbacks(assemblyTimeoutRunnable)
        handler.postDelayed(assemblyTimeoutRunnable, ASSEMBLY_TIMEOUT_MS)
    }

    private fun clearActiveTick(logMessage: String? = null) {
        if (logMessage != null && activeTick != null) {
            log(logMessage)
        }
        handler.removeCallbacks(assemblyTimeoutRunnable)
        activeTick = null
    }

    private fun rebuildFullPayload(tick: InProgressTick): ByteArray? {
        var size = 0
        for (v in tick.fragments.values) size += v.size

        val buffer = ByteBuffer.allocate(size)
        for (i in 0..tick.seqMax) {
            val frag = tick.fragments[i] ?: return null
            buffer.put(frag)
        }
        return buffer.array()
    }

    private fun onFullTickPayloadReceived(
        tickId: Int,
        cborBuffer: ByteArray,
    ) {
        log("Received full tick payload: $tickId (${cborBuffer.size} bytes)")

        // Optional: decode for debugging (does not affect flow output)
        try {
            val payload = com.example.testapp.model.TickDecoder.decode(cborBuffer)

            log("=== TICK DEBUG ===")
            log("  Tick ID: ${payload.tickId}")
            log("  Speed: ${payload.speed} km/h")
            log("  Health Mask: ${payload.healthMask}")
            log("  BSD Mask: ${payload.bsdMask}")
            log("  Alerts: ${payload.alerts.size}")
            payload.alerts.forEachIndexed { idx, alert ->
                log("  Alert[$idx]: type=${alert.type} severity=${alert.severity} rationale=${alert.rationale}")
            }
            log("==================")
        } catch (e: Exception) {
            log("DEBUG decode error: ${e.message}")
        }

        _packetFlow.tryEmit(cborBuffer)
    }

    fun setConnectionStatus(status: BleConnectionStatus) {
        _connectionStatus.value = status
    }

    // =======================================================
    // SECTION: TX QUEUE (PHONE -> JETSON)
    // =======================================================

    private val txQueue: ArrayDeque<ByteArray> = ArrayDeque()
    private var txWriting: Boolean = false

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private fun enqueueTx(packet: ByteArray) {
        if (txQueue.size > 256) {
            log("TX queue overflow; dropping oldest")
            if (!txQueue.isEmpty()) {
                txQueue.removeFirst()
            }
        }
        txQueue.addLast(packet)
        if (!txWriting) {
            pumpTxQueue()
        }
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private fun pumpTxQueue() {
        val gatt = bluetoothGatt
        val tx = commandChar
        if (gatt == null || tx == null) {
            txWriting = false
            return
        }

        if (txQueue.isEmpty()) {
            txWriting = false
            return
        }

        val nextPacket = txQueue.removeFirst()

        txWriting = true
        tx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        tx.value = nextPacket

        val ok = gatt.writeCharacteristic(tx)
        if (!ok) {
            // Requeue and retry shortly
            log("TX pump: writeCharacteristic returned false; retrying soon")
            txQueue.addFirst(nextPacket)
            txWriting = false
            handler.postDelayed({
                if (!txWriting) {
                    try {
                        pumpTxQueue()
                    } catch (e: SecurityException) {
                        log("TX pump permission error: ${e.message}")
                    }
                }
            }, 30L)
        } else {
            // Pace to avoid BLE congestion
            handler.postDelayed({
                try {
                    pumpTxQueue()
                } catch (e: SecurityException) {
                    log("TX pump permission error: ${e.message}")
                }
            }, 20L)
        }
    }

    private fun clearTxQueue() {
        txQueue.clear()
        txWriting = false
    }

    // =======================================================
    // SECTION: PUBLIC API (INIT / CONNECT / DISCONNECT)
    // =======================================================

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun initialize() {
        if (activeSession && (isScanning || bluetoothGatt != null)) {
            return
        }
        activeSession = true
        val bluetoothManager =
            context.getSystemService(Context.BLUETOOTH_SERVICE) as android.bluetooth.BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        if (bluetoothAdapter == null || bluetoothAdapter?.isEnabled != true) {
            setConnectionStatus(BleConnectionStatus.bleUnavailable())
            return
        }
        bluetoothLeScanner = bluetoothAdapter?.bluetoothLeScanner
        if (bluetoothLeScanner == null) {
            setConnectionStatus(BleConnectionStatus.bleUnavailable())
            return
        }
        scanForJetson()
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun connectToJetson(device: BluetoothDevice) {
        log("Connecting to ${device.address}...")
        setConnectionStatus(BleConnectionStatus.connecting())
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
        shutdown()
    }

    fun shutdown() {
        activeSession = false
        handler.removeCallbacksAndMessages(null)
        seenScanDevices.clear()
        if (isScanning) {
            try {
                bluetoothLeScanner?.stopScan(scanCallback)
            } catch (_: SecurityException) {
            } catch (_: Exception) {
            }
            isScanning = false
        }
        try {
            bluetoothGatt?.disconnect()
        } catch (_: SecurityException) {
        } catch (_: Exception) {
        }
        try {
            bluetoothGatt?.close()
        } catch (_: SecurityException) {
        } catch (_: Exception) {
        }
        bluetoothGatt = null
        alertStreamChar = null
        commandChar = null
        clearActiveTick()
        clearTxQueue()
        currentMtu = 23
        setConnectionStatus(BleConnectionStatus.disconnected())
    }

    // =======================================================
    // SECTION: SCANNING (DISCOVERY)
    // =======================================================

    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    private fun scanForJetson() {
        if (!activeSession) return

        if (bluetoothLeScanner == null || bluetoothAdapter?.isEnabled != true) {
            log("Bluetooth not supported or disabled")
            setConnectionStatus(BleConnectionStatus.bleUnavailable())
            return
        }

        if (isScanning) {
            log("Already scanning...")
            return
        }

        seenScanDevices.clear()
        log("Starting loose scan for ${cfg.deviceName}...")
        setConnectionStatus(BleConnectionStatus.scanning())
        isScanning = true

        val settings =
            ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
                .build()

        bluetoothLeScanner?.startScan(null, settings, scanCallback)

        handler.postDelayed({
            if (activeSession && isScanning && bluetoothGatt == null) {
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
        if (!activeSession || bluetoothGatt != null) return
        setConnectionStatus(BleConnectionStatus.retrying("Retrying in ${SCAN_RETRY_DELAY_MS / 1000}s..."))
        handler.postDelayed({
            if (activeSession && bluetoothGatt == null) {
                try {
                    scanForJetson()
                } catch (e: SecurityException) {
                    log("Permission error: ${e.message}")
                    setConnectionStatus(BleConnectionStatus.permissionRequired("Bluetooth permission error"))
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
                handleScanResult(result)
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onBatchScanResults(results: MutableList<android.bluetooth.le.ScanResult>?) {
                results?.forEach { handleScanResult(it) }
            }

            override fun onScanFailed(errorCode: Int) {
                log("Scan failed: $errorCode")
                isScanning = false
                setConnectionStatus(BleConnectionStatus.error("Scan failed: $errorCode"))
                scheduleRetry()
            }
        }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    private fun handleScanResult(result: android.bluetooth.le.ScanResult?) {
        val device = result?.device ?: return
        val scanRecord = result.scanRecord
        val advertisedName = scanRecord?.deviceName
        val cachedName = device.name
        val serviceUuids = scanRecord?.serviceUuids?.mapNotNull { it?.uuid }.orEmpty()
        val hasServiceMatch = serviceUuids.any { it == cfg.serviceUuid }
        val nameMatches =
            listOfNotNull(advertisedName, cachedName)
                .any { it == cfg.deviceName }

        if (seenScanDevices.add(device.address)) {
            val displayName = advertisedName ?: cachedName ?: "(no name)"
            val serviceSummary =
                if (serviceUuids.isEmpty()) {
                    "(none)"
                } else {
                    serviceUuids.joinToString()
                }
            log(
                "Scan hit: $displayName (${device.address}) " +
                    "advName=${advertisedName ?: "(none)"} " +
                    "cachedName=${cachedName ?: "(none)"} " +
                    "serviceMatch=$hasServiceMatch " +
                    "nameMatch=$nameMatches " +
                    "uuids=$serviceSummary",
            )
        }

        if (!hasServiceMatch && !nameMatches) {
            return
        }

        if (bluetoothGatt == null) {
            log(
                "Connecting to candidate ${device.address} " +
                    "(serviceMatch=$hasServiceMatch, nameMatch=$nameMatches)",
            )
            try {
                bluetoothLeScanner?.stopScan(scanCallback)
            } catch (_: SecurityException) {
            }
            isScanning = false
            connectToJetson(device)
        }
    }

    // =======================================================
    // SECTION: GATT CALLBACK (CORE BLE STATE MACHINE)
    // =======================================================

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
                    setConnectionStatus(BleConnectionStatus.connected())
                    gatt?.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    log("Disconnected from GATT server. status=$status")

                    try {
                        gatt?.close()
                    } catch (_: Exception) {
                    }

                    bluetoothGatt = null
                    alertStreamChar = null
                    commandChar = null
                    clearActiveTick()
                    clearTxQueue()

                    // Auto-reconnect
                    if (activeSession) {
                        setConnectionStatus(BleConnectionStatus.disconnected("Disconnected; retrying..."))
                        scheduleRetry()
                    } else {
                        setConnectionStatus(BleConnectionStatus.disconnected())
                    }
                }
            }

            @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
            override fun onServicesDiscovered(
                gatt: BluetoothGatt?,
                status: Int,
            ) {
                if (status != BluetoothGatt.GATT_SUCCESS) {
                    log("Service discovery failed: $status")
                    setConnectionStatus(BleConnectionStatus.error("Service discovery failed: $status"))
                    return
                }

                log("Services discovered.")
                val service = gatt?.getService(cfg.serviceUuid)
                if (service == null) {
                    log("ADAS Service not found!")
                    setConnectionStatus(BleConnectionStatus.error("ADAS service not found"))
                    return
                }

                // AlertStream (Notify) - Jetson -> Phone
                val rx = service.getCharacteristic(cfg.alertStreamUuid)
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

                // Command (Write) - Phone -> Jetson (used for sending speed CBOR)
                val tx = service.getCharacteristic(cfg.commandUuid)
                if (tx != null) {
                    commandChar = tx
                    log("Command characteristic found (TX ready).")
                } else {
                    log("Command characteristic not found! (check cfg.commandUuid)")
                }

                // Negotiate MTU per contract
                gatt.requestMtu(cfg.negotiatedMtu)
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
                        setConnectionStatus(BleConnectionStatus.connected("Connected and subscribed"))
                    } else {
                        log("Failed to subscribe (CCCD write): $status")
                        setConnectionStatus(BleConnectionStatus.error("Notification subscribe failed: $status"))
                    }
                }
            }

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

            @RequiresApi(Build.VERSION_CODES.O)
            override fun onCharacteristicChanged(
                gatt: BluetoothGatt?,
                characteristic: BluetoothGattCharacteristic?,
            ) {
                if (characteristic?.uuid == cfg.alertStreamUuid) {
                    val raw = characteristic.value ?: return
                    handleIncomingFragment(raw)
                }
            }
        }

    // =======================================================
    // SECTION: PUBLIC API (TX)
    // =======================================================

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun sendCommand(cmd: ByteArray) {
        if (bluetoothGatt == null) {
            log("sendCommand: not connected")
            return
        }
        if (commandChar == null) {
            log("sendCommand: commandChar not ready (or wrong UUID)")
            return
        }
        enqueueTx(cmd)
        log("sendCommand: enqueued ${cmd.size}B")
    }

    private var gpsSeq: Int = 0

    /**
     * Sends speed-only GpsData CBOR (chunked) over Command characteristic.
     * NOTE: This is NOT part of the Jetson->Phone TickPayload contract; it's app-specific TX framing.
     */
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun sendGpsData(data: GpsData) {
        val gatt =
            bluetoothGatt ?: run {
                log("sendGpsData: not connected")
                return
            }

        val tx =
            commandChar ?: run {
                log("sendGpsData: commandChar not ready (check cfg.commandUuid / discovery)")
                return
            }

        val cbor: ByteArray = SerializationDeserialization.cbor.encodeToByteArray(data)

        val packet = ByteArray(1 + cbor.size)
        packet[0] = 0x01
        System.arraycopy(cbor, 0, packet, 1, cbor.size)

        tx.writeType = BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE
        tx.value = packet

        val ok = gatt.writeCharacteristic(tx)
        if (!ok) log("sendGpsData: writeCharacteristic returned false")
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun requestMtu(mtu: Int) {
        bluetoothGatt?.requestMtu(mtu)
    }
}
