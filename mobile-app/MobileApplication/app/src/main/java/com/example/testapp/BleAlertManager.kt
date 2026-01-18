package com.example.testapp

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
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
import java.util.UUID

class BleManager(
    private val context: Context,
    private val serde: SerializationDeserialization,
) {
    private val ADAS_SERVICE_UUID: UUID =
        UUID.fromString("0000ada5-0000-1000-8000-00805f9b34fb")

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

    private val tickStreamDecoder = TickStreamDecoder(serde)
    private val _ticks = MutableSharedFlow<TickStreamPayload>(extraBufferCapacity = 128)
    val ticks: SharedFlow<TickStreamPayload> = _ticks.asSharedFlow()

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
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun connectToJetson(device: BluetoothDevice) {
        bluetoothGatt?.close()
        bluetoothGatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        activeTick = null
    }

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
