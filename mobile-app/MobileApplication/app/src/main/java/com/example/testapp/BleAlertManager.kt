package com.example.testapp

import android.Manifest
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.content.Context
import android.os.Build
import androidx.annotation.RequiresApi
import androidx.annotation.RequiresPermission
import java.nio.ByteBuffer
import java.nio.ByteOrder

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
 * EVERYTHING IS JUST A SKELETON FOR NOW
 */
class BleManager(private val context: Context) {

    // BLE Constants & UUID placeholders
    private val ADAS_SERVICE_UUID = java.util.UUID.fromString("0000xxxx-0000-1000-8000-00805f9b34fb")
    private val ADAS_ALERT_STREAM_UUID = java.util.UUID.fromString("0000xxxx-0000-1000-8000-00805f9b34fb")

    // Jetson peripheral reference
    private var bluetoothGatt: BluetoothGatt? = null

    // Latest ATT MTU (default 23)
    private var currentMtu: Int = 23

    // Computed slice_cap = MTU - 7
    private val sliceCap: Int
        get() = (currentMtu - 7).coerceAtLeast(1)

    // Active tick currently being rebuilt
    private data class InProgressTick(
        val tickId: Int,
        var seqMax: Int,
        val fragments: MutableMap<Int, ByteArray> = mutableMapOf(),
        var receivedCount: Int = 0
    )

    private var activeTick: InProgressTick? = null

    // Ring buffer for reconnect resend (placeholders)
    private val reconnectRing = ArrayDeque<ByteArray>()


    // BLE Header Definition (4 bytes)
    // Matches struct BLEHeader in Jetson code.
    data class BLEHeader(
        val tickId: Int,   // uint16
        val seqNo: Int,    // uint8
        val seqMax: Int    // uint8
    ) {
        companion object {
            private const val HEADER_SIZE = 4

            /**
             * Parse 4-byte header from BLE notification.
             * Byte layout:
             *   uint16 tick_id
             *   uint8  seq_no
             *   uint8  seq_max
             */
            @RequiresApi(Build.VERSION_CODES.O)
            fun fromBytes(bytes: ByteArray): BLEHeader? {
                if (bytes.size < HEADER_SIZE) return null

                val bb = ByteBuffer.wrap(bytes).order(ByteOrder.LITTLE_ENDIAN)

                // placeholder parsing only
                val tickId = bb.short.toInt() and 0xFFFF
                val seqNo = bb.get().toInt() and 0xFF
                val seqMax = bb.get().toInt() and 0xFF

                return BLEHeader(tickId, seqNo, seqMax)
            }
        }
    }

    // Initialize BLE stack
    fun initialize() {
        // TODO: Set up BluetoothAdapter, scanner, permissions, callbacks.
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun connectToJetson(device: BluetoothDevice) {
        // TODO: Connect using GATT
        bluetoothGatt = device.connectGatt(context, false, gattCallback)
    }

    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun disconnect() {
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
    }

    // BLE GATT Callback (core of receiving alerts)
    private val gattCallback = object : BluetoothGattCallback() {

        // whenever the phone connects or disconnects from the Jetson
        override fun onConnectionStateChange(gatt: BluetoothGatt?, status: Int, newState: Int) {
            // TODO: handle reconnect logic
        }

        // whenever the BLE packet size changes, update slice size
        override fun onMtuChanged(gatt: BluetoothGatt?, mtu: Int, status: Int) {
            // Update slice_cap = ATT_MTU - 7
            currentMtu = mtu
            // TODO: trigger “recompute slice_cap next tick only”
        }

        // once BLE services load, enable alert notifications
        fun onServicesDiscovered(gatt: BluetoothGatt?) {
            // TODO: Enable notifications for ADAS_ALERT_STREAM characteristic
        }

        // whenever a fragment arrives from Jetson, process here
        @RequiresApi(Build.VERSION_CODES.O)
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt?,
            characteristic: BluetoothGattCharacteristic?
        ) {
            if (characteristic?.uuid == ADAS_ALERT_STREAM_UUID) {
                val raw = characteristic.value ?: return
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
            activeTick = InProgressTick(
                tickId = header.tickId,
                seqMax = header.seqMax
            )
        }

        val tick = activeTick ?: return

        // TODO: Add fragment to tick.fragments[seqNo]
        // TODO: Increment receivedCount

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
        // TODO:
        // Concatenate fragments[0], fragments[1], ..., fragments[seqMax]
        return null
    }

    // Called when one full tick CBOR buffer has been reassembled
    private fun onFullTickPayloadReceived(tickId: Int, cborBuffer: ByteArray) {
        // TODO:
        // - Decode CBOR TickPayload {tick_id, seq_max, n, alerts[]}
        // - Emit to UI or ViewModel
        // - Insert into reconnect ring (for resend if needed)
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
    @RequiresPermission(Manifest.permission.BLUETOOTH_CONNECT)
    fun requestMtu(mtu: Int) {
        bluetoothGatt?.requestMtu(mtu)
    }
}