package com.example.testapp.model

import kotlinx.serialization.decodeFromByteArray
import java.util.ArrayDeque

class TickStreamDecoder(
    private val serde: SerializationDeserialization
) {

    private val buffer = ArrayDeque<Byte>()

    /**
     * Feed raw BLE bytes.
     * Returns zero or more fully decoded TickPayloads.
     */
    fun onBytes(chunk: ByteArray): List<TickStreamPayload> {
        chunk.forEach { buffer.addLast(it) }

        val out = mutableListOf<TickStreamPayload>()

        while (true) {
            if (buffer.size < 2) break

            val len =
                (buffer.elementAt(0).toInt() and 0xFF) or
                        ((buffer.elementAt(1).toInt() and 0xFF) shl 8)

            if (buffer.size < 2 + len) break

            // consume header
            buffer.removeFirst()
            buffer.removeFirst()

            val frame = ByteArray(len)
            repeat(len) { i -> frame[i] = buffer.removeFirst() }

            val tick = serde.cbor.decodeFromByteArray<TickStreamPayload>(frame)
            out += tick
        }

        return out
    }
}
