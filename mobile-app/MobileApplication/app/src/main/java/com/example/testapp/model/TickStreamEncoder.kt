package com.example.testapp.model
import kotlinx.serialization.encodeToByteArray

class TickStreamEncoder(
    private val serde: SerializationDeserialization
) {

    /**
     * Encode from JSON string (tests / debug tools)
     */
    fun encodeJson(jsonString: String): ByteArray {
        val tick = serde.json.decodeFromString<TickStreamPayload>(jsonString)
        return encodeTick(tick)
    }

    /**
     * Encode from model (normal runtime path)
     */
    fun encodeTick(tick: TickStreamPayload): ByteArray {
        val cborBytes = serde.cbor.encodeToByteArray(tick)
        return frame(cborBytes)
    }

    /**
     * Length-prefix framing (UInt16 LE)
     */
    private fun frame(payload: ByteArray): ByteArray {
        require(payload.size <= 65535) { "StreamTickPayload too large" }

        val len = payload.size
        return byteArrayOf(
            (len and 0xFF).toByte(),
            ((len shr 8) and 0xFF).toByte()
        ) + payload
    }
}
