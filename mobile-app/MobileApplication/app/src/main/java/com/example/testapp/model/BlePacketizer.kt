package com.example.testapp.model

object BlePacketizer {
    const val TYPE_GPS: Byte = 0x01

    data class Chunk(val bytes: ByteArray)

    fun chunk(
        type: Byte,
        seq: Int,
        payload: ByteArray,
        maxChunkPayload: Int,
    ): List<Chunk> {
        require(maxChunkPayload >= 8) { "maxChunkPayload too small" }

        val chunks = mutableListOf<Chunk>()
        var offset = 0
        var first = true

        while (offset < payload.size) {
            val header =
                if (first) {
                    // type(1) + seq(2) + totalLen(2)
                    ByteArray(5).also {
                        it[0] = type
                        it[1] = ((seq shr 8) and 0xFF).toByte()
                        it[2] = (seq and 0xFF).toByte()
                        it[3] = ((payload.size shr 8) and 0xFF).toByte()
                        it[4] = (payload.size and 0xFF).toByte()
                    }
                } else {
                    // type(1) + seq(2)
                    ByteArray(3).also {
                        it[0] = type
                        it[1] = ((seq shr 8) and 0xFF).toByte()
                        it[2] = (seq and 0xFF).toByte()
                    }
                }

            val room = maxChunkPayload - header.size
            val take = minOf(room, payload.size - offset)

            val packet = ByteArray(header.size + take)
            System.arraycopy(header, 0, packet, 0, header.size)
            System.arraycopy(payload, offset, packet, header.size, take)

            chunks += Chunk(packet)

            offset += take
            first = false
        }

        return chunks
    }
}
