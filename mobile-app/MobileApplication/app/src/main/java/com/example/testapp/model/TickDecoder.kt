package com.example.testapp.model

import kotlinx.serialization.cbor.Cbor
import kotlinx.serialization.decodeFromByteArray

object TickDecoder {
    private val cbor =
        Cbor {
            ignoreUnknownKeys = true
        }

    fun decode(bytes: ByteArray): TickPayload = cbor.decodeFromByteArray(bytes)
}
