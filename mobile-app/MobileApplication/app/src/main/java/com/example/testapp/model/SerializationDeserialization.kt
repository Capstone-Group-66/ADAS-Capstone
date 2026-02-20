package com.example.testapp.model
import kotlinx.serialization.cbor.Cbor
import kotlinx.serialization.json.Json

object SerializationDeserialization {
    val json =
        Json {
            ignoreUnknownKeys = true
            isLenient = true
            explicitNulls = false
        }

    val cbor =
        Cbor {
            ignoreUnknownKeys = true // future-proof Jetson ↔ Android
            encodeDefaults = false // smaller packets
        }
}
