package com.example.testapp

import kotlin.test.Test
import kotlin.test.assertEquals
import com.example.testapp.model.SerializationDeserialization
import com.example.testapp.model.TickStreamDecoder
import com.example.testapp.model.TickStreamEncoder
import com.example.testapp.model.TickStreamPayload

class TickStreamTest {

    private val serde = SerializationDeserialization

    private val encoder = TickStreamEncoder(serde)
    private val decoder = TickStreamDecoder(serde)

    @Test
    fun encodedTickTest() {
        val json = javaClass.classLoader!!
            .getResource("tick_sample.json")!!
            .readText()

        // Encode
        val framed = encoder.encodeJson(json)

        // Simulate BLE fragmentation
        val chunks = framed.asList().chunked(20).map { it.toByteArray() }

        val decoded = mutableListOf<TickStreamPayload>()
        for (chunk in chunks) {
            decoded += decoder.onBytes(chunk)
        }

        assertEquals(1, decoded.size)

        val original = serde.json.decodeFromString<TickStreamPayload>(json)
        assertEquals(original, decoded.single())
    }
}
