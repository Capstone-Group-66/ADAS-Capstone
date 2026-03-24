package com.example.testapp

import kotlin.test.Test
import kotlin.test.assertFalse
import kotlin.test.assertTrue

class BleConnectionStatusTest {
    @Test
    fun `connected state is authoritative`() {
        assertTrue(BleConnectionStatus.connected("Connected").isConnected)
    }

    @Test
    fun `disconnected label does not count as connected`() {
        assertFalse(BleConnectionStatus.disconnected("Disconnected").isConnected)
        assertFalse(BleConnectionStatus.error("Disconnected").isConnected)
    }
}
