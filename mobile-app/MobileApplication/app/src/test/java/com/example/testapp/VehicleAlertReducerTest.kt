package com.example.testapp

import com.example.testapp.model.AlertDto
import com.example.testapp.model.SonarColor
import com.example.testapp.model.TickPayload
import com.example.testapp.model.VehicleAlertReducer
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNotNull
import kotlin.test.assertNull
import kotlin.test.assertTrue

class VehicleAlertReducerTest {
    @Test
    fun `fcw warning tick sets front alert and latch`() {
        val tick =
            TickPayload(
                tickId = 1,
                speed = 42,
                healthMask = 0,
                bsdMask = 0,
                alerts = listOf(AlertDto(type = 0, severity = 1, rationale = "FCW warn")),
            )

        val reduced = VehicleAlertReducer.reduce(VehicleAlertReducer.initial(), tick)

        assertNotNull(reduced)
        assertEquals(SonarColor.RED, reduced.sonar.front)
        assertEquals(SonarColor.OFF, reduced.sonar.rear)
        assertEquals(42, reduced.telemetry.speedKmh)
        assertEquals(1, reduced.fcwSeverity)
        assertTrue(reduced.fcwExpiry >= reduced.timestampMs)
    }

    @Test
    fun `fcw critical tick uses same front reaction as warning`() {
        val tick =
            TickPayload(
                tickId = 2,
                speed = 37,
                healthMask = 0,
                bsdMask = 0,
                alerts = listOf(AlertDto(type = 0, severity = 2, rationale = "FCW critical")),
            )

        val reduced = VehicleAlertReducer.reduce(VehicleAlertReducer.initial(), tick)

        assertNotNull(reduced)
        assertEquals(SonarColor.RED, reduced.sonar.front)
        assertEquals(SonarColor.OFF, reduced.sonar.rear)
        assertEquals(2, reduced.fcwSeverity)
    }

    @Test
    fun `fcw latch keeps front alert active across non fcw tick until expiry`() {
        val initial =
            VehicleAlertReducer.reduce(
                VehicleAlertReducer.initial(),
                TickPayload(
                    tickId = 10,
                    speed = 55,
                    healthMask = 0,
                    bsdMask = 0,
                    alerts = listOf(AlertDto(type = 0, severity = 1, rationale = "FCW")),
                ),
            )

        assertNotNull(initial)

        val reduced =
            VehicleAlertReducer.reduce(
                initial,
                TickPayload(
                    tickId = 11,
                    speed = 54,
                    healthMask = 0,
                    bsdMask = 0,
                    alerts = emptyList(),
                ),
            )

        assertNotNull(reduced)
        assertEquals(SonarColor.RED, reduced.sonar.front)
        assertTrue(reduced.fcwExpiry >= reduced.timestampMs)
    }

    @Test
    fun `rcw tick lights only rear zone`() {
        val tick =
            TickPayload(
                tickId = 3,
                speed = 12,
                healthMask = 0,
                bsdMask = 0,
                alerts = listOf(AlertDto(type = 2, severity = 1, rationale = "RCW")),
            )

        val reduced = VehicleAlertReducer.reduce(VehicleAlertReducer.initial(), tick)

        assertNotNull(reduced)
        assertEquals(SonarColor.OFF, reduced.sonar.front)
        assertEquals(SonarColor.RED, reduced.sonar.rear)
        assertEquals(SonarColor.OFF, reduced.sonar.left)
        assertEquals(SonarColor.OFF, reduced.sonar.right)
    }

    @Test
    fun `bsd bitmask lights side specific colors`() {
        val tick =
            TickPayload(
                tickId = 4,
                speed = 0,
                healthMask = 0,
                bsdMask = 0b10,
                alerts = emptyList(),
            )

        val reduced = VehicleAlertReducer.reduce(VehicleAlertReducer.initial(), tick)

        assertNotNull(reduced)
        assertEquals(SonarColor.OFF, reduced.sonar.left)
        assertEquals(SonarColor.YELLOW, reduced.sonar.right)
    }

    @Test
    fun `ldw tick is ignored for active alerts`() {
        val tick =
            TickPayload(
                tickId = 5,
                speed = 22,
                healthMask = 0,
                bsdMask = 0,
                alerts = listOf(AlertDto(type = 1, severity = 1, rationale = "LDW")),
            )

        val reduced = VehicleAlertReducer.reduce(VehicleAlertReducer.initial(), tick)

        assertNotNull(reduced)
        assertEquals(SonarColor.OFF, reduced.sonar.front)
        assertEquals(SonarColor.OFF, reduced.sonar.rear)
        assertTrue(reduced.activeAlerts.isEmpty())
        assertNull(reduced.fcwSeverity)
    }
}
