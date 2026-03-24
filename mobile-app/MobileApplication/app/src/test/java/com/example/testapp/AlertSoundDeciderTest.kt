package com.example.testapp

import com.example.testapp.model.AlertSoundDecider
import com.example.testapp.model.AlertSoundEvent
import com.example.testapp.model.SonarColor
import kotlin.test.Test
import kotlin.test.assertEquals
import kotlin.test.assertNull

class AlertSoundDeciderTest {
    private fun state(
        front: SonarColor = SonarColor.OFF,
        rear: SonarColor = SonarColor.OFF,
        left: SonarColor = SonarColor.OFF,
        right: SonarColor = SonarColor.OFF,
    ): UpdateUIstate =
        UpdateUIstate(
            rearRcwOk = true,
            radarOk = true,
            bleConnected = true,
            frontCameraOk = true,
            speedKmh = 0,
            frontAlertColor = front,
            rearAlertColor = rear,
            leftBlindspotValue = left,
            rightBlindspotValue = right,
        )

    @Test
    fun `front alert edge has highest priority`() {
        val previous = AlertSoundDecider.snapshotOf(state())
        val current =
            AlertSoundDecider.snapshotOf(
                state(
                    front = SonarColor.RED,
                    right = SonarColor.YELLOW,
                ),
            )

        assertEquals(
            AlertSoundEvent.FrontAlert,
            AlertSoundDecider.nextEvent(previous, current),
        )
    }

    @Test
    fun `rear alert edge triggers rear sound`() {
        val previous = AlertSoundDecider.snapshotOf(state())
        val current = AlertSoundDecider.snapshotOf(state(rear = SonarColor.RED))

        assertEquals(
            AlertSoundEvent.RearAlert,
            AlertSoundDecider.nextEvent(previous, current),
        )
    }

    @Test
    fun `simultaneous bsd edges collapse into one combined event`() {
        val previous = AlertSoundDecider.snapshotOf(state())
        val current =
            AlertSoundDecider.snapshotOf(
                state(
                    left = SonarColor.YELLOW,
                    right = SonarColor.YELLOW,
                ),
            )

        assertEquals(
            AlertSoundEvent.BsdBoth,
            AlertSoundDecider.nextEvent(previous, current),
        )
    }

    @Test
    fun `steady held alert does not retrigger sound`() {
        val held = AlertSoundDecider.snapshotOf(state(front = SonarColor.RED))

        assertNull(AlertSoundDecider.nextEvent(held, held))
    }
}
