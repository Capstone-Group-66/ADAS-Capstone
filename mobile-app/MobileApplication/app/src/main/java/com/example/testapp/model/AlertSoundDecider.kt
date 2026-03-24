package com.example.testapp.model

import com.example.testapp.UpdateUIstate

enum class AlertSoundEvent {
    FrontAlert,
    RearAlert,
    BsdLeft,
    BsdRight,
    BsdBoth,
}

data class AlertSoundSnapshot(
    val frontActive: Boolean,
    val rearActive: Boolean,
    val leftActive: Boolean,
    val rightActive: Boolean,
)

object AlertSoundDecider {
    fun snapshotOf(state: UpdateUIstate): AlertSoundSnapshot =
        AlertSoundSnapshot(
            frontActive = state.frontAlertColor == SonarColor.RED,
            rearActive = state.rearAlertColor == SonarColor.RED,
            leftActive = state.leftBlindspotValue == SonarColor.YELLOW,
            rightActive = state.rightBlindspotValue == SonarColor.YELLOW,
        )

    fun nextEvent(
        previous: AlertSoundSnapshot?,
        current: AlertSoundSnapshot,
    ): AlertSoundEvent? {
        val frontEdge = current.frontActive && previous?.frontActive != true
        val rearEdge = current.rearActive && previous?.rearActive != true
        val leftEdge = current.leftActive && previous?.leftActive != true
        val rightEdge = current.rightActive && previous?.rightActive != true

        return when {
            frontEdge -> AlertSoundEvent.FrontAlert
            rearEdge -> AlertSoundEvent.RearAlert
            leftEdge && rightEdge -> AlertSoundEvent.BsdBoth
            leftEdge -> AlertSoundEvent.BsdLeft
            rightEdge -> AlertSoundEvent.BsdRight
            else -> null
        }
    }
}
