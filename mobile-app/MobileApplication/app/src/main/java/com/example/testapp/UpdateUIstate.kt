package com.example.testapp

import com.example.testapp.model.SonarColor

data class UpdateUIstate(
    val status1: Boolean,
    val status2: Boolean,
    val status3: Boolean,
    val status4: Boolean,
    val sonarValue: SonarColor,
    val leftBlindspotValue: SonarColor,
    val rightBlindspotValue: SonarColor,
    val timestamp: Long = 0,
    val fcwExpiry: Long = 0,
)
