package com.example.testapp

import com.example.testapp.model.SonarColor

data class UpdateUIstate(
    val rearRcwOk: Boolean,
    val radarOk: Boolean,
    val bleConnected: Boolean,
    val frontCameraOk: Boolean,
    val speedKmh: Int,
    val frontAlertColor: SonarColor,
    val rearAlertColor: SonarColor,
    val leftBlindspotValue: SonarColor,
    val rightBlindspotValue: SonarColor,
    val timestamp: Long = 0,
    val fcwExpiry: Long = 0,
    val fcwSeverity: Int? = null,
)
