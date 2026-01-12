package com.example.testapp.model

import java.util.UUID

data class VehicleAlert(
    val id: String = UUID.randomUUID().toString(),
    val cameras: CameraHealth,
    val radar: RadarHealth,
    val sonar: SonarColors,
    val telemetry: VehicleTelemetry,
    val detection: ObjectDetection,
    val timestampMs: Long = System.currentTimeMillis(),
)
