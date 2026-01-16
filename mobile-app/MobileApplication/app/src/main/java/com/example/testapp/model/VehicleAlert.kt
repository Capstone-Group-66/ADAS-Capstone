package com.example.testapp.model

import androidx.compose.ui.graphics.Color
import java.util.UUID

data class VehicleAlert(
    val id: String = UUID.randomUUID().toString(),
    val cameras: CameraHealth,
    val radar: RadarHealth,
    val sonar: SonarColors,
    val telemetry: VehicleTelemetry,
    val detection: ObjectDetection,
    val bsd: BlindSpotStatus,
    val lastTickId: Int = -1,
    val timestampMs: Long = System.currentTimeMillis(),
    val activeAlerts: List<com.example.testapp.model.AlertDto> = emptyList(),
)

data class CameraHealth(
    val frontOk: Boolean,
    val rearOk: Boolean,
)

data class RadarHealth(
    val ok: Boolean,
)

data class VehicleTelemetry(
    val speedKmh: Int,
)

enum class SonarColor(val color: Color) {
    OFF(Color.Gray),
    GREEN(Color.Green),
    YELLOW(Color.Yellow),
    RED(Color.Red),
}

data class SonarColors(
    val front: SonarColor,
    val rear: SonarColor,
    val left: SonarColor,
    val right: SonarColor,
)

data class BlindSpotStatus(
    val leftActive: Boolean,
    val rightActive: Boolean,
)

sealed class ObjectDetection {
    data object None : ObjectDetection()

    data class Car(
        val confidence: Int? = null,
    ) : ObjectDetection()

    data class Person(
        val confidence: Int? = null,
    ) : ObjectDetection()
}
