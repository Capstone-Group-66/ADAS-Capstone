package com.example.testapp.model

import androidx.compose.ui.graphics.Color
import java.util.UUID

data class VehicleAlert(
    val id: String = UUID.randomUUID().toString(),
    val health: SystemHealth,
    val sonar: SonarColors,
    val telemetry: VehicleTelemetry,
    val detection: ObjectDetection,
    val bsd: BlindSpotStatus,
    val lastTickId: Int = -1,
    val timestampMs: Long = System.currentTimeMillis(),
    val activeAlerts: List<AlertDto> = emptyList(),
    val fcwExpiry: Long = 0,
    val bsdLeftExpiry: Long = 0,
    val bsdRightExpiry: Long = 0,
    val fcwSeverity: Int? = null,
)

data class SystemHealth(
    val frontCameraOk: Boolean,
    val rearRcwOk: Boolean,
    val radarOk: Boolean,
)

data class VehicleTelemetry(
    val speedKmh: Int,
)

enum class SonarColor(
    val color: Color,
) {
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
