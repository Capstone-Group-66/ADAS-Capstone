package com.example.testapp.model

object VehicleAlertReducer {
    fun initial(): VehicleAlert =
        VehicleAlert(
            cameras = CameraHealth(frontOk = true, rearOk = true),
            radar = RadarHealth(ok = true),
            sonar =
                SonarColors(
                    front = SonarColor.OFF,
                    rear = SonarColor.OFF,
                    left = SonarColor.OFF,
                    right = SonarColor.OFF,
                ),
            severity = SonarColor.OFF,
            direction = Direction.FRONT,
            telemetry = VehicleTelemetry(speedKmh = 0),
            detection = ObjectDetection.None,
            bsd = BlindSpotStatus(leftActive = false, rightActive = false),
            lastTickId = -1,
        )

    /**
     * Returns:
     * - new VehicleAlert if tick is accepted
     * - null if dropped (out-of-order/duplicate)
     */
    fun reduce(
        prev: VehicleAlert,
        tick: TickStreamPayload,
    ): VehicleAlert? {
        // Strict ordering for remote ticks (tickId >= 0).
        // Local heartbeats (-1) and Manual Triggers (-2) bypass check.
        if (tick.tickId >= 0 && tick.tickId <= prev.lastTickId) return null

        // val (cameras, radar) = TickPayloadMapper.healthFromMask(tick.healthMask)
        val firstAlert = tick.alerts.firstOrNull()

<<<<<<< HEAD
        // Latch Logic: Extend FCW expiry if FCW alert present
        val hasIncomingFcw = tick.alerts.any { it.type == 0 }
        val now = System.currentTimeMillis()
        val newFcwExpiry = if (hasIncomingFcw) now + 3000 else prev.fcwExpiry

        // Determine active state based on Latch
        val isLatched = now < newFcwExpiry

        val sonar =
            if (isLatched) {
                prev.sonar.copy(front = SonarColor.RED)
            } else {
                prev.sonar.copy(front = SonarColor.GREEN)
            }

        val detection =
            if (isLatched) {
                val severity = tick.alerts.find { it.type == 0 }?.severity ?: 2
                ObjectDetection.Car(confidence = severity * 33)
            } else {
                ObjectDetection.None
            }

        // Don't update lastTickId if it's a local event (< 0)
        val newLastTickId = if (tick.tickId >= 0) tick.tickId else prev.lastTickId
=======
        // Get the severity from the first alert. If there are no alerts,
        val severityInt = firstAlert?.severity ?: -1 // Use -1 as a default "no change" value
>>>>>>> origin/main

        val severity = MapSeverityIntToColour(severityInt)

        val newDirection = firstAlert?.direction ?: "front"
        val direction = MapDirectionIntToDirection(newDirection)
        return prev.copy(
<<<<<<< HEAD
            cameras = cameras,
            radar = radar,
            bsd = bsd,
            telemetry = VehicleTelemetry(speedKmh = kotlin.math.abs(tick.speed).coerceIn(0, 300)),
            sonar = sonar,
            detection = detection,
            activeAlerts = tick.alerts,
            lastTickId = newLastTickId,
            timestampMs = now,
            fcwExpiry = newFcwExpiry,
=======
            cameras = prev.cameras,
            radar = prev.radar,
            telemetry = prev.telemetry,
            sonar = prev.sonar,
            detection = prev.detection,
            direction = direction,
            severity = severity,
            lastTickId = tick.tickId,
            timestampMs = System.currentTimeMillis(),
>>>>>>> origin/main
        )
    }

    private fun mapAlertsToSonar(
        prev: SonarColors,
        alerts: List<com.example.testapp.model.AlertDto>,
    ): SonarColors {
        return prev // Deprecated by inline logic above
    }

    private fun mapAlertsToDetection(
        prev: ObjectDetection,
        alerts: List<com.example.testapp.model.AlertDto>,
    ): ObjectDetection {
        return prev // Deprecated by inline logic above
    }

    fun MapSeverityIntToColour(severity: Int): SonarColor {
        when (severity) {
            0 -> SonarColor.GREEN
            1 -> SonarColor.YELLOW
            2 -> SonarColor.RED
            else -> SonarColor.OFF
        }
        return SonarColor.OFF
    }

    fun MapDirectionIntToDirection(direction: String): Direction {
        when (direction) {
            "front" -> Direction.FRONT
            "rear" -> Direction.REAR
            "left" -> Direction.LEFT
            "right" -> Direction.RIGHT
            else -> Direction.FRONT
        }
        return Direction.FRONT
    }
}
