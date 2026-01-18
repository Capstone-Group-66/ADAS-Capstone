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
        if (tick.tickId <= prev.lastTickId) return null

        // val (cameras, radar) = TickPayloadMapper.healthFromMask(tick.healthMask)
        val firstAlert = tick.alerts.firstOrNull()

        // Get the severity from the first alert. If there are no alerts,
        val severityInt = firstAlert?.severity ?: -1 // Use -1 as a default "no change" value

        val severity = MapSeverityIntToColour(severityInt)

        val newDirection = firstAlert?.direction ?: "front"
        val direction = MapDirectionIntToDirection(newDirection)
        return prev.copy(
            cameras = prev.cameras,
            radar = prev.radar,
            telemetry = prev.telemetry,
            sonar = prev.sonar,
            detection = prev.detection,
            direction = direction,
            severity = severity,
            lastTickId = tick.tickId,
            timestampMs = System.currentTimeMillis(),
        )
    }

    private fun mapAlertsToSonar(
        prev: SonarColors,
        alerts: List<com.example.testapp.model.AlertDto>,
    ): SonarColors {
        // TODO: implement based on how sonar zones are represented in alerts.
        return prev
    }

    private fun mapAlertsToDetection(
        prev: ObjectDetection,
        alerts: List<com.example.testapp.model.AlertDto>,
    ): ObjectDetection {
        // TODO: implement based on how object detection is represented in alerts.
        return prev
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
