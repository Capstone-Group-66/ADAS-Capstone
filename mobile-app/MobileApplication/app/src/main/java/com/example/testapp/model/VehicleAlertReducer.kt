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
            telemetry = VehicleTelemetry(speedKmh = 0),
            detection = ObjectDetection.None,
            bsd = BlindSpotStatus(leftActive = false, rightActive = false),
            lastTickId = -1,
            expiries = Expiries(
                fcw = 0,
                rcw = 0,
                bsd_l = 0,
                bsd_r = 0,
                ldw = 0,
            )
        )

    /**
     * Returns:
     * - new VehicleAlert if tick is accepted
     * - null if dropped (out-of-order/duplicate)
     */
    fun reduce(
        prev: VehicleAlert,
        tick: TickPayload,
    ): VehicleAlert? {
        // Strict ordering for remote ticks (tickId >= 0).
        // Local heartbeats (-1) and Manual Triggers (-2) bypass check.
        if (tick.tickId >= 0 && tick.tickId <= prev.lastTickId) return null

        val (cameras, radar) = TickPayloadMapper.healthFromMask(tick.healthMask)
        val bsd = TickPayloadMapper.bsdFromMask(tick.bsdMask)

        // Latch Logic: Extend FCW expiry if FCW alert present
        val hasIncomingFcw = tick.alerts.any { it.type == 0 }
        val now = System.currentTimeMillis()
        val newFcwExpiry = if (hasIncomingFcw) now + 3000 else prev.expiries.fcw
        val newRcwExpiry = if (tick.alerts.any { it.type == 2 && it.severity > 0 }) now + 3000 else prev.expiries.rcw

        // Determine active state based on Latch
        val isLatched = now < newFcwExpiry

        val sonar =
            prev.sonar.copy(
                front = if (isLatched) SonarColor.RED else SonarColor.GREEN,
                rear = if (now < newRcwExpiry) SonarColor.RED else SonarColor.GREEN,
            )

        val detection =
            if (isLatched) {
                val severity = tick.alerts.find { it.type == 0 }?.severity ?: 2
                ObjectDetection.Car(confidence = severity * 33)
            } else {
                ObjectDetection.None
            }

        // Don't update lastTickId if it's a local event (< 0)
        val newLastTickId = if (tick.tickId >= 0) tick.tickId else prev.lastTickId

        return prev.copy(
            cameras = cameras,
            radar = radar,
            bsd = bsd,
            telemetry = VehicleTelemetry(speedKmh = kotlin.math.abs(tick.speed).coerceIn(0, 300)),
            sonar = sonar,
            detection = detection,
            activeAlerts = tick.alerts,
            lastTickId = newLastTickId,
            timestampMs = now,
            expiries =
                Expiries(
                    fcw = newFcwExpiry,
                    rcw = newRcwExpiry,
                    bsd_r = 0,
                    bsd_l = 0,
                    ldw = 0,
                ),
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
}
