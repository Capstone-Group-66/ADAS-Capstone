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
        if (tick.tickId <= prev.lastTickId) return null

        val (cameras, radar) = TickPayloadMapper.healthFromMask(tick.healthMask)
        val bsd = TickPayloadMapper.bsdFromMask(tick.bsdMask)

        // Replace these mappings once your protocol defines sonar/detection in alerts:
        val sonar = mapAlertsToSonar(prev.sonar, tick.alerts)
        val detection = mapAlertsToDetection(prev.detection, tick.alerts)

        return prev.copy(
            cameras = cameras,
            radar = radar,
            bsd = bsd,
            telemetry = VehicleTelemetry(speedKmh = tick.speed.coerceIn(0, 300)),
            sonar = sonar,
            detection = detection,
            activeAlerts = tick.alerts,
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
}
