package com.example.testapp.model

object VehicleAlertReducer {
    private const val FcwType = 0
    private const val LdwType = 1
    private const val RcwType = 2
    private const val FcwLatchMs = 5000L
    private const val BsdLatchMs = 1000L

    fun initial(): VehicleAlert =
        VehicleAlert(
            health =
                SystemHealth(
                    frontCameraOk = false,
                    rearRcwOk = false,
                    radarOk = false,
                ),
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
        if (tick.tickId >= 0 && tick.tickId <= prev.lastTickId) return null

        val health = TickPayloadMapper.healthFromMask(tick.healthMask)
        val bsd = TickPayloadMapper.bsdFromMask(tick.bsdMask)
        val filteredAlerts = tick.alerts.filterNot { it.type == LdwType }

        val fcwAlert = filteredAlerts.firstOrNull { it.type == FcwType }
        val hasIncomingRcw = filteredAlerts.any { it.type == RcwType }
        val now = System.currentTimeMillis()
        val newFcwExpiry = if (fcwAlert != null) now + FcwLatchMs else prev.fcwExpiry
        val isFcwLatched = now < newFcwExpiry
        val fcwSeverity = fcwAlert?.severity ?: prev.fcwSeverity
        val newBsdLeftExpiry = if (bsd.leftActive) now + BsdLatchMs else prev.bsdLeftExpiry
        val newBsdRightExpiry = if (bsd.rightActive) now + BsdLatchMs else prev.bsdRightExpiry
        val isLeftBsdLatched = now < newBsdLeftExpiry
        val isRightBsdLatched = now < newBsdRightExpiry

        val sonar =
            SonarColors(
                front = if (isFcwLatched) SonarColor.RED else SonarColor.OFF,
                rear = if (hasIncomingRcw) SonarColor.RED else SonarColor.OFF,
                left = if (isLeftBsdLatched) SonarColor.YELLOW else SonarColor.OFF,
                right = if (isRightBsdLatched) SonarColor.YELLOW else SonarColor.OFF,
            )

        val detection =
            if (isFcwLatched) {
                ObjectDetection.Car(confidence = ((fcwSeverity ?: 1).coerceAtLeast(1) * 33))
            } else {
                ObjectDetection.None
            }

        val newLastTickId = if (tick.tickId >= 0) tick.tickId else prev.lastTickId

        return prev.copy(
            health = health,
            bsd = BlindSpotStatus(leftActive = isLeftBsdLatched, rightActive = isRightBsdLatched),
            telemetry = VehicleTelemetry(speedKmh = kotlin.math.abs(tick.speed).coerceIn(0, 300)),
            sonar = sonar,
            detection = detection,
            activeAlerts = filteredAlerts,
            lastTickId = newLastTickId,
            timestampMs = now,
            fcwExpiry = newFcwExpiry,
            bsdLeftExpiry = newBsdLeftExpiry,
            bsdRightExpiry = newBsdRightExpiry,
            fcwSeverity = if (isFcwLatched) fcwSeverity else null,
        )
    }
}
