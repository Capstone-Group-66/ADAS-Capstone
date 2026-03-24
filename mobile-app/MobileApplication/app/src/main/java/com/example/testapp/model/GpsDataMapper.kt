package com.example.testapp.model

import android.location.Location

private const val MinFallbackDeltaMs = 500L
private const val MaxFallbackDeltaMs = 5000L
private const val MaxReasonableAccuracyMeters = 35f

fun Location.toGpsData(previous: Location? = null): GpsData =
    GpsData(
        tsMs = time,
        speedMps = resolvedSpeedMps(previous),
    )

private fun Location.resolvedSpeedMps(previous: Location?): Float? {
    if (hasSpeed()) {
        return speed
    }

    if (previous == null) {
        return null
    }

    val dtMs = time - previous.time
    if (dtMs !in MinFallbackDeltaMs..MaxFallbackDeltaMs) {
        return null
    }

    if (hasAccuracy() && accuracy > MaxReasonableAccuracyMeters) {
        return null
    }
    if (previous.hasAccuracy() && previous.accuracy > MaxReasonableAccuracyMeters) {
        return null
    }

    val distanceMeters = distanceTo(previous)
    val dtSeconds = dtMs / 1000f
    if (dtSeconds <= 0f) {
        return null
    }

    return (distanceMeters / dtSeconds).coerceAtLeast(0f)
}
