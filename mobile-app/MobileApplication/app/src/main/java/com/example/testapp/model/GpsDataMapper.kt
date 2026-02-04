package com.example.testapp.model

import android.location.Location

fun Location.toGpsData(): GpsData =
    GpsData(
        tsMs = time,
        lat = latitude,
        lon = longitude,
        altM = if (hasAltitude()) altitude else null,
        speedMps = if (hasSpeed()) speed else null,
        bearingDeg = if (hasBearing()) bearing else null,
        accM = if (hasAccuracy()) accuracy else null,
    )
