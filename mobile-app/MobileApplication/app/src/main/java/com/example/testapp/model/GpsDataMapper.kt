package com.example.testapp.model

import android.location.Location

fun Location.toGpsData(): GpsData =
    GpsData(
        tsMs = time,
        speedMps = if (hasSpeed()) speed else null,
    )
