package com.example.testapp.model

data class GpsData(
    // timestamp
    val tsMs: Long,
    // latitude
    val lat: Double,
    // longitude
    val lon: Double,
    // altitude in meters
    val altM: Double? = null,
    // speed in meters per second
    val speedMps: Float? = null,
    // bearing in degrees
    val bearingDeg: Float? = null,
    // accuracy in meters
    val accM: Float? = null,
)
