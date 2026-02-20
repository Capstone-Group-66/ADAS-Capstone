package com.example.testapp.model

import kotlinx.serialization.Serializable

@Serializable
data class GpsData(
    // timestamp
    val tsMs: Long,
    // speed in meters per second
    val speedMps: Float? = null,
)
