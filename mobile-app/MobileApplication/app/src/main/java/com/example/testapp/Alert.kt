package com.example.testapp

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class Point(
    val x: Float,
    val y: Float
)

@Serializable
data class Alert(
    val id: String,
    @SerialName("type") val type: String = "FCW", // Default to FCW if missing
    val timestamp: Double,
    val coordinates: List<Point> = emptyList() // Default to empty list
)

@Serializable
data class TickPayload(
    @SerialName("tick_id") val tickId: Int,
    @SerialName("seq_max") val seqMax: Int, // Included for completeness, though used in header
    val n: Int, // Number of alerts
    val alerts: List<Alert>
)
