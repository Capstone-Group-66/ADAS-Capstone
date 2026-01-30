package com.example.testapp

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class Point(
    val x: Float,
    val y: Float,
)

@Serializable
data class Alert(
    val id: String,
    // Default to FCW if missing
    @SerialName("type") val type: String = "FCW",
    val timestamp: Double,
    // Default to empty list
    val coordinates: List<Point> = emptyList(),
)

@Serializable
data class TickPayload(
    @SerialName("tick_id") val tickId: Int,
    // Included for completeness, though used in header
    @SerialName("seq_max") val seqMax: Int,
    // Timestamp of the alert
    val timestamp: Long,
    // Number of alerts
    val n: Int,
    val alerts: List<Alert>,
)
