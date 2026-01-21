package com.example.testapp.model
import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable

@Serializable
data class TickPayload(
    @SerialName("t") val tickId: Int,
    @SerialName("v") val speed: Int,
    @SerialName("h") val healthMask: Int,
    @SerialName("b") val bsdMask: Int,
    @SerialName("a") val alerts: List<AlertDto>,
)

@Serializable
data class AlertDto(
    // Wire format: 0=FCW, 1=LDW, 2=RCW, 3=BSD
    @SerialName("id") val type: Int,
    @SerialName("s") val severity: Int,
    @SerialName("r") val rationale: String,
)
