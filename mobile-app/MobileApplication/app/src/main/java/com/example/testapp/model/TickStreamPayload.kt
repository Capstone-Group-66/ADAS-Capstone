package com.example.testapp.model

import kotlinx.serialization.SerialName
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.JsonElement

@Serializable
data class TickStreamPayload(
    @SerialName("tick_id")
    val tickId: Int,

    @SerialName("n")
    val n: Int = 0, // can be omitted in some cases; default keeps decoding resilient

    @SerialName("alerts")
    val alerts: List<StreamAlert> = emptyList()
)

@Serializable
data class StreamAlert(
    @SerialName("t_ms") val tMs: Long,
    @SerialName("id") val id: String,
    @SerialName("type") val type: Int,
    @SerialName("severity") val severity: Int,
    @SerialName("direction") val direction: String? = null,
    @SerialName("ttl_ms") val ttlMs: Int,
    @SerialName("schema") val schema: String,
    @SerialName("confidence") val confidence: Double,
    @SerialName("rationale") val rationale: StreamAlertRationale? = null
)

@Serializable
data class StreamAlertRationale(
    val ttc: Double? = null,
    val range: Double? = null,
    val closing: Double? = null,
    @SerialName("in_path") val inPath: Boolean? = null,
    @SerialName("d_ego_lane") val dEgoLane: Double? = null,
    val zone: String? = null
)
