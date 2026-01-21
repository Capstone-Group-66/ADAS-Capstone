package com.example.testapp.model

import kotlinx.coroutines.launch
import kotlinx.coroutines.delay
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.runningFold
import kotlinx.coroutines.flow.stateIn

open class BleTickRepository(
    blePackets: Flow<ByteArray>,
    scope: CoroutineScope,
) {
    private val debugFlow = kotlinx.coroutines.flow.MutableSharedFlow<TickPayload>()

    open val dashboardState: StateFlow<VehicleAlert> =
        kotlinx.coroutines.flow.merge(
            blePackets.map { TickDecoder.decode(it) },
            debugFlow
        )
            .runningFold(VehicleAlertReducer.initial()) { state, tick ->
                VehicleAlertReducer.reduce(state, tick) ?: state
            }.stateIn(
                scope = scope,
                started = SharingStarted.WhileSubscribed(5_000),
                initialValue = VehicleAlertReducer.initial(),
            )

    init {
        // Start a 1Hz heartbeat loop to refresh UI (handle Latch timing)
        scope.launch {
            while (true) {
                delay(1000)
                // Emit heartbeat tick (id=-1 means local update)
                val heartbeat = TickPayload(
                    tickId = -1,
                    speed = 0, healthMask = 0, bsdMask = 0, alerts = emptyList()
                )
                debugFlow.emit(heartbeat)
            }
        }
    }

    // Manual triggers for testing UI
    suspend fun simulateFcwAlert() {
        val alert = AlertDto(type = 0, severity = 2, rationale = "Debug FCW")
        // Use -2 to bypass sequence check and avoid corrupting state
        val tick = TickPayload(
            tickId = -2,
            speed = 88,
            healthMask = 7,
            bsdMask = 3,
            alerts = listOf(alert)
        )
        debugFlow.emit(tick)
    }

    suspend fun simulateClear() {
        val tick = TickPayload(
            tickId = -2,
            speed = 50,
            healthMask = 7,
            bsdMask = 0,
            alerts = emptyList()
        )
        debugFlow.emit(tick)
    }
}
