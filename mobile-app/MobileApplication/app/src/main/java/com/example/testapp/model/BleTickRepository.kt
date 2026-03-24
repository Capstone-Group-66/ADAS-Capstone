package com.example.testapp.model

import com.example.testapp.BleConnectionStatus
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.merge
import kotlinx.coroutines.flow.runningFold
import kotlinx.coroutines.flow.stateIn
import kotlinx.coroutines.launch

open class BleTickRepository(
    blePackets: Flow<ByteArray>,
    scope: CoroutineScope,
    logs: StateFlow<List<String>> = MutableStateFlow(emptyList()),
    connectionStatus: StateFlow<BleConnectionStatus> = MutableStateFlow(BleConnectionStatus.disconnected()),
) {
    private val debugFlow = MutableSharedFlow<TickPayload>()

    val bleLogs: StateFlow<List<String>> = logs
    val bleConnectionStatus: StateFlow<BleConnectionStatus> = connectionStatus

    open val dashboardState: StateFlow<VehicleAlert> =
        merge(
            blePackets.map { TickDecoder.decode(it) },
            debugFlow,
        ).runningFold(VehicleAlertReducer.initial()) { state, tick ->
            VehicleAlertReducer.reduce(state, tick) ?: state
        }.stateIn(
            scope = scope,
            started = SharingStarted.WhileSubscribed(5_000),
            initialValue = VehicleAlertReducer.initial(),
        )

    init {
        scope.launch {
            while (true) {
                delay(1000)
                val heartbeat =
                    TickPayload(
                        tickId = -1,
                        speed = 0,
                        healthMask = 0,
                        bsdMask = 0,
                        alerts = emptyList(),
                    )
                debugFlow.emit(heartbeat)
            }
        }
    }

    suspend fun simulateFcwAlert() {
        val alert = AlertDto(type = 0, severity = 2, rationale = "Debug FCW")
        val tick =
            TickPayload(
                tickId = -2,
                speed = 88,
                healthMask = 0,
                bsdMask = 0,
                alerts = listOf(alert),
            )
        debugFlow.emit(tick)
    }

    suspend fun simulateClear() {
        val tick =
            TickPayload(
                tickId = -2,
                speed = 50,
                healthMask = 0,
                bsdMask = 0,
                alerts = emptyList(),
            )
        debugFlow.emit(tick)
    }
}
