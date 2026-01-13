package com.example.testapp.model

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.runningFold
import kotlinx.coroutines.flow.stateIn

class BleTickRepository(
    blePackets: Flow<ByteArray>,
    scope: CoroutineScope,
) {
    val dashboardState: StateFlow<VehicleAlert> = blePackets
        .map { TickDecoder.decode(it) }
        .runningFold(VehicleAlertReducer.initial()) { state, tick ->
            VehicleAlertReducer.reduce(state, tick) ?: state
        }
        .stateIn(
            scope = scope,
            started = SharingStarted.WhileSubscribed(5_000),
            initialValue = VehicleAlertReducer.initial(),
        )
}
