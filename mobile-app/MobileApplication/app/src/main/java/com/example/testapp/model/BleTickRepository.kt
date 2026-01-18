package com.example.testapp.model

import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.flatMapConcat
import kotlinx.coroutines.flow.flowOf
import kotlinx.coroutines.flow.runningFold
import kotlinx.coroutines.flow.stateIn

open class BleTickRepository(
    blePackets: Flow<ByteArray>,
    serde: SerializationDeserialization,
    scope: CoroutineScope,
) {
    private val decoder = TickStreamDecoder(serde)

    @OptIn(ExperimentalCoroutinesApi::class)
    open val dashboardState: StateFlow<VehicleAlert> =
        blePackets
            .flatMapConcat { bytes ->
                flowOf(*decoder.onBytes(bytes).toTypedArray())
            }.runningFold(VehicleAlertReducer.initial()) { state, tick ->
                VehicleAlertReducer.reduce(state, tick) ?: state
            }.stateIn(
                scope = scope,
                started = SharingStarted.WhileSubscribed(5_000),
                initialValue = VehicleAlertReducer.initial(),
            )
}
