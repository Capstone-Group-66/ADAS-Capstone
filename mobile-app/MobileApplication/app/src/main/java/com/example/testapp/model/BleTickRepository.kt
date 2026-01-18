package com.example.testapp.model

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
    private val serde = SerializationDeserialization
    private val streamDecoder = TickStreamDecoder(serde)
    open val dashboardState: StateFlow<VehicleAlert> =
        blePackets
            .map{ streamDecoder.onBytes(it) }
            .runningFold(VehicleAlertReducer.initial()) { state, tick ->
                // Because of the change above, 'tick' is now the correct type (TickStreamPayload)
                VehicleAlertReducer.reduce(state, tick) ?: state
            }.stateIn(
                scope = scope,
                started = SharingStarted.WhileSubscribed(5_000),
                initialValue = VehicleAlertReducer.initial(),
            )
}
}
