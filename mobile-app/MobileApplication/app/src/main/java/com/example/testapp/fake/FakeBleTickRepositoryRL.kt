package com.example.testapp.fake

import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.VehicleAlert
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.emptyFlow

/*
The RL stands for remove later, it will be removed for production and is just here for testing
 */
class FakeBleTickRepositoryRL(
    initial: VehicleAlert,
) : BleTickRepository(
        blePackets = emptyFlow(),
        serde = com.example.testapp.model.SerializationDeserialization,
        scope = CoroutineScope(Dispatchers.Unconfined),
    ) {
    private val stateFlow = MutableStateFlow(initial)
    override val dashboardState: StateFlow<VehicleAlert> = stateFlow

    fun emit(alert: VehicleAlert) {
        stateFlow.value = alert
    }
}
