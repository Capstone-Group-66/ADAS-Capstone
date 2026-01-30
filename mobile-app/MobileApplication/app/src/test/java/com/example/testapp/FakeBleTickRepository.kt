package com.example.testapp

import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.VehicleAlert
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.emptyFlow

class FakeBleTickRepository(
    initial: VehicleAlert,
) : BleTickRepository(
        blePackets = emptyFlow(),
<<<<<<< HEAD
=======
        serde = com.example.testapp.model.SerializationDeserialization,
>>>>>>> origin/main
        scope = CoroutineScope(Dispatchers.Unconfined),
    ) {
    private val stateFlow = MutableStateFlow(initial)
    override val dashboardState: StateFlow<VehicleAlert> = stateFlow

    fun emit(alert: VehicleAlert) {
        stateFlow.value = alert
    }
}
