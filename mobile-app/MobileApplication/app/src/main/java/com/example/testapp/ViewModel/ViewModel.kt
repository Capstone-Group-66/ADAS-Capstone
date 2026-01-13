package com.example.testapp.viewmodel

import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue
import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.VehicleAlert
import com.example.testapp.updateUIstate
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn

class ViewModel(
    repository: BleTickRepository
) : ViewModel() {

    val driveState: StateFlow<updateUIstate> = repository.dashboardState
        .map { vehicleAlert ->
            updateUIstate(
                status1 = vehicleAlert.cameras.rearOk,
                status2 = vehicleAlert.radar.ok,
                status3 = vehicleAlert.bsd.rightActive,
                status4 = vehicleAlert.cameras.frontOk,
                sonarValue = vehicleAlert.sonar.front.ordinal,
            )
        }
        .stateIn(
            scope = viewModelScope,
            started = SharingStarted.WhileSubscribed(5_000),
            initialValue = updateUIstate(
                status1 = false,
                status2 = false,
                status3 = false,
                status4 = false,
                sonarValue = 0,
            )
        )
}
