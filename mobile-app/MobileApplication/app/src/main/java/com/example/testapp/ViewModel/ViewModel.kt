package com.example.testapp.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.testapp.UpdateUIstate
import com.example.testapp.model.BleTickRepository
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.flow.stateIn

class ViewModel(
    repository: BleTickRepository,
) : ViewModel() {
    val driveState: StateFlow<UpdateUIstate> =
        repository.dashboardState
            .map { vehicleAlert ->
                UpdateUIstate(
                    status1 = vehicleAlert.cameras.rearOk,
                    status2 = vehicleAlert.radar.ok,
                    status3 = vehicleAlert.bsd.rightActive,
                    status4 = vehicleAlert.cameras.frontOk,
                    sonarValue = vehicleAlert.sonar.front.ordinal,
                )
            }.stateIn(
                scope = viewModelScope,
                started = SharingStarted.WhileSubscribed(5_000),
                initialValue =
                    UpdateUIstate(
                        status1 = false,
                        status2 = false,
                        status3 = false,
                        status4 = false,
                        sonarValue = 0,
                    ),
            )
}
