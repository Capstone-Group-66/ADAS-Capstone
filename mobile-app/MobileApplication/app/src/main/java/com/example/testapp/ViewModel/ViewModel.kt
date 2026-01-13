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
    // Each badge has its own state
    private val _status1 = MutableStateFlow(true)
    val status1 = _status1.asStateFlow()

    private val _status2 = MutableStateFlow(false)
    val status2 = _status2.asStateFlow()

    private val _status3 = MutableStateFlow(true)
    val status3 = _status3.asStateFlow()

    private val _status4 = MutableStateFlow(false)
    val status4 = _status4.asStateFlow()

    var sonarValue = mutableStateOf(0)
        private set

    // Example: methods that update values
    fun setStatus1(value: Boolean) {
        _status1.value = value
    }

    fun setStatus2(value: Boolean) {
        _status2.value = value
    }

    fun setStatus3(value: Boolean) {
        _status3.value = value
    }

    fun setStatus4(value: Boolean) {
        _status4.value = value
    }

    fun updateValue(value: Int) {
        sonarValue.value = value
    }

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
