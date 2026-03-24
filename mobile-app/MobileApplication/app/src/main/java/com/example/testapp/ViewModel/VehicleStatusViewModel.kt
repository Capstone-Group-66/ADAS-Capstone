package com.example.testapp.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.testapp.BleConnectionStatus
import com.example.testapp.UpdateUIstate
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.SonarColor
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.stateIn

class VehicleStatusViewModel(
    repository: BleTickRepository,
    scope: CoroutineScope? = null,
) : ViewModel() {
    private val vmScope = scope ?: viewModelScope

    private val _alertSoundsEnabled = MutableStateFlow(true)
    val alertSoundsEnabled: StateFlow<Boolean> = _alertSoundsEnabled.asStateFlow()

    fun setAlertSoundsEnabled(enabled: Boolean) {
        _alertSoundsEnabled.value = enabled
    }

    val bleLogs: StateFlow<List<String>> = repository.bleLogs

    val connectionStatus: StateFlow<BleConnectionStatus> = repository.bleConnectionStatus

    val driveState: StateFlow<UpdateUIstate> =
        combine(
            repository.dashboardState,
            connectionStatus,
        ) { vehicleAlert, bleStatus ->
            UpdateUIstate(
                rearRcwOk = vehicleAlert.health.rearRcwOk,
                radarOk = vehicleAlert.health.radarOk,
                bleConnected = bleStatus.isConnected,
                frontCameraOk = vehicleAlert.health.frontCameraOk,
                speedKmh = vehicleAlert.telemetry.speedKmh,
                frontAlertColor = vehicleAlert.sonar.front,
                rearAlertColor = vehicleAlert.sonar.rear,
                leftBlindspotValue = vehicleAlert.sonar.left,
                rightBlindspotValue = vehicleAlert.sonar.right,
                timestamp = vehicleAlert.timestampMs,
                fcwExpiry = vehicleAlert.fcwExpiry,
                fcwSeverity = vehicleAlert.fcwSeverity,
            )
        }.stateIn(
            scope = vmScope,
            started = SharingStarted.WhileSubscribed(5_000),
            initialValue =
                UpdateUIstate(
                    rearRcwOk = false,
                    radarOk = false,
                    bleConnected = false,
                    frontCameraOk = false,
                    speedKmh = 0,
                    frontAlertColor = SonarColor.OFF,
                    rearAlertColor = SonarColor.OFF,
                    leftBlindspotValue = SonarColor.OFF,
                    rightBlindspotValue = SonarColor.OFF,
                ),
        )
}
