package com.example.testapp.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.viewModelScope
import com.example.testapp.BleConnectionStatus
import com.example.testapp.UpdateUIstate
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.GpsData
import com.example.testapp.model.SonarColor
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharingStarted
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.combine
import kotlinx.coroutines.flow.stateIn
import kotlin.math.abs
import kotlin.math.roundToInt

class VehicleStatusViewModel(
    repository: BleTickRepository,
    phoneGpsData: StateFlow<GpsData?> = MutableStateFlow(null),
    scope: CoroutineScope? = null,
) : ViewModel() {
    private val vmScope = scope ?: viewModelScope

    private val _alertSoundsEnabled = MutableStateFlow(true)
    val alertSoundsEnabled: StateFlow<Boolean> = _alertSoundsEnabled.asStateFlow()
    private val _developerModeEnabled = MutableStateFlow(true)
    val developerModeEnabled: StateFlow<Boolean> = _developerModeEnabled.asStateFlow()

    fun setAlertSoundsEnabled(enabled: Boolean) {
        _alertSoundsEnabled.value = enabled
    }

    fun setDeveloperModeEnabled(enabled: Boolean) {
        _developerModeEnabled.value = enabled
    }

    val bleLogs: StateFlow<List<String>> = repository.bleLogs

    val connectionStatus: StateFlow<BleConnectionStatus> = repository.bleConnectionStatus

    val driveState: StateFlow<UpdateUIstate> =
        combine(
            repository.dashboardState,
            connectionStatus,
            phoneGpsData,
        ) { vehicleAlert, bleStatus, gpsFix ->
            val localPhoneSpeedKmh =
                gpsFix
                    ?.speedMps
                    ?.let { speedMps -> abs(speedMps * 3.6f).roundToInt().coerceIn(0, 300) }
            val speedDebugHint =
                if (localPhoneSpeedKmh == null) {
                    "GPS waiting"
                } else {
                    null
                }
            UpdateUIstate(
                rearRcwOk = vehicleAlert.health.rearRcwOk,
                radarOk = vehicleAlert.health.radarOk,
                bleConnected = bleStatus.isConnected,
                frontCameraOk = vehicleAlert.health.frontCameraOk,
                speedKmh = localPhoneSpeedKmh ?: vehicleAlert.telemetry.speedKmh,
                speedDebugHint = speedDebugHint,
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
                    speedDebugHint = "GPS waiting",
                    frontAlertColor = SonarColor.OFF,
                    rearAlertColor = SonarColor.OFF,
                    leftBlindspotValue = SonarColor.OFF,
                    rightBlindspotValue = SonarColor.OFF,
                ),
        )
}
