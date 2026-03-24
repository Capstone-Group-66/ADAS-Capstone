package com.example.testapp

import android.Manifest
import android.annotation.SuppressLint
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.activity.result.contract.ActivityResultContracts
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.GpsData
import com.example.testapp.model.GpsTracker
import com.example.testapp.model.GpsTrackingRepository
import com.example.testapp.ui.components.TestAppApp
import com.example.testapp.ui.theme.TestAppTheme
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.flow.collectLatest
import kotlinx.coroutines.flow.distinctUntilChanged
import kotlinx.coroutines.flow.filterNotNull
import kotlinx.coroutines.flow.map
import kotlinx.coroutines.launch

class MainActivity : ComponentActivity() {
    private val appScope =
        CoroutineScope(
            SupervisorJob() + Dispatchers.Default,
        )

    private lateinit var bleManager: BleManager
    private lateinit var repository: BleTickRepository
    private lateinit var gpsRepository: GpsTrackingRepository
    private val phoneGpsData = MutableStateFlow<GpsData?>(null)
    private var permissionRequestInFlight = false
    private var gpsMonitoringJob: Job? = null
    private var gpsForwardingJob: Job? = null

    private val permissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { _ ->
            permissionRequestInFlight = false
            if (hasRequiredAppPermissions()) {
                startGpsMonitoring()
                maybeStartBle()
            } else {
                stopGpsMonitoring()
                bleManager.setConnectionStatus(
                    BleConnectionStatus.permissionRequired(permissionBlockedLabel()),
                )
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)

        bleManager = BleManager(this)
        gpsRepository = GpsTrackingRepository(GpsTracker(this))

        repository =
            BleTickRepository(
                blePackets = bleManager.packetFlow,
                scope = appScope,
                logs = bleManager.logFlow,
                connectionStatus = bleManager.connectionStatus,
            )

        enableEdgeToEdge()

        setContent {
            TestAppTheme {
                TestAppApp(
                    repository,
                    gpsData = phoneGpsData.asStateFlow(),
                    logs = bleManager.logFlow,
                    status = bleManager.connectionStatus,
                )
            }
        }
    }

    override fun onStart() {
        super.onStart()
        startGpsMonitoring()
        startGpsForwarding()
        maybeStartBle()
    }

    override fun onStop() {
        stopGpsMonitoring()
        stopGpsForwarding()
        bleManager.shutdown()
        super.onStop()
    }

    override fun onDestroy() {
        stopGpsMonitoring()
        stopGpsForwarding()
        bleManager.shutdown()
        appScope.cancel()
        super.onDestroy()
    }

    private fun maybeStartBle() {
        if (hasRequiredAppPermissions()) {
            startGpsMonitoring()
            try {
                bleManager.initialize()
            } catch (_: SecurityException) {
                bleManager.setConnectionStatus(
                    BleConnectionStatus.permissionRequired("Bluetooth or location permission error"),
                )
            }
            return
        }

        bleManager.setConnectionStatus(
            BleConnectionStatus.permissionRequired(permissionBlockedLabel()),
        )

        if (!permissionRequestInFlight) {
            permissionRequestInFlight = true
            permissionLauncher.launch(requiredAppPermissions())
        }
    }

    @SuppressLint("MissingPermission")
    private fun startGpsMonitoring() {
        if (gpsMonitoringJob != null || !hasLocationPermission()) {
            if (!hasLocationPermission()) {
                phoneGpsData.value = null
            }
            return
        }

        gpsMonitoringJob =
            appScope.launch {
                try {
                    gpsRepository.gpsDataFlow().collect { gpsFix ->
                        phoneGpsData.value = gpsFix
                    }
                } catch (_: SecurityException) {
                    phoneGpsData.value = null
                    bleManager.setConnectionStatus(
                        BleConnectionStatus.permissionRequired(permissionBlockedLabel()),
                    )
                }
            }
    }

    private fun stopGpsMonitoring() {
        gpsMonitoringJob?.cancel()
        gpsMonitoringJob = null
        phoneGpsData.value = null
    }

    @SuppressLint("MissingPermission")
    private fun startGpsForwarding() {
        if (gpsForwardingJob != null) {
            return
        }

        gpsForwardingJob =
            appScope.launch {
                bleManager.connectionStatus
                    .map { it.isConnected }
                    .distinctUntilChanged()
                    .collectLatest { isConnected ->
                        if (!isConnected || !hasLocationPermission()) {
                            return@collectLatest
                        }

                        phoneGpsData.filterNotNull().collect { gpsFix ->
                            if (gpsFix.speedMps == null) {
                                return@collect
                            }
                            try {
                                bleManager.sendGpsData(gpsFix)
                            } catch (_: SecurityException) {
                                bleManager.setConnectionStatus(
                                    BleConnectionStatus.permissionRequired(permissionBlockedLabel()),
                                )
                            }
                        }
                    }
            }
    }

    private fun stopGpsForwarding() {
        gpsForwardingJob?.cancel()
        gpsForwardingJob = null
    }

    private fun hasRequiredAppPermissions(): Boolean =
        requiredAppPermissions().all { permission ->
            checkSelfPermission(permission) == PackageManager.PERMISSION_GRANTED
        }

    private fun hasLocationPermission(): Boolean =
        checkSelfPermission(Manifest.permission.ACCESS_FINE_LOCATION) == PackageManager.PERMISSION_GRANTED

    private fun requiredAppPermissions(): Array<String> =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.ACCESS_FINE_LOCATION,
            )
        } else {
            arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }

    private fun permissionBlockedLabel(): String =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            "Bluetooth scan/connect and location permissions required"
        } else {
            "Location permission required for Bluetooth and GPS"
        }
}
