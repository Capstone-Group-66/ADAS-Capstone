package com.example.testapp.ui.components

import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Warning
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.compose.rememberNavController
import com.example.testapp.BleConnectionStatus
import com.example.testapp.model.BleTickRepository
import com.example.testapp.viewmodel.VehicleStatusViewModel
import com.example.testapp.viewmodel.VehicleStatusViewModelFactory
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.StateFlow

private const val DriveNavHideDelayMs = 5000L

@Composable
fun TestAppApp(
    repository: BleTickRepository,
    logs: StateFlow<List<String>>,
    status: StateFlow<BleConnectionStatus>,
) {
    var currentDestination by rememberSaveable { mutableStateOf(AppDestinations.HOME) }
    val navController = rememberNavController()
    val factory = remember { VehicleStatusViewModelFactory(repository) }
    val viewModel: VehicleStatusViewModel = viewModel(factory = factory)
    val developerModeEnabled by viewModel.developerModeEnabled.collectAsState()

    var bottomBarVisible by rememberSaveable { mutableStateOf(true) }
    var driveInteractionTick by rememberSaveable { mutableStateOf(0) }

    val shouldAutoHideNav =
        currentDestination == AppDestinations.DRIVE && !developerModeEnabled

    LaunchedEffect(currentDestination, developerModeEnabled, driveInteractionTick) {
        if (!shouldAutoHideNav) {
            bottomBarVisible = true
            return@LaunchedEffect
        }

        bottomBarVisible = true
        delay(DriveNavHideDelayMs)
        bottomBarVisible = false
    }

    AppNavigationUI(
        navController = navController,
        repository = repository,
        logs = logs,
        status = status,
        viewModel = viewModel,
        currentDestination = currentDestination,
        onDestinationChange = { destination ->
            currentDestination = destination
            bottomBarVisible = true
        },
        showBottomBar = bottomBarVisible,
        developerModeEnabled = developerModeEnabled,
        onDriveInteraction = {
            if (shouldAutoHideNav) {
                bottomBarVisible = true
                driveInteractionTick += 1
            }
        },
    )
}

enum class AppDestinations(
    val label: String,
    val icon: ImageVector,
) {
    HOME("Home", Icons.Default.Home),
    DRIVE("Drive", Icons.Default.Warning),
    SETTINGS("Settings", Icons.Default.Settings),
}
