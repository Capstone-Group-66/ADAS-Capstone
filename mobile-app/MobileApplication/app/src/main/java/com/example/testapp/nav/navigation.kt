package com.example.testapp.nav

import androidx.compose.runtime.Composable
import androidx.compose.runtime.rememberCoroutineScope
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import com.example.testapp.BleConnectionStatus
import com.example.testapp.audio.AlertSoundObserver
import com.example.testapp.model.BleTickRepository
import com.example.testapp.screens.Drive
import com.example.testapp.screens.Home
import com.example.testapp.screens.Settings
import com.example.testapp.viewmodel.VehicleStatusViewModel
import kotlinx.coroutines.launch

@Composable
fun Navigation(
    navController: NavHostController,
    repository: BleTickRepository,
    logs: kotlinx.coroutines.flow.StateFlow<List<String>>,
    status: kotlinx.coroutines.flow.StateFlow<BleConnectionStatus>,
    viewModel: VehicleStatusViewModel,
    developerModeEnabled: Boolean,
    showBottomBar: Boolean,
    onDriveInteraction: () -> Unit,
) {
    val scope = rememberCoroutineScope()

    AlertSoundObserver(viewModel = viewModel)

    NavHost(
        navController = navController,
        startDestination = "home",
    ) {
        composable("home") {
            Home(
                logs = logs,
                status = status,
                developerModeEnabled = developerModeEnabled,
            )
        }
        composable("drive") {
            Drive(
                vehicleStatusViewModel = viewModel,
                developerModeEnabled = developerModeEnabled,
                showBottomBar = showBottomBar,
                onUserInteraction = onDriveInteraction,
                onDebugTrigger = { scope.launch { repository.simulateFcwAlert() } },
                onDebugClear = { scope.launch { repository.simulateClear() } },
            )
        }

        composable("settings") {
            Settings(viewModel)
        }
    }
}
