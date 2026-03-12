package com.example.testapp.nav
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import com.example.testapp.model.BleTickRepository
import com.example.testapp.screens.Drive
import com.example.testapp.screens.home
import com.example.testapp.screens.settings
import com.example.testapp.viewmodel.VehicleStatusViewModel
import com.example.testapp.viewmodel.VehicleStatusViewModelFactory
import kotlinx.coroutines.launch

@Composable
fun Navigation(
    navController: NavHostController,
    repository: BleTickRepository,
    logs: kotlinx.coroutines.flow.StateFlow<List<String>>,
    status: kotlinx.coroutines.flow.StateFlow<String>,
) {
    val factory =
        remember {
            VehicleStatusViewModelFactory(repository)
        }

    val scope = rememberCoroutineScope()

    NavHost(
        navController = navController,
        startDestination = "home",
    ) {
        composable("home") {
            home(logs, status)
        }
        composable("drive") { backStackEntry ->
            val vm: VehicleStatusViewModel =
                viewModel(factory = factory)

            Drive(
                vehicleStatusViewModel = vm,
                onDebugTrigger = { scope.launch { repository.simulateFcwAlert() } },
                onDebugClear = { scope.launch { repository.simulateClear() } },
            )
        }

        composable("settings") { backStackEntry ->
            val vm: VehicleStatusViewModel =
                viewModel(factory = factory)

            settings(vm)
        }
    }
}
