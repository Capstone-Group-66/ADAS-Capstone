package com.example.testapp.nav
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
<<<<<<< HEAD
import androidx.compose.runtime.rememberCoroutineScope
=======
>>>>>>> origin/main
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
<<<<<<< HEAD
import kotlinx.coroutines.launch
=======
>>>>>>> origin/main

@Composable
fun Navigation(
    navController: NavHostController,
    repository: BleTickRepository,
<<<<<<< HEAD
    logs: kotlinx.coroutines.flow.StateFlow<List<String>>,
    status: kotlinx.coroutines.flow.StateFlow<String>,
=======
>>>>>>> origin/main
) {
    val factory =
        remember {
            VehicleStatusViewModelFactory(repository)
        }

<<<<<<< HEAD
    val scope = rememberCoroutineScope()

=======
>>>>>>> origin/main
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

<<<<<<< HEAD
            Drive(
                vehicleStatusViewModel = vm,
                onDebugFcw = { scope.launch { repository.simulateFcwAlert() } },
                onDebugClear = { scope.launch { repository.simulateClear() } },
            )
=======
            Drive(vm)
>>>>>>> origin/main
        }

        composable("settings") { backStackEntry ->
            val vm: VehicleStatusViewModel =
                viewModel(factory = factory)

            settings(vm)
        }
    }
}
