package com.example.testapp.nav
import androidx.compose.runtime.Composable
import androidx.compose.runtime.remember
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

@Composable
fun Navigation(
    navController: NavHostController,
    repository: BleTickRepository,
) {
    val factory =
        remember {
            VehicleStatusViewModelFactory(repository)
        }

    NavHost(
        navController = navController,
        startDestination = "home",
    ) {
        composable("home") { home() }
        composable("drive") { backStackEntry ->
            val vm: VehicleStatusViewModel =
                viewModel(factory = factory)

            Drive(vm)
        }

        composable("settings") { backStackEntry ->
            val vm: VehicleStatusViewModel =
                viewModel(factory = factory)

            settings(vm)
        }
    }
}
