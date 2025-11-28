package com.example.testapp.nav

import androidx.compose.runtime.Composable
import androidx.lifecycle.viewmodel.compose.viewModel
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import com.example.testapp.ViewModel.ViewModel
import com.example.testapp.screens.Drive
import com.example.testapp.screens.home
import com.example.testapp.screens.settings

@Composable
fun Navigation(navController: NavHostController) {
    NavHost(
        navController = navController,
        startDestination = "home",
    ) {
        composable("home") { home() }
        composable("drive") { backStackEntry ->
            val sharedVM: ViewModel = viewModel(backStackEntry)
            Drive(sharedVM)
        }

        composable("settings") { backStackEntry ->
            val sharedVM: ViewModel = viewModel(backStackEntry)
            settings(sharedVM)
        }
    }
}
