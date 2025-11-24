package com.example.testapp.nav

import androidx.compose.runtime.Composable
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
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
        composable("drive") { Drive() }
        composable("settings") { settings() }
    }
}
