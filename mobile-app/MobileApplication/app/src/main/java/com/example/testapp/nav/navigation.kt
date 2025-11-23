package com.example.testapp.nav

import android.net.wifi.hotspot2.pps.HomeSp
import androidx.compose.animation.rememberSplineBasedDecay
import androidx.compose.runtime.Composable
import androidx.navigation.NavHostController
import androidx.navigation.compose.NavHost
import androidx.navigation.compose.composable
import androidx.navigation.compose.rememberNavController
import com.example.testapp.screens.Drive
import com.example.testapp.screens.settings
import com.example.testapp.screens.home


@Composable
fun Navigation(navController: NavHostController) {


    NavHost(
        navController = navController,
        startDestination = "home"

    ) {
        composable("home") { home() }
        composable("drive") { Drive() }
        composable("settings") { settings() }

    }
}