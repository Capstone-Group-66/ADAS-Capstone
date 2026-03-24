package com.example.testapp.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.NavigationBarItemDefaults
import androidx.compose.material3.NavigationDrawerItemDefaults
import androidx.compose.material3.NavigationRailItemDefaults
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteDefaults
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteItemColors
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.navigation.compose.rememberNavController
import com.example.testapp.BleConnectionStatus
import com.example.testapp.model.BleTickRepository
import com.example.testapp.ui.theme.AccentCyan
import com.example.testapp.ui.theme.CardBackground
import com.example.testapp.ui.theme.Charcoal
import com.example.testapp.ui.theme.CharcoalLight
import com.example.testapp.ui.theme.TextSecondary
import kotlinx.coroutines.flow.StateFlow

@Composable
fun TestAppApp(
    repository: BleTickRepository,
    logs: StateFlow<List<String>>,
    status: StateFlow<BleConnectionStatus>,
) {
    var currentDestination by rememberSaveable { mutableStateOf(AppDestinations.HOME) }
    val navController = rememberNavController()

    val navColors = adasNavColors
    val itemColors = adasItemColors

    Box(
        modifier =
            Modifier
                .fillMaxSize()
                .background(Charcoal),
    ) {
        AppNavigationUI(
            navController = navController,
            repository = repository,
            logs = logs,
            status = status,
            currentDestination = currentDestination,
            onDestinationChange = { currentDestination = it },
            navColors = navColors,
            itemColors = itemColors,
        )
    }
}

enum class AppDestinations(
    val label: String,
    val icon: ImageVector,
) {
    HOME("Home", Icons.Default.Home),
    DRIVE("drive", Icons.Default.Warning),
    SETTINGS("Settings", Icons.Default.Settings),
}

private val adasItemColors
    @Composable get() =
        NavigationSuiteItemColors(
            navigationBarItemColors =
                NavigationBarItemDefaults.colors(
                    selectedIconColor = AccentCyan,
                    selectedTextColor = AccentCyan,
                    unselectedIconColor = TextSecondary,
                    unselectedTextColor = TextSecondary,
                    indicatorColor = AccentCyan.copy(alpha = 0.15f),
                ),
            navigationRailItemColors =
                NavigationRailItemDefaults.colors(
                    selectedIconColor = AccentCyan,
                    selectedTextColor = AccentCyan,
                    unselectedIconColor = TextSecondary,
                    unselectedTextColor = TextSecondary,
                    indicatorColor = AccentCyan.copy(alpha = 0.15f),
                ),
            navigationDrawerItemColors =
                NavigationDrawerItemDefaults.colors(
                    selectedContainerColor = AccentCyan.copy(alpha = 0.12f),
                    selectedIconColor = AccentCyan,
                    selectedTextColor = AccentCyan,
                    unselectedIconColor = TextSecondary,
                    unselectedTextColor = TextSecondary,
                ),
        )

private val adasNavColors
    @Composable get() =
        NavigationSuiteDefaults.colors(
            // Bottom nav bar / rail container
            navigationBarContainerColor = CardBackground,
            navigationRailContainerColor = CardBackground,
            navigationDrawerContainerColor = CharcoalLight,
        )
