package com.example.testapp.ui.components

import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.material3.Text
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteColors
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteItemColors
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteScaffold
import androidx.compose.runtime.Composable
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp
import androidx.navigation.NavHostController
import com.example.testapp.model.BleTickRepository
import com.example.testapp.nav.Navigation
import com.example.testapp.ui.theme.Charcoal
import kotlinx.coroutines.flow.StateFlow

@Composable
fun AppNavigationUI(
    navController: NavHostController,
    repository: BleTickRepository,
    logs: StateFlow<List<String>>,
    status: StateFlow<String>,
    currentDestination: AppDestinations,
    onDestinationChange: (AppDestinations) -> Unit,
    navColors: NavigationSuiteColors,
    itemColors: NavigationSuiteItemColors,
) {

    NavigationSuiteScaffold(
        navigationSuiteColors = navColors,
        navigationSuiteItems = {

            AppDestinations.entries.forEach { destination ->

                item(
                    icon = {
                        NavItemIcon(
                            icon = destination.icon,
                            label = destination.label,
                            selected = currentDestination == destination,
                        )
                    },
                    label = {
                        Text(
                            text = destination.label.uppercase(),
                            fontSize = 9.sp,
                            fontWeight = FontWeight.Bold,
                            letterSpacing = 1.sp,
                        )
                    },
                    selected = currentDestination == destination,
                    colors = itemColors,
                    onClick = {
                        onDestinationChange(destination)

                        when (destination) {
                            AppDestinations.HOME -> navController.navigate("home")
                            AppDestinations.DRIVE -> navController.navigate("drive")
                            AppDestinations.SETTINGS -> navController.navigate("settings")
                        }
                    }
                )
            }
        },
        modifier = Modifier.background(Charcoal),
    ) {
        Box(modifier = Modifier.fillMaxSize()) {

            Navigation(
                navController,
                repository,
                logs,
                status
            )

            TopAccentLine()
        }
    }
}
