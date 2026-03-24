package com.example.testapp.ui.components

import androidx.compose.animation.AnimatedVisibility
import androidx.compose.animation.fadeIn
import androidx.compose.animation.fadeOut
import androidx.compose.animation.slideInVertically
import androidx.compose.animation.slideOutVertically
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.NavHostController
import com.example.testapp.BleConnectionStatus
import com.example.testapp.model.BleTickRepository
import com.example.testapp.nav.Navigation
import com.example.testapp.ui.theme.AccentCyan
import com.example.testapp.ui.theme.CardBackground
import com.example.testapp.ui.theme.Charcoal
import com.example.testapp.ui.theme.DividerColor
import com.example.testapp.ui.theme.TextSecondary
import com.example.testapp.viewmodel.VehicleStatusViewModel
import kotlinx.coroutines.flow.StateFlow

@Composable
fun AppNavigationUI(
    navController: NavHostController,
    repository: BleTickRepository,
    logs: StateFlow<List<String>>,
    status: StateFlow<BleConnectionStatus>,
    viewModel: VehicleStatusViewModel,
    currentDestination: AppDestinations,
    onDestinationChange: (AppDestinations) -> Unit,
    showBottomBar: Boolean,
    developerModeEnabled: Boolean,
    onDriveInteraction: () -> Unit,
) {
    val contentBottomPadding =
        if (currentDestination != AppDestinations.DRIVE) {
            88.dp
        } else {
            0.dp
        }

    Box(
        modifier =
            Modifier
                .fillMaxSize()
                .background(Charcoal),
    ) {
        Box(
            modifier =
                Modifier
                    .fillMaxSize()
                    .padding(bottom = contentBottomPadding),
        ) {
            Navigation(
                navController = navController,
                repository = repository,
                logs = logs,
                status = status,
                viewModel = viewModel,
                developerModeEnabled = developerModeEnabled,
                showBottomBar = showBottomBar,
                onDriveInteraction = onDriveInteraction,
            )

            TopAccentLine()
        }

        AnimatedVisibility(
            visible = showBottomBar,
            enter = slideInVertically(initialOffsetY = { it / 2 }) + fadeIn(),
            exit = slideOutVertically(targetOffsetY = { it / 2 }) + fadeOut(),
            modifier = Modifier.align(Alignment.BottomCenter),
        ) {
            NavigationBar(
                containerColor = CardBackground,
                tonalElevation = 0.dp,
            ) {
                AppDestinations.entries.forEach { destination ->
                    NavigationBarItem(
                        selected = currentDestination == destination,
                        onClick = {
                            onDestinationChange(destination)
                            when (destination) {
                                AppDestinations.HOME -> navController.navigate("home")
                                AppDestinations.DRIVE -> navController.navigate("drive")
                                AppDestinations.SETTINGS -> navController.navigate("settings")
                            }
                        },
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
                        alwaysShowLabel = true,
                        colors =
                            androidx.compose.material3.NavigationBarItemDefaults.colors(
                                selectedIconColor = AccentCyan,
                                selectedTextColor = AccentCyan,
                                unselectedIconColor = TextSecondary,
                                unselectedTextColor = TextSecondary,
                                indicatorColor = DividerColor.copy(alpha = 0.2f),
                            ),
                    )
                }
            }
        }
    }
}
