package com.example.testapp

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.animation.core.EaseInOutSine
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.Icon
import androidx.compose.material3.NavigationBarItemDefaults
import androidx.compose.material3.NavigationDrawerItemDefaults
import androidx.compose.material3.NavigationRailItemDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteDefaults
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteItemColors
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteScaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.tooling.preview.PreviewScreenSizes
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.navigation.compose.rememberNavController
import com.example.testapp.fake.FakeBleTickRepositoryRL
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.BlindSpotStatus
import com.example.testapp.model.CameraHealth
import com.example.testapp.model.ObjectDetection
import com.example.testapp.model.RadarHealth
import com.example.testapp.model.SonarColor
import com.example.testapp.model.SonarColors
import com.example.testapp.model.VehicleAlert
import com.example.testapp.model.VehicleTelemetry
import com.example.testapp.nav.Navigation
import com.example.testapp.ui.theme.AccentCyan
import com.example.testapp.ui.theme.AccentCyanDim
import com.example.testapp.ui.theme.CardBackground
import com.example.testapp.ui.theme.Charcoal
import com.example.testapp.ui.theme.CharcoalLight
import com.example.testapp.ui.theme.TestAppTheme
import com.example.testapp.ui.theme.TextSecondary
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob

class MainActivity : ComponentActivity() {
    private val appScope =
        CoroutineScope(
            SupervisorJob() + Dispatchers.Default,
        )

    private lateinit var repository: BleTickRepository

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val bleManager = BleManager(this)

        repository =
            BleTickRepository(
                blePackets = bleManager.packetFlow,
                scope = appScope,
            )

        // Initialize BLE (scan/connect)
        if (checkSelfPermission(android.Manifest.permission.BLUETOOTH_CONNECT) == android.content.pm.PackageManager.PERMISSION_GRANTED) {
            bleManager.initialize()
        }

        enableEdgeToEdge()
        setContent {
            TestAppTheme {
                // sets the theme of the app (colours structure etc)
                // Pass logs and status to the app composable
                TestAppApp(
                    repository,
                    logs = bleManager.logFlow,
                    status = bleManager.connectionState,
                )
            }
        }
    }
}

// ── Custom NavigationSuite colours ────────────────────────────────────────────
private val adasNavColors
    @Composable get() = NavigationSuiteDefaults.colors(
        // Bottom nav bar / rail container
        navigationBarContainerColor  = CardBackground,
        navigationRailContainerColor = CardBackground,
        navigationDrawerContainerColor = CharcoalLight,


    )

@PreviewScreenSizes
@Composable
fun TestAppAppPreview() {
    val vehicleAlert =
        VehicleAlert(
            cameras = CameraHealth(frontOk = true, rearOk = true),
            radar = RadarHealth(ok = true),
            sonar =
                SonarColors(
                    front = SonarColor.GREEN,
                    rear = SonarColor.GREEN,
                    left = SonarColor.OFF,
                    right = SonarColor.YELLOW,
                ),
            telemetry = VehicleTelemetry(speedKmh = 50),
            detection = ObjectDetection.None,
            bsd = BlindSpotStatus(leftActive = true, rightActive = true),
        )

    val fakeRepository1 = FakeBleTickRepositoryRL(vehicleAlert)

    // Dummy flows for preview
    val logs = kotlinx.coroutines.flow.MutableStateFlow(listOf("Log 1", "Log 2"))
    val status = kotlinx.coroutines.flow.MutableStateFlow("Connected")

    TestAppApp(fakeRepository1, logs, status)
}

// ── App Entry Composable ──────────────────────────────────────────────────────
@Composable
fun TestAppApp(
    repository: BleTickRepository,
    logs: kotlinx.coroutines.flow.StateFlow<List<String>>,
    status: kotlinx.coroutines.flow.StateFlow<String>,
) {
    var currentDestination by rememberSaveable { mutableStateOf(AppDestinations.HOME) }
    val navController = rememberNavController()

    val navColors  = adasNavColors
    val itemColors = adasItemColors


    // Wrap everything in charcoal so no white flashes appear
    Box(modifier = Modifier
        .fillMaxSize()
        .background(Charcoal)
    ) {
        NavigationSuiteScaffold(
            navigationSuiteColors = navColors,
            navigationSuiteItems = {
                AppDestinations.entries.forEach { destination ->
                    item(
                        icon = {
                            NavItemIcon(
                                icon      = destination.icon,
                                label     = destination.label,
                                selected  = currentDestination == destination,
                            )
                        },
                        label = {
                            Text(
                                text       = destination.label.uppercase(),
                                fontSize   = 9.sp,
                                fontWeight = FontWeight.Bold,
                                letterSpacing = 1.sp,
                            )
                        },
                        selected = currentDestination == destination,
                        colors   = itemColors,
                        onClick = {
                            currentDestination = destination
                            when (destination) {
                                AppDestinations.HOME     -> navController.navigate("home")
                                AppDestinations.DRIVE    -> navController.navigate("drive")
                                AppDestinations.SETTINGS -> navController.navigate("settings")
                            }
                        },
                    )
                }
            },
            modifier = Modifier.background(Charcoal),
        ) {
            // Top cyan accent line above content area
            Box(modifier = Modifier.fillMaxSize()) {
                Navigation(navController, repository, logs, status)
                TopAccentLine()
            }
        }
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



// ── Custom item colours ───────────────────────────────────────────────────────
private val adasItemColors
    @Composable get() = NavigationSuiteItemColors(
        navigationBarItemColors = NavigationBarItemDefaults.colors(
            selectedIconColor    = AccentCyan,
            selectedTextColor    = AccentCyan,
            unselectedIconColor  = TextSecondary,
            unselectedTextColor  = TextSecondary,
            indicatorColor       = AccentCyan.copy(alpha = 0.15f),
        ),
        navigationRailItemColors = NavigationRailItemDefaults.colors(
            selectedIconColor    = AccentCyan,
            selectedTextColor    = AccentCyan,
            unselectedIconColor  = TextSecondary,
            unselectedTextColor  = TextSecondary,
            indicatorColor       = AccentCyan.copy(alpha = 0.15f),
        ),
        navigationDrawerItemColors = NavigationDrawerItemDefaults.colors(
            selectedContainerColor   = AccentCyan.copy(alpha = 0.12f),
            selectedIconColor        = AccentCyan,
            selectedTextColor        = AccentCyan,
            unselectedIconColor      = TextSecondary,
            unselectedTextColor      = TextSecondary,
        ),
    )

// ── Thin glowing cyan line that sits at the top of the content area ───────────
@Composable
private fun TopAccentLine() {
    Box(
        modifier = Modifier
            .fillMaxWidth()
            .height(1.dp)
            .drawBehind {
                drawLine(
                    brush = Brush.horizontalGradient(
                        listOf(
                            Color.Transparent,
                            AccentCyan.copy(alpha = 0.5f),
                            AccentCyanDim.copy(alpha = 0.8f),
                            AccentCyan.copy(alpha = 0.5f),
                            Color.Transparent,
                        )
                    ),
                    start = Offset(0f, 0f),
                    end   = Offset(size.width, 0f),
                    strokeWidth = size.height,
                )
            }
    )
}

fun drawLine(brush: Any, start: Any, end: Any, strokeWidth: Any) {}

// ── Icon with a pulsing glow ring when selected ───────────────────────────────
@Composable
private fun NavItemIcon(
    icon: ImageVector,
    label: String,
    selected: Boolean,
) {
    Box(contentAlignment = Alignment.Center) {
        if (selected) {
            val infiniteTransition = rememberInfiniteTransition(label = "navGlow")
            val glowAlpha by infiniteTransition.animateFloat(
                initialValue = 0.2f,
                targetValue  = 0.55f,
                animationSpec = infiniteRepeatable(
                    animation  = tween(1000, easing = EaseInOutSine),
                    repeatMode = RepeatMode.Reverse,
                ),
                label = "glowAlpha",
            )
            Box(
                modifier = Modifier
                    .size(32.dp)
                    .drawBehind {
                        drawCircle(
                            brush = Brush.radialGradient(
                                listOf(
                                    AccentCyan.copy(alpha = glowAlpha),
                                    Color.Transparent,
                                )
                            ),
                            radius = size.minDimension / 1.4f,
                        )
                    }
            )
        }
        Icon(
            imageVector     = icon,
            contentDescription = label,
            tint            = if (selected) AccentCyan else TextSecondary,
            modifier        = Modifier.size(22.dp),
        )
    }
}
