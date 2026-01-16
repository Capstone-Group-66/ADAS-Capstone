package com.example.testapp

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.Icon
import androidx.compose.material3.Text
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteScaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.tooling.preview.PreviewScreenSizes
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
import com.example.testapp.ui.theme.TestAppTheme
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.flow.emptyFlow

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
                blePackets = emptyFlow(),
                // ^replace with proper flow later
                scope = appScope,
            )

        enableEdgeToEdge()
        setContent {
            TestAppTheme {
                // sets the theme of the app (colours structure etc)

                // TestAppApp(repository), what we'll actually do on deploy
                TestAppAppPreview() // what were using for the IDE
            }
        }
    }
}

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

    TestAppApp(fakeRepository1)
}

@Composable
fun TestAppApp(repository: BleTickRepository) {
    var currentDestination by rememberSaveable { mutableStateOf(AppDestinations.HOME) }
    // give the app a navigation controller
    val navController = rememberNavController()

    NavigationSuiteScaffold(
        navigationSuiteItems = {
            // add the items to the navigation suite
            AppDestinations.entries.forEach { destination ->
                item(
                    icon = { Icon(destination.icon, contentDescription = destination.label) },
                    label = { Text(destination.label) },
                    selected = false,
                    // optional: can highlight current
                    onClick = {
                        // navigate via NavController
                        when (destination) {
                            AppDestinations.HOME -> navController.navigate("home")
                            AppDestinations.DRIVE -> navController.navigate("drive")
                            AppDestinations.SETTINGS -> navController.navigate("settings")
                        }
                    },
                )
            }
        },
    ) {
        // add the navigation graph to the scaffold and the repo
        Navigation(navController = navController, repository)
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
