package com.example.testapp

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.padding
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.AccountBox
import androidx.compose.material.icons.filled.Favorite
import androidx.compose.material.icons.filled.Home
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material.icons.filled.Warning
import androidx.compose.material3.Icon
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Text
import androidx.compose.material3.adaptive.navigationsuite.NavigationSuiteScaffold
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.saveable.rememberSaveable
import androidx.compose.runtime.setValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.tooling.preview.Preview
import androidx.compose.ui.tooling.preview.PreviewScreenSizes
import androidx.navigation.compose.rememberNavController
import com.example.testapp.nav.Navigation
import com.example.testapp.ui.theme.TestAppTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        setContent {
            TestAppTheme {
                // sets the theme of the app (colours structure etc)

                TestAppApp()
            }
        }
    }
}

@PreviewScreenSizes
@Composable
fun TestAppApp() {
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
                    selected = false, // optional: can highlight current
                    onClick = {
                        // navigate via NavController
                        when(destination) {
                            AppDestinations.HOME -> navController.navigate("home")
                            AppDestinations.DRIVE -> navController.navigate("drive")
                            AppDestinations.SETTINGS -> navController.navigate("settings")
                        }
                    }
                )
            }
        }
    ) {
        // add the navigation graph to the scaffold
        Navigation(navController = navController)
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

@Composable
fun Greeting(name: String, modifier: Modifier = Modifier) {
    Text(
        text = "Hello $name!",
        modifier = modifier
    )
}

@Preview(showBackground = true)
@Composable
fun GreetingPreview() {
    TestAppTheme {
        Greeting("Android")
    }
}