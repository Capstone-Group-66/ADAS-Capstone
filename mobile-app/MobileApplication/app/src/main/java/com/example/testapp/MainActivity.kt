package com.example.testapp

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.runtime.getValue
import androidx.compose.runtime.setValue
import com.example.testapp.model.BleTickRepository
import com.example.testapp.ui.components.TestAppApp
import com.example.testapp.ui.theme.TestAppTheme
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

        if (checkSelfPermission(android.Manifest.permission.BLUETOOTH_CONNECT)
            == android.content.pm.PackageManager.PERMISSION_GRANTED
        ) {
            bleManager.initialize()
        }

        enableEdgeToEdge()

        setContent {
            TestAppTheme {
                TestAppApp(
                    repository,
                    logs = bleManager.logFlow,
                    status = bleManager.connectionState,
                )
            }
        }
    }
}
