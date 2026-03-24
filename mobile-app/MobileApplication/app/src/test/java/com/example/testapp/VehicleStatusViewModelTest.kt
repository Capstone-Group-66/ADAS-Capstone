package com.example.testapp

import app.cash.turbine.test
import com.example.testapp.model.BlindSpotStatus
import com.example.testapp.model.ObjectDetection
import com.example.testapp.model.SonarColor
import com.example.testapp.model.SonarColors
import com.example.testapp.model.SystemHealth
import com.example.testapp.model.VehicleAlert
import com.example.testapp.model.VehicleAlertReducer
import com.example.testapp.model.VehicleTelemetry
import com.example.testapp.viewmodel.VehicleStatusViewModel
import junit.framework.TestCase.assertEquals
import junit.framework.TestCase.assertFalse
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.advanceUntilIdle
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class VehicleStatusViewModelTest {
    private val testDispatcher = StandardTestDispatcher()

    @Before
    fun setup() {
        Dispatchers.setMain(testDispatcher)
    }

    @After
    fun tearDown() {
        Dispatchers.resetMain()
    }

    @Test
    fun `driveState emits mapped UpdateUIstate`() =
        runTest {
            val vehicleAlert =
                VehicleAlert(
                    health =
                        SystemHealth(
                            frontCameraOk = true,
                            rearRcwOk = true,
                            radarOk = true,
                        ),
                    sonar =
                        SonarColors(
                            front = SonarColor.RED,
                            rear = SonarColor.GREEN,
                            left = SonarColor.OFF,
                            right = SonarColor.YELLOW,
                        ),
                    telemetry = VehicleTelemetry(speedKmh = 50),
                    detection = ObjectDetection.None,
                    bsd = BlindSpotStatus(leftActive = true, rightActive = true),
                    fcwSeverity = 2,
                )

            val repository = FakeBleTickRepository(vehicleAlert)

            val viewModel =
                VehicleStatusViewModel(
                    repository = repository,
                    scope = backgroundScope,
                )

            viewModel.driveState.test {
                skipItems(1)
                val item = awaitItem()

                assertEquals(true, item.rearRcwOk)
                assertEquals(true, item.radarOk)
                assertEquals(true, item.frontCameraOk)
                assertEquals(50, item.speedKmh)
                assertEquals(SonarColor.RED, item.frontAlertColor)
                assertEquals(SonarColor.GREEN, item.rearAlertColor)
                assertEquals(SonarColor.OFF, item.leftBlindspotValue)
                assertEquals(SonarColor.YELLOW, item.rightBlindspotValue)
                assertEquals(2, item.fcwSeverity)
                assertFalse(item.bleConnected)

                cancelAndIgnoreRemainingEvents()
            }
        }

    @Test
    fun `ble logs and link state come from repository flows`() =
        runTest {
            val logFlow = MutableStateFlow(listOf("connected", "scan started"))
            val connectionFlow = MutableStateFlow(BleConnectionStatus.connected("Connected"))
            val repository =
                FakeBleTickRepository(
                    initial = VehicleAlertReducer.initial(),
                    logs = logFlow,
                    connectionStatus = connectionFlow,
                )

            assertEquals(listOf("connected", "scan started"), repository.bleLogs.value)

            val viewModel =
                VehicleStatusViewModel(
                    repository = repository,
                    scope = backgroundScope,
                )

            advanceUntilIdle()

            viewModel.bleLogs.test {
                assertEquals(listOf("connected", "scan started"), awaitItem())
                cancelAndIgnoreRemainingEvents()
            }

            viewModel.driveState.test {
                val firstState = awaitItem()
                val effectiveState = if (firstState.bleConnected) firstState else awaitItem()
                assertEquals(true, effectiveState.bleConnected)
                cancelAndIgnoreRemainingEvents()
            }
        }
}
