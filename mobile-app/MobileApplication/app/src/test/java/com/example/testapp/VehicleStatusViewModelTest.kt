package com.example.testapp

import app.cash.turbine.test
import com.example.testapp.model.BlindSpotStatus
import com.example.testapp.model.CameraHealth
import com.example.testapp.model.ObjectDetection
import com.example.testapp.model.RadarHealth
import com.example.testapp.model.SonarColor
import com.example.testapp.model.SonarColors
import com.example.testapp.model.VehicleAlert
import com.example.testapp.model.VehicleTelemetry
import com.example.testapp.viewmodel.VehicleStatusViewModel
import junit.framework.TestCase.assertEquals
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.TestScope
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class VehicleStatusViewModelTest {
    private val testDispatcher = StandardTestDispatcher()
    private val testScope = TestScope(testDispatcher)

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
            // GIVEN
            val vehicleAlert =
                VehicleAlert(
                    cameras = CameraHealth(frontOk = true, rearOk = true),
                    radar = RadarHealth(ok = true),
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
                )

            val repository = FakeBleTickRepository(vehicleAlert)

            val viewModel =
                VehicleStatusViewModel(
                    repository = repository,
                    scope = backgroundScope,
                )

            // WHEN / THEN
            viewModel.driveState.test {
                skipItems(1)
                val item = awaitItem()
                println("Received UpdateUIstate: $item")

                assertEquals(true, item.status1) // rearOk
                assertEquals(true, item.status2) // radar.ok
                assertEquals(true, item.status3) // bsd.rightActive
                assertEquals(true, item.status4) // frontOk
                assertEquals(SonarColor.RED, item.sonarValue)

                cancelAndIgnoreRemainingEvents()
            }
        }
}
