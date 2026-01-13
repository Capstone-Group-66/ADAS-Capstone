package com.example.testapp

import androidx.compose.animation.core.copy
import com.example.testapp.viewmodel.ViewModel

import app.cash.turbine.test
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.VehicleAlert
import com.example.testapp.updateUIstate
import io.mockk.every
import io.mockk.mockk
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.test.StandardTestDispatcher
import kotlinx.coroutines.test.resetMain
import kotlinx.coroutines.test.runTest
import kotlinx.coroutines.test.setMain
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Before
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class ViewModelTest {

    // 1. Create a TestDispatcher to control coroutine execution
    private val testDispatcher = StandardTestDispatcher()

    // 2. Declare mocks for the dependencies
    private lateinit var mockRepository: BleTickRepository

    // 3. Declare the ViewModel instance
    private lateinit var viewModel: ViewModel

    @Before
    fun setUp() {
        // setup before tests
    }

    @After
    fun tearDown() {
        // clean up after tests
    }

    @Test
    fun `driveState correctly maps initial repository state`() = runTest(testDispatcher) {
        // The repository's initial state is a default VehicleAlert()
        // Let's verify our ViewModel's driveState reflects that initial mapping.

        val expectedInitialState = updateUIstate(
            status1 = false, // from vehicleAlert.cameras.rearOk
            status2 = true,  // from vehicleAlert.radar.ok
            status3 = false, // from vehicleAlert.bsd.rightActive
            status4 = false, // from vehicleAlert.cameras.frontOk
            sonarValue = 0   // from vehicleAlert.sonar.front.ordinal
        )

        assertEquals(expectedInitialState, viewModel.driveState.value)
    }

}


