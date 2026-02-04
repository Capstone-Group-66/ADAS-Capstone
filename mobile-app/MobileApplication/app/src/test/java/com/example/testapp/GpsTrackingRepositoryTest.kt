package com.example.testapp

import app.cash.turbine.test
import com.example.testapp.model.FakeLocationSource
import com.example.testapp.model.GpsData
import com.example.testapp.model.GpsTrackingRepository
import kotlinx.coroutines.ExperimentalCoroutinesApi
import kotlinx.coroutines.test.runTest
import org.junit.Assert.assertEquals
import org.junit.Test

@OptIn(ExperimentalCoroutinesApi::class)
class GpsTrackingRepositoryTest {
    @Test
    fun `gps tracking emits gps data`() =
        runTest {
            val fake = FakeLocationSource()
            val repo = GpsTrackingRepository(fake)

            val expected =
                GpsData(
                    tsMs = 123456789L,
                    lat = 45.5017,
                    lon = -73.5673,
                    accM = 5f,
                    speedMps = 2.5f,
                    bearingDeg = 90f,
                )

            repo.gpsDataFlow().test {
                // Turbine ensures the flow is collected before we emit
                fake.emit(expected)

                val gps = awaitItem()

                assertEquals(expected.tsMs, gps.tsMs)
                assertEquals(expected.lat, gps.lat, 1e-6)
                assertEquals(expected.lon, gps.lon, 1e-6)
                assertEquals(expected.accM, gps.accM)
                assertEquals(expected.speedMps, gps.speedMps)
                assertEquals(expected.bearingDeg, gps.bearingDeg)
            }
        }
}
