package com.example.testapp

import app.cash.turbine.test
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
                    speedMps = 2.5f,
                )

            repo.gpsDataFlow().test {
                fake.emit(expected)

                val gps = awaitItem()

                assertEquals(expected.tsMs, gps.tsMs)
                assertEquals(expected.speedMps, gps.speedMps)

                cancelAndIgnoreRemainingEvents()
            }
        }
}
