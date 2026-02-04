package com.example.testapp.model

import kotlinx.coroutines.channels.awaitClose
import kotlinx.coroutines.flow.Flow
import kotlinx.coroutines.flow.callbackFlow

class GpsTrackingRepository(
    private val source: LocationSource,
) {
    fun gpsDataFlow(): Flow<GpsData> =
        callbackFlow {
            source.start { fix ->
                trySend(
                    GpsData(
                        tsMs = fix.tsMs,
                        lat = fix.lat,
                        lon = fix.lon,
                        altM = fix.altM,
                        speedMps = fix.speedMps,
                        bearingDeg = fix.bearingDeg,
                        accM = fix.accM,
                    ),
                )
            }
            awaitClose { source.stop() }
        }
}
