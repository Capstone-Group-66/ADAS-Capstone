package com.example.testapp.model

import android.annotation.SuppressLint
import android.content.Context
import android.os.Looper
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority

class GpsTracker(
    context: Context,
) : LocationSource {
    private val client = LocationServices.getFusedLocationProviderClient(context)

    private val request =
        LocationRequest.Builder(
            Priority.PRIORITY_HIGH_ACCURACY,
            1000L,
        ).setMinUpdateIntervalMillis(500L)
            .setMinUpdateDistanceMeters(1f)
            .setWaitForAccurateLocation(true)
            .build()

    @Volatile
    private var onFix: ((GpsData) -> Unit)? = null

    @Volatile
    private var lastLocation: android.location.Location? = null

    private val callback =
        object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                result.lastLocation?.let { location ->
                    val fix = location.toGpsData(lastLocation)
                    lastLocation = location
                    onFix?.invoke(fix)
                }
            }
        }

    @SuppressLint("MissingPermission")
    override fun start(onFix: (GpsData) -> Unit) {
        this.onFix = onFix
        lastLocation = null
        client.requestLocationUpdates(request, callback, Looper.getMainLooper())
    }

    override fun stop() {
        onFix = null
        lastLocation = null
        client.removeLocationUpdates(callback)
    }
}
