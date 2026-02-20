package com.example.testapp.model

import android.annotation.SuppressLint
import android.content.Context
import android.location.Location
import com.google.android.gms.location.LocationCallback
import com.google.android.gms.location.LocationRequest
import com.google.android.gms.location.LocationResult
import com.google.android.gms.location.LocationServices
import com.google.android.gms.location.Priority

class GpsTracker(
    context: Context,
    private val onLocation: (Location) -> Unit,
) {
    private val client =
        LocationServices.getFusedLocationProviderClient(context)

    private val request =
        LocationRequest.Builder(
            Priority.PRIORITY_HIGH_ACCURACY,
            1000L,
        ).build()

    private val callback =
        object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                result.lastLocation?.let(onLocation)
            }
        }

    @SuppressLint("MissingPermission")
    fun start() {
        client.requestLocationUpdates(request, callback, null)
    }

    fun stop() {
        client.removeLocationUpdates(callback)
    }
}
