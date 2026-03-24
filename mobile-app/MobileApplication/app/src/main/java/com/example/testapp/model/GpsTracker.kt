package com.example.testapp.model

import android.annotation.SuppressLint
import android.content.Context
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
        ).build()

    @Volatile
    private var onFix: ((GpsData) -> Unit)? = null

    private val callback =
        object : LocationCallback() {
            override fun onLocationResult(result: LocationResult) {
                result.lastLocation?.toGpsData()?.let { fix ->
                    onFix?.invoke(fix)
                }
            }
        }

    @SuppressLint("MissingPermission")
    override fun start(onFix: (GpsData) -> Unit) {
        this.onFix = onFix
        client.requestLocationUpdates(request, callback, null)
    }

    override fun stop() {
        onFix = null
        client.removeLocationUpdates(callback)
    }
}
