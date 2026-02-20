package com.example.testapp

import com.example.testapp.model.GpsData
import com.example.testapp.model.LocationSource

class FakeLocationSource : LocationSource {
    private var cb: ((GpsData) -> Unit)? = null

    override fun start(onFix: (GpsData) -> Unit) {
        cb = onFix
    }

    override fun stop() {
        cb = null
    }

    fun emit(fix: GpsData) {
        cb?.invoke(fix)
    }
}
