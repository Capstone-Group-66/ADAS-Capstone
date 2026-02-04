package com.example.testapp.model

interface LocationSource {
    fun start(onFix: (GpsData) -> Unit)

    fun stop()
}
