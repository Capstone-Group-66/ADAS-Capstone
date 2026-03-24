package com.example.testapp.viewmodel

import androidx.lifecycle.ViewModel
import androidx.lifecycle.ViewModelProvider
import com.example.testapp.model.BleTickRepository
import com.example.testapp.model.GpsData
import kotlinx.coroutines.flow.StateFlow

class VehicleStatusViewModelFactory(
    private val repository: BleTickRepository,
    private val gpsData: StateFlow<GpsData?>,
) : ViewModelProvider.Factory {
    override fun <T : ViewModel> create(modelClass: Class<T>): T {
        if (modelClass.isAssignableFrom(VehicleStatusViewModel::class.java)) {
            return VehicleStatusViewModel(repository, gpsData) as T
        }
        throw IllegalArgumentException("Unknown ViewModel class")
    }
}
