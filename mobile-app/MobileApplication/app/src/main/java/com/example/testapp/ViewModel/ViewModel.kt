package com.example.testapp.ViewModel

import androidx.lifecycle.ViewModel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.asStateFlow

class ViewModel : ViewModel() {
    // Each badge has its own state
    private val _status1 = MutableStateFlow(true)
    val status1 = _status1.asStateFlow()

    private val _status2 = MutableStateFlow(false)
    val status2 = _status2.asStateFlow()

    private val _status3 = MutableStateFlow(true)
    val status3 = _status3.asStateFlow()

    private val _status4 = MutableStateFlow(false)
    val status4 = _status4.asStateFlow()

    // Example: methods that update values
    fun setStatus1(value: Boolean) { _status1.value = value }
    fun setStatus2(value: Boolean) { _status2.value = value }
    fun setStatus3(value: Boolean) { _status3.value = value }
    fun setStatus4(value: Boolean) { _status4.value = value }
}
