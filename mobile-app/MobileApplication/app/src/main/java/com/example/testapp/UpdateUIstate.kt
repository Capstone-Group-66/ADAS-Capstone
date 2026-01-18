package com.example.testapp

import com.example.testapp.model.Direction
import com.example.testapp.model.SonarColor

data class UpdateUIstate(
    val status1: Boolean,
    val status2: Boolean,
    val status3: Boolean,
    val status4: Boolean,
    val sonarValue: SonarColor,
    val alertDirection: Direction,
)
