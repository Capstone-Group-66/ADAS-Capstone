package com.example.testapp

<<<<<<< HEAD
=======
import com.example.testapp.model.Direction
>>>>>>> origin/main
import com.example.testapp.model.SonarColor

data class UpdateUIstate(
    val status1: Boolean,
    val status2: Boolean,
    val status3: Boolean,
    val status4: Boolean,
    val sonarValue: SonarColor,
<<<<<<< HEAD
    val timestamp: Long = 0,
    val fcwExpiry: Long = 0,
=======
    val alertDirection: Direction,
>>>>>>> origin/main
)
