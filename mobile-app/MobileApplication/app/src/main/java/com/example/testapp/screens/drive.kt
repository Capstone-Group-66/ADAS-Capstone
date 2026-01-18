package com.example.testapp.screens

import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.testapp.R
import com.example.testapp.UpdateUIstate
import com.example.testapp.model.Direction
import com.example.testapp.model.SonarColor
import com.example.testapp.viewmodel.VehicleStatusViewModel

@Composable
fun Drive(vehicleStatusViewModel: VehicleStatusViewModel) {
    val state by vehicleStatusViewModel.driveState.collectAsState()

    DriveContent(state = state)
}

@Composable
fun DriveContent(state: UpdateUIstate) {
    Column(Modifier.padding(16.dp)) {
        Text("drive page")

        StatusRow(
            s1 = state.status1,
            s2 = state.status2,
            s3 = state.status3,
            s4 = state.status4,
        )
    }

    CenteredCar()
    MapDetection(state.alertDirection, state.sonarValue)
}

@Composable
fun CenteredCar() {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center,
    ) {
        Image(
            painter = painterResource(R.drawable.ic_car),
            contentDescription = "Car",
            modifier =
                Modifier.size(400.dp).offset(y = 100.dp),
        )
    }
}

@Composable
fun StatusBadge(isGood: Boolean) {
    val backgroundColor = if (isGood) Color(0xFF4CAF50) else Color(0xFFF44336)

    Box(
        modifier =
            Modifier
                .size(16.dp)
                .background(backgroundColor, CircleShape),
    )
}

@Composable
fun FrontDetection(detectionValue: SonarColor) {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center,
    ) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "detection",
            // Will be changed to use detectionValue once the detection system is implemented
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier =
                Modifier.offset(y = -130.dp).size(300.dp).rotate(-90f),
        )
    }
}

@Composable
fun RearDetection(detectionValue: SonarColor) {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center,
    ) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "detection",
            // Will be changed to use detectionValue once the detection system is implemented
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier =
                Modifier.offset(y = -130.dp).size(300.dp).rotate(-90f),
        )
    }
}

@Composable
fun StatusItem(
    label: String,
    isGood: Boolean,
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(3.dp),
    ) {
        StatusBadge(isGood = isGood)
        Text(
            text = label,
            fontSize = 12.sp,
        )
    }
}

@Composable
fun StatusRow(
    s1: Boolean,
    s2: Boolean,
    s3: Boolean,
    s4: Boolean,
) {
    Row(
        modifier = Modifier.padding(16.dp),
        horizontalArrangement = Arrangement.spacedBy(8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        StatusItem("Rear Camera", s1)
        StatusItem("Side Radar", s2)
        StatusItem("Blindspot", s3)
        StatusItem("Front Camera", s4)
    }
}

@Composable
fun MapDetection(direction: Direction, detectionValue: SonarColor) {
    when (direction) {
        Direction.FRONT -> FrontDetection(detectionValue)
        Direction.REAR -> FrontDetection(detectionValue)
        Direction.LEFT -> FrontDetection(detectionValue)
        Direction.RIGHT -> FrontDetection(detectionValue)
        // ^fill the rest when rear/side detections are created
    }

}
