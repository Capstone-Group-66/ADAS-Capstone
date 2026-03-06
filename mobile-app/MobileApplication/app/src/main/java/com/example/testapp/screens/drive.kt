package com.example.testapp.screens

import androidx.annotation.DrawableRes
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
import com.example.testapp.model.SonarColor
import com.example.testapp.viewmodel.VehicleStatusViewModel

@Composable
fun Drive(
    vehicleStatusViewModel: VehicleStatusViewModel,
    onDebugFcw: () -> Unit,
    onDebugClear: () -> Unit,
) {
    val state by vehicleStatusViewModel.driveState.collectAsState()

    DriveContent(
        state = state,
        onDebugFcw = onDebugFcw,
        onDebugClear = onDebugClear,
    )
}

@Composable
fun DriveContent(
    state: UpdateUIstate,
    onDebugFcw: () -> Unit,
    onDebugClear: () -> Unit,
) {
    // Wrap everything in a Surface that fills the screen
    androidx.compose.material3.Surface(
        modifier = Modifier.fillMaxSize(),
        color = Color(0xFF121212), // Example: Dark Charcoal color
    ) {
        Column(Modifier.padding(16.dp)) {
            Text("drive page")

            StatusRow(
                s1 = state.status1,
                s2 = state.status2,
                s3 = state.status3,
                s4 = state.status4,
            )

            Row(
                horizontalArrangement = Arrangement.spacedBy(8.dp),
                modifier = Modifier.padding(top = 16.dp),
            ) {
                androidx.compose.material3.Button(onClick = onDebugFcw) {
                    Text("TRIGGER FCW")
                }
                androidx.compose.material3.Button(onClick = onDebugClear) {
                    Text("CLEAR")
                }
            }
        }

        CenteredCar()
        FrontDetection(state.sonarValue)
        RearDetection(state.sonarValue)
        FcwWarningOverlay(state)
        BsdLeft(0)
        BsdRight(0)
        Box(modifier = Modifier.fillMaxSize()) {
            LaneDepartureDetection(
                count = 9,
                modifier = Modifier.offset(x = 85.dp, y = 100.dp),
                lane = R.drawable.ic_right_lane,
                0,
            )

            LaneDepartureDetection(
                count = 9,
                modifier = Modifier.offset(x = 270.dp, y = 100.dp),
                lane = R.drawable.ic_right_lane,
                0,
            )
        }
    }
}

@Composable
fun FcwWarningOverlay(state: UpdateUIstate) {
    // Logic: Latch is 3s. Animation is 2s.
    // StartTime is derived from (Expiry - 3000).
    // If receiving continuous alerts, Expiry extends, StartTime advances, elapsed stays ~0 (Solid).
    // If alerts stop, Expiry freezes, elapsed increases -> Fade out.

    val startTime = state.fcwExpiry - 3000
    val elapsed = state.timestamp - startTime

    // Show only if within Latch window AND within Animation window
    if (state.timestamp < state.fcwExpiry && elapsed < 2000) {
        val alpha =
            when {
                elapsed < 500 -> 1.0f
                elapsed < 2000 -> 1.0f - ((elapsed - 500) / 1500.0f).coerceIn(0.0f, 1.0f)
                else -> 0.0f
            }

        if (alpha > 0) {
            Box(
                modifier = Modifier.fillMaxSize(),
                contentAlignment = Alignment.Center,
            ) {
                Text(
                    text = "Collision Warning!",
                    color = Color.Red.copy(alpha = alpha),
                    fontSize = 32.sp,
                    fontWeight = androidx.compose.ui.text.font.FontWeight.Bold,
                    modifier = Modifier.offset(y = (-250).dp),
                )
            }
        }
    }
}

@Composable
fun CenteredCar() {
    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center,
    ) {
        Image(
            painter = painterResource(R.drawable.image),
            contentDescription = "Car",
            modifier =
                Modifier
                    .size(320.dp)
                    .offset(y = 100.dp),
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
                Modifier
                    .offset(y = -130.dp)
                    .size(200.dp)
                    .rotate(-90f),
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
            painter = painterResource(R.drawable.ic_rear_detection),
            contentDescription = "detection",
            // Will be changed to use detectionValue once the detection system is implemented
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier =
                Modifier
                    .offset(x = -5.dp, y = 320.dp)
                    .size(200.dp)
                    .rotate(-272f),
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
fun BsdLeft(detectionLR: Int) {
    val tintColor =
        if (detectionLR == 1) {
            Color.Red
        } else {
            Color(0xFF121212)
        }

    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center,
    ) {
        Image(
            painter = painterResource(R.drawable.ic_bsd_right),
            contentDescription = "Blindspot left",
            colorFilter = ColorFilter.tint(tintColor),
            modifier =
                Modifier
                    .size(300.dp)
                    .offset(x = 215.dp, y = 210.dp),
        )
    }
}

@Composable
fun BsdRight(detectionLR: Int) {
    val tintColor =
        if (detectionLR == 1) {
            Color.Red
        } else {
            Color(0xFF121212)
        }

    Box(
        modifier = Modifier.fillMaxSize(),
        contentAlignment = Alignment.Center,
    ) {
        Image(
            painter = painterResource(R.drawable.ic_bsd_left),
            contentDescription = "Blindspot Right",
            colorFilter = ColorFilter.tint(tintColor),
            modifier =
                Modifier
                    .size(300.dp)
                    .offset(x = -60.dp, y = 210.dp),
        )
    }
}

@Composable
fun LaneDepartureDetection(
    count: Int,
    modifier: Modifier = Modifier,
    @DrawableRes lane: Int,
    detectionLR: Int,
) {
    val tintColor =
        if (detectionLR == 1) {
            Color.Red
        } else {
            Color(0xFF121212)
        }

    Column(
        modifier = modifier,
        verticalArrangement = Arrangement.spacedBy(30.dp),
    ) {
        repeat(count) {
            Image(
                painter = painterResource(lane),
                contentDescription = null,
                colorFilter = ColorFilter.tint(tintColor),
                modifier =
                    Modifier
                        .size(50.dp)
                        .rotate(90f),
            )
        }
    }
}
