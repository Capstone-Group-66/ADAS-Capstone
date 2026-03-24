package com.example.testapp.screens

import android.view.MotionEvent
import androidx.compose.animation.core.animateDpAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.BoxWithConstraints
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.offset
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.draw.rotate
import androidx.compose.ui.draw.shadow
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.input.pointer.pointerInteropFilter
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.testapp.R
import com.example.testapp.UpdateUIstate
import com.example.testapp.model.SonarColor
import com.example.testapp.viewmodel.VehicleStatusViewModel

private val Charcoal = Color(0xFF1C1E22)
private val CardBackground = Color(0xFF252830)
private val DividerColor = Color(0xFF3A3F47)
private val AccentCyan = Color(0xFF00D4FF)
private val TextPrimary = Color(0xFFECEFF4)
private val TextSecondary = Color(0xFF8A9BB0)
private val StatusGreen = Color(0xFF00E676)
private val StatusRed = Color(0xFFFF5252)
private val StatusAmber = Color(0xFFFFB300)

@Composable
fun Drive(
    vehicleStatusViewModel: VehicleStatusViewModel,
    developerModeEnabled: Boolean,
    showBottomBar: Boolean,
    onUserInteraction: () -> Unit,
    onDebugTrigger: () -> Unit,
    onDebugClear: () -> Unit,
) {
    val state by vehicleStatusViewModel.driveState.collectAsState()
    DriveContent(
        state = state,
        developerModeEnabled = developerModeEnabled,
        showBottomBar = showBottomBar,
        onUserInteraction = onUserInteraction,
        onDebugTrigger = onDebugTrigger,
        onDebugClear = onDebugClear,
    )
}

@Composable
fun DriveContent(
    state: UpdateUIstate,
    developerModeEnabled: Boolean,
    showBottomBar: Boolean,
    onUserInteraction: () -> Unit,
    onDebugTrigger: () -> Unit,
    onDebugClear: () -> Unit,
) {
    BoxWithConstraints(
        modifier =
            Modifier
                .fillMaxSize()
                .pointerInteropFilter {
                    if (it.actionMasked == MotionEvent.ACTION_DOWN) {
                        onUserInteraction()
                    }
                    false
                }
                .background(Charcoal)
                .drawBehind {
                    val dotColor = Color(0x18FFFFFF)
                    val spacing = 28.dp.toPx()
                    val radius = 1.2.dp.toPx()
                    var x = spacing
                    while (x < size.width) {
                        var y = spacing
                        while (y < size.height) {
                            drawCircle(color = dotColor, radius = radius, center = Offset(x, y))
                            y += spacing
                        }
                        x += spacing
                    }
                },
    ) {
        val sceneVerticalShift = -(maxHeight * 0.13f)
        val statusRowBottomPadding by animateDpAsState(
            targetValue = if (showBottomBar) 88.dp else 12.dp,
            animationSpec = tween(durationMillis = 220),
            label = "statusRowBottomPadding",
        )

        CenteredCar(sceneVerticalShift = sceneVerticalShift)
        FrontDetection(
            detectionValue = state.frontAlertColor,
            sceneVerticalShift = sceneVerticalShift,
        )
        RearDetection(
            detectionValue = state.rearAlertColor,
            sceneVerticalShift = sceneVerticalShift,
        )
        WarningOverlay(state)
        BsdLeft(
            detectionValue = state.leftBlindspotValue,
            sceneVerticalShift = sceneVerticalShift,
        )
        BsdRight(
            detectionValue = state.rightBlindspotValue,
            sceneVerticalShift = sceneVerticalShift,
        )

        SpeedDisplay(
            speedText = "${state.speedKmh} km/h",
            modifier =
                Modifier
                    .align(Alignment.TopEnd)
                    .padding(top = 20.dp, end = 16.dp),
        )

        if (developerModeEnabled) {
            Row(
                modifier =
                    Modifier
                        .align(Alignment.TopStart)
                        .padding(start = 16.dp, top = 20.dp),
                horizontalArrangement = Arrangement.spacedBy(8.dp),
            ) {
                DebugButton(label = "TRIGGER", onClick = onDebugTrigger, tint = StatusAmber)
                DebugButton(label = "CLEAR", onClick = onDebugClear, tint = AccentCyan)
            }
        }

        StatusRow(
            rearRcwOk = state.rearRcwOk,
            radarOk = state.radarOk,
            bleConnected = state.bleConnected,
            frontCameraOk = state.frontCameraOk,
            modifier =
                Modifier
                    .align(Alignment.BottomCenter)
                    .padding(bottom = statusRowBottomPadding),
        )
    }
}

@Composable
private fun DebugButton(
    label: String,
    onClick: () -> Unit,
    tint: Color,
) {
    Button(
        onClick = onClick,
        colors =
            ButtonDefaults.buttonColors(
                containerColor = tint.copy(alpha = 0.12f),
                contentColor = tint,
            ),
        border = BorderStroke(1.dp, tint.copy(alpha = 0.4f)),
        contentPadding = PaddingValues(horizontal = 14.dp, vertical = 6.dp),
        shape = RoundedCornerShape(7.dp),
    ) {
        Text(text = label, fontSize = 11.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
    }
}

@Composable
fun WarningOverlay(state: UpdateUIstate) {
    val startTime = state.fcwExpiry - 3000
    val elapsed = state.timestamp - startTime

    if (state.timestamp < state.fcwExpiry && elapsed < 2000) {
        val alpha =
            when {
                elapsed < 500 -> 1.0f
                elapsed < 2000 -> 1.0f - ((elapsed - 500) / 1500.0f).coerceIn(0f, 1f)
                else -> 0.0f
            }
        if (alpha > 0) {
            Box(
                modifier = Modifier.fillMaxSize(),
                contentAlignment = Alignment.Center,
            ) {
                Box(
                    modifier =
                        Modifier
                            .shadow(12.dp, RoundedCornerShape(8.dp))
                            .clip(RoundedCornerShape(8.dp))
                            .background(CardBackground.copy(alpha = alpha * 0.95f))
                            .border(
                                2.dp,
                                StatusRed.copy(alpha = alpha),
                                RoundedCornerShape(8.dp),
                            )
                            .padding(horizontal = 44.dp, vertical = 20.dp),
                ) {
                    Text(
                        text = "BRAKE",
                        color = StatusRed.copy(alpha = alpha),
                        fontSize = 52.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 4.sp,
                    )
                }
            }
        }
    }
}

@Composable
fun StatusRow(
    rearRcwOk: Boolean,
    radarOk: Boolean,
    bleConnected: Boolean,
    frontCameraOk: Boolean,
    modifier: Modifier = Modifier,
) {
    Row(
        modifier =
            modifier
                .padding(horizontal = 16.dp, vertical = 10.dp)
                .clip(RoundedCornerShape(10.dp))
                .background(CardBackground)
                .border(1.dp, DividerColor, RoundedCornerShape(10.dp))
                .padding(horizontal = 14.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.spacedBy(16.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        StatusItem("Rear RCW", rearRcwOk)
        StatusItem("Radar", radarOk)
        StatusItem("BLE Link", bleConnected)
        StatusItem("Front Cam", frontCameraOk)
    }
}

@Composable
fun StatusItem(
    label: String,
    isGood: Boolean,
) {
    Row(
        verticalAlignment = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(5.dp),
    ) {
        Box(
            modifier =
                Modifier
                    .size(9.dp)
                    .drawBehind {
                        val color = if (isGood) StatusGreen else StatusRed
                        drawCircle(color = color.copy(alpha = 0.35f), radius = size.minDimension)
                        drawCircle(color = color, radius = size.minDimension / 2f)
                    },
        )
        Text(text = label, fontSize = 11.sp, color = TextSecondary, fontWeight = FontWeight.Medium)
    }
}

@Composable
fun SpeedDisplay(
    speedText: String,
    modifier: Modifier = Modifier,
) {
    Box(
        modifier =
            modifier
                .clip(RoundedCornerShape(10.dp))
                .background(CardBackground)
                .border(1.dp, DividerColor, RoundedCornerShape(10.dp))
                .padding(horizontal = 14.dp, vertical = 8.dp),
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text = "SPEED",
                color = AccentCyan,
                fontSize = 9.sp,
                fontWeight = FontWeight.Bold,
                letterSpacing = 2.sp,
            )
            Text(
                text = speedText,
                color = TextPrimary,
                fontSize = 28.sp,
                fontWeight = FontWeight.ExtraBold,
            )
        }
    }
}

@Composable
fun CenteredCar(sceneVerticalShift: androidx.compose.ui.unit.Dp) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.image),
            contentDescription = "Car",
            modifier = Modifier.size(320.dp).offset(x = -5.dp, y = 70.dp + sceneVerticalShift),
        )
    }
}

@Composable
fun FrontDetection(
    detectionValue: SonarColor,
    sceneVerticalShift: androidx.compose.ui.unit.Dp,
) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "Front detection",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier = Modifier.offset(y = -130.dp + sceneVerticalShift).size(200.dp).rotate(-90f),
        )
    }
}

@Composable
fun RearDetection(
    detectionValue: SonarColor,
    sceneVerticalShift: androidx.compose.ui.unit.Dp,
) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_rear_detection),
            contentDescription = "Rear detection",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier =
                Modifier
                    .offset(x = -5.dp, y = 270.dp + sceneVerticalShift)
                    .size(200.dp)
                    .rotate(-272f),
        )
    }
}

@Composable
fun BsdLeft(
    detectionValue: SonarColor,
    sceneVerticalShift: androidx.compose.ui.unit.Dp,
) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "Blindspot left",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier =
                Modifier
                    .size(150.dp)
                    .offset(x = -110.dp, y = 160.dp + sceneVerticalShift)
                    .rotate(135f),
        )
    }
}

@Composable
fun BsdRight(
    detectionValue: SonarColor,
    sceneVerticalShift: androidx.compose.ui.unit.Dp,
) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "Blindspot right",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier =
                Modifier
                    .size(150.dp)
                    .offset(x = 110.dp, y = 160.dp + sceneVerticalShift)
                    .rotate(45f),
        )
    }
}
