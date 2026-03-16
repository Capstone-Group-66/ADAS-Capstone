package com.example.testapp.screens

import androidx.annotation.DrawableRes
import androidx.compose.foundation.Image
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.shape.CircleShape
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
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.ColorFilter
import androidx.compose.ui.res.painterResource
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.testapp.R
import com.example.testapp.UpdateUIstate
import com.example.testapp.model.SonarColor
import com.example.testapp.viewmodel.VehicleStatusViewModel

// ── Palette (mirrors Home.kt / theme) ────────────────────────────────────────
private val Charcoal       = Color(0xFF1C1E22)
private val CardBackground = Color(0xFF252830)
private val CharcoalMid    = Color(0xFF23262B)
private val DividerColor   = Color(0xFF3A3F47)
private val AccentCyan     = Color(0xFF00D4FF)
private val AccentCyanDim  = Color(0xFF0099BB)
private val TextPrimary    = Color(0xFFECEFF4)
private val TextSecondary  = Color(0xFF8A9BB0)
private val StatusGreen    = Color(0xFF00E676)
private val StatusRed      = Color(0xFFFF5252)
private val StatusAmber    = Color(0xFFFFB300)

// ── Entry ─────────────────────────────────────────────────────────────────────
@Composable
fun Drive(
    vehicleStatusViewModel: VehicleStatusViewModel,
    onDebugTrigger: () -> Unit,
    onDebugClear: () -> Unit,
) {
    val state by vehicleStatusViewModel.driveState.collectAsState()
    DriveContent(state = state, onDebugTrigger = onDebugTrigger, onDebugClear = onDebugClear)
}

@Composable
fun DriveContent(
    state: UpdateUIstate,
    onDebugTrigger: () -> Unit,
    onDebugClear: () -> Unit,
) {
    val laneActive = laneWarningActive(expiry = state.fcwExpiry, timestamp = state.timestamp)

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Charcoal)
            .drawBehind {
                // Dot-grid texture (matches Home)
                val dotColor = Color(0x18FFFFFF)
                val spacing = 28.dp.toPx()
                val r = 1.2.dp.toPx()
                var x = spacing
                while (x < size.width) {
                    var y = spacing
                    while (y < size.height) {
                        drawCircle(color = dotColor, radius = r, center = Offset(x, y))
                        y += spacing
                    }
                    x += spacing
                }
            }
    ) {
        // ── Car + sensor overlays (all original, untouched) ───────────────────
        CenteredCar()
        FrontDetection(state.sonarValue)
        RearDetection(state.sonarValue)
        WarningOverlay(state)
        BsdLeft(state.sonarValue)
        BsdRight(state.sonarValue)

        Box(modifier = Modifier.fillMaxSize()) {
            LaneDepartureDetection(
                count = 9,
                modifier = Modifier.offset(x = 85.dp, y = 100.dp),
                lane = R.drawable.ic_right_lane,
                detectionLR = laneActive,
            )
            LaneDepartureDetection(
                count = 9,
                modifier = Modifier.offset(x = 270.dp, y = 100.dp),
                lane = R.drawable.ic_right_lane,
                detectionLR = laneActive,
            )
        }

        // ── Speed chip (top-end) ──────────────────────────────────────────────
        SpeedDisplay(
            speedText = "72 km/h",
            modifier = Modifier
                .align(Alignment.TopEnd)
                .padding(top = 20.dp, end = 16.dp),
        )

        // ── Debug controls (top-start) ────────────────────────────────────────
        Row(
            modifier = Modifier
                .align(Alignment.TopStart)
                .padding(start = 16.dp, top = 20.dp),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            DebugButton(label = "TRIGGER", onClick = onDebugTrigger, tint = StatusAmber)
            DebugButton(label = "CLEAR",   onClick = onDebugClear,   tint = AccentCyan)
        }

        // ── Status bar (bottom) ───────────────────────────────────────────────
        StatusRow(
            s1 = state.status1,
            s2 = state.status2,
            s3 = state.status3,
            s4 = state.status4,
            modifier = Modifier
                .align(Alignment.BottomCenter)
                .padding(bottom = 3.dp),
        )
    }
}

// ── Debug button styled to palette ───────────────────────────────────────────
@Composable
private fun DebugButton(label: String, onClick: () -> Unit, tint: Color) {
    Button(
        onClick = onClick,
        colors = ButtonDefaults.buttonColors(
            containerColor = tint.copy(alpha = 0.12f),
            contentColor   = tint,
        ),
        border = androidx.compose.foundation.BorderStroke(1.dp, tint.copy(alpha = 0.4f)),
        contentPadding = PaddingValues(horizontal = 14.dp, vertical = 6.dp),
        shape = RoundedCornerShape(7.dp),
    ) {
        Text(text = label, fontSize = 11.sp, fontWeight = FontWeight.Bold, letterSpacing = 1.sp)
    }
}

// ── Warning overlay (original logic, themed colours) ─────────────────────────
@Composable
fun WarningOverlay(state: UpdateUIstate) {
    val startTime = state.fcwExpiry - 3000
    val elapsed   = state.timestamp - startTime

    if (state.timestamp < state.fcwExpiry && elapsed < 2000) {
        val alpha = when {
            elapsed < 500  -> 1.0f
            elapsed < 2000 -> 1.0f - ((elapsed - 500) / 1500.0f).coerceIn(0f, 1f)
            else           -> 0.0f
        }
        if (alpha > 0) {
            Box(
                modifier = Modifier.fillMaxSize(),
                contentAlignment = Alignment.Center,
            ) {
                Box(
                    modifier = Modifier
                        .shadow(12.dp, RoundedCornerShape(8.dp))
                        .clip(RoundedCornerShape(8.dp))
                        .background(CardBackground.copy(alpha = alpha * 0.95f))
                        .border(
                            2.dp,
                            StatusRed.copy(alpha = alpha),
                            RoundedCornerShape(8.dp)
                        )
                        .padding(horizontal = 44.dp, vertical = 20.dp),
                ) {
                    Text(
                        text       = "BRAKE",
                        color      = StatusRed.copy(alpha = alpha),
                        fontSize   = 52.sp,
                        fontWeight = FontWeight.ExtraBold,
                        letterSpacing = 4.sp,
                    )
                }
            }
        }
    }
}

// ── Status bar ────────────────────────────────────────────────────────────────
@Composable
fun StatusRow(
    s1: Boolean, s2: Boolean, s3: Boolean, s4: Boolean,
    modifier: Modifier = Modifier,
) {
    Row(
        modifier = modifier
            .padding(horizontal = 16.dp, vertical = 10.dp)
            .clip(RoundedCornerShape(10.dp))
            .background(CardBackground)
            .border(1.dp, DividerColor, RoundedCornerShape(10.dp))
            .padding(horizontal = 14.dp, vertical = 8.dp),
        horizontalArrangement = Arrangement.spacedBy(16.dp),
        verticalAlignment     = Alignment.CenterVertically,
    ) {
        StatusItem("Rear Cam",  s1)
        StatusItem("Radar",     s2)
        StatusItem("Blindspot", s3)
        StatusItem("Front Cam", s4)
    }
}

@Composable
fun StatusItem(label: String, isGood: Boolean) {
    Row(
        verticalAlignment     = Alignment.CenterVertically,
        horizontalArrangement = Arrangement.spacedBy(5.dp),
    ) {
        // Dot with subtle glow via drawBehind
        Box(
            modifier = Modifier
                .size(9.dp)
                .drawBehind {
                    val c = if (isGood) StatusGreen else StatusRed
                    drawCircle(color = c.copy(alpha = 0.35f), radius = size.minDimension)
                    drawCircle(color = c, radius = size.minDimension / 2f)
                }
        )
        Text(text = label, fontSize = 11.sp, color = TextSecondary, fontWeight = FontWeight.Medium)
    }
}

// Kept for backwards compat but StatusItem handles badge rendering now
@Composable
fun StatusBadge(isGood: Boolean) {
    Box(
        modifier = Modifier
            .size(10.dp)
            .background(if (isGood) StatusGreen else StatusRed, CircleShape)
    )
}

// ── Speed display ─────────────────────────────────────────────────────────────
@Composable
fun SpeedDisplay(speedText: String, modifier: Modifier = Modifier) {
    Box(
        modifier = modifier
            .clip(RoundedCornerShape(10.dp))
            .background(CardBackground)
            .border(1.dp, DividerColor, RoundedCornerShape(10.dp))
            .padding(horizontal = 14.dp, vertical = 8.dp),
    ) {
        Column(horizontalAlignment = Alignment.CenterHorizontally) {
            Text(
                text       = "SPEED",
                color      = AccentCyan,
                fontSize   = 9.sp,
                fontWeight = FontWeight.Bold,
                letterSpacing = 2.sp,
            )
            Text(
                text       = speedText,
                color      = TextPrimary,
                fontSize   = 28.sp,
                fontWeight = FontWeight.ExtraBold,
            )
        }
    }
}

// ── All original sensor/lane composables — untouched ─────────────────────────
@Composable
fun CenteredCar() {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.image),
            contentDescription = "Car",
            modifier = Modifier.size(320.dp).offset(x = -5.dp, y = 70.dp),
        )
    }
}

@Composable
fun FrontDetection(detectionValue: SonarColor) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "detection",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier = Modifier.offset(y = -130.dp).size(200.dp).rotate(-90f),
        )
    }
}

@Composable
fun RearDetection(detectionValue: SonarColor) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_rear_detection),
            contentDescription = "detection",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier = Modifier.offset(x = -5.dp, y = 270.dp).size(200.dp).rotate(-272f),
        )
    }
}

@Composable
fun BsdLeft(detectionValue: SonarColor) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "Blindspot left",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier = Modifier.size(150.dp).offset(x = -110.dp, y = 160.dp).rotate(135f),
        )
    }
}

@Composable
fun BsdRight(detectionValue: SonarColor) {
    Box(modifier = Modifier.fillMaxSize(), contentAlignment = Alignment.Center) {
        Image(
            painter = painterResource(R.drawable.ic_detection_re),
            contentDescription = "Blindspot right",
            colorFilter = ColorFilter.tint(detectionValue.color),
            modifier = Modifier.size(150.dp).offset(x = 110.dp, y = 160.dp).rotate(45f),
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
    val tintColor = if (detectionLR == 1) StatusRed else Color.Transparent
    Column(modifier = modifier, verticalArrangement = Arrangement.spacedBy(30.dp)) {
        repeat(count) {
            Image(
                painter = painterResource(lane),
                contentDescription = null,
                colorFilter = ColorFilter.tint(tintColor),
                modifier = Modifier.size(50.dp).rotate(90f),
            )
        }
    }
}

fun laneWarningActive(expiry: Long, timestamp: Long): Int {
    val elapsed = timestamp - (expiry - 3000)
    return if (timestamp < expiry && elapsed < 2000) 1 else 0
}
