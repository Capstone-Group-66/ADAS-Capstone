package com.example.testapp.screens

import androidx.compose.animation.core.*
import androidx.compose.foundation.*
import androidx.compose.foundation.layout.*
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import kotlinx.coroutines.flow.StateFlow

// ── Colour Palette ────────────────────────────────────────────────────────────
private val Charcoal = Color(0xFF1C1E22)
private val CharcoalLight = Color(0xFF2A2D32)
private val CharcoalMid = Color(0xFF23262B)
private val AccentCyan = Color(0xFF00D4FF)
private val AccentCyanDim = Color(0xFF0099BB)
private val AccentAmber = Color(0xFFFFB300)
private val TextPrimary = Color(0xFFECEFF4)
private val TextSecondary = Color(0xFF8A9BB0)
private val DividerColor = Color(0xFF3A3F47)
private val CardBackground = Color(0xFF252830)
private val StatusGreen = Color(0xFF00E676)
private val StatusRed = Color(0xFFFF5252)

// ── Entry Point ───────────────────────────────────────────────────────────────
@Composable
fun Home(
    logs: StateFlow<List<String>>,
    status: StateFlow<String>,
) {
    val logList by logs.collectAsState()
    val connectionStatus by status.collectAsState()

    val isConnected = connectionStatus.contains("connected", ignoreCase = true)

    Box(
        modifier =
            Modifier
                .fillMaxSize()
                .background(Charcoal),
    ) {
        // Subtle grid-line background texture
        GridBackground()

        Column(
            modifier =
                Modifier
                    .fillMaxSize()
                    .verticalScroll(rememberScrollState())
                    .padding(horizontal = 20.dp, vertical = 24.dp),
            verticalArrangement = Arrangement.spacedBy(20.dp),
        ) {
            // ── Title ─────────────────────────────────────────────────────────
            TitleSection()

            // ── How To Use ────────────────────────────────────────────────────
            HowToUseSection()

            // ── Status ────────────────────────────────────────────────────────
            StatusSection(
                connectionStatus = connectionStatus,
                isConnected = isConnected,
                logList = logList,
            )

            Spacer(modifier = Modifier.height(8.dp))
        }
    }
}

// ── Subtle dot-grid background ────────────────────────────────────────────────
@Composable
private fun GridBackground() {
    Box(
        modifier =
            Modifier
                .fillMaxSize()
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
                    // Subtle top cyan glow strip
                    drawLine(
                        brush =
                            Brush.horizontalGradient(
                                listOf(Color.Transparent, AccentCyan.copy(alpha = 0.25f), Color.Transparent),
                            ),
                        start = Offset(0f, 0f),
                        end = Offset(size.width, 0f),
                        strokeWidth = 2.dp.toPx(),
                    )
                },
    )
}

// ── Title Section ─────────────────────────────────────────────────────────────
@Composable
private fun TitleSection() {
    Column(
        modifier = Modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        // Cyan accent line above title
        Box(
            modifier =
                Modifier
                    .width(48.dp)
                    .height(3.dp)
                    .background(
                        brush =
                            Brush.horizontalGradient(
                                listOf(AccentCyanDim, AccentCyan, AccentCyanDim),
                            ),
                        shape = RoundedCornerShape(2.dp),
                    ),
        )
        Spacer(modifier = Modifier.height(12.dp))

        Text(
            text = "MOBILE ADAS",
            color = AccentCyan,
            fontSize = 11.sp,
            fontWeight = FontWeight.Bold,
            letterSpacing = 5.sp,
        )
        Spacer(modifier = Modifier.height(6.dp))
        Text(
            text = "Welcome To the\nMobile ADAS System",
            color = TextPrimary,
            fontSize = 26.sp,
            fontWeight = FontWeight.ExtraBold,
            lineHeight = 32.sp,
            textAlign = TextAlign.Center,
            letterSpacing = 0.3.sp,
        )
        Spacer(modifier = Modifier.height(10.dp))
        // Decorative divider
        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center,
            modifier = Modifier.fillMaxWidth(),
        ) {
            HorizontalDivider(
                modifier = Modifier.weight(1f),
                color = DividerColor,
                thickness = 1.dp,
            )
            Box(
                modifier =
                    Modifier
                        .padding(horizontal = 10.dp)
                        .size(6.dp)
                        .background(AccentCyan, CircleShape),
            )
            HorizontalDivider(
                modifier = Modifier.weight(1f),
                color = DividerColor,
                thickness = 1.dp,
            )
        }
    }
}

// ── How To Use Section ────────────────────────────────────────────────────────
@Composable
private fun HowToUseSection() {
    SectionCard(
        icon = Icons.Default.Info,
        title = "HOW TO USE",
    ) {
        val steps =
            listOf(
                Triple(
                    Icons.Default.BluetoothSearching,
                    "Connect via BLE",
                    "Enable Bluetooth on your device and tap Connect to pair with the ADAS hardware module.",
                ),
                Triple(
                    Icons.Default.Sensors,
                    "Mount Your Device",
                    "Secure your phone on the dashboard mount with a clear forward-facing view for optimal sensor coverage.",
                ),
                Triple(
                    Icons.Default.Speed,
                    "Begin Driving",
                    "Once connected, the system will automatically begin monitoring speed, proximity, and lane data in real time.",
                ),
                Triple(
                    Icons.Default.NotificationsActive,
                    "Monitor Alerts",
                    "Audio and visual alerts will trigger for detected hazards. Keep your eyes on the road at all times.",
                ),
            )

        Column(verticalArrangement = Arrangement.spacedBy(12.dp)) {
            steps.forEachIndexed { index, (icon, heading, desc) ->
                Row(
                    modifier = Modifier.fillMaxWidth(),
                    verticalAlignment = Alignment.Top,
                ) {
                    // Step number badge
                    Box(
                        contentAlignment = Alignment.Center,
                        modifier =
                            Modifier
                                .size(28.dp)
                                .background(AccentCyan.copy(alpha = 0.15f), CircleShape)
                                .border(1.dp, AccentCyan.copy(alpha = 0.4f), CircleShape),
                    ) {
                        Text(
                            text = "${index + 1}",
                            color = AccentCyan,
                            fontSize = 11.sp,
                            fontWeight = FontWeight.Bold,
                        )
                    }
                    Spacer(modifier = Modifier.width(12.dp))
                    Column(modifier = Modifier.weight(1f)) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Icon(
                                imageVector = icon,
                                contentDescription = null,
                                tint = AccentCyan,
                                modifier = Modifier.size(15.dp),
                            )
                            Spacer(modifier = Modifier.width(6.dp))
                            Text(
                                text = heading,
                                color = TextPrimary,
                                fontSize = 13.sp,
                                fontWeight = FontWeight.SemiBold,
                            )
                        }
                        Spacer(modifier = Modifier.height(3.dp))
                        Text(
                            text = desc,
                            color = TextSecondary,
                            fontSize = 12.sp,
                            lineHeight = 17.sp,
                        )
                    }
                }
                if (index < steps.lastIndex) {
                    HorizontalDivider(color = DividerColor, thickness = 0.5.dp)
                }
            }
        }
    }
}

// ── Status Section ────────────────────────────────────────────────────────────
@Composable
private fun StatusSection(
    connectionStatus: String,
    isConnected: Boolean,
    logList: List<String>,
) {
    SectionCard(
        icon = Icons.Default.Wifi,
        title = "SYSTEM STATUS",
    ) {
        // Connection status indicator
        Row(
            modifier =
                Modifier
                    .fillMaxWidth()
                    .clip(RoundedCornerShape(8.dp))
                    .background(
                        if (isConnected) {
                            StatusGreen.copy(alpha = 0.08f)
                        } else {
                            StatusRed.copy(alpha = 0.08f)
                        },
                    )
                    .border(
                        1.dp,
                        if (isConnected) {
                            StatusGreen.copy(alpha = 0.35f)
                        } else {
                            StatusRed.copy(alpha = 0.35f)
                        },
                        RoundedCornerShape(8.dp),
                    )
                    .padding(horizontal = 14.dp, vertical = 12.dp),
            verticalAlignment = Alignment.CenterVertically,
        ) {
            PulsingDot(color = if (isConnected) StatusGreen else StatusRed)
            Spacer(modifier = Modifier.width(10.dp))
            Column {
                Text(
                    text = if (isConnected) "Connected" else "Disconnected",
                    color = if (isConnected) StatusGreen else StatusRed,
                    fontSize = 14.sp,
                    fontWeight = FontWeight.Bold,
                )
                Text(
                    text = connectionStatus,
                    color = TextSecondary,
                    fontSize = 11.sp,
                )
            }
        }

        Spacer(modifier = Modifier.height(14.dp))

        // BLE Log section
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(bottom = 8.dp),
        ) {
            Icon(
                imageVector = Icons.Default.Terminal,
                contentDescription = null,
                tint = AccentCyan,
                modifier = Modifier.size(14.dp),
            )
            Spacer(modifier = Modifier.width(6.dp))
            Text(
                text = "BLE EVENT LOG",
                color = AccentCyan,
                fontSize = 10.sp,
                fontWeight = FontWeight.Bold,
                letterSpacing = 2.sp,
            )
        }

        Box(
            modifier =
                Modifier
                    .fillMaxWidth()
                    .heightIn(min = 80.dp, max = 220.dp)
                    .clip(RoundedCornerShape(8.dp))
                    .background(Color(0xFF181A1E))
                    .border(1.dp, DividerColor, RoundedCornerShape(8.dp))
                    .padding(horizontal = 12.dp, vertical = 10.dp),
        ) {
            if (logList.isEmpty()) {
                Text(
                    text = "No events recorded yet…",
                    color = TextSecondary.copy(alpha = 0.5f),
                    fontSize = 12.sp,
                    fontStyle = androidx.compose.ui.text.font.FontStyle.Italic,
                    modifier = Modifier.align(Alignment.Center),
                )
            } else {
                LazyColumn(
                    verticalArrangement = Arrangement.spacedBy(4.dp),
                ) {
                    items(logList) { log ->
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Text(
                                text = "›",
                                color = AccentCyan.copy(alpha = 0.6f),
                                fontSize = 12.sp,
                                modifier = Modifier.padding(end = 6.dp),
                            )
                            Text(
                                text = log,
                                color = TextPrimary.copy(alpha = 0.85f),
                                fontSize = 11.sp,
                                lineHeight = 15.sp,
                                fontFamily = androidx.compose.ui.text.font.FontFamily.Monospace,
                            )
                        }
                    }
                }
            }
        }
    }
}

// ── Pulsing dot animation ──────────────────────────────────────────────────────
@Composable
private fun PulsingDot(color: Color) {
    val infiniteTransition = rememberInfiniteTransition(label = "pulse")
    val alpha by infiniteTransition.animateFloat(
        initialValue = 0.3f,
        targetValue = 1f,
        animationSpec =
            infiniteRepeatable(
                animation = tween(900, easing = EaseInOutSine),
                repeatMode = RepeatMode.Reverse,
            ),
        label = "alpha",
    )
    Box(
        modifier =
            Modifier
                .size(10.dp)
                .background(color.copy(alpha = alpha), CircleShape),
    )
}

// ── Reusable Section Card ─────────────────────────────────────────────────────
@Composable
private fun SectionCard(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    title: String,
    content: @Composable ColumnScope.() -> Unit,
) {
    Column(
        modifier =
            Modifier
                .fillMaxWidth()
                .clip(RoundedCornerShape(14.dp))
                .background(CardBackground)
                .border(1.dp, DividerColor, RoundedCornerShape(14.dp))
                .padding(16.dp),
    ) {
        // Card header
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(bottom = 14.dp),
        ) {
            Box(
                contentAlignment = Alignment.Center,
                modifier =
                    Modifier
                        .size(30.dp)
                        .background(AccentCyan.copy(alpha = 0.12f), RoundedCornerShape(7.dp)),
            ) {
                Icon(
                    imageVector = icon,
                    contentDescription = null,
                    tint = AccentCyan,
                    modifier = Modifier.size(16.dp),
                )
            }
            Spacer(modifier = Modifier.width(10.dp))
            Text(
                text = title,
                color = TextPrimary,
                fontSize = 11.sp,
                fontWeight = FontWeight.Bold,
                letterSpacing = 2.sp,
            )
            Spacer(modifier = Modifier.weight(1f))
            // Decorative right-side accent bar
            Box(
                modifier =
                    Modifier
                        .width(24.dp)
                        .height(2.dp)
                        .background(
                            brush =
                                Brush.horizontalGradient(
                                    listOf(AccentCyan.copy(alpha = 0.6f), Color.Transparent),
                                ),
                            shape = RoundedCornerShape(1.dp),
                        ),
            )
        }

        content()
    }
}
