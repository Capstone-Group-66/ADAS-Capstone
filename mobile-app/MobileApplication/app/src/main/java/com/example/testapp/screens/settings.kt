package com.example.testapp.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.Groups
import androidx.compose.material.icons.filled.Info
import androidx.compose.material.icons.filled.Settings
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.clip
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontStyle
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.example.testapp.ui.theme.AccentCyan
import com.example.testapp.ui.theme.AccentCyanDim
import com.example.testapp.ui.theme.CardBackground
import com.example.testapp.ui.theme.Charcoal
import com.example.testapp.ui.theme.DividerColor
import com.example.testapp.ui.theme.TextPrimary
import com.example.testapp.ui.theme.TextSecondary
import com.example.testapp.viewmodel.VehicleStatusViewModel

private val StatusGreen = Color(0xFF00E676)
private val StatusRed = Color(0xFFFF5252)

@Composable
fun Settings(viewModel: VehicleStatusViewModel) {
    val logs by viewModel.bleLogs.collectAsState()
    val alertSoundsEnabled by viewModel.alertSoundsEnabled.collectAsState()

    Box(
        modifier = Modifier
            .fillMaxSize()
            .background(Charcoal)
    ) {
        SettingsGridBackground()

        LazyColumn(
            modifier = Modifier
                .fillMaxSize()
                .padding(horizontal = 20.dp, vertical = 24.dp),
            verticalArrangement = Arrangement.spacedBy(20.dp)
        ) {
            item {
                SettingsTitleSection()
            }

            item {
                SettingsSectionCard(
                    icon = Icons.Default.Settings,
                    title = "SYSTEM SETTINGS"
                ) {
                    SettingsSwitchRowStyled(
                        title = "Alert Sounds",
                        description = "Enable or disable in-app ADAS warning sounds.",
                        checked = alertSoundsEnabled,
                        onCheckedChange = viewModel::setAlertSoundsEnabled
                    )

                    HorizontalDivider(
                        color = DividerColor,
                        thickness = 0.6.dp,
                        modifier = Modifier.padding(vertical = 2.dp)
                    )
                }
            }

            item {
                SettingsSectionCard(
                    icon = Icons.Default.Bluetooth,
                    title = "BLE EVENT LOG"
                ) {
                    Box(
                        modifier = Modifier
                            .fillMaxWidth()
                            .heightIn(min = 100.dp, max = 220.dp)
                            .clip(RoundedCornerShape(10.dp))
                            .background(Color(0xFF181A1E))
                            .border(1.dp, DividerColor, RoundedCornerShape(10.dp))
                            .padding(horizontal = 12.dp, vertical = 10.dp)
                    ) {
                        if (logs.isEmpty()) {
                            Text(
                                text = "No BLE logs yet…",
                                color = TextSecondary.copy(alpha = 0.55f),
                                fontSize = 12.sp,
                                fontStyle = FontStyle.Italic,
                                modifier = Modifier.align(Alignment.Center)
                            )
                        } else {
                            LazyColumn(
                                verticalArrangement = Arrangement.spacedBy(6.dp)
                            ) {
                                items(logs) { log ->
                                    Row(verticalAlignment = Alignment.Top) {
                                        Text(
                                            text = "›",
                                            color = AccentCyan.copy(alpha = 0.7f),
                                            fontSize = 12.sp,
                                            modifier = Modifier.padding(end = 6.dp)
                                        )
                                        Text(
                                            text = log,
                                            color = TextPrimary.copy(alpha = 0.9f),
                                            fontSize = 11.sp,
                                            lineHeight = 15.sp,
                                            fontFamily = FontFamily.Monospace
                                        )
                                    }
                                }
                            }
                        }
                    }
                }
            }

            item {
                SettingsSectionCard(
                    icon = Icons.Default.Groups,
                    title = "CREDITS"
                ) {
                    SettingsInfoRowStyled(
                        title = "Group",
                        value = "66"
                    )

                    HorizontalDivider(color = DividerColor, thickness = 0.6.dp)
                    CreditRowStyled("Ajen")
                    HorizontalDivider(color = DividerColor, thickness = 0.6.dp)
                    CreditRowStyled("Damon")
                    HorizontalDivider(color = DividerColor, thickness = 0.6.dp)
                    CreditRowStyled("Jason")
                    HorizontalDivider(color = DividerColor, thickness = 0.6.dp)
                    CreditRowStyled("John")
                    HorizontalDivider(color = DividerColor, thickness = 0.6.dp)
                    CreditRowStyled("Rami")
                    HorizontalDivider(color = DividerColor, thickness = 0.6.dp)
                    CreditRowStyled("Ryan")
                }
            }

            item {
                SettingsSectionCard(
                    icon = Icons.Default.Info,
                    title = "APP INFO"
                ) {
                    Column(
                        modifier = Modifier.fillMaxWidth(),
                        horizontalAlignment = Alignment.CenterHorizontally
                    ) {
                        Text(
                            text = "ADAS Dashboard",
                            color = TextPrimary,
                            fontSize = 15.sp,
                            fontWeight = FontWeight.SemiBold,
                            textAlign = TextAlign.Center
                        )

                        Spacer(modifier = Modifier.height(6.dp))

                        Text(
                            text = "Version 1.0",
                            color = AccentCyan,
                            fontSize = 12.sp,
                            fontWeight = FontWeight.Bold,
                            letterSpacing = 1.5.sp,
                            textAlign = TextAlign.Center
                        )

                        Spacer(modifier = Modifier.height(8.dp))

                        Text(
                            text = "Vehicle monitoring and BLE diagnostics interface.",
                            color = TextSecondary,
                            fontSize = 12.sp,
                            textAlign = TextAlign.Center,
                            lineHeight = 17.sp
                        )
                    }
                }
            }
        }
    }
}

@Composable
private fun SettingsGridBackground() {
    Box(
        modifier = Modifier
            .fillMaxSize()
            .drawBehind {
                val dotColor = Color(0x18FFFFFF)
                val spacing = 28.dp.toPx()
                val radius = 1.2.dp.toPx()

                var x = spacing
                while (x < size.width) {
                    var y = spacing
                    while (y < size.height) {
                        drawCircle(
                            color = dotColor,
                            radius = radius,
                            center = Offset(x, y)
                        )
                        y += spacing
                    }
                    x += spacing
                }

                drawLine(
                    brush = Brush.horizontalGradient(
                        listOf(Color.Transparent, AccentCyan.copy(alpha = 0.25f), Color.Transparent)
                    ),
                    start = Offset(0f, 0f),
                    end = Offset(size.width, 0f),
                    strokeWidth = 2.dp.toPx()
                )
            }
    )
}

@Composable
private fun SettingsTitleSection() {
    Column(
        modifier = Modifier.fillMaxWidth(),
        horizontalAlignment = Alignment.CenterHorizontally
    ) {
        Box(
            modifier = Modifier
                .width(48.dp)
                .height(3.dp)
                .background(
                    brush = Brush.horizontalGradient(
                        listOf(AccentCyanDim, AccentCyan, AccentCyanDim)
                    ),
                    shape = RoundedCornerShape(2.dp)
                )
        )

        Spacer(modifier = Modifier.height(12.dp))

        Text(
            text = "SYSTEM PANEL",
            color = AccentCyan,
            fontSize = 11.sp,
            fontWeight = FontWeight.Bold,
            letterSpacing = 5.sp
        )

        Spacer(modifier = Modifier.height(6.dp))

        Text(
            text = "Settings &\nLogs",
            color = TextPrimary,
            fontSize = 26.sp,
            fontWeight = FontWeight.ExtraBold,
            lineHeight = 32.sp,
            textAlign = TextAlign.Center
        )

        Spacer(modifier = Modifier.height(10.dp))

        Row(
            verticalAlignment = Alignment.CenterVertically,
            horizontalArrangement = Arrangement.Center,
            modifier = Modifier.fillMaxWidth()
        ) {
            HorizontalDivider(
                modifier = Modifier.weight(1f),
                color = DividerColor,
                thickness = 1.dp
            )

            Box(
                modifier = Modifier
                    .padding(horizontal = 10.dp)
                    .size(6.dp)
                    .background(AccentCyan, CircleShape)
            )

            HorizontalDivider(
                modifier = Modifier.weight(1f),
                color = DividerColor,
                thickness = 1.dp
            )
        }
    }
}

@Composable
private fun SettingsSectionCard(
    icon: androidx.compose.ui.graphics.vector.ImageVector,
    title: String,
    content: @Composable ColumnScope.() -> Unit
) {
    Column(
        modifier = Modifier
            .fillMaxWidth()
            .clip(RoundedCornerShape(14.dp))
            .background(CardBackground)
            .border(1.dp, DividerColor, RoundedCornerShape(14.dp))
            .padding(16.dp)
    ) {
        Row(
            verticalAlignment = Alignment.CenterVertically,
            modifier = Modifier.padding(bottom = 14.dp)
        ) {
            Box(
                contentAlignment = Alignment.Center,
                modifier = Modifier
                    .size(30.dp)
                    .background(
                        AccentCyan.copy(alpha = 0.12f),
                        RoundedCornerShape(7.dp)
                    )
            ) {
                Icon(
                    imageVector = icon,
                    contentDescription = null,
                    tint = AccentCyan,
                    modifier = Modifier.size(16.dp)
                )
            }

            Spacer(modifier = Modifier.width(10.dp))

            Text(
                text = title,
                color = TextPrimary,
                fontSize = 11.sp,
                fontWeight = FontWeight.Bold,
                letterSpacing = 2.sp
            )

            Spacer(modifier = Modifier.weight(1f))

            Box(
                modifier = Modifier
                    .width(24.dp)
                    .height(2.dp)
                    .background(
                        brush = Brush.horizontalGradient(
                            listOf(AccentCyan.copy(alpha = 0.6f), Color.Transparent)
                        ),
                        shape = RoundedCornerShape(1.dp)
                    )
            )
        }

        content()
    }
}

@Composable
private fun SettingsSwitchRowStyled(
    title: String,
    description: String,
    checked: Boolean,
    onCheckedChange: (Boolean) -> Unit,
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 12.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Column(modifier = Modifier.weight(1f)) {
            Text(
                text = title,
                color = TextPrimary,
                fontSize = 15.sp,
                fontWeight = FontWeight.SemiBold
            )
            Text(
                text = description,
                color = TextSecondary,
                fontSize = 12.sp
            )
        }

        Switch(
            checked = checked,
            onCheckedChange = onCheckedChange,
            colors = SwitchDefaults.colors(
                checkedThumbColor = Charcoal,
                checkedTrackColor = AccentCyan,
                uncheckedThumbColor = TextSecondary,
                uncheckedTrackColor = DividerColor
            )
        )
    }
}

@Composable
private fun SettingsInfoRowStyled(
    title: String,
    value: String,
    valueColor: Color = TextSecondary
) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 12.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Text(
            text = title,
            color = TextPrimary,
            fontSize = 15.sp,
            fontWeight = FontWeight.SemiBold,
            modifier = Modifier.weight(1f)
        )

        Text(
            text = value,
            color = valueColor,
            fontSize = 13.sp,
            fontWeight = FontWeight.Medium,
            textAlign = TextAlign.End
        )
    }
}

@Composable
private fun CreditRowStyled(name: String) {
    Row(
        modifier = Modifier
            .fillMaxWidth()
            .padding(vertical = 13.dp),
        verticalAlignment = Alignment.CenterVertically
    ) {
        Box(
            modifier = Modifier
                .size(8.dp)
                .background(AccentCyan.copy(alpha = 0.85f), CircleShape)
        )

        Spacer(modifier = Modifier.width(10.dp))

        Text(
            text = name,
            color = TextPrimary,
            fontSize = 14.sp,
            fontWeight = FontWeight.Medium
        )
    }
}
