package com.example.testapp.ui.components

import androidx.compose.animation.core.EaseInOutSine
import androidx.compose.animation.core.RepeatMode
import androidx.compose.animation.core.animateFloat
import androidx.compose.animation.core.infiniteRepeatable
import androidx.compose.animation.core.rememberInfiniteTransition
import androidx.compose.animation.core.tween
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.size
import androidx.compose.material3.Icon
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.draw.drawBehind
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Brush
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.unit.dp
import com.example.testapp.ui.theme.AccentCyan
import com.example.testapp.ui.theme.AccentCyanDim
import com.example.testapp.ui.theme.TextSecondary

@Composable
fun TopAccentLine() {
    Box(
        modifier =
            Modifier
                .fillMaxWidth()
                .height(1.dp)
                .drawBehind {
                    drawLine(
                        brush =
                            Brush.horizontalGradient(
                                listOf(
                                    Color.Transparent,
                                    AccentCyan.copy(alpha = 0.5f),
                                    AccentCyanDim.copy(alpha = 0.8f),
                                    AccentCyan.copy(alpha = 0.5f),
                                    Color.Transparent,
                                ),
                            ),
                        start = Offset(0f, 0f),
                        end = Offset(size.width, 0f),
                        strokeWidth = size.height,
                    )
                },
    )
}

@Composable
fun NavItemIcon(
    icon: ImageVector,
    label: String,
    selected: Boolean,
) {
    Box(contentAlignment = Alignment.Center) {
        if (selected) {
            val infiniteTransition = rememberInfiniteTransition(label = "navGlow")

            val glowAlpha by infiniteTransition.animateFloat(
                initialValue = 0.2f,
                targetValue = 0.55f,
                animationSpec =
                    infiniteRepeatable(
                        animation = tween(1000, easing = EaseInOutSine),
                        repeatMode = RepeatMode.Reverse,
                    ),
                label = "glowAlpha",
            )

            Box(
                modifier =
                    Modifier
                        .size(32.dp)
                        .drawBehind {
                            drawCircle(
                                brush =
                                    Brush.radialGradient(
                                        listOf(
                                            AccentCyan.copy(alpha = glowAlpha),
                                            Color.Transparent,
                                        ),
                                    ),
                                radius = size.minDimension / 1.4f,
                            )
                        },
            )
        }

        Icon(
            imageVector = icon,
            contentDescription = label,
            tint = if (selected) AccentCyan else TextSecondary,
            modifier = Modifier.size(22.dp),
        )
    }
}
