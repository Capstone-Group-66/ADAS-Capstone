package com.example.testapp.audio

import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import com.example.testapp.model.AlertSoundDecider
import com.example.testapp.model.AlertSoundSnapshot
import com.example.testapp.viewmodel.VehicleStatusViewModel

@Composable
fun AlertSoundObserver(viewModel: VehicleStatusViewModel) {
    val driveState by viewModel.driveState.collectAsState()
    val soundsEnabled by viewModel.alertSoundsEnabled.collectAsState()
    val audioManager = remember { AlertAudioManager() }
    var previousSnapshot by remember { mutableStateOf<AlertSoundSnapshot?>(null) }

    DisposableEffect(Unit) {
        onDispose {
            audioManager.release()
        }
    }

    LaunchedEffect(
        driveState.frontAlertColor,
        driveState.rearAlertColor,
        driveState.leftBlindspotValue,
        driveState.rightBlindspotValue,
        soundsEnabled,
    ) {
        val currentSnapshot = AlertSoundDecider.snapshotOf(driveState)
        if (soundsEnabled) {
            AlertSoundDecider.nextEvent(previousSnapshot, currentSnapshot)?.let { event ->
                audioManager.play(event)
            }
        }
        previousSnapshot = currentSnapshot
    }
}
