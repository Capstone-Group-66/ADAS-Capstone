package com.example.testapp.audio

import android.media.AudioManager
import android.media.ToneGenerator
import com.example.testapp.model.AlertSoundEvent

class AlertAudioManager {
    private val toneGenerator: ToneGenerator? =
        try {
            ToneGenerator(AudioManager.STREAM_NOTIFICATION, 100)
        } catch (_: RuntimeException) {
            null
        }

    private var lastPlayAtMs: Long = 0

    fun play(event: AlertSoundEvent) {
        val now = System.currentTimeMillis()
        val minGapMs =
            when (event) {
                AlertSoundEvent.FrontAlert -> 700L
                AlertSoundEvent.RearAlert -> 500L
                AlertSoundEvent.BsdBoth -> 400L
                AlertSoundEvent.BsdLeft,
                AlertSoundEvent.BsdRight,
                -> 250L
            }

        if (now - lastPlayAtMs < minGapMs) {
            return
        }

        when (event) {
            AlertSoundEvent.FrontAlert ->
                toneGenerator?.startTone(ToneGenerator.TONE_CDMA_ALERT_CALL_GUARD, 350)

            AlertSoundEvent.RearAlert ->
                toneGenerator?.startTone(ToneGenerator.TONE_CDMA_NETWORK_BUSY_ONE_SHOT, 250)

            AlertSoundEvent.BsdLeft ->
                toneGenerator?.startTone(ToneGenerator.TONE_PROP_BEEP, 140)

            AlertSoundEvent.BsdRight ->
                toneGenerator?.startTone(ToneGenerator.TONE_PROP_BEEP2, 140)

            AlertSoundEvent.BsdBoth ->
                toneGenerator?.startTone(ToneGenerator.TONE_PROP_ACK, 180)
        }

        lastPlayAtMs = now
    }

    fun release() {
        toneGenerator?.release()
    }
}
