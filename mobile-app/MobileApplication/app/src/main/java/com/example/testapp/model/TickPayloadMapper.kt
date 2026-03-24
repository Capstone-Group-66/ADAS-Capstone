package com.example.testapp.model

object TickPayloadMapper {
    fun healthFromMask(mask: Int): SystemHealth {
        val frontCameraBroken = mask.isBitSet(0)
        val rearRcwBroken = mask.isBitSet(1)
        val radarBroken = mask.isBitSet(2)

        return SystemHealth(
            frontCameraOk = !frontCameraBroken,
            rearRcwOk = !rearRcwBroken,
            radarOk = !radarBroken,
        )
    }

    fun bsdFromMask(mask: Int): BlindSpotStatus =
        BlindSpotStatus(
            leftActive = mask.isBitSet(0),
            rightActive = mask.isBitSet(1),
        )
}
