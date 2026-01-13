package com.example.testapp.model

object TickPayloadMapper {
    fun healthFromMask(mask: Int): Pair<CameraHealth, RadarHealth> {
        val frontCamBroken = mask.isBitSet(0)
        val rearCamBroken = mask.isBitSet(1)
        val radarBroken = mask.isBitSet(2)

        val cameras =
            CameraHealth(
                frontOk = !frontCamBroken,
                rearOk = !rearCamBroken,
            )

        val radar = RadarHealth(ok = !radarBroken)

        return cameras to radar
    }

    fun bsdFromMask(mask: Int): BlindSpotStatus =
        BlindSpotStatus(
            leftActive = mask.isBitSet(0),
            rightActive = mask.isBitSet(1),
        )
}
