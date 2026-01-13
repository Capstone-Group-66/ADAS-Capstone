package com.example.testapp.model

internal fun Int.isBitSet(bit: Int): Boolean = (this and (1 shl bit)) != 0
