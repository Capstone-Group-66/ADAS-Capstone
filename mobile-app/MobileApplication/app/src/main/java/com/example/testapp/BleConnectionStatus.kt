package com.example.testapp

enum class BleLinkState {
    PermissionRequired,
    BleUnavailable,
    Disconnected,
    Scanning,
    Connecting,
    Connected,
    Retrying,
    Error,
}

data class BleConnectionStatus(
    val linkState: BleLinkState,
    val label: String,
) {
    val isConnected: Boolean
        get() = linkState == BleLinkState.Connected

    companion object {
        fun permissionRequired(label: String = "Bluetooth permissions required"): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.PermissionRequired, label)
        }

        fun bleUnavailable(label: String = "Bluetooth unavailable or off"): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.BleUnavailable, label)
        }

        fun disconnected(label: String = "Disconnected"): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.Disconnected, label)
        }

        fun scanning(label: String = "Scanning..."): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.Scanning, label)
        }

        fun connecting(label: String = "Connecting..."): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.Connecting, label)
        }

        fun connected(label: String = "Connected"): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.Connected, label)
        }

        fun retrying(label: String): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.Retrying, label)
        }

        fun error(label: String): BleConnectionStatus {
            return BleConnectionStatus(BleLinkState.Error, label)
        }
    }
}
