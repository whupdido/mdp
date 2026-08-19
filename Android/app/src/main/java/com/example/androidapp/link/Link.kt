package com.example.androidapp.link

import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow

/**
 * The seam between the map and the radio.
 *
 * Agreed once with Peter and then frozen. Everything to do with Bluetooth —
 * sockets, discovery, the reader thread, the reconnect loop (C.1, C.2, C.8) —
 * lives behind this interface in [BluetoothLink]. Everything to do with the
 * arena lives in front of it and never imports android.bluetooth.
 *
 * [FakeLink] implements the same interface, which is what lets the map be
 * built, demoed and unit-tested before the radio works.
 */
interface Link {

    val state: StateFlow<LinkState>

    /** One complete message per emission, already stripped of its line ending. */
    val incoming: SharedFlow<String>

    /** Returns false if the link is not currently connected. Never throws. */
    fun send(line: String): Boolean

    /** Devices already paired with this tablet. */
    fun pairedDevices(): List<RemoteDevice>

    /** Starts discovery; results append to [discovered]. Safe to call repeatedly. */
    fun startScan()

    fun stopScan()

    val discovered: StateFlow<List<RemoteDevice>>

    val scanning: StateFlow<Boolean>

    /**
     * Connect, and keep trying until [disconnect] is called. Passing null asks
     * the link to listen for an incoming connection instead.
     */
    fun connect(device: RemoteDevice?)

    /** Explicit user disconnect. Cancels any pending retry. */
    fun disconnect()

    fun shutdown()
}

data class RemoteDevice(val name: String, val address: String) {
    val label: String get() = if (name.isBlank()) address else name
}

sealed class LinkState {
    object Disconnected : LinkState()
    object Listening : LinkState()
    data class Connecting(val target: String) : LinkState()
    data class Connected(val device: RemoteDevice) : LinkState()
    data class Failed(val reason: String) : LinkState()

    val isConnected: Boolean get() = this is Connected

    /** Short text for the status chip. */
    val label: String
        get() = when (this) {
            Disconnected -> "Disconnected"
            Listening -> "Waiting for robot"
            is Connecting -> "Connecting to $target"
            is Connected -> "Connected to ${device.label}"
            is Failed -> reason
        }
}
