package com.example.androidapp.link

import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow

/**
 * A [Link] with no radio behind it.
 *
 * This is what lets the arena be built, demoed and tested before Bluetooth
 * works, and it is the reason the map schedule does not depend on the transport
 * schedule. In the app it is selected from the Simulator panel, where messages
 * can be injected by hand exactly as the AMD tool would send them.
 *
 * Nothing here is used in a graded demo — the sign-off runs against
 * [BluetoothLink]. It exists so that the day before a sign-off is spent on the
 * map rather than on waiting for someone else's module.
 */
class FakeLink : Link {

    private val _state = MutableStateFlow<LinkState>(LinkState.Disconnected)
    override val state: StateFlow<LinkState> = _state.asStateFlow()

    private val _incoming = MutableSharedFlow<String>(extraBufferCapacity = 256)
    override val incoming: SharedFlow<String> = _incoming.asSharedFlow()

    private val _discovered = MutableStateFlow<List<RemoteDevice>>(emptyList())
    override val discovered: StateFlow<List<RemoteDevice>> = _discovered.asStateFlow()

    private val _scanning = MutableStateFlow(false)
    override val scanning: StateFlow<Boolean> = _scanning.asStateFlow()

    /** Everything the app has tried to transmit, newest last. */
    private val _sent = MutableSharedFlow<String>(extraBufferCapacity = 256)
    val sent: SharedFlow<String> = _sent.asSharedFlow()

    override fun send(line: String): Boolean {
        if (!_state.value.isConnected) return false
        _sent.tryEmit(line)
        return true
    }

    /** Injects a line as though the robot had sent it. */
    fun receive(line: String) {
        _incoming.tryEmit(line)
    }

    override fun pairedDevices(): List<RemoteDevice> = listOf(SIMULATED)

    override fun startScan() {
        _scanning.value = true
        _discovered.value = listOf(SIMULATED)
        _scanning.value = false
    }

    override fun stopScan() {
        _scanning.value = false
    }

    override fun connect(device: RemoteDevice?) {
        _state.value = LinkState.Connected(device ?: SIMULATED)
    }

    override fun disconnect() {
        _state.value = LinkState.Disconnected
    }

    override fun shutdown() = Unit

    companion object {
        val SIMULATED = RemoteDevice("Simulated robot", "00:00:00:00:00:00")
    }
}
