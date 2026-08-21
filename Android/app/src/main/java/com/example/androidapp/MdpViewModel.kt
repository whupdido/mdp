package com.example.androidapp

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.example.androidapp.arena.Arena
import com.example.androidapp.arena.ArenaState
import com.example.androidapp.arena.Facing
import com.example.androidapp.arena.cleared
import com.example.androidapp.arena.withObstacleAdded
import com.example.androidapp.arena.withObstacleMoved
import com.example.androidapp.arena.withObstacleRemoved
import com.example.androidapp.arena.withRobotAt
import com.example.androidapp.arena.withTargetFace
import com.example.androidapp.arena.withTargetReported
import com.example.androidapp.link.BluetoothLink
import com.example.androidapp.link.FakeLink
import com.example.androidapp.link.Link
import com.example.androidapp.link.LinkState
import com.example.androidapp.link.RemoteDevice
import com.example.androidapp.protocol.Inbound
import com.example.androidapp.protocol.Move
import com.example.androidapp.protocol.Outbound
import com.example.androidapp.protocol.parseInbound
import com.example.androidapp.protocol.toCommand
import kotlinx.coroutines.Job
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MdpViewModel(app: Application) : AndroidViewModel(app) {

    // --- links ----------------------------------------------------------

    private val bluetooth = BluetoothLink(app.applicationContext)
    private val simulator = FakeLink()

    private val _usingSimulator = MutableStateFlow(false)
    val usingSimulator: StateFlow<Boolean> = _usingSimulator.asStateFlow()

    private val link: Link get() = if (_usingSimulator.value) simulator else bluetooth

    private val _linkState = MutableStateFlow<LinkState>(LinkState.Disconnected)
    val linkState: StateFlow<LinkState> = _linkState.asStateFlow()

    private val _discovered = MutableStateFlow<List<RemoteDevice>>(emptyList())
    val discovered: StateFlow<List<RemoteDevice>> = _discovered.asStateFlow()

    private val _scanning = MutableStateFlow(false)
    val scanning: StateFlow<Boolean> = _scanning.asStateFlow()

    // --- arena ----------------------------------------------------------

    private val _arena = MutableStateFlow(ArenaState())
    val arena: StateFlow<ArenaState> = _arena.asStateFlow()

    private val undoStack = ArrayDeque<ArenaState>()

    // --- text output ----------------------------------------------------

    /**
     * C.4. Deliberately *not* the raw stream — the checklist is explicit that
     * this box must show selected information only. Robot telemetry becomes a
     * sentence; unrecognised traffic never reaches here at all.
     */
    private val _status = MutableStateFlow(listOf(stamp("Ready. Not connected.")))
    val status: StateFlow<List<String>> = _status.asStateFlow()

    /** Everything, both directions. Debug drawer only. */
    private val _log = MutableStateFlow<List<String>>(emptyList())
    val log: StateFlow<List<String>> = _log.asStateFlow()

    private val _notices = MutableSharedFlow<String>(extraBufferCapacity = 8)
    val notices: SharedFlow<String> = _notices.asSharedFlow()

    private var collectors: Job? = null

    init {
        attach()
    }

    // -----------------------------------------------------------------
    // Link plumbing
    // -----------------------------------------------------------------

    private fun attach() {
        collectors?.cancel()
        collectors = viewModelScope.launch {
            val active = link
            launch { active.incoming.collect { onIncoming(it) } }
            launch {
                active.state.collect {
                    _linkState.value = it
                    onLinkState(it)
                }
            }
            launch { active.discovered.collect { _discovered.value = it } }
            launch { active.scanning.collect { _scanning.value = it } }
            launch { simulator.sent.collect { note("TX  $it") } }
        }
    }

    fun useSimulator(enabled: Boolean) {
        if (_usingSimulator.value == enabled) return
        link.disconnect()
        _usingSimulator.value = enabled
        _linkState.value = LinkState.Disconnected
        _discovered.value = emptyList()
        attach()
        say(if (enabled) "Simulator mode." else "Bluetooth mode.")
    }

    fun connect(device: RemoteDevice?) = link.connect(device)
    fun disconnect() = link.disconnect()
    fun startScan() = link.startScan()
    fun stopScan() = link.stopScan()
    fun pairedDevices(): List<RemoteDevice> = link.pairedDevices()

    val bluetoothSupported: Boolean get() = bluetooth.isSupported
    val bluetoothEnabled: Boolean get() = bluetooth.isEnabled

    private var lastAnnounced: String? = null

    private fun onLinkState(s: LinkState) {
        val text = when (s) {
            is LinkState.Connected -> "Connected to ${s.device.label}."
            LinkState.Disconnected -> "Disconnected. Retrying."
            LinkState.Listening -> "Waiting for the robot to connect."
            is LinkState.Connecting -> "Connecting to ${s.target}."
            is LinkState.Failed -> s.reason
        }
        if (text != lastAnnounced) {
            lastAnnounced = text
            say(text)
        }
    }

    // -----------------------------------------------------------------
    // Inbound — C.9 and C.10
    // -----------------------------------------------------------------

    private fun onIncoming(raw: String) {
        note("RX  $raw")

        when (val msg = parseInbound(raw)) {

            is Inbound.Robot -> {
                val next = _arena.value.withRobotAt(msg.x, msg.y, msg.facing)
                if (next == null) {
                    warn("Ignored ROBOT (${msg.x},${msg.y}) — outside the arena.")
                } else {
                    _arena.value = next
                }
            }

            is Inbound.Target -> {
                val next = _arena.value.withTargetReported(msg.obstacleId, msg.targetId, msg.face)
                if (next == null) {
                    warn("Ignored TARGET for obstacle ${msg.obstacleId} — unknown obstacle or ID out of range.")
                } else {
                    _arena.value = next
                    val where = msg.face?.let { " on its ${it.name} face" } ?: ""
                    say("Target ${msg.targetId} found at obstacle ${msg.obstacleId}$where.")
                }
            }

            is Inbound.Message -> say(msg.text)

            is Inbound.Forwarded -> say("Sent ${msg.command} to the robot.")

            // A receipt for our own map edit. Already in the traffic log from
            // the note() above; keeping it out of the status box is the point
            // of C.4.
            is Inbound.MapAck -> Unit

            is Inbound.Rejected -> warn("Robot rejected our message: ${msg.reason}")

            is Inbound.StmReply -> onStmReply(msg.reply)

            is Inbound.Unknown -> Unit // logged above, never surfaced, never thrown
        }
    }

    /**
     * Replies relayed by the Pi bridge from the STM board.
     *
     * STALL and TIMEOUT both mean the board gave up mid-move, and its own spec
     * says position is unknown afterwards. That makes the robot drawn on the
     * map a lie until someone re-references it, so those two get a toast rather
     * than a quiet line in the status box.
     */
    private fun onStmReply(reply: String) = when (reply) {
        "READY" -> say("Robot ready.")
        "DONE" -> say("Move complete.")
        "ACK" -> say("Stop acknowledged.")
        "BUSY" -> warn("Robot was still moving — that command was discarded.")
        "STALL" -> warn("Robot stalled. Its position on the map is no longer trustworthy.")
        "TIMEOUT" -> warn("Move timed out. Its position on the map is no longer trustworthy.")
        "ERR" -> warn("Robot did not recognise that command.")
        "NO_REPLY" -> warn("No reply from the robot within 25 s.")
        else -> say("Robot: $reply")
    }

    // -----------------------------------------------------------------
    // Outbound — C.6, C.7 and C.3
    // -----------------------------------------------------------------

    /** C.6: tap an empty cell. */
    fun addObstacle(x: Int, y: Int) {
        val result = _arena.value.withObstacleAdded(x, y) ?: return
        pushUndo()
        _arena.value = result.first
        transmit(Outbound.add(result.second.id, x, y))
    }

    /**
     * C.6: called once, on finger lift. Dragging emits nothing until then —
     * a supervisor watching the AMD tool fill with one line per pixel notices.
     */
    fun commitObstacleMove(id: Int, x: Int, y: Int) {
        val next = _arena.value.withObstacleMoved(id, x, y) ?: return
        pushUndo()
        _arena.value = next
        transmit(Outbound.add(id, x, y))
    }

    /** C.6: dragged past the boundary. Survivors keep their numbers. */
    fun removeObstacle(id: Int) {
        if (_arena.value.obstacle(id) == null) return
        pushUndo()
        _arena.value = _arena.value.withObstacleRemoved(id)
        transmit(Outbound.sub(id))
    }

    /** C.7: face chosen from the quadrant selector. */
    fun setTargetFace(id: Int, face: Facing) {
        if (_arena.value.obstacle(id) == null) return
        pushUndo()
        _arena.value = _arena.value.withTargetFace(id, face)
        transmit(Outbound.face(id, face))
    }

    /** C.3 */
    fun move(move: Move, distanceCm: Int, angleDeg: Int) {
        transmit(move.toCommand(distanceCm, angleDeg))
    }

    private fun transmit(line: String) {
        val ok = link.send(line)
        if (!ok) {
            note("TX  $line  (not sent — no link)")
            warn("Not connected. \"$line\" was not sent.")
        } else if (!_usingSimulator.value) {
            note("TX  $line")
        }
    }

    // -----------------------------------------------------------------
    // Editing helpers
    // -----------------------------------------------------------------

    private fun pushUndo() {
        undoStack.addLast(_arena.value)
        if (undoStack.size > UNDO_DEPTH) undoStack.removeFirst()
    }

    fun undo() {
        val previous = undoStack.removeLastOrNull() ?: run {
            warn("Nothing to undo.")
            return
        }
        _arena.value = previous
        say("Undone.")
    }

    fun resetArena() {
        pushUndo()
        _arena.value = _arena.value.cleared()
        say("Arena cleared.")
    }

    /** Seeds a small layout so C.5 can be demonstrated before anything is connected. */
    fun loadDemoLayout() {
        pushUndo()
        var s = ArenaState()
        listOf(5 to 12, 12 to 15, 15 to 6, 8 to 4).forEach { (x, y) ->
            s = s.withObstacleAdded(x, y)?.first ?: s
        }
        s = s.withTargetFace(1, Facing.S)
        _arena.value = s.withRobotAt(1, 1, Facing.N) ?: s
        say("Demo layout loaded.")
    }

    /** Simulator panel only. */
    fun injectInbound(line: String) {
        if (!_usingSimulator.value) {
            warn("Switch to Simulator mode to inject messages.")
            return
        }
        simulator.receive(line)
    }

    fun clearLog() {
        _log.value = emptyList()
    }

    // -----------------------------------------------------------------

    private fun say(text: String) {
        _status.value = (_status.value + stamp(text)).takeLast(STATUS_DEPTH)
    }

    private fun warn(text: String) {
        say(text)
        _notices.tryEmit(text)
    }

    private fun note(text: String) {
        _log.value = (_log.value + stamp(text)).takeLast(LOG_DEPTH)
    }

    override fun onCleared() {
        bluetooth.shutdown()
        simulator.shutdown()
        super.onCleared()
    }

    companion object {
        private const val STATUS_DEPTH = 60
        private const val LOG_DEPTH = 300
        private const val UNDO_DEPTH = 30
        private val CLOCK = SimpleDateFormat("HH:mm:ss", Locale.UK)
        private fun stamp(text: String) = "${CLOCK.format(Date())}  $text"

        /** Exposed for the arena view's bounds checks. */
        val gridSize = Arena.SIZE
    }
}
