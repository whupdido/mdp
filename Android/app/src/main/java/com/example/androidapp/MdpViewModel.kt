package com.example.androidapp

import android.app.Application
import androidx.lifecycle.AndroidViewModel
import androidx.lifecycle.viewModelScope
import com.example.androidapp.arena.Arena
import com.example.androidapp.arena.ArenaState
import com.example.androidapp.arena.Facing
import com.example.androidapp.arena.ReplayState
import com.example.androidapp.arena.RunFrame
import com.example.androidapp.arena.RunPhase
import com.example.androidapp.arena.RunState
import com.example.androidapp.arena.cleared
import com.example.androidapp.arena.Task
import com.example.androidapp.arena.allIdentified
import com.example.androidapp.arena.runBlocker
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
import kotlinx.coroutines.delay
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

    // --- run recording and replay ---------------------------------------

    /**
     * Every robot pose and target report during a run, so it can be scrubbed
     * back afterwards. Recording is always on and costs nothing — you never
     * know a run was worth keeping until after it has happened.
     */
    private val recorded = mutableListOf<RunFrame>()

    private val _replay = MutableStateFlow(ReplayState())
    val replay: StateFlow<ReplayState> = _replay.asStateFlow()

    private var playback: Job? = null
    private var runStartedAt = 0L

    /**
     * Set when the board reports it cut a move short, cleared by the DONE that
     * follows. Without it that DONE would print "Move complete." right under
     * the collision warning, and the two would contradict each other.
     */
    private var moveCutShort = false

    /**
     * Elapsed time since the robot first moved, as mm:ss.
     *
     * Task 1 times out at 6 minutes and the fastest-car run at 3, so during an
     * attempt this is the number anyone actually wants on screen. It starts
     * itself on the first ROBOT message rather than needing a button, because
     * nobody remembers to press start.
     */
    private val _runClock = MutableStateFlow("--:--")
    val runClock: StateFlow<String> = _runClock.asStateFlow()

    /**
     * The Task 1 attempt. Separate from [runClock], which times any movement
     * at all: this one only exists between the START press and the robot
     * stopping, because that is the interval the rules score.
     */
    private val _run = MutableStateFlow(RunState())
    val run: StateFlow<RunState> = _run.asStateFlow()

    private var runTicker: Job? = null
    private var runStartMs = 0L

    private var ticker: Job? = null

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
                    record(next, null)
                }
            }

            is Inbound.Target -> {
                val next = _arena.value.withTargetReported(msg.obstacleId, msg.targetId, msg.face)
                if (next == null) {
                    warn("Ignored TARGET for obstacle ${msg.obstacleId} — unknown obstacle or ID out of range.")
                } else {
                    _arena.value = next
                    val where = msg.face?.let { " on its ${it.name} face" } ?: ""
                    val glyph = Arena.glyphFor(msg.targetId)?.let { " ($it)" } ?: ""
                    say("Target ${msg.targetId}$glyph found at obstacle ${msg.obstacleId}$where.")
                    record(next, "Target ${msg.targetId}$glyph at obstacle ${msg.obstacleId}")
                    refreshRunTally()
                }
            }

            is Inbound.Message -> say(msg.text)

            is Inbound.Forwarded -> say("Sent ${msg.command} to the robot.")

            // A receipt for our own map edit. Already in the traffic log from
            // the note() above; keeping it out of the status box is the point
            // of C.4.
            is Inbound.MapAck -> Unit

            is Inbound.Rejected -> warn("Robot rejected our message: ${msg.reason}")

            is Inbound.StmReply -> onStmReply(msg)

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
     *
     * A collision stop is the same situation wearing a different reply: the
     * board sends `[WARN] COLLISION …` and then DONE, so it is the warning
     * line that carries the truth and the DONE that has to be talked down.
     */
    private fun onStmReply(msg: Inbound.StmReply) = when (msg.reply) {
        "READY" -> say("Robot ready.")
        "DONE" -> if (moveCutShort) {
            moveCutShort = false
            say("Move ended early.")
        } else {
            say("Move complete.")
        }
        "ACK" -> say("Stop acknowledged.")
        "BUSY" -> warn("Robot was still moving — that command was discarded.")
        "STALL" -> warn("Robot stalled. Its position on the map is no longer trustworthy.")
        "TIMEOUT" -> warn("Move timed out. Its position on the map is no longer trustworthy.")
        // The board's own word for a collision stop, once command.c reports
        // how a move ended. The [WARN] line has normally already raised the
        // toast, so this only speaks up if it arrives alone (older firmware
        // that sends the line but not the code, or a dropped line).
        "BLOCKED" -> if (moveCutShort) {
            moveCutShort = false
            say("Move ended early.")
        } else {
            warn("Robot stopped short of an obstacle. Its position on the map is no longer trustworthy.")
        }
        "ERR" -> warn("Robot did not recognise that command.")
        "NO_REPLY" -> warn("No reply from the robot within 25 s.")
        else -> when {
            msg.stoppedShort -> {
                moveCutShort = true
                warn("Robot stopped short of an obstacle. Its position on the map is no longer trustworthy.")
            }
            msg.isWarning -> warn("Robot: ${msg.reply}")
            else -> say("Robot: ${msg.reply}")
        }
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
        moveCutShort = false // a fresh move gets a fresh verdict
        transmit(move.toCommand(distanceCm, angleDeg))
    }

    // -----------------------------------------------------------------
    // Task 1 run
    // -----------------------------------------------------------------

    /** Choose which assessed run the START button drives. */
    fun selectTask(task: Task) {
        if (_run.value.running || _run.value.task == task) return
        _run.value = RunState(task = task)
    }

    /** Why START is refused right now, or null if it would go through. */
    fun startBlocker(): String? = when {
        linkState.value !is LinkState.Connected -> "Not connected to the robot."
        else -> _arena.value.runBlocker(_run.value.task)
    }

    /**
     * Re-send the whole map, spaced out, and then START.
     *
     * Map edits already go out as they are made, which is what C.6 and C.7
     * were signed off on. That is not enough on its own: `rpi/run_task1.py`
     * only collects obstacles once it is running, so anything keyed in before
     * someone launched it on the Pi was simply lost, and the run would plan
     * around a map missing those obstacles without complaining.
     *
     * Re-publishing here closes that hole. It is safe to repeat: `ADD` and
     * `FACE` on the Pi overwrite by obstacle number rather than accumulate
     * (`a1_bridge.handle_map_message`), so sending the map twice leaves the
     * same state as sending it once.
     *
     * [MAP_GAP_MS] between lines is deliberate. The whole map arrives as one
     * burst, and RFCOMM plus the bridge's line-at-a-time reader are happier
     * with a gap than with eight messages inside one buffer.
     */
    private suspend fun publishMapThenStart(task: Task) {
        if (task == Task.TASK1) {
            val obstacles = _arena.value.obstacles
            if (obstacles.isNotEmpty()) {
                note("-- re-sending ${obstacles.size} obstacle(s) before START --")
                for (obstacle in obstacles) {
                    transmit(Outbound.add(obstacle.id, obstacle.x, obstacle.y))
                    delay(MAP_GAP_MS)
                    obstacle.targetFace?.let {
                        transmit(Outbound.face(obstacle.id, it))
                        delay(MAP_GAP_MS)
                    }
                }
            }
        }
        transmit(Outbound.start(task))
    }

    /**
     * The one button the team is allowed to touch during an attempt.
     *
     * Starts the clock here rather than waiting for the robot to move: the
     * rules time the attempt from the press, and for Task 1 the planning
     * happens before the first wheel turns.
     */
    fun startRun() {
        if (_run.value.running) return
        val task = _run.value.task
        clearRecording()
        _run.value = RunState(
            task = task,
            phase = RunPhase.RUNNING,
            placed = _arena.value.obstacles.size,
            identified = _arena.value.obstacles.count { it.targetId != null },
        )
        runStartMs = System.currentTimeMillis()
        say("${task.label} started. ${task.budgetSec / 60} minutes.")
        viewModelScope.launch { publishMapThenStart(task) }
        runTicker?.cancel()
        runTicker = viewModelScope.launch {
            while (true) {
                val elapsed = (System.currentTimeMillis() - runStartMs) / 1000
                val live = _run.value
                if (!live.running) return@launch
                val overrun = elapsed >= live.task.budgetSec
                if (overrun && live.phase != RunPhase.OVERRUN) {
                    warn("Time is up. A run stopped by hand is scored incomplete.")
                }
                _run.value = live.copy(
                    elapsedSec = elapsed,
                    phase = if (overrun) RunPhase.OVERRUN else RunPhase.RUNNING,
                )
                delay(250)
            }
        }
    }

    /**
     * Stop the robot mid-attempt. The rules allow it but score the run as
     * incomplete, so the wording says so rather than pretending otherwise.
     */
    fun abortRun() {
        transmit(Outbound.STOP)
        if (!_run.value.running) return
        runTicker?.cancel()
        runTicker = null
        _run.value = _run.value.copy(phase = RunPhase.IDLE)
        warn("Run stopped by hand. That scores as incomplete.")
    }

    /** Back to a fresh attempt, without touching the map the supervisor keyed in. */
    fun resetRun() {
        runTicker?.cancel()
        runTicker = null
        _run.value = RunState(task = _run.value.task)
    }

    /**
     * Called whenever an image ID lands. The attempt is over once every
     * obstacle carries one, because that is the moment the supervisor's
     * timing stops -- not when the robot happens to stop moving.
     */
    private fun refreshRunTally() {
        val live = _run.value
        if (!live.running || live.task != Task.TASK1) return
        val obstacles = _arena.value.obstacles
        val identified = obstacles.count { it.targetId != null }
        val next = live.copy(identified = identified, placed = obstacles.size)
        _run.value =
            if (_arena.value.allIdentified()) {
                runTicker?.cancel()
                runTicker = null
                val took = RunState.formatClock(next.elapsedSec)
                say("All ${obstacles.size} images identified in $took.")
                next.copy(phase = RunPhase.FINISHED)
            } else {
                next
            }
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
    // Run recording and replay
    // -----------------------------------------------------------------

    private fun record(state: ArenaState, note: String?) {
        if (recorded.isEmpty()) {
            runStartedAt = System.currentTimeMillis()
            startRunClock()
        }
        recorded += RunFrame(
            atMs = System.currentTimeMillis() - runStartedAt,
            robot = state.robot,
            trail = state.trail,
            obstacles = state.obstacles,
            note = note,
        )
        if (recorded.size > MAX_FRAMES) recorded.removeAt(0)
    }

    fun openReplay() {
        if (recorded.size < 2) {
            warn("Nothing recorded yet. Drive the robot, then replay it.")
            return
        }
        _replay.value = ReplayState(
            active = true,
            frames = recorded.toList(),
            index = 0,
        )
        say("Replaying ${recorded.size} frames.")
    }

    fun closeReplay() {
        playback?.cancel()
        playback = null
        _replay.value = ReplayState()
    }

    fun scrubTo(index: Int) {
        val current = _replay.value
        if (!current.active) return
        _replay.value = current.copy(index = index.coerceIn(0, current.frames.lastIndex))
    }

    fun toggleReplayPlayback() {
        val current = _replay.value
        if (!current.active) return
        if (current.playing) {
            playback?.cancel()
            playback = null
            _replay.value = current.copy(playing = false)
            return
        }
        // Restart from the beginning if we are already parked at the end.
        val from = if (current.index >= current.frames.lastIndex) 0 else current.index
        _replay.value = current.copy(playing = true, index = from)
        playback = viewModelScope.launch {
            var i = from
            while (i < _replay.value.frames.lastIndex) {
                delay(FRAME_MS)
                i++
                val live = _replay.value
                if (!live.active || !live.playing) return@launch
                _replay.value = live.copy(index = i)
            }
            _replay.value = _replay.value.copy(playing = false)
            playback = null
        }
    }

    private fun startRunClock() {
        ticker?.cancel()
        ticker = viewModelScope.launch {
            while (true) {
                val elapsed = (System.currentTimeMillis() - runStartedAt) / 1000
                _runClock.value = "%02d:%02d".format(elapsed / 60, elapsed % 60)
                delay(500)
            }
        }
    }

    fun clearRecording() {
        closeReplay()
        recorded.clear()
        ticker?.cancel()
        ticker = null
        _runClock.value = "--:--"
        say("Recording cleared.")
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
        private const val MAX_FRAMES = 600
        private const val FRAME_MS = 320L

        /** Gap between lines when the whole map is republished at START. */
        private const val MAP_GAP_MS = 50L
        private val CLOCK = SimpleDateFormat("HH:mm:ss", Locale.UK)
        private fun stamp(text: String) = "${CLOCK.format(Date())}  $text"

        /** Exposed for the arena view's bounds checks. */
        val gridSize = Arena.SIZE
    }
}
