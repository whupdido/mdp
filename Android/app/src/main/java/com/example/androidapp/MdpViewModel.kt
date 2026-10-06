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
import com.example.androidapp.arena.isRunOverNotice
import com.example.androidapp.arena.runBlocker
import com.example.androidapp.arena.withObstacleAdded
import com.example.androidapp.arena.withStartPose
import com.example.androidapp.arena.startsInCarpark
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
import com.example.androidapp.protocol.PlanState
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
    private var armingTicker: Job? = null
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

            is Inbound.Message -> {
                say(msg.text)
                // The Task 1 route runner says when it has stopped driving.
                // It stops early on a blocked or stalled move, so this can
                // arrive with images still missing -- the attempt is over
                // either way, and the tally already says how it went.
                if (_run.value.task == Task.TASK1 && isRunOverNotice(msg.text)) {
                    finishRun("Run ended")
                }
            }

            is Inbound.Forwarded -> say("Sent ${msg.command} to the robot.")

            // A receipt for our own map edit. Already in the traffic log from
            // the note() above; keeping it out of the status box is the point
            // of C.4.
            is Inbound.MapAck -> Unit

            is Inbound.Rejected -> warn("Robot rejected our message: ${msg.reason}")

            is Inbound.Plan -> onPlan(msg)

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
        // In Task 2 the whole routine is one command, so this DONE is the
        // board saying the run itself is over -- not that a move finished.
        // Without this a clean 2:30 run would keep counting down, go red at
        // 3:00 and tell the operator the time was up.
        "DONE" -> when {
            _run.value.running && _run.value.task == Task.TASK2 ->
                finishRun("Task 2 complete")

            moveCutShort -> {
                moveCutShort = false
                say("Move ended early.")
            }

            else -> say("Move complete.")
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
        invalidatePlan("Obstacle added")
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
        invalidatePlan("Obstacle moved")
    }

    /** C.6: dragged past the boundary. Survivors keep their numbers. */
    fun removeObstacle(id: Int) {
        if (_arena.value.obstacle(id) == null) return
        pushUndo()
        _arena.value = _arena.value.withObstacleRemoved(id)
        transmit(Outbound.sub(id))
        invalidatePlan("Obstacle removed")
    }

    /** C.7: face chosen from the quadrant selector. */
    fun setTargetFace(id: Int, face: Facing) {
        if (_arena.value.obstacle(id) == null) return
        pushUndo()
        _arena.value = _arena.value.withTargetFace(id, face)
        transmit(Outbound.face(id, face))
        invalidatePlan("Face changed")
    }

    /** C.3 */
    fun move(move: Move, distanceCm: Int, angleDeg: Int) {
        if (move == Move.STOP) {
            emergencyStop()
            return
        }
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

    /**
     * Drag the robot to the cell the run will start from. C.5-adjacent: the
     * planner is told this pose, so it has to be editable.
     */
    fun setStartCell(x: Int, y: Int) {
        if (_run.value.running) return
        val next = _arena.value.withStartPose(x, y, _arena.value.robot.facing) ?: return
        pushUndo()
        _arena.value = next
        invalidatePlan("Start moved")
    }

    /** Tap the robot, pick a quadrant: which way it is parked. */
    fun setStartFacing(facing: Facing) {
        if (_run.value.running) return
        val r = _arena.value.robot
        if (r.facing == facing) return
        val next = _arena.value.withStartPose(r.x, r.y, facing) ?: return
        pushUndo()
        _arena.value = next
        invalidatePlan("Start facing changed")
    }

    /**
     * Any edit to the map or the start pose makes an existing route stale, so
     * START goes back to dead and COMPUTE has to be pressed again. Silently
     * running a route planned for a different layout is exactly the kind of
     * mistake the rules give no second chance for.
     */
    private fun invalidatePlan(why: String) {
        val live = _run.value
        if (live.running || live.phase == RunPhase.IDLE) return
        _run.value = live.copy(phase = RunPhase.IDLE, plannedSteps = 0)
        say("$why — plan again before starting.")
    }

    /** Why the next press is refused right now, or null if it would go through. */
    fun startBlocker(): String? = when {
        linkState.value !is LinkState.Connected -> "Not connected to the robot."
        !_arena.value.startsInCarpark() ->
            "Robot is not fully inside the carpark. Leaving it during prep is a disqualification."
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
    private suspend fun publishMap() {
        val arena = _arena.value
        note("-- sending start pose and ${arena.obstacles.size} obstacle(s) --")
        // Start pose first: the planner needs somewhere to route from, and
        // (1,1,N) is only the default, not necessarily where we are parked.
        transmit(Outbound.robotAt(arena.robot.x, arena.robot.y, arena.robot.facing))
        delay(MAP_GAP_MS)
        for (obstacle in arena.obstacles) {
            transmit(Outbound.add(obstacle.id, obstacle.x, obstacle.y))
            delay(MAP_GAP_MS)
            obstacle.targetFace?.let {
                transmit(Outbound.face(obstacle.id, it))
                delay(MAP_GAP_MS)
            }
        }
    }

    /**
     * Task 1, first press: send the map and ask the Pi to plan a route.
     *
     * Off the clock on purpose. Planning takes long enough that doing it
     * inside the six minutes would be giving budget away, and the rules give
     * two minutes of preparation precisely for this kind of setup. START
     * stays dead until the Pi answers.
     */
    fun computeRoute() {
        val live = _run.value
        if (live.task != Task.TASK1 || live.running) return
        _run.value = live.copy(phase = RunPhase.COMPUTING, plannedSteps = 0)
        say("Planning a route…")
        viewModelScope.launch {
            publishMap()
            transmit(Outbound.COMPUTE)
        }
    }

    /**
     * Second press: ask the Pi to pre-flight and hold.
     *
     * In the preparation window, so it costs nothing. The hold length is
     * the Pi's to decide and it announces it; zero is a perfectly good
     * answer and is the default, since nothing in the rules asks us to wait
     * and the clock starts at START.
     */
    fun armRun() {
        val live = _run.value
        if (!live.canPlan) return
        _run.value = live.copy(phase = RunPhase.ARMING)
        say("Checking the robot…")
        viewModelScope.launch { transmit(Outbound.ARM) }
    }

    /** The Pi reporting on the route it was asked for. */
    private fun onPlan(msg: Inbound.Plan) {
        val live = _run.value
        when (msg.state) {
            PlanState.WORKING -> {
                if (!live.running) _run.value = live.copy(phase = RunPhase.COMPUTING)
                say("Robot is planning…")
            }

            PlanState.CHECKING -> {
                if (!live.running) _run.value = live.copy(phase = RunPhase.ARMING)
                say("Checking the robot…")
            }

            PlanState.SET -> {
                if (!live.running) _run.value = _run.value.copy(phase = RunPhase.ARMED, armingSec = 0)
                say("Robot ready. START is live.")
            }

            PlanState.READY -> {
                if (!live.running) {
                    _run.value = live.copy(phase = RunPhase.PLANNED, plannedSteps = msg.value ?: 0)
                }
                val moves = msg.value?.let { " $it moves." } ?: ""
                say("Route ready.$moves START is live.")
            }

            PlanState.FAILED -> {
                if (!live.running) _run.value = live.copy(phase = RunPhase.IDLE, plannedSteps = 0)
                warn("Planning failed: ${msg.detail.ifEmpty { "no reason given" }}")
            }

            // The Pi waits before it drives so the team can step back and put
            // the tablet down. Counting it down here keeps the operator from
            // thinking the robot has hung.
            PlanState.ARMED -> {
                val secs = (msg.value ?: 0).toLong()
                _run.value = _run.value.copy(phase = RunPhase.ARMING, armingSec = secs)
                if (secs > 0) say("Holding ${secs}s — step back.")
                armingTicker?.cancel()
                armingTicker = viewModelScope.launch {
                    var left = secs
                    while (left > 0 && _run.value.running) {
                        delay(1000)
                        left -= 1
                        _run.value = _run.value.copy(armingSec = left.coerceAtLeast(0))
                    }
                }
            }
        }
    }

    /**
     * The one button the team is allowed to touch during an attempt.
     *
     * Starts the clock here rather than waiting for the robot to move: the
     * rules time the attempt from the press, and for Task 1 the planning
     * happens before the first wheel turns.
     */
    fun startRun() {
        val previous = _run.value
        if (previous.running || !previous.canStart) return
        val task = previous.task
        clearRecording()
        _run.value = RunState(
            task = task,
            phase = RunPhase.RUNNING,
            placed = _arena.value.obstacles.size,
            identified = _arena.value.obstacles.count { it.targetId != null },
            plannedSteps = previous.plannedSteps,
        )
        runStartMs = System.currentTimeMillis()
        say("${task.label} started. ${task.budgetSec / 60} minutes.")
        // The clock starts at the press, not when the wheels turn: that is
        // what the supervisor is timing, and the Pi's arming delay is spent
        // out of our budget whether we like it or not.
        viewModelScope.launch { transmit(Outbound.start(task)) }
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
        emergencyStop()
    }

    /**
     * STOP is a transport barrier, not an ordinary movement command.
     * Cancel any pending arming UI work before forwarding it so the app
     * cannot make the stopped run look ready again.
     */
    private fun emergencyStop() {
        armingTicker?.cancel()
        armingTicker = null
        transmit(Outbound.STOP)
        if (!_run.value.running) {
            say("Emergency stop sent. Pending commands cleared.")
            return
        }
        runTicker?.cancel()
        runTicker = null
        _run.value = _run.value.copy(phase = RunPhase.IDLE)
        warn("Run stopped by hand. That scores as incomplete.")
    }

    /** Back to a fresh attempt, without touching the map the supervisor keyed in. */
    fun resetRun() {
        armingTicker?.cancel()
        armingTicker = null
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
        _run.value = live.copy(
            identified = obstacles.count { it.targetId != null },
            placed = obstacles.size,
        )
        if (_arena.value.allIdentified()) {
            finishRun("All ${obstacles.size} images identified")
        }
    }

    /**
     * Stop the clock: the robot is done and the attempt is over.
     *
     * Every way a run can legitimately end comes through here, because the
     * phase decides what the panel says and getting it wrong at the end of an
     * attempt is worse than getting it wrong at the start. The ways are:
     *
     *  - Task 1, every obstacle has an image ID. The rules stop the timing
     *    when the IDs are showing, not when the wheels stop.
     *  - Task 1, the route runner says it has finished. It stops early on a
     *    blocked or stalled move, so this can arrive with images still
     *    missing -- the attempt is still over, and the tally already says
     *    how it went.
     *  - Task 2, the board reports the routine returned.
     */
    private fun finishRun(note: String) {
        val live = _run.value
        if (!live.running) return
        runTicker?.cancel()
        runTicker = null
        armingTicker?.cancel()
        armingTicker = null
        _run.value = live.copy(phase = RunPhase.FINISHED, armingSec = 0)
        say("$note in ${RunState.formatClock(live.elapsedSec)}.")
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
