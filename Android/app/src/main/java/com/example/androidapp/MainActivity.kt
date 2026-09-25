package com.example.androidapp

import android.graphics.Color
import android.graphics.drawable.ColorDrawable
import android.os.Bundle
import android.view.KeyEvent
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.EditorInfo
import android.widget.BaseAdapter
import android.widget.SeekBar
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.activity.viewModels
import androidx.appcompat.app.AlertDialog
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import com.example.androidapp.arena.Arena
import com.example.androidapp.arena.RunPhase
import com.example.androidapp.arena.Task
import com.example.androidapp.arena.RunState
import com.example.androidapp.databinding.ActivityMainBinding
import com.example.androidapp.databinding.DialogDevicesBinding
import com.example.androidapp.databinding.ItemDeviceBinding
import com.example.androidapp.link.BluetoothLink
import com.example.androidapp.link.LinkState
import com.example.androidapp.link.RemoteDevice
import com.example.androidapp.protocol.Move
import com.google.android.material.button.MaterialButton
import kotlinx.coroutines.launch

/**
 * Single screen. The activity does wiring and nothing else: every decision
 * lives in [MdpViewModel], and everything about drawing lives in ArenaView.
 */
class MainActivity : AppCompatActivity() {

    private lateinit var ui: ActivityMainBinding
    private val vm: MdpViewModel by viewModels()

    private var distanceCm = 10
    private var angleDeg = 90

    private val askPermissions =
        registerForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { grants ->
            if (grants.values.any { !it }) {
                toast(getString(R.string.permission_needed))
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        ui = ActivityMainBinding.inflate(layoutInflater)
        setContentView(ui.root)

        requestBluetoothPermissions()
        wireArena()
        wireLinkControls()
        wireDrivePad()
        wireMapActions()
        wireRunControls()
        wireTrafficDrawer()
        wireReplay()
        observe()
    }

    private fun requestBluetoothPermissions() {
        val missing = BluetoothLink.requiredPermissions().filter {
            ContextCompat.checkSelfPermission(this, it) != android.content.pm.PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) askPermissions.launch(missing.toTypedArray())
    }

    // -----------------------------------------------------------------
    // Arena — C.5, C.6, C.7
    // -----------------------------------------------------------------

    private fun wireArena() = with(ui.arena) {
        onAddObstacle = { x, y -> vm.addObstacle(x, y) }
        onMoveObstacle = { id, x, y -> vm.commitObstacleMove(id, x, y) }
        onRemoveObstacle = { id -> vm.removeObstacle(id) }
        onSetTargetFace = { id, face -> vm.setTargetFace(id, face) }
        onHover = { x, y ->
            ui.hoverChip.visibility = View.VISIBLE
            ui.hoverChip.text = getString(
                R.string.hover_format, x, y, x * Arena.CELL_CM, y * Arena.CELL_CM
            )
        }
        onHoverEnd = { ui.hoverChip.visibility = View.INVISIBLE }
    }

    // -----------------------------------------------------------------
    // Link — C.1, C.2, C.8
    // -----------------------------------------------------------------

    private fun wireLinkControls() {
        ui.btnConnect.setOnClickListener { showDevicePicker() }
        ui.btnDisconnect.setOnClickListener { vm.disconnect() }
        ui.switchSimulator.setOnCheckedChangeListener { _, checked ->
            vm.useSimulator(checked)
            if (checked && ui.trafficPanel.visibility != View.VISIBLE) toggleTraffic()
        }
    }

    private fun showDevicePicker() {
        if (!vm.usingSimulator.value) {
            if (!vm.bluetoothSupported) { toast("This device has no Bluetooth adapter."); return }
            if (!vm.bluetoothEnabled) { toast(getString(R.string.bluetooth_off)); return }
            requestBluetoothPermissions()
        }

        val content = DialogDevicesBinding.inflate(layoutInflater)
        val rows = mutableListOf<Row>()
        val adapter = DeviceAdapter(rows)
        content.deviceList.adapter = adapter

        fun refresh() {
            val paired = vm.pairedDevices().map { Row(it, paired = true) }
            val seen = paired.mapTo(HashSet()) { it.device.address }
            val nearby = vm.discovered.value.filterNot { it.address in seen }.map { Row(it, paired = false) }
            rows.clear()
            rows += paired
            rows += nearby
            adapter.notifyDataSetChanged()
            content.emptyHint.visibility = if (rows.isEmpty()) View.VISIBLE else View.GONE
        }
        refresh()

        val dialog = AlertDialog.Builder(this)
            .setView(content.root)
            .create()

        // The dialog window's own background is a light, square-cornered
        // rectangle that shows behind the custom rounded surface. Clearing it
        // lets the layout be the whole dialog.
        dialog.window?.setBackgroundDrawable(ColorDrawable(Color.TRANSPARENT))
        content.btnCloseDialog.setOnClickListener { vm.stopScan(); dialog.dismiss() }

        content.btnScanDialog.setOnClickListener {
            if (vm.scanning.value) vm.stopScan() else vm.startScan()
        }

        content.deviceList.setOnItemClickListener { _, _, position, _ ->
            val chosen = rows.getOrNull(position) ?: return@setOnItemClickListener
            vm.stopScan()
            vm.connect(chosen.device)
            dialog.dismiss()
        }

        // Keep the dialog live while discovery runs.
        val job = lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch { vm.discovered.collect { refresh() } }
                launch {
                    vm.scanning.collect { on ->
                        content.scanSpinner.visibility = if (on) View.VISIBLE else View.INVISIBLE
                        content.btnScanDialog.text =
                            getString(if (on) R.string.stop_scan else R.string.scan)
                    }
                }
            }
        }
        dialog.setOnDismissListener { job.cancel(); vm.stopScan() }
        dialog.show()
    }

    private data class Row(val device: RemoteDevice, val paired: Boolean)

    private inner class DeviceAdapter(private val rows: List<Row>) : BaseAdapter() {
        override fun getCount() = rows.size
        override fun getItem(position: Int) = rows[position]
        override fun getItemId(position: Int) = position.toLong()
        override fun getView(position: Int, convertView: View?, parent: ViewGroup?): View {
            val binding = convertView?.tag as? ItemDeviceBinding
                ?: ItemDeviceBinding.inflate(layoutInflater, parent, false).also { it.root.tag = it }
            val row = rows[position]
            binding.deviceName.text = row.device.label
            binding.deviceMeta.text = getString(
                if (row.paired) R.string.device_meta_paired else R.string.device_meta_nearby,
                row.device.address,
            )
            return binding.root
        }
    }

    // -----------------------------------------------------------------
    // Drive pad — C.3
    // -----------------------------------------------------------------

    private fun wireDrivePad() {
        // One command at a time: the STM discards anything sent while a move is
        // running and replies BUSY, so these are single-shot, not auto-repeat.
        ui.btnForward.setOnClickListener { drive(Move.FORWARD) }
        ui.btnBackward.setOnClickListener { drive(Move.BACKWARD) }
        ui.btnFwdLeft.setOnClickListener { drive(Move.FWD_LEFT) }
        ui.btnFwdRight.setOnClickListener { drive(Move.FWD_RIGHT) }
        ui.btnBackLeft.setOnClickListener { drive(Move.BACK_LEFT) }
        ui.btnBackRight.setOnClickListener { drive(Move.BACK_RIGHT) }
        ui.btnStop.setOnClickListener { drive(Move.STOP) }

        // C.3 gesture control. Buttons stay the default because that is what
        // the checklist was signed on; the pad is the faster one in practice.
        ui.gesturePad.onMove = { drive(it) }
        ui.btnDriveMode.setOnClickListener {
            val toPad = ui.gesturePad.visibility != View.VISIBLE
            ui.gesturePad.visibility = if (toPad) View.VISIBLE else View.GONE
            ui.buttonGrid.visibility = if (toPad) View.GONE else View.VISIBLE
            ui.btnDriveMode.setText(if (toPad) R.string.mode_buttons else R.string.mode_pad)
        }

        ui.btnDistDown.setOnClickListener { distanceCm = (distanceCm - 10).coerceAtLeast(10); showStep() }
        ui.btnDistUp.setOnClickListener { distanceCm = (distanceCm + 10).coerceAtMost(150); showStep() }
        ui.stepReadout.setOnClickListener {
            angleDeg = when (angleDeg) { 45 -> 90; 90 -> 180; 180 -> 360; else -> 45 }
            showStep()
        }
        showStep()
    }

    private fun drive(move: Move) = vm.move(move, distanceCm, angleDeg)

    private fun showStep() {
        ui.stepReadout.text = getString(R.string.step_format, distanceCm, angleDeg)
    }

    // -----------------------------------------------------------------
    // Map actions and the traffic drawer
    // -----------------------------------------------------------------

    private fun wireMapActions() {
        ui.btnUndo.setOnClickListener { vm.undo() }
        ui.btnClear.setOnClickListener { vm.resetArena() }
        ui.btnDemo.setOnClickListener { vm.loadDemoLayout() }
        ui.btnReplay.setOnClickListener { vm.openReplay() }
    }

    // -----------------------------------------------------------------
    // Task 1 run
    // -----------------------------------------------------------------

    /**
     * One button, three jobs, depending on where the attempt is.
     *
     * START goes through on a single press when the map is complete, because
     * the supervisor has just said go and the clock is running. It only asks
     * for confirmation when something is actually wrong -- an obstacle with no
     * image face is one the planner will silently skip.
     */
    private fun wireRunControls() {
        ui.btnTask1.setOnClickListener { vm.selectTask(Task.TASK1) }
        ui.btnTask2.setOnClickListener { vm.selectTask(Task.TASK2) }
        ui.btnStart.setOnClickListener {
            when (vm.run.value.phase) {
                RunPhase.RUNNING, RunPhase.OVERRUN -> confirmStopRun()
                RunPhase.FINISHED -> vm.resetRun()
                RunPhase.IDLE -> {
                    val blocker = vm.startBlocker()
                    if (blocker == null) vm.startRun() else confirmStartAnyway(blocker)
                }
            }
        }
    }

    private fun confirmStartAnyway(blocker: String) {
        AlertDialog.Builder(this)
            .setTitle(R.string.run_check_title)
            .setMessage(blocker)
            .setPositiveButton(R.string.run_start_anyway) { _, _ -> vm.startRun() }
            .setNegativeButton(R.string.cancel, null)
            .show()
    }

    private fun confirmStopRun() {
        AlertDialog.Builder(this)
            .setTitle(R.string.run_stop_title)
            .setMessage(R.string.run_stop_body)
            .setPositiveButton(R.string.run_stop) { _, _ -> vm.abortRun() }
            .setNegativeButton(R.string.cancel, null)
            .show()
    }

    /** Paint the run panel for the task and phase it is in. */
    private fun renderRun(state: RunState) {
        renderTaskPicker(state)

        ui.runCountdown.text = state.clock
        // Idle, the big number IS the budget, so repeating it underneath says
        // nothing. Once it is counting down, the caption is what tells you
        // what it is counting down from.
        ui.runBudget.visibility = if (state.phase == RunPhase.IDLE) View.INVISIBLE else View.VISIBLE
        ui.runBudget.text =
            getString(R.string.run_budget, RunState.formatClock(state.task.budgetSec))

        val urgent = state.running && state.remainingSec <= RunState.WARN_SEC
        ui.runCountdown.setTextColor(
            color(
                when {
                    state.phase == RunPhase.OVERRUN -> R.color.arena_bad
                    urgent -> R.color.arena_accent
                    state.running -> R.color.arena_text
                    state.phase == RunPhase.FINISHED -> R.color.arena_go_lit
                    else -> R.color.arena_text_muted
                }
            )
        )

        // Task 2 is scored on time alone, so an image tally there is noise.
        ui.tallyRow.visibility = if (state.showsTally) View.VISIBLE else View.GONE
        ui.runTally.text = if (state.placed == 0) getString(R.string.run_tally_empty) else state.tally

        ui.btnStart.setText(
            when (state.phase) {
                RunPhase.IDLE -> R.string.run_start
                RunPhase.FINISHED -> R.string.run_again
                else -> R.string.run_stop
            }
        )
        ui.btnStart.setKey(
            if (state.running) R.drawable.btn_danger_selector else R.drawable.btn_go_selector
        )
        ui.btnStart.isEnabled = state.running || state.phase == RunPhase.FINISHED ||
            vm.linkState.value is LinkState.Connected

        // During a run the drive pad and the map actions are both forbidden
        // and dangerous, and the status box wants their space.
        val setup = if (state.running) View.GONE else View.VISIBLE
        ui.drivePanel.visibility = setup
        ui.actionRow.visibility = setup

        // The hint carries the reason START is refused, so the button never
        // looks broken. While running it gets out of the way.
        val hint = when {
            state.running -> null
            state.phase == RunPhase.FINISHED -> when (state.task) {
                Task.TASK1 -> getString(R.string.run_done_hint, state.identified, state.placed)
                Task.TASK2 -> getString(R.string.run_done_hint_t2)
            }
            else -> vm.startBlocker() ?: when (state.task) {
                Task.TASK1 -> getString(R.string.run_ready_hint, vm.arena.value.obstacles.size)
                Task.TASK2 -> getString(R.string.run_idle_hint_t2)
            }
        }
        ui.runHint.visibility = if (hint == null) View.GONE else View.VISIBLE
        ui.runHint.text = hint.orEmpty()
    }

    /**
     * The selected task reads as a lit key, the other as an unlit one. Both
     * are dead while a run is going: switching task mid-attempt would silently
     * change the budget the clock is counting against.
     */
    private fun renderTaskPicker(state: RunState) {
        val pairs = listOf(ui.btnTask1 to Task.TASK1, ui.btnTask2 to Task.TASK2)
        for ((button, task) in pairs) {
            val on = state.task == task
            button.setKey(if (on) R.drawable.btn_accent_selector else R.drawable.btn_pad_selector)
            button.setTextColor(color(if (on) R.color.arena_on_accent else R.color.arena_text_muted))
            button.isEnabled = !state.running
        }
    }

    private fun color(id: Int) = ContextCompat.getColor(this, id)

    /**
     * Swap a key's face at runtime.
     *
     * MaterialButton keeps its own background tint and will paint it over
     * anything we set, so the tint has to be cleared as well -- that is the
     * whole reason this is a helper rather than a plain `background =`.
     */
    private fun MaterialButton.setKey(drawable: Int) {
        backgroundTintList = null
        background = ContextCompat.getDrawable(this@MainActivity, drawable)
    }

    private fun wireReplay() {
        ui.btnReplayPlay.setOnClickListener { vm.toggleReplayPlayback() }
        ui.btnReplayClose.setOnClickListener { vm.closeReplay() }
        ui.replaySeek.setOnSeekBarChangeListener(object : SeekBar.OnSeekBarChangeListener {
            override fun onProgressChanged(bar: SeekBar?, value: Int, fromUser: Boolean) {
                if (fromUser) vm.scrubTo(value)
            }
            override fun onStartTrackingTouch(bar: SeekBar?) = Unit
            override fun onStopTrackingTouch(bar: SeekBar?) = Unit
        })
    }

    private fun wireTrafficDrawer() {
        ui.btnTraffic.setOnClickListener { toggleTraffic() }
        ui.btnInject.setOnClickListener { injectTyped() }
        // Soft keyboards disagree about which action a plain Enter reports, and
        // some report none at all, so accept the lot rather than only SEND.
        ui.injectField.setOnEditorActionListener { _, actionId, event ->
            val enterPressed = event?.keyCode == KeyEvent.KEYCODE_ENTER &&
                event.action == KeyEvent.ACTION_DOWN
            when (actionId) {
                EditorInfo.IME_ACTION_SEND,
                EditorInfo.IME_ACTION_DONE,
                EditorInfo.IME_ACTION_GO,
                EditorInfo.IME_ACTION_NEXT,
                EditorInfo.IME_ACTION_UNSPECIFIED -> { injectTyped(); true }
                else -> if (enterPressed) { injectTyped(); true } else false
            }
        }
    }

    /**
     * Status and Traffic occupy the same slot. The side column on a Tab A7
     * Lite is only about 600dp tall and the panels above and below it are
     * fixed, so showing both text panels at once leaves neither with room for
     * any text.
     */
    private fun toggleTraffic() {
        val showingTraffic = ui.trafficPanel.visibility == View.VISIBLE
        ui.trafficPanel.visibility = if (showingTraffic) View.GONE else View.VISIBLE
        ui.statusPanel.visibility = if (showingTraffic) View.VISIBLE else View.GONE
        ui.btnTraffic.text = if (showingTraffic) "⌄" else "⌃"
    }

    private fun injectTyped() {
        val line = ui.injectField.text.toString().trim()
        if (line.isEmpty()) return
        vm.injectInbound(line)
        ui.injectField.setText("")
    }

    // -----------------------------------------------------------------
    // Observation
    // -----------------------------------------------------------------

    private fun observe() {
        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {

                launch {
                    vm.arena.collect { state ->
                        ui.arena.state = state
                        val r = state.robot
                        ui.robotChip.text = getString(R.string.robot_format, r.x, r.y, r.facing.letter)
                        // The run panel reads the map (how many obstacles, which
                        // are missing a face), so it has to repaint when the map
                        // changes and not only when the run state does.
                        renderRun(vm.run.value)
                    }
                }

                launch {
                    vm.linkState.collect { s ->
                        ui.linkStateText.text = s.label
                        val tint = when {
                            s.isConnected -> R.color.arena_ok
                            s is LinkState.Disconnected -> R.color.arena_text_faint
                            s is LinkState.Failed -> R.color.arena_bad
                            else -> R.color.arena_accent
                        }
                        ui.statusDot.backgroundTintList =
                            ContextCompat.getColorStateList(this@MainActivity, tint)
                        ui.btnDisconnect.isEnabled = s !is LinkState.Disconnected
                        // Likewise: START is disabled while disconnected, so
                        // connecting has to re-enable it.
                        renderRun(vm.run.value)
                    }
                }

                launch {
                    vm.status.collect { lines ->
                        ui.statusBox.text = lines.joinToString("\n")
                        ui.statusScroll.post { ui.statusScroll.fullScroll(View.FOCUS_DOWN) }
                    }
                }

                launch {
                    vm.log.collect { lines ->
                        ui.logBox.text = lines.joinToString("\n")
                        ui.logScroll.post { ui.logScroll.fullScroll(View.FOCUS_DOWN) }
                    }
                }

                launch {
                    vm.replay.collect { r ->
                        ui.replayBar.visibility = if (r.active) View.VISIBLE else View.GONE
                        // The live pose chip is wrong during replay, and the
                        // scrubber sits exactly where it lives.
                        ui.robotChip.visibility = if (r.active) View.GONE else View.VISIBLE
                        if (!r.active) {
                            ui.arena.replayPose = null
                            ui.arena.replayTrail = null
                            ui.replayNote.visibility = View.GONE
                            return@collect
                        }
                        val frame = r.current
                        ui.arena.replayPose = frame?.robot
                        ui.arena.replayTrail = frame?.trail
                        ui.replaySeek.max = (r.total - 1).coerceAtLeast(0)
                        ui.replaySeek.progress = r.index
                        ui.replayLabel.text =
                            getString(R.string.replay_counter, r.index + 1, r.total)
                        ui.btnReplayPlay.setText(
                            if (r.playing) R.string.replay_pause else R.string.replay_play
                        )
                        val note = frame?.note
                        ui.replayNote.visibility = if (note != null) View.VISIBLE else View.GONE
                        if (note != null) ui.replayNote.text = note
                    }
                }

                launch { vm.runClock.collect { ui.runClock.text = it } }

                launch { vm.run.collect { renderRun(it) } }

                launch { vm.notices.collect { toast(it) } }
            }
        }
    }

    private fun toast(text: String) = Toast.makeText(this, text, Toast.LENGTH_SHORT).show()
}
