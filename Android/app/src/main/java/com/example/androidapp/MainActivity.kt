package com.example.androidapp

import android.os.Bundle
import android.view.View
import android.view.ViewGroup
import android.view.inputmethod.EditorInfo
import android.widget.BaseAdapter
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
import com.example.androidapp.databinding.ActivityMainBinding
import com.example.androidapp.databinding.DialogDevicesBinding
import com.example.androidapp.databinding.ItemDeviceBinding
import com.example.androidapp.link.BluetoothLink
import com.example.androidapp.link.LinkState
import com.example.androidapp.link.RemoteDevice
import com.example.androidapp.protocol.Move
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
        wireTrafficDrawer()
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
            .setNegativeButton(android.R.string.cancel) { d, _ -> vm.stopScan(); d.dismiss() }
            .create()

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
    }

    private fun wireTrafficDrawer() {
        ui.btnTraffic.setOnClickListener { toggleTraffic() }
        ui.btnInject.setOnClickListener { injectTyped() }
        ui.injectField.setOnEditorActionListener { _, actionId, _ ->
            if (actionId == EditorInfo.IME_ACTION_SEND) { injectTyped(); true } else false
        }
    }

    private fun toggleTraffic() {
        val showing = ui.trafficPanel.visibility == View.VISIBLE
        ui.trafficPanel.visibility = if (showing) View.GONE else View.VISIBLE
        ui.btnTraffic.text = if (showing) "⌄" else "⌃"
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

                launch { vm.notices.collect { toast(it) } }
            }
        }
    }

    private fun toast(text: String) = Toast.makeText(this, text, Toast.LENGTH_SHORT).show()
}
