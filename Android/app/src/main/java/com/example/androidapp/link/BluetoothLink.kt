package com.example.androidapp.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothServerSocket
import android.bluetooth.BluetoothSocket
import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import android.content.IntentFilter
import android.content.pm.PackageManager
import android.os.Build
import android.util.Log
import androidx.core.content.ContextCompat
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancelAndJoin
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asSharedFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.io.IOException
import java.io.InputStream
import java.io.OutputStream
import java.util.UUID

/**
 * Bluetooth Serial Port Profile transport. Covers C.1, C.2 and C.8.
 *
 * Two things make C.8 work rather than merely appear to:
 *
 *  1. While disconnected the link runs a *client* retry and a *server* accept
 *     at the same time. The checklist wants the app to come back when "the
 *     Bluetooth device connects with the AA again" — the remote side is the
 *     one initiating — so the tablet has to be listening, not only dialling.
 *
 *  2. Nothing blocking ever runs on the main thread. A dropped link surfaces
 *     as an IOException on a background coroutine, the state flow flips to
 *     Disconnected, and the supervision loop starts trying again. The UI is
 *     never given a chance to hang.
 */
class BluetoothLink(private val context: Context) : Link {

    private val scope = CoroutineScope(SupervisorJob() + Dispatchers.IO)

    private val _state = MutableStateFlow<LinkState>(LinkState.Disconnected)
    override val state: StateFlow<LinkState> = _state.asStateFlow()

    private val _incoming = MutableSharedFlow<String>(extraBufferCapacity = 256)
    override val incoming: SharedFlow<String> = _incoming.asSharedFlow()

    private val _discovered = MutableStateFlow<List<RemoteDevice>>(emptyList())
    override val discovered: StateFlow<List<RemoteDevice>> = _discovered.asStateFlow()

    private val _scanning = MutableStateFlow(false)
    override val scanning: StateFlow<Boolean> = _scanning.asStateFlow()

    private val adapter: BluetoothAdapter? =
        (context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager)?.adapter

    private var superviseJob: Job? = null

    @Volatile private var wantConnection = false
    @Volatile private var target: RemoteDevice? = null
    @Volatile private var liveSocket: BluetoothSocket? = null
    @Volatile private var output: OutputStream? = null
    @Volatile private var serverSocket: BluetoothServerSocket? = null

    val isSupported: Boolean get() = adapter != null
    val isEnabled: Boolean get() = adapter?.isEnabled == true

    // -----------------------------------------------------------------
    // Discovery — C.2
    // -----------------------------------------------------------------

    private val discoveryReceiver = object : BroadcastReceiver() {
        @SuppressLint("MissingPermission")
        override fun onReceive(ctx: Context?, intent: Intent?) {
            when (intent?.action) {
                BluetoothDevice.ACTION_FOUND -> {
                    val device: BluetoothDevice? =
                        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE, BluetoothDevice::class.java)
                        } else {
                            @Suppress("DEPRECATION")
                            intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE)
                        }
                    device ?: return
                    if (!canConnect()) return
                    val found = RemoteDevice(device.name.orEmpty(), device.address)
                    if (_discovered.value.none { it.address == found.address }) {
                        _discovered.value = _discovered.value + found
                    }
                }
                BluetoothAdapter.ACTION_DISCOVERY_STARTED -> _scanning.value = true
                BluetoothAdapter.ACTION_DISCOVERY_FINISHED -> _scanning.value = false
            }
        }
    }

    init {
        val filter = IntentFilter().apply {
            addAction(BluetoothDevice.ACTION_FOUND)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED)
            addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED)
        }
        ContextCompat.registerReceiver(
            context, discoveryReceiver, filter, ContextCompat.RECEIVER_NOT_EXPORTED
        )
    }

    @SuppressLint("MissingPermission")
    override fun pairedDevices(): List<RemoteDevice> {
        if (!canConnect()) return emptyList()
        return adapter?.bondedDevices.orEmpty().map { RemoteDevice(it.name.orEmpty(), it.address) }
    }

    @SuppressLint("MissingPermission")
    override fun startScan() {
        if (!canScan()) return
        val a = adapter ?: return
        _discovered.value = emptyList()
        if (a.isDiscovering) a.cancelDiscovery()
        a.startDiscovery()
    }

    @SuppressLint("MissingPermission")
    override fun stopScan() {
        if (!canScan()) return
        adapter?.takeIf { it.isDiscovering }?.cancelDiscovery()
        _scanning.value = false
    }

    // -----------------------------------------------------------------
    // Connection lifecycle — C.1 and C.8
    // -----------------------------------------------------------------

    override fun connect(device: RemoteDevice?) {
        target = device
        wantConnection = true
        closeSockets() // see disconnect(): unblock the reader before joining
        scope.launch {
            superviseJob?.cancelAndJoin()
            superviseJob = scope.launch { supervise() }
        }
    }

    override fun disconnect() {
        wantConnection = false
        target = null
        // Close first. The reader is parked inside a blocking InputStream.read()
        // that cancellation cannot interrupt — closing the socket is what makes
        // it throw and return. Waiting on cancelAndJoin() beforehand would hang
        // until the robot happened to send something.
        closeSockets()
        scope.launch {
            superviseJob?.cancelAndJoin()
            superviseJob = null
            _state.value = LinkState.Disconnected
        }
    }

    override fun shutdown() {
        wantConnection = false
        runCatching { context.unregisterReceiver(discoveryReceiver) }
        closeSockets()
        scope.coroutineContext[Job]?.cancel()
    }

    private suspend fun supervise() {
        // Discovery cripples RFCOMM throughput and connect latency.
        stopScan()

        while (scope.isActive && wantConnection) {
            val t = target
            _state.value = if (t != null) LinkState.Connecting(t.label) else LinkState.Listening

            val socket = openSocket(t)
            if (socket == null) {
                if (!wantConnection) break
                delay(RETRY_DELAY_MS)
                continue
            }

            liveSocket = socket
            output = runCatching { socket.outputStream }.getOrNull()
            val remote = describe(socket)
            _state.value = LinkState.Connected(remote)

            // Blocks here for the life of the connection.
            pump(socket)

            liveSocket = null
            output = null
            runCatching { socket.close() }

            if (!wantConnection) break
            _state.value = LinkState.Disconnected
            delay(RETRY_DELAY_MS)
        }

        if (!wantConnection) _state.value = LinkState.Disconnected
    }

    /**
     * Races an outgoing connect against an incoming accept and returns whichever
     * completes first. Either can be null on this pass; the caller retries.
     */
    @SuppressLint("MissingPermission")
    private suspend fun openSocket(t: RemoteDevice?): BluetoothSocket? = withContext(Dispatchers.IO) {
        if (!canConnect() || adapter?.isEnabled != true) return@withContext null

        // Server first, so it is already listening while the client attempt runs.
        val server = runCatching {
            adapter.listenUsingRfcommWithServiceRecord(SERVICE_NAME, SPP_UUID)
        }.getOrNull()
        serverSocket = server

        val accepted = java.util.concurrent.atomic.AtomicReference<BluetoothSocket?>()
        val acceptThread = Thread {
            runCatching { server?.accept() }.getOrNull()?.let { accepted.set(it) }
        }.apply { isDaemon = true; start() }

        // Client attempt, if the user picked a device.
        var client: BluetoothSocket? = null
        if (t != null) {
            client = runCatching {
                val device = adapter.getRemoteDevice(t.address)
                device.createRfcommSocketToServiceRecord(SPP_UUID).also { s ->
                    s.connect() // blocking, ~12 s worst case
                }
            }.onFailure { Log.d(TAG, "client connect failed: ${it.message}") }.getOrNull()
        } else {
            // Nothing to dial: give the accept thread a window before retrying.
            var waited = 0L
            while (waited < ACCEPT_WINDOW_MS && accepted.get() == null && wantConnection) {
                delay(100); waited += 100
            }
        }

        val incomingSocket = accepted.get()
        val winner = client ?: incomingSocket

        // Tear down whichever lost.
        runCatching { server?.close() }
        serverSocket = null
        acceptThread.interrupt()
        if (winner !== incomingSocket) runCatching { incomingSocket?.close() }

        winner
    }

    /**
     * Reads until the socket dies.
     *
     * Handles both framed and unframed senders: complete lines are emitted on
     * every newline, and if a read leaves a partial buffer with nothing further
     * waiting, that buffer is emitted too. The AMD tool sends one message per
     * write and does not always terminate it, so without the second rule a
     * perfectly good message would sit in the buffer forever.
     */
    private fun pump(socket: BluetoothSocket) {
        val input: InputStream = runCatching { socket.inputStream }.getOrNull() ?: return
        val chunk = ByteArray(1024)
        val acc = StringBuilder()

        try {
            while (wantConnection) {
                val n = input.read(chunk)
                if (n < 0) break
                if (n == 0) continue
                acc.append(String(chunk, 0, n, Charsets.UTF_8))

                var cut = acc.indexOfFirst { it == '\n' || it == '\r' }
                while (cut >= 0) {
                    val line = acc.substring(0, cut).trim()
                    acc.delete(0, cut + 1)
                    if (line.isNotEmpty()) _incoming.tryEmit(line)
                    cut = acc.indexOfFirst { it == '\n' || it == '\r' }
                }

                if (acc.isNotEmpty() && runCatching { input.available() }.getOrDefault(0) == 0) {
                    val line = acc.toString().trim()
                    acc.clear()
                    if (line.isNotEmpty()) _incoming.tryEmit(line)
                }
            }
        } catch (e: IOException) {
            Log.d(TAG, "link dropped: ${e.message}")
        }
    }

    override fun send(line: String): Boolean {
        val out = output ?: return false
        return try {
            synchronized(this) {
                out.write((line + "\n").toByteArray(Charsets.UTF_8))
                out.flush()
            }
            true
        } catch (e: IOException) {
            Log.d(TAG, "send failed: ${e.message}")
            false
        }
    }

    // -----------------------------------------------------------------

    @SuppressLint("MissingPermission")
    private fun describe(socket: BluetoothSocket): RemoteDevice {
        val d = socket.remoteDevice
        val name = if (canConnect()) d?.name.orEmpty() else ""
        return RemoteDevice(name, d?.address.orEmpty())
    }

    private fun closeSockets() {
        runCatching { liveSocket?.close() }
        liveSocket = null
        output = null
        runCatching { serverSocket?.close() }
        serverSocket = null
    }

    private fun canConnect(): Boolean =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) granted(android.Manifest.permission.BLUETOOTH_CONNECT)
        else granted(android.Manifest.permission.BLUETOOTH)

    private fun canScan(): Boolean =
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) granted(android.Manifest.permission.BLUETOOTH_SCAN)
        else granted(android.Manifest.permission.ACCESS_FINE_LOCATION)

    private fun granted(permission: String): Boolean =
        ContextCompat.checkSelfPermission(context, permission) == PackageManager.PERMISSION_GRANTED

    private inline fun CharSequence.indexOfFirst(predicate: (Char) -> Boolean): Int {
        for (i in indices) if (predicate(this[i])) return i
        return -1
    }

    companion object {
        private const val TAG = "BluetoothLink"

        /** Standard Serial Port Profile UUID. The Pi's RFCOMM server advertises this. */
        val SPP_UUID: UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB")

        private const val SERVICE_NAME = "MDP-Tablet"
        private const val RETRY_DELAY_MS = 2_000L
        private const val ACCEPT_WINDOW_MS = 8_000L

        /** Permissions the activity must hold before any of this works. */
        fun requiredPermissions(): Array<String> =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
                arrayOf(
                    android.Manifest.permission.BLUETOOTH_CONNECT,
                    android.Manifest.permission.BLUETOOTH_SCAN,
                )
            } else {
                arrayOf(
                    android.Manifest.permission.BLUETOOTH,
                    android.Manifest.permission.BLUETOOTH_ADMIN,
                    android.Manifest.permission.ACCESS_FINE_LOCATION,
                )
            }
    }
}
