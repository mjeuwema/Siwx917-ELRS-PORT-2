package com.mjeuw.elrsblecompanion

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattService
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.os.SystemClock
import org.json.JSONObject
import java.nio.charset.Charset
import java.util.UUID
import java.util.concurrent.Executors
import java.util.regex.Pattern

data class ElrsDevice(
    val name: String,
    val address: String,
    val device: BluetoothDevice,
)

data class CommandResult(
    val command: String,
    val response: String,
)

class ElrsBleClient(
    private val context: Context,
    private val listener: Listener,
) {
    interface Listener {
        fun onDeviceFound(device: ElrsDevice)
        fun onConnectionState(text: String)
        fun onLog(text: String)
        fun onConfigLoaded(values: Map<String, String>)
        fun onCommandComplete(result: CommandResult)
        fun onError(text: String)
    }

    private enum class PendingKind {
        WRITE,
        READ,
    }

    private data class OperationResult(
        val ok: Boolean,
        val data: ByteArray = ByteArray(0),
        val message: String = "",
    )

    private val mainHandler = Handler(Looper.getMainLooper())
    private val executor = Executors.newSingleThreadExecutor()
    private val opLock = Object()
    private val seenDevices = linkedSetOf<String>()
    private val utf8: Charset = Charsets.UTF_8

    private val bluetoothManager =
        context.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
    private val bluetoothAdapter = bluetoothManager.adapter
    private val scanner get() = bluetoothAdapter.bluetoothLeScanner

    private var gatt: BluetoothGatt? = null
    private var commandChar: BluetoothGattCharacteristic? = null
    private var responseChar: BluetoothGattCharacteristic? = null
    private var pendingKind: PendingKind? = null
    private var pendingDone = false
    private var pendingResult = OperationResult(false, message = "not started")

    @Volatile
    private var ready = false

    private val scanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val recordName = result.scanRecord?.deviceName
            val deviceName = safeDeviceName(result.device)
            val name = recordName ?: deviceName ?: "Unknown"
            val services = result.scanRecord?.serviceUuids.orEmpty()
            val hasService = services.any { it.uuid == SERVICE_UUID }
            val looksLikeElrs = name == DEVICE_NAME || hasService

            if (!looksLikeElrs || !seenDevices.add(result.device.address)) {
                return
            }

            post {
                listener.onDeviceFound(
                    ElrsDevice(
                        name = name,
                        address = result.device.address,
                        device = result.device,
                    ),
                )
            }
        }

        override fun onScanFailed(errorCode: Int) {
            postError("BLE scan failed: $errorCode")
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                log("Connected; discovering services")
                postState("Connected, discovering services...")
                @Suppress("DEPRECATION")
                gatt.discoverServices()
                return
            }

            if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                ready = false
                commandChar = null
                responseChar = null
                log("Disconnected, status=$status")
                postState("Disconnected")
                completePending(PendingKind.WRITE, false, message = "disconnected")
                completePending(PendingKind.READ, false, message = "disconnected")
            }
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                postError("Service discovery failed: $status")
                return
            }

            val service = gatt.getService(SERVICE_UUID)
            if (service == null) {
                postError("ELRS service E7E0 not found")
                return
            }

            commandChar = service.getCharacteristic(COMMAND_UUID)
            responseChar = service.getCharacteristic(RESPONSE_UUID)
            if (commandChar == null || responseChar == null) {
                postError("ELRS command/response characteristics not found")
                return
            }

            commandChar?.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
            ready = true
            logService(service)
            postState("Ready")
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int,
        ) {
            completePending(
                PendingKind.WRITE,
                status == BluetoothGatt.GATT_SUCCESS,
                message = "write status=$status",
            )
        }

        @Deprecated("Used on Android 12 and older")
        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            status: Int,
        ) {
            @Suppress("DEPRECATION")
            completePending(
                PendingKind.READ,
                status == BluetoothGatt.GATT_SUCCESS,
                characteristic.value ?: ByteArray(0),
                "read status=$status",
            )
        }

        override fun onCharacteristicRead(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray,
            status: Int,
        ) {
            completePending(
                PendingKind.READ,
                status == BluetoothGatt.GATT_SUCCESS,
                value,
                "read status=$status",
            )
        }
    }

    @SuppressLint("MissingPermission")
    fun startScan() {
        seenDevices.clear()
        postState("Scanning...")
        log("Scan started")

        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()

        try {
            scanner?.startScan(null, settings, scanCallback)
        } catch (ex: SecurityException) {
            postError("Missing BLE scan permission: ${ex.message}")
        } catch (ex: RuntimeException) {
            postError("Could not start BLE scan: ${ex.message}")
        }
    }

    @SuppressLint("MissingPermission")
    fun stopScan() {
        try {
            scanner?.stopScan(scanCallback)
            log("Scan stopped")
        } catch (_: Exception) {
            // Best-effort cleanup.
        }
    }

    @SuppressLint("MissingPermission")
    fun connect(device: BluetoothDevice) {
        stopScan()
        disconnect()
        ready = false
        postState("Connecting to ${safeDeviceName(device) ?: device.address}...")
        log("Connecting to ${device.address}")
        gatt = device.connectGatt(context, false, gattCallback, BluetoothDevice.TRANSPORT_LE)
    }

    @SuppressLint("MissingPermission")
    fun disconnect() {
        ready = false
        commandChar = null
        responseChar = null
        try {
            gatt?.disconnect()
            gatt?.close()
        } catch (_: Exception) {
            // Best effort.
        }
        gatt = null
    }

    fun runCommandAsync(command: String) {
        executor.execute {
            try {
                val response = runCommandBlocking(command)
                post {
                    listener.onCommandComplete(CommandResult(command, response))
                }
            } catch (ex: Exception) {
                postError("Command '$command' failed: ${ex.message}")
            }
        }
    }

    fun loadConfigAsync() {
        executor.execute {
            try {
                val values = linkedMapOf<String, String>()
                for (field in EDITOR_READ_FIELDS) {
                    val response = runCommandBlocking("jget $field")
                    val pair = parseJget(response)
                    values[pair.first] = pair.second
                }
                post {
                    listener.onConfigLoaded(values)
                }
            } catch (ex: Exception) {
                postError("Config load failed: ${ex.message}")
            }
        }
    }

    fun applyChangesAsync(changes: List<Pair<String, String>>) {
        executor.execute {
            try {
                val responses = mutableListOf<String>()
                for ((key, value) in changes) {
                    val command = "set $key=$value"
                    responses += "$command -> ${runCommandBlocking(command)}"
                }
                responses += "jstatus -> ${runCommandBlocking("jstatus")}"
                post {
                    listener.onCommandComplete(
                        CommandResult("apply ${changes.size} change(s)", responses.joinToString("\n")),
                    )
                }
            } catch (ex: Exception) {
                postError("Apply failed: ${ex.message}")
            }
        }
    }

    fun saveAsync() = runCommandAsync("save")

    fun reloadAsync() {
        executor.execute {
            try {
                val reload = runCommandBlocking("reload")
                val values = linkedMapOf<String, String>()
                for (field in EDITOR_READ_FIELDS) {
                    val pair = parseJget(runCommandBlocking("jget $field"))
                    values[pair.first] = pair.second
                }
                post {
                    listener.onCommandComplete(CommandResult("reload", reload))
                    listener.onConfigLoaded(values)
                }
            } catch (ex: Exception) {
                postError("Reload failed: ${ex.message}")
            }
        }
    }

    fun resetAsync() {
        executor.execute {
            try {
                val reset = runCommandBlocking("reset")
                val values = linkedMapOf<String, String>()
                for (field in EDITOR_READ_FIELDS) {
                    val pair = parseJget(runCommandBlocking("jget $field"))
                    values[pair.first] = pair.second
                }
                post {
                    listener.onCommandComplete(CommandResult("reset", reset))
                    listener.onConfigLoaded(values)
                }
            } catch (ex: Exception) {
                postError("Reset failed: ${ex.message}")
            }
        }
    }

    private fun runCommandBlocking(command: String): String {
        ensureReady()
        log("TX $command")

        if (isHelperCommand(command)) {
            writeAndWait(command)
            Thread.sleep(READ_SETTLE_MS)
            return readTextAndWait().trimEnd('\u0000', ' ', '\r', '\n').also {
                log("RX $it")
            }
        }

        writeAndWait(command)
        Thread.sleep(COMMAND_SETTLE_MS)

        writeAndWait("len")
        Thread.sleep(READ_SETTLE_MS)
        val lenText = readTextAndWait().trimEnd('\u0000', ' ', '\r', '\n')
        val expectedLength = parseLength(lenText)
        if (expectedLength == 0) {
            return ""
        }

        val pageCount = (expectedLength + PAGE_SIZE - 1) / PAGE_SIZE
        val response = StringBuilder(expectedLength + PAGE_SIZE)
        for (page in 0 until pageCount) {
            writeAndWait("page $page")
            Thread.sleep(READ_SETTLE_MS)
            response.append(readTextAndWait())
        }

        val assembled = response.toString()
            .replace("\u0000", "")
            .let { if (it.length > expectedLength) it.substring(0, expectedLength) else it }

        log("RX $assembled")
        return assembled
    }

    private fun parseJget(response: String): Pair<String, String> {
        val obj = JSONObject(response)
        if (!obj.optBoolean("ok", false)) {
            throw IllegalStateException(response)
        }
        val key = obj.getString("key")
        val rawValue = obj.opt("value")
        val value = if (rawValue == null || rawValue == JSONObject.NULL) "" else rawValue.toString()
        return key to value
    }

    private fun parseLength(text: String): Int {
        val matcher = LEN_PATTERN.matcher(text)
        if (!matcher.find()) {
            throw IllegalStateException("expected len=N, got '$text'")
        }
        return matcher.group(1)?.toIntOrNull()
            ?: throw IllegalStateException("bad len response '$text'")
    }

    private fun ensureReady() {
        if (!ready || gatt == null || commandChar == null || responseChar == null) {
            throw IllegalStateException("BLE is not ready")
        }
    }

    @SuppressLint("MissingPermission")
    private fun writeAndWait(text: String) {
        val g = gatt ?: throw IllegalStateException("not connected")
        val ch = commandChar ?: throw IllegalStateException("command characteristic missing")
        val payload = text.toByteArray(utf8)

        startPending(PendingKind.WRITE)
        val started = try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                g.writeCharacteristic(
                    ch,
                    payload,
                    BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT,
                ) == BluetoothGatt.GATT_SUCCESS
            } else {
                @Suppress("DEPRECATION")
                run {
                    ch.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                    ch.value = payload
                    g.writeCharacteristic(ch)
                }
            }
        } catch (ex: SecurityException) {
            completePending(PendingKind.WRITE, false, message = ex.message ?: "security error")
            false
        }

        if (!started) {
            failPending(PendingKind.WRITE, "write start failed")
        }

        val result = waitForPending()
        if (!result.ok) {
            throw IllegalStateException(result.message.ifEmpty { "write failed" })
        }
    }

    @SuppressLint("MissingPermission")
    private fun readTextAndWait(): String {
        val g = gatt ?: throw IllegalStateException("not connected")
        val ch = responseChar ?: throw IllegalStateException("response characteristic missing")

        startPending(PendingKind.READ)
        val started = try {
            g.readCharacteristic(ch)
        } catch (ex: SecurityException) {
            completePending(PendingKind.READ, false, message = ex.message ?: "security error")
            false
        }

        if (!started) {
            failPending(PendingKind.READ, "read start failed")
        }

        val result = waitForPending()
        if (!result.ok) {
            throw IllegalStateException(result.message.ifEmpty { "read failed" })
        }
        return String(result.data, utf8)
    }

    private fun startPending(kind: PendingKind) {
        synchronized(opLock) {
            pendingKind = kind
            pendingDone = false
            pendingResult = OperationResult(false, message = "pending")
        }
    }

    private fun completePending(
        expected: PendingKind,
        ok: Boolean,
        data: ByteArray = ByteArray(0),
        message: String = "",
    ) {
        synchronized(opLock) {
            if (pendingKind != expected) {
                return
            }
            pendingResult = OperationResult(ok, data, message)
            pendingDone = true
            pendingKind = null
            opLock.notifyAll()
        }
    }

    private fun failPending(kind: PendingKind, message: String): Nothing {
        completePending(kind, false, message = message)
        val result = waitForPending()
        throw IllegalStateException(result.message.ifEmpty { message })
    }

    private fun waitForPending(): OperationResult {
        val deadline = SystemClock.elapsedRealtime() + OP_TIMEOUT_MS
        synchronized(opLock) {
            while (!pendingDone) {
                val remaining = deadline - SystemClock.elapsedRealtime()
                if (remaining <= 0L) {
                    val kind = pendingKind
                    pendingKind = null
                    pendingDone = true
                    pendingResult = OperationResult(false, message = "timeout waiting for $kind")
                    break
                }
                opLock.wait(remaining)
            }
            return pendingResult
        }
    }

    private fun logService(service: BluetoothGattService) {
        log("ELRS service discovered")
        log("Response ${responseChar?.uuid}")
        log("Command ${commandChar?.uuid}")
        for (characteristic in service.characteristics) {
            log("Characteristic ${characteristic.uuid} props=0x${characteristic.properties.toString(16)}")
        }
    }

    private fun isHelperCommand(command: String): Boolean {
        val trimmed = command.trim()
        return trimmed == "len" ||
            trimmed.startsWith("page ", ignoreCase = true) ||
            trimmed.startsWith("chunk ", ignoreCase = true)
    }

    @SuppressLint("MissingPermission")
    private fun safeDeviceName(device: BluetoothDevice): String? {
        return try {
            device.name
        } catch (_: SecurityException) {
            null
        }
    }

    private fun post(block: () -> Unit) {
        mainHandler.post(block)
    }

    private fun postState(text: String) {
        post { listener.onConnectionState(text) }
    }

    private fun postError(text: String) {
        log("ERROR $text")
        post { listener.onError(text) }
    }

    private fun log(text: String) {
        post { listener.onLog(text) }
    }

    companion object {
        const val DEVICE_NAME = "ELRS-RX-BLE"
        val SERVICE_UUID: UUID = UUID.fromString("0000E7E0-0000-1000-8000-00805F9B34FB")
        val RESPONSE_UUID: UUID = UUID.fromString("0000E7E1-0000-1000-8000-00805F9B34FB")
        val COMMAND_UUID: UUID = UUID.fromString("0000E7E2-0000-1000-8000-00805F9B34FB")
        private const val OP_TIMEOUT_MS = 2_000L
        private const val COMMAND_SETTLE_MS = 120L
        private const val READ_SETTLE_MS = 45L
        private const val PAGE_SIZE = 20
        private val LEN_PATTERN = Pattern.compile("len=(\\d+)")

        val EDITOR_READ_FIELDS = listOf(
            "version",
            "flags",
            "uid",
            "serial",
            "failsafe",
            "rate",
            "model",
            "tlm_off",
            "tlm_interval",
            "power",
            "domL",
            "domH",
            "web_domain",
            "wifi_interval",
            "uart_baud",
            "wifi_channel",
            "wifi_ssid",
            "wifi_custom",
            "lock_on_first",
            "is_airport",
            "dji_armed",
            "mav_tgt",
            "mav_src",
            "team_ch",
            "team_pos",
            "bind",
            "vbind",
            "web_custom",
            "dirty",
        )
    }
}
