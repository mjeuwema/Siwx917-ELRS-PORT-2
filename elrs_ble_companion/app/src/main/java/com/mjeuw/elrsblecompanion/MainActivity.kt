package com.mjeuw.elrsblecompanion

import android.Manifest
import android.app.Activity
import android.app.AlertDialog
import android.content.pm.PackageManager
import android.graphics.Color
import android.os.Build
import android.os.Bundle
import android.text.InputType
import android.view.Gravity
import android.view.View
import android.view.ViewGroup
import android.widget.ArrayAdapter
import android.widget.Button
import android.widget.EditText
import android.widget.LinearLayout
import android.widget.ScrollView
import android.widget.Spinner
import android.widget.Switch
import android.widget.TextView
import android.widget.Toast
import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale

class MainActivity : Activity(), ElrsBleClient.Listener {
    private lateinit var client: ElrsBleClient
    private lateinit var statusText: TextView
    private lateinit var unsavedBanner: TextView
    private lateinit var devicesContainer: LinearLayout
    private lateinit var editorContainer: LinearLayout
    private lateinit var logText: TextView
    private lateinit var rawInput: EditText

    private val fieldControls = linkedMapOf<String, FieldControl>()
    private val loadedValues = linkedMapOf<String, String>()
    private val logLines = ArrayDeque<String>()
    private val deviceButtons = linkedMapOf<String, Button>()
    private val timeFormat = SimpleDateFormat("HH:mm:ss", Locale.US)

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        client = ElrsBleClient(this, this)
        buildUi()
        ensurePermissions()
    }

    override fun onDestroy() {
        client.disconnect()
        super.onDestroy()
    }

    override fun onDeviceFound(device: ElrsDevice) {
        if (deviceButtons.containsKey(device.address)) {
            return
        }

        val button = Button(this).apply {
            text = "${device.name}\n${device.address}"
            setAllCaps(false)
            setOnClickListener {
                client.connect(device.device)
            }
        }
        deviceButtons[device.address] = button
        devicesContainer.addView(button, matchWrap())
    }

    override fun onConnectionState(text: String) {
        statusText.text = "BLE: $text"
        appendLog("STATE $text")
        if (text == "Ready") {
            client.loadConfigAsync()
        }
    }

    override fun onLog(text: String) {
        appendLog(text)
    }

    override fun onConfigLoaded(values: Map<String, String>) {
        loadedValues.clear()
        loadedValues.putAll(values)
        for ((key, control) in fieldControls) {
            control.setValue(values[key].orEmpty())
        }
        updateDirtyBanner(values["dirty"] == "1")
        appendLog("Loaded ${values.size} config fields")
    }

    override fun onCommandComplete(result: CommandResult) {
        appendLog("> ${result.command}")
        appendLog(result.response)

        if (result.command.startsWith("apply ") ||
            result.command == "save" ||
            result.command == "reset"
        ) {
            client.loadConfigAsync()
        }
    }

    override fun onError(text: String) {
        appendLog("ERROR $text")
        Toast.makeText(this, text, Toast.LENGTH_LONG).show()
    }

    private fun buildUi() {
        val root = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setBackgroundColor(Color.rgb(247, 248, 242))
        }

        statusText = TextView(this).apply {
            text = "BLE: Not connected"
            textSize = 16f
            setTextColor(Color.WHITE)
            setBackgroundColor(Color.rgb(21, 54, 66))
            setPadding(18, 16, 18, 16)
        }
        root.addView(statusText, matchWrap())

        val scroll = ScrollView(this)
        val content = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
            setPadding(18, 18, 18, 28)
        }
        scroll.addView(content)
        root.addView(scroll, LinearLayout.LayoutParams(-1, 0, 1f))
        setContentView(root)

        addTitle(content, "ELRS BLE Companion")
        addBodyText(
            content,
            "This app talks directly to the SiWx917 RX over BLE. It loads config with short jget field reads first, so it does not depend on long meta/keys reads.",
        )

        val scanRow = row()
        scanRow.addView(button("Scan") {
            if (ensurePermissions()) {
                devicesContainer.removeAllViews()
                deviceButtons.clear()
                client.startScan()
            }
        }, weightWrap())
        scanRow.addView(button("Stop") { client.stopScan() }, weightWrap())
        scanRow.addView(button("Disconnect") { client.disconnect() }, weightWrap())
        content.addView(scanRow, matchWrap())

        devicesContainer = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
        }
        content.addView(devicesContainer, matchWrap())

        addSection(content, "Dashboard")
        unsavedBanner = TextView(this).apply {
            text = "No unsaved changes reported"
            setPadding(14, 12, 14, 12)
            setTextColor(Color.WHITE)
            setBackgroundColor(Color.rgb(74, 105, 85))
        }
        content.addView(unsavedBanner, matchWrap())

        val actionRow = row()
        actionRow.addView(button("Refresh") { client.loadConfigAsync() }, weightWrap())
        actionRow.addView(button("Apply") { applyChangedFields() }, weightWrap())
        actionRow.addView(button("Save") { client.saveAsync() }, weightWrap())
        content.addView(actionRow, matchWrap())

        val actionRow2 = row()
        actionRow2.addView(button("Reload") { client.reloadAsync() }, weightWrap())
        actionRow2.addView(button("Factory Reset") { confirmReset() }, weightWrap())
        content.addView(actionRow2, matchWrap())

        addSection(content, "Edit Config")
        editorContainer = LinearLayout(this).apply {
            orientation = LinearLayout.VERTICAL
        }
        content.addView(editorContainer, matchWrap())
        buildEditor(editorContainer)

        addSection(content, "Raw Command")
        rawInput = EditText(this).apply {
            hint = "Try: ping, jget serial, diag"
            setSingleLine(true)
            inputType = InputType.TYPE_CLASS_TEXT
        }
        content.addView(rawInput, matchWrap())
        val rawRow = row()
        rawRow.addView(button("Run") {
            val command = rawInput.text.toString().trim()
            if (command.isNotEmpty()) {
                client.runCommandAsync(command)
            }
        }, weightWrap())
        rawRow.addView(button("Diag") { client.runCommandAsync("diag") }, weightWrap())
        rawRow.addView(button("Meta") { client.runCommandAsync("meta") }, weightWrap())
        content.addView(rawRow, matchWrap())

        addSection(content, "Debug Log")
        logText = TextView(this).apply {
            textSize = 12f
            setTextColor(Color.rgb(24, 31, 35))
            setBackgroundColor(Color.rgb(232, 235, 224))
            setPadding(12, 12, 12, 12)
            text = "Ready.\n"
        }
        content.addView(logText, matchWrap())
    }

    private fun buildEditor(container: LinearLayout) {
        addReadOnly(container, "Config version", "version")
        addReadOnly(container, "Flags", "flags")
        addReadOnly(container, "UID", "uid")
        addReadOnly(container, "Web custom flag", "web_custom")
        addReadOnly(container, "Dirty flag", "dirty")
        addSpinner(
            container,
            "Serial protocol",
            "serial",
            listOf("0" to "CRSF", "2" to "SBUS", "4" to "SUMD", "7" to "MAVLink"),
        )
        addSpinner(
            container,
            "Failsafe",
            "failsafe",
            listOf("0" to "No pulses", "1" to "Hold last", "2" to "Set positions"),
        )
        addNumber(container, "Packet rate index", "rate")
        addNumber(container, "Model ID (255 disables model match)", "model")
        addSwitch(container, "Force telemetry off", "tlm_off")
        addNumber(container, "Telemetry interval", "tlm_interval")
        addSpinner(
            container,
            "RX telemetry power",
            "power",
            listOf("match" to "Match TX", "10" to "10 dBm", "14" to "14 dBm", "17" to "17 dBm", "20" to "20 dBm"),
        )
        addNumber(container, "Low-band domain raw", "domL")
        addNumber(container, "High-band domain raw", "domH")
        addSpinner(
            container,
            "Web domain",
            "web_domain",
            listOf("0" to "AU915", "1" to "FCC915", "2" to "EU868", "3" to "IN866", "4" to "AU433", "5" to "EU433"),
        )
        addSignedNumber(container, "WiFi-on interval seconds (-1 disables)", "wifi_interval")
        addNumber(container, "UART baud", "uart_baud")
        addNumber(container, "WiFi channel", "wifi_channel")
        addTextField(container, "WiFi SSID", "wifi_ssid")
        addPassword(container, "WiFi password (leave blank to keep current)", "wifi_password")
        addSwitch(container, "Use custom WiFi", "wifi_custom")
        addSwitch(container, "Lock on first connection", "lock_on_first")
        addSwitch(container, "Airport mode", "is_airport")
        addSwitch(container, "DJI permanently armed", "dji_armed")
        addNumber(container, "MAVLink target system ID", "mav_tgt")
        addNumber(container, "MAVLink source system ID", "mav_src")
        addNumber(container, "Team race channel", "team_ch")
        addNumber(container, "Team race position", "team_pos")
        addSpinner(
            container,
            "Bind storage",
            "bind",
            listOf("0" to "Persistent", "1" to "Volatile", "2" to "Returnable", "3" to "Administered"),
        )
        addNumber(container, "Voltage bind threshold", "vbind")
    }

    private fun applyChangedFields() {
        val changes = mutableListOf<Pair<String, String>>()
        for ((key, control) in fieldControls) {
            if (!control.writable) {
                continue
            }

            val current = control.getValue().trim()
            if (key == "wifi_password" && current.isEmpty()) {
                continue
            }

            val previous = loadedValues[key].orEmpty()
            if (key == "wifi_password" || current != previous) {
                changes += key to current
            }
        }

        if (changes.isEmpty()) {
            appendLog("No edited fields to apply")
            return
        }

        confirmRiskyChangesThenApply(changes)
    }

    private fun confirmRiskyChangesThenApply(changes: List<Pair<String, String>>) {
        val warnings = mutableListOf<String>()
        if (changes.any { it.first == "serial" }) {
            warnings += "Serial protocol changed. Make sure your flight controller wiring/protocol matches before saving."
        }
        val modelChange = changes.firstOrNull { it.first == "model" }?.second
        if (modelChange != null && modelChange != "255") {
            warnings += "Model match will be enabled for model ID $modelChange."
        }

        if (warnings.isEmpty()) {
            client.applyChangesAsync(changes)
            return
        }

        AlertDialog.Builder(this)
            .setTitle("Apply these changes?")
            .setMessage(warnings.joinToString("\n\n"))
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Apply") { _, _ -> client.applyChangesAsync(changes) }
            .show()
    }

    private fun confirmReset() {
        AlertDialog.Builder(this)
            .setTitle("Factory reset receiver config?")
            .setMessage("This writes default config to NVM3. You cannot undo it from the app.")
            .setNegativeButton("Cancel", null)
            .setPositiveButton("Reset") { _, _ -> client.resetAsync() }
            .show()
    }

    private fun ensurePermissions(): Boolean {
        val missing = requiredPermissions().filter {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
                checkSelfPermission(it) != PackageManager.PERMISSION_GRANTED
            } else {
                false
            }
        }
        if (missing.isEmpty()) {
            return true
        }

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
            requestPermissions(missing.toTypedArray(), PERMISSION_REQUEST)
        }
        return false
    }

    override fun onRequestPermissionsResult(
        requestCode: Int,
        permissions: Array<out String>,
        grantResults: IntArray,
    ) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults)
        if (requestCode == PERMISSION_REQUEST &&
            grantResults.isNotEmpty() &&
            grantResults.all { it == PackageManager.PERMISSION_GRANTED }
        ) {
            appendLog("BLE permissions granted")
        } else {
            appendLog("BLE permissions denied")
        }
    }

    private fun requiredPermissions(): Array<String> {
        return if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
            )
        } else {
            arrayOf(
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
            )
        }
    }

    private fun addReadOnly(parent: LinearLayout, label: String, key: String) {
        val text = TextView(this).apply {
            text = "(not loaded)"
            textSize = 16f
            setPadding(0, 6, 0, 12)
        }
        addLabeledView(parent, label, text)
        fieldControls[key] = TextControl(key, text, writable = false)
    }

    private fun addTextField(parent: LinearLayout, label: String, key: String) {
        val input = EditText(this).apply {
            setSingleLine(true)
            inputType = InputType.TYPE_CLASS_TEXT
        }
        addLabeledView(parent, label, input)
        fieldControls[key] = EditControl(key, input)
    }

    private fun addPassword(parent: LinearLayout, label: String, key: String) {
        val input = EditText(this).apply {
            setSingleLine(true)
            inputType = InputType.TYPE_CLASS_TEXT or InputType.TYPE_TEXT_VARIATION_PASSWORD
        }
        addLabeledView(parent, label, input)
        fieldControls[key] = EditControl(key, input)
    }

    private fun addNumber(parent: LinearLayout, label: String, key: String) {
        val input = EditText(this).apply {
            setSingleLine(true)
            inputType = InputType.TYPE_CLASS_NUMBER
        }
        addLabeledView(parent, label, input)
        fieldControls[key] = EditControl(key, input)
    }

    private fun addSignedNumber(parent: LinearLayout, label: String, key: String) {
        val input = EditText(this).apply {
            setSingleLine(true)
            inputType = InputType.TYPE_CLASS_NUMBER or InputType.TYPE_NUMBER_FLAG_SIGNED
        }
        addLabeledView(parent, label, input)
        fieldControls[key] = EditControl(key, input)
    }

    private fun addSwitch(parent: LinearLayout, label: String, key: String) {
        @Suppress("DEPRECATION")
        val toggle = Switch(this).apply {
            text = label
            textSize = 15f
            setPadding(0, 8, 0, 8)
        }
        parent.addView(toggle, matchWrap())
        fieldControls[key] = SwitchControl(key, toggle)
    }

    private fun addSpinner(
        parent: LinearLayout,
        label: String,
        key: String,
        options: List<Pair<String, String>>,
    ) {
        val spinner = Spinner(this)
        val adapter = ArrayAdapter(
            this,
            android.R.layout.simple_spinner_item,
            options.map { it.second },
        )
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
        spinner.adapter = adapter
        addLabeledView(parent, label, spinner)
        fieldControls[key] = SpinnerControl(key, spinner, options)
    }

    private fun addLabeledView(parent: LinearLayout, label: String, view: View) {
        val labelView = TextView(this).apply {
            text = label
            textSize = 14f
            setTextColor(Color.rgb(44, 61, 64))
            setPadding(0, 12, 0, 2)
        }
        parent.addView(labelView, matchWrap())
        parent.addView(view, matchWrap())
    }

    private fun addTitle(parent: LinearLayout, text: String) {
        parent.addView(TextView(this).apply {
            this.text = text
            textSize = 26f
            setTextColor(Color.rgb(21, 54, 66))
            setPadding(0, 0, 0, 8)
        }, matchWrap())
    }

    private fun addSection(parent: LinearLayout, text: String) {
        parent.addView(TextView(this).apply {
            this.text = text
            textSize = 20f
            setTextColor(Color.rgb(21, 54, 66))
            setPadding(0, 24, 0, 8)
        }, matchWrap())
    }

    private fun addBodyText(parent: LinearLayout, text: String) {
        parent.addView(TextView(this).apply {
            this.text = text
            textSize = 14f
            setTextColor(Color.rgb(58, 72, 75))
            setPadding(0, 0, 0, 14)
        }, matchWrap())
    }

    private fun button(text: String, action: () -> Unit): Button {
        return Button(this).apply {
            this.text = text
            setAllCaps(false)
            setOnClickListener { action() }
        }
    }

    private fun row(): LinearLayout {
        return LinearLayout(this).apply {
            orientation = LinearLayout.HORIZONTAL
            gravity = Gravity.CENTER_VERTICAL
        }
    }

    private fun updateDirtyBanner(isDirty: Boolean) {
        if (isDirty) {
            unsavedBanner.text = "Unsaved changes are staged on the receiver"
            unsavedBanner.setBackgroundColor(Color.rgb(154, 83, 42))
        } else {
            unsavedBanner.text = "No unsaved changes reported"
            unsavedBanner.setBackgroundColor(Color.rgb(74, 105, 85))
        }
    }

    private fun appendLog(text: String) {
        val stamp = timeFormat.format(Date())
        logLines.addLast("[$stamp] $text")
        while (logLines.size > 180) {
            logLines.removeFirst()
        }
        if (::logText.isInitialized) {
            logText.text = logLines.joinToString("\n")
        }
    }

    private fun matchWrap() = LinearLayout.LayoutParams(
        ViewGroup.LayoutParams.MATCH_PARENT,
        ViewGroup.LayoutParams.WRAP_CONTENT,
    )

    private fun weightWrap() = LinearLayout.LayoutParams(
        0,
        ViewGroup.LayoutParams.WRAP_CONTENT,
        1f,
    )

    private interface FieldControl {
        val key: String
        val writable: Boolean
        fun setValue(value: String)
        fun getValue(): String
    }

    private class TextControl(
        override val key: String,
        private val textView: TextView,
        override val writable: Boolean,
    ) : FieldControl {
        override fun setValue(value: String) {
            textView.text = value.ifEmpty { "(empty)" }
        }

        override fun getValue(): String = textView.text.toString()
    }

    private class EditControl(
        override val key: String,
        private val editText: EditText,
    ) : FieldControl {
        override val writable = true

        override fun setValue(value: String) {
            if (key == "wifi_password") {
                editText.setText("")
            } else {
                editText.setText(value)
            }
        }

        override fun getValue(): String = editText.text.toString()
    }

    private class SwitchControl(
        override val key: String,
        private val switch: Switch,
    ) : FieldControl {
        override val writable = true

        override fun setValue(value: String) {
            switch.isChecked = value == "1" || value.equals("true", ignoreCase = true)
        }

        override fun getValue(): String = if (switch.isChecked) "1" else "0"
    }

    private class SpinnerControl(
        override val key: String,
        private val spinner: Spinner,
        private val options: List<Pair<String, String>>,
    ) : FieldControl {
        override val writable = true

        override fun setValue(value: String) {
            val index = options.indexOfFirst { it.first == value }.takeIf { it >= 0 } ?: 0
            spinner.setSelection(index)
        }

        override fun getValue(): String {
            return options.getOrNull(spinner.selectedItemPosition)?.first ?: options.first().first
        }
    }

    companion object {
        private const val PERMISSION_REQUEST = 917
    }
}
