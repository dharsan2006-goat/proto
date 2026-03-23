package com.bitchat.android.mesh

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.Build
import android.os.ParcelUuid
import android.util.Log
import com.bitchat.android.protocol.BinaryProtocol
import com.bitchat.android.protocol.BitchatPacket
import com.bitchat.android.model.RoutedPacket
import kotlinx.coroutines.*
import java.util.concurrent.ConcurrentLinkedQueue

/**
 * Manages BLE connections to ESP32 LoRa bridge nodes.
 *
 * This is SEPARATE from the existing BLE mesh logic:
 * - Existing mesh uses SERVICE_UUID F47B5E2D... for phone-to-phone
 * - ESP32 bridges use SERVICE_UUID 12345678... (different)
 *
 * Detection: Scans for ESP32_SERVICE_UUID in BLE advertisements.
 * Fallback: Checks manufacturer data for "ESP32_LORA_NODE".
 *
 * The ESP32 bridge is transparent — it receives bitchat binary packets
 * via BLE WRITE, relays them over LoRa, and pushes LoRa-received packets
 * back via BLE NOTIFY.
 */
@SuppressLint("MissingPermission")
class ESP32BridgeManager(
    private val context: Context,
    private val scope: CoroutineScope
) {

    companion object {
        private const val TAG = "ESP32BridgeManager"
    }

    // Callback for packets received from LoRa via ESP32
    var onPacketReceived: ((BitchatPacket, String) -> Unit)? = null

    // Device tracking
    val deviceTracker = ESP32DeviceTracker()

    // BLE components
    private var bleScanner: BluetoothLeScanner? = null
    private var connectedGatt: BluetoothGatt? = null
    private var writeCharacteristic: BluetoothGattCharacteristic? = null
    private var notifyCharacteristic: BluetoothGattCharacteristic? = null

    // State
    @Volatile
    private var isScanning = false
    @Volatile
    private var isConnecting = false
    @Volatile
    var isActive = false
        private set

    private var scanJob: Job? = null
    private var reconnectAttempts = 0

    // Queue for packets to send when not yet connected
    private val pendingPackets = ConcurrentLinkedQueue<ByteArray>()

    // ===================== PUBLIC API =====================

    /**
     * Start the ESP32 bridge manager — begins scanning for ESP32 devices
     */
    fun start() {
        if (isActive) return
        isActive = true
        reconnectAttempts = 0

        val btManager = context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
        bleScanner = btManager?.adapter?.bluetoothLeScanner

        if (bleScanner == null) {
            Log.e(TAG, "BLE scanner not available")
            return
        }

        Log.i(TAG, "ESP32 Bridge Manager started")
        startPeriodicScan()
    }

    /**
     * Stop the ESP32 bridge manager — disconnects and stops scanning
     */
    fun stop() {
        isActive = false
        scanJob?.cancel()
        scanJob = null
        stopScan()
        disconnectFromESP32()
        Log.i(TAG, "ESP32 Bridge Manager stopped")
    }

    /**
     * Send a bitchat packet to ESP32 for LoRa relay.
     * The packet is already in binary format.
     */
    fun forwardToESP32(packet: BitchatPacket) {
        val encoded = BinaryProtocol.encode(packet) ?: run {
            Log.w(TAG, "Failed to encode packet for ESP32")
            return
        }

        if (writeCharacteristic != null && connectedGatt != null) {
            writeToESP32(encoded)
        } else {
            // Queue for later if not connected
            if (pendingPackets.size < 50) {
                pendingPackets.add(encoded)
            }
        }
    }

    /**
     * Send LoRa configuration to the connected ESP32.
     * The config is sent as a special command packet (first byte = 0xCF).
     * The ESP32 firmware recognizes this and applies the LoRa settings.
     */
    fun sendLoRaConfig(config: LoRaConfig) {
        val configBytes = config.toBytes()
        if (writeCharacteristic != null && connectedGatt != null) {
            writeToESP32(configBytes)
            Log.i(TAG, "Sent LoRa config to ESP32: freq=${config.frequency.label}, SF=${config.spreadingFactor}, BW=${config.bandwidth.label}, TX=${config.txPower}dBm")
        } else {
            Log.w(TAG, "Cannot send config — no ESP32 connected")
        }
    }

    /**
     * Check if an ESP32 bridge is currently connected
     */
    fun isConnected(): Boolean = deviceTracker.hasConnectedBridge()

    /**
     * Get number of discovered ESP32 bridges
     */
    fun getDiscoveredCount(): Int = deviceTracker.getDiscoveredCount()

    // ===================== SCANNING =====================

    /**
     * Start periodic scanning for ESP32 bridges
     */
    private fun startPeriodicScan() {
        scanJob?.cancel()
        scanJob = scope.launch {
            while (isActive) {
                if (!deviceTracker.hasConnectedBridge()) {
                    startScan()
                    delay(ESP32BridgeConstants.SCAN_INTERVAL_MS)
                    stopScan()
                }
                // Clean up stale devices
                deviceTracker.cleanupStaleDevices()
                delay(ESP32BridgeConstants.SCAN_INTERVAL_MS)
            }
        }
    }

    /**
     * Start BLE scan specifically for ESP32 bridge UUID
     */
    private fun startScan() {
        if (isScanning) return

        try {
            val scanFilter = ScanFilter.Builder()
                .setServiceUuid(ParcelUuid(ESP32BridgeConstants.SERVICE_UUID))
                .build()

            val scanSettings = ScanSettings.Builder()
                .setScanMode(ScanSettings.SCAN_MODE_BALANCED)
                .setMatchMode(ScanSettings.MATCH_MODE_AGGRESSIVE)
                .build()

            bleScanner?.startScan(listOf(scanFilter), scanSettings, esp32ScanCallback)
            isScanning = true
            Log.d(TAG, "Started scanning for ESP32 bridges")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start ESP32 scan: ${e.message}")
        }
    }

    /**
     * Stop BLE scan
     */
    private fun stopScan() {
        if (!isScanning) return
        try {
            bleScanner?.stopScan(esp32ScanCallback)
        } catch (_: Exception) { }
        isScanning = false
    }

    /**
     * BLE scan callback — only receives devices advertising ESP32_SERVICE_UUID
     */
    private val esp32ScanCallback = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            handleESP32ScanResult(result)
        }

        override fun onBatchScanResults(results: List<ScanResult>) {
            results.forEach { handleESP32ScanResult(it) }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.w(TAG, "ESP32 scan failed: $errorCode")
            isScanning = false
        }
    }

    /**
     * Process a scan result that matched our ESP32 filter
     */
    private fun handleESP32ScanResult(result: ScanResult) {
        val device = result.device
        val rssi = result.rssi
        val scanRecord = result.scanRecord ?: return

        // Double-check: verify it has our ESP32 service UUID
        val hasESP32Service = scanRecord.serviceUuids?.any {
            it.uuid == ESP32BridgeConstants.SERVICE_UUID
        } == true

        if (!hasESP32Service) {
            // Fallback: check manufacturer data
            val mfgData = scanRecord.manufacturerSpecificData
            var hasManufacturerMatch = false
            if (mfgData != null) {
                for (i in 0 until mfgData.size()) {
                    val data = mfgData.valueAt(i)
                    val str = String(data, Charsets.US_ASCII)
                    if (str.contains(ESP32BridgeConstants.MANUFACTURER_DATA_STRING)) {
                        hasManufacturerMatch = true
                        break
                    }
                }
            }
            if (!hasManufacturerMatch) return
        }

        // Track the ESP32 device
        deviceTracker.onDeviceDiscovered(device, rssi)

        // Auto-connect to the best ESP32 if not already connected
        if (!deviceTracker.hasConnectedBridge() && !isConnecting) {
            connectToBestESP32()
        }
    }

    // ===================== CONNECTION =====================

    /**
     * Connect to the strongest available ESP32 bridge
     */
    private fun connectToBestESP32() {
        val best = deviceTracker.getBestDevice() ?: return

        if (isConnecting) return
        isConnecting = true

        Log.d(TAG, "Connecting to ESP32: ${best.device.address} (RSSI: ${best.rssi})")

        try {
            connectedGatt = best.device.connectGatt(
                context, false, gattCallback, BluetoothDevice.TRANSPORT_LE
            )
        } catch (e: Exception) {
            Log.e(TAG, "Failed to initiate GATT connection: ${e.message}")
            isConnecting = false
        }
    }

    /**
     * Disconnect from current ESP32
     */
    private fun disconnectFromESP32() {
        try {
            connectedGatt?.disconnect()
            connectedGatt?.close()
        } catch (_: Exception) { }
        connectedGatt = null
        writeCharacteristic = null
        notifyCharacteristic = null

        deviceTracker.connectedDeviceAddress?.let {
            deviceTracker.onDeviceDisconnected(it)
        }
    }

    /**
     * GATT callback for ESP32 connection
     */
    private val gattCallback = object : BluetoothGattCallback() {

        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            when (newState) {
                BluetoothProfile.STATE_CONNECTED -> {
                    Log.i(TAG, "Connected to ESP32 GATT: ${gatt.device.address}")
                    isConnecting = false
                    reconnectAttempts = 0
                    deviceTracker.onDeviceConnected(gatt.device.address)
                    // Request higher MTU for larger packets
                    gatt.requestMtu(517)
                }

                BluetoothProfile.STATE_DISCONNECTED -> {
                    Log.w(TAG, "Disconnected from ESP32: ${gatt.device.address}")
                    isConnecting = false
                    val addr = gatt.device.address
                    deviceTracker.onDeviceDisconnected(addr)

                    writeCharacteristic = null
                    notifyCharacteristic = null

                    try {
                        gatt.close()
                    } catch (_: Exception) { }

                    if (connectedGatt == gatt) {
                        connectedGatt = null
                    }

                    // Auto-reconnect
                    if (isActive && reconnectAttempts < ESP32BridgeConstants.MAX_RECONNECT_ATTEMPTS) {
                        reconnectAttempts++
                        scope.launch {
                            delay(ESP32BridgeConstants.RECONNECT_DELAY_MS)
                            if (isActive && !deviceTracker.hasConnectedBridge()) {
                                connectToBestESP32()
                            }
                        }
                    }
                }
            }
        }

        override fun onMtuChanged(gatt: BluetoothGatt, mtu: Int, status: Int) {
            Log.d(TAG, "ESP32 MTU changed to $mtu (status: $status)")
            // Discover services after MTU negotiation
            gatt.discoverServices()
        }

        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) {
                Log.e(TAG, "ESP32 service discovery failed: $status")
                return
            }

            val service = gatt.getService(ESP32BridgeConstants.SERVICE_UUID)
            if (service == null) {
                Log.e(TAG, "ESP32 service not found!")
                return
            }

            // Get WRITE characteristic
            writeCharacteristic = service.getCharacteristic(ESP32BridgeConstants.WRITE_CHARACTERISTIC_UUID)
            if (writeCharacteristic == null) {
                Log.e(TAG, "ESP32 WRITE characteristic not found!")
                return
            }

            // Get NOTIFY characteristic and subscribe
            notifyCharacteristic = service.getCharacteristic(ESP32BridgeConstants.NOTIFY_CHARACTERISTIC_UUID)
            val notifyChar = notifyCharacteristic
            if (notifyChar != null) {
                gatt.setCharacteristicNotification(notifyChar, true)
                // Enable notifications via CCCD descriptor
                val descriptor = notifyChar.getDescriptor(
                    java.util.UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
                )
                if (descriptor != null) {
                    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                        gatt.writeDescriptor(descriptor, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
                    } else {
                        @Suppress("DEPRECATION")
                        descriptor.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                        @Suppress("DEPRECATION")
                        gatt.writeDescriptor(descriptor)
                    }
                }
                Log.i(TAG, "Subscribed to ESP32 NOTIFY characteristic")
            }

            Log.i(TAG, "ESP32 bridge fully connected and ready")

            // Flush pending packets
            scope.launch {
                while (pendingPackets.isNotEmpty()) {
                    val pkt = pendingPackets.poll() ?: break
                    writeToESP32(pkt)
                    delay(50) // Small gap between writes
                }
            }
        }

        @Suppress("DEPRECATION")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic
        ) {
            // Legacy callback for API < 33
            if (characteristic.uuid == ESP32BridgeConstants.NOTIFY_CHARACTERISTIC_UUID) {
                handleLoRaPacketFromESP32(characteristic.value)
            }
        }

        // Modern callback for API 33+
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            if (characteristic.uuid == ESP32BridgeConstants.NOTIFY_CHARACTERISTIC_UUID) {
                handleLoRaPacketFromESP32(value)
            }
        }
    }

    // ===================== DATA TRANSFER =====================

    /**
     * Write a binary packet to ESP32 via BLE GATT WRITE characteristic
     */
    private fun writeToESP32(data: ByteArray) {
        val gatt = connectedGatt ?: return
        val char = writeCharacteristic ?: return

        try {
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
                gatt.writeCharacteristic(
                    char, data, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                )
            } else {
                @Suppress("DEPRECATION")
                char.value = data
                @Suppress("DEPRECATION")
                char.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                @Suppress("DEPRECATION")
                gatt.writeCharacteristic(char)
            }
            Log.d(TAG, "Sent ${data.size} bytes to ESP32 for LoRa relay")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to write to ESP32: ${e.message}")
        }
    }

    /**
     * Handle a packet received from ESP32 (originally from LoRa).
     * Decode it as a bitchat binary packet and inject into the mesh pipeline.
     */
    private fun handleLoRaPacketFromESP32(data: ByteArray) {
        if (data.isEmpty()) return

        Log.d(TAG, "Received ${data.size} bytes from ESP32 (LoRa)")

        try {
            val packet = BinaryProtocol.decode(data)
            if (packet == null) {
                Log.w(TAG, "Failed to decode LoRa packet")
                return
            }

            // Extract peer ID from sender
            val peerID = packet.senderID
                .take(8)
                .joinToString("") { "%02x".format(it.toInt() and 0xFF) }

            // Inject into mesh processing pipeline
            onPacketReceived?.invoke(packet, peerID)

            Log.d(TAG, "Injected LoRa packet into mesh pipeline from peer $peerID")
        } catch (e: Exception) {
            Log.e(TAG, "Error processing LoRa packet: ${e.message}")
        }
    }
}
