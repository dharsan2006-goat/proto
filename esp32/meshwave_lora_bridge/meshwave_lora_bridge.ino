/*
 * MeshWave LoRa Bridge — ESP32 Firmware
 *
 * Hardware: ESP32 Dev Module + SX1278 LoRa (433 MHz)
 * Purpose: Transparent bridge between BLE mesh and LoRa network
 *
 * Flow:
 *   Phone --BLE GATT--> ESP32 --LoRa 433MHz--> ESP32 --BLE GATT--> Phone
 *
 * Wiring (SX1278 to ESP32):
 *   SS/CS  -> GPIO 5
 *   RST    -> GPIO 14
 *   DIO0   -> GPIO 2
 *   SCK    -> GPIO 18
 *   MISO   -> GPIO 19
 *   MOSI   -> GPIO 23
 *   VCC    -> 3.3V
 *   GND    -> GND
 *
 * Board: ESP32 Dev Module (Arduino IDE)
 * Required Libraries:
 *   - LoRa by Sandeep Mistry (install via Library Manager)
 *   - ArduinoJson by Benoit Blanchon (install via Library Manager)
 *   - ESP32 BLE Arduino (bundled with ESP32 board package)
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <SPI.h>
#include <LoRa.h>

// ===================== PIN CONFIGURATION =====================
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2
#define LORA_SCK   18
#define LORA_MISO  19
#define LORA_MOSI  23

// ===================== LORA PARAMETERS =====================
#define LORA_FREQUENCY    433E6    // 433 MHz
#define LORA_SF           7        // Spreading Factor 7 (fastest)
#define LORA_BANDWIDTH    125E3    // 125 kHz
#define LORA_CODING_RATE  5        // 4/5
#define LORA_SYNC_WORD    0x12     // Private sync word
#define LORA_TX_POWER     20       // Max transmit power (dBm)
#define LORA_PREAMBLE     8        // Preamble length

// ===================== BLE UUIDs =====================
// ESP32 bridge uses DIFFERENT UUID than mesh phones
// Phones use: F47B5E2D-4A9E-4C5A-9B3F-8E1D2C3A4B5C
// ESP32 uses: 12345678-1234-1234-1234-1234567890AB
#define ESP32_SERVICE_UUID          "12345678-1234-1234-1234-1234567890ab"
#define WRITE_CHARACTERISTIC_UUID   "12345678-1234-1234-1234-1234567890cd"
#define NOTIFY_CHARACTERISTIC_UUID  "12345678-1234-1234-1234-1234567890ef"

// ===================== PROTOCOL CONSTANTS =====================
#define MAX_BLE_PACKET_SIZE  512    // Max packet from phone
#define MAX_LORA_PACKET_SIZE 255    // LoRa max payload
#define LORA_FRAGMENT_HEADER 5      // 4 bytes ID + 1 byte index/total
#define MAX_LORA_DATA        (MAX_LORA_PACKET_SIZE - LORA_FRAGMENT_HEADER)
#define DEDUP_CACHE_SIZE     200    // Circular buffer for dedup
#define DEDUP_EXPIRY_MS      300000 // 5 minutes
#define TTL_OFFSET           2      // Byte offset for TTL in bitchat binary header

// ===================== MANUFACTURER DATA =====================
static const char* MANUFACTURER_STRING = "ESP32_LORA_NODE";

// ===================== GLOBALS =====================
BLEServer* pServer = NULL;
BLECharacteristic* pWriteCharacteristic = NULL;
BLECharacteristic* pNotifyCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

// Deduplication cache
struct DedupEntry {
  uint32_t hash;
  unsigned long timestamp;
  bool valid;
};
DedupEntry dedupCache[DEDUP_CACHE_SIZE];
int dedupIndex = 0;

// LoRa fragment reassembly
struct LoRaFragment {
  uint32_t fragmentId;
  uint8_t totalFragments;
  uint8_t receivedCount;
  uint8_t* fragments[16];    // Max 16 fragments
  uint8_t fragmentSizes[16];
  unsigned long timestamp;
  bool active;
};
#define MAX_PENDING_REASSEMBLY 8
LoRaFragment pendingReassembly[MAX_PENDING_REASSEMBLY];

// Stats
unsigned long bleReceived = 0;
unsigned long loraSent = 0;
unsigned long loraReceived = 0;
unsigned long bleSent = 0;
unsigned long duplicatesDropped = 0;

// ===================== UTILITY FUNCTIONS =====================

/**
 * Compute CRC32 hash for deduplication
 */
uint32_t computeHash(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      crc = (crc >> 1) ^ (0xEDB88320 & (-(crc & 1)));
    }
  }
  return crc ^ 0xFFFFFFFF;
}

/**
 * Check if packet is a duplicate. Returns true if duplicate (should drop).
 */
bool isDuplicate(const uint8_t* data, size_t len) {
  uint32_t hash = computeHash(data, len);
  unsigned long now = millis();

  // Check existing entries
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    if (dedupCache[i].valid && dedupCache[i].hash == hash) {
      if (now - dedupCache[i].timestamp < DEDUP_EXPIRY_MS) {
        return true;  // Duplicate found
      } else {
        // Expired entry with same hash — overwrite
        dedupCache[i].timestamp = now;
        return false;
      }
    }
  }

  // Not found — add to cache
  dedupCache[dedupIndex].hash = hash;
  dedupCache[dedupIndex].timestamp = now;
  dedupCache[dedupIndex].valid = true;
  dedupIndex = (dedupIndex + 1) % DEDUP_CACHE_SIZE;

  return false;
}

/**
 * Decrement TTL in bitchat binary packet.
 * TTL is at byte offset 2 in the header.
 * Returns false if TTL is already 0 (should drop).
 */
bool decrementTTL(uint8_t* data, size_t len) {
  if (len < 3) return false;  // Too short to have TTL

  uint8_t ttl = data[TTL_OFFSET];
  if (ttl == 0) return false;  // Already expired

  data[TTL_OFFSET] = ttl - 1;
  return true;
}

/**
 * Clean up expired dedup entries periodically
 */
void cleanupDedupCache() {
  unsigned long now = millis();
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    if (dedupCache[i].valid && (now - dedupCache[i].timestamp > DEDUP_EXPIRY_MS)) {
      dedupCache[i].valid = false;
    }
  }
}

// ===================== LORA FRAGMENTATION =====================

/**
 * Generate a random 4-byte fragment ID
 */
uint32_t generateFragmentId() {
  return (uint32_t)esp_random();
}

/**
 * Send data via LoRa, fragmenting if necessary.
 * Fragment header: [4 bytes ID][1 byte: high nibble=index, low nibble=total]
 */
void sendViaLoRa(const uint8_t* data, size_t len) {
  if (len <= MAX_LORA_PACKET_SIZE) {
    // Fits in one LoRa packet — send directly
    LoRa.beginPacket();
    LoRa.write(data, len);
    LoRa.endPacket();
    loraSent++;
    Serial.printf("[LoRa TX] %d bytes (no fragmentation)\n", len);
    return;
  }

  // Need fragmentation
  uint32_t fragId = generateFragmentId();
  int totalFragments = (len + MAX_LORA_DATA - 1) / MAX_LORA_DATA;

  if (totalFragments > 15) {
    Serial.printf("[LoRa TX] ERROR: Packet too large (%d bytes, %d fragments)\n", len, totalFragments);
    return;
  }

  for (int i = 0; i < totalFragments; i++) {
    size_t offset = i * MAX_LORA_DATA;
    size_t chunkSize = min((size_t)MAX_LORA_DATA, len - offset);

    uint8_t header[LORA_FRAGMENT_HEADER];
    header[0] = (fragId >> 24) & 0xFF;
    header[1] = (fragId >> 16) & 0xFF;
    header[2] = (fragId >> 8) & 0xFF;
    header[3] = fragId & 0xFF;
    header[4] = ((i & 0x0F) << 4) | (totalFragments & 0x0F);

    LoRa.beginPacket();
    LoRa.write(header, LORA_FRAGMENT_HEADER);
    LoRa.write(data + offset, chunkSize);
    LoRa.endPacket();

    loraSent++;
    Serial.printf("[LoRa TX] Fragment %d/%d (%d bytes)\n", i + 1, totalFragments, chunkSize);
    delay(50);  // Small delay between fragments
  }
}

/**
 * Initialize reassembly slots
 */
void initReassembly() {
  for (int i = 0; i < MAX_PENDING_REASSEMBLY; i++) {
    pendingReassembly[i].active = false;
    for (int j = 0; j < 16; j++) {
      pendingReassembly[i].fragments[j] = NULL;
    }
  }
}

/**
 * Free a reassembly slot
 */
void freeReassemblySlot(int slot) {
  for (int j = 0; j < 16; j++) {
    if (pendingReassembly[slot].fragments[j] != NULL) {
      free(pendingReassembly[slot].fragments[j]);
      pendingReassembly[slot].fragments[j] = NULL;
    }
  }
  pendingReassembly[slot].active = false;
}

/**
 * Handle a received LoRa fragment. Returns assembled data if complete, NULL otherwise.
 * Caller must free() the returned buffer.
 */
uint8_t* handleLoRaFragment(const uint8_t* data, size_t len, size_t* outLen) {
  if (len < LORA_FRAGMENT_HEADER) return NULL;

  uint32_t fragId = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                    ((uint32_t)data[2] << 8) | data[3];
  uint8_t index = (data[4] >> 4) & 0x0F;
  uint8_t total = data[4] & 0x0F;

  if (index >= total || total > 15) return NULL;

  const uint8_t* payload = data + LORA_FRAGMENT_HEADER;
  size_t payloadLen = len - LORA_FRAGMENT_HEADER;

  // Find existing slot or allocate new one
  int slot = -1;
  for (int i = 0; i < MAX_PENDING_REASSEMBLY; i++) {
    if (pendingReassembly[i].active && pendingReassembly[i].fragmentId == fragId) {
      slot = i;
      break;
    }
  }

  if (slot == -1) {
    // Find free slot (or oldest)
    unsigned long oldest = ULONG_MAX;
    int oldestSlot = 0;
    for (int i = 0; i < MAX_PENDING_REASSEMBLY; i++) {
      if (!pendingReassembly[i].active) {
        slot = i;
        break;
      }
      if (pendingReassembly[i].timestamp < oldest) {
        oldest = pendingReassembly[i].timestamp;
        oldestSlot = i;
      }
    }
    if (slot == -1) {
      // Evict oldest
      freeReassemblySlot(oldestSlot);
      slot = oldestSlot;
    }

    pendingReassembly[slot].active = true;
    pendingReassembly[slot].fragmentId = fragId;
    pendingReassembly[slot].totalFragments = total;
    pendingReassembly[slot].receivedCount = 0;
    pendingReassembly[slot].timestamp = millis();
  }

  LoRaFragment& frag = pendingReassembly[slot];

  // Store fragment
  if (frag.fragments[index] == NULL) {
    frag.fragments[index] = (uint8_t*)malloc(payloadLen);
    if (frag.fragments[index] == NULL) return NULL;
    memcpy(frag.fragments[index], payload, payloadLen);
    frag.fragmentSizes[index] = payloadLen;
    frag.receivedCount++;
  }

  // Check if complete
  if (frag.receivedCount >= frag.totalFragments) {
    // Calculate total size
    size_t totalSize = 0;
    for (int i = 0; i < frag.totalFragments; i++) {
      if (frag.fragments[i] == NULL) {
        freeReassemblySlot(slot);
        return NULL;  // Missing fragment
      }
      totalSize += frag.fragmentSizes[i];
    }

    // Assemble
    uint8_t* assembled = (uint8_t*)malloc(totalSize);
    if (assembled == NULL) {
      freeReassemblySlot(slot);
      return NULL;
    }

    size_t pos = 0;
    for (int i = 0; i < frag.totalFragments; i++) {
      memcpy(assembled + pos, frag.fragments[i], frag.fragmentSizes[i]);
      pos += frag.fragmentSizes[i];
    }

    *outLen = totalSize;
    freeReassemblySlot(slot);

    Serial.printf("[LoRa RX] Reassembled %d fragments -> %d bytes\n", total, totalSize);
    return assembled;
  }

  return NULL;  // Still waiting for more fragments
}

/**
 * Clean up expired reassembly slots (older than 30 seconds)
 */
void cleanupReassembly() {
  unsigned long now = millis();
  for (int i = 0; i < MAX_PENDING_REASSEMBLY; i++) {
    if (pendingReassembly[i].active && (now - pendingReassembly[i].timestamp > 30000)) {
      Serial.printf("[LoRa] Reassembly timeout for fragment ID 0x%08X\n", pendingReassembly[i].fragmentId);
      freeReassemblySlot(i);
    }
  }
}

// ===================== LORA CONFIG FROM PHONE =====================

// Bandwidth lookup table (matches LoRaBandwidth enum codes in Android app)
static const double bwTable[] = {
  7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
  41.7E3, 62.5E3, 125E3, 250E3, 500E3
};

/**
 * Apply LoRa configuration received from the phone via BLE.
 * Config packet format: [0xCF][freq 4B][SF 1B][BW 1B][CR 1B][TXP 1B][SW 1B][CRC 1B]
 */
void applyLoRaConfig(const uint8_t* data, size_t len) {
  if (len != 11 || data[0] != 0xCF) return;

  // Parse frequency (4 bytes, big-endian)
  uint32_t freqHz = ((uint32_t)data[1] << 24) | ((uint32_t)data[2] << 16) |
                    ((uint32_t)data[3] << 8) | data[4];
  uint8_t sf      = data[5];
  uint8_t bwCode  = data[6];
  uint8_t cr      = data[7];
  uint8_t txPow   = data[8];
  uint8_t sw      = data[9];
  bool    crcOn   = data[10] != 0;

  // Validate ranges
  if (sf < 6 || sf > 12) sf = 7;
  if (bwCode > 9) bwCode = 7;
  if (cr < 5 || cr > 8) cr = 5;
  if (txPow < 2 || txPow > 20) txPow = 20;

  Serial.println("\n[CONFIG] Applying new LoRa settings from phone:");
  Serial.printf("  Frequency:  %lu Hz (%.1f MHz)\n", freqHz, freqHz / 1E6);
  Serial.printf("  SF:         %d\n", sf);
  Serial.printf("  Bandwidth:  %.1f kHz (code %d)\n", bwTable[bwCode] / 1E3, bwCode);
  Serial.printf("  Coding Rate: 4/%d\n", cr);
  Serial.printf("  TX Power:   %d dBm\n", txPow);
  Serial.printf("  Sync Word:  0x%02X\n", sw);
  Serial.printf("  CRC:        %s\n", crcOn ? "ON" : "OFF");

  // Apply to LoRa module
  LoRa.sleep();  // Must sleep before changing frequency

  LoRa.setFrequency(freqHz);
  LoRa.setSpreadingFactor(sf);
  LoRa.setSignalBandwidth((long)bwTable[bwCode]);
  LoRa.setCodingRate4(cr);
  LoRa.setTxPower(txPow);
  LoRa.setSyncWord(sw);

  if (crcOn) {
    LoRa.enableCrc();
  } else {
    LoRa.disableCrc();
  }

  LoRa.idle();  // Return to idle (ready to receive)

  Serial.println("[CONFIG] LoRa settings applied successfully!\n");
}

// ===================== BLE CALLBACKS =====================

/**
 * Server connection callbacks
 */
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    Serial.println("[BLE] Phone connected");
    // Allow multiple connections
    BLEDevice::startAdvertising();
  }

  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
    Serial.println("[BLE] Phone disconnected");
    // Restart advertising
    delay(500);
    BLEDevice::startAdvertising();
  }
};

/**
 * Write characteristic callback — Phone sends packet to ESP32
 */
class WriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    String value = pCharacteristic->getValue();
    size_t len = value.length();

    if (len < 3) {
      Serial.printf("[BLE RX] Packet too short (%d bytes), ignoring\n", len);
      return;
    }

    // Check if this is a config command (first byte = 0xCF)
    if ((uint8_t)value[0] == 0xCF && len == 11) {
      applyLoRaConfig((const uint8_t*)value.c_str(), len);
      return;
    }

    bleReceived++;
    Serial.printf("[BLE RX] %d bytes from phone\n", len);

    // Copy to mutable buffer
    uint8_t* packet = (uint8_t*)malloc(len);
    if (packet == NULL) return;
    memcpy(packet, value.c_str(), len);

    // Check duplicate
    if (isDuplicate(packet, len)) {
      Serial.println("[BLE RX] Duplicate packet, dropping");
      duplicatesDropped++;
      free(packet);
      return;
    }

    // Decrement TTL
    if (!decrementTTL(packet, len)) {
      Serial.println("[BLE RX] TTL expired, dropping");
      free(packet);
      return;
    }

    // Forward via LoRa
    sendViaLoRa(packet, len);
    free(packet);
  }
};

// ===================== SETUP =====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n========================================");
  Serial.println("  MeshWave LoRa Bridge v1.0");
  Serial.println("  ESP32 + SX1278 (433 MHz)");
  Serial.println("========================================\n");

  // Initialize dedup cache
  for (int i = 0; i < DEDUP_CACHE_SIZE; i++) {
    dedupCache[i].valid = false;
  }
  initReassembly();

  // ---- Initialize LoRa ----
  Serial.println("[LoRa] Initializing...");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQUENCY)) {
    Serial.println("[LoRa] ERROR: Init failed! Check wiring.");
    while (1) {
      delay(1000);  // Halt
    }
  }

  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BANDWIDTH);
  LoRa.setCodingRate4(LORA_CODING_RATE);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.setTxPower(LORA_TX_POWER);
  LoRa.setPreambleLength(LORA_PREAMBLE);
  LoRa.enableCrc();

  Serial.println("[LoRa] Ready — 433 MHz, SF7, BW125, CR4/5");

  // ---- Initialize BLE ----
  Serial.println("[BLE] Initializing...");
  BLEDevice::init("MeshWave-LoRa");

  // Create GATT server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // Create service with ESP32-specific UUID
  BLEService* pService = pServer->createService(ESP32_SERVICE_UUID);

  // WRITE characteristic — Phone writes packets here
  pWriteCharacteristic = pService->createCharacteristic(
    WRITE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  pWriteCharacteristic->setCallbacks(new WriteCallbacks());
  pWriteCharacteristic->addDescriptor(new BLE2902());

  // NOTIFY characteristic — ESP32 pushes LoRa-received packets to phone
  pNotifyCharacteristic = pService->createCharacteristic(
    NOTIFY_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
  );
  pNotifyCharacteristic->addDescriptor(new BLE2902());

  // Start service
  pService->start();

  // Configure advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(ESP32_SERVICE_UUID);

  // Add manufacturer data: "ESP32_LORA_NODE"
  BLEAdvertisementData advData;
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advData.setCompleteServices(BLEUUID(ESP32_SERVICE_UUID));

  // Manufacturer data: company ID (0xFFFF = test) + "ESP32_LORA_NODE"
  String mfgData;
  mfgData += (char)0xFF;  // Company ID low byte
  mfgData += (char)0xFF;  // Company ID high byte
  const char* mfgStr = MANUFACTURER_STRING;
  for (int i = 0; mfgStr[i] != '\0'; i++) {
    mfgData += mfgStr[i];
  }
  advData.setManufacturerData(mfgData);
  pAdvertising->setAdvertisementData(advData);

  // Scan response with device name
  BLEAdvertisementData scanRsp;
  scanRsp.setName("MeshWave-LoRa");
  pAdvertising->setScanResponseData(scanRsp);

  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising as 'MeshWave-LoRa'");
  Serial.printf("[BLE] Service UUID: %s\n", ESP32_SERVICE_UUID);

  Serial.println("\n[READY] Bridge active — waiting for connections...\n");
}

// ===================== MAIN LOOP =====================

unsigned long lastCleanup = 0;
unsigned long lastStats = 0;

void loop() {
  unsigned long now = millis();

  // ---- Check for incoming LoRa packets ----
  int packetSize = LoRa.parsePacket();
  if (packetSize > 0) {
    uint8_t buffer[MAX_LORA_PACKET_SIZE];
    int bytesRead = 0;

    while (LoRa.available() && bytesRead < MAX_LORA_PACKET_SIZE) {
      buffer[bytesRead++] = LoRa.read();
    }

    loraReceived++;
    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();
    Serial.printf("[LoRa RX] %d bytes (RSSI: %d, SNR: %.1f)\n", bytesRead, rssi, snr);

    // Determine if this is a fragment or complete packet
    uint8_t* finalPacket = NULL;
    size_t finalLen = 0;
    bool needsFree = false;

    // If packet is larger than header minimum and could be a fragment
    // We detect fragments by checking if the total size is exactly
    // LORA_FRAGMENT_HEADER + data, and the nibble encoding makes sense
    if (bytesRead > LORA_FRAGMENT_HEADER) {
      uint8_t nibbles = buffer[4];
      uint8_t idx = (nibbles >> 4) & 0x0F;
      uint8_t tot = nibbles & 0x0F;

      // Heuristic: if this looks like a fragment header (total > 1, index < total)
      if (tot > 1 && idx < tot && tot <= 15) {
        finalPacket = handleLoRaFragment(buffer, bytesRead, &finalLen);
        needsFree = true;
        if (finalPacket == NULL) {
          // Still waiting for more fragments
          return;
        }
      }
    }

    // Not a fragment (or single-packet message)
    if (finalPacket == NULL) {
      finalPacket = buffer;
      finalLen = bytesRead;
      needsFree = false;
    }

    // Check duplicate
    if (isDuplicate(finalPacket, finalLen)) {
      Serial.println("[LoRa RX] Duplicate packet, dropping");
      duplicatesDropped++;
      if (needsFree) free(finalPacket);
      return;
    }

    // Decrement TTL (work on a copy if needed)
    uint8_t* mutablePacket;
    if (needsFree) {
      mutablePacket = finalPacket;
    } else {
      mutablePacket = (uint8_t*)malloc(finalLen);
      if (mutablePacket == NULL) return;
      memcpy(mutablePacket, finalPacket, finalLen);
      needsFree = true;
    }

    if (!decrementTTL(mutablePacket, finalLen)) {
      Serial.println("[LoRa RX] TTL expired, dropping");
      if (needsFree) free(mutablePacket);
      return;
    }

    // Forward to connected phone via BLE NOTIFY
    if (deviceConnected && pNotifyCharacteristic != NULL) {
      // BLE MTU may limit single notification — split if needed
      size_t sent = 0;
      while (sent < finalLen) {
        size_t chunk = min((size_t)500, finalLen - sent);  // Conservative BLE chunk
        pNotifyCharacteristic->setValue(mutablePacket + sent, chunk);
        pNotifyCharacteristic->notify();
        sent += chunk;
        if (sent < finalLen) delay(20);  // Small gap between chunks
      }
      bleSent++;
      Serial.printf("[BLE TX] Forwarded %d bytes to phone\n", finalLen);
    } else {
      Serial.println("[BLE TX] No phone connected, packet buffered in mesh");
    }

    if (needsFree) free(mutablePacket);
  }

  // ---- Handle BLE connection state changes ----
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("[BLE] Restarting advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
  }

  // ---- Periodic cleanup (every 60 seconds) ----
  if (now - lastCleanup > 60000) {
    cleanupDedupCache();
    cleanupReassembly();
    lastCleanup = now;
  }

  // ---- Stats output (every 30 seconds) ----
  if (now - lastStats > 30000) {
    Serial.println("--- MeshWave Bridge Stats ---");
    Serial.printf("  BLE received:  %lu\n", bleReceived);
    Serial.printf("  LoRa sent:     %lu\n", loraSent);
    Serial.printf("  LoRa received: %lu\n", loraReceived);
    Serial.printf("  BLE sent:      %lu\n", bleSent);
    Serial.printf("  Duplicates:    %lu\n", duplicatesDropped);
    Serial.printf("  Connected:     %s\n", deviceConnected ? "YES" : "NO");
    Serial.printf("  Free heap:     %d bytes\n", ESP.getFreeHeap());
    Serial.println("-----------------------------");
    lastStats = now;
  }
}
