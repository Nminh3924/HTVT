#include <LoRa.h>
#include <SPI.h>

// ========== CONFIGURATION ==========
// LoRa SX1278 Ra-02 Pins
#define LORA_SCK 18
#define LORA_MISO 19
#define LORA_MOSI 23
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define LORA_BAND 433E6 // 433MHz

// Test mode - để giả lập mất gói
#define TEST_MODE true
#define DROP_RATE 15

// ========== BIT ERROR SIMULATION - Test Hamming(7,4) FEC ==========
// BITS_TO_FLIP = 1 → Hamming SỬA ĐƯỢC lỗi
// BITS_TO_FLIP = 2 → Hamming KHÔNG SỬA ĐƯỢC → CRC fail → Kalman predict
// MIXED_ERROR_MODE = true → Ngẫu nhiên 1 hoặc 2 bit (demo cả 2 trường hợp)
#define BIT_ERROR_MODE true // Bật/tắt chế độ nhiễu bit
#define BIT_ERROR_RATE 40   // % packets bị thêm nhiễu (0-100)
#define BITS_TO_FLIP                                                           \
  1 // Số bit lỗi mặc định (dùng khi MIXED_ERROR_MODE = false)
#define MIXED_ERROR_MODE                                                       \
  true // true = ngẫu nhiên 1-2 bit, false = cố định BITS_TO_FLIP
#define TWO_BIT_ERROR_CHANCE                                                   \
  50 // % packets bị 2 bit lỗi (khi MIXED_ERROR_MODE = true)

// ========== DATA STRUCTURES ==========
struct __attribute__((packed)) Packet {
  uint16_t seqNum;
  uint32_t timestamp;
  float temperature;
  float humidity;
  uint16_t crc;
  // Total: 16 bytes (packed, no padding)
};

// ========== KALMAN FILTER ==========
class SimpleKalmanFilter {
private:
  float Q; // Process noise
  float R; // Measurement noise
  float P; // Estimation error
  float K; // Kalman gain
  float X; // Estimated value

public:
  SimpleKalmanFilter(float processNoise, float measurementNoise,
                     float estimationError, float initialValue) {
    Q = processNoise;
    R = measurementNoise;
    P = estimationError;
    X = initialValue;
  }

  float update(float measurement) {
    // Prediction
    P = P + Q;

    // Update
    K = P / (P + R);
    X = X + K * (measurement - X);
    P = (1 - K) * P;

    return X;
  }

  float predict() {
    // Prediction only (no measurement update)
    P = P + Q;
    return X;
  }

  float getEstimate() { return X; }
};

// Kalman filters for prediction when packets are lost
SimpleKalmanFilter kfTemp(0.05, 0.5, 1, 25.0);
SimpleKalmanFilter kfHumidity(0.05, 2.0, 1, 50.0);

// ========== HAMMING(7,4) FEC DECODER ==========
// Decode 7-bit codeword back to 4-bit nibble with error correction
uint8_t hammingDecode(uint8_t codeword, bool *errorDetected,
                      bool *errorCorrected) {
  // Extract bits
  uint8_t p1 = (codeword >> 6) & 1;
  uint8_t p2 = (codeword >> 5) & 1;
  uint8_t d1 = (codeword >> 4) & 1;
  uint8_t p3 = (codeword >> 3) & 1;
  uint8_t d2 = (codeword >> 2) & 1;
  uint8_t d3 = (codeword >> 1) & 1;
  uint8_t d4 = (codeword >> 0) & 1;

  // Calculate syndrome
  uint8_t c1 = p1 ^ d1 ^ d2 ^ d4;
  uint8_t c2 = p2 ^ d1 ^ d3 ^ d4;
  uint8_t c3 = p3 ^ d2 ^ d3 ^ d4;

  uint8_t syndrome = (c3 << 2) | (c2 << 1) | c1;

  *errorDetected = (syndrome != 0);
  *errorCorrected = false;

  if (syndrome != 0) {
    // Error at position 'syndrome'
    if (syndrome <= 7) {
      codeword ^= (1 << (7 - syndrome)); // Flip bit
      *errorCorrected = true;

      // Re-extract after correction
      d1 = (codeword >> 4) & 1;
      d2 = (codeword >> 2) & 1;
      d3 = (codeword >> 1) & 1;
      d4 = (codeword >> 0) & 1;
    }
  }

  // Reconstruct nibble
  uint8_t nibble = (d1 << 3) | (d2 << 2) | (d3 << 1) | d4;
  return nibble;
}

// Decode 2 codewords back to 1 byte
uint8_t hammingDecodeByte(uint8_t *encoded, bool *errorDetected,
                          bool *errorCorrected) {
  bool err1, corr1, err2, corr2;

  uint8_t highNibble = hammingDecode(encoded[0], &err1, &corr1);
  uint8_t lowNibble = hammingDecode(encoded[1], &err2, &corr2);

  *errorDetected = err1 || err2;
  *errorCorrected = corr1 || corr2;

  return (highNibble << 4) | lowNibble;
}

// ========== CRC CALCULATION ==========
uint16_t calculateCRC(uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) {
        crc = (crc >> 1) ^ 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

// ========== PACKET DECODING WITH FEC ==========
bool decodePacketWithFEC(uint8_t *encoded, size_t encodedLen, Packet *pkt,
                         int *errorsDetected, int *errorsCorrected) {
  if (encodedLen != 32) { // Expected: 16 bytes * 2 = 32 bytes
    return false;
  }

  uint8_t *rawData = (uint8_t *)pkt;
  *errorsDetected = 0;
  *errorsCorrected = 0;

  // Decode each byte
  for (size_t i = 0; i < sizeof(Packet); i++) {
    bool errorDet, errorCorr;
    rawData[i] = hammingDecodeByte(&encoded[i * 2], &errorDet, &errorCorr);

    if (errorDet)
      (*errorsDetected)++;
    if (errorCorr)
      (*errorsCorrected)++;
  }

  // Verify CRC
  uint16_t calculatedCRC = calculateCRC(rawData, sizeof(Packet) - 2);

  if (calculatedCRC != pkt->crc) {
    return false; // CRC mismatch - uncorrectable error
  }

  return true;
}

// ========== GLOBAL VARIABLES ==========
uint16_t lastSeqNum = 0;
bool firstPacket = true;

// Statistics
uint32_t totalPacketsReceived = 0;
uint32_t packetsLost = 0;
uint32_t totalErrorsDetected = 0;
uint32_t totalErrorsCorrected = 0;
uint32_t uncorrectableErrors = 0;
uint32_t packetsPredicted = 0;
float totalRSSI = 0;
float minRSSI = 0;
float maxRSSI = -200;

// Frequency Error statistics
long totalFreqError = 0;
long minFreqError = 999999;
long maxFreqError = -999999;

// Bit error injection statistics
uint32_t totalBitsInjected = 0;
uint32_t packetsWithInjectedErrors = 0;
uint32_t packetsWithOneBitError = 0; // 1-bit errors (Hamming should correct)
uint32_t packetsWithTwoBitError = 0; // 2-bit errors (Hamming cannot correct)
uint32_t oneBitErrorsCorrected = 0;  // Successfully corrected 1-bit errors

// For Kalman prediction
float lastTemp = 25.0;
float lastHum = 50.0;

// ========== FORWARD DECLARATIONS ==========
void printStatistics();
void handleLostPackets(uint16_t currentSeq);
void handleCorruptedPacket(int rssi,
                           float snr); // NEW: Handle CRC failed packets

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("ESP32 RECEIVER");
  Serial.println("FEC Decoder + Kalman Predictor + LoRa");
  Serial.println("========================================\n");

  // Initialize LoRa
  Serial.print("Initializing LoRa SX1278... ");
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_BAND)) {
    Serial.println("FAILED!");
    Serial.println("Check wiring and module!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("OK");

  // Configure LoRa (must match transmitter)
  Serial.println("\nConfiguring LoRa parameters:");
  LoRa.setSpreadingFactor(11);
  Serial.println("  Spreading Factor: 11");

  LoRa.setSignalBandwidth(125E3);
  Serial.println("  Bandwidth: 125 kHz");

  LoRa.setCodingRate4(5);
  Serial.println("  Coding Rate: 4/5");

  LoRa.setSyncWord(0x12);
  Serial.println("  Sync Word: 0x12");

  if (TEST_MODE) {
    Serial.println("\n*** TEST MODE ENABLED ***");
    Serial.print("*** Simulating ");
    Serial.print(DROP_RATE);
    Serial.println("% packet loss ***\n");
  }

  if (BIT_ERROR_MODE) {
    Serial.println("\n*** BIT ERROR MODE ENABLED ***");
    Serial.print("*** Injecting errors in ");
    Serial.print(BIT_ERROR_RATE);
    Serial.println("% of packets ***");
    if (MIXED_ERROR_MODE) {
      Serial.println("*** MIXED MODE: Random 1 or 2 bit errors ***");
      Serial.print("***   - 1-bit errors (Hamming corrects): ");
      Serial.print(100 - TWO_BIT_ERROR_CHANCE);
      Serial.println("% ***");
      Serial.print("***   - 2-bit errors (CRC fail + Kalman): ");
      Serial.print(TWO_BIT_ERROR_CHANCE);
      Serial.println("% ***");
    } else {
      Serial.print("*** Fixed ");
      Serial.print(BITS_TO_FLIP);
      Serial.println(" bit error(s) per packet ***");
    }
    Serial.println("*** This tests Hamming(7,4) FEC + Kalman prediction ***\n");
  }

  Serial.println("\n========================================");
  Serial.println("RECEIVER READY - Waiting for packets...");
  Serial.println("========================================\n");
  Serial.println("Seq | Temp | Hum  | RSSI | ErrDet | ErrCorr | Status");
  Serial.println("----+------+------+------+--------+---------+----------");
}

// ========== MAIN LOOP ==========
void loop() {
  int packetSize = LoRa.parsePacket();

  if (packetSize) {
    // Read encoded packet
    uint8_t encodedData[64];
    int len = 0;

    while (LoRa.available() && len < 64) {
      encodedData[len++] = LoRa.read();
    }

    int rssi = LoRa.packetRssi();
    float snr = LoRa.packetSnr();

    // TEST MODE: Simulate packet loss
    if (TEST_MODE && random(100) < DROP_RATE) {
      Serial.print("*** SIMULATED DROP (seq would be around ");
      Serial.print(lastSeqNum + 1);
      Serial.println(") ***");
      return;
    }

    // NEW: BIT ERROR MODE - Inject artificial bit errors to test Hamming FEC
    int bitsFlipped = 0;
    if (BIT_ERROR_MODE && random(100) < BIT_ERROR_RATE) {
      // Determine how many bits to flip
      int bitsToFlipNow = BITS_TO_FLIP;
      if (MIXED_ERROR_MODE) {
        // Randomly choose 1 or 2 bits based on TWO_BIT_ERROR_CHANCE
        bitsToFlipNow = (random(100) < TWO_BIT_ERROR_CHANCE) ? 2 : 1;
      }

      if (bitsToFlipNow == 1) {
        // 1-bit error: flip any random bit - Hamming can correct this
        int bytePos = random(len);
        int bitPos = random(8);
        encodedData[bytePos] ^= (1 << bitPos);
        bitsFlipped = 1;
      } else {
        // 2-bit error: MUST flip 2 bits in SAME codeword (same byte in encoded
        // data) Each byte in encodedData is a 7-bit Hamming codeword (stored in
        // 8 bits) Hamming(7,4) can only correct 1-bit errors, so 2 bits in same
        // codeword = uncorrectable
        int bytePos = random(len); // Pick a random codeword
        int bitPos1 = random(7);   // First bit position (0-6, only 7 bits used)
        int bitPos2;
        do {
          bitPos2 = random(7); // Second bit position, must be different
        } while (bitPos2 == bitPos1);

        encodedData[bytePos] ^= (1 << bitPos1); // Flip first bit
        encodedData[bytePos] ^= (1 << bitPos2); // Flip second bit in SAME byte
        bitsFlipped = 2;
      }

      // Update injection statistics
      totalBitsInjected += bitsFlipped;
      packetsWithInjectedErrors++;
      if (bitsFlipped == 1) {
        packetsWithOneBitError++;
      } else {
        packetsWithTwoBitError++;
      }

      Serial.print("*** INJECTED ");
      Serial.print(bitsFlipped);
      if (bitsFlipped == 1) {
        Serial.println(" BIT ERROR - Hamming should CORRECT ***");
      } else {
        Serial.println(" BIT ERRORS in SAME CODEWORD - CRC will FAIL, Kalman "
                       "will PREDICT ***");
      }
    }

    // Decode packet with FEC
    Packet pkt;
    int errorsDetected = 0;
    int errorsCorrected = 0;
    bool decodeSuccess = decodePacketWithFEC(encodedData, len, &pkt,
                                             &errorsDetected, &errorsCorrected);

    if (!decodeSuccess) {
      // FEC failed to correct errors - use Kalman prediction instead of
      // dropping
      uncorrectableErrors++;
      handleCorruptedPacket(rssi, snr); // Use Kalman to predict values
      return;
    }

    // Track successful 1-bit error correction
    if (bitsFlipped == 1 && errorsCorrected > 0) {
      oneBitErrorsCorrected++;
    }

    // Update statistics
    totalPacketsReceived++;
    totalErrorsDetected += errorsDetected;
    totalErrorsCorrected += errorsCorrected;
    totalRSSI += rssi;
    if (rssi < minRSSI || minRSSI == 0)
      minRSSI = rssi;
    if (rssi > maxRSSI)
      maxRSSI = rssi;

    // Update Frequency Error statistics
    long freqError = LoRa.packetFrequencyError();
    totalFreqError += freqError;
    if (freqError < minFreqError)
      minFreqError = freqError;
    if (freqError > maxFreqError)
      maxFreqError = freqError;

    // Check for lost packets
    if (!firstPacket) {
      if (pkt.seqNum != lastSeqNum + 1) {
        handleLostPackets(pkt.seqNum);
      }
    } else {
      firstPacket = false;
    }

    lastSeqNum = pkt.seqNum;

    // Update Kalman filter with received data
    kfTemp.update(pkt.temperature);
    kfHumidity.update(pkt.humidity);
    lastTemp = pkt.temperature;
    lastHum = pkt.humidity;

    // Print packet data
    Serial.print(pkt.seqNum);
    Serial.print(" | ");
    Serial.print(pkt.temperature, 1);
    Serial.print(" | ");
    Serial.print(pkt.humidity, 1);
    Serial.print(" | ");
    Serial.print(rssi);
    Serial.print(" | ");
    Serial.print(errorsDetected);
    Serial.print("      | ");
    Serial.print(errorsCorrected);
    Serial.print("       | ");

    if (errorsCorrected > 0) {
      Serial.print("CORRECTED");
    } else if (errorsDetected > 0) {
      Serial.print("ERR_DET");
    } else {
      Serial.print("OK");
    }
    Serial.println();

    // Print statistics every 50 packets
    if (totalPacketsReceived % 50 == 0) {
      printStatistics();
    }
  }
}

// ========== HANDLE LOST PACKETS ==========
void handleLostPackets(uint16_t currentSeq) {
  int lostCount = currentSeq - lastSeqNum - 1;
  packetsLost += lostCount;

  Serial.println("----+------+------+------+--------+---------+----------");
  Serial.print("!!! LOST ");
  Serial.print(lostCount);
  Serial.print(" PACKET");
  if (lostCount > 1)
    Serial.print("S");
  Serial.print(" (seq ");
  Serial.print(lastSeqNum + 1);
  if (lostCount > 1) {
    Serial.print(" to ");
    Serial.print(currentSeq - 1);
  }
  Serial.println(") !!!");

  // Use Kalman Filter to predict missing values
  Serial.println(">>> Using Kalman Filter to predict missing data:");

  for (int i = 0; i < lostCount; i++) {
    uint16_t missedSeq = lastSeqNum + 1 + i;

    // Predict using Kalman Filter
    float predictedTemp = kfTemp.predict();
    float predictedHum = kfHumidity.predict();

    packetsPredicted++;

    Serial.print(missedSeq);
    Serial.print(" | ");
    Serial.print(predictedTemp, 1);
    Serial.print(" | ");
    Serial.print(predictedHum, 1);
    Serial.print(" | ");
    Serial.print("N/A");
    Serial.print("  | ");
    Serial.print("N/A");
    Serial.print("    | ");
    Serial.print("N/A");
    Serial.print("     | ");
    Serial.println("PREDICTED");
  }

  Serial.println("----+------+------+------+--------+---------+----------");
}

// ========== STATISTICS ==========
void printStatistics() {
  Serial.println("\n========================================");
  Serial.println("           STATISTICS");
  Serial.println("========================================");

  uint32_t totalExpected = totalPacketsReceived + packetsLost;

  Serial.print("Total packets received: ");
  Serial.println(totalPacketsReceived);

  Serial.print("Packets lost: ");
  Serial.print(packetsLost);
  Serial.print(" (");
  if (totalExpected > 0) {
    Serial.print(100.0 * packetsLost / totalExpected, 2);
  } else {
    Serial.print("0.00");
  }
  Serial.println("%)");

  Serial.print("Packets predicted: ");
  Serial.println(packetsPredicted);

  Serial.print("Total bit errors detected: ");
  Serial.println(totalErrorsDetected);

  Serial.print("Total bit errors corrected: ");
  Serial.print(totalErrorsCorrected);
  if (totalErrorsDetected > 0) {
    Serial.print(" (");
    Serial.print(100.0 * totalErrorsCorrected / totalErrorsDetected, 1);
    Serial.print("% correction rate)");
  }
  Serial.println();

  Serial.print("Uncorrectable errors: ");
  Serial.println(uncorrectableErrors);

  // Bit injection statistics (only if BIT_ERROR_MODE is enabled)
  if (BIT_ERROR_MODE) {
    Serial.println("-------- BIT INJECTION TEST --------");
    Serial.print("Packets with 1-bit errors: ");
    Serial.print(packetsWithOneBitError);
    Serial.print(" (corrected: ");
    Serial.print(oneBitErrorsCorrected);
    Serial.println(")");
    Serial.print("Packets with 2-bit errors: ");
    Serial.print(packetsWithTwoBitError);
    Serial.println(" (uncorrectable)");
    Serial.print("Hamming success rate: ");
    if (packetsWithOneBitError > 0) {
      // Success = 1-bit errors that were corrected / total 1-bit errors
      float successRate =
          100.0 * (float)oneBitErrorsCorrected / (float)packetsWithOneBitError;
      Serial.print(successRate, 1);
      Serial.println("%");
    } else {
      Serial.println("N/A (no 1-bit errors injected)");
    }
  }

  // ===== NEW: TELECOMMUNICATIONS METRICS =====
  Serial.println("-------- RF METRICS --------");

  // RSSI
  Serial.print("Average RSSI: ");
  if (totalPacketsReceived > 0) {
    Serial.print(totalRSSI / totalPacketsReceived, 1);
  } else {
    Serial.print("N/A");
  }
  Serial.print(" dBm (min: ");
  Serial.print(minRSSI);
  Serial.print(", max: ");
  Serial.print(maxRSSI);
  Serial.println(")");

  Serial.println("-------- ERROR RATES --------");

  // BER (Bit Error Rate)
  Serial.print("BER (Bit Error Rate): ");
  if (totalPacketsReceived > 0) {
    // Total bits = packets * 32 bytes * 8 bits/byte
    float totalBits = (float)totalPacketsReceived * 32.0 * 8.0;
    float BER = (float)totalErrorsDetected / totalBits;
    Serial.print(BER, 8); // Show 8 decimal places for small BER
    Serial.print(" (");
    Serial.print(BER * 100.0, 6);
    Serial.println("%)");
  } else {
    Serial.println("N/A");
  }

  // PER (Packet Error Rate)
  Serial.print("PER (Packet Error Rate): ");
  if (totalExpected > 0) {
    float PER =
        (float)(uncorrectableErrors + packetsLost) / (float)totalExpected;
    Serial.print(PER, 6);
    Serial.print(" (");
    Serial.print(PER * 100.0, 2);
    Serial.println("%)");
  } else {
    Serial.println("N/A");
  }

  // FEC Efficiency
  Serial.print("FEC Efficiency: ");
  if (totalErrorsDetected > 0) {
    float fecEff =
        100.0 * (float)totalErrorsCorrected / (float)totalErrorsDetected;
    Serial.print(fecEff, 1);
    Serial.println("%");
  } else {
    Serial.println("100% (no errors detected)");
  }

  Serial.println("-----------------------------");

  Serial.print("Reliability: ");
  if (totalPacketsReceived > 0) {
    float reliability = 100.0 * (totalPacketsReceived - uncorrectableErrors) /
                        totalPacketsReceived;
    Serial.print(reliability, 2);
    Serial.println("%");
  } else {
    Serial.println("N/A");
  }

  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");

  Serial.println("========================================\n");
  Serial.println("Seq | Temp | Hum  | RSSI | ErrDet | ErrCorr | Status");
  Serial.println("----+------+------+------+--------+---------+----------");
}

// ========== HANDLE CORRUPTED PACKET (CRC FAILED) ==========
void handleCorruptedPacket(int rssi, float snr) {
  // Increment expected sequence number (we know a packet was sent)
  uint16_t expectedSeq = lastSeqNum + 1;

  Serial.println("----+------+------+------+--------+---------+----------");
  Serial.print("!!! CRC FAILED for expected seq ");
  Serial.print(expectedSeq);
  Serial.print(" | RSSI=");
  Serial.print(rssi);
  Serial.println(" dBm");

  // Use Kalman Filter to predict the values
  float predictedTemp = kfTemp.predict();
  float predictedHum = kfHumidity.predict();

  packetsPredicted++;

  // Display predicted values
  Serial.println(">>> Using Kalman Filter to predict corrupted data:");
  Serial.print(expectedSeq);
  Serial.print(" | ");
  Serial.print(predictedTemp, 1);
  Serial.print(" | ");
  Serial.print(predictedHum, 1);
  Serial.print(" | ");
  Serial.print(rssi);
  Serial.print(" | ");
  Serial.print("CRC");
  Serial.print("    | ");
  Serial.print("FAIL");
  Serial.print("    | ");
  Serial.println("PREDICTED");

  Serial.println("----+------+------+------+--------+---------+----------");

  // Update last sequence number
  lastSeqNum = expectedSeq;
}