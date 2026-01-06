#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

// ========== CONFIGURATION ==========
// LoRa SX1278 Ra-02 Pins
#define LORA_SCK    18
#define LORA_MISO   19
#define LORA_MOSI   23
#define LORA_SS     5
#define LORA_RST    14
#define LORA_DIO0   2
#define LORA_BAND   433E6  // 433MHz

// DHT11 Sensor
#define DHT_PIN     4
#define DHT_TYPE    DHT11

// Timing
#define SAMPLE_INTERVAL 1000  // ms - đọc cảm biến mỗi 1 giây

// ========== GLOBAL OBJECTS ==========
DHT dht(DHT_PIN, DHT_TYPE);

// ========== DATA STRUCTURES ==========
struct __attribute__((packed)) Packet {
  uint16_t seqNum;        // 2 bytes - Sequence number
  uint32_t timestamp;     // 4 bytes - Timestamp (ms)
  float temperature;      // 4 bytes - Temperature (°C)
  float humidity;         // 4 bytes - Humidity (%)
  uint16_t crc;          // 2 bytes - CRC checksum
  // Total: 16 bytes (packed, no padding)
};

// ========== KALMAN FILTER ==========
class SimpleKalmanFilter {
private:
  float Q; // Process noise covariance
  float R; // Measurement noise covariance
  float P; // Estimation error covariance
  float K; // Kalman gain
  float X; // Estimated value
  
public:
  SimpleKalmanFilter(float processNoise, float measurementNoise, float estimationError, float initialValue) {
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
  
  float getEstimate() {
    return X;
  }
};

// Kalman filters cho temperature và humidity
SimpleKalmanFilter kfTemp(0.01, 0.5, 1, 25.0);  // Q, R, P, initial
SimpleKalmanFilter kfHumidity(0.01, 2.0, 1, 50.0);

// ========== HAMMING(7,4) FEC ==========
// Encode 4 bits thành 7 bits với Hamming code
uint8_t hammingEncode(uint8_t nibble) {
  // Extract 4 data bits
  uint8_t d1 = (nibble >> 3) & 1;
  uint8_t d2 = (nibble >> 2) & 1;
  uint8_t d3 = (nibble >> 1) & 1;
  uint8_t d4 = (nibble >> 0) & 1;
  
  // Calculate parity bits
  uint8_t p1 = d1 ^ d2 ^ d4;
  uint8_t p2 = d1 ^ d3 ^ d4;
  uint8_t p3 = d2 ^ d3 ^ d4;
  
  // Build 7-bit codeword: [p1 p2 d1 p3 d2 d3 d4]
  uint8_t codeword = 0;
  codeword |= (p1 << 6);
  codeword |= (p2 << 5);
  codeword |= (d1 << 4);
  codeword |= (p3 << 3);
  codeword |= (d2 << 2);
  codeword |= (d3 << 1);
  codeword |= (d4 << 0);
  
  return codeword;
}

// Encode 1 byte thành 2 codewords (mỗi nibble 1 codeword)
void hammingEncodeByte(uint8_t byte, uint8_t* output) {
  uint8_t highNibble = (byte >> 4) & 0x0F;
  uint8_t lowNibble = byte & 0x0F;
  
  output[0] = hammingEncode(highNibble);
  output[1] = hammingEncode(lowNibble);
}

// ========== CRC CALCULATION ==========
uint16_t calculateCRC(uint8_t* data, size_t length) {
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

// ========== PACKET ENCODING WITH FEC ==========
void encodePacketWithFEC(Packet* pkt, uint8_t* output, size_t* outputLen) {
  // Convert struct to byte array
  uint8_t* rawData = (uint8_t*)pkt;
  
  // Calculate CRC (excluding CRC field itself)
  pkt->crc = calculateCRC(rawData, sizeof(Packet) - 2);
  
  // Encode each byte with Hamming
  *outputLen = 0;
  for (size_t i = 0; i < sizeof(Packet); i++) {
    hammingEncodeByte(rawData[i], &output[*outputLen]);
    *outputLen += 2;  // Each byte becomes 2 encoded bytes
  }
  // Total: 16 bytes * 2 = 32 bytes encoded
}

// ========== GLOBAL VARIABLES ==========
uint16_t sequenceNumber = 0;
unsigned long lastSampleTime = 0;

// Statistics
uint32_t totalPacketsSent = 0;
uint32_t dhtReadErrors = 0;

// ========== FORWARD DECLARATIONS ==========
void printStatistics();

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n========================================");
  Serial.println("ESP32 TRANSMITTER");
  Serial.println("FEC + Kalman Filter + LoRa SX1278");
  Serial.println("========================================\n");
  
  // Initialize DHT11
  Serial.print("Initializing DHT11... ");
  dht.begin();
  delay(2000);  // DHT11 cần thời gian khởi động
  Serial.println("OK");
  
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
  
  // Configure LoRa parameters
  Serial.println("\nConfiguring LoRa parameters:");
  LoRa.setSpreadingFactor(11);      // SF7 = fastest, shortest range
  Serial.println("  Spreading Factor: 11");
  
  LoRa.setSignalBandwidth(125E3);  // 125kHz bandwidth
  Serial.println("  Bandwidth: 125 kHz");
  
  LoRa.setCodingRate4(5);          // CR = 4/5
  Serial.println("  Coding Rate: 4/5");
  
  LoRa.setTxPower(20);             // Max power = 20dBm
  Serial.println("  TX Power: 20 dBm");
  
  LoRa.setSyncWord(0x12);          // Sync word (0x12 = private network)
  Serial.println("  Sync Word: 0x12");
  
  Serial.println("\n========================================");
  Serial.println("TRANSMITTER READY");
  Serial.println("========================================\n");
  Serial.println("Format: [Seq] T=xx.x°C H=xx.x% | Raw: xx.x/xx.x | Filtered: xx.x/xx.x | Size: xx bytes\n");
  
  lastSampleTime = millis();
}

// ========== MAIN LOOP ==========
void loop() {
  unsigned long currentTime = millis();
  
  // Check if it's time to sample
  if (currentTime - lastSampleTime >= SAMPLE_INTERVAL) {
    lastSampleTime = currentTime;
    
    // ===== READ SENSOR =====
    float rawTemp = dht.readTemperature();
    float rawHum = dht.readHumidity();
    
    // Check for DHT read errors
    if (isnan(rawTemp) || isnan(rawHum)) {
      dhtReadErrors++;
      Serial.print("[ERROR] DHT11 read failed (total errors: ");
      Serial.print(dhtReadErrors);
      Serial.println(")");
      return;  // Skip this cycle
    }
    
    // ===== APPLY KALMAN FILTER =====
    float filteredTemp = kfTemp.update(rawTemp);
    float filteredHum = kfHumidity.update(rawHum);
    
    // ===== CREATE PACKET =====
    Packet pkt;
    pkt.seqNum = sequenceNumber;
    pkt.timestamp = currentTime;
    pkt.temperature = filteredTemp;
    pkt.humidity = filteredHum;
    pkt.crc = 0;  // Will be calculated during encoding
    
    // ===== ENCODE WITH FEC =====
    uint8_t encodedData[64];  // Buffer for encoded packet
    size_t encodedLen;
    encodePacketWithFEC(&pkt, encodedData, &encodedLen);
    
    // ===== TRANSMIT VIA LoRa =====
    LoRa.beginPacket();
    LoRa.write(encodedData, encodedLen);
    LoRa.endPacket();
    
    totalPacketsSent++;
    sequenceNumber++;
    
    // ===== PRINT TO SERIAL =====
    Serial.print("[");
    Serial.print(pkt.seqNum);
    Serial.print("] ");
    
    Serial.print("T=");
    Serial.print(filteredTemp, 1);
    Serial.print("°C H=");
    Serial.print(filteredHum, 1);
    Serial.print("% | ");
    
    Serial.print("Raw: ");
    Serial.print(rawTemp, 1);
    Serial.print("/");
    Serial.print(rawHum, 1);
    Serial.print(" | ");
    
    Serial.print("Filtered: ");
    Serial.print(filteredTemp, 1);
    Serial.print("/");
    Serial.print(filteredHum, 1);
    Serial.print(" | ");
    
    Serial.print("Size: ");
    Serial.print(encodedLen);
    Serial.println(" bytes");
    
    // Print statistics every 50 packets
    if (totalPacketsSent % 50 == 0) {
      printStatistics();
    }
  }
}

// ========== STATISTICS ==========
void printStatistics() {
  Serial.println("\n========== STATISTICS ==========");
  Serial.print("Total packets sent: ");
  Serial.println(totalPacketsSent);
  Serial.print("DHT read errors: ");
  Serial.println(dhtReadErrors);
  Serial.print("Success rate: ");
  Serial.print(100.0 * (totalPacketsSent) / (totalPacketsSent + dhtReadErrors), 1);
  Serial.println("%");
  Serial.print("Uptime: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  Serial.println("================================\n");
}