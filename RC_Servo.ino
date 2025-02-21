#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

const int steeringPin = 4;  // First servo for steering
const int servoPin = 2;     // Second servo for LIDAR scanning

const int steeringChannel = 0;
const int steeringFreq = 50; 
const int steeringResolution = 16; 

const int servoChannel = 1; 
const int servoFreq = 50;
const int servoResolution = 16; 

int currentAngle = 0;
int step = 1;  // Increased step size for more noticeable movement
unsigned long lastUpdate = 0;
const unsigned long interval = 15;  // 15ms interval - typical update rate for these servos

int wifiChannel = 1; 

uint8_t recieverMacAddress[] = {0x08, 0xA6, 0xF7, 0x65, 0xCF, 0xF4};

struct PacketData {
  int xAxisValue;
  int yAxisValue;
};
PacketData receivedData;

struct DistanceData{
  int distanceValue;
};
DistanceData data;

int angleToDutyCycle(int angle) {
    // PWM duty cycle values for 16-bit resolution
    const int SERVO_MIN_PULSEWIDTH = 1638;  // 0.5ms
    const int SERVO_MID_PULSEWIDTH = 4915;  // 1.5ms
    const int SERVO_MAX_PULSEWIDTH = 8192;  // 2.5ms
    
    return map(angle, 0, 180, SERVO_MIN_PULSEWIDTH, SERVO_MAX_PULSEWIDTH);
}

void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received Joystick Y-axis Value: ");
    Serial.println(receivedData.yAxisValue);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void setup() {
    Serial.begin(115200);  // Increased baud rate for better debugging
    
    // WiFi and ESP-NOW setup
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    Serial.print("Receiver 2 MAC Address: ");
    Serial.println(WiFi.macAddress());
    
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    Serial.println("ESP-NOW initialized successfully");
    
    esp_now_register_recv_cb(onDataReceive);

    esp_now_register_send_cb(onDataSent);
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, recieverMacAddress, 6);
    peerInfo.channel = wifiChannel;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
       Serial.println("Failed to add peer");
    } else {
        Serial.println("Peer added successfully");
    }

    if (!lox.begin()) {
        Serial.println(F("Failed to initialize VL53L0X"));
        while(1); // Don't proceed if initialization fails
    }
    Serial.println(F("VL53L0X initialized successfully"));
    
    // Servo setup
    ledcSetup(steeringChannel, steeringFreq, steeringResolution);
    ledcAttachPin(steeringPin, steeringChannel);

    ledcSetup(servoChannel, servoFreq, servoResolution);
    ledcAttachPin(servoPin, servoChannel);

    // Initialize scanning servo to starting position
    ledcWrite(servoChannel, angleToDutyCycle(0));
    delay(500); // Give servo time to reach starting position
}

void loop() {
    unsigned long currentTime = millis();
    
    Serial.print("Processing Y-axis Value: ");
    Serial.println(receivedData.yAxisValue);

    int y = receivedData.yAxisValue;
    if (y == 0) {
        ledcWrite(steeringChannel, angleToDutyCycle(0));
    } else if (y >= 100 && y <= 4000) {
        ledcWrite(steeringChannel, angleToDutyCycle(90));
    } else if (y == 4095) {
        ledcWrite(steeringChannel, angleToDutyCycle(180));
    }

    // Handle scanning servo movement
    if (currentTime - lastUpdate >= interval) {
        lastUpdate = currentTime;
        
        // Update angle
        currentAngle += step;
        
        // Reverse direction at limits
        if (currentAngle >= 180) {
            currentAngle = 180;
            step = -step;
        } else if (currentAngle <= 0) {
            currentAngle = 0;
            step = -step;
        }
        
        // Move servo to new position
        ledcWrite(servoChannel, angleToDutyCycle(currentAngle));
        
        // Debug output
        Serial.print("Scanning Angle: ");
        Serial.println(currentAngle);
    }

    int measuredDistance;
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      measuredDistance = measure.RangeMilliMeter;
      Serial.print("Distance(mm): ");
      Serial.println(measuredDistance);
    } else {
      measuredDistance = 0;
      Serial.println("out of range");
    }

    data.distanceValue = measuredDistance;

    esp_err_t result = esp_now_send(recieverMacAddress, (uint8_t *)&data, sizeof(data));
    if (result == ESP_OK) {
      Serial.println("Distance data sent successfully to Receiver");
    } else {
      Serial.println("Failed to send Distance data to Receiver");
    }
  

    // Reduced delay to prevent jerky movement
    delay(10);
}
