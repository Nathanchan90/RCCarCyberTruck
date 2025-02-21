
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HardwareSerial.h>

#define JoyStick_X 36 // Joystick X-axis pin
#define JoyStick_Y 35 // Joystick Y-axis pin

#define WHEEL_DIAMETER 0.096 // Wheel diameter in meters
#define WHEEL_CIRCUMFERENCE (3.1416 * WHEEL_DIAMETER) // Circumference in meters

#define TXD_PIN 17
#define RXD_PIN 16

int wifiChannel = 1; 

uint8_t receiverMacAddress1[] = {0xCC, 0xDB, 0xA7, 0x92, 0xDC, 0x60};
uint8_t receiverMacAddress2[] = {0x08, 0xA6, 0xF7, 0x65, 0xDA, 0x10};

struct PacketData {
  int xAxisValue;
  int yAxisValue;
};
PacketData data;

struct RPMData {
  int rpmValue;
};
RPMData receivedData;

float calculateSpeed(int rpm) {
  // Formula: Speed (m/s) = (RPM * Circumference) / 60
  return (rpm * WHEEL_CIRCUMFERENCE) / 60.0;
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received RPM Value: ");
    Serial.println(receivedData.rpmValue);
}
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN); // Initialize UART1
    // Set WiFi mode first
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();  // Add this to ensure clean state
    
    // Configure the channel
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    
    Serial.print("Current Wi-Fi Channel: ");
    Serial.println(wifiChannel);
    
    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }
    
    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataReceive);
    
    // Add first peer with interface specification
    esp_now_peer_info_t peerInfo1;
    memset(&peerInfo1, 0, sizeof(peerInfo1));  // Zero out the structure first
    memcpy(peerInfo1.peer_addr, receiverMacAddress1, 6);
    peerInfo1.channel = wifiChannel;
    peerInfo1.encrypt = false;
    peerInfo1.ifidx = WIFI_IF_STA;  // Specify the interface
    
    if (esp_now_add_peer(&peerInfo1) != ESP_OK) {
        Serial.println("Failed to add peer 1");
    } else {
        Serial.println("Peer 1 added successfully");
    }
    
    // Add second peer with interface specification
    esp_now_peer_info_t peerInfo2;
    memset(&peerInfo2, 0, sizeof(peerInfo2));  // Zero out the structure first
    memcpy(peerInfo2.peer_addr, receiverMacAddress2, 6);
    peerInfo2.channel = wifiChannel;
    peerInfo2.encrypt = false;
    peerInfo2.ifidx = WIFI_IF_STA;  // Specify the interface
    
    if (esp_now_add_peer(&peerInfo2) != ESP_OK) {
        Serial.println("Failed to add peer 2");
    } else {
        Serial.println("Peer 2 added successfully");
    }
    
    pinMode(JoyStick_X, INPUT);
    pinMode(JoyStick_Y, INPUT);
}
void loop() {
  // Read joystick value
  data.xAxisValue = analogRead(JoyStick_X);
  data.yAxisValue = analogRead(JoyStick_Y);

  // Log the value being transmitted
  Serial.print("Transmitting Joystick X-axis Value: ");
  Serial.println(data.xAxisValue);
  Serial.print("Transmitting Joystick Y-axis Value: ");
  Serial.println(data.yAxisValue);

  // Send data to receiver 1
  esp_err_t result1 = esp_now_send(receiverMacAddress1, (uint8_t *)&data, sizeof(data));
  if (result1 == ESP_OK) {
    Serial.println("Joystick data sent successfully to Receiver 1");
  } else {
    Serial.println("Failed to send joystick data to Receiver 1");
  }

  // Send data to receiver 2
  esp_err_t result2 = esp_now_send(receiverMacAddress2, (uint8_t *)&data, sizeof(data));
  if (result2 == ESP_OK) {
    Serial.println("Joystick data sent successfully to Receiver 2");
  } else {
    Serial.println("Failed to send joystick data to Receiver 2");
  }

  // Calculate speed based on received RPM
  float speed = calculateSpeed(receivedData.rpmValue);

  // Log calculated speed
  Serial.print("Calculated Speed (m/s): ");
  Serial.println(speed);

  // Send speed data to LCD ESP32 via UART
  Serial1.println(speed); // Send the speed value as a float over UART

  delay(50); // Adjust as needed for update frequency
}

