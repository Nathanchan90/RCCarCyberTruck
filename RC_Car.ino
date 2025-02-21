#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

int en1 = 25;
int en2 = 4;
int in1 = 26;
int in2 = 27;
int in3 = 16;
int in4 = 17;


const uint32_t freq = 5000;
const uint8_t resolution = 8;
const uint8_t channel1 = 1;
const uint8_t channel2 = 3; 


volatile int pulseCount = 0;
float rpm = 0;
unsigned long lastTime = 0;
const unsigned long interval = 1000;
int wifiChannel = 1; 

uint8_t recieverMacAddress[] = {0xC0, 0x49, 0xEF, 0xB5, 0x7C, 0x40};

struct PacketData {
  int xAxisValue;
  int yAxisValue;
};
PacketData receivedData;

struct RPMData {
  int rpmValue; 
};
RPMData data;

void IRAM_ATTR pulseISR() {
  pulseCount++;
}

void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received Joystick X-axis Value: ");
    Serial.println(receivedData.xAxisValue);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

void setup() {
  Serial.begin(9600);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  ledcSetup(channel1, freq, resolution);
  ledcAttachPin(en1, channel1);
  ledcSetup(channel2, freq, resolution);
  ledcAttachPin(en2, channel2);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
  Serial.print("Current Wi-Fi Channel: ");
  Serial.println(wifiChannel);

  if (esp_now_init() != ESP_OK) {
  Serial.println("Error initializing ESP-NOW");
} else {
  Serial.println("ESP-NOW initialized successfully");
}

  esp_now_register_recv_cb(onDataReceive);
  Serial.println("Receive callback registered");

  pinMode(15, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(15), pulseISR, RISING);

  Serial.println("Receiver setup complete");

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
}

void loop() {
  // Map received joystick data to motor control
  int x = receivedData.xAxisValue;

  Serial.print("Processing X-axis Value in loop: ");
  Serial.println(x);


  if (x == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    digitalWrite(in3, LOW);
    digitalWrite(in4, HIGH);
    ledcWrite(channel1, 220); 
    ledcWrite(channel2, 220); 
  } else if (x >= 100 && x <= 4000) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW); 
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
  } else if (x == 4095){
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    digitalWrite(in3, HIGH);
    digitalWrite(in4, LOW);
    ledcWrite(channel1, 220);
    ledcWrite(channel2, 220); 
  }
  // RPM calculation (unchanged)
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= interval) {
    noInterrupts();
    int localPulseCount = pulseCount;
    pulseCount = 0;
    interrupts();

    rpm = (localPulseCount / 20.0) * (60000.0 / interval);
    lastTime = currentTime;

    Serial.print("RPM: ");
    Serial.println(rpm);
    data.rpmValue = static_cast<int>(rpm); 
  }
  
  esp_err_t result = esp_now_send(recieverMacAddress, (uint8_t *)&data, sizeof(data));
  if (result == ESP_OK) {
    Serial.println("RPM data sent successfully to Receiver");
  } else {
    Serial.println("Failed to send RPM data to Receiver");
  }
  
  delay(200);
}


