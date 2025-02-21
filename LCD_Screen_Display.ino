#include <TFT_eSPI.h> 
#include <SPI.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <HardwareSerial.h>

TFT_eSPI tft = TFT_eSPI();       // Invoke custom library

#define TFT_GREY 0x5AEB
#define LOOP_PERIOD 35 // Display updates every 35 ms

#define RXD_PIN 16
#define TXD_PIN 17

// Analog meter globals
float ltx = 0;    // Saved x coord of bottom of needle
uint16_t osx = 120, osy = 120; // Saved x & y coords
uint32_t updateTime = 0;       // time for next update
int old_analog = -999; // Value last displayed

// Modified Radar display globals
#define RADAR_PIVOT_X 120    // Center X coordinate
#define RADAR_PIVOT_Y 317    // Center Y coordinate for semicircle
#define RADAR_LENGTH 180     // Maximum length of radar line
#define HISTORY_POINTS 180   // One point per degree

float oldRadarX = RADAR_PIVOT_X;
float oldRadarY = RADAR_PIVOT_Y - RADAR_LENGTH;
int oldDistance = 0;
float sweepAngle = 0;      // Current sweep angle (0-180)
bool movingRight = true;   // Track direction of sweep
int distanceHistory[HISTORY_POINTS];
unsigned long lastRadarUpdate = 0;
const int RADAR_UPDATE_INTERVAL = 30;

float receivedSpeed = 0.0;
int wifiChannel = 1;

struct DistanceData {
    int distanceValue;
};
DistanceData receivedData;

void analogMeter() {
    // Meter outline
    tft.fillRect(0, 0, 239, 126, TFT_RED);
    tft.fillRect(5, 3, 230, 119, TFT_BLACK);

    tft.setTextColor(TFT_WHITE);  // Text colour

    // Draw ticks every 5 degrees from -50 to +50 degrees (100 deg. FSD swing)
    for (int i = -50; i < 51; i += 5) {
        // Long scale tick length
        int tl = 15;

        // Coordinates of tick to draw
        float sx = cos((i - 90) * 0.0174532925);
        float sy = sin((i - 90) * 0.0174532925);
        uint16_t x0 = sx * (100 + tl) + 120;
        uint16_t y0 = sy * (100 + tl) + 140;
        uint16_t x1 = sx * 100 + 120;
        uint16_t y1 = sy * 100 + 140;

        // Coordinates of next tick for zone fill
        float sx2 = cos((i + 5 - 90) * 0.0174532925);
        float sy2 = sin((i + 5 - 90) * 0.0174532925);
        int x2 = sx2 * (100 + tl) + 120;
        int y2 = sy2 * (100 + tl) + 140;
        int x3 = sx2 * 100 + 120;
        int y3 = sy2 * 100 + 140;

        // Yellow zone limits
        if (i >= 0 && i < 25) {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_YELLOW);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_YELLOW);
        }

        // Orange zone limits
        if (i >= 25 && i < 50) {
            tft.fillTriangle(x0, y0, x1, y1, x2, y2, TFT_ORANGE);
            tft.fillTriangle(x1, y1, x2, y2, x3, y3, TFT_ORANGE);
        }

        // Short scale tick length
        if (i % 25 != 0) tl = 8;

        // Recalculate coords incase tick length changed
        x0 = sx * (100 + tl) + 120;
        y0 = sy * (100 + tl) + 140;
        x1 = sx * 100 + 120;
        y1 = sy * 100 + 140;

        // Draw tick
        tft.drawLine(x0, y0, x1, y1, TFT_WHITE);

        // Check if labels should be drawn, with position tweaks
        if (i % 25 == 0) {
            // Calculate label positions
            x0 = sx * (100 + tl + 10) + 120;
            y0 = sy * (100 + tl + 10) + 140;
            switch (i / 25) {
                case -2: tft.drawCentreString("0", x0, y0 - 12, 2); break;
                case -1: tft.drawCentreString("10", x0, y0 - 9, 2); break;
                case 0: tft.drawCentreString("20", x0, y0 - 6, 2); break;
                case 1: tft.drawCentreString("30", x0, y0 - 9, 2); break;
                case 2: tft.drawCentreString("40", x0, y0 - 12, 2); break;
            }
        }

        // Now draw the arc of the scale
        sx = cos((i + 5 - 90) * 0.0174532925);
        sy = sin((i + 5 - 90) * 0.0174532925);
        x0 = sx * 100 + 120;
        y0 = sy * 100 + 140;
        if (i < 50) tft.drawLine(x0, y0, x1, y1, TFT_WHITE);
    }

    tft.drawRect(5, 3, 230, 119, TFT_WHITE); // Draw bezel line
    plotNeedle(0, 0); // Put meter needle at 0
}

void plotNeedle(int value, byte ms_delay) {
    // Ensure value is within the displayable range
    if (value < -10) value = -10; 
    if (value > 110) value = 110;

    // Convert the mapped value back to actual speed in km/h (0-40 range)
    int actualSpeed = map(value, -10, 110, 0, 40);
    
    // Format the value with leading zeros
    char buf[8];
    sprintf(buf, "%02d", actualSpeed);

    // Move the needle until new value reached
    while (!(value == old_analog)) {
        if (old_analog < value) old_analog++;
        else old_analog--;

        if (ms_delay == 0) old_analog = value; // Update immediately if delay is 0

        float sdeg = map(old_analog, -10, 110, -150, -30); // Map value to angle
        // Calculate tip of needle coords
        float sx = cos(sdeg * 0.0174532925);
        float sy = sin(sdeg * 0.0174532925);

        // Erase old needle image
        tft.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_BLACK);
        tft.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_BLACK);
        tft.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_BLACK);

        // Redraw the speed value
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString(buf, 120, 70, 6);
        tft.drawString("M/S", 150, 90, 2);

        // Store new needle end coords for next erase
        ltx = tan((sdeg + 90) * 0.0174532925);
        osx = sx * 98 + 120;
        osy = sy * 98 + 140;

        // Draw the needle in the new position
        tft.drawLine(120 + 20 * ltx - 1, 140 - 20, osx - 1, osy, TFT_RED);
        tft.drawLine(120 + 20 * ltx, 140 - 20, osx, osy, TFT_RED);
        tft.drawLine(120 + 20 * ltx + 1, 140 - 20, osx + 1, osy, TFT_RED);

        // Slow needle down slightly as it approaches new position
        if (abs(old_analog - value) < 10) ms_delay += ms_delay / 5;

        // Wait before next update
        delay(ms_delay);
    }
}

void onDataReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));
    Serial.print("Received Distance Value: ");
    Serial.println(receivedData.distanceValue);
}

void setup(void) {
    tft.init();
    tft.setRotation(0);
    Serial.begin(115200);
    Serial1.begin(115200, SERIAL_8N1, RXD_PIN, TXD_PIN);

    tft.fillScreen(TFT_BLACK);

    // Initialize distanceHistory array
    for(int i = 0; i < HISTORY_POINTS; i++) {
        distanceHistory[i] = 0;
    }

    // Initialize radar variables
    sweepAngle = 0;
    movingRight = true;

    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataReceive);

    analogMeter();
    updateTime = millis();
    
    // Draw initial radar background
    drawRadarBackground();
}

void loop() {
    if (Serial1.available()) {
        String speedStr = Serial1.readStringUntil('\n');
        receivedSpeed = speedStr.toFloat();
        Serial.print("Received Speed (m/s): ");
        Serial.println(receivedSpeed);
    }

    // Update speed meter
    if (updateTime <= millis()) {
        updateTime = millis() + LOOP_PERIOD;
        int meterValue = map(receivedSpeed * 3.6, 0, 36, -10, 110);
        meterValue = constrain(meterValue, -10, 110);
        plotNeedle(meterValue, 10);
    }

    // Update radar display
    drawRadar(receivedData.distanceValue);
}

void drawRadarBackground() {
    // Draw radar box and background
    tft.fillRect(0, 126, 240, 194, TFT_RED);
    tft.fillRect(5, 129, 230, 188, TFT_BLACK);
    
    // Calculate the visible area boundaries
    int minY = 129;  // Top boundary
    int maxY = 317;  // Bottom boundary
    int minX = 5;    // Left boundary
    int maxX = 235;  // Right boundary
    
    // Draw semicircle arcs
    for(int r = RADAR_LENGTH/4; r <= RADAR_LENGTH; r += RADAR_LENGTH/4) {
        for(int angle = 0; angle <= 180; angle += 2) {
            float rad = angle * 0.0174532925;
            float nextRad = (angle + 2) * 0.0174532925;
            
            int x1 = RADAR_PIVOT_X + r * cos(rad);
            int y1 = RADAR_PIVOT_Y - r * sin(rad);
            int x2 = RADAR_PIVOT_X + r * cos(nextRad);
            int y2 = RADAR_PIVOT_Y - r * sin(nextRad);
            
            // Only draw if within bounds
            if (x1 >= minX && x1 <= maxX && y1 >= minY && y1 <= maxY &&
                x2 >= minX && x2 <= maxX && y2 >= minY && y2 <= maxY) {
                tft.drawLine(x1, y1, x2, y2, TFT_BLUE);
            }
        }
    }
    
    // Draw angle lines (grid lines)
    for(int angle = 0; angle <= 180; angle += 30) {
        float rad = angle * 0.0174532925;
        int x = RADAR_PIVOT_X + RADAR_LENGTH * cos(rad);
        int y = RADAR_PIVOT_Y - RADAR_LENGTH * sin(rad);
        
        // Ensure lines stay within boundaries
        if (x < minX) x = minX;
        if (x > maxX) x = maxX;
        if (y < minY) y = minY;
        if (y > maxY) y = maxY;
        
        // Draw the full line from center to edge
        tft.drawLine(RADAR_PIVOT_X, RADAR_PIVOT_Y, x, y, TFT_BLUE);
        
        // Draw angle labels only for 30° to 150°
        if (angle >= 30 && angle <= 150) {
            char angleStr[4];
            sprintf(angleStr, "%d°", angle);
            int textX = RADAR_PIVOT_X + (RADAR_LENGTH - 20) * cos(rad);
            int textY = RADAR_PIVOT_Y - (RADAR_LENGTH - 20) * sin(rad);
            
            // Adjust text position to stay within bounds
            if (textX < minX + 10) textX = minX + 10;
            if (textX > maxX - 20) textX = maxX - 20;
            if (textY < minY + 10) textY = minY + 10;
            if (textY > maxY - 10) textY = maxY - 10;
            
            tft.setTextColor(TFT_WHITE);
            tft.drawString(angleStr, textX-10, textY-5, 2);
        }
    }
    
    // Draw the baseline (180-degree line)
    tft.drawLine(minX, RADAR_PIVOT_Y, maxX, RADAR_PIVOT_Y, TFT_BLUE);
    
    // Draw the border last to ensure it's on top
    tft.drawRect(5, 129, 230, 188, TFT_WHITE);
}
void drawRadar(int distance) {
    if (millis() - lastRadarUpdate < RADAR_UPDATE_INTERVAL) {
        return;
    }
    lastRadarUpdate = millis();

    // Clear the entire radar background periodically to prevent artifacts
    static unsigned long lastBackgroundClear = 0;
    if (millis() - lastBackgroundClear > 5000) {  // Clear every 5 seconds
        drawRadarBackground();
        lastBackgroundClear = millis();
    }

    // Clear previous line
    tft.drawLine(RADAR_PIVOT_X, RADAR_PIVOT_Y, oldRadarX, oldRadarY, TFT_BLACK);
    
    // Update sweep angle based on current direction
    if (movingRight) {
        sweepAngle += 1;
        if (sweepAngle >= 180) {
            movingRight = false;
            sweepAngle = 179;  // Prevent overshooting
        }
    } else {
        sweepAngle -= 1;
        if (sweepAngle <= 0) {
            movingRight = true;
            sweepAngle = 1;  // Prevent overshooting
        }
    }

    // Store distance in history at current angle
    distanceHistory[(int)sweepAngle] = distance;
    
    // Draw historical points with fading effect
    for (int angle = 0; angle < 180; angle++) {
        if (distanceHistory[angle] > 0) {
            float rad = angle * 0.0174532925;
            int lineLength = map(constrain(distanceHistory[angle], 0, 400), 0, 400, 0, RADAR_LENGTH);
            int histX = RADAR_PIVOT_X + lineLength * cos(rad);
            int histY = RADAR_PIVOT_Y - lineLength * sin(rad);
            
            // Use different colors for historical points
            uint16_t pointColor;
            int angleDiff = abs(angle - (int)sweepAngle);
            if (angleDiff < 10) {
                pointColor = TFT_GREEN;  // Recent points
            } else {
                pointColor = TFT_BLUE;   // Older points
            }
            
            tft.drawLine(RADAR_PIVOT_X, RADAR_PIVOT_Y, histX, histY, pointColor);
        }
    }
    
    // Draw active sweep line
    float rad = sweepAngle * 0.0174532925;
    int lineLength = map(constrain(distance, 0, 400), 0, 400, 0, RADAR_LENGTH);
    float newX = RADAR_PIVOT_X + lineLength * cos(rad);
    float newY = RADAR_PIVOT_Y - lineLength * sin(rad);
    tft.drawLine(RADAR_PIVOT_X, RADAR_PIVOT_Y, newX, newY, TFT_GREEN);
    
    // Update old coordinates
    oldRadarX = newX;
    oldRadarY = newY;
    
    // Draw center point
    tft.fillCircle(RADAR_PIVOT_X, RADAR_PIVOT_Y, 3, TFT_RED);
    
    // Display distance value
    tft.fillRect(10, 140, 70, 20, TFT_BLACK);
    char distStr[10];
    sprintf(distStr, "%dcm", distance);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawString(distStr, 10, 140, 2);
}