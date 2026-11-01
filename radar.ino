#include <Arduino_GFX_Library.h>

#define LCD_BL 14  // backlight pin, active low

// colors (rgb565)
#define OSD_BG        0x0000  // black
#define OSD_DIM       0x0210  // dim teal, used for grid and inactive stuff
#define OSD_CYAN      0x07FF  // main cyan
#define OSD_AMBER     0xFD20  // amber
#define OSD_YELLOW    0xFFE0  // bright yellow so the numbers pop
#define OSD_MAGENTA   0xF81F  // strong rf gate bars
#define OSD_WHITE     0xFFFF  // white for markers and static state

// sensor tuning
#define MIN_GATE_ENERGY     35     // ignore anything weaker than this
#define SPEED_DEADBAND_KMH  0.5f   // treat tiny sway under 0.5 km/h as standing still
#define MAX_RANGE_CM        1000   // full scale is 10 m

// spi pins for the waveshare esp32-s3-lcd-1.9
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    11 /* DC */, 12 /* CS */, 10 /* SCK */, 13 /* MOSI */, -1 /* MISO */, FSPI
);

Arduino_GFX *gfx = new Arduino_ST7789(
    bus, 9 /* RST */, 1 /* rotation: landscape 320x170 */, true /* IPS */, 
    170, 320, 35, 0, 35, 0
);

// uart pins that i checked: RX=1 (blue wire), TX=2 (orange wire)
#define RADAR_RX_PIN 1
#define RADAR_TX_PIN 2

// command that switches the radar into report mode
const uint8_t CMD_ENABLE_REPORT_MODE[] = {
  0xFD, 0xFC, 0xFB, 0xFA, 
  0x08, 0x00, 0x12, 0x00, 
  0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 
  0x04, 0x03, 0x02, 0x01
};

// current readings
uint8_t  targetState = 0;
uint16_t rawDistanceCm = 0;
float    smoothDistCm = 0;
float    prevSmoothDistCm = 0;
float    velocityKmh = 0.0f;   // signed, negative means coming toward us
float    absSpeedKmh = 0.0f;   // just the magnitude
uint16_t gateEnergy[16] = {0};
uint16_t activeEnergy = 0;

// direction state: 0 = nothing, 1 = inbound (-), 2 = outbound (+), 3 = static
uint8_t  vectorMode = 0;

// last 5 distances for the median filter
uint16_t distHistory[5] = {0};
uint8_t  histIdx = 0;
unsigned long lastVelTime = 0;
unsigned long lastDraw = 0;

// forward declarations
uint16_t getMedian5(uint16_t *arr);
void drawCornerBracket(int x, int y, int w, int h, int len, uint16_t col);
void drawStaticOSD();
void processRadarFrame(uint8_t state, uint16_t distCm);
void parseRadarStream();
void updateOSD();

void setup() {
  Serial.begin(115200);
  Serial1.setRxBufferSize(1024);
  Serial1.begin(115200, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  pinMode(LCD_BL, OUTPUT);
  digitalWrite(LCD_BL, LOW);

  gfx->begin();
  gfx->fillScreen(OSD_BG);

  // give the radar a moment to boot before sending the command
  delay(200);
  Serial1.write(CMD_ENABLE_REPORT_MODE, sizeof(CMD_ENABLE_REPORT_MODE));

  drawStaticOSD();
}

// sorts a copy of the array and returns the middle value
uint16_t getMedian5(uint16_t *arr) {
  uint16_t temp[5];
  memcpy(temp, arr, sizeof(temp));
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (temp[j] < temp[i]) {
        uint16_t swap = temp[i];
        temp[i] = temp[j];
        temp[j] = swap;
      }
    }
  }
  return temp[2];
}

// draws just the four corners of a box instead of the whole outline
void drawCornerBracket(int x, int y, int w, int h, int len, uint16_t col) {
  gfx->drawFastHLine(x, y, len, col);
  gfx->drawFastVLine(x, y, len, col);
  gfx->drawFastHLine(x + w - len, y, len, col);
  gfx->drawFastVLine(x + w - 1, y, len, col);
  gfx->drawFastHLine(x, y + h - 1, len, col);
  gfx->drawFastVLine(x, y + h - len, len, col);
  gfx->drawFastHLine(x + w - len, y + h - 1, len, col);
  gfx->drawFastVLine(x + w - 1, y + h - len, len, col);
}

// stuff that only needs to be drawn once
void drawStaticOSD() {
  // title bar
  gfx->setTextColor(OSD_CYAN);
  gfx->setTextSize(1);
  gfx->setCursor(6, 4);
  gfx->print("SYS.OPTIC // SPATIAL TELEMETRY NODE_24G (10m)");
  gfx->drawFastHLine(0, 15, 320, OSD_DIM);
  gfx->drawFastHLine(0, 16, 85, OSD_CYAN);

  // depth scale labels on the left, 0 to 10 m
  gfx->setTextColor(OSD_DIM);
  gfx->setCursor(6, 24);  gfx->print("0m");
  gfx->setCursor(6, 44);  gfx->print("2m");
  gfx->setCursor(6, 64);  gfx->print("4m");
  gfx->setCursor(6, 84);  gfx->print("6m");
  gfx->setCursor(6, 104); gfx->print("8m");
  gfx->setCursor(6, 124); gfx->print("10m");

  // corner brackets around the three readout boxes
  drawCornerBracket(82, 22, 114, 62, 8, OSD_CYAN);   // distance
  drawCornerBracket(202, 22, 112, 62, 8, OSD_CYAN);  // speed
  drawCornerBracket(82, 90, 232, 36, 6, OSD_DIM);    // direction

  // line above the gate bars at the bottom
  gfx->drawFastHLine(6, 134, 308, OSD_DIM);
  gfx->setTextColor(OSD_DIM);
  gfx->setCursor(6, 138);
  gfx->print("RF.ECHO");
}

// takes one decoded frame and works out distance, speed and direction
void processRadarFrame(uint8_t state, uint16_t distCm) {
  targetState = state;
  rawDistanceCm = distCm;

  // each gate is roughly 70 cm wide
  int centerGate = constrain(rawDistanceCm / 70, 0, 15);
  activeEnergy = gateEnergy[centerGate];

  // no target, or the echo is too weak to trust, so reset everything
  if (targetState == 0 || rawDistanceCm == 0 || activeEnergy < MIN_GATE_ENERGY) {
    vectorMode = 0;
    absSpeedKmh = 0.0f;
    velocityKmh = 0.0f;
    return;
  }

  // median filter first to knock out random spikes
  distHistory[histIdx] = rawDistanceCm;
  histIdx = (histIdx + 1) % 5;
  uint16_t medianCm = getMedian5(distHistory);

  // then a bit of smoothing on top
  if (smoothDistCm == 0) {
    // first valid reading, just seed the filter with it
    smoothDistCm = medianCm;
    prevSmoothDistCm = medianCm;
    lastVelTime = millis();
  } else {
    smoothDistCm = (smoothDistCm * 0.65f) + ((float)medianCm * 0.35f);
  }

  unsigned long now = millis();
  float dt = (now - lastVelTime) / 1000.0f;

  // only recalculate speed every 200 ms or so, otherwise it's too noisy
  if (dt >= 0.20f) {
    float deltaMeters = (smoothDistCm - prevSmoothDistCm) / 100.0f;
    float instantMs = deltaMeters / dt;
    float instantKmh = instantMs * 3.6f;

    // smooth the speed too so the number doesn't jump around
    velocityKmh = (velocityKmh * 0.55f) + (instantKmh * 0.45f);
    absSpeedKmh = fabs(velocityKmh);

    if (absSpeedKmh < SPEED_DEADBAND_KMH) {
      absSpeedKmh = 0.0f;
      vectorMode = 3; // static
    } else if (velocityKmh < -SPEED_DEADBAND_KMH) {
      vectorMode = 1; // inbound
    } else {
      vectorMode = 2; // outbound
    }

    prevSmoothDistCm = smoothDistCm;
    lastVelTime = now;
  }
}

// reads the serial stream and pulls out complete frames
void parseRadarStream() {
  static uint8_t buf[64];
  static uint8_t idx = 0;
  static uint8_t headerStep = 0;

  while (Serial1.available()) {
    uint8_t b = Serial1.read();

    // walk through the 4 byte header F4 F3 F2 F1 one byte at a time
    if (headerStep == 0 && b == 0xF4) { headerStep = 1; continue; }
    if (headerStep == 1 && b == 0xF3) { headerStep = 2; continue; }
    if (headerStep == 2 && b == 0xF2) { headerStep = 3; continue; }
    if (headerStep == 3 && b == 0xF1) { headerStep = 4; idx = 0; continue; }

    // header matched, now collect the payload
    if (headerStep == 4) {
      if (idx < sizeof(buf)) {
        buf[idx++] = b;

        // check if the last 4 bytes are the footer F8 F7 F6 F5
        if (idx >= 4 && buf[idx-4] == 0xF8 && buf[idx-3] == 0xF7 && buf[idx-2] == 0xF6 && buf[idx-1] == 0xF5) {
          // make sure the frame is long enough to be valid
          if (idx >= 39) {
            uint8_t state = buf[2];
            uint16_t dist = buf[3] | (buf[4] << 8);
            for (int g = 0; g < 16; g++) {
              gateEnergy[g] = buf[5 + g * 2] | (buf[6 + g * 2] << 8);
            }
            processRadarFrame(state, dist);
          }
          headerStep = 0;
          idx = 0;
        }
      } else {
        // buffer overflowed without a footer, throw it away and start over
        headerStep = 0;
        idx = 0;
      }
    } else {
      // lost sync, but if this byte could be a new header start then keep it
      headerStep = (b == 0xF4) ? 1 : 0;
    }
  }
}

// redraws everything that changes
void updateOSD() {
  // left depth gauge (10 m full scale)
  gfx->fillRect(24, 22, 50, 108, OSD_BG);
  gfx->drawFastVLine(48, 24, 102, OSD_DIM);

  for (int y = 26; y <= 122; y += 10) {
    gfx->drawFastHLine(44, y, 9, OSD_DIM);
  }

  // wipe the inside of the readout boxes
  gfx->fillRect(86, 26, 106, 54, OSD_BG);  
  gfx->fillRect(206, 26, 104, 54, OSD_BG); 
  gfx->fillRect(86, 94, 224, 28, OSD_BG);  

  if (vectorMode > 0 && smoothDistCm > 0) {
    // pick a color based on direction
    uint16_t vecColor = OSD_CYAN;
    if (vectorMode == 1) vecColor = OSD_AMBER;       
    else if (vectorMode == 2) vecColor = OSD_CYAN;   
    else vecColor = OSD_WHITE;                       

    // map 0 to 1000 cm onto the height of the gauge
    int clampedCm = constrain((int)smoothDistCm, 10, MAX_RANGE_CM);
    int markerY = map(clampedCm, 10, MAX_RANGE_CM, 26, 128);

    gfx->fillRect(28, markerY - 2, 40, 5, vecColor);
    gfx->fillTriangle(24, markerY - 4, 24, markerY + 4, 30, markerY, OSD_WHITE);
    gfx->fillTriangle(72, markerY - 4, 72, markerY + 4, 66, markerY, OSD_WHITE);

    // range box (meters)
    float distM = smoothDistCm / 100.0f;
    gfx->setTextSize(1);
    gfx->setTextColor(OSD_CYAN);
    gfx->setCursor(90, 28);
    gfx->print("RANGE (m)");

    gfx->setTextSize(3);
    gfx->setTextColor(OSD_YELLOW);
    gfx->setCursor(90, 46);
    gfx->printf("%.2f", distM);

    // speed box (km/h)
    gfx->setTextSize(1);
    gfx->setTextColor(OSD_CYAN);
    gfx->setCursor(210, 28);
    gfx->print("SPEED (km/h)");

    gfx->setTextSize(3);
    gfx->setTextColor(vecColor);
    gfx->setCursor(210, 46);
    gfx->printf("%.1f", absSpeedKmh);

    // direction box
    gfx->setTextSize(1);
    gfx->setTextColor(OSD_DIM);
    gfx->setCursor(90, 96);
    gfx->print("TRAJECTORY VECTOR");

    gfx->setTextSize(2);
    gfx->setTextColor(vecColor);
    gfx->setCursor(90, 106);
    if (vectorMode == 1) {
      gfx->print("<< INBOUND   [-]");
    } else if (vectorMode == 2) {
      gfx->print(">> OUTBOUND  [+]");
    } else {
      gfx->print("== STATIC    [0]");
    }

  } else {
    // nothing detected, show the idle placeholders
    gfx->setTextSize(1);
    gfx->setTextColor(OSD_DIM);
    gfx->setCursor(90, 28);
    gfx->print("RANGE (m)");
    gfx->setTextSize(3);
    gfx->setCursor(90, 46);
    gfx->print("---");

    gfx->setTextSize(1);
    gfx->setCursor(210, 28);
    gfx->print("SPEED (km/h)");
    gfx->setTextSize(3);
    gfx->setCursor(210, 46);
    gfx->print("0.0");

    gfx->setTextSize(2);
    gfx->setTextColor(OSD_DIM);
    gfx->setCursor(90, 102);
    gfx->print("IDLE // NO ECHO");
  }

  // bottom row, one bar per rf gate
  gfx->fillRect(55, 138, 260, 30, OSD_BG);
  for (int g = 0; g < 16; g++) {
    int bx = 58 + (g * 16);
    int barH = map(constrain(gateEnergy[g], 0, 1800), 0, 1800, 2, 26);
    // strong echoes go magenta, weak ones stay cyan
    uint16_t barCol = (gateEnergy[g] > 350) ? OSD_MAGENTA : OSD_CYAN;
    gfx->fillRect(bx, 166 - barH, 11, barH, barCol);
  }
}

void loop() {
  parseRadarStream();
  // redraw about every 60 ms
  if (millis() - lastDraw > 60) {
    lastDraw = millis();
    updateOSD();
  }
}
