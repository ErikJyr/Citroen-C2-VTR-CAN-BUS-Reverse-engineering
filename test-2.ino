// =============================================================================
//  Citroen C2 VTR 2007 — Gauge Cluster Test Sketch
//  Hardware : Arduino UNO + Keyestudio CAN Shield V2
//             CAN CS = pin 10,  INT = pin 2,  125 kbps
//
//  All signals confirmed against real CAN dump + autowp.github.io database.
//
//  What animates:
//    RPM   : 800 → 6000 → 800 (sine sweep, ~10s cycle)
//    Speed : 0   → 100  → 0   (tracks RPM)
//    Coolant: warms from 40°C to 95°C over ~60s then holds
//
//  What is static (real values from capture):
//    Fuel level    : 78% (0xC8 in 0x14C)
//    Outside temp  : 30°C
//    Odometer      : 69 km (0x001AF4 / 100)
//    Range on fuel : 260 km
//    Consumption   : 10.0 L/100km
//    Ignition      : ON
//    Sidelights    : ON (so cluster illuminates)
//    All warnings  : OFF
//
//  Libraries: mcp_can by Cory J. Fowler (Library Manager)
//
//  Serial monitor at 115200 baud shows live values.
//  If CAN init loops: try changing MCP_16MHZ → MCP_8MHZ below.
// =============================================================================

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN  10
MCP_CAN CAN(CAN_CS_PIN);

// ── Timing ───────────────────────────────────────────────────────────────────
uint32_t last0B6  = 0;   // 50ms  — RPM + speed (primary gauge frame)
uint32_t last036  = 0;   // 100ms — BSI ignition / dash lighting
uint32_t last128  = 0;   // 200ms — warning lights / indicators
uint32_t last168  = 0;   // 200ms — coolant temp (gauge needle)
uint32_t last14C  = 0;   // 100ms — fuel level (gauge needle)
uint32_t last0F6  = 0;   // 500ms — outside temp / odometer / reverse
uint32_t last221  = 0;   // 1000ms — trip computer (range / consumption)
uint32_t last13E  = 0;   // 100ms — secondary RPM/speed frame
uint32_t last167  = 0;   // 100ms — BSI status
uint32_t last17E  = 0;   // 100ms — BSI alive
uint32_t last3F6  = 0;   // 100ms — recurring captured frame
uint32_t lastAnim = 0;   // animation step timer

// ── Animation ─────────────────────────────────────────────────────────────────
#define N_STEPS        20
#define STEP_MS       500    // 500ms per step = 10s full sweep

// RPM raw = RPM * 8  (confirmed: 0xB6 bytes[0:1] >> 3 = RPM)
// Speed raw = km/h * 100  (confirmed: 0xB6 bytes[2:3] / 100 = km/h)
// Sine sweep: 800→6000→800 RPM,  0→100→0 km/h
const uint16_t RPM_RAW[N_STEPS] = {
   6400,  12152,  18104,  23872,  29320,  33880,  37640,  40376,  42072,  42800,
  42800,  42072,  40376,  37640,  33880,  29320,  23872,  18104,  12152,   6400
};
const uint16_t SPD_RAW[N_STEPS] = {
      0,   1974,   3896,   5710,   7370,   8828,  10044,  10988,  11632,  11958,
  11958,  11632,  10988,  10044,   8828,   7370,   5710,   3896,   1974,      0
};

uint8_t  animStep   = 0;
uint32_t lastStep   = 0;

// ── Coolant warmup ────────────────────────────────────────────────────────────
// 0x168 byte[0] - 40 = °C  →  byte = °C + 40
// 0x14C byte[0]: cold start shows 0x50, running shows 0x8C (100°C)
// We'll warm from 0x50 (40°C) to 0x8C (100°C) gradually... but
// for the TEST we use 0xF6 coolant formula (byte[1] - 39 = outside temp)
// Engine coolant for gauge: 0x168 byte[0], warm from 80 (0x80=88°C) to 0x8C (100°C)
uint8_t coolant168  = 0x50;   // starts at 0x50 (40°C gauge)
uint32_t lastTempUp = 0;

// ── Rolling counter for 0xB6 byte[6] ─────────────────────────────────────────
// CRITICAL: cluster detects frozen counter and ignores RPM/speed
uint8_t b6Counter = 0x00;

// ── setup() ──────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("C2 VTR Gauge Cluster Test"));
  Serial.println(F("Initialising CAN at 125kbps..."));

  while (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("  Init failed — check wiring or try MCP_8MHZ. Retrying..."));
    delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
  Serial.println(F("  OK — broadcasting"));
  Serial.println(F(""));
  Serial.println(F("   RPM  | km/h | Coolant | Step"));
}

// ── loop() ───────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ── Advance animation step ─────────────────────────────────────────────────
  if (now - lastStep >= STEP_MS) {
    lastStep = now;
    animStep = (animStep + 1) % N_STEPS;
  }

  // ── Warm up coolant ────────────────────────────────────────────────────────
  if (now - lastTempUp >= 1000) {
    lastTempUp = now;
    if (coolant168 < 0x8C) coolant168++;
  }

  // ── 0x0B6 — RPM + Speed + fuel counter  (50ms) ───────────────────────────
  // CONFIRMED: bytes[0:1] = RPM*8, bytes[2:3] = speed*100
  //            byte[6]    = rolling counter (MUST increment or cluster ignores)
  //            byte[7]    = 0xD0 engine running
  if (now - last0B6 >= 50) {
    last0B6 = now;
    b6Counter++;   // increment every frame
    uint16_t rpmRaw = RPM_RAW[animStep];
    uint16_t spdRaw = SPD_RAW[animStep];
    uint8_t buf[8] = {
      (uint8_t)(rpmRaw >> 8),
      (uint8_t)(rpmRaw & 0xFF),
      (uint8_t)(spdRaw >> 8),
      (uint8_t)(spdRaw & 0xFF),
      0x00,          // odometer from start high byte (cm)
      0x00,          // odometer from start low byte
      b6Counter,     // fuel consumption counter — MUST roll
      0xD0           // engine running
    };
    CAN.sendMsgBuf(0x0B6, 0, 8, buf);
  }

  // ── 0x036 — BSI ignition + dashboard lighting  (100ms) ───────────────────
  // byte[3] = 0x0F: brightness bits[3:0]=15 (max), L=0 (lights off in sim)
  // byte[4] = 0x21: ignition ON (MMM=001), bit5=settled flag
  // byte[7] = 0xA0: fixed running value
  if (now - last036 >= 100) {
    last036 = now;
    uint8_t buf[8] = {
      0x00, 0x00, 0x00, 0x0F, 0x21, 0x00, 0x00, 0xA0
    };
    CAN.sendMsgBuf(0x036, 0, 8, buf);
  }

  // ── 0x128 — Dashboard warning lights  (200ms) ─────────────────────────────
  // byte[0]: bit5=parking brake OFF (0x00), bit6=seatbelt warn OFF
  // byte[1]: bit4=door open OFF, bit7=fixed
  // byte[4]: bit7=low fuel OFF, bit6=sidelights ON (0x40 so cluster illuminates)
  //          bit5=low beam OFF, bit4=high beam OFF
  //          bit3=front fog OFF, bit2=rear fog OFF, bit1=right ind OFF, bit0=left ind OFF
  // byte[5]: 0x80 fixed
  // byte[6]: 0xA0 fixed
  // byte[7]: bits[3:0]=brightness=1
  if (now - last128 >= 200) {
    last128 = now;
    uint8_t buf[8] = {
      0x80,   // bit7 fixed, parking brake OFF, seatbelt OFF
      0x80,   // bit7 fixed, door closed
      0x00,
      0x00,
      0x40,   // sidelights ON (bit6) so cluster illuminates — all warnings OFF
      0x80,   // fixed
      0xA0,   // fixed
      0x01    // brightness level 1
    };
    CAN.sendMsgBuf(0x128, 0, 8, buf);
  }

  // ── 0x168 — Engine coolant temp gauge  (200ms) ───────────────────────────
  // CONFIRMED: byte[0] - 40 = °C
  // Warms from 40°C (0x50) up to 100°C (0x8C)
  if (now - last168 >= 200) {
    last168 = now;
    uint8_t buf[8] = {
      coolant168, 0x00, 0x00, 0xA2, 0x24, 0x01, 0x00, 0x00
    };
    CAN.sendMsgBuf(0x168, 0, 8, buf);
  }

  // ── 0x14C — Fuel level gauge  (100ms) ────────────────────────────────────
  // byte[0] / 2.55 = % fuel.  0xC8 = 200 = ~78%
  if (now - last14C >= 100) {
    last14C = now;
    uint8_t buf[5] = {
      0xC8, 0x00, 0x00, 0x00, 0x00
    };
    CAN.sendMsgBuf(0x14C, 0, 5, buf);
  }

  // ── 0x0F6 — Outside temp + odometer + reverse  (500ms) ──────────────────
  // byte[0]: 0x8E = ignition on, settled
  // byte[1]: outside temp = C-39, 30°C = 0x45 (39+30=69=0x45... wait: 30+39=69=0x45)
  // bytes[2:4]: odometer 24-bit, unit = 0.01km, so 69km = 6900 = 0x001AF4
  // byte[5]: 0x8E fixed
  // byte[6]: outside temp alt formula round(T/2-39.5), 30°C -> T=139=0x8B
  // byte[7]: 0x20 = no indicators, no reverse (bit7=reverse, bit1=right, bit0=left)
  if (now - last0F6 >= 500) {
    last0F6 = now;
    uint8_t buf[8] = {
      0x8E,          // ignition on, settled
      0x45,          // outside temp: 30+39 = 69 = 0x45 -> 69-39 = 30°C ✓
      0x00, 0x1A, 0xF4,  // odometer: 6900 = 69km (/100)
      0x8E,          // fixed
      0x8B,          // outside temp alt: round(139/2-39.5)=30°C ✓
      0x20           // no reverse, no indicators
    };
    CAN.sendMsgBuf(0x0F6, 0, 8, buf);
  }

  // ── 0x221 — Trip computer  (1000ms) ──────────────────────────────────────
  // byte[0]: 0x00 = valid data
  // bytes[1:2]: L/100km * 10 = 0x0064 = 100 = 10.0 L/100km
  // bytes[3:4]: range on fuel = 260km = 0x0104
  // bytes[5:6]: range to destination = 0xFFFF = no destination
  if (now - last221 >= 1000) {
    last221 = now;
    uint8_t buf[7] = {
      0x00,
      0x00, 0x64,    // 10.0 L/100km
      0x01, 0x04,    // 260 km range
      0xFF, 0xFF     // no destination
    };
    CAN.sendMsgBuf(0x221, 0, 7, buf);
  }

  // ── 0x13E — Secondary RPM/speed frame  (100ms) ───────────────────────────
  // byte[0]: rolling counter (seen 0x44-0x4B in capture, increments ~every 10s)
  // bytes[1:2]: RPM * 4 (secondary formula, different from 0xB6)
  // byte[4]: 0xC1 = engine running stationary, 0x01 = driving
  // bytes[5:6]: speed * 100
  if (now - last13E >= 100) {
    last13E = now;
    static uint8_t ctr13E = 0x44;
    static uint32_t lastCtr = 0;
    if (now - lastCtr >= 10000) { lastCtr = now; if (ctr13E < 0x4B) ctr13E++; }

    uint16_t rpmRaw4 = (RPM_RAW[animStep] / 8) * 4;  // convert back from *8 to *4
    uint16_t spdRaw  = SPD_RAW[animStep];
    uint8_t  b4      = (spdRaw > 0) ? 0x01 : 0xC1;   // driving vs stationary

    uint8_t buf[8] = {
      ctr13E,
      (uint8_t)(rpmRaw4 >> 8), (uint8_t)(rpmRaw4 & 0xFF),
      0xFF,
      b4,
      (uint8_t)(spdRaw >> 8), (uint8_t)(spdRaw & 0xFF),
      0x00
    };
    CAN.sendMsgBuf(0x13E, 0, 8, buf);
  }

  // ── 0x167 — BSI status  (100ms) ──────────────────────────────────────────
  if (now - last167 >= 100) {
    last167 = now;
    uint8_t buf[8] = {
      0x08, 0x06, 0xFF, 0xFF, 0x7F, 0xFF, 0x00, 0x00
    };
    CAN.sendMsgBuf(0x167, 0, 8, buf);
  }

  // ── 0x17E — BSI alive  (100ms) ───────────────────────────────────────────
  if (now - last17E >= 100) {
    last17E = now;
    uint8_t buf[8] = {
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    CAN.sendMsgBuf(0x17E, 0, 8, buf);
  }

  // ── 0x3F6 — recurring captured frame  (100ms) ────────────────────────────
  // Capture shows 02 50 E0 16 00 80 01
  if (now - last3F6 >= 100) {
    last3F6 = now;
    uint8_t buf[7] = {
      0x02, 0x50, 0xE0, 0x16, 0x00, 0x80, 0x01
    };
    CAN.sendMsgBuf(0x3F6, 0, 7, buf);
  }

  // ── Serial readout every 500ms ────────────────────────────────────────────
  static uint32_t lastPrint = 0;
  if (now - lastPrint >= 500) {
    lastPrint = now;
    uint16_t rpm   = RPM_RAW[animStep] / 8;
    uint16_t speed = SPD_RAW[animStep] / 100;
    int8_t   temp  = (int8_t)coolant168 - 40;
    Serial.print(F("  "));
    if (rpm < 1000) Serial.print(' ');
    if (rpm < 100)  Serial.print(' ');
    Serial.print(rpm);
    Serial.print(F("  |  "));
    if (speed < 100) Serial.print(' ');
    if (speed < 10)  Serial.print(' ');
    Serial.print(speed);
    Serial.print(F("  |   "));
    Serial.print(temp);
    Serial.print(F("°C    |  step "));
    Serial.println(animStep);
  }
}