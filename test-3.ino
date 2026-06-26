// =============================================================================
//  Citroen C2 VTR 2007 — Hazards Only Test
//  Hardware : Arduino UNO + Keyestudio CAN Shield V2
//             CAN CS = pin 10,  INT = pin 2,  125 kbps
//
//  This sketch only flashes both turn indicators using the captured 0x128 frame.
//  No gauge, temp, fuel, or trip frames are sent.
// =============================================================================

#include <SPI.h>
#include <mcp_can.h>

#define CAN_CS_PIN 10
MCP_CAN CAN(CAN_CS_PIN);

static uint32_t lastFlash = 0;
static bool flashOn = false;

void setup() {
  Serial.begin(115200);
  Serial.println(F("C2 VTR Hazards Only Test"));
  Serial.println(F("Initialising CAN at 125kbps..."));

  while (CAN.begin(MCP_ANY, CAN_125KBPS, MCP_16MHZ) != CAN_OK) {
    Serial.println(F("  Init failed — check wiring or try MCP_8MHZ. Retrying..."));
    delay(1000);
  }

  CAN.setMode(MCP_NORMAL);
  Serial.println(F("  OK — flashing hazards only"));
}

void loop() {
  uint32_t now = millis();

  if (now - lastFlash >= 500) {
    lastFlash = now;
    flashOn = !flashOn;

    // 0x128 hazard flash pattern from capture:
    // a1 90 00 00 c0 80 a0 01  (hazards on)
    // a1 90 00 00 00 80 a0 01  (hazards off)
    uint8_t buf[8] = {
      0xA1,
      0x90,
      0x00,
      0x00,
      flashOn ? 0xC0 : 0x00,
      0x80,
      0xA0,
      0x01
    };

    CAN.sendMsgBuf(0x128, 0, 8, buf);

    Serial.println(flashOn ? F("hazards on") : F("hazards off"));
  }
}