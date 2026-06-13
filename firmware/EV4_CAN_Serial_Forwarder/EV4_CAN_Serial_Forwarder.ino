/*
 * ============================================================
 *  BER (Bearcats Electric Racing) - EV4 dual-CAN <-> Serial bridge
 *
 *  Two MCP2515 boards on ONE shared HSPI bus, each on its own
 *  physical CAN bus:
 *    Bus 0 (board 1): Vehicle bus + Inverter 2 + IMD
 *    Bus 1 (board 2): Inverter 1 (isolated to cut bus load)
 *
 *  --- CAN -> Serial -----------------------------------------
 *  One frame per line, tagged with its bus number:
 *      <bus>:<ID_HEX>#<DATA_HEX>[x]\n
 *  e.g.  0:7#5A14C2003C000000      (bus 0, std id 0x7)
 *        1:A5#1027...              (bus 1, Inverter 1)
 *        0:18FF01F4#...x           (bus 0, extended IMD)
 *
 *  --- Serial -> CAN -----------------------------------------
 *  PC sends ASCII command lines (\n terminated):
 *    H <bus> <idHex> <dataHex>   set repeating heartbeat frame on <bus>
 *    S                           stop the heartbeat
 *    O <bus> <idHex> <dataHex>   send one frame once on <bus>
 *  (Outgoing inverter commands must go on the bus that inverter
 *  lives on — the PC picks <bus>.)
 *
 *  Deadman: if a heartbeat is active but the PC goes silent for
 *  DEADMAN_MS, it's cleared so the motor stops safely.
 *
 *  Baud: 1000000.   Both buses: 500 kbps.
 *  Shared HSPI:  SCK->14  SI->13  SO->12
 *  Board 1 CS->15 (bus 0)   Board 2 CS->27 (bus 1)
 *  10k pull-up on each CS so both idle high at boot.
 * ============================================================
 */

#include <SPI.h>
#include <mcp_can.h>

#define HSPI_SCK  14
#define HSPI_MISO 12
#define HSPI_MOSI 13
#define CAN0_CS   15        // board 1 -> bus 0
#define CAN1_CS   27        // board 2 -> bus 1  (must match the physical CS wire)

// Crystal of each MCP2515 module (read the silver can: 8.000 or 16.000).
#define CAN0_XTAL MCP_8MHZ
#define CAN1_XTAL MCP_8MHZ
#define CAN_SPEED CAN_500KBPS

#define SERIAL_BAUD  1000000   // must match DEFAULT_BAUD in ev4_dashboard.py
#define HEARTBEAT_MS 20
#define DEADMAN_MS   500
#define NUM_BUSES    2

SPIClass hspi(HSPI);
MCP_CAN  CANbus[NUM_BUSES] = { MCP_CAN(&hspi, CAN0_CS), MCP_CAN(&hspi, CAN1_CS) };
const uint8_t CAN_XTAL[NUM_BUSES] = { CAN0_XTAL, CAN1_XTAL };
bool busOk[NUM_BUSES] = { false, false };   // which buses initialized

static const char HEX_DIGITS[] = "0123456789ABCDEF";

// -- Heartbeat (repeating command) state --
bool     hbActive = false;
uint8_t  hbBus = 0;
uint32_t hbId = 0;
bool     hbExt = false;
uint8_t  hbData[8] = {0};
uint32_t hbLastSendMs = 0;
uint32_t hbLastRxMs   = 0;

char    inBuf[64];
uint8_t inLen = 0;

bool tryInitBus(uint8_t b) {
  if (CANbus[b].begin(MCP_ANY, CAN_SPEED, CAN_XTAL[b]) == CAN_OK) {
    CANbus[b].setMode(MCP_NORMAL);
    return true;
  }
  return false;
}

void setupCAN() {
  // Drive both CS lines high BEFORE any SPI activity so neither chip is
  // accidentally selected during the other's init (covers the missing CS
  // pull-ups at boot).
  pinMode(CAN0_CS, OUTPUT); digitalWrite(CAN0_CS, HIGH);
  pinMode(CAN1_CS, OUTPUT); digitalWrite(CAN1_CS, HIGH);
  hspi.begin(HSPI_SCK, HSPI_MISO, HSPI_MOSI, -1);

  // Non-blocking: try each bus a few times, then move on. A failed bus is
  // skipped (and retried in the background) so the working bus still streams.
  for (uint8_t b = 0; b < NUM_BUSES; b++) {
    for (uint8_t tries = 0; tries < 5 && !busOk[b]; tries++) {
      busOk[b] = tryInitBus(b);
      if (!busOk[b]) delay(100);
    }
    Serial.print("# bus "); Serial.print(b);
    Serial.println(busOk[b] ? " ready" : " FAILED (skipped; other bus still runs, will retry)");
  }
}

// Periodically re-init any bus that hasn't come up (e.g. board plugged in later).
void retryDeadBuses() {
  static uint32_t last = 0;
  if (millis() - last < 2000) return;
  last = millis();
  for (uint8_t b = 0; b < NUM_BUSES; b++) {
    if (!busOk[b] && (busOk[b] = tryInitBus(b))) {
      Serial.print("# bus "); Serial.print(b); Serial.println(" came online");
    }
  }
}

// If a transmitting controller hits too many errors it latches BUS-OFF and goes
// silent. Detect it (EFLG bit TXBO = 0x20) and re-init immediately so one bad
// write doesn't black the bus out for seconds.
void checkBusHealth() {
  static uint32_t last = 0;
  if (millis() - last < 250) return;
  last = millis();
  for (uint8_t b = 0; b < NUM_BUSES; b++) {
    if (!busOk[b]) continue;
    if (CANbus[b].getError() & 0x20) {            // TXBO = bus-off
      Serial.print("# bus "); Serial.print(b);
      Serial.println(" BUS-OFF (TX errors) -> reinit");
      busOk[b] = tryInitBus(b);                    // recover now
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(200);
  Serial.println("# EV4 dual-CAN bridge boot");
  setupCAN();
  Serial.println("# init done @500k (see per-bus status above)");
}

// ---- CAN -> Serial ----
void emitFrame(uint8_t bus, uint32_t id, uint8_t len, const uint8_t *buf, bool ext) {
  char line[44];
  uint8_t n = 0;
  line[n++] = '0' + bus;
  line[n++] = ':';
  char idbuf[9]; uint8_t ni = 0; uint32_t tmp = id;
  if (tmp == 0) idbuf[ni++] = '0';
  else while (tmp) { idbuf[ni++] = HEX_DIGITS[tmp & 0xF]; tmp >>= 4; }
  while (ni) line[n++] = idbuf[--ni];
  line[n++] = '#';
  for (uint8_t i = 0; i < len; i++) {
    line[n++] = HEX_DIGITS[(buf[i] >> 4) & 0xF];
    line[n++] = HEX_DIGITS[buf[i] & 0xF];
  }
  if (ext) line[n++] = 'x';
  line[n++] = '\n';
  Serial.write((const uint8_t *)line, n);
}

void pollCANrx() {
  for (uint8_t b = 0; b < NUM_BUSES; b++) {
    if (!busOk[b]) continue;
    while (CANbus[b].checkReceive() == CAN_MSGAVAIL) {
      uint32_t rxId; uint8_t len; uint8_t buf[8];
      if (CANbus[b].readMsgBuf(&rxId, &len, buf) == CAN_OK) {
        bool ext = (rxId & 0x80000000) != 0;
        emitFrame(b, rxId & 0x1FFFFFFF, len, buf, ext);
      }
    }
  }
}

// ---- helpers ----
int hexNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
void parseData(const char *s, uint8_t *out) {
  for (uint8_t i = 0; i < 8; i++) out[i] = 0;
  uint8_t nb = 0;
  while (s[0] && s[1] && nb < 8) {
    int hi = hexNibble(s[0]); int lo = hexNibble(s[1]);
    if (hi < 0 || lo < 0) break;
    out[nb++] = (uint8_t)((hi << 4) | lo);
    s += 2;
  }
}
uint32_t parseHexU32(const char *s) {
  uint32_t v = 0;
  while (*s) { int d = hexNibble(*s); if (d < 0) break; v = (v << 4) | d; s++; }
  return v;
}
void sendCAN(uint8_t bus, uint32_t id, bool ext, uint8_t len, uint8_t *data) {
  if (bus < NUM_BUSES && busOk[bus]) CANbus[bus].sendMsgBuf(id, ext ? 1 : 0, len, data);
}

// next whitespace-separated token, NUL-terminated in place
char *nextTok(char **p) {
  while (**p == ' ') (*p)++;
  char *t = *p;
  while (**p && **p != ' ') (*p)++;
  if (**p) { **p = '\0'; (*p)++; }
  return t;
}

void handleLine(char *line) {
  hbLastRxMs = millis();
  char cmd = line[0];
  if (cmd == 'S' || cmd == 's') { hbActive = false; return; }
  if (cmd != 'H' && cmd != 'h' && cmd != 'O' && cmd != 'o') return;

  char *p = line + 1;
  uint8_t bus = (uint8_t)atoi(nextTok(&p));
  uint32_t id = parseHexU32(nextTok(&p));
  uint8_t data[8];
  parseData(nextTok(&p), data);
  bool ext = (id > 0x7FF);

  if (cmd == 'O' || cmd == 'o') {
    sendCAN(bus, id, ext, 8, data);
  } else {
    hbBus = bus; hbId = id; hbExt = ext;
    for (uint8_t i = 0; i < 8; i++) hbData[i] = data[i];
    hbActive = true;
    sendCAN(hbBus, hbId, hbExt, 8, hbData);
    hbLastSendMs = millis();
  }
}

void pollSerialRx() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (inLen > 0) { inBuf[inLen] = '\0'; handleLine(inBuf); inLen = 0; }
    } else if (inLen < sizeof(inBuf) - 1) {
      inBuf[inLen++] = c;
    }
  }
}

void serviceHeartbeat() {
  uint32_t now = millis();
  if (!hbActive) return;
  if (now - hbLastRxMs > DEADMAN_MS) { hbActive = false; return; }
  if (now - hbLastSendMs >= HEARTBEAT_MS) {
    sendCAN(hbBus, hbId, hbExt, 8, hbData);
    hbLastSendMs = now;
  }
}

void loop() {
  pollCANrx();
  pollSerialRx();
  serviceHeartbeat();
  retryDeadBuses();
  checkBusHealth();
}
