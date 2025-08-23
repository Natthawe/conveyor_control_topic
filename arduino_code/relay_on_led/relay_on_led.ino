#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// ===== Ethernet =====
byte MAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01 };
IPAddress LOCAL_IP(10, 1, 100, 211);    // Arduino IP
const uint16_t UDP_PORT_LISTEN = 8001;  // UDP port
EthernetUDP Udp;

// ===== Relay pins & polarity =====
const int RELAY1_PIN = 8;
const int RELAY2_PIN = 9;
const bool RELAY_ACTIVE_LOW = true;  // Active-LOW

void relayWrite(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else digitalWrite(pin, on ? HIGH : LOW);
}

void applyCommand(int32_t cmd) {
  switch (cmd) {
    case 1:  // RED -> open Relay1, close Relay2
      relayWrite(RELAY1_PIN, false);
      relayWrite(RELAY2_PIN, true);
      Serial.println(F("CMD 1: Relay1=ON, Relay2=OFF -> RED"));
      break;
    case -1:  // YELLOW -> open both relays
      relayWrite(RELAY1_PIN, false);
      relayWrite(RELAY2_PIN, false);
      Serial.println(F("CMD -1: Relay1=ON, Relay2=ON -> YELLOW"));
      break;
    case 0:  // GREEN -> close Relay1, open Relay2
      relayWrite(RELAY1_PIN, true);
      relayWrite(RELAY2_PIN, false);      
      Serial.println(F("CMD 0: Relay1=OFF, Relay2=ON -> GREEN"));
      break;
    case 2:  // BLACK (OFF) -> close both relays
      relayWrite(RELAY1_PIN, true);
      relayWrite(RELAY2_PIN, true);
      Serial.println(F("CMD 2: Relay1=OFF, Relay2=OFF -> BLACK"));
      break;      
    default:
      Serial.print(F("Unknown CMD: "));
      Serial.println(cmd);
  }
}

int32_t parseCmdFromPacket(int packetSize) {
  uint8_t buf[32];
  int len = Udp.read(buf, min(packetSize, (int)sizeof(buf) - 1));
  if (len <= 0) return 9999; // invalid marker

  // ---- กรณี 1: Binary int32 (4 bytes, little endian) ----
  if (len == 4) {
    int32_t val;
    memcpy(&val, buf, 4);
    return val;
  }

  // ---- กรณี 2: ASCII ข้อความ ----
  buf[len] = 0;
  char *s = (char*)buf;

  // trim ซ้าย/ขวา
  while (*s == ' ' || *s == '\t') s++;
  while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ' || s[len-1] == '\t')) {
    s[--len] = 0;
  }

  int v = atoi(s);
  return v;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\n=== Nano UDP Relay Controller (Int32) ==="));
  Serial.println(F(" -> -1 = YELLOW\n -> 0 = GREEN\n -> 1 = RED\n -> 2 = BLACK"));

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  relayWrite(RELAY1_PIN, true);
  relayWrite(RELAY2_PIN, false);  

  Ethernet.init(10);  // CS pin D10
  Ethernet.begin(MAC, LOCAL_IP);
  delay(1000);
  Serial.print(F("IP address: "));
  Serial.println(Ethernet.localIP());

  if (Udp.begin(UDP_PORT_LISTEN)) {
    Serial.print(F("Listening UDP port "));
    Serial.println(UDP_PORT_LISTEN);
  } else {
    Serial.println(F("UDP begin() FAILED!"));
  }
}

void loop() {
  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    int32_t cmd = parseCmdFromPacket(packetSize);
    applyCommand(cmd);
  }

  // Serial test
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length()) {
      int val = s.toInt();
      applyCommand((int32_t)val);
    }
  }
}
