// -1  - YELLOW 
// 0   - GREEN
// 1   - RED
// 2   - BLACK

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

void applyCommand(int8_t cmd) {
  switch (cmd) {
    case 1:  // แดง -> เปิดรีเลย์ 1, ปิดรีเลย์ 2
      relayWrite(RELAY1_PIN, true);
      relayWrite(RELAY2_PIN, false);
      Serial.println(F("CMD -1: Relay1=ON, Relay2=OFF -> RED"));
      break;
    case -1:  // เหลือง -> เปิดทั้งคู่
      relayWrite(RELAY1_PIN, false);
      relayWrite(RELAY2_PIN, false);
      Serial.println(F("CMD 0: Relay1=ON, Relay2=ON -> YELLOW"));
      break;
    case 0:  // เขียว -> ปิดรีเลย์ 1, เปิดรีเลย์ 2
      relayWrite(RELAY1_PIN, false);
      relayWrite(RELAY2_PIN, true);
      Serial.println(F("CMD 1: Relay1=OFF, Relay2=ON -> GREEN"));
      break;
    case 2:  // ปิด LED -> ปิดรีเลย์ทั้งคู่
      relayWrite(RELAY1_PIN, true);
      relayWrite(RELAY2_PIN, true);
      Serial.println(F("CMD 1: Relay1=OFF, Relay2=OFF -> BLACK"));
      break;      
    default:
      Serial.print(F("Unknown CMD: "));
      Serial.println(cmd);
  }
}

int8_t parseCmdFromPacket(int packetSize) {
  uint8_t buf[16];
  int len = Udp.read(buf, min(packetSize, (int)sizeof(buf) - 1));
  if (len <= 0) return 2; // invalid

  // กรณี 1: ไบต์เดียวแบบ binary int8 (-1/0/1/2)
  if (len == 1 && (buf[0] == 0xFF || buf[0] == 0x00 || buf[0] == 0x01)) {
    if (buf[0] == 0xFF) return -1;
    if (buf[0] == 0x00) return 0;
    return 1; // 0x01
  }

  // กรณี 2: ข้อความ ASCII เช่น "-1", "0", "1", "2"
  buf[len] = 0;
  char *s = (char*)buf;

  // trim ซ้าย/ขวา
  while (*s == ' ' || *s == '\t') s++;
  while (len > 0 && (s[len-1] == '\r' || s[len-1] == '\n' || s[len-1] == ' ' || s[len-1] == '\t')) {
    s[--len] = 0;
  }

  if (!strcmp(s, "-1")) return -1;
  if (!strcmp(s, "0"))  return 0;
  if (!strcmp(s, "1"))  return 1;
  if (!strcmp(s, "2"))  return 2;

  // atoi เผื่อมีเว้นวรรค
  int v = atoi(s);
  if (v == -1 || v == 0 || v == 1 || v == 2) return (int8_t)v;

  // debug: แสดงค่าไบต์ดิบ
  Serial.print(F("UDP raw bytes: "));
  for (int i = 0; i < len; i++) { Serial.print((int)buf[i]); Serial.print(' '); }
  Serial.println();
  return 2; // invalid
}


void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println(F("\n=== Nano UDP Relay Controller ==="));
  Serial.println(F(" -> -1 = YELLOW\n ->  0 = GREEN\n ->  1 = RED\n ->  2 = BLACK"));

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  relayWrite(RELAY1_PIN, false);
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
  // ===== UDP receive (1-byte signed int: -1/0/1) =====
  //   int packetSize = Udp.parsePacket();
  //   if (packetSize > 0) {
  //     int8_t cmd = 2; // invalid init
  //     if (packetSize >= 1) {
  //       uint8_t b;
  //       Udp.read(&b, 1);
  //       while (Udp.available()) Udp.read();
  //       cmd = (int8_t)b;
  //     }
  //     applyCommand(cmd);
  //   }

  int packetSize = Udp.parsePacket();
  if (packetSize > 0) {
    int8_t cmd = parseCmdFromPacket(packetSize);
    applyCommand(cmd);
  }
  // ===== Serial test: พิมพ์ -1 / 0 / 1 / 2แล้ว Enter =====
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length()) {
      int val = s.toInt();  // รองรับ -1, 0, 1, 2
      applyCommand((int8_t)val);
    }
  }
}
