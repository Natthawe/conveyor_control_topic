#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// =========================
// Config
// =========================
byte MAC[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };     // MAC บอร์ด
IPAddress LOCAL_IP(10, 1, 100, 222);                      // IP ของบอร์ดนี้
IPAddress ROS2_IP(10, 1, 100, 100);                       // เครื่อง ROS2 ที่รับสถานะ

// พอร์ตสายพาน
const uint16_t PORT_CONV_LISTEN = 30001;  // รับคำสั่ง
const uint16_t PORT_CONV_STATUS = 30002;  // ส่งสถานะไป ROS2

// พอร์ตรีเลย์
const uint16_t PORT_REL_LISTEN  = 30010;  // รับคำสั่ง
const uint16_t PORT_REL_STATUS  = 30011;  // ส่งสถานะไป ROS2

// =========================
// I/O Pins
// =========================
#define SENSOR_LEFT_PIN      3
#define SENSOR_RIGHT_PIN     2
#define MOTOR_TURN_LEFT_PIN  6
#define MOTOR_TURN_RIGHT_PIN 7
#define LED_NANO             13

// รีเลย์
const int  RELAY1_PIN = 9;
const int  RELAY2_PIN = 8;
const bool RELAY_ACTIVE_LOW = true;  // รีเลย์ Active-LOW

// =========================
// UDP Sockets
// =========================
EthernetUDP UdpConvCmd;   // รับคำสั่งสายพาน
EthernetUDP UdpRelCmd;    // รับคำสั่งรีเลย์
EthernetUDP UdpTx;        // ใช้ส่งสถานะออก (reuse)

// จำปลายทางที่สั่งล่าสุด (สำหรับตอบกลับ/ดีบัก)
IPAddress lastConvIP;  uint16_t lastConvPort = 0;
IPAddress lastRelIP;   uint16_t lastRelPort  = 0;

// =========================
// Timing / Debounce / HB
// =========================
const unsigned long HB_PERIOD_MS       = 500;
const unsigned long SENSOR_DEBOUNCE_MS = 20;
unsigned long lastHBms = 0;
unsigned long lastChangeL = 0, lastChangeR = 0;
int rawL = 0, rawR = 0, stableL = 0, stableR = 0;   // 0/1 (1=มีวัตถุ หลัง map active-low)

// Motor dir report
int currentDir = 0;      // -1 left, 0 stop, 1 right
int lastReportedDir = 0;

// =========================
// Relay State
// =========================
bool relay1On = false;   // ความหมายหลัง map แล้ว (true=เปิด)
bool relay2On = false;

// =========================
// Conveyor Logic (คงเดิมจากโค้ดแรก)
// =========================
enum ConveyorState {
  IDLE,
  MOVING_RIGHT,
  MOVING_LEFT,
  RETURNING_LEFT,
  RETURNING_RIGHT,
  STOP_CASE5
};
ConveyorState conveyorState = IDLE;

bool initialObjectLeft = false;
bool initialObjectRight = false;
bool waitForClearLeft = false;
bool waitForClearRight = false;

bool waitingForItemFromLeft = false;
bool waitingForItemFromRight = false;

unsigned long moveStartTime = 0;
const unsigned long TIMEOUT_DURATION = 10000;

unsigned long stopDelayStartTime = 0;
const unsigned long STOP_DELAY_LEFT  = 3000;
const unsigned long STOP_DELAY_RIGHT = 3000;
const unsigned long STOP_DELAY_CASE5 = 500;
bool delayBeforeStop = false;

bool boxFromLeft = false;
bool boxFromRight = false;

bool passedLeftSensor = false;
bool passedRightSensor = false;

bool passingFromLeftOnly = false;
bool passingFromRightOnly = false;

// =========================
// Utils
// =========================
void ethernetInit() {
  Ethernet.init(10);                 // CS D10
  Ethernet.begin(MAC, LOCAL_IP);
}

void pinInit() {
  pinMode(SENSOR_LEFT_PIN,  INPUT_PULLUP);
  pinMode(SENSOR_RIGHT_PIN, INPUT_PULLUP);
  pinMode(MOTOR_TURN_LEFT_PIN,  OUTPUT);
  pinMode(MOTOR_TURN_RIGHT_PIN, OUTPUT);
  pinMode(LED_NANO, OUTPUT);

  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  // ค่าเริ่มต้นรีเลย์: r1=true, r2=false (ตามโค้ดเดิม)
  relay1On = true;
  relay2On = false;
}

void blinkLED(int n=5, int ms=100) {
  for (int i=0;i<n;i++) {
    digitalWrite(LED_NANO, HIGH); delay(ms);
    digitalWrite(LED_NANO, LOW);  delay(ms);
  }
}

void motorRight() {
  digitalWrite(MOTOR_TURN_LEFT_PIN, LOW);
  digitalWrite(MOTOR_TURN_RIGHT_PIN, HIGH);
  currentDir = 1;
}

void motorLeft() {
  digitalWrite(MOTOR_TURN_LEFT_PIN, HIGH);
  digitalWrite(MOTOR_TURN_RIGHT_PIN, LOW);
  currentDir = -1;
}

void motorStop() {
  digitalWrite(MOTOR_TURN_LEFT_PIN, LOW);
  digitalWrite(MOTOR_TURN_RIGHT_PIN, LOW);
  currentDir = 0;
}

void resetConveyorState() {
  motorStop();
  conveyorState = IDLE;
  boxFromLeft = boxFromRight = false;
  passedLeftSensor = passedRightSensor = false;
  waitForClearLeft = waitForClearRight = false;
  waitingForItemFromLeft = waitingForItemFromRight = false;
  delayBeforeStop = false;
  passingFromLeftOnly = passingFromRightOnly = false;
}

// =========================
// Relay helpers
// =========================
void relayWrite(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(pin, on ? LOW : HIGH);
  else                  digitalWrite(pin, on ? HIGH : LOW);
}

void applyRelayOutputs() {
  relayWrite(RELAY1_PIN, relay1On);
  relayWrite(RELAY2_PIN, relay2On);
}

// =========================
// Status send helpers
// =========================
void sendUDP(IPAddress ip, uint16_t port, const String& s) {
  UdpTx.beginPacket(ip, port);
  UdpTx.write(s.c_str());
  UdpTx.endPacket();
}

// สถานะสายพาน: ส่งไปยัง (1) ROS2 (30002) และ (2) ผู้ส่งล่าสุดของสายพาน (ถ้ามี)
void sendConvStatus(const String& line) {
  // ROS2
  sendUDP(ROS2_IP, PORT_CONV_STATUS, line);
  // Echo ไปปลายทางล่าสุด (เช่น UI/ดีบัก)
  if (lastConvPort != 0) sendUDP(lastConvIP, lastConvPort, line);
}

// สถานะรีเลย์: ส่งไปยัง (1) ROS2 (30011) และ (2) ผู้ส่งล่าสุดของรีเลย์ (ถ้ามี)
void sendRelayStatus(const String& line) {
  // ROS2
  sendUDP(ROS2_IP, PORT_REL_STATUS, line);
  // Echo
  if (lastRelPort != 0) sendUDP(lastRelIP, lastRelPort, line);
}

void reportSensors() {
  String line = "S," + String(stableL) + "," + String(stableR);
  sendConvStatus(line);
}

void reportMotorIfChanged(bool force=false) {
  if (force || currentDir != lastReportedDir) {
    lastReportedDir = currentDir;
    String line = "M," + String(currentDir);
    sendConvStatus(line);
  }
}

void reportRelay() {
  // แปลงสถานะเป็น 0/1 หลัง map แล้ว
  String line = "R," + String(relay1On ? 1 : 0) + "," + String(relay2On ? 1 : 0);
  sendRelayStatus(line);
}

// =========================
// Command parsers
// =========================
int readIntFromUdp(EthernetUDP& udp, int packetSize, char* buf, size_t buflen) {
  int n = udp.read((uint8_t*)buf, min(packetSize, (int)buflen-1));
  if (n <= 0) return 9999;
  if (n == 4) {
    int32_t val; memcpy(&val, buf, 4);
    return (int)val;
  }
  buf[n] = '\0';
  // trim
  while (*buf==' '||*buf=='\t') ++buf;
  int len = strlen(buf);
  while (len>0 && (buf[len-1]=='\r'||buf[len-1]=='\n'||buf[len-1]==' '||buf[len-1]=='\t')) buf[--len]=0;
  return atoi(buf);
}

void processConveyorCommand(int cmd) {
  bool sensorLeft  = (digitalRead(SENSOR_LEFT_PIN)  == LOW);
  bool sensorRight = (digitalRead(SENSOR_RIGHT_PIN) == LOW);

  if (cmd == 1) {
    Serial.println(F("Conv CMD: MOVE_RIGHT"));
    motorRight();
    conveyorState = MOVING_RIGHT;
    boxFromLeft = true;  boxFromRight = false;
    passedLeftSensor = false;
    moveStartTime = millis();

    initialObjectLeft = sensorLeft;
    initialObjectRight = sensorRight;
    waitForClearRight = initialObjectLeft && initialObjectRight;
    waitingForItemFromLeft = (!sensorLeft && !sensorRight);
    passingFromLeftOnly = sensorLeft && !sensorRight;
    reportMotorIfChanged(true);

  } else if (cmd == -1) {
    Serial.println(F("Conv CMD: MOVE_LEFT"));
    motorLeft();
    conveyorState = MOVING_LEFT;
    boxFromRight = true; boxFromLeft = false;
    passedRightSensor = false;
    moveStartTime = millis();

    initialObjectLeft = sensorLeft;
    initialObjectRight = sensorRight;
    waitForClearLeft = initialObjectLeft && initialObjectRight;
    waitingForItemFromRight = (!sensorLeft && !sensorRight);
    passingFromRightOnly = sensorRight && !sensorLeft;
    reportMotorIfChanged(true);

  } else if (cmd == 0) {
    Serial.println(F("Conv CMD: STOP"));
    resetConveyorState();
    reportMotorIfChanged(true);

  } else {
    Serial.print(F("Conv Unknown CMD: ")); Serial.println(cmd);
  }
}

void processRelayCommand(int32_t cmd) {
  // คง mapping จากโค้ดเดิม:
  //  1 -> RED     : R1=ON,  R2=OFF
  // -1 -> YELLOW  : R1=ON,  R2=ON
  //  0 -> GREEN   : R1=OFF, R2=ON
  //  2 -> BLACK   : R1=OFF, R2=OFF
  switch (cmd) {
    case 1:   relay1On = true;  relay2On = false; Serial.println(F("Rel CMD 1: RED"));    break;
    case -1:  relay1On = true;  relay2On = true;  Serial.println(F("Rel CMD -1: YELLOW")); break;
    case 0:   relay1On = false; relay2On = true;  Serial.println(F("Rel CMD 0: GREEN"));   break;
    case 2:   relay1On = false; relay2On = false; Serial.println(F("Rel CMD 2: BLACK"));   break;
    default:  Serial.print(F("Rel Unknown CMD: ")); Serial.println(cmd); return;
  }
  applyRelayOutputs();
  reportRelay();
}

// =========================
// Setup / Loop
// =========================
void setup() {
  delay(200);
  Serial.begin(115200);

  pinInit();
  blinkLED();

  ethernetInit();
  delay(250);

  // เปิดพอร์ต UDP
  bool ok1 = UdpConvCmd.begin(PORT_CONV_LISTEN);
  bool ok2 = UdpRelCmd.begin(PORT_REL_LISTEN);
  if (!ok1 || !ok2) {
    Serial.println(F("ERROR: UDP begin() failed on one of the ports!"));
  }

  Serial.print(F("IP address: ")); Serial.println(Ethernet.localIP());
  Serial.print(F("Listening CONVEYOR CMD @ ")); Serial.println(PORT_CONV_LISTEN);
  Serial.print(F("Listening RELAY CMD    @ ")); Serial.println(PORT_REL_LISTEN);
  Serial.println(F("CONVEYOR status  -> ROS2 @ 30002 (S,L,R / M,dir)"));
  Serial.println(F("RELAY status     -> ROS2 @ 30011 (R,r1,r2)"));

  // init sensor stable values (active-low map → 1=มีของ)
  rawL = (digitalRead(SENSOR_LEFT_PIN)  == LOW) ? 1 : 0;
  rawR = (digitalRead(SENSOR_RIGHT_PIN) == LOW) ? 1 : 0;
  stableL = rawL; stableR = rawR;

  // ตั้งเอาต์พุตรีเลย์ตามสถานะเริ่มต้น
  applyRelayOutputs();

  // ส่งสถานะเริ่มต้น
  reportSensors();
  reportMotorIfChanged(true);
  reportRelay();
}

void loop() {
  unsigned long now = millis();

  // -------------------------
  // รับคำสั่ง "สายพาน"
  // -------------------------
  int szC = UdpConvCmd.parsePacket();
  if (szC > 0) {
    char buf[64];
    int cmd = readIntFromUdp(UdpConvCmd, szC, buf, sizeof(buf));
    Serial.print(F("Conv UDP cmd: ")); Serial.println(cmd);
    processConveyorCommand(cmd);

    lastConvIP   = UdpConvCmd.remoteIP();
    lastConvPort = UdpConvCmd.remotePort();

    // ACK กลับไปยังผู้สั่ง (ดีบัก)
    String ack = String("[ACK Conv] cmd=") + String(cmd);
    sendUDP(lastConvIP, lastConvPort, ack);
  }

  // -------------------------
  // รับคำสั่ง "รีเลย์"
  // -------------------------
  int szR = UdpRelCmd.parsePacket();
  if (szR > 0) {
    char buf[64];
    int cmd = readIntFromUdp(UdpRelCmd, szR, buf, sizeof(buf));
    Serial.print(F("Rel UDP cmd: ")); Serial.println(cmd);
    processRelayCommand((int32_t)cmd);

    lastRelIP   = UdpRelCmd.remoteIP();
    lastRelPort = UdpRelCmd.remotePort();

    // ACK (หากอยากตอบกลับผู้ส่ง)
    String ack = String("[ACK Relay] cmd=") + String(cmd);
    sendUDP(lastRelIP, lastRelPort, ack);
  }

  // -------------------------
  // รับคำสั่งจาก Serial (ทั้งสองระบบ)
  // รูปแบบ: "C:<int>" = คุมสายพาน, "R:<int>" = คุมรีเลย์
  // เช่น C:1 , C:0 , C:-1 หรือ R:1 , R:0 , R:-1 , R:2
  // -------------------------
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.length()) {
      if (s.startsWith("C:")) {
        int v = s.substring(2).toInt();
        processConveyorCommand(v);
      } else if (s.startsWith("R:")) {
        int v = s.substring(2).toInt();
        processRelayCommand(v);
      } else {
        // backward-compat: ถ้าเป็นตัวเลขล้วน ให้ถือว่าเป็น "สายพาน"
        int v = s.toInt();
        processConveyorCommand(v);
      }
    }
  }

  // -------------------------
  // อ่านเซนเซอร์ + debounce
  // -------------------------
  int vL = (digitalRead(SENSOR_LEFT_PIN)  == LOW) ? 1 : 0;
  int vR = (digitalRead(SENSOR_RIGHT_PIN) == LOW) ? 1 : 0;

  if (vL != rawL) { rawL = vL; lastChangeL = now; }
  if (vR != rawR) { rawR = vR; lastChangeR = now; }

  bool changed = false;
  if (rawL != stableL && (now - lastChangeL) >= SENSOR_DEBOUNCE_MS) { stableL = rawL; changed = true; }
  if (rawR != stableR && (now - lastChangeR) >= SENSOR_DEBOUNCE_MS) { stableR = rawR; changed = true; }
  if (changed) reportSensors();

  // -------------------------
  // ลอจิกสายพาน (คงเดิม)
  // -------------------------
  switch (conveyorState) {
    case IDLE:
      break;

    case MOVING_RIGHT:
      if (now - moveStartTime > TIMEOUT_DURATION && !passedLeftSensor) {
        Serial.println(F("[Case0] Timeout (LEFT sensor not triggered)"));
        resetConveyorState(); reportMotorIfChanged(true);
        break;
      }
      if (stableL && !passedLeftSensor) {
        Serial.println(F("Passed sensorLeft"));
        passedLeftSensor = true;
      }
      if (waitingForItemFromLeft && passedLeftSensor && stableR) {
        Serial.println(F("[Case1] Box moved from LEFT to RIGHT"));
        stopDelayStartTime = now; delayBeforeStop = true; conveyorState = STOP_CASE5; passingFromLeftOnly = false;
        break;
      }
      if (waitForClearRight && passedLeftSensor && !stableL && !stableR) {
        Serial.println(F("[Case3] Both sensors cleared after MOVE_RIGHT"));
        stopDelayStartTime = now; delayBeforeStop = true; conveyorState = RETURNING_RIGHT;
        break;
      }
      if (passingFromLeftOnly && stableR) {
        Serial.println(F("[Case5] Left->RIGHT -> Stop"));
        stopDelayStartTime = now; delayBeforeStop = true; conveyorState = STOP_CASE5; passingFromLeftOnly = false;
        break;
      }
      break;

    case MOVING_LEFT:
      if (now - moveStartTime > TIMEOUT_DURATION && !passedRightSensor) {
        Serial.println(F("[Case0] Timeout (RIGHT sensor not triggered)"));
        resetConveyorState(); reportMotorIfChanged(true);
        break;
      }
      if (stableR && !passedRightSensor) {
        Serial.println(F("Passed sensorRight"));
        passedRightSensor = true;
      }
      if (waitingForItemFromRight && passedRightSensor && stableL) {
        Serial.println(F("[Case2] Box moved from RIGHT to LEFT"));
        stopDelayStartTime = now; delayBeforeStop = true; conveyorState = STOP_CASE5; passingFromRightOnly = false;
        break;
      }
      if (waitForClearLeft && passedRightSensor && !stableR && !stableL) {
        Serial.println(F("[Case4] Both sensors cleared after MOVE_LEFT"));
        stopDelayStartTime = now; delayBeforeStop = true; conveyorState = RETURNING_LEFT;
        break;
      }
      if (passingFromRightOnly && stableL) {
        Serial.println(F("[Case5] Right->LEFT -> Stop"));
        stopDelayStartTime = now; delayBeforeStop = true; conveyorState = STOP_CASE5; passingFromRightOnly = false;
        break;
      }
      break;

    case RETURNING_RIGHT:
      if (now - stopDelayStartTime >= STOP_DELAY_RIGHT) {
        Serial.println(F("Stopping after delay (RIGHT)"));
        resetConveyorState(); reportMotorIfChanged(true);
      }
      break;

    case RETURNING_LEFT:
      if (now - stopDelayStartTime >= STOP_DELAY_LEFT) {
        Serial.println(F("Stopping after delay (LEFT)"));
        resetConveyorState(); reportMotorIfChanged(true);
      }
      break;

    case STOP_CASE5:
      if (now - stopDelayStartTime >= STOP_DELAY_CASE5) {
        Serial.println(F("Stopping after delay"));
        resetConveyorState(); reportMotorIfChanged(true);
      }
      break;
  }

  // -------------------------
  // Heartbeat: ส่งสถานะซ้ำทุก 500ms
  // -------------------------
  if (now - lastHBms >= HB_PERIOD_MS) {
    lastHBms = now;
    reportSensors();
    reportMotorIfChanged(); // ส่งเฉพาะตอนเปลี่ยน (เว้นแต่ว่า force=true ตอนเรียก)
    reportRelay();
  }

  delay(5);
}
