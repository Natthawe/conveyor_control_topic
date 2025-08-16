/*
  Dual TB6600 — rotate exactly 90° per command
  Command: -1 = 90° CCW, 0 = abort/stop, 1 = 90° CW
  - Two motors step together each pulse, with SAME or MIRROR direction mode
  - UDP + Serial input (ASCII "-1/0/1" or binary int8)
  - Safe delays after Enable and after DIR change
  - 0 (stop) สามารถแทรกกลางทางเพื่อหยุดได้ทันที

  Wiring (Common-Anode typical):
    PUL+ DIR+ ENA+ -> +5V (Arduino)
    M1: PUL- -> D6, DIR- -> D7, ENA- -> D2
    M2: PUL- -> D9, DIR- -> D8, ENA- -> D3
    GNDs tied (TB6600 <-> Arduino)
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// =================== NETWORK ===================
byte mac[]       = {0xDE,0xAD,0xBE,0xEF,0xFE,0x21};
IPAddress ip     (10, 1, 100, 210);   // Arduino IP
const uint16_t LISTEN_PORT = 8000;    // UDP port
EthernetUDP Udp;

// =================== PINS ======================
// Motor 1
const uint8_t M1_STEP = 6;
const uint8_t M1_DIR  = 7;
const uint8_t M1_EN   = 2;
// Motor 2
const uint8_t M2_STEP = 9;
const uint8_t M2_DIR  = 8;
const uint8_t M2_EN   = 3;

// =================== POLARITY ==================
// ถ้ามอเตอร์ใด "ยังไม่ตื่น/หมุน" ลองสลับ true/false ของตัวนั้น
const bool EN_ACTIVE_LOW_M1   = true;  // ENA- LOW = enable
const bool EN_ACTIVE_LOW_M2   = true;
const bool STEP_ACTIVE_LOW_M1 = true;  // pulse "on" = LOW (common-anode)
const bool STEP_ACTIVE_LOW_M2 = true;

// ทิศ CW สำหรับแต่ละมอเตอร์ (เผื่อสายไขว้/เฟสกลับ)
const bool DIR_HIGH_IS_CW_M1  = true;   // DIR=HIGH แปลว่า CW?
const bool DIR_HIGH_IS_CW_M2  = true;   // ปรับได้ตามจริง

// =================== MODE ======================
bool MIRROR_MODE = false;  // false=SAME (หมุนทิศเดียวกัน), true=MIRROR (สลับทิศ)
// (ออปชัน) อ่านโหมดจากสวิตช์ภายนอก
const bool USE_MODE_PIN = false;
const uint8_t MODE_PIN  = 4;   // HIGH= mirror, LOW = same

// =================== MOTION ====================
// มอเตอร์ 1.8° -> 200 steps/rev
const int STEPS_PER_REV = 1400;
// ตั้งให้ตรงกับ DIP microstep ของ TB6600 (1,2,4,8,16,32 ...)
const int MICROSTEP     = 2;
const int STEPS_PER_90  = (STEPS_PER_REV * MICROSTEP) / 4;
// const int STEPS_PER_90  = STEPS_PER_REV / 4;

// ความเร็วพัลส์ (ปรับตามชุดจริง)
const uint16_t STEP_HIGH_US = 500;   // pulse high width
const uint16_t STEP_LOW_US  = 500;   // pulse low  width

// หน่วงความปลอดภัย (คุณทดสอบแล้วว่าเวิร์ก)
const uint32_t ENA_SETUP_MS = 100;   // รอหลัง Enable
const uint16_t DIR_SETUP_US = 100;   // รอหลังสลับทิศ

// =================== STATE =====================
// แยกสถานะคำสั่ง: รอทำ (-1/1) กับคำสั่งหยุด (0)
volatile int8_t pending_cmd = 0;     // -1 หรือ 1 ที่พร้อมจะเริ่มหมุน 90°
volatile bool   abort_requested = false; // true เมื่อมีคำสั่ง 0 ระหว่างทาง
bool is_moving = false;

// =================== HELPERS ===================
inline void setEnableMotor(uint8_t motor, bool enable) {
  if (motor == 1) {
    digitalWrite(M1_EN, (EN_ACTIVE_LOW_M1 ? (enable ? LOW : HIGH)
                                          : (enable ? HIGH : LOW)));
  } else {
    digitalWrite(M2_EN, (EN_ACTIVE_LOW_M2 ? (enable ? LOW : HIGH)
                                          : (enable ? HIGH : LOW)));
  }
}
inline void setEnableBoth(bool enable) {
  setEnableMotor(1, enable);
  setEnableMotor(2, enable);
}

inline void setDirMotor(uint8_t motor, bool cw) {
  // map cw -> HIGH/LOW ตามสายจริง (DIR_HIGH_IS_CW_* บอกความหมาย)
  if (motor == 1) {
    bool wantHigh = (cw == DIR_HIGH_IS_CW_M1);
    digitalWrite(M1_DIR, wantHigh ? HIGH : LOW);
  } else {
    bool wantHigh = (cw == DIR_HIGH_IS_CW_M2);
    digitalWrite(M2_DIR, wantHigh ? HIGH : LOW);
  }
}

inline void setBothDirections(int8_t cmd_dir) {
  // cmd_dir: -1(CCW), 1(CW)
  bool cw_m1 = (cmd_dir > 0);             // CW?
  bool cw_m2 = MIRROR_MODE ? !cw_m1 : cw_m1; // สลับทิศถ้า MIRROR
  setDirMotor(1, cw_m1);
  setDirMotor(2, cw_m2);
}

inline void pulseBothOnce() {
  // STEP on-phase
  digitalWrite(M1_STEP, STEP_ACTIVE_LOW_M1 ? LOW  : HIGH);
  digitalWrite(M2_STEP, STEP_ACTIVE_LOW_M2 ? LOW  : HIGH);
  delayMicroseconds(STEP_HIGH_US);
  // STEP off-phase
  digitalWrite(M1_STEP, STEP_ACTIVE_LOW_M1 ? HIGH : LOW);
  digitalWrite(M2_STEP, STEP_ACTIVE_LOW_M2 ? HIGH : LOW);
  delayMicroseconds(STEP_LOW_US);
}

// =================== INPUT (UDP/Serial) =========
bool parseCmdFromPacket(char* buf, int len, int8_t &outCmd) {
  if (len <= 0) return false;
  if (len == 1) {
    int8_t v = (int8_t)buf[0];
    if (v == -1 || v == 0 || v == 1) { outCmd = v; return true; }
    if (buf[0] == '1')  { outCmd = 1;  return true; }
    if (buf[0] == '0')  { outCmd = 0;  return true; }
    if (buf[0] == '-')  { return false; }
    return false;
  }
  // treat as string
  const int MAXN = 16;
  if (len >= MAXN) len = MAXN - 1;
  char tmp[MAXN];
  memcpy(tmp, buf, len);
  tmp[len] = '\0';
  // trim
  int i = 0, j = strlen(tmp) - 1;
  while (tmp[i] == ' ' || tmp[i] == '\n' || tmp[i] == '\r' || tmp[i] == '\t') i++;
  while (j >= i && (tmp[j] == ' ' || tmp[j] == '\n' || tmp[j] == '\r' || tmp[j] == '\t')) j--;
  tmp[j + 1] = '\0';
  int v = atoi(&tmp[i]);
  if (v == -1 || v == 0 || v == 1) { outCmd = (int8_t)v; return true; }
  return false;
}

void handleUdp() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;
  char pkt[32];
  int n = Udp.read(pkt, min(packetSize, (int)sizeof(pkt)));
  if (n <= 0) return;
  int8_t c;
  if (parseCmdFromPacket(pkt, n, c)) {
    if (c == 0) abort_requested = true;
    else pending_cmd = c; // -1 หรือ 1
  }
}

void handleSerial() {
  while (Serial.available() > 0) {
    String s = Serial.readStringUntil('\n');
    s.trim();
    if (s.equalsIgnoreCase("mode mirror")) { MIRROR_MODE = true;  Serial.println("Mode = MIRROR"); }
    else if (s.equalsIgnoreCase("mode same")) { MIRROR_MODE = false; Serial.println("Mode = SAME"); }
    else {
      int v = s.toInt();
      if (v == 0) abort_requested = true;
      else if (v == -1 || v == 1) pending_cmd = (int8_t)v;
    }
  }
}

// =================== MOVE LOGIC =================
void rotate90_once(int8_t dirCmd) {
  if (dirCmd == 0) return;

  // เลือกทิศทางสองมอเตอร์ตามโหมด
  setBothDirections(dirCmd);

  // Enable ทั้งคู่ แล้วรอให้วงจรพร้อม
  setEnableBoth(true);
  delay(ENA_SETUP_MS);

  // รอหลังสลับทิศ
  delayMicroseconds(DIR_SETUP_US);

  is_moving = true;

  for (int i = 0; i < STEPS_PER_90; i++) {
    // ถ้ามีคำสั่งหยุด (0) ตอนกลางทาง -> หยุดทันที
    if (abort_requested) {
      is_moving = false;
      setEnableBoth(false);
      abort_requested = false;   // เคลียร์เพื่อพร้อมรับคำสั่งใหม่
      Serial.println("Aborted by cmd=0");
      return;
    }

    pulseBothOnce();

    // เปิดโอกาสให้รับคำสั่งใหม่จาก UDP/Serial ระหว่างหมุน
    handleUdp();
    handleSerial();

    // ถ้ามีคำสั่งทิศตรงข้ามเข้ามา (เช่นกำลัง 1 แล้วมี -1 เข้ามา)
    // เราจะ "หยุดงานนี้" แล้วให้ลูปหลักเริ่มคำสั่งใหม่รอบถัดไป
    if (pending_cmd == -dirCmd) {
      is_moving = false;
      setEnableBoth(false);
      Serial.println("Opposite cmd detected mid-move; stop current move.");
      return;
    }
  }

  is_moving = false;
  setEnableBoth(false);
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_STEP, OUTPUT);
  pinMode(M1_DIR,  OUTPUT);
  pinMode(M1_EN,   OUTPUT);
  pinMode(M2_STEP, OUTPUT);
  pinMode(M2_DIR,  OUTPUT);
  pinMode(M2_EN,   OUTPUT);

  digitalWrite(M1_STEP, STEP_ACTIVE_LOW_M1 ? HIGH : LOW);
  digitalWrite(M2_STEP, STEP_ACTIVE_LOW_M2 ? HIGH : LOW);
  setEnableBoth(false);

  if (USE_MODE_PIN) {
    pinMode(MODE_PIN, INPUT_PULLUP);
    MIRROR_MODE = (digitalRead(MODE_PIN) == HIGH); // HIGH=mirror
  }

  Ethernet.init(10);
  Ethernet.begin(mac, ip);
  Udp.begin(LISTEN_PORT);

  Serial.println("Ready: send -1 / 0 / 1  (UDP or Serial).");
  Serial.println("Type 'mode mirror' or 'mode same' to switch direction mode.");
}

void loop() {
  handleUdp();
  handleSerial();

  // อ่านโหมดจากสวิตช์ทุกลูป (ถ้าใช้)
  if (USE_MODE_PIN) {
    MIRROR_MODE = (digitalRead(MODE_PIN) == HIGH);
  }

  // ถ้ายังไม่กำลังเคลื่อนที่ และมีคำสั่งรอทำ -> เริ่มหมุน 90°
  if (!is_moving) {
    int8_t c = pending_cmd;
    if (c == 1 || c == -1) {
      pending_cmd = 0;          // เคลียร์คิวก่อนเริ่ม
      abort_requested = false;  // เริ่มงานใหม่ เคลียร์ abort
      rotate90_once(c);
    }
  }
}
