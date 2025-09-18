/*
  Dual TB6600 — rotate exactly 90° per command + UDP ACK (+ Backlash compensation + Pulse logging)

  Command set:
    -2 = Relay OFF
    -1 = Rotate 90° CCW
     0 = Abort/Stop
     1 = Rotate 90° CW
     2 = Relay ON

  Packet (from ROS -> Arduino) RAW mode:
    [cmd:int8][seq:uint8]

  ACK Packet (Arduino -> ROS):
    [0xAC][seq][status][cmd][mirror(0/1)][moving(0/1)]
      status: 0=received, 1=started, 2=completed, 3=aborted

  Wiring (TB6600 Common-Anode typical):
    PUL+ DIR+ ENA+ -> +5V (Arduino)
    M1: PUL- -> D6, DIR- -> D7, ENA- -> D2
    M2: PUL- -> D9, DIR- -> D8, ENA- -> D3
    Relay -> D5 (HIGH=ON)
    GNDs tied (TB6600 <-> Arduino)

  Notes:
    - EN_ACTIVE_LOW_* = true for TB6600 ENA- active LOW
    - STEP_ACTIVE_LOW_* = true for common-anode pulse
    - GEAR_RATIO = 14.0 -> 90° = 700 * MICROSTEP pulses (integer)
    - Backlash compensation: เติม "take-up pulses" เฉพาะตอนสลับทิศ, รองรับค่าทศนิยม
    - เพิ่ม Serial.print log ของจำนวนพัลส์ที่สั่งยิง (take-up / steps90 / total)
*/

#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <math.h>
#include <string.h>
// #include <ctype.h>

// =================== NETWORK ===================
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x21 };
IPAddress ip(10, 1, 100, 210);      // Arduino IP
const uint16_t LISTEN_PORT = 8000;  // UDP port
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

// =================== RELAY ======================
const uint8_t RELAY_PIN = 5;
inline void relay_on()  { digitalWrite(RELAY_PIN, HIGH); }
inline void relay_off() { digitalWrite(RELAY_PIN, LOW);  }

// =================== POLARITY ==================
const bool EN_ACTIVE_LOW_M1   = true;  // ENA- LOW = enable
const bool EN_ACTIVE_LOW_M2   = true;
const bool STEP_ACTIVE_LOW_M1 = true;  // pulse on = LOW (common-anode)
const bool STEP_ACTIVE_LOW_M2 = true;
const bool DIR_HIGH_IS_CW_M1  = true;  // DIR=HIGH means CW?
const bool DIR_HIGH_IS_CW_M2  = true;

// =================== MODE ======================
bool MIRROR_MODE = true;         // false=SAME, true=MIRROR
const bool USE_MODE_PIN = false; // ถ้าต่อสวิตช์ภายนอกให้ตั้ง true
const uint8_t MODE_PIN = 4;      // HIGH=mirror, LOW=same (เมื่อ USE_MODE_PIN=true)

// =================== MOTION ====================
// มอเตอร์ 1.8° = 200 step/rev
const int   BASE_STEPS_PER_REV = 200;

// DIP microstep ของ TB6600 (1,2,4,8,16,32,...)
const int   MICROSTEP          = 1;

// อัตราทดเกียร์ตั้งเป็น 14.0
const float GEAR_RATIO         = 14.0f;

// จำนวนพัลส์ต่อ 90°: 200*1*14/4 = 700
const int STEPS_PER_90 = (int)lround((BASE_STEPS_PER_REV * (double)MICROSTEP * GEAR_RATIO) / 4.0); // = 700*MICROSTEP

// Trim ต่อทิศ (ถ้าพบ bias คงที่) -> บวก/ลบเป็น “พัลส์”
const int TRIM_CW  = 0;
const int TRIM_CCW = 0;

// --- Backlash compensation on direction change ---
// ใส่ค่าเป็น "จำนวนพัลส์ที่อยากชดเชยต่อการสลับทิศ 1 ครั้ง" (ทศนิยม)
// เช่น 0.5 = เติม 1 พัลส์ทุก ๆ 2 ครั้งที่สลับทิศ (เฉลี่ย 0.5)
const double BACKLASH_PULSES = 0.0;

// สะสมเฉพาะตอน "สลับทิศ" (ทศนิยม)
static double rev_accum = 0.0;

// จำทิศของการเคลื่อนครั้งล่าสุด (0 = ยังไม่เคยหมุน)
static int8_t last_move_dir = 0;

// ความเร็วพัลส์
const uint16_t STEP_HIGH_US  = 500;  // pulse high width
const uint16_t STEP_LOW_US   = 500;  // pulse low  width

// หน่วงความปลอดภัย
const uint32_t ENA_SETUP_MS  = 200;  // รอหลัง Enable
const uint16_t DIR_SETUP_US  = 200;  // รอหลังเปลี่ยน DIR

// =================== STATE =====================
volatile int8_t pending_cmd = 0;        // -1,1 ที่รอทำ
volatile bool   abort_requested = false; // มีคำสั่ง 0 แทรกกลางทาง
bool is_moving = false;

// =================== ACK / FEEDBACK ============
IPAddress last_sender_ip;
uint16_t  last_sender_port = 0;
uint8_t   last_seq = 0;
int8_t    last_cmd_for_seq = 0;

// status: 0=received, 1=started, 2=completed, 3=aborted
void sendAck(uint8_t seq, uint8_t status, int8_t cmd, IPAddress ip, uint16_t port) {
  uint8_t pkt[6];
  pkt[0] = 0xAC;
  pkt[1] = seq;
  pkt[2] = status;
  pkt[3] = (uint8_t)cmd;
  pkt[4] = MIRROR_MODE ? 1 : 0;
  pkt[5] = is_moving ? 1 : 0;

  Udp.beginPacket(ip, port);
  Udp.write(pkt, sizeof(pkt));
  Udp.endPacket();
}

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
  if (motor == 1) {
    bool wantHigh = (cw == DIR_HIGH_IS_CW_M1);
    digitalWrite(M1_DIR, wantHigh ? HIGH : LOW);
  } else {
    bool wantHigh = (cw == DIR_HIGH_IS_CW_M2);
    digitalWrite(M2_DIR, wantHigh ? HIGH : LOW);
  }
}

inline void setBothDirections(int8_t cmd_dir) {
  bool cw_m1 = (cmd_dir > 0);
  bool cw_m2 = MIRROR_MODE ? !cw_m1 : cw_m1;
  setDirMotor(1, cw_m1);
  setDirMotor(2, cw_m2);
}

inline void pulseBothOnce() {
  // on-phase
  digitalWrite(M1_STEP, STEP_ACTIVE_LOW_M1 ? LOW : HIGH);
  digitalWrite(M2_STEP, STEP_ACTIVE_LOW_M2 ? LOW : HIGH);
  delayMicroseconds(STEP_HIGH_US);
  // off-phase
  digitalWrite(M1_STEP, STEP_ACTIVE_LOW_M1 ? HIGH : LOW);
  digitalWrite(M2_STEP, STEP_ACTIVE_LOW_M2 ? HIGH : LOW);
  delayMicroseconds(STEP_LOW_US);
}

// --- Backlash take-up: คืนจำนวนพัลส์ที่ต้องยิง "ก่อน" หมุน 90° จริง เมื่อสลับทิศ ---
int takeup_pulses_if_reversed(int8_t this_dir) {
  int take = 0;
  if (last_move_dir != 0 && this_dir != last_move_dir) {
    rev_accum += BACKLASH_PULSES;   // สะสมทศนิยม
    take = (int)floor(rev_accum + 1e-9);
    rev_accum -= take;              // เก็บเศษไว้รอบต่อไป
  }
  // ถือว่าหลังจบงานนี้ ทิศล่าสุดคือ this_dir
  last_move_dir = this_dir;
  if (take < 0) take = 0;
  return take;
}

// คืนจำนวนพัลส์สำหรับ 90° ในทิศที่กำหนด + TRIM
int steps_for_90(int8_t dir) {
  int steps = STEPS_PER_90 + ((dir > 0) ? TRIM_CW : TRIM_CCW);
  if (steps < 0) steps = 0;
  return steps;
}

// =================== RELAY & COMMAND ===========
void applyRelayCommand(int8_t cmd) {
  if (cmd == 2) {
    relay_on();
    Serial.println(F("[Relay] ON (cmd=2)"));
  } else if (cmd == -2) {
    relay_off();
    Serial.println(F("[Relay] OFF (cmd=-2)"));
  } else if (cmd == 1) {
    pending_cmd = 1;
    Serial.println(F("[CMD] Queue rotate 90° CW (cmd=1)"));
  } else if (cmd == -1) {
    pending_cmd = -1;
    Serial.println(F("[CMD] Queue rotate 90° CCW (cmd=-1)"));
  } else if (cmd == 0) {
    abort_requested = true;
    relay_off();
    Serial.println(F("[CMD] Abort requested (cmd=0) + Relay OFF"));
  }
}

// =================== INPUT (UDP/Serial) =========
bool parseCmdFromPacket(char* buf, int len, int8_t& outCmd, uint8_t& outSeq, IPAddress& rip, uint16_t& rport) {
  if (len <= 0) return false;
  rip   = Udp.remoteIP();
  rport = Udp.remotePort();

  // RAW + seq (>= 2 bytes)
  if (len >= 2) {
    int8_t vbin = (int8_t)buf[0];
    if (vbin == -2 || vbin == -1 || vbin == 0 || vbin == 1 || vbin == 2) {
      outCmd = vbin;
      outSeq = (uint8_t)buf[1];
      return true;
    }
  }

  // Compatibility: single-byte raw or ASCII single-char
  if (len == 1) {
    int8_t vbin = (int8_t)buf[0];
    if (vbin == -2 || vbin == -1 || vbin == 0 || vbin == 1 || vbin == 2) {
      outCmd = vbin; outSeq = 0; return true;
    }
    if (buf[0] == '0' || buf[0] == '1' || buf[0] == '2') {
      outCmd = (buf[0]-'0'); outSeq = 0; return true;
    }
    return false;
  }

  // ASCII number string: "-2", "-1", " 0", " 1", " 2"
  const int MAXN = 16;
  if (len >= MAXN) len = MAXN - 1;
  char tmp[MAXN]; memcpy(tmp, buf, len); tmp[len] = '\0';
  int i=0,j=len-1;
  while (i<=j && (tmp[i]==' '||tmp[i]=='\n'||tmp[i]=='\r'||tmp[i]=='\t')) i++;
  while (j>=i && (tmp[j]==' '||tmp[j]=='\n'||tmp[j]=='\r'||tmp[j]=='\t')) j--;
  tmp[j+1]='\0';
  long v = strtol(&tmp[i], nullptr, 10);
  if (v == -2 || v == -1 || v == 0 || v == 1 || v == 2) { outCmd=(int8_t)v; outSeq=0; return true; }
  return false;
}

void handleUdp() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;

  char pkt[32];
  int n = Udp.read(pkt, min(packetSize, (int)sizeof(pkt)));
  if (n <= 0) return;

  int8_t c; uint8_t seq; IPAddress rip; uint16_t rport;
  if (parseCmdFromPacket(pkt, n, c, seq, rip, rport)) {
    last_sender_ip    = rip;
    last_sender_port  = rport;
    last_seq          = seq;
    last_cmd_for_seq  = c;

    // Log การรับแพ็กเก็ต
    Serial.print(F("[UDP] RX ")); Serial.print(rip);
    Serial.print(F(":")); Serial.print(rport);
    Serial.print(F(" cmd=")); Serial.print(c);
    Serial.print(F(" seq=")); Serial.println(seq);

    applyRelayCommand(c);

    // status: 0=received, 1=started, 2=completed, 3=aborted
    // void sendAck(uint8_t seq, uint8_t status, int8_t cmd, IPAddress ip, uint16_t port)

    // ACK: received
    sendAck(seq, 0, c, rip, rport);

    // คำสั่งรีเลย์เสร็จทันที -> ส่ง completed
    if (c == 2 || c == -2) {
      sendAck(seq, 2 /*completed*/, c, rip, rport);
    }
  }
}

void handleSerial() {
  static char buf[24];
  static uint8_t idx = 0;

  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[idx] = '\0';
      idx = 0;

      if (strcasecmp(buf, "mode mirror") == 0) { MIRROR_MODE = true;  Serial.println(F("[Mode] MIRROR")); continue; }
      if (strcasecmp(buf, "mode same")   == 0) { MIRROR_MODE = false; Serial.println(F("[Mode] SAME"));   continue; }

      long v = strtol(buf, nullptr, 10);
      if (v == -2 || v == -1 || v == 0 || v == 1 || v == 2) {
        applyRelayCommand((int8_t)v);
        // ไม่ส่ง ACK ผ่าน Serial
      } else if (buf[0] != '\0') {
        Serial.print(F("[Serial] Unknown cmd: '")); Serial.print(buf); Serial.println("'");
      }
    } else {
      if (idx < sizeof(buf)-1) buf[idx++] = c; else idx = 0;
    }
  }
}

// =================== MOVE LOGIC =================
void rotate90_once(int8_t dirCmd) {
  if (dirCmd == 0) return;

  setBothDirections(dirCmd);
  setEnableBoth(true);
  delay(ENA_SETUP_MS);
  delayMicroseconds(DIR_SETUP_US);

  is_moving = true;
  // ACK: started
  if (last_sender_port != 0) {
    sendAck(last_seq, 1, last_cmd_for_seq, last_sender_ip, last_sender_port);
  }

  // ★ Backlash take-up: ยิงพัลส์เล็กน้อยเพื่อ "กินระยะฟรี" เฉพาะตอนสลับทิศ
  const int take = takeup_pulses_if_reversed(dirCmd);

  // พัลส์หมุน 90° (จำนวนเต็มพอดีเมื่อ GEAR_RATIO=14.0) + TRIM ต่อทิศ
  const int steps90 = steps_for_90(dirCmd);

  // ----- LOG ค่าพัลส์ทั้งหมดที่จะสั่งยิงในรอบนี้ -----
  Serial.print(F("[MOVE] Dir="));
  Serial.print((dirCmd > 0) ? F("CW") : F("CCW"));
  Serial.print(F(" take-up=")); Serial.print(take);
  Serial.print(F(" steps90=")); Serial.print(steps90);
  Serial.print(F(" total="));   Serial.println(take + steps90);

  // ---------- ยิงพัลส์ take-up ----------
  for (int i = 0; i < take; ++i) {
    if (abort_requested) {
      is_moving = false; setEnableBoth(false); abort_requested = false;
      Serial.println(F("[MOVE] Aborted during backlash take-up"));
      if (last_sender_port != 0) sendAck(last_seq, 3, last_cmd_for_seq, last_sender_ip, last_sender_port);
      return;
    }
    pulseBothOnce();
    handleUdp(); handleSerial();
    if (pending_cmd == -dirCmd) {
      is_moving = false; setEnableBoth(false);
      Serial.println(F("[MOVE] Opposite cmd during take-up; stop."));
      if (last_sender_port != 0) sendAck(last_seq, 3, last_cmd_for_seq, last_sender_ip, last_sender_port);
      return;
    }
  }

  // ---------- ยิงพัลส์หมุน 90° ----------
  for (int i = 0; i < steps90; i++) {
    if (abort_requested) {
      is_moving = false; setEnableBoth(false); abort_requested = false;
      Serial.println(F("[MOVE] Aborted by cmd=0"));
      if (last_sender_port != 0) sendAck(last_seq, 3, last_cmd_for_seq, last_sender_ip, last_sender_port);
      return;
    }

    pulseBothOnce();
    handleUdp();
    handleSerial();

    if (pending_cmd == -dirCmd) {
      is_moving = false; setEnableBoth(false);
      Serial.println(F("[MOVE] Opposite cmd detected mid-move; stop current move."));
      if (last_sender_port != 0) sendAck(last_seq, 3, last_cmd_for_seq, last_sender_ip, last_sender_port);
      return;
    }
  }

  is_moving = false;
  setEnableBoth(false);

  Serial.println(F("[MOVE] Completed 90° move"));
  // ACK: completed
  if (last_sender_port != 0) {
    sendAck(last_seq, 2, last_cmd_for_seq, last_sender_ip, last_sender_port);
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(M1_STEP, OUTPUT); pinMode(M1_DIR, OUTPUT); pinMode(M1_EN, OUTPUT);
  pinMode(M2_STEP, OUTPUT); pinMode(M2_DIR, OUTPUT); pinMode(M2_EN, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT); relay_off();
  digitalWrite(M1_STEP, STEP_ACTIVE_LOW_M1 ? HIGH : LOW);
  digitalWrite(M2_STEP, STEP_ACTIVE_LOW_M2 ? HIGH : LOW);
  setEnableBoth(false);

  if (USE_MODE_PIN) {
    pinMode(MODE_PIN, INPUT_PULLUP);
    MIRROR_MODE = (digitalRead(MODE_PIN) == HIGH);
    Serial.println("MODE PIN");
  }

  Ethernet.init(10);        // CS = D10
  Ethernet.begin(mac, ip);
  Udp.begin(LISTEN_PORT);

  Serial.println(F("=== INIT ==="));
  Serial.print(F("IP: ")); Serial.println(ip);
  Serial.print(F("Listen UDP port: ")); Serial.println(LISTEN_PORT);
  // Serial.print(F("MICROSTEP: ")); Serial.println(MICROSTEP);
  // Serial.print(F("GEAR_RATIO: ")); Serial.println(GEAR_RATIO, 4);
  // Serial.print(F("STEPS_PER_90: ")); Serial.println(STEPS_PER_90);
  // Serial.print(F("TRIM_CW/CCW: ")); Serial.print(TRIM_CW); Serial.print(F("/")); Serial.println(TRIM_CCW);
  // Serial.print(F("BACKLASH_PULSES: ")); Serial.println(BACKLASH_PULSES, 4);
  // Serial.print(F("STEP widths us (H/L): ")); Serial.print(STEP_HIGH_US); Serial.print(F("/")); Serial.println(STEP_LOW_US);
  // Serial.println(F("Ready: send -1 / 0 / 1 / 2 / -2 (UDP RAW+SEQ or Serial)."));
  // Serial.println(F("Type 'mode mirror' or 'mode same' to switch direction mode."));
}

void loop() {
  handleUdp();
  handleSerial();

  if (USE_MODE_PIN) {
    MIRROR_MODE = (digitalRead(MODE_PIN) == HIGH);
    Serial.println("MODE PIN");
  }

  if (!is_moving) {
    int8_t c = pending_cmd;
    if (c == 1 || c == -1) {
      pending_cmd = 0;
      abort_requested = false;
      rotate90_once(c);
    }
  }
}
