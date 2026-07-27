/*
 * buzzer.ino — 2성부 부저 플레이어 (Arduino Uno / Nano, ATmega328P)
 *
 * 배선
 *   D9  (OC1A, Timer1) ── 피에조 부저 A  ── GND      ; 멜로디
 *   D10 (OC2A, Timer2) ── 피에조 부저 B  ── GND      ; 화음/베이스
 *   (부저에 직렬 100~220Ω 저항을 넣으면 소리가 부드럽고 핀도 보호된다)
 *
 * 왜 내장 tone() 을 안 쓰는가
 *   Arduino 의 tone() 은 타이머 하나를 공유해 **동시에 한 음만** 낸다.
 *   두 번째 tone() 을 부르면 첫 음이 끊긴다. 그래서 화음이 안 된다.
 *   여기서는 Timer1(16bit) 과 Timer2(8bit) 를 각각 직접 잡아 두 핀에
 *   독립된 구형파를 내보낸다 → 두 음이 진짜로 같이 울린다.
 *   Timer0 은 건드리지 않는다(millis()/delay() 가 쓴다).
 *
 * 프로토콜 (PC → 보드, 개행으로 끝나는 한 줄)
 *   PLAY:victory     곡 재생 (아래 tuneNames 참조)
 *   STOP             즉시 멈춤
 *   LIST             곡 목록을 시리얼로 되돌려준다
 *
 * 재생은 논블로킹이다 — loop() 가 매번 "다음 음으로 넘어갈 때가 됐나"만 보고
 * 지나가므로, 곡이 흐르는 중에도 STOP 이나 다른 PLAY 를 바로 받는다.
 *
 * 곡을 고치려면: 아래 악보 문자열만 바꾸고 다시 업로드하면 된다.
 * 파이썬 쪽 sorting/tunes.py 와 이름·음을 맞춰두면 헷갈리지 않는다.
 */

#include <Arduino.h>

// ── 핀 ───────────────────────────────────────────────────────────────────
const uint8_t PIN_LEAD = 9;    // Timer1 / OC1A
const uint8_t PIN_BASS = 10;   // Timer2 / OC2A

// ── 악보 ─────────────────────────────────────────────────────────────────
// 표기: "음이름옥타브-길이"  (길이는 분음표, 점은 1.5배, R=쉼표)
//   예) C5-8  E5-16  G4-8.  R-4
//
// ⚠ kirby 는 기억에 의존한 초안이라 실제 곡과 다를 수 있다.
//   울려보고 이 두 줄만 고치면 된다.
const char TUNE_KIRBY_LEAD[] PROGMEM =
    "C5-16 D5-16 E5-16 F5-16 G5-8 G5-16 A5-16 G5-8 E5-8 C6-4 R-8 G5-16 A5-16 C6-2";
const char TUNE_KIRBY_BASS[] PROGMEM =
    "C3-8 C3-8 G3-8 G3-8 C3-8 E3-8 G3-8 C4-2";

const char TUNE_FANFARE_LEAD[] PROGMEM = "C5-8 E5-8 G5-8 C6-4 G5-8 C6-2";
const char TUNE_FANFARE_BASS[] PROGMEM = "C3-4 C3-4 G2-4 C3-2";

const char TUNE_WARN_LEAD[] PROGMEM = "A4-16 R-16 A4-16 R-16 A4-16 R-16 A4-8";
const char TUNE_WARN_BASS[] PROGMEM = "A3-16 R-16 A3-16 R-16 A3-16 R-16 A3-8";

const char TUNE_RESUME_LEAD[] PROGMEM = "C5-16 G5-8";
const char TUNE_RESUME_BASS[] PROGMEM = "C4-16 E4-8";

const char TUNE_PICKOK_LEAD[] PROGMEM = "E6-16";
const char TUNE_PICKOK_BASS[] PROGMEM = "";

const char TUNE_START_LEAD[] PROGMEM = "G4-16 C5-16 E5-8";
const char TUNE_START_BASS[] PROGMEM = "C3-16 C3-16 C3-8";

struct Tune {
  const char *name;
  const char *lead;
  const char *bass;
  uint16_t bpm;
};

const char NAME_VICTORY[] PROGMEM = "victory";
const char NAME_KIRBY[]   PROGMEM = "kirby";
const char NAME_FANFARE[] PROGMEM = "fanfare";
const char NAME_WARN[]    PROGMEM = "warn";
const char NAME_RESUME[]  PROGMEM = "resume";
const char NAME_PICKOK[]  PROGMEM = "pick_ok";
const char NAME_START[]   PROGMEM = "start";

const Tune TUNES[] = {
    {NAME_VICTORY, TUNE_KIRBY_LEAD,   TUNE_KIRBY_BASS,   150},
    {NAME_KIRBY,   TUNE_KIRBY_LEAD,   TUNE_KIRBY_BASS,   150},
    {NAME_FANFARE, TUNE_FANFARE_LEAD, TUNE_FANFARE_BASS, 140},
    {NAME_WARN,    TUNE_WARN_LEAD,    TUNE_WARN_BASS,    200},
    {NAME_RESUME,  TUNE_RESUME_LEAD,  TUNE_RESUME_BASS,  160},
    {NAME_PICKOK,  TUNE_PICKOK_LEAD,  TUNE_PICKOK_BASS,  160},
    {NAME_START,   TUNE_START_LEAD,   TUNE_START_BASS,   160},
};
const uint8_t TUNE_COUNT = sizeof(TUNES) / sizeof(TUNES[0]);

// ── 톤 생성 ──────────────────────────────────────────────────────────────
//
// Timer1: 16bit, CTC 모드(WGM12), OC1A 토글. 분주 8 → 2MHz.
//   토글 주파수 f 를 만들려면 OCR1A = 2MHz / (2*f) - 1.
//   16bit 라 낮은 음(수십 Hz)까지 문제없이 나온다.
static void leadTone(uint16_t hz) {
  if (hz == 0) {
    TCCR1A = 0;                       // 출력 분리 → 핀이 조용해진다
    TCCR1B = 0;
    digitalWrite(PIN_LEAD, LOW);
    return;
  }
  uint32_t ocr = (2000000UL / (2UL * hz)) - 1;
  if (ocr > 65535UL) ocr = 65535UL;
  noInterrupts();
  TCCR1A = _BV(COM1A0);               // OC1A 토글
  TCCR1B = _BV(WGM12) | _BV(CS11);    // CTC, 분주 8
  OCR1A = (uint16_t)ocr;
  TCNT1 = 0;
  interrupts();
}

// Timer2: 8bit 라 OCR 가 255 까지밖에 안 된다. 그래서 음이 낮을수록 분주를
// 키워 가며 맞는 조합을 찾는다(아래 표). 베이스 성부라 낮은 음이 많다.
static const uint16_t T2_PRESCALE[] = {8, 32, 64, 128, 256, 1024};
static const uint8_t  T2_CSBITS[]   = {2, 3, 4, 5, 6, 7};

static void bassTone(uint16_t hz) {
  if (hz == 0) {
    TCCR2A = 0;
    TCCR2B = 0;
    digitalWrite(PIN_BASS, LOW);
    return;
  }
  for (uint8_t i = 0; i < 6; i++) {
    uint32_t ocr = (F_CPU / ((uint32_t)T2_PRESCALE[i] * 2UL * hz)) - 1UL;
    if (ocr <= 255UL) {
      noInterrupts();
      TCCR2A = _BV(COM2A0) | _BV(WGM21);   // OC2A 토글, CTC
      TCCR2B = T2_CSBITS[i];
      OCR2A = (uint8_t)ocr;
      TCNT2 = 0;
      interrupts();
      return;
    }
  }
  bassTone(0);                        // 너무 낮아 못 내는 음은 쉼표 처리
}

// ── 음이름 → 주파수 ──────────────────────────────────────────────────────
// A4=440Hz 기준. MIDI 번호로 바꾼 뒤 표에서 찾는다. 부동소수 pow() 를 피하려고
// 한 옥타브치 주파수만 표로 두고, 옥타브 차이는 2배씩 나눈다(정수 연산).
static const uint16_t C8_TABLE[12] = {
    4186, 4435, 4699, 4978, 5274, 5588, 5920, 6272, 6645, 7040, 7459, 7902};

static uint16_t noteToHz(const char *note, uint8_t len) {
  if (len == 0 || note[0] == 'R' || note[0] == 'r') return 0;

  int8_t semitone;
  switch (note[0]) {
    case 'C': semitone = 0;  break;
    case 'D': semitone = 2;  break;
    case 'E': semitone = 4;  break;
    case 'F': semitone = 5;  break;
    case 'G': semitone = 7;  break;
    case 'A': semitone = 9;  break;
    case 'B': semitone = 11; break;
    default:  return 0;
  }

  uint8_t i = 1;
  while (i < len && (note[i] == '#' || note[i] == 'b')) {
    semitone += (note[i] == '#') ? 1 : -1;
    i++;
  }
  if (i >= len) return 0;

  int8_t octave = note[i] - '0';
  if (semitone < 0)  { semitone += 12; octave--; }
  if (semitone > 11) { semitone -= 12; octave++; }
  if (octave < 0 || octave > 8) return 0;

  // 옥타브를 내릴 때마다 2로 나눈다. 그냥 >> 하면 매번 버림이 쌓여 낮은 음이
  // 눈에 띄게 처져서(D#2 에서 78→77Hz) 파이썬 쪽 계산과 어긋난다. 절반을
  // 더해 반올림한다. octave==8 이면 시프트가 0 이라 (1 << -1) 을 피해야 한다.
  uint8_t shift = (uint8_t)(8 - octave);
  uint16_t base = C8_TABLE[semitone];
  if (shift == 0) return base;
  return (uint16_t)((base + (1U << (shift - 1))) >> shift);
}

// ── 성부 재생기 ──────────────────────────────────────────────────────────
struct Voice {
  const char *score;      // PROGMEM 악보
  uint16_t pos;           // 다음에 읽을 문자 위치
  uint32_t nextAt;        // 이 시각이 되면 다음 음으로
  bool active;
  void (*emit)(uint16_t);
};

Voice lead = {nullptr, 0, 0, false, leadTone};
Voice bass = {nullptr, 0, 0, false, bassTone};
uint16_t currentBpm = 140;

static void voiceStart(Voice &v, const char *score) {
  v.score = score;
  v.pos = 0;
  v.nextAt = millis();
  v.active = (score != nullptr && pgm_read_byte(score) != '\0');
  if (!v.active) v.emit(0);
}

// 악보에서 토큰 하나를 읽어 소리를 내고, 다음 전환 시각을 잡는다.
static void voiceStep(Voice &v, uint32_t now) {
  if (!v.active || (int32_t)(now - v.nextAt) < 0) return;

  char token[10];
  uint8_t n = 0;
  char c;

  while ((c = pgm_read_byte(v.score + v.pos)) == ' ') v.pos++;   // 앞 공백 건너뛰기
  while ((c = pgm_read_byte(v.score + v.pos)) != '\0' && c != ' ' &&
         n < sizeof(token) - 1) {
    token[n++] = c;
    v.pos++;
  }
  token[n] = '\0';

  if (n == 0) {                        // 악보 끝
    v.active = false;
    v.emit(0);
    return;
  }

  uint8_t dash = 0;                    // 'C5-8' 의 '-' 위치
  while (dash < n && token[dash] != '-') dash++;
  if (dash >= n) { v.active = false; v.emit(0); return; }

  uint16_t hz = noteToHz(token, dash);

  bool dotted = (token[n - 1] == '.');
  uint8_t denom = (uint8_t)atoi(token + dash + 1);
  if (denom == 0) denom = 4;

  uint32_t wholeMs = 240000UL / currentBpm;         // 온음표 = 4박
  uint32_t durMs = wholeMs / denom;
  if (dotted) durMs += durMs / 2;

  v.emit(hz);
  // 음 사이를 아주 살짝 띄운다 — 같은 음이 이어질 때 두 번으로 들리게
  v.nextAt = now + durMs;
}

static void allOff() {
  lead.active = bass.active = false;
  leadTone(0);
  bassTone(0);
}

// ── 명령 처리 ────────────────────────────────────────────────────────────
static void playByName(const char *name) {
  for (uint8_t i = 0; i < TUNE_COUNT; i++) {
    char buf[16];
    strcpy_P(buf, TUNES[i].name);
    if (strcmp(buf, name) == 0) {
      currentBpm = TUNES[i].bpm;
      voiceStart(lead, TUNES[i].lead);
      voiceStart(bass, TUNES[i].bass);
      Serial.print(F("OK "));
      Serial.println(buf);
      return;
    }
  }
  Serial.print(F("ERR unknown tune: "));
  Serial.println(name);
}

static void handleLine(char *line) {
  while (*line == ' ') line++;
  char *end = line + strlen(line);
  while (end > line && (end[-1] == '\r' || end[-1] == ' ')) *--end = '\0';

  if (strncmp(line, "PLAY:", 5) == 0) {
    playByName(line + 5);
  } else if (strcmp(line, "STOP") == 0) {
    allOff();
    Serial.println(F("OK stop"));
  } else if (strcmp(line, "LIST") == 0) {
    for (uint8_t i = 0; i < TUNE_COUNT; i++) {
      char buf[16];
      strcpy_P(buf, TUNES[i].name);
      Serial.println(buf);
    }
  } else if (*line) {
    Serial.print(F("ERR bad command: "));
    Serial.println(line);
  }
}

// ── 표준 진입점 ──────────────────────────────────────────────────────────
char rxBuf[48];
uint8_t rxLen = 0;

void setup() {
  pinMode(PIN_LEAD, OUTPUT);
  pinMode(PIN_BASS, OUTPUT);
  allOff();
  Serial.begin(115200);
  Serial.println(F("READY soarm-buzzer 2voice"));
  playByName("start");                 // 부팅했다는 걸 귀로 알린다
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    }
  }

  uint32_t now = millis();
  voiceStep(lead, now);
  voiceStep(bass, now);
}
