#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RTClib.h>
#include <ESP32Servo.h>
#include "HX711.h"

// ============================================================
// CONFIGURATION
// ============================================================
const char* WIFI_SSID  = "P302B";
const char* WIFI_PASS  = "12345678@";

#define SERVO_PIN       13
#define OPEN_ANGLE      95
#define CLOSE_ANGLE     45

#define HX_DT           4
#define HX_SCK          5
#define HX_SCALE        2280.0f   // calibrate if scale is wrong

#define IR_PIN          15
#define BUTTON_PIN      16

#define EEPROM_SIZE     256
#define EEPROM_MAGIC    0xAB

#define FEED_TIMEOUT    15000UL   // ms max open servo
#define FEED_OPEN_WAIT  300UL     // ms wait servo open
#define FEED_CLOSE_WAIT 500UL     // ms wait servo close

#define LCD_INTERVAL    500UL     // ms LCD update interval
#define WIFI_RETRY_MS   30000UL   // ms WiFi retry interval
#define EEPROM_DEBOUNCE 5000UL    // ms debounce ghi EEPROM

#define WEIGHT_SAMPLES  5
#define WEIGHT_READ_MS  80UL      // ms between HX711 reads

// ============================================================
// FORWARD DECLARATIONS
// ============================================================
void  handleRoot();
void  handleSet();
void  handleData();
void  handleTime();
void  handleSetTime();
void  handleReset();
void  feedNow(int idx = -1);
void  saveDataDeferred();
void  saveDataNow();
void  taskReadWeight();
void  taskFeedStateMachine();
void  taskButton(int schedState, int schedIdx);
void  taskIR(int schedState, int schedIdx);
void  taskAutoSchedule(DateTime& now, int schedState);
void  taskEEPROM();
void  taskWiFiReconnect();
void  updateLCD(DateTime& now);
int   getScheduleState(DateTime& now, int& idx);
void  checkNewDay(DateTime& now);
float getWeightFiltered();
float medianOf5(float* a);

// ============================================================
// HARDWARE
// ============================================================
WebServer           server(80);
LiquidCrystal_I2C  lcd(0x27, 16, 2);
RTC_DS3231         rtc;
Servo              myServo;
HX711              scale;

// ============================================================
// EEPROM DATA STRUCTURE
// ============================================================
struct Data {
    uint8_t magic;
    uint8_t maxFeed;
    uint8_t fedToday;
    int32_t lastDay;        // day + month*100
    uint8_t hour[5];
    uint8_t minute[5];
    uint8_t fedFlag[5];     // 1 = this schedule already fed today
    float   feedWeight[5];
    float   defaultWeight;
};

Data cfg;
bool scheduleFed[5];

bool rtcBad = false;   // true if RTC lost power or error

// ============================================================
// EEPROM - write with debounce
// ============================================================
bool          eepromDirty   = false;
unsigned long eepromDirtyAt = 0;

void saveDataDeferred() {
    eepromDirty   = true;
    eepromDirtyAt = millis();
}

void saveDataNow() {
    cfg.magic = EEPROM_MAGIC;
    EEPROM.put(0, cfg);
    EEPROM.commit();
    eepromDirty = false;
}

void taskEEPROM() {
    if (eepromDirty && (millis() - eepromDirtyAt >= EEPROM_DEBOUNCE)) {
        saveDataNow();
    }
}

void loadData() {
    EEPROM.get(0, cfg);
    if (cfg.magic != EEPROM_MAGIC) {
        cfg.magic         = EEPROM_MAGIC;
        cfg.maxFeed       = 2;
        cfg.fedToday      = 0;
        cfg.lastDay       = 0;
        cfg.defaultWeight = 50.0f;
        for (int i = 0; i < 5; i++) {
            cfg.hour[i]       = 0;
            cfg.minute[i]     = 0;
            cfg.fedFlag[i]    = 0;
            cfg.feedWeight[i] = 50.0f;
        }
        saveDataNow();
    }
    for (int i = 0; i < 5; i++)
        scheduleFed[i] = (cfg.fedFlag[i] == 1);
}

// ============================================================
// NEW DAY RESET
// ============================================================
void checkNewDay(DateTime& now) {
    int32_t today = (int32_t)now.day() + now.month() * 100;
    if (cfg.lastDay == today) return;

    cfg.fedToday = 0;
    cfg.lastDay  = today;
    for (int i = 0; i < 5; i++) {
        scheduleFed[i] = false;
        cfg.fedFlag[i] = 0;
    }
    saveDataDeferred();
}

// ============================================================
// SCHEDULE STATE
//   0 = free (no schedule)
//   1 = blocked zone: 21-60 min before scheduled time
//   2 = window: +-20 min around scheduled time
// ============================================================
int getScheduleState(DateTime& now, int& idx) {
    int current = now.hour() * 60 + now.minute();
    int bestIdx = -1, bestDiff = 9999;

    for (int i = 0; i < cfg.maxFeed; i++) {
        if (cfg.hour[i] == 0 && cfg.minute[i] == 0) continue;
        if (scheduleFed[i]) continue;

        int sched = cfg.hour[i] * 60 + cfg.minute[i];
        int diff  = sched - current;   // positive=not yet, negative=past

        // Blocked zone: 21-60 min before schedule
        if (diff > 20 && diff <= 60) return 1;

        // Feed window: +-20 min
        if (diff >= -20 && diff <= 20) {
            int ab = abs(diff);
            if (ab < bestDiff) { bestDiff = ab; bestIdx = i; }
        }
    }
    if (bestIdx != -1) { idx = bestIdx; return 2; }
    return 0;
}

// ============================================================
// LOADCELL FILTER - Median rolling 5 samples, non-blocking
// ============================================================
float         wBuf[WEIGHT_SAMPLES] = {};
uint8_t       wBufIdx              = 0;
float         wFiltered            = 0.0f;
unsigned long wLastRead            = 0;

float medianOf5(float* a) {
    float b[WEIGHT_SAMPLES];
    memcpy(b, a, sizeof(b));
    // Insertion sort
    for (int i = 1; i < WEIGHT_SAMPLES; i++) {
        float key = b[i]; int j = i - 1;
        while (j >= 0 && b[j] > key) { b[j+1] = b[j]; j--; }
        b[j+1] = key;
    }
    return b[WEIGHT_SAMPLES / 2];
}

void taskReadWeight() {
    if ((millis() - wLastRead) < WEIGHT_READ_MS) return;
    if (!scale.is_ready()) return;
    wLastRead = millis();

    float raw = scale.get_units(1);
    if (raw < 0) raw = 0.0f;
    wBuf[wBufIdx] = raw;
    wBufIdx = (wBufIdx + 1) % WEIGHT_SAMPLES;

    float med = medianOf5(wBuf);
    wFiltered = (med < 0.0f) ? 0.0f : med;
}

float getWeightFiltered() { return wFiltered; }

// ============================================================
// FEEDING - State Machine (non-blocking)
// ============================================================
enum FeedState { FEED_IDLE, FEED_OPENING, FEED_DISPENSING, FEED_CLOSING };
FeedState     feedState  = FEED_IDLE;
float         feedTarget = 0.0f;
float         feedStartW = 0.0f;
unsigned long feedT0     = 0;
int           feedIdx    = -1;

void feedNow(int idx) {
    if (feedState != FEED_IDLE)          return;
    if (cfg.fedToday >= cfg.maxFeed)     return;
    if (idx != -1 && scheduleFed[idx])   return;

    feedIdx    = idx;
    feedTarget = (idx != -1 && cfg.feedWeight[idx] > 0)
                   ? cfg.feedWeight[idx] - 2.0f
                   : cfg.defaultWeight - 2.0f;

    if (idx != -1) {
        scheduleFed[idx] = true;
        cfg.fedFlag[idx] = 1;
    }

    feedStartW = getWeightFiltered();
    feedT0     = millis();
    feedState  = FEED_OPENING;
    myServo.write(OPEN_ANGLE);
}

void taskFeedStateMachine() {
    switch (feedState) {

        case FEED_IDLE: break;

        case FEED_OPENING:
            if (millis() - feedT0 >= FEED_OPEN_WAIT)
                feedState = FEED_DISPENSING;
            break;

        case FEED_DISPENSING: {
            float dispensed = getWeightFiltered() - feedStartW;
            bool  timeout   = (millis() - feedT0 > FEED_TIMEOUT);

            if (dispensed >= feedTarget || timeout) {
                myServo.write(CLOSE_ANGLE);
                feedState = FEED_CLOSING;
                feedT0    = millis();

                cfg.fedToday++;
                saveDataDeferred();
            }
            break;
        }

        case FEED_CLOSING:
            if (millis() - feedT0 >= FEED_CLOSE_WAIT)
                feedState = FEED_IDLE;
            break;
    }
}

// ============================================================
// BUTTON - Non-blocking debounce
// ============================================================
bool          btnLast    = HIGH;
unsigned long btnAt      = 0;
bool          btnHandled = false;

void taskButton(int schedState, int schedIdx) {
    bool cur = digitalRead(BUTTON_PIN);

    if (cur == LOW && btnLast == HIGH) {
        btnAt      = millis();
        btnHandled = false;
    }
    if (cur == LOW && !btnHandled && (millis() - btnAt) >= 50) {
        btnHandled = true;
        if      (schedState == 0) feedNow();
        else if (schedState == 2) feedNow(schedIdx);
        // schedState == 1: blocked zone - do nothing
    }
    if (cur == HIGH) btnHandled = false;
    btnLast = cur;
}

// ============================================================
// IR - Non-blocking 5 seconds
// ============================================================
bool          irActive = false;
unsigned long irStart  = 0;
bool          irFired  = false;

void taskIR(int schedState, int schedIdx) {
    bool det = (digitalRead(IR_PIN) == LOW);

    if (det) {
        if (!irActive) {
            irActive = true;
            irStart  = millis();
            irFired  = false;
        } else if (!irFired && (millis() - irStart) >= 5000) {
            irFired = true;
            if      (schedState == 0) feedNow();
            else if (schedState == 2) feedNow(schedIdx);
        }
    } else {
        irActive = false;
        irFired  = false;
    }
}

// ============================================================
// AUTO SCHEDULE - Fallback if no button press in window
// ============================================================
void taskAutoSchedule(DateTime& now, int schedState) {
    if (schedState == 1) return;

    for (int i = 0; i < cfg.maxFeed; i++) {
        if (cfg.hour[i] == 0 && cfg.minute[i] == 0) continue;
        if (scheduleFed[i]) continue;
        if (cfg.fedToday >= cfg.maxFeed) break;

        if (now.hour() == cfg.hour[i] && now.minute() == cfg.minute[i]) {
            feedNow(i);
            break;
        }
    }
}

// ============================================================
// LCD - Non-blocking, 500ms
// ============================================================
unsigned long lcdLastUpdate = 0;

void updateLCD(DateTime& now) {
    if ((millis() - lcdLastUpdate) < LCD_INTERVAL) return;
    lcdLastUpdate = millis();

    char r0[17], r1[17];

    if (rtcBad) {
        snprintf(r0, sizeof(r0), "RTC ERROR!      ");
    } else {
        snprintf(r0, sizeof(r0), "%02d/%02d   %02d:%02d:%02d",
                 now.day(), now.month(), now.hour(), now.minute(), now.second());
    }

    if (feedState != FEED_IDLE) {
        snprintf(r1, sizeof(r1), "Dang cho an...  ");
    } else {
        // W:XXX.Xg right-padded, fed/max right-aligned to col 16
        char wPart[10];
        snprintf(wPart, sizeof(wPart), "W:%.1fg", getWeightFiltered());
        snprintf(r1, sizeof(r1), "%-9s  %d/%d     ",
                 wPart, cfg.fedToday, cfg.maxFeed);
    }

    lcd.setCursor(0, 0); lcd.print(r0);
    lcd.setCursor(0, 1); lcd.print(r1);
}

// ============================================================
// WIFI RECONNECT
// ============================================================
unsigned long wifiLastCheck = 0;

void taskWiFiReconnect() {
    if ((millis() - wifiLastCheck) < WIFI_RETRY_MS) return;
    wifiLastCheck = millis();

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Mat ket noi, dang reconnect...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
}

// ============================================================
// WEB - HTML using PROGMEM + chunked send, no String+=
// ============================================================
static const char HTML_HEAD[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="vi">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Pet Feeder</title>
<style>
/* === RESET === */
*,*::before,*::after{box-sizing:border-box;margin:0;padding:0}
html{-webkit-text-size-adjust:100%}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;
     background:#0f1117;color:#e8eaf0;min-height:100vh}

/* === LAYOUT SHELL === */
.shell{
  width:100%;
  max-width:560px;
  margin:0 auto;
  padding:16px 16px 56px;
}

/* === HEADER === */
.hdr{text-align:center;padding:20px 0 16px}
.hdr-title{font-size:14px;font-weight:700;letter-spacing:2.5px;
           color:#6ee7b7;text-transform:uppercase;margin-bottom:6px}
.hdr-dot{display:inline-block;width:7px;height:7px;border-radius:50%;
          background:#6ee7b7;margin-right:6px;vertical-align:middle;
          animation:blink 2s ease-in-out infinite}
.hdr-clock{font-size:12px;color:#6b7280}
@keyframes blink{0%,100%{opacity:1}50%{opacity:.1}}

/* === STATUS STRIP === */
.stat-row{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-bottom:10px}
.stat-card{background:#1a1d27;border:1px solid #2a2d3e;border-radius:14px;
           padding:18px 10px;text-align:center}
.stat-lbl{font-size:10px;color:#6b7280;text-transform:uppercase;
          letter-spacing:.8px;margin-bottom:8px}
.stat-val{font-size:30px;font-weight:700;color:#e8eaf0;line-height:1;
          word-break:break-all}
.stat-unit{font-size:10px;color:#6b7280;margin-top:6px}

/* === FEED ALERT === */
.feed-alert{display:none;background:rgba(251,191,36,.08);
            border:1px solid rgba(251,191,36,.3);border-radius:10px;
            padding:10px 16px;font-size:13px;color:#fbbf24;
            text-align:center;margin-bottom:10px;
            animation:pulse 1.5s ease-in-out infinite}
.feed-alert.on{display:block}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}

/* === WARN BAR === */
.warn{background:rgba(251,191,36,.07);border:1px solid rgba(251,191,36,.28);
      border-radius:10px;padding:10px 14px;font-size:12px;
      color:#fbbf24;margin-bottom:10px}

/* === SECTION HEADING === */
.sec-head{font-size:10px;font-weight:600;color:#6b7280;
          text-transform:uppercase;letter-spacing:1.2px;
          margin:16px 0 8px 2px}

/* === GENERIC CARD === */
.card{background:#1a1d27;border:1px solid #2a2d3e;
      border-radius:14px;padding:18px 16px;margin-bottom:10px}
.card-title{font-size:10px;font-weight:600;color:#6b7280;
            text-transform:uppercase;letter-spacing:1px;margin-bottom:14px}

/* === FORM FIELDS === */
.field{margin-bottom:14px}
.field:last-child{margin-bottom:0}
.field-lbl{display:block;font-size:12px;color:#9ca3af;margin-bottom:5px}
.field-inp{
  display:block;width:100%;
  background:#1e2235;border:1.5px solid #2a2d3e;border-radius:9px;
  padding:11px 13px;font-size:16px;color:#e8eaf0;outline:none;
  -webkit-appearance:none;appearance:none;
  transition:border-color .15s;
}
.field-inp:focus{border-color:#6ee7b7}

/* === SCHEDULE GRID (desktop 2-col, mobile 1-col) === */
.sched-grid{
  display:grid;
  grid-template-columns:repeat(auto-fill,minmax(220px,1fr));
  gap:10px;
  margin-bottom:10px;
}

/* === SINGLE SCHEDULE CARD === */
.sched-card{
  background:#1a1d27;border:1px solid #2a2d3e;
  border-radius:14px;padding:14px 14px 16px;
}
.sched-name{font-size:11px;font-weight:700;color:#6ee7b7;
            text-transform:uppercase;letter-spacing:.8px;margin-bottom:12px}
.sched-fields{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
.sched-col{}
.sched-col-lbl{font-size:10px;color:#6b7280;margin-bottom:5px;text-align:center}
.sched-inp{
  display:block;width:100%;
  background:#1e2235;border:1.5px solid #2a2d3e;border-radius:8px;
  padding:11px 4px;font-size:16px;color:#e8eaf0;
  text-align:center;outline:none;
  -webkit-appearance:none;appearance:none;
  transition:border-color .15s;
}
.sched-inp:focus{border-color:#6ee7b7}

/* === BUTTON GROUP (always stacked column) === */
.btn-group{display:flex;flex-direction:column;gap:8px;margin-top:4px}
.btn-primary{
  display:block;width:100%;border:none;border-radius:12px;
  padding:15px;font-size:15px;font-weight:700;cursor:pointer;
  background:#6ee7b7;color:#0a0e14;letter-spacing:.2px;
  -webkit-tap-highlight-color:transparent;
  transition:opacity .15s;
}
.btn-primary:active{opacity:.8}
.btn-secondary{
  display:block;width:100%;border-radius:12px;
  padding:14px;font-size:14px;font-weight:500;cursor:pointer;
  background:transparent;color:#e8eaf0;
  border:1.5px solid #2a2d3e;text-align:center;
  -webkit-tap-highlight-color:transparent;
  transition:border-color .15s;
}
.btn-secondary:hover{border-color:#6ee7b7}
.btn-danger{
  display:block;width:100%;border-radius:12px;
  padding:14px;font-size:14px;font-weight:500;cursor:pointer;
  background:transparent;color:#f87171;
  border:1.5px solid rgba(248,113,113,.28);text-align:center;
  -webkit-tap-highlight-color:transparent;
}
</style>
</head>
<body>
<div class="shell">
<div class="hdr">
  <div class="hdr-title"><span class="hdr-dot"></span>Pet Feeder</div>
  <div class="hdr-clock" id="clk">--</div>
</div>
)rawliteral";

static const char HTML_CARDS[] PROGMEM = R"rawliteral(
<div class="stat-row">
  <div class="stat-card">
    <div class="stat-lbl">Can nang</div>
    <div class="stat-val" id="w">0.0</div>
    <div class="stat-unit">gram</div>
  </div>
  <div class="stat-card">
    <div class="stat-lbl">Da cho an</div>
    <div class="stat-val" id="f">0/0</div>
    <div class="stat-unit">lan hom nay</div>
  </div>
</div>
<div class="feed-alert" id="fs">Dang cho an...</div>
)rawliteral";

static const char HTML_FOOT[] PROGMEM = R"rawliteral(
<div class="btn-group">
  <button type="button" class="btn-secondary" onclick="syncTime()">Dong bo gio</button>
  <button type="button" class="btn-danger"    onclick="doReset()">Reset mac dinh</button>
</div>
</div><!-- /shell -->
<script>
function syncTime(){
  var n=new Date();
  var url='/settime?y='+n.getFullYear()+'&mo='+(n.getMonth()+1)+'&d='+n.getDate()
         +'&h='+n.getHours()+'&mi='+n.getMinutes()+'&s='+n.getSeconds();
  fetch(url).then(function(r){return r.text();})
    .then(function(t){alert(t==='OK'?'Dong bo gio thanh cong!':'Loi: '+t);})
    .catch(function(){alert('Khong ket noi duoc ESP');});
}
function doReset(){
  if(!confirm('Reset toan bo ve mac dinh?'))return;
  fetch('/reset',{method:'POST'}).then(function(){
    document.body.innerHTML='<div style="text-align:center;padding:80px 20px;font-size:16px;color:#e8eaf0">Dang reset, vui long cho...</div>';
    setTimeout(function(){location.reload();},4000);
  });
}
setInterval(function(){
  fetch('/data').then(function(r){return r.json();}).then(function(d){
    document.getElementById('w').textContent=d.weight.toFixed(1);
    document.getElementById('f').textContent=d.fed+'/'+d.max;
    var el=document.getElementById('fs');
    if(d.feeding){el.classList.add('on');}else{el.classList.remove('on');}
  }).catch(function(){});
  fetch('/time').then(function(r){return r.text();}).then(function(t){
    document.getElementById('clk').textContent=t;
  }).catch(function(){});
},1000);
</script>
</body></html>
)rawliteral";

void handleRoot() {
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=UTF-8", "");

    server.sendContent_P(HTML_HEAD);

    if (rtcBad)
        server.sendContent("<div class='warn'>!! RTC mat nguon - gio co the sai. Nhan Dong bo gio de sua.</div>");

    server.sendContent_P(HTML_CARDS);

    // ── General settings ──────────────────────────────────────────────
    // Each sendContent call is a short, fixed string. No snprintf needed.
    server.sendContent("<form action='/set' method='get'>");
    server.sendContent("<div class='sec-head'>Cai dat chung</div>");
    server.sendContent("<div class='card'>");

    // Field 1: max feeds — only the value changes, format it alone
    char num[16];
    server.sendContent("<div class='field'>");
    server.sendContent("<label class='field-lbl'>So lan cho an moi ngay</label>");
    server.sendContent("<input class='field-inp' type='number' name='max' min='1' max='5' inputmode='numeric' value='");
    snprintf(num, sizeof(num), "%d", cfg.maxFeed);
    server.sendContent(num);
    server.sendContent("'></div>");

    // Field 2: default weight
    server.sendContent("<div class='field'>");
    server.sendContent("<label class='field-lbl'>Khau phan mac dinh (gram)</label>");
    server.sendContent("<input class='field-inp' type='number' name='wd' min='1' inputmode='numeric' value='");
    snprintf(num, sizeof(num), "%d", (int)cfg.defaultWeight);
    server.sendContent(num);
    server.sendContent("'></div>");

    server.sendContent("</div>");  // /card

    // ── Schedule section ───────────────────────────────────────────────
    server.sendContent("<div class='sec-head'>Lich cho an</div>");
    server.sendContent("<div class='sched-grid'>");

    for (int i = 0; i < cfg.maxFeed; i++) {

        server.sendContent("<div class='sched-card'>");

        // Title "Lan N"
        server.sendContent("<div class='sched-name'>Lan ");
        snprintf(num, sizeof(num), "%d", i + 1);
        server.sendContent(num);
        server.sendContent("</div>");

        server.sendContent("<div class='sched-fields'>");

        // Column: Gio
        server.sendContent("<div class='sched-col'>");
        server.sendContent("<div class='sched-col-lbl'>Gio</div>");
        server.sendContent("<input class='sched-inp' type='number' name='h");
        snprintf(num, sizeof(num), "%d", i); server.sendContent(num);
        server.sendContent("' min='0' max='23' inputmode='numeric' value='");
        snprintf(num, sizeof(num), "%d", cfg.hour[i]); server.sendContent(num);
        server.sendContent("'>");
        server.sendContent("</div>");

        // Column: Phut
        server.sendContent("<div class='sched-col'>");
        server.sendContent("<div class='sched-col-lbl'>Phut</div>");
        server.sendContent("<input class='sched-inp' type='number' name='m");
        snprintf(num, sizeof(num), "%d", i); server.sendContent(num);
        server.sendContent("' min='0' max='59' inputmode='numeric' value='");
        snprintf(num, sizeof(num), "%d", cfg.minute[i]); server.sendContent(num);
        server.sendContent("'>");
        server.sendContent("</div>");

        // Column: Gram
        server.sendContent("<div class='sched-col'>");
        server.sendContent("<div class='sched-col-lbl'>Gram</div>");
        server.sendContent("<input class='sched-inp' type='number' name='w");
        snprintf(num, sizeof(num), "%d", i); server.sendContent(num);
        server.sendContent("' min='1' inputmode='numeric' value='");
        snprintf(num, sizeof(num), "%d", (int)cfg.feedWeight[i]); server.sendContent(num);
        server.sendContent("'>");
        server.sendContent("</div>");

        server.sendContent("</div>");  // /sched-fields
        server.sendContent("</div>");  // /sched-card
    }

    server.sendContent("</div>");  // /sched-grid

    // ── Save button ────────────────────────────────────────────────────
    server.sendContent("<div class='btn-group'>");
    server.sendContent("<button type='submit' class='btn-primary'>Luu cai dat</button>");
    server.sendContent("</div>");
    server.sendContent("</form>");

    // ── Utility buttons (outside form) ────────────────────────────────
    server.sendContent_P(HTML_FOOT);
    server.sendContent("");
}
void handleData() {
    char buf[96];
    snprintf(buf, sizeof(buf),
             "{\"weight\":%.1f,\"fed\":%d,\"max\":%d,\"feeding\":%s}",
             getWeightFiltered(), cfg.fedToday, cfg.maxFeed,
             (feedState != FEED_IDLE) ? "true" : "false");
    server.send(200, "application/json", buf);
}

void handleTime() {
    if (rtcBad) { server.send(200, "text/plain", "RTC ERROR"); return; }
    DateTime now = rtc.now();
    char buf[32];
    snprintf(buf, sizeof(buf), "%02d/%02d/%04d %02d:%02d:%02d",
             now.day(), now.month(), now.year(),
             now.hour(), now.minute(), now.second());
    server.send(200, "text/plain", buf);
}

void handleSetTime() {
    int y  = server.arg("y").toInt();
    int mo = server.arg("mo").toInt();
    int d  = server.arg("d").toInt();
    int h  = server.arg("h").toInt();
    int mi = server.arg("mi").toInt();
    int s  = server.arg("s").toInt();

    if (y > 2000 && mo >= 1 && mo <= 12 && d >= 1 && d <= 31) {
        rtc.adjust(DateTime(y, mo, d, h, mi, s));
        rtcBad = false;
        server.send(200, "text/plain", "OK");
    } else {
        server.send(400, "text/plain", "Bad params");
    }
}

void handleSet() {
    int newMax = constrain(server.arg("max").toInt(), 1, 5);
    cfg.maxFeed = newMax;

    for (int i = 0; i < 5; i++) {
        if (i < cfg.maxFeed) {
            cfg.hour[i]       = (uint8_t)constrain(server.arg("h"+String(i)).toInt(), 0, 23);
            cfg.minute[i]     = (uint8_t)constrain(server.arg("m"+String(i)).toInt(), 0, 59);
            cfg.feedWeight[i] = server.arg("w"+String(i)).toFloat();
            if (cfg.feedWeight[i] <= 0) cfg.feedWeight[i] = cfg.defaultWeight;
        } else {
            cfg.hour[i] = cfg.minute[i] = 0;
            cfg.feedWeight[i] = 0;
        }
        scheduleFed[i]  = false;
        cfg.fedFlag[i]  = 0;
    }

    float wd = server.arg("wd").toFloat();
    cfg.defaultWeight = (wd > 0) ? wd : 50.0f;

    saveDataDeferred();
    server.sendHeader("Location", "/");
    server.send(303);
}

void handleReset() {
    EEPROM.write(0, 0);
    EEPROM.commit();
    server.send(200, "text/plain", "OK");
    delay(300);
    ESP.restart();
}

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== PET FEEDER BOOT ===");

    EEPROM.begin(EEPROM_SIZE);
    loadData();

    Wire.begin(21, 22);
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("Pet Feeder v3   ");
    lcd.setCursor(0,1); lcd.print("Khoi dong...    ");

    // RTC
    if (!rtc.begin()) {
        Serial.println("[RTC] Khong tim thay module!");
        rtcBad = true;
    } else {
        if (rtc.lostPower()) {
        Serial.println("[RTC] Mat nguon - set compile time");
            rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
            rtcBad = true;
        }
        DateTime t = rtc.now();
        Serial.printf("[RTC] %02d/%02d/%04d %02d:%02d:%02d\n",
            t.day(), t.month(), t.year(), t.hour(), t.minute(), t.second());
    }

    // HX711
    scale.begin(HX_DT, HX_SCK);
    delay(400);
    scale.set_scale(HX_SCALE);
    scale.tare();
    // Pre-fill weight buffer
    for (int i = 0; i < WEIGHT_SAMPLES; i++) wBuf[i] = 0.0f;

    // Servo
    myServo.attach(SERVO_PIN);
    myServo.write(CLOSE_ANGLE);

    // GPIO
    pinMode(IR_PIN,     INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    // WiFi
    lcd.setCursor(0,1); lcd.print("Ket noi WiFi... ");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
        delay(300); Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("\n[WiFi] http://%s\n", WiFi.localIP().toString().c_str());
        char ipbuf[17];
        snprintf(ipbuf, sizeof(ipbuf), "%-16s", WiFi.localIP().toString().c_str());
        lcd.setCursor(0,1); lcd.print(ipbuf);
    } else {
        Serial.println("\n[WiFi] Offline");
        lcd.setCursor(0,1); lcd.print("WiFi that bai!  ");
    }

    // Routes
    server.on("/",        handleRoot);
    server.on("/set",     handleSet);
    server.on("/data",    handleData);
    server.on("/time",    handleTime);
    server.on("/settime", handleSetTime);
    server.on("/reset",   HTTP_POST, handleReset);
    server.begin();

    delay(1200);  // hien thi IP 1.2s
    Serial.println("[BOOT] Ready.");
}

// ============================================================
// LOOP - Non-blocking, khong delay()
// ============================================================
void loop() {
    // 1. Web server
    server.handleClient();

    // 2. Doc can non-blocking (moi 80 ms)
    taskReadWeight();

    // 3. RTC + kiem tra ngay moi
    DateTime now = rtcBad ? DateTime(2000,1,1,0,0,0) : rtc.now();
    if (!rtcBad) checkNewDay(now);

    // 4. Trang thai lich hen
    int schedIdx   = -1;
    int schedState = rtcBad ? 0 : getScheduleState(now, schedIdx);

    // 5. State-machine feeding (luon chay)
    taskFeedStateMachine();

    // 6-8. Input + auto-schedule (chi khi idle)
    if (feedState == FEED_IDLE) {
        taskButton(schedState, schedIdx);
        taskIR(schedState, schedIdx);
        taskAutoSchedule(now, schedState);
    }

    // 9. LCD (moi 500 ms)
    updateLCD(now);

    // 10. Ghi EEPROM debounce
    taskEEPROM();

    // 11. WiFi reconnect (moi 30 s)
    taskWiFiReconnect();
}