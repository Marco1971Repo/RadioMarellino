#include <WiFi.h>
#include <WebServer.h>
#include <Audio.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <RotaryEncoder.h>
#include <OneButton.h>
#include <Adafruit_NeoPixel.h>
#include <vector>
#include "esp_sleep.h"
#include <ESPmDNS.h>

// ── Struttura stazione ────────────────────────────────────────────────────────
struct Station {
    String name;
    String url;
    String nameLang;  // es. "it", "en", "de"
};

// Vettore globale stazioni in RAM
std::vector<Station> stations;

// ── Debug ─────────────────────────────────────────────────────────────────────
//#define DEBUGGAME

// ── Pin I2S ───────────────────────────────────────────────────────────────────
#define I2S_DOUT   12
#define I2S_BCLK   13
#define I2S_LRC    14

// ── Pin Encoder Volume (KY-040) ───────────────────────────────────────────────
#define PIN_DT   42
#define PIN_CLK  41
#define PIN_SW   9   // Pulsante encoder volume → deep sleep

// ── Pin Encoder Stazioni ──────────────────────────────────────────────────────
#define PIN_ST_DT   4
#define PIN_ST_CLK  5
#define PIN_ST_SW   10

// ── LED RGB WS2812B ───────────────────────────────────────────────────────────
#define PIN_LED_RGB  48
#define NUM_LEDS     1
Adafruit_NeoPixel rgb(NUM_LEDS, PIN_LED_RGB, NEO_GRB + NEO_KHZ800);

// Colori LED
#define LED_OFF    rgb.Color(0,   0,   0)
#define LED_YELLOW rgb.Color(255, 255,  0)   // attesa WiFi
#define LED_RED    rgb.Color(255,   0,  0)   // modalità AP
#define LED_GREEN  rgb.Color(0,   255,  0)   // riproduzione
#define LED_CYAN   rgb.Color(0,   255, 255)  // annuncio TTS

// ── Parametri encoder volume ──────────────────────────────────────────────────
#define ROTARYSTEPS    1
#define ROTARYMIN      0
#define ROTARYMAX      64
#define VOLUME_DEFAULT 16

// ── GPIO wakeup deep sleep ────────────────────────────────────────────────────
#define SLEEP_WAKEUP_GPIO  GPIO_NUM_9

// ── Soglia pressione prolungata pulsante → ciclo preset ──────────────────────
#define PRESET_LONGPRESS_MS  500
// ── Soglia pressione molto prolungata pulsante → ingresso modalità AP ────────
#define AP_LONGPRESS_MS      4000

// ── RTC memory: sopravvive al deep sleep ──────────────────────────────────────
RTC_DATA_ATTR int rtcStationIdx    = 0;
RTC_DATA_ATTR int rtcVolume        = VOLUME_DEFAULT;
RTC_DATA_ATTR int rtcWifiPowerIdx  = 0;
RTC_DATA_ATTR int rtcPresetIdx     = 0;

// ── Parametri Equalizzatore ───────────────────────────────────────────────────
float eqLow  = 0.0f;
float eqMid  = 0.0f;
float eqHigh = 0.0f;


// ── Preset audio hardcoded (speculari alla WebUI) ─────────────────────────────
struct AudioPreset {
    const char* name;
    float low;
    float mid;
    float high;
    uint32_t ledColor;  // colore feedback LED
};

const AudioPreset audioPresets[] = {
    { "Flat",            0.0f,  0.0f,  0.0f, 0xFFFFFF },  // bianco
    { "Pure Evoke",      5.0f, -4.0f,  4.0f, 0x00BFFF },  // azzurro
    { "Warm Vintage",    3.0f,  1.0f, -1.0f, 0xFFD700 },  // oro
    { "Rock Overdrive",  4.0f, -1.0f,  3.5f, 0xFF0000 },  // rosso
    { "Parlato",        -3.0f,  4.0f, -2.0f, 0x8A2BE2 },  // viola
};

const int NUM_PRESETS = sizeof(audioPresets) / sizeof(audioPresets[0]);

// ── State machine ─────────────────────────────────────────────────────────────
enum MachineStates {
    STATE_INIT,
    STATE_WAITWIFICONNECTION,
    STATE_PLAYER,
    STATE_START_AP,
    STATE_AP_MODE,
    STATE_STATIONS_CONFIG
};

MachineStates currentState = STATE_INIT;

// ── Oggetti principali ────────────────────────────────────────────────────────
Audio     audio;
WebServer server(80);

// ── Encoder e pulsante ────────────────────────────────────────────────────────
RotaryEncoder encoderVolume  (PIN_DT,    PIN_CLK,    RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoderStazioni(PIN_ST_DT, PIN_ST_CLK, RotaryEncoder::LatchMode::TWO03);
OneButton     btnVolume(PIN_SW, true, true);  
OneButton     btnStazioni(PIN_ST_SW, true, true); 

// ── Variabili encoder ─────────────────────────────────────────────────────────
int lastPos           = -1;
int lastStPos         = 0;
int currentStationIdx = 0;
int currentPresetIdx  = 0;
bool vuMeterAttivo    = false;  

// ── Credenziali WiFi ──────────────────────────────────────────────────────────
String wifiSsid = "";
String wifiPass = "";

// ── Timeout connessione WiFi ──────────────────────────────────────────────────
unsigned long connectionStartTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 5000;

// Indice per i tentativi di potenza WiFi
int uiRetry = 0;

// ── Stato riproduzione ────────────────────────────────────────────────────────
String pendingUrl        = "";
bool   hasPendingPlay    = false;
bool   isSpeakingStation = false;

// ── WA: timer fallback fine annuncio TTS ──────────────────────────────────────
unsigned long ttsStartTime  = 0;
const unsigned long TTS_TIMEOUT_MS = 15000;  

// ── Lampeggio LED in AP mode ──────────────────────────────────────────────────
unsigned long lastBlinkTime = 0;
bool          blinkState    = false;

// ── Throttle salvataggio stato su flash ───────────────────────────────────────
unsigned long lastSaveTime  = 0;
bool          pendingSave   = false;
const unsigned long SAVE_DEBOUNCE_MS = 2000;  

// ── Timer e contatore lampeggi feedback preset ────────────────────────────────
unsigned long presetBlinkTimer   = 0;
int           presetBlinkCount   = 0;
int           presetBlinkTarget  = 0;
bool          presetBlinkPhase   = false;  // true=acceso, false=spento
bool          presetBlinkActive  = false;
const unsigned long PRESET_BLINK_MS = 150;

// ── DIAG: log periodico RSSI/buffer per debug audio a scatti ──────────────────
unsigned long lastDiagLogTime      = 0;
const unsigned long DIAG_LOG_INTERVAL_MS = 2000;
uint32_t      diagLoopCounter      = 0;   // giri di loop() nell'ultima finestra
unsigned long diagLoopWindowStart  = 0;
uint32_t      diagLastLoopHz       = 0;   // giri di loop() al secondo, sull'ultima finestra

// ── Prototipi ─────────────────────────────────────────────────────────────────
void logSuSeriale(const __FlashStringHelper *frmt, ...);
bool loadStations();
bool saveStationsToFS();
bool loadWifiConfig();
bool saveWifiConfig(const String& ssid, const String& pass);
void loadState();
void saveState();
void scheduleSave();
void goToDeepSleep();
void setLed(uint32_t color);
void handleRoot();
void handleSave();
void handleManageStations();
void handleAddStation();
void handleDeleteStation();
void markSkipSleepOnBoot();
bool consumeSkipSleepFlag();
void enterApModeFromButton();
void applyPreset(int idx);
void startPresetBlinkFeedback(int idx);
void tickPresetBlink();
String jsonEscape(const String& s);
void aggiornaLedVuMeter(uint8_t livello);

// ─────────────────────────────────────────────────────────────────────────────
// LED helper
// ─────────────────────────────────────────────────────────────────────────────
void setLed(uint32_t color) {
    rgb.setPixelColor(0, color);
    rgb.show();
}

// ─────────────────────────────────────────────────────────────────────────────
// Applicazione preset audio
// ─────────────────────────────────────────────────────────────────────────────
void applyPreset(int idx) {
    if (idx < 0 || idx >= NUM_PRESETS) return;
    eqLow  = audioPresets[idx].low;
    eqMid  = audioPresets[idx].mid;
    eqHigh = audioPresets[idx].high;
    audio.setTone(eqLow, eqMid, eqHigh);
    logSuSeriale(F("[PRESET] Applicato: %s (idx=%d)\n"), audioPresets[idx].name, idx);
}

// ─────────────────────────────────────────────────────────────────────────────
// Feedback LED lampeggi preset (non bloccante)
// ─────────────────────────────────────────────────────────────────────────────
void startPresetBlinkFeedback(int idx) {
    if (idx < 0 || idx >= NUM_PRESETS) return;
    presetBlinkTarget = idx + 1;  // numero di lampeggi = indice + 1
    presetBlinkCount  = 0;
    presetBlinkPhase  = true;
    presetBlinkActive = true;
    presetBlinkTimer  = millis();
    rgb.setPixelColor(0, audioPresets[idx].ledColor);
    rgb.show();
}

void tickPresetBlink() {
    if (!presetBlinkActive) return;
    if (millis() - presetBlinkTimer < PRESET_BLINK_MS) return;
    presetBlinkTimer = millis();

    if (presetBlinkPhase) {
        setLed(LED_OFF);
        presetBlinkPhase = false;
    } else {
        presetBlinkCount++;
        if (presetBlinkCount >= presetBlinkTarget) {
            presetBlinkActive = false;
            if (!vuMeterAttivo && !isSpeakingStation) setLed(LED_GREEN);
        } else {
            rgb.setPixelColor(0, audioPresets[currentPresetIdx].ledColor);
            rgb.show();
            presetBlinkPhase = true;
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Persistenza stato su LittleFS
// ─────────────────────────────────────────────────────────────────────────────
void loadState() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[STATE] LittleFS mount fallito in lettura\n"));
        return;
    }

    File f = LittleFS.open("/state.json", "r");
    if (!f) {
        logSuSeriale(F("[STATE] state.json non trovato, uso valori default\n"));
        LittleFS.end();
        return;
    }

    JsonDocument doc;
    if (deserializeJson(doc, f) == DeserializationError::Ok) {
        rtcStationIdx   = doc["station"] | 0;
        rtcVolume       = doc["volume"]  | VOLUME_DEFAULT;
        rtcWifiPowerIdx = doc["wifi_power_idx"] | 0;
        rtcPresetIdx    = doc["preset_idx"] | 0;
        eqLow           = doc["eq_low"]  | 0.0f;
        eqMid           = doc["eq_mid"]  | 0.0f;
        eqHigh          = doc["eq_high"] | 0.0f;
        
        logSuSeriale(F("[STATE] Caricato da flash: stazione=%d, vol=%d, preset=%d, EQ=(%.1f,%.1f,%.1f), wifi_power_idx=%d\n"),
                     rtcStationIdx, rtcVolume, rtcPresetIdx, eqLow, eqMid, eqHigh, rtcWifiPowerIdx);
    } else {
        logSuSeriale(F("[STATE] Errore parsing state.json\n"));
    }

    f.close();
    LittleFS.end();
}

void saveState() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[STATE] LittleFS mount fallito in scrittura\n"));
        return;
    }

    File f = LittleFS.open("/state.json", "w");
    if (!f) {
        logSuSeriale(F("[STATE] Impossibile aprire state.json in scrittura\n"));
        LittleFS.end();
        return;
    }

    JsonDocument doc;
    doc["station"]        = currentStationIdx;
    doc["volume"]         = lastPos;
    doc["wifi_power_idx"] = uiRetry;
    doc["preset_idx"]     = currentPresetIdx;
    doc["eq_low"]         = eqLow;
    doc["eq_mid"]         = eqMid;
    doc["eq_high"]        = eqHigh;
    
    serializeJson(doc, f);
    f.close();
    LittleFS.end();

    logSuSeriale(F("[STATE] Salvato su flash: stazione=%d, vol=%d, preset=%d, EQ=(%.1f,%.1f,%.1f), wifi_power_idx=%d\n"),
                 currentStationIdx, lastPos, currentPresetIdx, eqLow, eqMid, eqHigh, uiRetry);
}

void scheduleSave() {
    lastSaveTime = millis();
    pendingSave  = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// WA: marker su flash per distinguere un ESP.restart()
// ─────────────────────────────────────────────────────────────────────────────
void markSkipSleepOnBoot() {
    if (!LittleFS.begin(true)) return;
    File f = LittleFS.open("/skip_sleep.flag", "w");
    if (f) f.close();
    LittleFS.end();
}

bool consumeSkipSleepFlag() {
    if (!LittleFS.begin(true)) return false;
    bool skip = LittleFS.exists("/skip_sleep.flag");
    if (skip) LittleFS.remove("/skip_sleep.flag");
    LittleFS.end();
    return skip;
}

// ─────────────────────────────────────────────────────────────────────────────
// Ingresso in modalità AP su richiesta utente
// ─────────────────────────────────────────────────────────────────────────────
void enterApModeFromButton() {
    logSuSeriale(F("[BTN] Pressione prolungata rilevata: richiesta modalità AP\n"));
    audio.stopSong();
    isSpeakingStation = false;
    hasPendingPlay    = false;
    setLed(LED_OFF);
    currentState = STATE_START_AP;
}

// ─────────────────────────────────────────────────────────────────────────────
// Deep Sleep
// ─────────────────────────────────────────────────────────────────────────────
void goToDeepSleep() {
    rtcStationIdx = currentStationIdx;
    rtcVolume     = (lastPos >= ROTARYMIN) ? lastPos : VOLUME_DEFAULT;
    rtcWifiPowerIdx = uiRetry;
    rtcPresetIdx  = currentPresetIdx;

    saveState();
    audio.stopSong();
    setLed(LED_OFF);
    delay(100);

    logSuSeriale(F("[SLEEP] Entro in deep sleep.\n"));
    Serial.flush();

    esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKEUP_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

// ─────────────────────────────────────────────────────────────────────────────
// Configurazione WiFi (LittleFS)
// ─────────────────────────────────────────────────────────────────────────────
bool loadWifiConfig() {
    if (!LittleFS.begin(true)) return false;
    File f = LittleFS.open("/config.json", "r");
    if (!f) { LittleFS.end(); return false; }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    LittleFS.end();

    if (err) return false;

    wifiSsid = doc["wifi_ssid"] | "";
    wifiPass = doc["wifi_pass"] | "";

    return !wifiSsid.isEmpty();
}

bool saveWifiConfig(const String& ssid, const String& pass) {
    if (!LittleFS.begin(true)) return false;
    File f = LittleFS.open("/config.json", "w");
    if (!f) { LittleFS.end(); return false; }

    JsonDocument doc;
    doc["wifi_ssid"] = ssid;
    doc["wifi_pass"] = pass;

    if (serializeJson(doc, f) == 0) {
        f.close();
        LittleFS.end();
        return false;
    }

    f.close();
    LittleFS.end();
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// WebServer Handlers
// ─────────────────────────────────────────────────────────────────────────────
void handleRoot() {
    String html = F(
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>body{font-family:sans-serif; margin:20px;}"
        "input[type=text],input[type=password]{width:100%;padding:12px;margin:8px 0;border:1px solid #ccc;box-sizing:border-box;}"
        "button{background-color:#4CAF50;color:white;padding:14px 20px;margin:8px 0;border:none;width:100%;cursor:pointer;}</style>"
        "<title>ESP32 Radio Config</title></head><body>"
        "<h2>Configurazione WiFi</h2>"
        "<form action='/save' method='POST'>"
        "<label>SSID</label><input type='text' name='ssid' required>"
        "<label>Password</label><input type='password' name='password'>"
        "<button type='submit'>Salva e Riavvia</button>"
        "</form></body></html>"
    );
    server.send(200, "text/html", html);
}

void handleSave() {
    if (server.hasArg("ssid")) {
        String reqSsid = server.arg("ssid");
        String reqPass = server.arg("password");

        if (currentState == STATE_STATIONS_CONFIG) {
            if (reqSsid.isEmpty()) reqSsid = wifiSsid;
            if (reqPass.isEmpty()) reqPass = wifiPass;
        }

        server.send(200, "text/html", F("<!DOCTYPE html><html><body><h3>Dati salvati. Riavvio in corso...</h3></body></html>"));
        delay(1000);

        saveWifiConfig(reqSsid, reqPass);
        markSkipSleepOnBoot();
        ESP.restart();
    } else if (currentState == STATE_STATIONS_CONFIG) {
        server.send(200, "text/html", F("<!DOCTYPE html><html><body><h3>Configurazione completata. Riavvio in corso...</h3></body></html>"));
        delay(1000);
        markSkipSleepOnBoot();
        ESP.restart();
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Callback audio
// ─────────────────────────────────────────────────────────────────────────────
void my_audio_info(Audio::msg_t m) {
    logSuSeriale(F("[AUDIO-RAW] evt=%d s=%s msg=%s\n"), (int)m.e, m.s, m.msg);
    if (m.e == Audio::evt_eof) {
        if (isSpeakingStation) {
            isSpeakingStation = false;
            //hasPendingPlay = true;
            if (!vuMeterAttivo) setLed(LED_GREEN);
            logSuSeriale(F("Qui: %s\n"), stations[currentStationIdx].url.c_str());
            if (!audio.connecttohost(stations[currentStationIdx].url.c_str()))
                ESP.restart();
            logSuSeriale(F("qui\n"));
        }
    }     
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
wifi_power_t wifiPWR[11];

void setup() {
    Audio::audio_info_callback = my_audio_info;
#ifdef DEBUGGAME
    Serial.begin(115200);
#endif

    rgb.begin();
    rgb.setBrightness(255);
    setLed(LED_OFF);

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        logSuSeriale(F("[BOOT] Wakeup da pulsante, avvio normale\n"));
    } else if (consumeSkipSleepFlag()) {
        logSuSeriale(F("[BOOT] Restart dopo config WiFi, avvio normale\n"));
    } else {
        logSuSeriale(F("[BOOT] Power-on da corrente, torno in sleep\n"));
        Serial.flush();
        esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKEUP_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
        esp_deep_sleep_start();
    }

    loadState();

    // Spostata qui l'inizializzazione Hardware dell'Audio
    audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
    audio.setVolumeSteps(64);
    audio.setVolume(rtcVolume);
    audio.setTone(eqLow, eqMid, eqHigh);

    currentStationIdx = rtcStationIdx;
    currentPresetIdx  = rtcPresetIdx;
    int startVolume   = rtcVolume;
    uiRetry           = rtcWifiPowerIdx;

    encoderVolume.setPosition(startVolume / ROTARYSTEPS);
    lastPos = startVolume;

    encoderStazioni.setPosition(currentStationIdx);
    lastStPos = currentStationIdx;

    pinMode(PIN_SW, INPUT_PULLUP);
    btnVolume.setPressMs(PRESET_LONGPRESS_MS);
    btnVolume.attachClick([]() { goToDeepSleep(); });
    btnVolume.attachLongPressStop([]() {
        if (btnVolume.getPressedMs() >= AP_LONGPRESS_MS) {
            enterApModeFromButton();
        } else if (currentState == STATE_PLAYER) {
            currentPresetIdx = (currentPresetIdx + 1) % NUM_PRESETS;
            applyPreset(currentPresetIdx);
            rtcPresetIdx = currentPresetIdx;
            scheduleSave();
            startPresetBlinkFeedback(currentPresetIdx);
        }
    });

    pinMode(PIN_ST_SW, INPUT_PULLUP);

    btnStazioni.attachClick([]() {
        if (currentState == STATE_PLAYER) {
            vuMeterAttivo = !vuMeterAttivo;
            if (!vuMeterAttivo && !isSpeakingStation) {
                setLed(LED_GREEN);
            }
        }
    });

    btnStazioni.attachLongPressStart([]() {
        if (currentState == STATE_PLAYER) {
            audio.stopSong();
            isSpeakingStation = false;
            hasPendingPlay    = false;
            vuMeterAttivo     = false;

            server.on("/",       HTTP_GET,  handleManageStations);
            server.on("/add",    HTTP_POST, handleAddStation);
            server.on("/delete", HTTP_GET,  handleDeleteStation);
            server.on("/save",   HTTP_POST, handleSave);
            server.begin();

            setLed(LED_CYAN);
            currentState = STATE_STATIONS_CONFIG;
        }
    });
    btnStazioni.setPressMs(AP_LONGPRESS_MS);

    gpio_set_drive_capability((gpio_num_t)I2S_BCLK, GPIO_DRIVE_CAP_3);

    loadStations();

    if (!stations.empty() && currentStationIdx >= (int)stations.size())
        currentStationIdx = 0;

    wifiPWR[10]=WIFI_POWER_19_5dBm; wifiPWR[9]=WIFI_POWER_19dBm; wifiPWR[8]=WIFI_POWER_18_5dBm;
    wifiPWR[7]=WIFI_POWER_17dBm;   wifiPWR[6]=WIFI_POWER_15dBm; wifiPWR[5]=WIFI_POWER_13dBm;
    wifiPWR[4]=WIFI_POWER_11dBm;   wifiPWR[3]=WIFI_POWER_8_5dBm;wifiPWR[2]=WIFI_POWER_7dBm;
    wifiPWR[1]=WIFI_POWER_5dBm;    wifiPWR[0]=WIFI_POWER_2dBm;    
}

// ─────────────────────────────────────────────────────────────────────────────
// Gestione dinamica LED VU-Meter
// ─────────────────────────────────────────────────────────────────────────────
void aggiornaLedVuMeter(uint8_t livello) {
    static uint8_t  ultimoLivello = 0;
    static float    maxDinamico   = 120.0f;
    static uint32_t ultimoDecadimento = 0;

    if (livello == ultimoLivello) return;
    ultimoLivello = livello;

    if (livello > maxDinamico) maxDinamico = livello;

    if (millis() - ultimoDecadimento > 100) {
        if (maxDinamico > 80.0f) maxDinamico -= 0.5f;
        ultimoDecadimento = millis();
    }

    float percentuale = (float)livello / maxDinamico;
    if (percentuale > 1.0f) percentuale = 1.0f;

    uint8_t r = 0, g = 0, b = 0;

    if (percentuale <= 0.50f) {
        g = (uint8_t)map(percentuale * 100, 0, 50, 30, 255);
    } else if (percentuale <= 0.80f) {
        uint8_t lux = (uint8_t)map(percentuale * 100, 51, 80, 100, 255);
        r = lux; g = lux;
    } else {
        r = (uint8_t)map(percentuale * 100, 81, 100, 180, 255);
    }

    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    switch (currentState) {

        case STATE_INIT:
        if (loadWifiConfig()) {
            /*
            IPAddress local_IP(192, 168, 1, 201);
            IPAddress gateway(192, 168, 1, 1);
            IPAddress subnet(255, 255, 255, 0);
            IPAddress primaryDNS(192, 168, 1, 1);
            */
            setLed(LED_YELLOW);
            WiFi.disconnect(true, true);
            delay(100);
            WiFi.setAutoReconnect(false);
            delay(100);
            WiFi.mode(WIFI_STA);
            delay(100);
            //WiFi.config(local_IP, gateway, subnet, primaryDNS);
            //WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
            //WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
            WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
            connectionStartTime = millis();
            currentState = STATE_WAITWIFICONNECTION;
            WiFi.setTxPower(wifiPWR[uiRetry]);
        } else {
            currentState = STATE_START_AP;
        }
        break;

        case STATE_WAITWIFICONNECTION:
            btnVolume.tick();
            btnStazioni.tick();
            if (currentState == STATE_START_AP) break;
            if (WiFi.status() != WL_CONNECTED) {
                if (millis() - connectionStartTime > WIFI_TIMEOUT_MS) {
                    WiFi.disconnect();
                    uiRetry++;
                    if (uiRetry < 11) {
                        currentState = STATE_INIT;
                    } else {
                        uiRetry = 0;
                        currentState = STATE_START_AP;
                    }
                } else {
                    delay(500);
                }
            } else {
                logSuSeriale(F("[WIFI] Connesso a BSSID=%s canale=%d rssi=%d dBm\n"),
                             WiFi.BSSIDstr().c_str(), WiFi.channel(), WiFi.RSSI());
                if (rtcWifiPowerIdx != uiRetry) {
                    WiFi.setSleep(WIFI_PS_NONE); 
                    rtcWifiPowerIdx = uiRetry;
                    saveState();
                }
                if (MDNS.begin("ESPRetroRadio")) {
                    logSuSeriale(F("[MDNS] Avviato: ESPRetroRadio.local\n"));
                    MDNS.addService("http", "tcp", 80);
                } else {
                    logSuSeriale(F("[MDNS] Avvio fallito\n"));
                }
                setLed(LED_CYAN);
                isSpeakingStation = true;
                ttsStartTime = millis();
                //audio.stopSong();
                delay(50);
                audio.connecttospeech(stations[currentStationIdx].name.c_str(), stations[currentStationIdx].nameLang.c_str());
                currentState = STATE_PLAYER;
            }
            break;

        case STATE_START_AP:
        {
            WiFi.disconnect();
            WiFi.mode(WIFI_AP);
            IPAddress local_IP(192, 168, 1, 1);
            IPAddress gateway(192, 168, 1, 1);
            IPAddress subnet(255, 255, 255, 0);

            WiFi.softAPConfig(local_IP, gateway, subnet);
            WiFi.setTxPower(WIFI_POWER_2dBm);
            WiFi.softAP("ESPRetroRadio_Setup");

            server.on("/",     HTTP_GET,  handleRoot);
            server.on("/save", HTTP_POST, handleSave);
            server.begin();

            lastBlinkTime = millis();
            uiRetry = 0;
            rtcWifiPowerIdx = uiRetry;
            saveState();
            currentState = STATE_AP_MODE;
            break;
        }

        case STATE_AP_MODE:
            if (millis() - lastBlinkTime >= 500) {
                lastBlinkTime = millis();
                blinkState = !blinkState;
                setLed(blinkState ? LED_RED : LED_OFF);
            }
            btnVolume.tick();
            server.handleClient();
            delay(2);
            break;

        case STATE_PLAYER:
        {
            diagLoopCounter++;

            audio.loop();
            encoderVolume.tick();
            encoderStazioni.tick();
            btnVolume.tick();
            btnStazioni.tick();

            // ── DIAG: RSSI + stato buffer audio + velocità loop() ──────────────
            if (millis() - lastDiagLogTime >= DIAG_LOG_INTERVAL_MS) {
                unsigned long windowMs = millis() - diagLoopWindowStart;
                diagLastLoopHz = (windowMs > 0) ? (diagLoopCounter * 1000UL / windowMs) : 0;

                logSuSeriale(F("[DIAG] rssi=%d dBm buf_filled=%u buf_free=%u bitrate=%u loopHz=%u speaking=%d pendingPlay=%d\n"),
                             WiFi.RSSI(),
                             (unsigned)audio.inBufferFilled(),
                             (unsigned)audio.inBufferFree(),
                             (unsigned)audio.getBitRate(),
                             (unsigned)diagLastLoopHz,
                             (int)isSpeakingStation,
                             (int)hasPendingPlay);

                lastDiagLogTime     = millis();
                diagLoopCounter      = 0;
                diagLoopWindowStart  = millis();
            }

            if (vuMeterAttivo && !isSpeakingStation && !presetBlinkActive) { 
                //aggiornaLedVuMeter(audio.getVUlevel());
            }

            tickPresetBlink();

            if (pendingSave && (millis() - lastSaveTime >= SAVE_DEBOUNCE_MS)) {
                pendingSave = false;
                saveState();
            }

            if (isSpeakingStation && (millis() - ttsStartTime >= TTS_TIMEOUT_MS)) {
                isSpeakingStation = false;
                audio.stopSong();
                delay(50);
                if (!vuMeterAttivo) setLed(LED_GREEN);
                pendingUrl     = stations[currentStationIdx].url;
                hasPendingPlay = true;
            }

            if (hasPendingPlay) {
                hasPendingPlay = false;
                audio.connecttohost(pendingUrl.c_str());
                audio.setTone(eqLow, eqMid, eqHigh);
            }

            int newPos = encoderVolume.getPosition() * ROTARYSTEPS;

            if (newPos < ROTARYMIN) {
                encoderVolume.setPosition(ROTARYMIN / ROTARYSTEPS);
                newPos = ROTARYMIN;
            } else if (newPos > ROTARYMAX) {
                encoderVolume.setPosition(ROTARYMAX / ROTARYSTEPS);
                newPos = ROTARYMAX;
            }

            if (lastPos != newPos) {
                lastPos = newPos;
                audio.setVolume(lastPos);
                rtcVolume = lastPos;
                scheduleSave();
            }

            int newStPos = encoderStazioni.getPosition();
            if (lastStPos != newStPos) {
                if (isSpeakingStation) {
                    encoderStazioni.setPosition(currentStationIdx);
                    lastStPos = currentStationIdx;
                } else if (!stations.empty()) {
                    if (newStPos > lastStPos)
                        currentStationIdx = (currentStationIdx + 1) % (int)stations.size();
                    else
                        currentStationIdx = (currentStationIdx - 1 + (int)stations.size()) % (int)stations.size();

                    encoderStazioni.setPosition(currentStationIdx);
                    lastStPos     = currentStationIdx;
                    rtcStationIdx = currentStationIdx;
                    scheduleSave();

                    setLed(LED_CYAN);
                    isSpeakingStation = true;
                    ttsStartTime = millis();
                    audio.connecttospeech(stations[currentStationIdx].name.c_str(),
                                          stations[currentStationIdx].nameLang.c_str());
                }
            }

            vTaskDelay(1);
            break;
        }

        case STATE_STATIONS_CONFIG:
            btnVolume.tick();
            server.handleClient();
            if (millis() - lastBlinkTime >= 500) {
                lastBlinkTime = millis();
                blinkState = !blinkState;
                setLed(blinkState ? LED_CYAN : LED_OFF);
            }  
            delay(2);
            break;

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Caricamento / Salvataggio stazioni
// ─────────────────────────────────────────────────────────────────────────────
bool loadStations() {
    // 1. Inizializziamo LittleFS una sola volta
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[STATIONS] Impossibile montare LittleFS\n"));
        return false;
    }

    bool caricamentoRiuscito = false;

    // 2. TENTATIVO DI LETTURA DA FLASH
    if (LittleFS.exists("/stations.json")) {
        File f = LittleFS.open("/stations.json", "r");
        if (f) {
            JsonDocument doc;
            DeserializationError err = deserializeJson(doc, f);
            f.close();

            if (!err) {
                JsonArray arr = doc["stations"].as<JsonArray>();
                // Verifichiamo che l'array esista e non sia vuoto
                if (!arr.isNull() && arr.size() > 0) {
                    stations.clear();
                    stations.reserve(arr.size());

                    for (JsonObject s : arr) {
                        stations.push_back({
                            s["name"].as<String>(),
                            s["url"].as<String>(),
                            s["nameLang"] | "it"
                        });
                    }

                    logSuSeriale(F("[STATIONS] Caricate %d stazioni da LittleFS\n"), stations.size());
                    caricamentoRiuscito = true;
                }
            } else {
                logSuSeriale(F("[STATIONS] Errore nel parsing del JSON: %s\n"), err.c_str());
            }
        }
    }

    // 3. SE LA LETTURA È ANDATA A BUON FINE
    // Chiudiamo il file system e usciamo
    if (caricamentoRiuscito) {
        LittleFS.end();
        return true;
    }

    // 4. FALLBACK (File non trovato, vuoto o corrotto)
    logSuSeriale(F("[STATIONS] File /stations.json assente o non valido. Inizializzazione stazioni predefinite...\n"));

    stations.clear();
    stations.push_back({"Radio Deejay", "http://streamcdnb1-4c4b867c89244861ac216426883d1ad0.msvdn.net/radiodeejay/radiodeejay/play1.m3u8", "en"});
    stations.push_back({"Controradio", "http://streaming.controradio.it:8190/;?type=http&nocache=76494", "it"});
    stations.push_back({"Virgin Radio", "http://icy.unitedradio.it/Virgin.mp3", "en"});
    stations.push_back({"Mitology", "http://onair15.xdevel.com:9120/;stream.mp3", "en"});
    stations.push_back({"BBC World Service", "http://stream.live.vc.bbcmedia.co.uk/bbc_world_service", "en"});
    stations.push_back({"Virgin Rock 80", "http://icy.unitedradio.it/VirginRock80.mp3", "en"});
    stations.push_back({"Virgin Rock 90", "http://icy.unitedradio.it/Virgin_03.mp3", "en"});
    stations.push_back({"Virgin Classic Rock", "http://icy.unitedradio.it/VirginRockClassics.mp3", "en"});
    stations.push_back({"Deejay 80", "http://streamcdnf25-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejay80/live.m3u8", "en"});
    stations.push_back({"On The Road", "http://streamcdnm5-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejayontheroad/live.m3u8", "en"});
    stations.push_back({"Tropical Pizza", "http://streamcdnm12-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejaytropicalpizza/live.m3u8", "it"});
    stations.push_back({"RTL 102.5", "https://dd782ed59e2a4e86aabf6fc508674b59.msvdn.net/live/S97044836/tbbP8T1ZRPBL/playlist_audio.m3u8", "it"});
    stations.push_back({"Radio 105", "http://icecast.unitedradio.it/Radio105.mp3", "it"});
    stations.push_back({"Subasio", "http://icy.unitedradio.it/Subasio.mp3", "it"});
    stations.push_back({"M2O", "http://streamcdnf26-4c4b867c89244861ac216426883d1ad0.msvdn.net/radiom2o/radiom2o/play1.m3u8", "it"});

    // Chiudiamo il FS prima di chiamare saveStationsToFS (che gestirà la sua apertura in scrittura)
    LittleFS.end();

    // Salva le stazioni di default su memoria flash per i successivi riavvii
    return saveStationsToFS();
}

bool saveStationsToFS() {
    if (!LittleFS.begin(true)) return false;
    File f = LittleFS.open("/stations.json", "w");
    if (!f) { LittleFS.end(); return false; }

    JsonDocument doc;
    JsonArray arr = doc["stations"].to<JsonArray>();
    for (const auto& s : stations) {
        JsonObject obj = arr.add<JsonObject>();
        obj["name"] = s.name;
        obj["url"] = s.url;
        obj["nameLang"] = s.nameLang;
    }

    if (serializeJson(doc, f) == 0) {
        f.close();
        LittleFS.end();
        return false;
    }

    f.close();
    LittleFS.end();
    return true;
}

String jsonEscape(const String& s) {
    String out;
    out.reserve(s.length());
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '`':  out += "\\`";  break;
            case '\n': out += "\\n";  break;
            case '\r': break;
            case '<': out += "\\u003c"; break;
            default:   out += c;
        }
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Gestori WebServer dedicati alla modifica delle stazioni
// ─────────────────────────────────────────────────────────────────────────────
void handleManageStations() {
    String stationsJson = "[";
    for (size_t i = 0; i < stations.size(); i++) {
        stationsJson += "{\"name\":\"" + jsonEscape(stations[i].name) + "\",\"url\":\"" + jsonEscape(stations[i].url) + "\",\"lang\":\"" + jsonEscape(stations[i].nameLang) + "\"}";
        if (i < stations.size() - 1) stationsJson += ",";
    }
    stationsJson += "]";

    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    server.sendContent(R"raw(<!DOCTYPE html>
<html lang='it'>
<head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1.0, user-scalable=no'>
    <title>Gestione Web Radio</title>
    <style>
        * { box-sizing: border-box; margin: 0; padding: 0; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif; }
        body { background-color: #f4f5f7; color: #212529; padding: 16px; display: flex; flex-direction: column; align-items: center; }
        .phone-wrapper { width: 100%; max-width: 480px; display: flex; flex-direction: column; gap: 16px; }
        h2 { font-size: 1.4rem; color: #1e293b; text-align: center; margin: 8px 0; display: flex; align-items: center; justify-content: center; gap: 8px; }
        h3 { font-size: 1.1rem; color: #334155; margin-bottom: 12px; border-bottom: 2px solid #e2e8f0; padding-bottom: 6px; }
        .card { background: #ffffff; border-radius: 12px; padding: 16px; box-shadow: 0 4px 12px rgba(0,0,0,0.05); border: 1px solid #e2e8f0; }
        label { display: block; font-size: 0.85rem; font-weight: 600; color: #64748b; margin-bottom: 4px; text-transform: uppercase; }
        input[type="text"], input[type="password"] { width: 100%; padding: 12px; margin-bottom: 14px; border: 1px solid #cbd5e1; border-radius: 8px; font-size: 1rem; background-color: #f8fafc; -webkit-appearance: none; }

        button, .btn { display: inline-flex; align-items: center; justify-content: center; width: 100%; padding: 14px; border: none; border-radius: 8px; font-size: 1rem; font-weight: 600; cursor: pointer; text-decoration: none; }
        button:active, .btn:active { transform: scale(0.98); }
        .btn-blue { background-color: #3b82f6; color: white; }
        .btn-green { background-color: #10b981; color: white; box-shadow: 0 4px 6px rgba(16,185,129,0.2); }
        .btn-orange { background-color: #f59e0b; color: white; padding: 8px 12px; font-size: 0.85rem; border-radius: 6px; width: auto; }
        .btn-red { background-color: #ef4444; color: white; padding: 8px 12px; font-size: 0.85rem; border-radius: 6px; width: auto; }
        .btn-cancel { background-color: #94a3b8; color: white; margin-top: -6px; margin-bottom: 12px; padding: 10px; font-size: 0.9rem; }
        .station-item { display: flex; align-items: center; justify-content: space-between; padding: 12px 0; border-bottom: 1px solid #f1f5f9; gap: 12px; }
        .station-item:last-child { border-bottom: none; }
        .station-info { display: flex; flex-direction: column; gap: 2px; min-width: 0; flex-grow: 1; }
        .station-name { font-weight: 600; font-size: 1rem; color: #1e293b; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; }
        .station-details { font-size: 0.8rem; color: #64748b; display: flex; align-items: center; gap: 6px; }
        .lang-badge { background-color: #e2e8f0; color: #475569; padding: 2px 6px; border-radius: 4px; font-weight: bold; font-size: 0.75rem; text-transform: uppercase; }
        .station-url { white-space: nowrap; overflow: hidden; text-overflow: ellipsis; max-width: 140px; }
        .action-buttons { display: flex; gap: 6px; flex-shrink: 0; }
        #empty-label { text-align: center; color: #94a3b8; padding: 20px 0; font-style: italic; }
    </style>
</head>
<body>
<div class='phone-wrapper'>
    <h2>📻 Gestione Web Radio</h2>
    
    <div class='card'>
        <h3>⚙️ Cambia Rete WiFi (Opzionale)</h3>
        <form action='/save' method='POST'>
            <label for='ssid'>Nuovo SSID</label>
            <input type='text' id='ssid' name='ssid' placeholder='Lascia vuoto per non cambiare' autocomplete='off'>
            <label for='password'>Nuova Password</label>
            <input type='password' id='password' name='password' placeholder='Lascia vuoto per non cambiare'>
            <button type='submit' class='btn btn-blue' style='padding: 10px; font-size: 0.9rem; background-color: #475569;'>🔄 Aggiorna solo WiFi e Riavvia</button>
        </form>
    </div>

    <div class='card'>
        <h3 id='form-title'>➕ Nuova Radio</h3>
        <form id='add-form' action='/add' method='POST'>
            <input type='hidden' name='edit_id' id='edit-id' value=''>
            <label for='name'>Nome Stazione</label>
            <input type='text' id='name' name='name' required placeholder='es. Radio Capital' autocomplete='off'>
            
            <label for='url'>URL Stream</label>
            <input type='text' id='url' name='url' required placeholder='es. http://...' autocomplete='off'>
            
            <label for='lang'>Lingua TTS</label>
            <input type='text' id='lang' name='lang' value='it' required autocomplete='off'>
            
            <button type='submit' id='submit-btn' class='btn btn-blue'>Aggiungi Stazione</button>
        </form>
        <button id='cancel-btn' class='btn btn-cancel' style='display: none;' onclick='resetForm()'>Annulla Modifica</button>
    </div>

    <div class='card'>
        <h3>📜 Stazioni in Memoria</h3>
        <div id='stations-list'></div>
    </div>

    <div style='margin-top: 8px;'>
        <form action='/save' method='POST'>
            <button type='submit' class='btn btn-green'>💾 Salva ed Esci (Riavvia)</button>
        </form>
    </div>
</div>
)raw");

    server.sendContent("\n<script>\nlet stations = " + stationsJson + ";\n");

    server.sendContent(R"raw(
    function renderStations() {
        const listDiv = document.getElementById('stations-list');
        listDiv.innerHTML = '';
        if (stations.length === 0) {
            listDiv.innerHTML = '<div id="empty-label">Nessuna radio in memoria</div>';
            return;
        }
        stations.forEach((station, index) => {
            const item = document.createElement('div');
            item.className = 'station-item';

            const info = document.createElement('div');
            info.className = 'station-info';
            const nameEl = document.createElement('span');
            nameEl.className = 'station-name';
            nameEl.textContent = station.name;
            const details = document.createElement('span');
            details.className = 'station-details';
            const badge = document.createElement('span');
            badge.className = 'lang-badge';
            badge.textContent = station.lang;
            const urlEl = document.createElement('span');
            urlEl.className = 'station-url';
            urlEl.title = station.url;
            urlEl.textContent = station.url;
            details.appendChild(badge);
            details.appendChild(urlEl);
            info.appendChild(nameEl);
            info.appendChild(details);

            const actions = document.createElement('div');
            actions.className = 'action-buttons';
            const editBtn = document.createElement('button');
            editBtn.className = 'btn btn-orange';
            editBtn.textContent = 'Modifica';
            editBtn.onclick = () => startEdit(index);
            const delBtn = document.createElement('a');
            delBtn.className = 'btn btn-red';
            delBtn.href = '/delete?id=' + index;
            delBtn.textContent = 'Elimina';
            delBtn.onclick = () => confirm('Eliminare "' + station.name + '"?');
            actions.appendChild(editBtn);
            actions.appendChild(delBtn);

            item.appendChild(info);
            item.appendChild(actions);
            listDiv.appendChild(item);
        });
    }

    function startEdit(index) {
        const station = stations[index];
        document.getElementById('name').value = station.name;
        document.getElementById('url').value = station.url;
        document.getElementById('lang').value = station.lang;
        document.getElementById('edit-id').value = index;

        document.getElementById('form-title').innerText = "✏️ Modifica Radio";
        document.getElementById('submit-btn').innerText = "Aggiorna Stazione";
        document.getElementById('submit-btn').className = "btn btn-orange";
        document.getElementById('cancel-btn').style.display = "block";
        window.scrollTo({ top: 0, behavior: 'smooth' });
    }

    function resetForm() {
        document.getElementById('name').value = '';
        document.getElementById('url').value = '';
        document.getElementById('lang').value = 'it';
        document.getElementById('edit-id').value = '';

        document.getElementById('form-title').innerText = "➕ Nuova Radio";
        document.getElementById('submit-btn').innerText = "Aggiungi Stazione";
        document.getElementById('submit-btn').className = "btn btn-blue";
        document.getElementById('cancel-btn').style.display = "none";
    }

    renderStations();
</script>
</body>
</html>
)raw");

    server.sendContent("");
}

void handleAddStation() {
    if (server.hasArg("name") && server.hasArg("url") && server.hasArg("lang")) {
        String name = server.arg("name");
        String url = server.arg("url");
        String lang = server.arg("lang");
        String editIdStr = server.arg("edit_id");

        if (!editIdStr.isEmpty()) {
            int editId = editIdStr.toInt();
            if (editId >= 0 && editId < (int)stations.size()) {
                stations[editId] = {name, url, lang};
            }
        } else {
            stations.push_back({name, url, lang});
        }

        saveStationsToFS();

        server.sendHeader("Location", "/");
        server.send(303, "text/plain", "Redirecting...");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

void handleDeleteStation() {
    if (server.hasArg("id")) {
        int id = server.arg("id").toInt();
        if (id >= 0 && id < (int)stations.size()) {
            stations.erase(stations.begin() + id);
            saveStationsToFS();
        }
        server.sendHeader("Location", "/");
        server.send(303, "text/plain", "Redirecting...");
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Debug log
// ─────────────────────────────────────────────────────────────────────────────
void logSuSeriale(const __FlashStringHelper *frmt, ...) {
#ifdef DEBUGGAME
    va_list args;
    va_start(args, frmt);
    static const uint MSG_BUF_SIZE = 256;
    char msg_buf[MSG_BUF_SIZE] = {0};
    vsnprintf_P(msg_buf, MSG_BUF_SIZE, (const char *)frmt, args);
    Serial.print(msg_buf);
    va_end(args);
#endif
}
