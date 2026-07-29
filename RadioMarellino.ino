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

// ── Soglia pressione prolungata pulsante → ingresso modalità AP ──────────────
#define AP_LONGPRESS_MS  3000

// ── RTC memory: sopravvive al deep sleep ──────────────────────────────────────
RTC_DATA_ATTR int rtcStationIdx = 0;
RTC_DATA_ATTR int rtcVolume     = VOLUME_DEFAULT;
RTC_DATA_ATTR int rtcWifiPowerIdx = 0;

// ── Parametri Equalizzatore ───────────────────────────────────────────────────
float eqLow  = 0.0f;
float eqMid  = 0.0f;
float eqHigh = 0.0f;

// ==========================================
// PARAMETRI TUBE DSP (Simulazione Valvolare)
// ==========================================
struct TubeConfig {
    float drive = 1.0f;         // 1.0 = Clean/Spento
    float cutoffAlpha = 1.0f;   
    float makeUpGain = 1.0f;    
};

TubeConfig tubeParams;

// Stato interno del filtro
float lastSampleL = 0.0f;
float lastSampleR = 0.0f;

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
const unsigned long TTS_TIMEOUT_MS = 8000;  

// ── Lampeggio LED in AP mode ──────────────────────────────────────────────────
unsigned long lastBlinkTime = 0;
bool          blinkState    = false;

// ── Throttle salvataggio stato su flash ───────────────────────────────────────
unsigned long lastSaveTime  = 0;
bool          pendingSave   = false;
const unsigned long SAVE_DEBOUNCE_MS = 2000;  

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
void handleSetEq();
void markSkipSleepOnBoot();
bool consumeSkipSleepFlag();
void enterApModeFromButton();
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
        eqLow           = doc["eq_low"]  | 0.0f;
        eqMid           = doc["eq_mid"]  | 0.0f;
        eqHigh          = doc["eq_high"] | 0.0f;
        
        tubeParams.drive       = doc["tube_drive"]  | 1.0f;
        tubeParams.cutoffAlpha = doc["tube_cutoff"] | 1.0f;
        tubeParams.makeUpGain  = doc["tube_gain"]   | 1.0f;

        logSuSeriale(F("[STATE] Caricato da flash: stazione=%d, vol=%d, EQ=(%.1f,%.1f,%.1f), Tube=(%.2f,%.2f,%.2f)\n"),
                     rtcStationIdx, rtcVolume, eqLow, eqMid, eqHigh, 
                     tubeParams.drive, tubeParams.cutoffAlpha, tubeParams.makeUpGain);
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
    doc["eq_low"]         = eqLow;
    doc["eq_mid"]         = eqMid;
    doc["eq_high"]        = eqHigh;
    
    doc["tube_drive"]  = tubeParams.drive;
    doc["tube_cutoff"] = tubeParams.cutoffAlpha;
    doc["tube_gain"]   = tubeParams.makeUpGain;

    serializeJson(doc, f);
    f.close();
    LittleFS.end();

    logSuSeriale(F("[STATE] Salvato su flash: stazione=%d, vol=%d, EQ=(%.1f,%.1f,%.1f), Tube=(%.2f,%.2f,%.2f)\n"),
                 currentStationIdx, lastPos, eqLow, eqMid, eqHigh,
                 tubeParams.drive, tubeParams.cutoffAlpha, tubeParams.makeUpGain);
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
            if (!vuMeterAttivo) setLed(LED_GREEN);
            if (!audio.connecttohost(stations[currentStationIdx].url.c_str()))
                ESP.restart();
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
    int startVolume   = rtcVolume;
    uiRetry           = rtcWifiPowerIdx;

    encoderVolume.setPosition(startVolume / ROTARYSTEPS);
    lastPos = startVolume;

    encoderStazioni.setPosition(currentStationIdx);
    lastStPos = currentStationIdx;

    pinMode(PIN_SW, INPUT_PULLUP);
    btnVolume.attachClick([]() { goToDeepSleep(); });
    btnVolume.attachLongPressStart([]() { enterApModeFromButton(); });
    btnVolume.setPressMs(AP_LONGPRESS_MS);

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
            server.on("/seteq",  HTTP_POST, handleSetEq);
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

    wifiPWR[0]=WIFI_POWER_19_5dBm; wifiPWR[1]=WIFI_POWER_19dBm; wifiPWR[2]=WIFI_POWER_18_5dBm;
    wifiPWR[3]=WIFI_POWER_17dBm;   wifiPWR[4]=WIFI_POWER_15dBm; wifiPWR[5]=WIFI_POWER_13dBm;
    wifiPWR[6]=WIFI_POWER_11dBm;   wifiPWR[7]=WIFI_POWER_8_5dBm;wifiPWR[8]=WIFI_POWER_7dBm;
    wifiPWR[9]=WIFI_POWER_5dBm;    wifiPWR[10]=WIFI_POWER_2dBm;    
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
                IPAddress local_IP(192, 168, 1, 201);
                IPAddress gateway(192, 168, 1, 1);
                IPAddress subnet(255, 255, 255, 0);
                IPAddress primaryDNS(192, 168, 1, 1);                 
                setLed(LED_YELLOW);
                WiFi.disconnect(true, true); 
                delay(100);
                WiFi.setAutoReconnect(false);
                delay(100);
                WiFi.mode(WIFI_STA);
                delay(100);
                WiFi.config(local_IP, gateway, subnet, primaryDNS);
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
                if (rtcWifiPowerIdx != uiRetry) {
                    rtcWifiPowerIdx = uiRetry;
                    saveState();
                }

                setLed(LED_CYAN);
                isSpeakingStation = true;
                ttsStartTime = millis();
                audio.stopSong();
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
            audio.loop();
            encoderVolume.tick();
            encoderStazioni.tick();
            btnVolume.tick();
            btnStazioni.tick();

            if (vuMeterAttivo && !isSpeakingStation) { 
                aggiornaLedVuMeter(audio.getVUlevel());
            }

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
    if (!LittleFS.begin(true)) return false;
    File f = LittleFS.open("/stations.json", "r");
    if (!f) { LittleFS.end(); return false; }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    LittleFS.end();

    if (err) return false;

    JsonArray arr = doc["stations"].as<JsonArray>();
    if (arr.isNull()) return false;

    stations.clear();
    stations.reserve(arr.size());

    for (JsonObject s : arr) {
        stations.push_back({
            s["name"].as<String>(),
            s["url"].as<String>(),
            s["nameLang"] | "it"
        });
    }
    return true;
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
        
        .eq-group { margin-bottom: 14px; }

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
        <h3>🎛️ Equalizzatore (Preset Tono)</h3>
        <div class='eq-group'>
            <label for='eq-preset'>Profilo Audio</label>
            <select id='eq-preset' style='width: 100%; padding: 12px; border-radius: 8px; border: 1px solid #cbd5e1; font-size: 1rem; background-color: #f8fafc;' onchange='applyPreset(this.value)'>
                <option value='0,0,0,1,1,1'>Flat / Neutro</option>
                <option value='5,-4,4,1,1,1.1'>Pure Evoke (Originale)</option>
                <option value='5,-4,4,1.2,0.4,1.1'>Pure Evoke Valvolare</option>
                <option value='3,1,-1,1.25,0.3,1.1'>Warm Vintage</option>
                <option value='4,-1,3.5,1.35,0.45,1'>Rock Overdrive</option>
                <option value='-3,4,-2,1,1,1'>Parlato / News</option>
            </select>
        </div>
    </div>

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
    server.sendContent("let currentEq = { low: " + String(eqLow, 2) + ", mid: " + String(eqMid, 2) + ", high: " + String(eqHigh, 2) + " };\n");
    server.sendContent("let tubeParams = { drive: " + String(tubeParams.drive, 2) + ", cutoff: " + String(tubeParams.cutoffAlpha, 2) + ", gain: " + String(tubeParams.makeUpGain, 2) + " };\n");

    server.sendContent(R"raw(
    function initEq() {
        const select = document.getElementById('eq-preset');
        
        // Confronto basato sulla convergenza dei float
        const currentVec = [
            parseFloat(currentEq.low),
            parseFloat(currentEq.mid),
            parseFloat(currentEq.high),
            parseFloat(tubeParams.drive),
            parseFloat(tubeParams.cutoff),
            parseFloat(tubeParams.gain)
        ];

        let found = false;
        for (let i = 0; i < select.options.length; i++) {
            const optParts = select.options[i].value.split(',').map(v => parseFloat(v));
            
            let match = true;
            for(let j = 0; j < 6; j++) {
                if (Math.abs(optParts[j] - currentVec[j]) > 0.01) {
                    match = false;
                    break;
                }
            }

            if (match) {
                select.selectedIndex = i;
                found = true;
                break;
            }
        }

        if (!found) {
            let opt = document.createElement('option');
            opt.value = `${currentEq.low},${currentEq.mid},${currentEq.high},${tubeParams.drive},${tubeParams.cutoff},${tubeParams.gain}`;
            opt.innerText = `Personalizzato (${currentEq.low}, ${currentEq.mid}, ${currentEq.high})`;
            opt.selected = true;
            select.appendChild(opt);
        }
    }

    function applyPreset(valStr) {
        const parts = valStr.split(',');
        if (parts.length < 3) return;

        const l = parts[0];
        const m = parts[1];
        const h = parts[2];
        const drive = parts[3] !== undefined ? parts[3] : 1.0;
        const cutoff = parts[4] !== undefined ? parts[4] : 1.0;
        const gain = parts[5] !== undefined ? parts[5] : 1.0;

        fetch('/seteq', {
            method: 'POST',
            headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
            body: `low=${l}&mid=${m}&high=${h}&drive=${drive}&cutoff=${cutoff}&gain=${gain}`
        }).then(res => {
            if (!res.ok) alert('Errore nell\'applicazione del preset!');
        });
    }

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

    initEq();
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

void handleSetEq() {
    if (server.hasArg("low") && server.hasArg("mid") && server.hasArg("high")) {
        eqLow  = server.arg("low").toFloat();
        eqMid  = server.arg("mid").toFloat();
        eqHigh = server.arg("high").toFloat();

        if (server.hasArg("drive")) tubeParams.drive = server.arg("drive").toFloat();
        if (server.hasArg("cutoff")) tubeParams.cutoffAlpha = server.arg("cutoff").toFloat();
        if (server.hasArg("gain")) tubeParams.makeUpGain = server.arg("gain").toFloat();

        audio.setTone(eqLow, eqMid, eqHigh);
        saveState();

        logSuSeriale(F("[EQ] Nuovo EQ applicato: Low=%.1f, Mid=%.1f, High=%.1f, Tube=(%.2f,%.2f,%.2f)\n"), 
                     eqLow, eqMid, eqHigh, tubeParams.drive, tubeParams.cutoffAlpha, tubeParams.makeUpGain);
        server.send(200, "text/plain", "OK");
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

// Processing Audio RAW (Valvolare)
void audio_process_raw_samples(int32_t* outBuff, int16_t validSamples) {
    if (tubeParams.drive <= 1.0f) return;

    const int totalSamples = validSamples * 2; 

    for (int i = 0; i < totalSamples; i++) {
        float x = (float)outBuff[i] / 2147483647.0f;
        
        x = x * tubeParams.drive;

        if (x > 1.0f) {
            x = 1.0f;
        } else if (x < -1.0f) {
            x = -1.0f;
        } else {
            x = x - (x * x * x) * 0.3333f;
        }

        x = x * tubeParams.makeUpGain;

        if (i % 2 == 0) {
            x = lastSampleL + tubeParams.cutoffAlpha * (x - lastSampleL);
            lastSampleL = x;
        } else {
            x = lastSampleR + tubeParams.cutoffAlpha * (x - lastSampleR);
            lastSampleR = x;
        }

        if (x > 0.999f)  x = 0.999f;
        if (x < -0.999f) x = -0.999f;

        outBuff[i] = (int32_t)(x * 2147483647.0f);
    }
}