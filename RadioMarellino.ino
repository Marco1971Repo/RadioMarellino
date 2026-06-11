#include <WiFi.h>
#include <WebServer.h>
#include <Audio.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <RotaryEncoder.h>
#include <OneButton.h>
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

// ── Parametri encoder volume ──────────────────────────────────────────────────
#define ROTARYSTEPS  1
#define ROTARYMIN    0
#define ROTARYMAX    64
#define VOLUME_DEFAULT 16

// ── GPIO wakeup deep sleep ────────────────────────────────────────────────────
#define SLEEP_WAKEUP_GPIO  GPIO_NUM_9

// ── RTC memory: sopravvive al deep sleep ──────────────────────────────────────
RTC_DATA_ATTR int  rtcStationIdx = 0;
RTC_DATA_ATTR int  rtcVolume     = VOLUME_DEFAULT;

// ── State machine ─────────────────────────────────────────────────────────────
enum MachineStates {
    STATE_INIT,
    STATE_WAITWIFICONNECTION,
    STATE_PLAYER,
    STATE_START_AP,
    STATE_AP_MODE,
};

MachineStates currentState = STATE_INIT;

// ── Oggetti principali ────────────────────────────────────────────────────────
Audio     audio;
WebServer server(80);

// ── Encoder e pulsante ────────────────────────────────────────────────────────
RotaryEncoder encoderVolume  (PIN_DT,     PIN_CLK,     RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoderStazioni(PIN_ST_DT,  PIN_ST_CLK,  RotaryEncoder::LatchMode::TWO03);
OneButton     btnVolume(PIN_SW, true, true);  // attivo LOW, pull-up interno

// ── Variabili encoder ─────────────────────────────────────────────────────────
int lastPos      = -1;
int lastStPos    = 0;
int currentStationIdx = 0;

// ── Credenziali WiFi ──────────────────────────────────────────────────────────
String wifiSsid = "";
String wifiPass = "";

// ── Timeout connessione WiFi ──────────────────────────────────────────────────
unsigned long connectionStartTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 15000;

// ── Stato riproduzione ────────────────────────────────────────────────────────
String pendingUrl       = "";
bool   hasPendingPlay   = false;
bool   isSpeakingStation = false;

// ── Prototipi ─────────────────────────────────────────────────────────────────
void logSuSeriale(const __FlashStringHelper *frmt, ...);
bool loadStations();
bool loadWifiConfig();
bool saveWifiConfig(const String& ssid, const String& pass);
void goToDeepSleep();
void handleRoot();
void handleSave();

// ─────────────────────────────────────────────────────────────────────────────
// Deep Sleep
// ─────────────────────────────────────────────────────────────────────────────
void goToDeepSleep() {
    logSuSeriale(F("[SLEEP] Salvo stato: stazione=%d, volume=%d\n"),
                 currentStationIdx, lastPos);

    rtcStationIdx = currentStationIdx;
    rtcVolume     = (lastPos >= ROTARYMIN) ? lastPos : VOLUME_DEFAULT;

    audio.stopSong();
    delay(100);

    logSuSeriale(F("[SLEEP] Entro in deep sleep. Premi il pulsante per riaccendere.\n"));
    Serial.flush();

    esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKEUP_GPIO,
                                 ESP_EXT1_WAKEUP_ANY_LOW);
    esp_deep_sleep_start();
}

// ─────────────────────────────────────────────────────────────────────────────
// Configurazione WiFi (LittleFS)
// ─────────────────────────────────────────────────────────────────────────────
bool loadWifiConfig() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[CFG] LittleFS mount fallito\n"));
        return false;
    }

    File f = LittleFS.open("/config.json", "r");
    if (!f) {
        logSuSeriale(F("[CFG] config.json non trovato\n"));
        LittleFS.end();
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    LittleFS.end();

    if (err) {
        logSuSeriale(F("[CFG] JSON parse error: %s\n"), err.c_str());
        return false;
    }

    wifiSsid = doc["wifi_ssid"] | "";
    wifiPass = doc["wifi_pass"] | "";

    if (wifiSsid.isEmpty()) {
        logSuSeriale(F("[CFG] SSID vuoto nel config.json\n"));
        return false;
    }

    logSuSeriale(F("[CFG] WiFi config OK: %s\n"), wifiSsid.c_str());
    return true;
}

bool saveWifiConfig(const String& ssid, const String& pass) {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[CFG] LittleFS mount fallito in scrittura\n"));
        return false;
    }

    File f = LittleFS.open("/config.json", "w");
    if (!f) {
        logSuSeriale(F("[CFG] Impossibile creare config.json\n"));
        LittleFS.end();
        return false;
    }

    JsonDocument doc;
    doc["wifi_ssid"] = ssid;
    doc["wifi_pass"] = pass;

    if (serializeJson(doc, f) == 0) {
        logSuSeriale(F("[CFG] Errore nella serializzazione del JSON\n"));
        f.close();
        LittleFS.end();
        return false;
    }

    f.close();
    LittleFS.end();
    logSuSeriale(F("[CFG] Nuove credenziali salvate con successo\n"));
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// WebServer Handlers (modalità AP)
// ─────────────────────────────────────────────────────────────────────────────
void handleRoot() {
    String html = F(
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>body{font-family:sans-serif; margin:20px;}"
        "input[type=text],input[type=password]{width:100%;padding:12px;margin:8px 0;"
        "border:1px solid #ccc;box-sizing:border-box;}"
        "button{background-color:#4CAF50;color:white;padding:14px 20px;margin:8px 0;"
        "border:none;width:100%;cursor:pointer;}</style>"
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

        server.send(200, "text/html",
            F("<!DOCTYPE html><html><body>"
              "<h3>Dati salvati. Il dispositivo si sta riavviando...</h3>"
              "</body></html>"));
        delay(1000);

        saveWifiConfig(reqSsid, reqPass);
        logSuSeriale(F("[AP] Riavvio in corso...\n"));
        ESP.restart();
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Callback audio
// ─────────────────────────────────────────────────────────────────────────────
void audio_info(const char* info) {
    // Decommenta per debug audio dettagliato:
    // logSuSeriale(F("[AUDIO] %s\n"), info);
}

void audio_eof_speech(const char* info) {
    if (!isSpeakingStation) return;
    isSpeakingStation = false;
    logSuSeriale(F("[TTS] Fine annuncio: %s\n"), stations[currentStationIdx].name.c_str());
    if (!audio.connecttohost(stations[currentStationIdx].url.c_str()))
        ESP.restart();
}

// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
//#define CONFIG_WRITE_JSON_RADIO

void setup() {
#ifdef DEBUGGAME
    Serial.begin(115200);
#endif

    // ── Log causa wakeup ──────────────────────────────────────────────────────
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT1)
        logSuSeriale(F("[BOOT] Wakeup da deep sleep (pulsante)\n"));
    else{
        logSuSeriale(F("[BOOT] Power-on da corrente, torno in sleep\n"));
        Serial.flush();
        esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKEUP_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
        esp_deep_sleep_start();
    }
    // ── Ripristino stato da RTC memory ────────────────────────────────────────
    currentStationIdx = rtcStationIdx;
    int startVolume   = rtcVolume;
    logSuSeriale(F("[BOOT] Stato ripristinato: stazione=%d, volume=%d\n"),
                 currentStationIdx, startVolume);

    // ── Encoder volume: posizione iniziale dal valore salvato ─────────────────
    encoderVolume.setPosition(startVolume / ROTARYSTEPS);
    lastPos = startVolume;

    // ── Encoder stazioni: posizione iniziale allineata alla stazione corrente ─
    encoderStazioni.setPosition(currentStationIdx);
    lastStPos = currentStationIdx;

    // ── Pulsante encoder volume → deep sleep al click ─────────────────────────
    pinMode(PIN_SW, INPUT_PULLUP);
    btnVolume.attachClick([]() {
        goToDeepSleep();
    });
    // Opzionale: doppio click per funzione futura (es. mute, cambio modalità)
    // btnVolume.attachDoubleClick([]() { ... });

    // ── BCLK drive strength (necessario con due MAX98357A in parallelo) ────────
    gpio_set_drive_capability((gpio_num_t)I2S_BCLK, GPIO_DRIVE_CAP_3);

#ifdef CONFIG_WRITE_JSON_RADIO
    // Blocco di ripristino/creazione forzata del JSON stazioni
    if (LittleFS.begin(true)) {
        LittleFS.remove("/stations.json");
        File f = LittleFS.open("/stations.json", "w");
        if (f) {
            f.print(R"(
{"stations":[
{"name":"Radio Deejay","url":"http://streamcdnb1-4c4b867c89244861ac216426883d1ad0.msvdn.net/radiodeejay/radiodeejay/play1.m3u8","nameLang":"en"},
{"name":"Virgin Radio","url":"http://icy.unitedradio.it/Virgin.mp3","nameLang":"en"},
{"name":"Virgin Rock 80","url":"http://icy.unitedradio.it/VirginRock80.mp3","nameLang":"en"},
{"name":"Virgin Rock 90","url":"http://icy.unitedradio.it/Virgin_03.mp3","nameLang":"en"},
{"name":"Virgin Classic Rock","url":"http://icy.unitedradio.it/VirginRockClassics.mp3","nameLang":"en"},
{"name":"Virgin Queen","url":"http://icy.unitedradio.it/Virgin_05.mp3","nameLang":"en"},
{"name":"Virgin AC-DC","url":"http://icy.unitedradio.it/um1026.mp3","nameLang":"en"},
{"name":"Controradio","url":"http://streaming.controradio.it:8190/;?type=http&nocache=76494","nameLang":"it"},
{"name":"Deejay 80","url":"http://streamcdnf25-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejay80/live.m3u8","nameLang":"en"},
{"name":"On The Road","url":"http://streamcdnm5-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejayontheroad/live.m3u8","nameLang":"en"},
{"name":"Tropical Pizza","url":"http://streamcdnm12-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejaytropicalpizza/live.m3u8","nameLang":"it"},
{"name":"Mitology","url":"http://onair15.xdevel.com:9120/;stream.mp3","nameLang":"en"},
{"name":"RTL 102.5","url":"https://dd782ed59e2a4e86aabf6fc508674b59.msvdn.net/live/S97044836/tbbP8T1ZRPBL/playlist_audio.m3u8","nameLang":"it"},
{"name":"Radio 105","url":"http://icecast.unitedradio.it/Radio105.mp3","nameLang":"it"},
{"name":"Subasio","url":"http://icy.unitedradio.it/Subasio.mp3","nameLang":"it"},
{"name":"M2O","url":"http://4c4b867c89244861ac216426883d1ad0.msvdn.net/radiom2o/radiom2o/master_ma.m3u8","nameLang":"it"}
]})");
            f.close();
        }
        LittleFS.end();
    }
#endif

    loadStations();

    // ── Clamp stazione salvata in caso il JSON sia cambiato ───────────────────
    if (!stations.empty() && currentStationIdx >= (int)stations.size())
        currentStationIdx = 0;

    // ── Diagnostica memoria ───────────────────────────────────────────────────
    uint32_t psram_size = ESP.getPsramSize();
    if (psram_size > 0)
        logSuSeriale(F("PSRAM OK: %u KB tot, %u KB liberi\n"),
                     psram_size / 1024, ESP.getFreePsram() / 1024);
    else
        logSuSeriale(F("PSRAM: non rilevata\n"));

    logSuSeriale(F("SRAM: %u KB tot, %u KB liberi\n"),
                 ESP.getHeapSize() / 1024, ESP.getFreeHeap() / 1024);
    logSuSeriale(F("--------------------------------\n"));
}

// ─────────────────────────────────────────────────────────────────────────────
// Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    switch (currentState) {

        // ── INIT ──────────────────────────────────────────────────────────────
        case STATE_INIT:
            logSuSeriale(F("[STATE] INIT\n"));
            audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
            audio.setVolumeSteps(64);
            audio.setVolume(lastPos);  // Volume ripristinato da RTC

            if (loadWifiConfig()) {
                logSuSeriale(F("[WiFi] Tentativo connessione a: %s\n"), wifiSsid.c_str());
                WiFi.mode(WIFI_STA);
                WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
                connectionStartTime = millis();
                currentState = STATE_WAITWIFICONNECTION;
            } else {
                logSuSeriale(F("[WiFi] Credenziali non trovate. Avvio AP.\n"));
                currentState = STATE_START_AP;
            }
            break;

        // ── ATTESA WIFI ───────────────────────────────────────────────────────
        case STATE_WAITWIFICONNECTION:
            if (WiFi.status() != WL_CONNECTED) {
                if (millis() - connectionStartTime > WIFI_TIMEOUT_MS) {
                    logSuSeriale(F("\n[WiFi] Timeout. Passaggio ad AP.\n"));
                    WiFi.disconnect();
                    currentState = STATE_START_AP;
                } else {
                    delay(500);
                    logSuSeriale(F("."));
                }
            } else {
                logSuSeriale(F("\n[WiFi] Connesso - IP: %s\n"),
                             WiFi.localIP().toString().c_str());
                // Annuncio stazione ripristinata
                isSpeakingStation = true;
                audio.connecttospeech(stations[currentStationIdx].name.c_str(),
                                      stations[currentStationIdx].nameLang.c_str());
                currentState = STATE_PLAYER;
            }
            break;

        // ── AVVIO AP ──────────────────────────────────────────────────────────
        case STATE_START_AP:
        {
            logSuSeriale(F("[STATE] START AP\n"));
            WiFi.disconnect();
            WiFi.mode(WIFI_AP);
            IPAddress local_IP(192, 168, 1, 1);
            IPAddress gateway(192, 168, 1, 1);
            IPAddress subnet(255, 255, 255, 0);

            if (!WiFi.softAPConfig(local_IP, gateway, subnet))
                logSuSeriale(F("[AP] Configurazione IP statico fallita!\n"));

            WiFi.softAP("RadioMarellino_Setup", "");
            logSuSeriale(F("[AP] SSID: RadioMarellino_Setup  IP: %s\n"),
                         WiFi.softAPIP().toString().c_str());

            server.on("/",     HTTP_GET,  handleRoot);
            server.on("/save", HTTP_POST, handleSave);
            server.begin();
            logSuSeriale(F("[HTTP] Server avviato\n"));

            currentState = STATE_AP_MODE;
            break;
        }

        // ── AP MODE ───────────────────────────────────────────────────────────
        case STATE_AP_MODE:
            // Anche in AP permettiamo il deep sleep dal pulsante
            btnVolume.tick();
            server.handleClient();
            delay(2);
            break;

        // ── PLAYER ───────────────────────────────────────────────────────────
        case STATE_PLAYER:
        {
            audio.loop();
            encoderVolume.tick();
            encoderStazioni.tick();
            btnVolume.tick();   // gestione click → deep sleep

            // ── Pending play (redirect) ───────────────────────────────────────
            if (hasPendingPlay) {
                hasPendingPlay = false;
                logSuSeriale(F("[PLAYER] Riconnessione a: %s\n"), pendingUrl.c_str());
                audio.connecttohost(pendingUrl.c_str());
            }

            // ── Encoder Volume ────────────────────────────────────────────────
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
                // Aggiorno subito RTC così un eventuale crash non perde il volume
                rtcVolume = lastPos;
                logSuSeriale(F("[VOL] Volume: %d\n"), lastPos);
            }

            // ── Encoder Stazioni ──────────────────────────────────────────────
            int newStPos = encoderStazioni.getPosition();
            if (lastStPos != newStPos) {
                if (isSpeakingStation) {
                    // Annuncio in corso: ignora i tick accumulati
                    encoderStazioni.setPosition(currentStationIdx);
                    lastStPos = currentStationIdx;
                } else if (!stations.empty()) {
                    if (newStPos > lastStPos)
                        currentStationIdx = (currentStationIdx + 1) % (int)stations.size();
                    else
                        currentStationIdx = (currentStationIdx - 1 + (int)stations.size()) % (int)stations.size();

                    encoderStazioni.setPosition(currentStationIdx);
                    lastStPos = currentStationIdx;

                    // Aggiorno RTC subito
                    rtcStationIdx = currentStationIdx;

                    logSuSeriale(F("[PLAYER] Cambio stazione: %s\n"),
                                 stations[currentStationIdx].name.c_str());
                    isSpeakingStation = true;
                    audio.connecttospeech(stations[currentStationIdx].name.c_str(),
                                          stations[currentStationIdx].nameLang.c_str());
                }
            }

            vTaskDelay(1);
            break;
        }

        default:
            break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Caricamento stazioni da LittleFS
// ─────────────────────────────────────────────────────────────────────────────
bool loadStations() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[ERR] Impossibile inizializzare LittleFS\n"));
        return false;
    }

    File f = LittleFS.open("/stations.json", "r");
    if (!f) {
        logSuSeriale(F("[ERR] Impossibile aprire stations.json\n"));
        LittleFS.end();
        return false;
    }

    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    LittleFS.end();

    if (err) {
        logSuSeriale(F("[ERR] Errore parsing JSON: %s\n"), err.c_str());
        return false;
    }

    JsonArray arr = doc["stations"].as<JsonArray>();
    if (arr.isNull()) {
        logSuSeriale(F("[ERR] Formato JSON non valido (manca array 'stations')\n"));
        return false;
    }

    stations.clear();
    stations.reserve(arr.size());

    for (JsonObject s : arr) {
        stations.push_back({
            s["name"].as<String>(),
            s["url"].as<String>(),
            s["nameLang"] | "it"   // default "it" se manca il campo
        });
    }

    logSuSeriale(F("[CFG] Stazioni caricate: %d\n"), stations.size());
    return true;
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
