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
// WA: il flag RTC_DATA_ATTR per distinguere un ESP.restart() voluto da un
// power-on da blackout NON sopravvive al soft-reset su questa board/SDK
// (verificato sul campo). Sostituito con un marker su LittleFS, vedi
// markSkipSleepOnBoot() / consumeSkipSleepFlag() più sotto.

// ── State machine ─────────────────────────────────────────────────────────────
enum MachineStates {
    STATE_INIT,
    STATE_WAITWIFICONNECTION,
    STATE_PLAYER,
    STATE_START_AP,
    STATE_AP_MODE,
    STATE_STATIONS_CONFIG, // <-- NUOVO STATO: Portale modifica stazioni da connessi
};

MachineStates currentState = STATE_INIT;

// ── Oggetti principali ────────────────────────────────────────────────────────
Audio     audio;
WebServer server(80);

// ── Encoder e pulsante ────────────────────────────────────────────────────────
RotaryEncoder encoderVolume  (PIN_DT,    PIN_CLK,    RotaryEncoder::LatchMode::TWO03);
RotaryEncoder encoderStazioni(PIN_ST_DT, PIN_ST_CLK, RotaryEncoder::LatchMode::TWO03);
OneButton     btnVolume(PIN_SW, true, true);  // attivo LOW, pull-up interno
OneButton     btnStazioni(PIN_ST_SW, true, true); // <-- NUOVO PULSANTE STAZIONI

// ── Variabili encoder ─────────────────────────────────────────────────────────
int lastPos           = -1;
int lastStPos         = 0;
int currentStationIdx = 0;
bool vuMeterAttivo    = false;  // <-- MODIFICA: Flag per gestire lo stato ON/OFF del VU-Meter

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
// La libreria ESP32-audioI2S non genera sempre evt_eof al termine di
// connecttospeech() (segnalato a Wolle). Questo timer forza comunque il
// passaggio allo stream della stazione se l'evento non arriva in tempo.
unsigned long ttsStartTime  = 0;
const unsigned long TTS_TIMEOUT_MS = 8000;  // timeout di sicurezza annuncio TTS

// ── Lampeggio LED in AP mode ──────────────────────────────────────────────────
unsigned long lastBlinkTime = 0;
bool          blinkState    = false;

// ── Throttle salvataggio stato su flash ───────────────────────────────────────
// Evita scritture LittleFS continue durante la rotazione dell'encoder volume
unsigned long lastSaveTime  = 0;
bool          pendingSave   = false;
const unsigned long SAVE_DEBOUNCE_MS = 2000;  // salva 2s dopo l'ultima modifica

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
// Persistenza stato su LittleFS (sopravvive al blackout)
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
        rtcStationIdx = doc["station"] | 0;
        rtcVolume     = doc["volume"]  | VOLUME_DEFAULT;
        rtcWifiPowerIdx = doc["wifi_power_idx"] | 0; // <-- ADESSO ASSEGNATO CORRETTAMENTE QUI
        logSuSeriale(F("[STATE] Caricato da flash: stazione=%d, volume=%d, wifi_power_idx=%d\n"),
                     rtcStationIdx, rtcVolume, rtcWifiPowerIdx);
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
    doc["station"] = currentStationIdx;
    doc["volume"]  = lastPos;
    doc["wifi_power_idx"] = uiRetry;
    serializeJson(doc, f);
    f.close();
    LittleFS.end();

    logSuSeriale(F("[STATE] Salvato su flash: stazione=%d, volume=%d, wifi_power_idx=%d\n"),
                 currentStationIdx, lastPos, uiRetry);
}

// Pianifica un salvataggio ritardato (evita scritture flash continue
// durante la rotazione dell'encoder volume)
void scheduleSave() {
    lastSaveTime = millis();
    pendingSave  = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// WA: marker su flash per distinguere un ESP.restart() voluto (es. dopo
// salvataggio credenziali WiFi dal portale AP) da un power-on da blackout. Sostituisce il
// tentativo con RTC_DATA_ATTR, che non sopravvive al soft-reset su questa
// board/SDK (verificato sul campo).
// ─────────────────────────────────────────────────────────────────────────────
void markSkipSleepOnBoot() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[BOOT] LittleFS mount fallito, impossibile scrivere skip_sleep.flag\n"));
        return;
    }
    File f = LittleFS.open("/skip_sleep.flag", "w");
    if (f) {
        f.close();
        logSuSeriale(F("[BOOT] skip_sleep.flag scritto\n"));
    } else {
        logSuSeriale(F("[BOOT] Impossibile creare skip_sleep.flag\n"));
    }
    LittleFS.end();
}

bool consumeSkipSleepFlag() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[BOOT] LittleFS mount fallito, impossibile leggere skip_sleep.flag\n"));
        return false;
    }
    bool skip = LittleFS.exists("/skip_sleep.flag");
    if (skip) {
        LittleFS.remove("/skip_sleep.flag");  // one-shot: consumato subito
        logSuSeriale(F("[BOOT] skip_sleep.flag trovato e consumato\n"));
    }
    LittleFS.end();
    return skip;
}

// ─────────────────────────────────────────────────────────────────────────────
// Ingresso in modalità AP su richiesta utente (pressione prolungata pulsante)
// ─────────────────────────────────────────────────────────────────────────────
void enterApModeFromButton() {
    logSuSeriale(F("[BTN] Pressione prolungata rilevata: richiesta modalità AP\n"));

    // Ferma la riproduzione/eventuale annuncio TTS in corso, per non lasciare
    // l'I2S "appeso" mentre si passa alla configurazione WiFi
    audio.stopSong();
    isSpeakingStation = false;
    hasPendingPlay    = false;
    setLed(LED_OFF);

    // Il case STATE_START_AP nel loop() si occupa di WiFi.disconnect(),
    // avvio softAP e webserver: basta impostare lo stato, viene gestito
    // al prossimo giro di loop()
    currentState = STATE_START_AP;
}

// ─────────────────────────────────────────────────────────────────────────────
// Deep Sleep
// ─────────────────────────────────────────────────────────────────────────────
void goToDeepSleep() {
    logSuSeriale(F("[SLEEP] Salvo stato: stazione=%d, volume=%d\n"),
                 currentStationIdx, lastPos);

    // Aggiorno RTC memory
    rtcStationIdx = currentStationIdx;
    rtcVolume     = (lastPos >= ROTARYMIN) ? lastPos : VOLUME_DEFAULT;
    rtcWifiPowerIdx = uiRetry;

    // Salvo su flash (garantisce persistenza anche dopo blackout)
    saveState();

    audio.stopSong();
    setLed(LED_OFF);
    delay(100);

    logSuSeriale(F("[SLEEP] Entro in deep sleep. Premi il pulsante per riaccendere.\n"));
    Serial.flush();

    esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKEUP_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
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
        "2<h2>Configurazione WiFi</h2>"
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

        // Se siamo connessi e usiamo il form opzionale, recuperiamo i vecchi dati se i campi sono vuoti
        if (currentState == STATE_STATIONS_CONFIG) {
            // Se l'utente ha lasciato l'SSID vuoto, mantieni quello attuale in RAM/config
            if (reqSsid.isEmpty()) {
                reqSsid = wifiSsid;
            }
            // Se l'utente ha lasciato la Password vuota, mantieni quella attuale in RAM/config
            if (reqPass.isEmpty()) {
                reqPass = wifiPass;
            }
        }
        // -------------------------------------------------------

        server.send(200, "text/html",
            F("<!DOCTYPE html><html><body>"
              "<h3>Dati salvati. Il dispositivo si sta riavviando...</h3>"
              "</body></html>"));
        delay(1000);

        saveWifiConfig(reqSsid, reqPass);
        markSkipSleepOnBoot();  // WA: al prossimo boot salta il ritorno in sleep (marker su flash)
        logSuSeriale(F("[AP] Riavvio in corso...\n"));
        ESP.restart();
    } else if (currentState == STATE_STATIONS_CONFIG) {
        // Se siamo in config stazioni, l'azione "Salva e Riavvia" (pulsante verde in fondo) 
        // richiede solo il reboot salvando lo stato delle stazioni
        server.send(200, "text/html",
            F("<!DOCTYPE html><html><body>"
              "<h3>Configurazione completata. Riavvio in corso...</h3>"
              "</body></html>"));
        delay(1000);
        markSkipSleepOnBoot();
        logSuSeriale(F("[CFG] Salvo ed esco. Riavvio in corso...\n"));
        ESP.restart();
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}
// ─────────────────────────────────────────────────────────────────────────────
// Callback audio
// ─────────────────────────────────────────────────────────────────────────────
void my_audio_info(Audio::msg_t m) {
    // Decommenta per debug audio dettagliato:
    logSuSeriale(F("[AUDIO-RAW] evt=%d s=%s msg=%s\n"), (int)m.e, m.s, m.msg);
    if (m.e == Audio::evt_eof) {
        if (isSpeakingStation) {
            isSpeakingStation = false;
            logSuSeriale(F("[TTS] Fine annuncio: %s\n"), stations[currentStationIdx].name.c_str());
            if (!vuMeterAttivo) {
                setLed(LED_GREEN);   // verde: riproduzione (solo se VU-Meter disattivato)
            }
            if (!audio.connecttohost(stations[currentStationIdx].url.c_str()))
                ESP.restart();
        }
    }     
}


// ─────────────────────────────────────────────────────────────────────────────
// Setup
// ─────────────────────────────────────────────────────────────────────────────
//#define CONFIG_WRITE_JSON_RADIO
wifi_power_t wifiPWR[11];

void setup() {
    Audio::audio_info_callback = my_audio_info;
#ifdef DEBUGGAME
    Serial.begin(115200);
#endif

    // ── LED: init immediato, visibile fin dal boot ────────────────────────────
    rgb.begin();
    rgb.setBrightness(255);
    setLed(LED_OFF);

    // ── Causa wakeup ──────────────────────────────────────────────────────────
    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    if (cause == ESP_SLEEP_WAKEUP_EXT1) {
        logSuSeriale(F("[BOOT] Wakeup da pulsante, avvio normale\n"));
    } else if (consumeSkipSleepFlag()) {
        // WA: restart voluto (es. dopo salvataggio credenziali WiFi dal portale AP):
        // salto il ritorno in sleep e procedo con il boot normale
        logSuSeriale(F("[BOOT] Restart dopo config WiFi, avvio normale\n"));
    } else {
        // Power-on da corrente (blackout, prima accensione): torna in sleep
        logSuSeriale(F("[BOOT] Power-on da corrente, torno in sleep\n"));
        Serial.flush();
        esp_sleep_enable_ext1_wakeup(1ULL << SLEEP_WAKEUP_GPIO, ESP_EXT1_WAKEUP_ANY_LOW);
        esp_deep_sleep_start();
    }

    // ── Ripristino stato: prima da flash (resiste al blackout),
    //    poi i valori finiscono in rtcStationIdx/rtcVolume ────────────────────
    loadState();   // sovrascrive rtcStationIdx e rtcVolume con i dati da flash

    currentStationIdx = rtcStationIdx;
    int startVolume   = rtcVolume;
    uiRetry           = rtcWifiPowerIdx;
    logSuSeriale(F("[BOOT] Stato ripristinato: stazione=%d, volume=%d, wifi_power_idx=%d\n"),
                 currentStationIdx, startVolume, uiRetry);

    // ── Encoder volume: posizione iniziale dal valore salvato ─────────────────
    encoderVolume.setPosition(startVolume / ROTARYSTEPS);
    lastPos = startVolume;

    // ── Encoder stazioni: allineato alla stazione corrente ────────────────────
    encoderStazioni.setPosition(currentStationIdx);
    lastStPos = currentStationIdx;

    // ── Pulsante encoder volume ────────────────────────────────────────────────
    // Click breve  → deep sleep
    // Pressione prolungata (>= AP_LONGPRESS_MS) → modalità AP per reinserire
    // le credenziali WiFi. OneButton distingue i due casi da sola: se la
    // pressione supera la soglia, il click breve NON scatta più.
    pinMode(PIN_SW, INPUT_PULLUP);
    btnVolume.attachClick([]() {
        goToDeepSleep();
    });
    btnVolume.attachLongPressStart([]() {
        enterApModeFromButton();
    });
    btnVolume.setPressMs(AP_LONGPRESS_MS);

    // ── Pulsante encoder stazioni ─────────────────────────────────────────────
    pinMode(PIN_ST_SW, INPUT_PULLUP);

    // MODIFICA: CLICK BREVE → Attiva / Disattiva il VU-Meter
    btnStazioni.attachClick([]() {
        if (currentState == STATE_PLAYER) {
            vuMeterAttivo = !vuMeterAttivo;
            logSuSeriale(F("[BTN] Click breve stazioni: VU-Meter %s\n"), vuMeterAttivo ? "ATTIVATO" : "DISATTIVATO");
            if (!vuMeterAttivo && !isSpeakingStation) {
                setLed(LED_GREEN); // Ripristina LED verde fisso se disattiviamo il VU-meter
            }
        }
    });

    // Pressione prolungata (>= AP_LONGPRESS_MS) da accesi → apre l'editor web
    // delle stazioni sulla rete locale.
    btnStazioni.attachLongPressStart([]() {
        if (currentState == STATE_PLAYER) {
            logSuSeriale(F("[BTN] Pressione prolungata stazioni: configurazione radio ed editor web\n"));
            audio.stopSong();
            isSpeakingStation = false;
            hasPendingPlay    = false;
            vuMeterAttivo     = false; // Disattiva VU-meter se entriamo in config

            // Associa gli handler per l'editor stazioni
            server.on("/",     HTTP_GET,  handleManageStations);
            server.on("/add",  HTTP_POST, handleAddStation);
            server.on("/delete", HTTP_GET, handleDeleteStation);
            server.on("/save", HTTP_POST, handleSave);
            server.begin();

            logSuSeriale(F("[HTTP] Server gestione stazioni avviato su IP: %s\n"), WiFi.localIP().toString().c_str());
            setLed(LED_CYAN); // Lascia il led fisso su Ciano (Annuncio/Config)
            currentState = STATE_STATIONS_CONFIG;
        }
    });
    btnStazioni.setPressMs(AP_LONGPRESS_MS);

    // ── BCLK drive strength (necessario con due MAX98357A in parallelo) ────────
    gpio_set_drive_capability((gpio_num_t)I2S_BCLK, GPIO_DRIVE_CAP_3);

#ifdef CONFIG_WRITE_JSON_RADIO
    if (LittleFS.begin(true)) {
        LittleFS.remove("/stations.json");
        File f = LittleFS.open("/stations.json", "w");
        if (f) {
            f.print(R"(
{"stations":[
{"name":"Radio Deejay","url":"http://streamcdnb1-4c4b867c89244861ac216426883d1ad0.msvdn.net/radiodeejay/radiodeejay/play1.m3u8","nameLang":"en"},
{"name":"Controradio","url":"http://streaming.controradio.it:8190/;?type=http&nocache=76494","nameLang":"it"},
{"name":"Virgin Radio","url":"http://icy.unitedradio.it/Virgin.mp3","nameLang":"en"},
{"name":"Mitology","url":"http://onair15.xdevel.com:9120/;stream.mp3","nameLang":"en"},
{"name":"BBC World Service","url":"http://stream.live.vc.bbcmedia.co.uk/bbc_world_service","nameLang":"en"},
{"name":"Virgin Rock 80","url":"http://icy.unitedradio.it/VirginRock80.mp3","nameLang":"en"},
{"name":"Virgin Rock 90","url":"http://icy.unitedradio.it/Virgin_03.mp3","nameLang":"en"},
{"name":"Virgin Classic Rock","url":"http://icy.unitedradio.it/VirginRockClassics.mp3","nameLang":"en"},
{"name":"Deejay 80","url":"http://streamcdnf25-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejay80/live.m3u8","nameLang":"en"},
{"name":"On The Road","url":"http://streamcdnm5-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejayontheroad/live.m3u8","nameLang":"en"},
{"name":"Tropical Pizza","url":"http://streamcdnm12-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejaytropicalpizza/live.m3u8","nameLang":"it"},
{"name":"RTL 102.5","url":"https://dd782ed59e2a4e86aabf6fc508674b59.msvdn.net/live/S97044836/tbbP8T1ZRPBL/playlist_audio.m3u8","nameLang":"it"},
{"name":"Radio 105","url":"http://icecast.unitedradio.it/Radio105.mp3","nameLang":"it"},
{"name":"Subasio","url":"http://icy.unitedradio.it/Subasio.mp3","nameLang":"it"},
{"name":"M2O","url":"http://streamcdnf26-4c4b867c89244861ac216426883d1ad0.msvdn.net/radiom2o/radiom2o/play1.m3u8","nameLang":"it"} 
]})");
            f.close();
        }
        LittleFS.end();
    }
#endif

    loadStations();

    // ── Clamp stazione in caso il JSON sia cambiato ───────────────────────────
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
    wifiPWR[0]=WIFI_POWER_19_5dBm;
    wifiPWR[1]=WIFI_POWER_19dBm;
    wifiPWR[2]=WIFI_POWER_18_5dBm;
    wifiPWR[3]=WIFI_POWER_17dBm;
    wifiPWR[4]=WIFI_POWER_15dBm;
    wifiPWR[5]=WIFI_POWER_13dBm;
    wifiPWR[6]=WIFI_POWER_11dBm;
    wifiPWR[7]=WIFI_POWER_8_5dBm;
    wifiPWR[8]=WIFI_POWER_7dBm;
    wifiPWR[9]=WIFI_POWER_5dBm;
    wifiPWR[10]=WIFI_POWER_2dBm;    
}

// ─────────────────────────────────────────────────────────────────────────────
// Gestione dinamica LED come barra VU-Meter basata sul livello (0-255)
// ─────────────────────────────────────────────────────────────────────────────
void aggiornaLedVuMeter(uint8_t livello) {
    static uint8_t  ultimoLivello = 0;
    static float    maxDinamico   = 120.0f; // Partiamo da un picco stimato basso
    static uint32_t ultimoDecadimento = 0;

    if (livello == ultimoLivello) {
        return;
    }
    ultimoLivello = livello;

    // 1. Se il livello attuale supera il massimo storico, aggiorna subito il picco
    if (livello > maxDinamico) {
        maxDinamico = livello;
    }

    // 2. Decadimento lento del picco massimo ogni 100ms
    // Serve a riadattare la scala se la musica passa da un pezzo forte a uno piano
    if (millis() - ultimoDecadimento > 100) {
        if (maxDinamico > 80.0f) { // Non scendere sotto la soglia minima di rumore
            maxDinamico -= 0.5f;   // Fa scendere il picco lentamente
        }
        ultimoDecadimento = millis();
    }

    // 3. Normalizza il valore letto in percentuale (0.0 -> 1.0) rispetto al max attuale
    float percentuale = (float)livello / maxDinamico;
    if (percentuale > 1.0f) percentuale = 1.0f;

    uint8_t r = 0, g = 0, b = 0;

    // 4. Soglie relative in percentuale:
    //    0%  - 50%  -> VERDE
    //    51% - 80%  -> GIALLO
    //    81% - 100% -> ROSSO
    if (percentuale <= 0.50f) {
        // Fascia VERDE
        g = (uint8_t)map(percentuale * 100, 0, 50, 30, 255);
    } 
    else if (percentuale <= 0.80f) {
        // Fascia GIALLA (R + G)
        uint8_t lux = (uint8_t)map(percentuale * 100, 51, 80, 100, 255);
        r = lux;
        g = lux;
    } 
    else {
        // Fascia ROSSA (Picchi relativi)
        r = (uint8_t)map(percentuale * 100, 81, 100, 180, 255);
    }

    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}
/*void aggiornaLedVuMeter(uint8_t livello) {
    static uint8_t ultimoLivello = 0;
    
    // Se il livello non è cambiato rispetto al loop precedente, esci subito
    if (livello == ultimoLivello) {
        return; 
    }
    logSuSeriale(F("[VU-METER] Livello audio: %d\n"), livello);
    uint8_t r = 0, g = 0, b = 0;
    ultimoLivello = livello;
    if (livello <= 95) {
        // Fascia VERDE: la luminosità (G) cresce linearmente da 0 a 255
        // Rimappiamo il range 0-128 sul range di luminosità 0-255
        g = map(livello, 0, 128, 0, 255);
    } 
    else if (livello <= 135) {
        // Fascia GIALLA (Rosso + Verde): la luminosità cresce linearmente da 40 a 255
        uint8_t lux = map(livello, 129, 199, 40, 255);
        r = lux;
        g = lux;
    } 
    else {
        // Fascia ROSSA: la luminosità (R) cresce linearmente da 100 a 255
        r = map(livello, 200, 255, 100, 255);
    }

    // Aggiorna il singolo pixel senza toccare la luminosità globale (setBrightness)
    rgb.setPixelColor(0, rgb.Color(r, g, b));
    rgb.show();
}*/


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
            audio.setVolume(lastPos);  // volume ripristinato da flash
            // ── EQUALIZZATORE TIPO "PURE EVOKE" ─────────────────────────────────
            // audio.setTone(bassi, medi, alti) - Valori espressi in dB
                audio.setTone(5, -4, 4);   
            // ────────────────────────────────────────────────────────────────────
            if (loadWifiConfig()) {
                // Define static IP details shoud make connection faster
                IPAddress local_IP(192, 168, 1, 201);
                IPAddress gateway(192, 168, 1, 1);
                IPAddress subnet(255, 255, 255, 0);
                IPAddress primaryDNS(192, 168, 1, 1);                 
                logSuSeriale(F("[WiFi] Tentativo connessione a: %s-%s (Potenza indice %d)\n"), wifiSsid.c_str(), wifiPass.c_str(), uiRetry);
                setLed(LED_YELLOW);    // giallo: connessione in corso
                WiFi.disconnect(true, true); 
                delay(100);
                WiFi.setAutoReconnect(false);
                delay(100);
                WiFi.mode(WIFI_STA);
                delay(100);
                // Configure the static IP
                if (!WiFi.config(local_IP, gateway, subnet, primaryDNS)) {
                    logSuSeriale(F("STA Configuration Failed"));
                }
                WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
                connectionStartTime = millis();
                currentState = STATE_WAITWIFICONNECTION;
                WiFi.setTxPower(wifiPWR[uiRetry]);
            } else {
                logSuSeriale(F("[WiFi] Credenziali non trovate. Avvio AP.\n"));
                currentState = STATE_START_AP;
            }
            break;

        // ── ATTESA WIFI ───────────────────────────────────────────────────────
        case STATE_WAITWIFICONNECTION:
            btnVolume.tick();  // permette la pressione prolungata anche in questa fase
            btnStazioni.tick();
            if (currentState == STATE_START_AP) break;  // richiesta AP appena arrivata: non sovrascriverla
            if (WiFi.status() != WL_CONNECTED) {
                if (millis() - connectionStartTime > WIFI_TIMEOUT_MS) {
                    logSuSeriale(F("\n[WiFi] Timeout con potenza indice %d.\n"), uiRetry);
                    WiFi.disconnect();
                    uiRetry++;
                    if (uiRetry < 11) {
                        logSuSeriale(F("[WiFi] Riprovo con un nuovo livello di potenza...\n"));
                        currentState = STATE_INIT;
                    } else {
                        logSuSeriale(F("[WiFi] Tutti i livelli di potenza falliti. Passaggio ad AP.\n"));
                        uiRetry = 0;
                        currentState = STATE_START_AP;
                    }
                } else {
                    delay(500);
                    logSuSeriale(F("."));
                }
            } else {
                logSuSeriale(F("\n[WiFi] Connesso - IP: %s\n"),
                             WiFi.localIP().toString().c_str());
                
                // Salvo l'indice di potenza che ha funzionato sia in RTC che su flash
                if (rtcWifiPowerIdx != uiRetry) {
                    rtcWifiPowerIdx = uiRetry;
                    saveState();
                }

                setLed(LED_CYAN);      // ciano: annuncio TTS stazione
                isSpeakingStation = true;
                ttsStartTime = millis();  // WA: avvio timer fallback annuncio TTS
                audio.stopSong(); // Ferma qualsiasi cosa prima
                delay(50);
                audio.connecttospeech(stations[currentStationIdx].name.c_str(), stations[currentStationIdx].nameLang.c_str());
                //audio.connecttohost(stations[currentStationIdx].url.c_str());
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
            WiFi.setTxPower(WIFI_POWER_2dBm); // parti dal minimo, non dal massimo
            bool apOk = WiFi.softAP("RadioMarellino_Setup");
            Serial.printf("[AP] softAP() risultato: %s\n", apOk ? "OK" : "FALLITO");
            logSuSeriale(F("[AP] SSID: RadioMarellino_Setup  IP: %s\n"),
                         WiFi.softAPIP().toString().c_str());

            server.on("/",     HTTP_GET,  handleRoot);
            server.on("/save", HTTP_POST, handleSave);
            server.begin();
            logSuSeriale(F("[HTTP] Server avviato\n"));

            lastBlinkTime = millis();
            uiRetry = 0;
            rtcWifiPowerIdx = uiRetry;
            saveState();
            currentState = STATE_AP_MODE;
            break;
        }

        // ── AP MODE ───────────────────────────────────────────────────────────
        case STATE_AP_MODE:
            // Rosso lampeggiante: nessuna rete configurata
            if (millis() - lastBlinkTime >= 500) {
                lastBlinkTime = millis();
                blinkState = !blinkState;
                setLed(blinkState ? LED_RED : LED_OFF);
            }
            btnVolume.tick();
            server.handleClient();
            delay(2);
            break;

        // ── PLAYER ────────────────────────────────────────────────────────────
        case STATE_PLAYER:
        {
            audio.loop();
            encoderVolume.tick();
            encoderStazioni.tick();
            btnVolume.tick();
            btnStazioni.tick(); // <-- Rileva pressioni sul selettore stazione

            // MODIFICA: Aggiorna il VU-meter solo se è stato abilitato dall'utente
            if (vuMeterAttivo && !isSpeakingStation) { 
                aggiornaLedVuMeter(audio.getVUlevel());
            }

            // ── Salvataggio ritardato su flash ────────────────────────────────
            // Evita scritture continue durante la rotazione dell'encoder volume:
            // scrive su LittleFS solo dopo SAVE_DEBOUNCE_MS di inattività
            if (pendingSave && (millis() - lastSaveTime >= SAVE_DEBOUNCE_MS)) {
                pendingSave = false;
                saveState();
            }

            // ── WA: timeout annuncio TTS (fallback se manca evt_eof) ───────────
            // Se la libreria non genera l'evento di fine annuncio entro
            // TTS_TIMEOUT_MS, forziamo comunque il passaggio allo stream
            // riusando il meccanismo di riconnessione pendente qui sotto.
            if (isSpeakingStation && (millis() - ttsStartTime >= TTS_TIMEOUT_MS)) {
                isSpeakingStation = false;
                logSuSeriale(F("[TTS] Timeout annuncio, forzo passaggio allo stream: %s\n"),
                             stations[currentStationIdx].name.c_str());
                audio.stopSong();
                delay(50);
                if (!vuMeterAttivo) setLed(LED_GREEN);   // verde fisso solo se VU-meter spento
                pendingUrl     = stations[currentStationIdx].url;
                hasPendingPlay = true;
            }

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
                rtcVolume = lastPos;   // aggiorna RTC subito (resiste al crash)
                scheduleSave();        // pianifica scrittura flash ritardata
                logSuSeriale(F("[VOL] Volume: %d\n"), lastPos);
            }

            // ── Encoder Stazioni ──────────────────────────────────────────────
            int newStPos = encoderStazioni.getPosition();
            if (lastStPos != newStPos) {
                if (isSpeakingStation) {
                    // Annuncio in corso: ignora tick accumulati
                    encoderStazioni.setPosition(currentStationIdx);
                    lastStPos = currentStationIdx;
                } else if (!stations.empty()) {
                    if (newStPos > lastStPos)
                        currentStationIdx = (currentStationIdx + 1) % (int)stations.size();
                    else
                        currentStationIdx = (currentStationIdx - 1 + (int)stations.size()) % (int)stations.size();

                    encoderStazioni.setPosition(currentStationIdx);
                    lastStPos     = currentStationIdx;
                    rtcStationIdx = currentStationIdx;  // aggiorna RTC subito
                    scheduleSave();                     // pianifica scrittura flash

                    logSuSeriale(F("[PLAYER] Cambio stazione: %s\n"),
                                 stations[currentStationIdx].name.c_str());
                    setLed(LED_CYAN);   // ciano: annuncio TTS
                    isSpeakingStation = true;
                    ttsStartTime = millis();  // WA: avvio timer fallback annuncio TTS
                    audio.connecttospeech(stations[currentStationIdx].name.c_str(),
                                          stations[currentStationIdx].nameLang.c_str());
                }
            }

            vTaskDelay(1);
            break;
        }

        // ── CONFIGURAZIONE STAZIONI (DA CONNESSI) ─────────────────────────────
        case STATE_STATIONS_CONFIG:
            btnVolume.tick();      // Permette di rientrare in sleep anche in config
            server.handleClient(); // Gestisce le modifiche web alle stazioni
            // Blink ciano per indicare config attiva
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
// Scrittura stazioni su LittleFS (JSON)
// ─────────────────────────────────────────────────────────────────────────────
bool saveStationsToFS() {
    if (!LittleFS.begin(true)) {
        logSuSeriale(F("[ERR] Impossibile avviare LittleFS per salvare\n"));
        return false;
    }

    File f = LittleFS.open("/stations.json", "w");
    if (!f) {
        logSuSeriale(F("[ERR] Impossibile creare stations.json in scrittura\n"));
        LittleFS.end();
        return false;
    }

    JsonDocument doc;
    JsonArray arr = doc["stations"].to<JsonArray>();
    for (const auto& s : stations) {
        JsonObject obj = arr.add<JsonObject>();
        obj["name"] = s.name;
        obj["url"] = s.url;
        obj["nameLang"] = s.nameLang;
    }

    if (serializeJson(doc, f) == 0) {
        logSuSeriale(F("[ERR] Fallita la serializzazione stazioni\n"));
        f.close();
        LittleFS.end();
        return false;
    }

    f.close();
    LittleFS.end();
    logSuSeriale(F("[CFG] stations.json aggiornato con successo\n"));
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Escaping per iniettare stringhe utente dentro il letterale JS `let stations = [...]`
// (name/url arrivano da un <input type=text>: possono contenere ", \ o backtick)
// ─────────────────────────────────────────────────────────────────────────────
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
            case '\r': break;  // scartato
            default:   out += c;
        }
    }
    return out;
}

// ─────────────────────────────────────────────────────────────────────────────
// Gestori WebServer dedicati alla modifica delle stazioni
// ─────────────────────────────────────────────────────────────────────────────
void handleManageStations() {
    // Serializza la lista attuale in formato JSON array per passarla a JavaScript
    // (nomi/url escapati: possono contenere caratteri digitati liberamente dall'utente)
    String stationsJson = "[";
    for (size_t i = 0; i < stations.size(); i++) {
        stationsJson += "{\"name\":\"" + jsonEscape(stations[i].name) + "\",\"url\":\"" + jsonEscape(stations[i].url) + "\",\"lang\":\"" + jsonEscape(stations[i].nameLang) + "\"}";
        if (i < stations.size() - 1) stationsJson += ",";
    }
    stationsJson += "]";

    // Utilizziamo un unico blocco HTML statico con stringhe raw per evitare pesi di concatenazione
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");

    // Blocco Head e Stili CSS (Mobile-First)
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
        input[type="text"]:focus, input[type="password"]:focus { outline: none; border-color: #3b82f6; background-color: #ffffff; box-shadow: 0 0 0 3px rgba(59,130,246,0.15); }
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

    // Inietta l'array JSON calcolato dinamicamente dall'ESP32 (già escapato in jsonEscape())
    server.sendContent("\n<script>\nlet stations = " + stationsJson + ";\n");

    // Blocco logica client-side in JS per riempire la lista e gestire il ripopolamento del form per la modifica.
    // Costruzione via DOM (createElement/textContent), non innerHTML: evita che caratteri
    // come < o > digitati dall'utente in nome/url vengano interpretati come markup.
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

    server.sendContent(""); // Chiude l'invio
}

void handleAddStation() {
    if (server.hasArg("name") && server.hasArg("url") && server.hasArg("lang")) {
        String name = server.arg("name");
        String url = server.arg("url");
        String lang = server.arg("lang");
        String editIdStr = server.arg("edit_id");

        if (!editIdStr.isEmpty()) {
            // Se edit_id è popolato stiamo sovrascrivendo una stazione esistente
            int editId = editIdStr.toInt();
            if (editId >= 0 && editId < (int)stations.size()) {
                stations[editId] = {name, url, lang};
                logSuSeriale(F("[CFG] Stazione %d modificata: %s\n"), editId, name.c_str());
            }
        } else {
            // Altrimenti si tratta di un inserimento standard
            stations.push_back({name, url, lang});
            logSuSeriale(F("[CFG] Nuova stazione aggiunta: %s\n"), name.c_str());
        }

        saveStationsToFS(); // Aggiorna LittleFS

        // Redirect automatico alla pagina di configurazione stazioni
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
            logSuSeriale(F("[CFG] Elimino stazione %d: %s\n"), id, stations[id].name.c_str());
            stations.erase(stations.begin() + id);
            saveStationsToFS(); // Aggiorna LittleFS
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