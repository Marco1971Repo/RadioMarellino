#include <WiFi.h>
#include <WebServer.h>
#include <Audio.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <RotaryEncoder.h>
#include <ArduinoJson.h>
#include <vector>

// Definizione della struttura semplificata
struct Station {
    String name;
    String url;
};

// Il vettore globale che conterrà le stazioni in RAM
std::vector<Station> stations;
#define DEBUGGAME
// I2S pin
#define I2S_DOUT   12
#define I2S_BCLK   13
#define I2S_LRC    14

// kY-040
#define PIN_DT  42
#define PIN_CLK 41
#define PIN_SW  38

enum MachineStates {
    STATE_INIT,
    STATE_WAITWIFICONNECTION,
    STATE_PLAYER,
    STATE_START_AP,
    STATE_AP_MODE,
    STATE_OTA_START,
    STATE_OTA,
    STATE_UPLOAD,
    STATE_BT_SPEAKER
};

MachineStates currentState = STATE_INIT;
Audio audio;
WebServer server(80);

// Credenziali WiFi caricate da flash
String wifiSsid = "";
String wifiPass = "";

// Gestione timeout connessione (15 secondi)
unsigned long connectionStartTime = 0;
const unsigned long WIFI_TIMEOUT_MS = 15000;

// Prototipi funzioni debug
void logSuSeriale(const __FlashStringHelper *frmt, ...);

RotaryEncoder encoder(PIN_DT, PIN_CLK, RotaryEncoder::LatchMode::TWO03);

#define ROTARYSTEPS 2
#define ROTARYMIN 0
#define ROTARYMAX 64

// Last known rotary position.
int lastPos = -1;

// ── Variabili globali pending play ───────────────────────────────────────────
String pendingUrl    = "";
bool hasPendingPlay  = false;

// ── Gestione File di Configurazione ──────────────────────────────────────────
bool loadStations();
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

// ── WebServer Handlers ───────────────────────────────────────────────────────

void handleRoot() {
    String html = F(
        "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1'>"
        "<style>body{font-family:sans-serif; margin:20px;} input[type=text], input[type=password]{width:100%; padding:12px; margin:8px 0; display:inline-block; border:1px solid #ccc; box-sizing:border-box;} button{background-color:#4CAF50; color:white; padding:14px 20px; margin:8px 0; border:none; width:100%; cursor:pointer;}</style>"
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

        server.send(200, "text/html", F("<!DOCTYPE html><html><body><h3>Dati salvati. Il dispositivo si sta riavviando...</h3></body></html>"));
        delay(1000);
        
        saveWifiConfig(reqSsid, reqPass);
        
        logSuSeriale(F("[AP] Riavvio in corso...\n"));
        ESP.restart();
    } else {
        server.send(400, "text/plain", "Bad Request");
    }
}

// ── Callback Audio (nuova API 3.x) ───────────────────────────────────────────

void my_audio_info(Audio::msg_t m) {
    // Intercetta redirect verso HTTPS e forza HTTP
    if (m.e == Audio::evt_info) {
        String s = String(m.msg);
        if (s.indexOf("redirect to new host") >= 0) {
            if (s.indexOf("https://") >= 0 && s.indexOf("play1.m3u8") >= 0) {
                int start = s.indexOf("https://");
                String url = s.substring(start);
                url.trim();
                url.replace("\"", "");
                url.replace("https://", "http://");
                pendingUrl      = url;
                hasPendingPlay  = true;
                logSuSeriale(F("[REDIRECT] Forzato HTTP: %s\n"), url.c_str());
            }
        }
    }

    switch (m.e) {
        case Audio::evt_info:           logSuSeriale(F("info: ....... %s\n"), m.msg); break;
        case Audio::evt_eof:            logSuSeriale(F("end of file:  %s\n"), m.msg); break;
        case Audio::evt_bitrate:        logSuSeriale(F("bitrate: .... %s\n"), m.msg); break;
        case Audio::evt_icyurl:         logSuSeriale(F("icy URL: .... %s\n"), m.msg); break;
        case Audio::evt_id3data:        logSuSeriale(F("ID3 data: ... %s\n"), m.msg); break;
        case Audio::evt_lasthost:       logSuSeriale(F("last URL: ... %s\n"), m.msg); break;
        case Audio::evt_name:           logSuSeriale(F("station name: %s\n"), m.msg); break;
        case Audio::evt_streamtitle:    logSuSeriale(F("stream title: %s\n"), m.msg); break;
        case Audio::evt_icylogo:        logSuSeriale(F("icy logo: ... %s\n"), m.msg); break;
        case Audio::evt_icydescription: logSuSeriale(F("icy descr: .. %s\n"), m.msg); break;
        case Audio::evt_log:            logSuSeriale(F("audio_logs:   %s\n"), m.msg); break;
        default:                        logSuSeriale(F("message:..... %s\n"), m.msg); break;
    }
}

// ── Setup & Loop ─────────────────────────────────────────────────────────────
//#define CONFIG_FIRST_RUN
void setup() {
#ifdef DEBUGGAME
    Serial.begin(115200);
#endif
    encoder.setPosition(32 / ROTARYSTEPS);
    Audio::audio_info_callback = my_audio_info;
#ifdef CONFIG_FIRST_RUN
    // Blocco di ripristino/creazione forzata del JSON (messo sotto ifdef come concordato)
    if (LittleFS.begin(true)) {
        File f = LittleFS.open("/stations.json", "w");
        if (f) {
            f.print(R"(
                {"stations":[{"name":"Radio Deejay","url":"http://streamcdnb1-4c4b867c89244861ac216426883d1ad0.msvdn.net/radiodeejay/radiodeejay/play1.m3u8"},
                {"name":"Radio Deejay 1","url":"http://4c4b867c89244861ac216426883d1ad0.msvdn.net/radiodeejay/radiodeejay/master_ma.m3u8"},
                {"name":"Virgin Radio","url":"http://icy.unitedradio.it/Virgin.mp3"},
                {"name":"Virgin Rock 80","url":"http://icy.unitedradio.it/VirginRock80.mp3"},
                {"name":"Virgin Rock 90","url":"http://icy.unitedradio.it/Virgin_03.mp3"},
                {"name":"Vr Classic Rock","url":"http://icy.unitedradio.it/VirginRockClassics.mp3"},
                {"name":"Vr Queen","url":"http://icy.unitedradio.it/Virgin_05.mp3"},
                {"name":"Vr AC-DC","url":"http://icy.unitedradio.it/um1026.mp3"},
                {"name":"Controradio","url":"http://streaming.controradio.it:8190/;?type=http&nocache=76494"},
                {"name":"Deejay 80","url":"http://streamcdnf25-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejay80/live.m3u8"},
                {"name":"On The Road","url":"http://streamcdnm5-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejayontheroad/live.m3u8"},
                {"name":"Tropical Pizza","url":"http://streamcdnm12-4c4b867c89244861ac216426883d1ad0.msvdn.net/webradio/deejaytropicalpizza/live.m3u8"},
                {"name":"Mitology","url":"http://onair15.xdevel.com:9120/;stream.mp3"},
                {"name":"RTL 102.5","url":"https://dd782ed59e2a4e86aabf6fc508674b59.msvdn.net/live/S97044836/tbbP8T1ZRPBL/playlist_audio.m3u8"},
                {"name":"Radio 105","url":"http://icecast.unitedradio.it/Radio105.mp3"},{"name":"Subasio","url":"http://icy.unitedradio.it/Subasio.mp3"},
                {"name":"M2O","url":"http://4c4b867c89244861ac216426883d1ad0.msvdn.net/radiom2o/radiom2o/master_ma.m3u8"}]}
                )");
            f.close();
        }
        LittleFS.end();
    }
 #endif
    loadStations();
    // ── Diagnostica memoria ──────────────────────────
    uint32_t psram_size = ESP.getPsramSize();
    if (psram_size > 0) {
        uint32_t free_psram = ESP.getFreePsram();
        logSuSeriale(F("PSRAM OK: %u KB totali, %u KB liberi\n"),
                        psram_size / 1024, free_psram / 1024);
    } else {
        logSuSeriale(F("PSRAM: non rilevata\n"));
    }

    logSuSeriale(F("SRAM: %u KB totali, %u KB liberi\n"), ESP.getHeapSize() / 1024, ESP.getFreeHeap() / 1024);
    logSuSeriale(F("--------------------------------\n"));
    gpio_set_drive_capability((gpio_num_t)13, GPIO_DRIVE_CAP_3);
}

void loop() {
    switch (currentState) {

        case STATE_INIT:
            logSuSeriale(F("[STATE] INIT\n"));
            audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
            audio.setVolumeSteps(64);
            audio.setVolume(16);
            
            
            // Prova a caricare la configurazione da Flash
            if (loadWifiConfig()) {
                logSuSeriale(F("[WiFi] Tentativo di connessione a: %s\n"), wifiSsid.c_str());
                WiFi.mode(WIFI_STA);
                WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());
                connectionStartTime = millis();
                currentState = STATE_WAITWIFICONNECTION;
            } else {
                logSuSeriale(F("[WiFi] Credenziali non trovate o non valide. Avvio AP.\n"));
                currentState = STATE_START_AP;
            }
            break;

        case STATE_WAITWIFICONNECTION:
            if (WiFi.status() != WL_CONNECTED) {
                if (millis() - connectionStartTime > WIFI_TIMEOUT_MS) {
                    logSuSeriale(F("\n[WiFi] Timeout connessione esaurito. Passaggio ad AP.\n"));
                    WiFi.disconnect();
                    currentState = STATE_START_AP;
                } else {
                    delay(500);
                    logSuSeriale(F("."));
                }
            } else {
                logSuSeriale(F("\n[WiFi] Connesso - IP: %s\n"), WiFi.localIP().toString().c_str());
                audio.connecttohost("http://4c4b867c89244861ac216426883d1ad0.msvdn.net/radiodeejay/radiodeejay/master_ma.m3u8");
                currentState = STATE_PLAYER;
            }
            break;

        case STATE_START_AP:
        {
            logSuSeriale(F("[STATE] START AP\n"));
            WiFi.disconnect();
            WiFi.mode(WIFI_AP);
            IPAddress local_IP(192, 168, 1, 1);
            IPAddress gateway(192, 168, 1, 1);
            IPAddress subnet(255, 255, 255, 0);
            
            if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
                logSuSeriale(F("[AP] Configurazione IP statico fallita!\n"));
            }
            WiFi.softAP("RadioMarellino_Setup", ""); 
            
            logSuSeriale(F("[AP] Configurato. SSID: RadioMarellino_Setup\n"));
            logSuSeriale(F("[AP] IP: %s\n"), WiFi.softAPIP().toString().c_str());

            server.on("/", HTTP_GET, handleRoot);
            server.on("/save", HTTP_POST, handleSave);
            server.begin();
            logSuSeriale(F("[HTTP] Server avviato\n"));

            currentState = STATE_AP_MODE;
            break;
        }

        case STATE_AP_MODE:
            server.handleClient();
            delay(2);
            break;

        case STATE_PLAYER:
        {
            audio.loop();
            encoder.tick();

            if (hasPendingPlay) {
                hasPendingPlay = false;
                logSuSeriale(F("[PLAYER] Riconnessione a: %s\n"), pendingUrl.c_str());
                audio.connecttohost(pendingUrl.c_str());
            }

            int newPos = encoder.getPosition() * ROTARYSTEPS;

            if (newPos < ROTARYMIN) {
                encoder.setPosition(ROTARYMIN / ROTARYSTEPS);
                newPos = ROTARYMIN;
            } else if (newPos > ROTARYMAX) {
                encoder.setPosition(ROTARYMAX / ROTARYSTEPS);
                newPos = ROTARYMAX;
            }

            if (lastPos != newPos) {
                lastPos = newPos;
                audio.setVolume(lastPos);
            }

            vTaskDelay(1);
            break;
        }

        default:
            break;
    }
}

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

    // Alloca il documento per il parsing (ArduinoJson v7 gestisce la RAM dinamicamente)
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, f);
    f.close();
    LittleFS.end(); // Chiudiamo il fìlesystem appena abbiamo finito

    if (err) {
        logSuSeriale(F("[ERR] Errore parsing JSON: %s\n"), err.c_str());
        return false;
    }

    // Estrae l'array principale e verifica che sia valido
    JsonArray arr = doc["stations"].as<JsonArray>();
    if (arr.isNull()) {
        logSuSeriale(F("[ERR] Formato JSON non valido (manca l'array 'stations')\n"));
        return false;
    }

    stations.clear();
    
    // Ottimizzazione: riserva lo spazio in memoria per evitare riallocazioni continue nel vettore
    stations.reserve(arr.size()); 

    // Popola il vettore
    for (JsonObject s : arr) {
        stations.push_back({
            s["name"].as<String>(),
            s["url"].as<String>()
        });
    }

    logSuSeriale(F("[CFG] Stazioni caricate con successo: %d\n"), stations.size());
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
