#include <WiFi.h>
#include <WebServer.h>
#include <Audio.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <RotaryEncoder.h>
#define DEBUGGAME
// I2S pin
#define I2S_DOUT   12
#define I2S_BCLK   13
#define I2S_LRC    14
//#define I2S_MCLK   19

// kY-040
#define PIN_DT  44
#define PIN_CLK 41
#define PIN_SW  43
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

// A pointer to the dynamic created rotary encoder instance.
// This will be done in setup()
RotaryEncoder *encoder = nullptr;

/**
 * @brief The interrupt service routine will be called on any change of one of the input signals.
 */
IRAM_ATTR void checkPosition()
{
  encoder->tick(); // just call tick() to check the state.
}


// Prototipi funzioni debug
void logSuSeriale(const __FlashStringHelper *frmt, ...);

// ── Gestione File di Configurazione ──────────────────────────────────────────

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

// ── Setup & Loop ─────────────────────────────────────────────────────────────

void setup() {
#ifdef DEBUGGAME
    Serial.begin(115200);
#endif
  // use TWO03 mode when PIN_IN1, PIN_IN2 signals are both LOW or HIGH in latch position.
    encoder = new RotaryEncoder(PIN_DT, PIN_CLK, RotaryEncoder::LatchMode::TWO03);

    // register interrupt routine
    attachInterrupt(digitalPinToInterrupt(PIN_CLK), checkPosition, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_DT), checkPosition, CHANGE);
    // ── Diagnostica memoria ──────────────────────────
    uint32_t psram_size = ESP.getPsramSize();
    if (psram_size > 0) {
        uint32_t free_psram = ESP.getFreePsram();
        logSuSeriale(F("PSRAM OK: %u KB totali, %u KB liberi\n"),
                        psram_size / 1024, free_psram / 1024);
    } else {
        logSuSeriale(F("PSRAM: non rilevata\n"));
    }

    logSuSeriale(F("SRAM: %u KB totali, %u KB liberi\n"),  ESP.getHeapSize() / 1024, ESP.getFreeHeap() / 1024);
    logSuSeriale(F("--------------------------------\n"));
}

void loop() {
    switch (currentState) {

        case STATE_INIT:
            logSuSeriale(F("[STATE] INIT\n"));
            audio.setPinout(I2S_BCLK, I2S_LRC, I2S_DOUT);
            audio.setVolume(32);            
            
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
                // Controlla se il timeout è scaduto
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
            // Configurazione IP Statico per l'Access Point
            IPAddress local_IP(192, 168, 1, 1);
            IPAddress gateway(192, 168, 1, 1);
            IPAddress subnet(255, 255, 255, 0);
            
            if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
                logSuSeriale(F("[AP] Configurazione IP statico fallita!\n"));
            }            
            // Configura un IP statico per l'AP (opzionale, ma consigliato per stabilità)
            WiFi.softAP("RadioMarellino_Setup", ""); 
            
            logSuSeriale(F("[AP] Configurato. SSID: ESP32_Radio_Setup\n"));
            logSuSeriale(F("[AP] IP: %s\n"), WiFi.softAPIP().toString().c_str());

            // Configura i path del WebServer
            server.on("/", HTTP_GET, handleRoot);
            server.on("/save", HTTP_POST, handleSave);
            server.begin();
            logSuSeriale(F("[HTTP] Server avviato\n"));

            currentState = STATE_AP_MODE;
            break;
        }
        case STATE_AP_MODE:
            server.handleClient();
            delay(2); // Cede il controllo al background task dell'ESP32
            break;

        case STATE_PLAYER:
            audio.loop();   
            break;                        
        
        default:
            break;            
    }
}
// ─────────────────────────────────────────────────────────────────────────────
// Callback Audio
// ─────────────────────────────────────────────────────────────────────────────
void audio_info(const char* info) {
    logSuSeriale(F("[AUDIO] %s\n"), info);
}

void audio_showstation(const char* info) {
    logSuSeriale(F("[STATION] %s\n"), info);
    
}

void audio_showstreamtitle(const char* info) {
    logSuSeriale(F("[TITLE] %s\n"), info);
    
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