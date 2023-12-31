#ifdef ESPHOME_LOG_LEVEL_NONE
using namespace esphome;
#ifndef ESPHOME_LOG_LEVEL
#define ESPHOME_LOG_LEVEL ESPHOME_LOG_LEVEL_DEBUG
#endif
#endif

#include "esp_log.h"

#ifndef tubUART
#define tubUART 1
#endif

#ifndef BALBOAGL_H
#define BALBOAGL_H

#include <Arduino.h>
#include <ArduinoQueue.h>
#include <CircularBuffer.h>

#include "constants.h"

static const char* BALBOA_TAG = "BalboaGL";

// Perform measurements or read nameplate values on your tub to define the power [kW]
// for each device in order to calculate tub power usage
const float POWER_HEATER = 2.8;
const float POWER_PUMP_CIRCULATION = 0.3;
const float POWER_PUMP1_LOW = 0.31;
const float POWER_PUMP1_HIGH = 1.3;
const float POWER_PUMP2_LOW = 0.3;
const float POWER_PUMP2_HIGH = 0.6;

// Tweak for your tub - would be nice to auto-learn in the future to allow for outside temp etc
const int MINUTES_PER_DEGC = 45;

extern ArduinoQueue<String> sendBuffer;

extern String result;
extern int msgLength;

struct BalboaStatus {
    float power;
    String rawData;
    String rawData2; // TODO: better name
    String rawData3; // TODO: better name
    String rawData7; // TODO: better name
    float targetTemp;
    float temp;
    float tempFromF;
    float timeToTemp;
    u_int8_t tempUnit;
    u_int8_t mode;
    u_int8_t pump1;
    u_int8_t pump2;
    String aux;
    String time;
    boolean heater;
    boolean light;
    String state;
    char lcd[5];
    u_int8_t commandQueue;
};

extern struct BalboaStatus status;

extern void telnetSend(String message);

extern void log(const char *format, ...);

class balboaGL {
    public:
    balboaGL(HardwareSerial* serial, int rtsPin, int panelSelectPin, esp_log_level_t logLevel = ESP_LOG_INFO, int ledPin = 2) {
        
        esp_log_level_set(BALBOA_TAG, logLevel);
        
        this->tub = serial;
        this->RTS_PIN = rtsPin;
        this->PIN_5_PIN = panelSelectPin;
        this->LED_PIN = ledPin;

        this->delayTime = 20;

        pinMode(RTS_PIN, OUTPUT);
        log("Setting pin %u LOW\n", RTS_PIN);
        digitalWrite(RTS_PIN, LOW);
        pinMode(PIN_5_PIN, INPUT);
    }

    void set_delay_time(int delay) {
        this->delayTime = delay;
    }


void buttonPressUp();
void buttonPressDown();
void buttonPressMode();
void buttonPressTime();



void setLight(boolean state);

void setTemp(float temperature);

void setPumpState(u_int8_t pump, u_int8_t stateIndex);

void setMode(u_int8_t targetMode);

boolean readSerial();

void detachPanelInterrupt();
void attachPanelInterrupt();

private:

    int RTS_PIN;
    int PIN_5_PIN;
    int LED_PIN;

    HardwareSerial* tub;

    int delayTime;

    #define BUFFER_SIZE 23
    CircularBuffer<uint8_t, BUFFER_SIZE> Q_in;

    unsigned long lastCmdTime = 0;
    bool commandPending = false;
    byte sendByteBuffer[9] = {0};

String HexString2TimeString(String hexstring);

String HexString2ASCIIString(String hexstring);

byte nibble(char c);

void queueCommand(String command, int count=1);
void dequeueCommand();
void setOption(u_int8_t currentIndex, u_int8_t targetIndex, u_int8_t options, String command);
void hexCharacterStringToBytes(byte* byteArray, const char* hexString);
void setTimeToTemp(double currentTemp);
int waitforGLBytes();
void sendCommand();
void handleMessage(size_t len, uint8_t buf[]);
int getPanelSelectPin();
int getRTSPin();

};

#endif
