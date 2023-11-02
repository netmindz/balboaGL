#include "balboaGL.h"

#include <driver/uart.h>

String result = "";
int msgLength = 0;

ArduinoQueue<String> sendBuffer(10);  // TODO: might be better bigger for large temp changes. Would need testing

struct BalboaStatus status;

void balboaGL::queueCommand(String command, int count) {
    log("Sending %s - %u times\n", command.c_str(), count);
    for (int i = 0; i < count; i++) {
        sendBuffer.enqueue(command.c_str());
    }
}

void balboaGL::setOption(int currentIndex, int targetIndex, int options, String command) {
    if (targetIndex > currentIndex) {
        queueCommand(command, (targetIndex - currentIndex));
    } else if (currentIndex != targetIndex) {
        int presses = (options - currentIndex) + targetIndex;
        queueCommand(command, presses);
    }
}

// Triggered when pin5 falling - ie our panel is selected
// clears serial receive buffer
void IRAM_ATTR clearRXBuffer() {
    // clear the rx buffer
    uart_flush(tubUART);
}

void balboaGL::attachPanelInterrupt() {
    attachInterrupt(digitalPinToInterrupt(PIN_5_PIN), clearRXBuffer, FALLING);
}

void balboaGL::detachPanelInterrupt() {
    detachInterrupt(digitalPinToInterrupt(PIN_5_PIN));
}

void balboaGL::setTimeToTemp(double currentTemp) {

    if (status.heater && (currentTemp < status.targetTemp)) {
        double tempDiff = (status.targetTemp - currentTemp);
        float timeToTempValue = (tempDiff * MINUTES_PER_DEGC);
        status.timeToTemp = timeToTempValue;
    } else {
        status.timeToTemp = 0;
    }
}

void balboaGL::handleMessage(size_t len, uint8_t buf[]) {
    if (buf[0] == 0xFA) {
        sendCommand();
    }
    result = "";
    Q_in.clear();
    for (int i = 0; i < len; i++) {
        if (buf[i] < 0x10) {
            result += '0';
        }
        result += String(buf[i], HEX);
        Q_in.push(buf[i]);
    }

    // Set as static, so only allocating memory once, not per method call - as when was global
    static float tubpowerCalc = 0;
    static float tubTemp = -1;
    static String state = "unknown";
    static String lastRaw = "";
    static String lastRaw2 = "";
    static String lastRaw3 = "";
    static String lastRaw4 = "";
    static String lastRaw5 = "";
    static String lastRaw6 = "";
    static String lastRaw7 = "";
    static String timeString = "";

    //      log("message = ");
    //      log(result);

    if (result.substring(0, 4) == "fa14") {
        // log("FA 14");
        // telnetSend(result);

        // fa1433343043 = header + 340C = 34.0C

        // If messages is temp or ---- for temp, it is status message
        if (result.substring(10, 12) == "43" || result.substring(10, 12) == "2d") {
            tubpowerCalc = 0;
            String pump = result.substring(13, 14);

            if (pump == "0") {  // Pump 1 Off - Pump 2 Off - 0b0000
                status.pump1 = 0;
                status.pump2 = 0;
            } else if (pump == "1") {  // Pump 1 Low - Pump 2 Off - 0b0001
                status.pump1 = 1;
                status.pump2 = 0;
                tubpowerCalc += POWER_PUMP1_LOW;
            } else if (pump == "2") {  // Pump 1 High - Pump 2 Off - 0b0010
                status.pump1 = PUMP1_STATE_HIGH;
                status.pump2 = 0;
                tubpowerCalc += POWER_PUMP1_HIGH;
            } else if (pump == "4") {  // Pump 1 Off - Pump 2 Low - 0b0100
                status.pump1 = 0;
                status.pump2 = 1;
                tubpowerCalc += POWER_PUMP2_LOW;
            } else if (pump == "5") {  // Pump 1 Low - Pump 2 Low - 0b0101
                status.pump1 = 1;
                status.pump2 = 1;
                tubpowerCalc += POWER_PUMP1_LOW;
                tubpowerCalc += POWER_PUMP2_LOW;
            } else if (pump == "6") {  // Pump 1 High - Pump 2 Low - 0b0110
                status.pump1 = PUMP1_STATE_HIGH;
                status.pump2 = 1;
                tubpowerCalc += POWER_PUMP1_HIGH;
                tubpowerCalc += POWER_PUMP2_LOW;
            } else if (pump == "8") {  // Pump 1 Off - Pump 2 High - 0b1000
                status.pump1 = 0;
                status.pump2 = PUMP2_STATE_HIGH;
                tubpowerCalc += POWER_PUMP2_HIGH;
            } else if (pump == "9") {  // Pump 1 Low - Pump 2 High - 0b1001
                status.pump1 = 1;
                status.pump2 = PUMP2_STATE_HIGH;
                tubpowerCalc += POWER_PUMP1_LOW;
                tubpowerCalc += POWER_PUMP2_HIGH;
            } else if (pump == "a") {  // Pump 1 High - Pump 2 HIGH = 0b1010
                status.pump1 = PUMP1_STATE_HIGH;
                status.pump2 = PUMP2_STATE_HIGH;
                tubpowerCalc += POWER_PUMP1_HIGH;
                tubpowerCalc += POWER_PUMP2_HIGH;
            }

            String aux = result.substring(12,13);
            status.aux = aux; // TODO: put raw value into status till values known

            String heater = result.substring(14, 15);
            if (heater == "0") {
                status.heater = false;
            } else if (heater == "1") {
                status.heater = true;
                tubpowerCalc += POWER_HEATER;
            } else if (heater == "2") {
                status.heater = true;  // heater off, verifying temp change, but creates noisy state if we return false
                tubpowerCalc += POWER_HEATER;
            }

            status.power = tubpowerCalc;

            String light = result.substring(15, 16);
            if (light == "0") {
                status.light = false;
            } else if (light == "3") {
                status.light = true;
            }

            /* LCD DISPLAY (2,3,4,5) */
            sprintf(status.lcd, "%c%c%c%c", Q_in[2], Q_in[3], Q_in[4], Q_in[5]);

            // Ignore last 2 bytes as possibly checksum, given we have temp earlier making look more complex than
            // perhaps it is
            String newRaw = result.substring(17, 44);
            if (lastRaw != newRaw) {
                lastRaw = newRaw;
                status.rawData = lastRaw.c_str();

                String s = result.substring(17, 18);
                if (s == "4") {
                    state = "Sleep";
                    status.mode = MODE_IDX_SLP;
                } else if (s == "9") {
                    state = "Circulation ?";
                    status.mode = MODE_IDX_STD;  // TODO: confirm
                } else if (s == "1") {
                    state = "Standard";
                    status.mode = MODE_IDX_STD;
                } else if (s == "2") {
                    state = "Economy";
                    status.mode = MODE_IDX_ECO;
                } else if (s == "a") {
                    state = "Cleaning";  // TODO: can't tell our actual mode here - could be any of the 3 I think
                } else if (s == "c") {
                    state = "Circulation in sleep?";
                    status.mode = MODE_IDX_SLP;
                } else if (s == "b" || s == "3") {
                    state = "Std in Eco";  // Was in eco, Swap to STD for 1 hour only
                    status.mode = MODE_IDX_STD;
                } else {
                    state = "Unknown " + s;
                }

                String menu = result.substring(18, 20);
                if (menu == "00") {
                    // idle
                } else if (menu == "4c") {
                    state = "Set Mode";
                } else if (menu == "5a") {
                    state = "Standby?";  // WT: not tested to confirm if this is the act of setting Standby or just seen
                                         // when in standby
                } else {                 // 46 for set temp, but also other things like filter time
                    state = "menu " + menu;
                }

                // 94600008002ffffff0200000000f5

                if (result.substring(28, 32) != "ffff") {
                    timeString = HexString2TimeString(result.substring(28, 32));
                } else {
                    timeString = "--:--";
                }
                status.time = timeString.c_str();

                // temp up - ff0100000000?? - end varies

                // temp down - ff0200000000?? - end varies

                String cmd = result.substring(34, 44);
                if (cmd == "0000000000") {
                    // none
                } else if (cmd.substring(0, 4) == "01") {
                    state = "Temp Up";
                } else if (cmd.substring(0, 4) == "02") {
                    state = "Temp Down";
                } else {
                    telnetSend("CMD: " + cmd);
                }
                if (!lastRaw3.equals(cmd)) {
                    if(commandPending) {
                        // Controller responded to command
                        sendBuffer.dequeue();
                        commandPending = false;
                        log("YAY: command response : %u\n", delayTime);
                        delayTime = 0;
                    }
                }

                if (!lastRaw3.equals(cmd) && cmd != "0000000000") {  // ignore idle command
                    lastRaw3 = cmd;
                    status.rawData3 = lastRaw3.c_str();
                }


                // Check for degrees C (0x43) or F (0x46), then we have a temperature
                if (Q_in[5] == 0x43 || Q_in[5] == 0x46) {
                    status.tempUnit = Q_in[5];
                    char temp[4] = {char(Q_in[2]), char(Q_in[3]), char(Q_in[4])};

                    //When display showing C or F, and menu mode 0x46 we're in temperature setpoint adjustment
                    if(Q_in[9] == 0x46){
                        status.targetTemp = atoi(temp)/10.0;
                        log("Sent target temp data %f\n", status.targetTemp);
                    }else{
                        status.temp = atoi(temp)/10.0;
                        if (tubTemp != status.temp) {
                            tubTemp = status.temp;
                            log("Sent temp data %f\n", tubTemp);
                        }
                        setTimeToTemp(status.temp);
                    }
                }
                else if(Q_in[5] == 0x2D) {
                    // Showing '-'
                }

                /* TEMP IN F (16) */
                if(menu == "00") {
                    if(status.tempUnit == 0x43){
                        // Unit is Celsius, doing conversion
                        status.tempFromF = (Q_in[16]-32)*.55556;
                        setTimeToTemp(status.tempFromF);
                    }else if(status.tempUnit == 0x46){
                        // Unit is Fahrenheit, no conversion needed.
                        status.tempFromF = Q_in[16];
                        setTimeToTemp(status.tempFromF);
                    }
                }

                status.state = state.c_str();
            }
        } else {
            // FA but not temp data
            lastRaw2 = result.substring(4, 28);
            status.rawData2 = lastRaw2.c_str();
        }

        if (result.length() >= 64) {  // "Long" messages only
            String tail = result.substring(46, 64);
            if (tail != lastRaw7) {
                lastRaw7 = tail;
                status.rawData7 = lastRaw7.c_str();
            }
        }

        // end of FA14
    } else if (result.substring(0, 2) == "fb") {
        if (result != lastRaw7) {
            lastRaw7 = result;
            status.rawData7 = lastRaw7.c_str();
        }
    } else if (result.substring(0, 4) == "ae0d") {
        // log("AE 0D");
        // telnetSend(result);

        String message = result.substring(0, 32);  // ignore any FB ending

        if (result.substring(0, 6) == "ae0d01" && message != "ae0d010000000000000000000000005a") {
            if (!lastRaw4.equals(message)) {
                lastRaw4 = message;
                // status.rawData4 = lastRaw4.c_str();
            }
        } else if (result.substring(0, 6) == "ae0d02" && message != "ae0d02000000000000000000000000c3") {
            if (!lastRaw5.equals(message)) {
                lastRaw5 = message;
                // status.rawData5 = lastRaw5.c_str();
            }
        } else if (result.substring(0, 6) == "ae0d03" && message != "ae0d03000000000000000000000000b4") {
            if (!lastRaw6.equals(message)) {
                lastRaw6 = message;
                // status.rawData6 = lastRaw6.c_str();
            }
        }
        // end of AE 0D
    } else {
        log("Unknown message (%u): ", result.length());
        // log(result);
        telnetSend("U: " + result);
    }
}

void balboaGL::sendCommand() {
    if (sendBuffer.isEmpty()) {
        return;
    }
    if((millis() - lastCmdTime) >= 500) {
        lastCmdTime = millis();
        commandPending = true;
        digitalWrite(RTS_PIN, HIGH);
        digitalWrite(LED_PIN, HIGH);

        delayMicroseconds(delayTime);
        byte byteArray[9] = {0};
        String cmd = sendBuffer.getHead();
        hexCharacterStringToBytes(byteArray, cmd.c_str());
        tub->write(byteArray, sizeof(byteArray));
        if (digitalRead(PIN_5_PIN) != LOW) {
            log("ERROR: Pin5 went high before command before flush : %u\n", delayTime);
            delayTime = 0;
            // sendBuffer.dequeue();
        }
        // wait for tx to finish and flush the rx buffer
        tub->flush(false);
        if (digitalRead(PIN_5_PIN) == LOW) {
            // sendBuffer.dequeue(); // TODO: trying to resend now till we see response
            log("Sent %s with delay of %u\n", cmd.c_str(), delayTime);
            // delayTime += 10;
        }
        else {
           log("ERROR: Pin5 went high before command could be sent after flush");
        }
        digitalWrite(RTS_PIN, LOW);
        digitalWrite(LED_PIN, LOW);
    }
    status.commandQueue = sendBuffer.itemCount();
}

String balboaGL::HexString2TimeString(String hexstring) {
    // Convert "HHMM" in HEX to "HH:MM" with decimal representation
    String time = "";
    int hour = strtol(hexstring.substring(0, 2).c_str(), NULL, 16);
    int minute = strtol(hexstring.substring(2, 4).c_str(), NULL, 16);

    if (hour < 10) time.concat("0");  // Add leading zero
    time.concat(hour);
    time.concat(":");
    if (minute < 10) time.concat("0");  // Add leading zero
    time.concat(minute);

    return time;
}

String balboaGL::HexString2ASCIIString(String hexstring) {
    String temp = "", sub = "", result;
    char buf[3];
    for (int i = 0; i < hexstring.length(); i += 2) {
        sub = hexstring.substring(i, i + 2);
        sub.toCharArray(buf, 3);
        char b = (char)strtol(buf, 0, 16);
        if (b == '\0') break;
        temp += b;
    }
    return temp;
}

byte balboaGL::nibble(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;  // Not a valid hexadecimal character
}

void balboaGL::hexCharacterStringToBytes(byte* byteArray, const char* hexString) {
    bool oddLength = strlen(hexString) & 1;

    byte currentByte = 0;
    byte byteIndex = 0;

    for (byte charIndex = 0; charIndex < strlen(hexString); charIndex++) {
        bool oddCharIndex = charIndex & 1;

        if (oddLength) {
            // If the length is odd
            if (oddCharIndex) {
                // odd characters go in high nibble
                currentByte = nibble(hexString[charIndex]) << 4;
            } else {
                // Even characters go into low nibble
                currentByte |= nibble(hexString[charIndex]);
                byteArray[byteIndex++] = currentByte;
                currentByte = 0;
            }
        } else {
            // If the length is even
            if (!oddCharIndex) {
                // Odd characters go into the high nibble
                currentByte = nibble(hexString[charIndex]) << 4;
            } else {
                // Odd characters go into low nibble
                currentByte |= nibble(hexString[charIndex]);
                byteArray[byteIndex++] = currentByte;
                currentByte = 0;
            }
        }
    }
}

int balboaGL::getPanelSelectPin() {
    return PIN_5_PIN;
}

int balboaGL::getRTSPin() {
    return RTS_PIN;
}

const unsigned long readBytesTimeout = 2500; // Timeout in microseconds (2.5 ms)

/*
 * waitforGLBytes checks the first byte in the serial buffer
 * since we've cleared the serial buffer when pin5 went low
 * there will only be data meant for our panel in the buffer.
 * Depending on the message type, wait for the expected number
 * of bytes to be available with a timeout of 2ms
 * returns number of bytes to read
 */
int balboaGL::waitforGLBytes() {
    int msgLength = 0;
    // define message length from starting Byte
    switch (tub->peek()) {
        case 0xFA:
            msgLength = 23;
            break;
        case 0xFB:
            msgLength = 9;
            break;
        case 0xAE:
            msgLength = 16;
            break;
        default:
            ESP_LOGW(BALBOA_TAG, "Unknown message start Byte: ");
            int unknownByte = tub->read();
            //ESP_LOGW(BALBOA_TAG, unknownByte, HEX);
            return 0;
    }
    // we'll wait here for up to 2.5ms until the expected number of bytes are available
    unsigned long startTime = micros();
    while (tub->available() < msgLength) {
        if (micros() - startTime >= readBytesTimeout) {
            ESP_LOGW(BALBOA_TAG, "Timeout: %u bytes not available in 2.5ms\n", msgLength);
            return 0;
        }
    }
    return msgLength;
}

size_t balboaGL::readSerial() {
    bool panelSelect = digitalRead(PIN_5_PIN);  // LOW when we are meant to read data
    // is data available and we are selected
    if ((tub->available() > 0) && (panelSelect == LOW)) {
        int msgLength = waitforGLBytes();
        // only do something if we've got a message
        if (msgLength > 0) {
            uint8_t buf[msgLength];
            tub->read(buf, msgLength);
            handleMessage(msgLength, buf);
        }
        return msgLength;
    }
    else {
        status.commandQueue = sendBuffer.itemCount();
        return 0;
    }
}

void balboaGL::setLight(boolean state) {
    if (state != status.light) {
        log("setLight - Toggle");
        sendBuffer.enqueue(COMMAND_LIGHT);
    } else {
        log("setLight - No change needed");
    }
}

void balboaGL::setTemp(float temperature) {

    if (status.targetTemp <= 0) {
        log("ERROR: can't adjust target as current value not known");
        sendBuffer.enqueue(COMMAND_UP);  // Enter set temp mode - won't change, but should allow us to capture the set target value
        return;
    }

    int target = temperature * 2;  // 0.5 inc so double
    int current = status.targetTemp * 2;
    sendBuffer.enqueue(COMMAND_UP);  // Enter set temp mode
    sendBuffer.enqueue(COMMAND_EMPTY);

    if (temperature > status.targetTemp) {
        for (int i = 0; i < (target - current); i++) {
            log("Raise the temp");
            sendBuffer.enqueue(COMMAND_UP);
            // sendBuffer.enqueue(COMMAND_EMPTY);
        }
    } else {
        for (int i = 0; i < (current - target); i++) {
            log("Lower the temp");
            sendBuffer.enqueue(COMMAND_DOWN);
            // sendBuffer.enqueue(COMMAND_EMPTY);
        }
    }
}

void balboaGL::buttonPressUp() {
    queueCommand(COMMAND_UP, 1);
}
void balboaGL::buttonPressDown() {
    queueCommand(COMMAND_DOWN, 1);

}
void balboaGL::buttonPressMode() {
    queueCommand(COMMAND_CHANGE_MODE, 1);
}
