// Team Members:
// Shreyas Bhaktharam (sb9855)
// Monish Raman Vishakraman (mv2734)
// Rohan Raju Dhengale (rd3668)

#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h" 
#include "FlashIAP.h"

#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00010000
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b00001000
#define OUT_X_L 0x28
#define SPI_FLAG 1
#define DATA_READY_FLAG 2
#define STABILITY_THRESHOLD 500
#define STABILITY_COUNT 20
#define MOVEMENT_THRESHOLD 1000
#define MOVEMENT_DURATION 5

// Movement detection states
enum Movement {
    NONE,
    UP,
    DOWN,
    LEFT,
    RIGHT,
    Z_ANTICLOCKWISE,
    Z_CLOCKWISE
};

enum ProgramState {
    NORMAL,
    RECORDING,
    VERIFYING
};

#define VERIFICATION_TIMEOUT 10000  // 10 seconds in milliseconds
#define MATCH_DISPLAY_TIME 2000  // 2 seconds in milliseconds
#define MAX_SEQUENCE_LENGTH 10

#define FLASH_PAGE_SIZE 2048  // STM32F429 sector size
#define SEQUENCE_STORAGE_ADDRESS 0x08060000 

struct SequenceData {
    uint8_t movements[MAX_SEQUENCE_LENGTH];
    uint8_t length;
    uint8_t isValid;
};

FlashIAP flash;
EventFlags flags;
ProgramState currentState = NORMAL;
Timer verificationTimer;

void spi_cb(int event) {
    flags.set(SPI_FLAG);
}
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

DigitalOut led1(LED1, 0), led2(LED2, 1);
LCD_DISCO_F429ZI lcd;
SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
InterruptIn int2(PA_2, PullDown);


bool isRecording = false;

class FlashStorage {
public:
    FlashStorage() {
        flash.init();
    }

    ~FlashStorage() {
        flash.deinit();
    }

    bool saveSequence(const SequenceData& data) {
        int sector_size = flash.get_sector_size(SEQUENCE_STORAGE_ADDRESS);
        
        if (flash.erase(SEQUENCE_STORAGE_ADDRESS, sector_size) != 0) {
            return false;
        }

        if (flash.program(&data, SEQUENCE_STORAGE_ADDRESS, sizeof(SequenceData)) != 0) {
            return false;
        }

        return true;
    }

    bool loadSequence(SequenceData& data) {
        memcpy(&data, (void*)SEQUENCE_STORAGE_ADDRESS, sizeof(SequenceData));
        return data.isValid == 1;
    }
};

SequenceData currentSequence;
FlashStorage flashStorage;

Movement detect_movement(int gx, int gy, int gz) {
    if (abs(gx) > abs(gy)) {  // Stronger movement in X axis
        if (gx > MOVEMENT_THRESHOLD)
            return RIGHT;
        else if (gx < -MOVEMENT_THRESHOLD)
            return LEFT;
    }
    else if (abs(gy) > abs(gx)) {  // Stronger movement in Y axis
        if (gy > MOVEMENT_THRESHOLD)
            return UP;
        else if (gy < -MOVEMENT_THRESHOLD)
            return DOWN;
    }

    if (gz > MOVEMENT_THRESHOLD)
        return Z_ANTICLOCKWISE;
    else if (gz < -MOVEMENT_THRESHOLD)
        return Z_CLOCKWISE;
    
    return NONE;
}

void display_movement(Movement mov) {
    lcd.SetTextColor(LCD_COLOR_BLUE);
    switch(mov) {
        case UP:
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(3, LINE(7), (uint8_t *)"CLOCKWISE", CENTER_MODE);
            break;

        case DOWN:
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"ANTI CLOCKWISE", CENTER_MODE);
            break;

        case LEFT:
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"MOVING DOWN", CENTER_MODE);
            break;

        case RIGHT:
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"MOVING UP", CENTER_MODE);
            break;

        case Z_ANTICLOCKWISE:
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Z ANTICLOCKWISE", CENTER_MODE);
            break;

        case Z_CLOCKWISE:
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"Z CLOCKWISE", CENTER_MODE);
            break;

        default:
            lcd.Clear(LCD_COLOR_WHITE);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(7), (uint8_t *)"NO MOVEMENT", CENTER_MODE);
            break;

    }
}

void start_recording() {
    isRecording = true;
    currentSequence.length = 0;
    lcd.SetTextColor(LCD_COLOR_RED);
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"RECORDING", CENTER_MODE);
}

void stop_recording() {
    isRecording = false;
    currentSequence.isValid = 1;
    flashStorage.saveSequence(currentSequence);
    lcd.SetTextColor(LCD_COLOR_GREEN);
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"SAVED", CENTER_MODE);
    thread_sleep_for(1000);
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"      ", CENTER_MODE);
}

void display_stored_sequence() {
    SequenceData stored;
    if (flashStorage.loadSequence(stored)) {
        lcd.Clear(LCD_COLOR_BLACK);
        lcd.SetTextColor(LCD_COLOR_YELLOW);
        lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Stored Sequence:", CENTER_MODE);
        
        for (int i = 0; i < stored.length; i++) {
            char movement[20];
            switch(stored.movements[i]) {
                case UP:
                    sprintf(movement, "UP");
                    break;
                case DOWN:
                    sprintf(movement, "DOWN");
                    break;
                case LEFT:
                    sprintf(movement, "LEFT");
                    break;
                case RIGHT:
                    sprintf(movement, "RIGHT");
                    break;
            }
            lcd.DisplayStringAt(0, LINE(3 + i), (uint8_t *)movement, CENTER_MODE);
        }
        thread_sleep_for(3000);
        lcd.Clear(LCD_COLOR_BLACK);
    }
}

class SequenceVerifier {
private:
    SequenceData storedSequence;
    uint8_t currentIndex;
    Timer timeoutTimer;
    bool verificationStarted;

public:
    SequenceVerifier() : currentIndex(0), verificationStarted(false) {}

    void startVerification() {
        if (flashStorage.loadSequence(storedSequence)) {
            currentIndex = 0;
            verificationStarted = true;
            timeoutTimer.start();
            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_YELLOW);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"VERIFY SEQUENCE", CENTER_MODE);
            displayNextExpectedMove();
        } else {
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"NO SEQUENCE STORED", CENTER_MODE);
            thread_sleep_for(2000);
            lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"                ", CENTER_MODE);
        }
    }

    void displayNextExpectedMove() {
        if (currentIndex < storedSequence.length) {
            lcd.SetTextColor(LCD_COLOR_CYAN);
            char message[30];
            sprintf(message, "Next Move: %s", getMovementName(storedSequence.movements[currentIndex]));
            lcd.DisplayStringAt(0, LINE(2), (uint8_t *)message, CENTER_MODE);
            
            // Display progress
            sprintf(message, "Progress: %d/%d", currentIndex + 1, storedSequence.length);
            lcd.DisplayStringAt(0, LINE(3), (uint8_t *)message, CENTER_MODE);
        }
    }

    const char* getMovementName(uint8_t movement) {
        switch(movement) {
            case UP: return "UP";
            case DOWN: return "DOWN";
            case LEFT: return "LEFT";
            case RIGHT: return "RIGHT";
            default: return "NONE";
        }
    }

    bool checkMovement(Movement detected) {
        if (!verificationStarted || currentIndex >= storedSequence.length) {
            return false;
        }

        if (timeoutTimer.read_ms() > VERIFICATION_TIMEOUT) {
            endVerification(false);
            return false;
        }

        if (detected != NONE && detected == storedSequence.movements[currentIndex]) {
            currentIndex++;
            timeoutTimer.reset();
            
            if (currentIndex >= storedSequence.length) {
                endVerification(true);
            } else {
                displayNextExpectedMove();
            }
            return true;
        }
        else if (detected != NONE && detected != storedSequence.movements[currentIndex]) {
            endVerification(false);
            return false;
        }

        return false;
    }

    void endVerification(bool success) {
        verificationStarted = false;
        timeoutTimer.stop();
        lcd.Clear(LCD_COLOR_BLACK);
        
        if (success) {
            lcd.SetTextColor(LCD_COLOR_GREEN);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"SEQUENCE MATCHED!", CENTER_MODE);
        } else {
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"SEQUENCE FAILED!", CENTER_MODE);
        }
        
        thread_sleep_for(MATCH_DISPLAY_TIME);
        lcd.Clear(LCD_COLOR_BLACK);
        currentState = NORMAL;
    }

    bool isVerifying() {
        return verificationStarted;
    }
};

SequenceVerifier sequenceVerifier;

int main() {
    uint8_t write_buf[32], read_buf[32];
    int16_t raw_gx, raw_gy, raw_gz;
    int gx, gy, gz;
    int stable_count = 0;
    bool is_locked = false;
    Movement current_movement = NONE;
    Movement last_movement = NONE;
    int movement_count = 0;

    int2.rise(&data_cb);
    spi.format(8, 3);
    spi.frequency(1000000);

    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    while (1) {
        flags.wait_all(DATA_READY_FLAG, 0xFF, true);
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        raw_gx = (read_buf[2] << 8) | read_buf[1];
        raw_gy = (read_buf[4] << 8) | read_buf[3];
        raw_gz = (read_buf[6] << 8) | read_buf[5];

        gx = raw_gx;
        gy = raw_gy;
        gz = raw_gz;

        printf("X: %d, Y: %d, Z: %d\n", gx, gy, gz);

if (abs(gx) < STABILITY_THRESHOLD && abs(gy) < STABILITY_THRESHOLD && 
            abs(gz) < STABILITY_THRESHOLD) {
            stable_count++;
            if (stable_count >= STABILITY_COUNT) {
                if (!is_locked) {
                    is_locked = true;
                    if (currentState == RECORDING) {
                        stop_recording();
                        // After recording, automatically start verification
                        currentState = VERIFYING;
                        sequenceVerifier.startVerification();
                    }
                    lcd.Clear(LCD_COLOR_WHITE);
                    lcd.SetTextColor(LCD_COLOR_RED);
                    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"LOCKED", CENTER_MODE);
                    led1 = 1;
                    led2 = 0;
                }
            }
        } else {
            stable_count = 0;
            if (is_locked) {
                is_locked = false;
                lcd.Clear(LCD_COLOR_BLACK);
                lcd.SetTextColor(LCD_COLOR_GREEN);
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"UNLOCKED", CENTER_MODE);
                led1 = 0;
                led2 = 1;
                
                if (currentState == NORMAL) {
                    currentState = RECORDING;
                    start_recording();
                }
            }

            if (!is_locked) {
                Movement detected = detect_movement(gx, gy, gz);
                
                if (detected == last_movement && detected != NONE) {
                    movement_count++;
                    if (movement_count >= MOVEMENT_DURATION && detected != current_movement) {
                        current_movement = detected;
                        display_movement(current_movement);
                        
                        switch(currentState) {
                            case RECORDING:
                                if (currentSequence.length < MAX_SEQUENCE_LENGTH) {
                                    currentSequence.movements[currentSequence.length++] = current_movement;
                                }
                                break;
                                
                            case VERIFYING:
                                sequenceVerifier.checkMovement(current_movement);
                                break;
                                
                            default:
                                break;
                        }
                    }
                } else {
                    movement_count = 0;
                    last_movement = detected;
                    if (detected == NONE && current_movement != NONE) {
                        current_movement = NONE;
                        display_movement(NONE);
                    }
                }
            }
        }

        thread_sleep_for(100);
    }
}