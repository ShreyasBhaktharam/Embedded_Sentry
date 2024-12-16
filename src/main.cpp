#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h" 

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

EventFlags flags;
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

int main() {
    uint8_t write_buf[32], read_buf[32];
    int16_t raw_gx, raw_gy, raw_gz;
    int gx, gy, gz;
    int stable_count = 0;
    bool is_locked = false;

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

        if (abs(gx) < STABILITY_THRESHOLD && abs(gy) < STABILITY_THRESHOLD && abs(gz) < STABILITY_THRESHOLD) {
            stable_count++;
            if (stable_count >= STABILITY_COUNT && !is_locked) {
                is_locked = true;
                lcd.Clear(LCD_COLOR_WHITE);
                lcd.SetTextColor(LCD_COLOR_RED);
                lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"LOCKED", CENTER_MODE);
                led1 = 1;
                led2 = 0;
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
            }
        }

        thread_sleep_for(100);
    }
}