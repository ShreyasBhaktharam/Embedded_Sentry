#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h" // Access library locally for the LCD !!!
// ==========================================================================
// * Recitation 8: Final Recitation - Intro to DSP + Using the LCD Screen!! *
// ==========================================================================

// TODOs:
// [1] Introduction to Filters for data smoothening!
// [2] LCD Screen - Reading the datasheet and writing the code to display text and shapes on to the Screen!
// [3] Extra --> Screen Examples!

// INTRODUCTION TO DSP --> No extra Libraries used!
// Note: You can only use 1 serial terminal at one time --> either the serial monitor or Teleplot!

// STEP 1: Initializations and Definitions
// Define control register addresses and configurations --> Gyroscope!
// Control Register 1
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01'10'1'1'1'1

// Control Register 4
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b0'0'01'0'00'0

// Control Register 3
#define CTRL_REG3 0x22
#define CTRL_REG3_CONFIG 0b0'0'0'0'1'000

// Output Register --> X axis
#define OUT_X_L 0x28

// Define Flag bits for the EventFlags object
#define SPI_FLAG 1
#define DATA_READY_FLAG 2

// Scaling Factor for data conversion dps --> rps (make sure its the right vale?!)
#define SCALING_FACTOR (17.5f * 0.0174532925199432957692236907684886f / 1000.0f)

// Window Size for Moving Average Window
#define WINDOW_SIZE 10

// Filter Coefficient for lpf and hpf
#define FILTER_COEFFICIENT 0.1f // Adjust this coefficient as needed!

// EventFlags Object Declaration
EventFlags flags;

// Callback function for SPI Transfer Completion
void spi_cb(int event) {
    flags.set(SPI_FLAG);
}

// Callback function for Data Ready Interrupt
void data_cb() {
    flags.set(DATA_READY_FLAG);
}

// Variable Definitions for Filters
    uint16_t raw_gx, raw_gy, raw_gz; // raw gyro values
    float gx, gy, gz; // converted gyro values
    float filtered_gx = 0.0f, filtered_gy = 0.0f, filtered_gz = 0.0f; //lpf filtered values    
    float high_pass_gx = 0.0f, high_pass_gy = 0.0f, high_pass_gz = 0.0f; //hpf filtered values

// Moving Average Filter Buffer --> only to be used with the Moving Average Filter!
float window_gx[WINDOW_SIZE] = {0}, window_gy[WINDOW_SIZE] = {0}, window_gz[WINDOW_SIZE] = {0};
int window_index = 0;

// FIR LPF Coefficients --> only to be used for FIR LPF!
#define FIR_SIZE 5
float fir_coefficients[FIR_SIZE] = {0.2f, 0.2f, 0.2f, 0.2f, 0.2f};
float fir_buffer[FIR_SIZE] = {0};
int fir_buffer_index = 0;

// Functions for FIR filters --> only to be used for FIR LPF!
float fir_filter(float input) {
    fir_buffer[fir_buffer_index] = input;
    float output = 0.0f;
    for (int i = 0; i < FIR_SIZE; i++) {
        output += fir_coefficients[i] * fir_buffer[(fir_buffer_index - i + FIR_SIZE) % FIR_SIZE];
    }
    fir_buffer_index = (fir_buffer_index + 1) % FIR_SIZE;
    return output;
}

int main() {

    //Initialise LED
    DigitalOut led(LED1, 0);
    DigitalOut led1(LED1, 0), led2(LED2, 1);

    //Initialise LCD
    LCD_DISCO_F429ZI lcd;
    const uint8_t spacing = 10;

    
    // STEP 2: SPI and Interrupt Initialization
    
    // SPI
    SPI spi(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    uint8_t write_buf[32], read_buf[32];
    
    // Interrupt
    InterruptIn int2(PA_2, PullDown);
    int2.rise(&data_cb);

    // SPI Data transmission format and frequency
    spi.format(8, 3);
    spi.frequency(1'000'000);

    // STEP 3: GYRO Configuration!
    // 1. Control Register 1 
    write_buf[0] = CTRL_REG1;
    write_buf[1] = CTRL_REG1_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    // 2. Control Register 4
    write_buf[0] = CTRL_REG4;
    write_buf[1] = CTRL_REG4_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    // 3. Control Register 3
    write_buf[0] = CTRL_REG3;
    write_buf[1] = CTRL_REG3_CONFIG;
    spi.transfer(write_buf, 2, read_buf, 2, &spi_cb);
    flags.wait_all(SPI_FLAG);

    // dUMMY BYTE for write_buf[1] --> Placeholder Value!
    // We have to send an address and a value for write operation!
    // We have the address but have to send a placeholder value as well!
    write_buf[1] = 0xFF;


    // There are many reasons why sometimes interrupts don't work: Delays, Missed Edges, Noise, etc..
    // You can implement an additional polling loop for the data ready flag like the one below
    // This is redundant but helps when the interrupt doesn't work!
    // You don't need to specify a timeout value for the interrupt if you're using this polling loop!

    if (!(flags.get() & DATA_READY_FLAG) && (int2.read() == 1)) {
    flags.set(DATA_READY_FLAG);
    }

    // STEP 4: Loop for Data Collection and Filtering!
    while (1) {

        // Wait for data ready flag (timeout 255 ms --> 0xFF)
        // Hover over and please have a look at the function signature of flags.wait_all !! 
        // 0xFF in the Timeout value means the program will wait for 255 ms for the data ready flag
        // true as the third argument --> for clearing the flag after automatically
        // If nothing happens, it will continue execution!
        // This helps with preventing blocking the whole code!
        // If this is used then even polling MIGHT not be necessary! 
        flags.wait_all(DATA_READY_FLAG, 0xFF, true);
        
        // Read GYRO Data using SPI transfer --> 6 bytes!
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(SPI_FLAG);

        // Extract raw 16-bit gyroscope data for X, Y, Z
        raw_gx = (read_buf[2] << 8) | read_buf[1];
        raw_gy = (read_buf[4] << 8) | read_buf[3];
        raw_gz = (read_buf[6] << 8) | read_buf[5];

        // Convert raw data to radians per second!
        gx = raw_gx * SCALING_FACTOR;
        gy = raw_gy * SCALING_FACTOR;
        gz = raw_gz * SCALING_FACTOR;

        // 1. No Filter
        // printf("RAW -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", gx, gy, gz);
        // printf(">RAW X axis-> gx: %4.5f|g\n", gx);
        // printf(">RAW Y axis-> gy: %4.5f|g\n", gy);
        // printf(">RAW Z axis-> gz: %4.5f|g\n", gz);

        // // 2. Moving Average FIR
        window_gx[window_index] = gx;
        window_gy[window_index] = gy;
        window_gz[window_index] = gz;
        float avg_gx = 0.0f, avg_gy = 0.0f, avg_gz = 0.0f;
        for (int i = 0; i < WINDOW_SIZE; i++) {
            avg_gx += window_gx[i];
            avg_gy += window_gy[i];
            avg_gz += window_gz[i];
        }
        avg_gx /= WINDOW_SIZE;
        avg_gy /= WINDOW_SIZE;
        avg_gz /= WINDOW_SIZE;
        window_index = (window_index + 1) % WINDOW_SIZE;
        printf("Moving Average -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", avg_gx, avg_gy, avg_gz);
        printf(">Moving Average X axis-> gx: %4.5f|g\n", avg_gx);
        printf(">Moving Average Y axis-> gy: %4.5f|g\n", avg_gy);
        printf(">Moving Average Z axis-> gz: %4.5f|g\n", avg_gz);


        // // 3. LPF IIR
        filtered_gx = FILTER_COEFFICIENT * gx + (1 - FILTER_COEFFICIENT) * filtered_gx;
        filtered_gy = FILTER_COEFFICIENT * gy + (1 - FILTER_COEFFICIENT) * filtered_gy;
        filtered_gz = FILTER_COEFFICIENT * gz + (1 - FILTER_COEFFICIENT) * filtered_gz;
        printf("LPF IIR -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", filtered_gx, filtered_gy, filtered_gz);
        printf(">LPF IIR X axis-> gx: %4.5f|g\n", filtered_gx);
        printf(">LPF IIR Y axis-> gy: %4.5f|g\n", filtered_gy);
        printf(">LPF IIR Z axis-> gz: %4.5f|g\n", filtered_gz);

        // // 4. LPF FIR
        float fir_gx = fir_filter(gx);
        printf("LPF FIR -> gx: %4.5f\n", fir_gx);

        // // 5. HPF IIR
        // high_pass_gx = gx - filtered_gx;
        // high_pass_gy = gy - filtered_gy;
        // high_pass_gz = gz - filtered_gz;
        // printf("HPF IIR -> gx: %4.5f, gy: %4.5f, gz: %4.5f\n", high_pass_gx, high_pass_gy, high_pass_gz);
        // printf(">HPF IIR -> gx: %4.5f|g\n", high_pass_gx);
        // printf(">HPF IIR -> gy: %4.5f|g\n", high_pass_gy);
        // printf(">HPF IIR -> gz: %4.5f|g\n", high_pass_gz);

        // // 6. HPF FIR
        // float hpf_fir_gx = gx - fir_gx;
        // printf("HPF FIR -> gx: %4.5f\n", hpf_fir_gx);

        //Store the filtered values in the buffer for plotting
        // data_buffer[0] = filtered_gx;
        // data_buffer[1] = filtered_gy;
        // data_buffer[2] = filtered_gz;

        //Compare the filtered values with the threshold
        if (filtered_gx > 6.0f || filtered_gy > 0.5f && filtered_gz > 0.5f) {
            led1 = !led1;
            led2 = !led2;

            lcd.Clear(LCD_COLOR_WHITE);
            lcd.SetTextColor(LCD_COLOR_RED);

            // Smiley face using polygons and triangles
            Point smile[] = {{70, LINE(10)}, {120, LINE(15)}, {170, LINE(10)}};
            lcd.DrawPolygon(smile, 3);
            lcd.FillPolygon(smile, 3);

            // Draw rectangle for the mouth
            lcd.DrawRect(70, LINE(10), 100, 19);
        } else {
            led1 = 1;
            led2 = 0;

            lcd.Clear(LCD_COLOR_BLACK);
            lcd.SetTextColor(LCD_COLOR_RED);
            lcd.DisplayChar(0 + 0 * spacing, 0, 'H');
            lcd.DisplayChar(0 + 1 * spacing, 0, 'e');
            lcd.DisplayChar(0 + 2 * spacing, 0, 'l');
            lcd.DisplayChar(0 + 3 * spacing, 0, 'l');
            lcd.DisplayChar(0 + 4 * spacing, 0, 'o');
        }

        //Detect when board is stationary
        // if(filtered_gx < 0.1f && filtered_gy < 0.1f && filtered_gz < 0.1f) {
        //     led1 = 1;
        //     led2 = 0;

        //     lcd.SetTextColor(LCD_COLOR_RED);
        //     lcd.DisplayChar(0 + 0 * spacing, 0, 'H');
        //     lcd.DisplayChar(0 + 1 * spacing, 0, 'e');
        //     lcd.DisplayChar(0 + 2 * spacing, 0, 'l');
        //     lcd.DisplayChar(0 + 3 * spacing, 0, 'l');
        //     lcd.DisplayChar(0 + 4 * spacing, 0, 'o');
        // }

        thread_sleep_for(100);
    }
}










