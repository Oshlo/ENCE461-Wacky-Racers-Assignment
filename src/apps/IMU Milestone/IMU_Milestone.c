/*  File: IMU_Milestone.c
    Author: Adam Finlayson, Harrison Pollard
    Date: 11/05/2021
    Desc: Milestone Week 9. Working IMU and blinking LEDs
*/

#include "pit.h"
#include <stdio.h>
#include <pio.h>
#include <fcntl.h>
#include "target.h"
#include "pacer.h"
#include "usb_serial.h"
#include "mpu9250.h"
#include "math.h"
#include "nrf24.h"
#include "adc.h"
#include "pwm.h"

#define ADC_CLOCK_FREQ 24000000
#define PWM_FREQ_HZ 1000

/* Defines how fast the while loop is polled (Hz) */
enum
{
    POLL_RATE = 10
};

/* Defines how fast the LEDs are polled (Hz) */
enum
{
    LED_FLASH_RATE = 2
};

static int16_t motor_speed[2];
static int8_t sent = 0;

/* Defines the USB CDC configuration structure.
     Set = 0 for non-blocking I/O */
static usb_serial_cfg_t usb_serial_cfg =
    {
        .read_timeout_us = 1,
        .write_timeout_us = 1,
}; // Blocking I/O fills up buffers of info so code can still run while transmitting (eg UART)

static const pwm_cfg_t pwm1_cfg =
{
    .pio = PWM1_PIO,
    .period = PWM_PERIOD_DIVISOR (PWM_FREQ_HZ),
    .duty = PWM_DUTY_DIVISOR (PWM_FREQ_HZ, 50),
    .align = PWM_ALIGN_LEFT,
    .polarity = PWM_POLARITY_LOW,
    .stop_state = PIO_OUTPUT_LOW
};

/* Defines the TWI configuration structure. */
static twi_cfg_t mpu_twi_cfg =
    {
        .channel = TWI_CHANNEL_0,             // Sets the controller channel to 0
        .period = TWI_PERIOD_DIVISOR(100000), // 100 kHz Clock
        .slave_addr = 0                       // Not using as slave ?? Set to master (0)
};

static spi_cfg_t nrf_spi = {
    .channel = 0,
    .clock_speed_kHz = 1000,
    .cs = RADIO_CS_PIO,
    .mode = SPI_MODE_0,
    .cs_mode = SPI_CS_MODE_FRAME,
    .bits = 8,
};

static const adc_cfg_t adc_cfg1 =
{
    .bits = 12,
    .channel = ADC_CHANNEL_0,
    .trigger = ADC_TRIGGER_SW,
    .clock_speed_kHz = ADC_CLOCK_FREQ / 1000
};

static const adc_cfg_t adc_cfg2 =
{
    .bits = 12,
    .channels= BIT (ADC_CHANNEL_4) | BIT (ADC_CHANNEL_5),
    .trigger = ADC_TRIGGER_SW,
    .clock_speed_kHz = ADC_CLOCK_FREQ / 1000
};

/* Initialisation function for PIO components used in main */
uint8_t init_pio(void)
{
    // Initialise LED pins as pull up or down
    pio_config_set(LED1_PIO, PIO_OUTPUT_HIGH);
    pio_config_set(LED2_PIO, PIO_OUTPUT_LOW);

    // Initialise Radio Select Channel Pins (INPUT, LOW)
    pio_config_set(RADIO_S1, PIO_PULLUP);
    pio_config_set(RADIO_S2, PIO_PULLUP);
    pio_config_set(RADIO_S3, PIO_PULLUP);
    pio_config_set(RADIO_S4, PIO_PULLUP);

    // Initialise IMU vs Joystick controll
    pio_config_set(IMU_PIO, PIO_PULLDOWN);

    // printf("%d, ", pio_input_get(RADIO_S1));
    // printf("%d, ", pio_input_get(RADIO_S2));
    // printf("%d, ", pio_input_get(RADIO_S3));
    // printf("%d\n", pio_input_get(RADIO_S4));
    printf("%d\n", pio_input_get(IMU_PIO));

#ifdef RADIO_PWR_EN
    pio_config_set(RADIO_PWR_EN, PIO_OUTPUT_HIGH);
#endif

    return 1;
}

/* Initialisation function for USB used in main */
uint8_t init_usb(void)
{
    // Create non-blocking tty device for USB CDC connection.
    usb_serial_init(&usb_serial_cfg, "/dev/usb_tty"); // connects this file (stdin & stdout) to a read/write file

    freopen("/dev/usb_tty", "a", stdout); // freopen() makes printf work (connected to ubs_tty file)
    freopen("/dev/usb_tty", "r", stdin);

    return 1;
}

static void panic(void)
{
    while (1)
    {
        pio_output_set(LED1_PIO, 0);
    }
}

/* A function that prints the board tilt angle to a USB serial COM port */
void get_speed_from_imu(mpu_t *mpu)
{
    if (!mpu9250_is_imu_ready(mpu)) // Check if the IMU is ready
    {
        printf("Waiting for IMU to be ready...\n");
    }
    else
    {
        int16_t accel[3];       // Initialise an array of acceleration variables (x, y, z) to be read from the IMU
        int8_t theta[2];        // Initialise an array of angles

        if (mpu9250_read_accel(mpu, accel)) // Check if IMU data can be read
        {
            theta[0] = atan(((double_t)accel[0]) / ((double_t)accel[2])) * 180 / M_PI; // theta_xz (degrees)
            theta[1] = atan(((double_t)accel[1]) / ((double_t)accel[2])) * 180 / M_PI; // theta_yz (degrees)
            if (theta[0] > 25)
            {
                theta[0] = 25; // Ensure the maximum angle is 25 degrees
            }
            else if (theta[0] < -25)
            {
                theta[0] = -25;
            }
            if (theta[1] > 25)
            {
                theta[1] = 25; // Ensure the maximum angle is 25 degrees
            }
            else if (theta[1] < -25)
            {
                theta[1] = -25;
            }

            motor_speed[0] = (theta[0] * -20) + 250;
            motor_speed[1] = (theta[0] * -20) + 250;

            if (theta[1] >= 0)
            {
                motor_speed[1] = motor_speed[1] - (theta[1] * 10);
            }
            else
            {
                motor_speed[0] = motor_speed[0] + (theta[1] * 10);
            }

            if (accel[2] <= 0) {
                motor_speed[0] = 0;
                motor_speed[1] = 0;
            }
            
        }
        else
        {
            printf("ERROR: failed to read acceleration\n");
        }
    }
}

void get_speed_from_joystick(adc_t adc)
{
    uint16_t joystick_adc[2];
    int16_t mapped[2];

    adc_read (adc, joystick_adc, sizeof (joystick_adc));

    if (joystick_adc[0] < 1950 && joystick_adc[0] > 1850) {
        mapped[0] = 0; 
    } else {  
        mapped[0] = ((joystick_adc[0] - 1900) / 2) ;
    }
    if (joystick_adc[1] < 2130 && joystick_adc[1] > 2020) {
        mapped[1] = 0;
    } else {
        mapped[1] = ((joystick_adc[1] - 250 - 1875) / 2);
    }

    if (mapped[1] >= 0) {
        // Moving forward
        if (mapped[0] > 0) {
            // turning left
            motor_speed[0] = mapped[1] - mapped[0];
            motor_speed[1] = mapped[1];
        } else if (mapped[0] < 0) {
            // turn right
            motor_speed[0] = mapped[1];
            motor_speed[1] = mapped[1] + mapped[0];
        } else if (mapped[0] == 0) {
            motor_speed[0] = mapped[1];
            motor_speed[1] = mapped[1];
        }
    } else if (mapped[1] < 0) {
        // moving backwards
        motor_speed[0] = mapped[1];
        motor_speed[1] = mapped[1];
    }
}

uint8_t blink_led(uint8_t led_count)
{
    led_count++;                                     // Increase the LED counter
    if (led_count >= POLL_RATE / LED_FLASH_RATE * 2) // Check when LEDs should be turned on
    {
        led_count = 0; // Reset Counter

        pio_output_toggle(LED1_PIO); // Toggle LED 1
        pio_output_toggle(LED2_PIO); // Toggle LED 2
    }

    return led_count;
}

void data_tx(nrf24_t *nrf)
{
    char buffer[32];

    pio_output_toggle(LED2_PIO);
    pio_output_set(LED1_PIO, 1);

    sprintf(buffer, "%d,%d\r\n", motor_speed[1], motor_speed[0]);
    
    if (!nrf24_write(nrf, buffer, sizeof(buffer)))
    {
        pio_output_set(LED1_PIO, 0);
        // if (!sent)
        // {
        //     sent = 1;
        //     printf("ERROR: Data not sent.\n");
        // }
    }
    else
    {
        pio_output_set(LED1_PIO, 1);
        // if (sent)
        // {
        //     sent = 0;
        //     printf(buffer);
        // }
    }

}

int channel_select(void)
{
    int channel = 64;

    if (pio_input_get(RADIO_S1)) {
        channel = 120;
    } else if (pio_input_get(RADIO_S2)) {
        channel = 73;
    } else if (pio_input_get(RADIO_S3)) {
        channel = 91;
    } else if (pio_input_get(RADIO_S4)) {
        channel = 26;
    }

    printf ("Radio Channel: %d\n", channel);

    return channel;
}

uint16_t get_battery_voltage(adc_t adc)
{
    uint16_t bat_adc[1];
    adc_read (adc, bat_adc, sizeof (bat_adc));
    return bat_adc[0];
}

int main(void)
{
    mcu_jtag_disable();
    if (init_usb()) {
        printf("Serial Initialised\n");
    } // Initialise the USB
    pacer_init(POLL_RATE); // Initialise pacer poll rate
    if (init_pio()) {
        printf("PIO Initialised\n");
    } // Initialise the PIO

    // Initialise Variables
    int channel = channel_select();
    uint8_t flush_inits = 0;
    uint16_t battery_voltage = 0;

    pwm_t pwm1 = pwm_init (&pwm1_cfg);
    adc_t adc1 = adc_init (&adc_cfg1);
    adc_t adc2 = adc_init (&adc_cfg2);
    twi_t twi_mpu = twi_init(&mpu_twi_cfg);            // Initialise the TWI (I2C) bus for the MPU
    mpu_t *mpu = mpu9250_create(twi_mpu, MPU_ADDRESS); // Initialise the MPU9250 IMU
    spi_t spi = spi_init(&nrf_spi);
    nrf24_t *nrf;
    nrf = nrf24_create(spi, RADIO_CE_PIO, RADIO_IRQ_PIO);
    nrf24_begin(nrf, channel, 0x0548896548, 32);
    pwm_channels_start (pwm_channel_mask (pwm1) );

    printf(".............\n\n");

    while (1)
    {
        pacer_wait(); // Wait for 100ms

        if (pio_input_get(IMU_PIO)) {
            get_speed_from_imu(mpu); // Poll IMU
        } else {
            get_speed_from_joystick(adc2);
        }
 
        data_tx(nrf);

        // led_count = blink_led(led_count);
        battery_voltage = get_battery_voltage(adc1);
        if (battery_voltage < 2220) {

        }
        

        printf("\n");
        fflush(stdout); // Flushes the buffer so print functions are printed in time.
    }
}

/*  Notes:
        -   The IMU is initialised to a signed 16 bit interger, so the range of acceleration values
            are from 32,768 to 32,767.
        -   A value of 16,384 corresponds to 1G or 9.81m/s
        -   The movement of the racer will correspond with the tilt of the board (angle)
        -   The board angle can then be found by pythagorus (thetha = arctan( (x or y) /z ) )
*/