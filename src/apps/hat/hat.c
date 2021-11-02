/*  File: IMU_Milestone.c
    Author: Adam Finlayson, Harrison Pollard
    Date: 11/05/2021
    Desc: Milestone Week 9. Working IMU and blinking LEDs
*/

#include "pit.h"
#include "target.h"
#include "pacer.h"
#include "usb_serial.h"
#include "adc.h"

#define ADC_CLOCK_FREQ 24000000

/* Defines how fast the while loop is polled (Hz) */
enum
{
    POLL_RATE = 10
};

/* Defines the USB CDC configuration structure.
     Set = 0 for non-blocking I/O */
static usb_serial_cfg_t usb_serial_cfg =
    {
        .read_timeout_us = 1,
        .write_timeout_us = 1,
}; // Blocking I/O fills up buffers of info so code can still run while transmitting (eg UART)

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

int main (void)
{

    // Initialise pacer poll rate
    pacer_init(POLL_RATE);

    // Create non-blocking tty device for USB CDC connection.
    usb_serial_init(&usb_serial_cfg, "/dev/usb_tty"); // connects this file (stdin & stdout) to a read/write file

    freopen("/dev/usb_tty", "a", stdout); // freopen() makes printf work (connected to ubs_tty file)
    freopen("/dev/usb_tty", "r", stdin);

    printf("INITIALISED\n");


    // TODO: write hat program here...
    while (1) 
    {
        pacer_wait(); // Wait for 100ms

        fflush(stdout);
    }
}
