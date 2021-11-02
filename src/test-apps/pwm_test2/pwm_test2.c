/* File:   pwm_test2.c
   Author: M. P. Hayes, UCECE
   Date:   15 April 2013
   Descr:  This example starts two channels simultaneously; one inverted
           with respect to the other.
*/
#include "pwm.h"
#include "pio.h"
#include "mcu.h"


#define PWM1_PIO PA0_PIO
#define PWM2_PIO PB5_PIO

#define PWM_FREQ_HZ 1000


static const pwm_cfg_t pwm1_cfg =
{
    .pio = PWM1_PIO,
    .period = PWM_PERIOD_DIVISOR (PWM_FREQ_HZ),
    .duty = PWM_DUTY_DIVISOR (PWM_FREQ_HZ, 50),
    .align = PWM_ALIGN_LEFT,
    .polarity = PWM_POLARITY_LOW,
    .stop_state = PIO_OUTPUT_LOW
};

// static const pwm_cfg_t pwm2_cfg =
// {
//     .pio = PWM2_PIO,
//     .period = PWM_PERIOD_DIVISOR (PWM_FREQ_HZ),
//     .duty = PWM_DUTY_DIVISOR (PWM_FREQ_HZ, 50),
//     .align = PWM_ALIGN_LEFT,
//     .polarity = PWM_POLARITY_HIGH,
//     .stop_state = PIO_OUTPUT_LOW
// };


int
main (void)
{
    mcu_jtag_disable();

    pwm_t pwm1;
    // pwm_t pwm2;

    pwm1 = pwm_init (&pwm1_cfg);
    // pwm2 = pwm_init (&pwm2_cfg);

    pwm_channels_start (pwm_channel_mask (pwm1) ); // | pwm_channel_mask (pwm2)

    while (1)
        continue;
    
    return 0;
}
