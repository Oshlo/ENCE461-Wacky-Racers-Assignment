/** @file   target.h
    @author Harrison Pollard, Adam Finlayson
    @date   4 May 2021
    @brief
*/
#ifndef TARGET_H
#define TARGET_H

#include "mat91lib.h"

/* This is for the carhat (chart) board configured as a hat!  */

/* System clocks  */
#define F_XTAL 12.00e6
#define MCU_PLL_MUL 16
#define MCU_PLL_DIV 1

#define MCU_USB_DIV 2
/* 192 MHz  */
#define F_PLL (F_XTAL / MCU_PLL_DIV * MCU_PLL_MUL)
/* 96 MHz  */
#define F_CPU (F_PLL / 2)

/* TWI  */
#define TWI_TIMEOUT_US_DEFAULT 10000
#define MPU_ADDRESS 0x68

/* USB  */
//#define USB_VBUS_PIO PA5_PIO
#define USB_CURRENT_MA 500

/* ADC  */
#define ADC_BATTERY PA17_PIO
#define ADC_JOYSTICK_X PB1_PIO
#define ADC_JOYSTICK_Y PB0_PIO

/* IMU  */
#define IMU_INT_PIO PA2_PIO

/* LEDs  */
#define LED1_PIO PB2_PIO
#define LED2_PIO PB3_PIO

/* General  */
#define APPENDAGE_PIO PA1_PIO
#define SERVO_PWM_PIO PA2_PIO

/* Button  */
#define BUTTON_PIO PA2_PIO

/* Radio  */
#define RADIO_CS_PIO PA11_PIO
#define RADIO_CE_PIO PA26_PIO
#define RADIO_IRQ_PIO PA25_PIO

/* Radio Channel Select */
#define RADIO_S1 PA10_PIO
#define RADIO_S2 PA9_PIO
#define RADIO_S3 PA8_PIO
#define RADIO_S4 PA7_PIO

/* IMU or Joystick controll */
#define IMU_PIO PA29_PIO

/* Piezo */
#define PWM1_PIO PA0_PIO


#endif /* TARGET_H  */
