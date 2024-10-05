/* mbed Microcontroller Library
 * Copyright (c) 2024 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */

#include "mbed.h"
#include "../../../KRAI_library/Motor/Motor.h"
#include "../../KRAI_library/encoderKRAI/encoderKRAI.h"
#include "../../KRAI_library/Pinout/BoardManagerV1.h"
#include "../../KRAI_library/MovingAverage/MovingAverage.h"
#include "../../KRAI_library/ADRC_V2/ADRC_V2.h"

#define PPR 537.6

//============================SETUP BASIC TIMER======================================
Ticker ms_tick;
uint32_t millis = 0;
void onMillisecondTicker(void)
{
    // this code will run every millisecond
    millis++;
}

// SETUP TIMER IN US_TICKER_READ
// US = µ Second -> Timer dengan satuan detik 10^6 (µ)
int ms_ticker_read(){

    return us_ticker_read()/1000;

}
//-----------------------------------------------------------------------------------

//=========================SETUP UART SERIAL PRINT===================================
#define SERIAL_TX PB_6
#define SERIAL_RX PB_7
#define SERIAL_BAUDRATE 115200

static BufferedSerial serial_port(SERIAL_TX, SERIAL_RX, SERIAL_BAUDRATE);
FileHandle *mbed::mbed_override_console(int fd) {
    return &serial_port;
}
//-----------------------------------------------------------------------------------

Motor flyKanan(BMV1_PWM_MOTOR_1,BMV1_FOR_MOTOR_1,BMV1_REV_MOTOR_1);
Motor flyKiri(BMV1_PWM_MOTOR_2,BMV1_FOR_MOTOR_2,BMV1_REV_MOTOR_2);

encoderKRAI encKanan(BMV1_ENCODER_1_B, BMV1_ENCODER_1_A, PPR, Encoding::X4_ENCODING);
encoderKRAI encKiri(BMV1_ENCODER_2_B, BMV1_ENCODER_2_A, PPR, Encoding::X4_ENCODING);

MovingAverage movAVGKanan(10);
MovingAverage movAVGKiri(10);

uint32_t Ts = 7;
uint32_t TimeNow = millis;

// Parameter for normal speed 
float b0_KananMotor_normal = 430; 
float b0_KiriMotor_normal = 430; 
float tSettle_normal = 0.5; 
float zESO_normal = 7.25; 
float incrementInt_normal = 0.09;

ADRC_V2 controllerKanan(Ts / 1000.0f, b0_KananMotor_normal, tSettle_normal, zESO_normal, incrementInt_normal);
ADRC_V2 controllerKiri(Ts / 1000.0f, b0_KiriMotor_normal, tSettle_normal, zESO_normal, incrementInt_normal);

int main()
{
    // while (true){
    // printf("test\n");
    // }
    float pulseThen_Kiri = encKiri.getPulses();
    float pulseThen_Kanan = encKanan.getPulses();
    float rotatePerSec_Kiri = 0;
    float rotatePerSec_Kanan = 0;
    float PWMMotorKanan = 0;
    float PWMMotorKiri = 0;
    float targetSpeed = -9;

    ms_tick.attach_us(onMillisecondTicker,1000);
    while (true)
    {
        //printf("test = %d\n", millis - TimeNow);
        if (millis - TimeNow > Ts)       
        {

            rotatePerSec_Kiri = float(encKiri.getPulses() - pulseThen_Kiri )* 1000.0 / (PPR * Ts);
            rotatePerSec_Kanan = float(encKanan.getPulses() - pulseThen_Kanan)* 1000.0 / (PPR * Ts);

            rotatePerSec_Kiri = movAVGKiri.movingAverage(rotatePerSec_Kiri);
            rotatePerSec_Kanan = movAVGKanan.movingAverage(rotatePerSec_Kanan);

            pulseThen_Kiri = encKiri.getPulses();
            pulseThen_Kanan = encKanan.getPulses();

            TimeNow = millis;
            PWMMotorKiri = controllerKiri.createInputSignal(controllerKiri.fhan_setPointTrajectory(-targetSpeed, 75), rotatePerSec_Kiri, 1.0f);
            PWMMotorKanan = controllerKanan.createInputSignal(controllerKanan.fhan_setPointTrajectory(targetSpeed, 100), rotatePerSec_Kanan, 1.0f);
            
            flyKiri.speed(PWMMotorKiri); 
            flyKanan.speed(PWMMotorKanan);

        }
        
        //printf("encKiri = %d encKanan = %d \n",encKiri.getPulses(), encKanan.getPulses());
        printf("rps_KIRI = %f , rps_KANAN = %f \n",rotatePerSec_Kiri ,rotatePerSec_Kanan);
    
    }
    return 0;

}