// robotcontrol.c:  This program uses the ADC to control motor and steerling PWM

#include <stdio.h>
#include <stdlib.h>
#include <EFM8LB1.h>

void Timer3us(unsigned char us);
void waitms(unsigned int ms);

// ~C51~

#define SYSCLK 72000000L
#define BAUDRATE 115200L
#define SARCLK 18000000L

#define rightmotor1 P3_3
#define rightmotor2 P3_2
#define leftmotor1  P3_1
#define leftmotor2  P3_0

#define leftsensor   P2_5
#define rightsensor  P2_4
#define intersection P2_3

#define LED_RIGHT P1_6
#define LED_LEFT  P1_7
#define speaker P1_0

bit path_complete = 0;

void InitADC(void)
{
    SFRPAGE = 0x00;
    ADEN = 0; // Disable ADC

    ADC0CN1 =
        (0x2 << 6) | // 0x0: 10-bit, 0x1: 12-bit, 0x2: 14-bit
        (0x0 << 3) | // 0x0: No shift.
        (0x0 << 0);  // Accumulate 1 conversion

    ADC0CF0 =
        ((SYSCLK/SARCLK) << 3) | // SAR Clock Divider
        (0x0 << 2);               // SYSCLK

    ADC0CF1 =
        (0    << 7) | // Disable low power mode
        (0x1E << 0);  // Conversion Tracking Time

    ADC0CN0 =
        (0x0 << 7) | // ADEN
        (0x0 << 6) | // IPOEN
        (0x0 << 5) | // ADINT
        (0x0 << 4) | // ADBUSY
        (0x0 << 3) | // ADWINT
        (0x0 << 2) | // ADGN: PGA gain=1
        (0x0 << 0);  // TEMPE: disable temp sensor

    ADC0CF2 =
        (0x0  << 7) | // GNDSL: GND pin
        (0x1  << 5) | // REFSL: VDD pin
        (0x1F << 0);  // ADPWR

    ADC0CN2 =
        (0x0 << 7) | // PACEN
        (0x0 << 0);  // ADCM: ADBUSY

    ADEN = 1; // Enable ADC
}

#define VDD 3.3035 // The measured value of VDD in volts

void InitPinADC(unsigned char portno, unsigned char pin_num)
{
    unsigned char mask = 1 << pin_num;

    SFRPAGE = 0x20;
    switch (portno)
    {
        case 0:
            P0MDIN &= (~mask);
            P0SKIP  |= mask;
            break;
        case 1:
            P1MDIN &= (~mask);
            P1SKIP  |= mask;
            break;
        case 2:
            P2MDIN &= (~mask);
            P2SKIP  |= mask;
            break;
        default:
            break;
    }
    SFRPAGE = 0x00;
}

unsigned int ADC_at_Pin(unsigned char pin)
{
    ADC0MX = pin;
    ADINT  = 0;
    ADBUSY = 1;
    while (!ADINT);
    return (ADC0);
}

float Volts_at_Pin(unsigned char pin)
{
    return ((ADC_at_Pin(pin) * VDD) / 16383.0);
}

typedef enum
{
    dir_forward = 0,
    dir_left,
    dir_right,
    dir_stop,
} direction_var;

typedef enum
{
    path1 = 0,
    path2,
    path3,
} path_choose;

const direction_var path1config[8] =
{
    dir_forward,
    dir_left,
    dir_left,
    dir_forward,
    dir_right,
    dir_left,
    dir_right,
    dir_stop,
};

const direction_var path2config[8] =
{
    dir_left,
    dir_right,
    dir_left,
    dir_right,
    dir_forward,
    dir_forward,
    dir_stop,
    dir_stop,
};

const direction_var path3config[8] =
{
    dir_right,
    dir_forward,
    dir_right,
    dir_left,
    dir_right,
    dir_left,
    dir_forward,
    dir_stop,
};

void Motors_Stop(void)
{
    rightmotor1 = 0; rightmotor2 = 0;
    leftmotor1  = 0; leftmotor2  = 0;
  	LED_LEFT = 1; LED_RIGHT = 1;
}

void Motors_Forward(void)
{
    rightmotor1 = 0; rightmotor2 = 1;
    leftmotor1  = 0; leftmotor2  = 1;
    LED_LEFT = 1; LED_RIGHT = 1;
}

void Motors_Backward(void)
{
    rightmotor1 = 1; rightmotor2 = 0;
    leftmotor1  = 1; leftmotor2  = 0;
    LED_LEFT = 1; LED_RIGHT = 1;
}

void Motors_Left(void)
{
    rightmotor1 = 0; rightmotor2 = 1;
    leftmotor1  = 1; leftmotor2  = 0;
    LED_LEFT = 0; LED_RIGHT = 1;
}

void Motors_Right(void)
{
    rightmotor1 = 1; rightmotor2 = 0;
    leftmotor1  = 0; leftmotor2  = 1;
    LED_RIGHT = 0; LED_LEFT = 1;
}

void PWM_Steer(unsigned char n_cycles, unsigned char left_duty, unsigned char right_duty)
{
    unsigned char i, gap, dead;
    for (i = 0; i < n_cycles; i++)
    {
        leftmotor1  = 0; leftmotor2  = 1;
        rightmotor1 = 0; rightmotor2 = 1; 

        if (left_duty <= right_duty)
        {
            Timer3us(left_duty);
            leftmotor2 = 0;
            gap = right_duty - left_duty;
            if (gap  > 0) Timer3us(gap);
            rightmotor2 = 0;
            dead = 100 - right_duty;
            if (dead > 0) Timer3us(dead);
        }
        else
        {
            Timer3us(right_duty);
            rightmotor2 = 0;
            gap = left_duty - right_duty;
            if (gap  > 0) Timer3us(gap);
            leftmotor2 = 0;
            dead = 100 - left_duty;
            if (dead > 0) Timer3us(dead);
        }
    }
}

float absolute(float num) { return (num < 0.0) ? -num : num; }

path_choose   selected_path;
unsigned char intersection_count  = 0;
bit           intersection_latched = 0;
bit           in_intersection      = 0;

direction_var get_next_command(void)
{
    if (selected_path == path1)
    {
        if (intersection_count < 8) return path1config[intersection_count];
    }
    else if (selected_path == path2)
    {
        if (intersection_count < 8) return path2config[intersection_count];
    }
    else if (selected_path == path3)
    {
        if (intersection_count < 8) return path3config[intersection_count];
    }
    return dir_stop;
}

void execute_intersection_command(direction_var command)
{
    switch (command)
    {
        case dir_forward:
            Motors_Forward();
            waitms(500);
            break;

        case dir_left:
            Motors_Left();
            waitms(400);
            break;

        case dir_right:
            Motors_Right();
            waitms(400);
            break;

        case dir_stop:
        default:
            Motors_Stop();
            path_complete = 1;
            return;
    }

    Motors_Forward();
    waitms(300);

    in_intersection = 0;
}