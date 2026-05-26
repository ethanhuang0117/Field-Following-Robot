#include <EFM8LB1.h>
#include <stdio.h>
#include "vl53l0x.h"

#define  SYSCLK         72000000L
#define  BAUDRATE       115200L
#define  SMB_FREQUENCY  400000L

#define rightmotor1 P3_3
#define rightmotor2 P3_2
#define leftmotor1  P3_1
#define leftmotor2  P3_0

#define leftsensor   P2_5
#define rightsensor  P2_4
#define intersection P2_3

#define SERVO_PIN P1_1

#define LED_PATH1 P1_4
#define LED_PATH2 P1_3
#define LED_PATH3 P1_2
#define LED_MODE  P1_5
#define LED_RIGHT P1_6
#define LED_LEFT  P1_7

#define TRIG_PIN P0_6
#define ECHO_PIN P0_7

#define IR_receiver P2_0
#define unit_s 1000L

#define speaker P1_0

unsigned char key1, key2;
volatile int offset;
extern bit path_complete;

char _c51_external_startup (void)
{
    SFRPAGE = 0x00;
    WDTCN = 0xDE;
    WDTCN = 0xAD;

    VDM0CN = 0x80;
    RSTSRC = 0x02|0x04;

#if (SYSCLK == 48000000L)
    SFRPAGE = 0x10;
    PFE0CN  = 0x10;
    SFRPAGE = 0x00;
#elif (SYSCLK == 72000000L)
    SFRPAGE = 0x10;
    PFE0CN  = 0x20;
    SFRPAGE = 0x00;
#endif

#if (SYSCLK == 12250000L)
    CLKSEL = 0x10;
    CLKSEL = 0x10;
    while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 24500000L)
    CLKSEL = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 48000000L)
    CLKSEL = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x07;
    CLKSEL = 0x07;
    while ((CLKSEL & 0x80) == 0);
#elif (SYSCLK == 72000000L)
    CLKSEL = 0x00;
    CLKSEL = 0x00;
    while ((CLKSEL & 0x80) == 0);
    CLKSEL = 0x03;
    CLKSEL = 0x03;
    while ((CLKSEL & 0x80) == 0);
#else
    #error SYSCLK must be either 12250000L, 24500000L, 48000000L, or 72000000L
#endif

#if (((SYSCLK/BAUDRATE)/(12L*2L)) > 0x100)
    #error Can not configure baudrate using timer 1
#endif

    SCON0 = 0x10;
    TH1   = 0x100-((SYSCLK/BAUDRATE)/(12L*2L));
    TL1   = TH1;
    TMOD &= ~0xf0;
    TMOD |=  0x20;
    TR1 = 1;
    TI  = 1;

    P0MDOUT |= 0x10;
    P1MDOUT |= 0xFF;
    P0MDOUT |= 0x50;

    XBR0 = 0b_0000_0101;
    XBR1 = 0x00;
    XBR2 = 0x40;

    CKCON0 |= 0b_0000_0100;
    TMOD   &= 0xf0;
    TMOD   |= 0x02;
    TL0 = TH0 = 256-(SYSCLK/SMB_FREQUENCY/3);
    TR0 = 1;

    SMB0CF  = 0b_0101_1100;
    SMB0CF |= 0b_1000_0000;

    SFRPAGE  = 0x10;
    TMR3CN1 |= 0b_0110_0000;
    SFRPAGE  = 0x00;

    return 0;
}

#define CONST_SIZE 4

#define SaveFdata(X,Y) \
{ FLKEY  = 0xA5; \
  FLKEY  = 0xF1; \
  PSCTL  = 0x01; \
  *((unsigned char xdata *) X) = Y; \
  PSCTL  = 0x00; }

#define EraseFdataPage(X) \
{ FLKEY  = 0xA5; \
  FLKEY  = 0xF1; \
  PSCTL  = 0x03; \
  *((unsigned char xdata *) X) = 0; \
  PSCTL  = 0x00; }

#define ReadFdata(X) (*((unsigned char code *) X))
#define BASE_FDATA 0xf800

void Save_Vars (void)
{
    bit saved_EA;
    unsigned int j;
    unsigned int address;
    unsigned char *ptr;

    saved_EA = EA;
    EA = 0;
    EraseFdataPage(BASE_FDATA);

    key1    = 0x55;
    key2    = 0xaa;
    address = BASE_FDATA;
    ptr     = &key1;

    for (j = 0; j < CONST_SIZE; j++)
    {
        SaveFdata(address++, *ptr);
        ptr++;
    }
    EA = saved_EA;
}

void Restore_Vars (void)
{
    unsigned int j;
    unsigned int address;
    unsigned char *ptr;

    if ((ReadFdata(BASE_FDATA) != 0x55) || (ReadFdata(BASE_FDATA+1) != 0xaa))
    {
        offset = 0;
    }
    else
    {
        address = BASE_FDATA;
        ptr     = &key1;
        for (j = 0; j < CONST_SIZE; j++)
        {
            *ptr = ReadFdata(address++);
            ptr++;
        }
    }
}

void Timer3us (unsigned char us)
{
    unsigned char i;
    CKCON0 |= 0b_0100_0000;
    TMR3RL  = (-(SYSCLK)/1000000L);
    TMR3    = TMR3RL;
    TMR3CN0 = 0x04;
    for (i = 0; i < us; i++)
    {
        while (!(TMR3CN0 & 0x80));
        TMR3CN0 &= ~(0x80);
    }
    TMR3CN0 = 0;
}

void waitms (unsigned int ms)
{
    unsigned int j;
    unsigned char k;
    for (j = 0; j < ms; j++)
        for (k = 0; k < 4; k++)
            Timer3us(250);
}

void Wait_SI (void)
{
    unsigned int I2C_t = 5000;
    while ((!SI) && (I2C_t > 0)) I2C_t--;
}

void Wait_STO (void)
{
    unsigned int I2C_t = 5000;
    while ((STO) && (I2C_t > 0)) I2C_t--;
}

void I2C_write (unsigned char output_data)
{
    SMB0DAT = output_data;
    SI = 0;
    Wait_SI();
}

unsigned char I2C_read (bit ack)
{
    ACK = ack;
    SI  = 0;
    Wait_SI();
    return SMB0DAT;
}

void I2C_start (void)
{
    ACK = 0; STO = 0; STA = 1; SI = 0;
    Wait_SI();
}

void I2C_stop (void)
{
    ACK = 0; STA = 0; STO = 1; SI = 0;
    Wait_STO();
    STO = 0;
}

bit i2c_read_addr8_data8 (unsigned char address, unsigned char *value)
{
    I2C_start(); I2C_write(0x52); I2C_write(address); I2C_stop();
    I2C_start(); I2C_write(0x53); *value = I2C_read(1); I2C_stop();
    return 1;
}

bit i2c_read_addr8_data16 (unsigned char address, unsigned int *value)
{
    I2C_start(); I2C_write(0x52); I2C_write(address); I2C_stop();
    I2C_start(); I2C_write(0x53);
    *value  = I2C_read(0) * 256;
    *value += I2C_read(1);
    I2C_stop();
    return 1;
}

bit i2c_write_addr8_data8 (unsigned char address, unsigned char value)
{
    I2C_start(); I2C_write(0x52); I2C_write(address); I2C_write(value); I2C_stop();
    return 1;
}

void validate_I2C_interface (void)
{
    unsigned char val8  = 0;
    unsigned int  val16 = 0;
    printf("\n");
    i2c_read_addr8_data8(0xc0, &val8);   printf("Reg(0xc0): 0x%02x\n", val8);
    i2c_read_addr8_data8(0xc1, &val8);   printf("Reg(0xc1): 0x%02x\n", val8);
    i2c_read_addr8_data8(0xc2, &val8);   printf("Reg(0xc2): 0x%02x\n", val8);
    i2c_read_addr8_data16(0x51, &val16); printf("Reg(0x51): 0x%04x\n", val16);
    i2c_read_addr8_data16(0x61, &val16); printf("Reg(0x61): 0x%04x\n", val16);
    printf("\n");
}

void  InitADC (void);
void  InitPinADC (unsigned char portno, unsigned char pin_num);
float Volts_at_Pin (unsigned char pin);
void  Motors_Stop (void);
void  Motors_Forward (void);
void  Motors_Backward (void);
void  Motors_Left (void);
void  Motors_Right (void);
void  PWM_Steer (unsigned char n_cycles, unsigned char left_duty, unsigned char right_duty);
float absolute (float num);

typedef enum { dir_forward = 0, dir_left, dir_right, dir_stop } direction_var;
typedef enum { path1 = 0, path2, path3 } path_choose;

extern path_choose   selected_path;
extern unsigned char intersection_count;
extern bit           intersection_latched;
extern bit           in_intersection;

direction_var get_next_command (void);
void          execute_intersection_command (direction_var command);


int IR_read (void)
{
    unsigned char result = 0;
    int i, start = 0;

    if (IR_receiver) return 0;
    while (IR_receiver != 0);

    TMOD |= 0x01;
    TL0 = 0; TH0 = 0; TF0 = 0; TR0 = 1;
    while (IR_receiver == 0) {}
    TR0 = 0;
    if (TF0 == 0) return 0;

    while (IR_receiver != 0);
    for (i = 0; i < 8; i++)
    {
        while (IR_receiver == 0);
        start = 0;
        while (IR_receiver != 0) { Timer3us(1); start++; }
        if (start > 1300 && start < 3000) result |= (1 << i);
    }
    while (IR_receiver == 0);
    return result;
}


void servo_move (unsigned int pulse_us, unsigned int duration_ms)
{
    unsigned int i, cycles = duration_ms / 20, remaining;
    for (i = 0; i < cycles; i++)
    {
        SERVO_PIN = 1;
        remaining = pulse_us;
        while (remaining >= 250) { Timer3us(250); remaining -= 250; }
        if (remaining > 0) Timer3us((unsigned char)remaining);
        SERVO_PIN = 0;
        waitms(18);
    }
}


unsigned int HCSR04_read (void)
{
    unsigned int duration = 0;
    unsigned int timeout  = 0;

    TRIG_PIN = 1;
    Timer3us(10);
    TRIG_PIN = 0;

    while (ECHO_PIN == 0)
    {
        Timer3us(1);
        timeout++;
        if (timeout > 38000) return 0;
    }

    while (ECHO_PIN == 1)
    {
        Timer3us(1);
        duration++;
        if (duration > 38000) break;
    }

    return duration * 10 / 58;
}

void arc_right_back (unsigned int ms)
{
    leftmotor1  = 1; leftmotor2  = 0;
    rightmotor1 = 0; rightmotor2 = 0;
    waitms(ms);
    Motors_Stop();
}

void arc_left_back (unsigned int ms)
{
    leftmotor1  = 0; leftmotor2  = 0;
    rightmotor1 = 1; rightmotor2 = 0;
    waitms(ms);
    Motors_Stop();
}


void do_parallel_park (void)
{
    printf("Parking...\n");

    Motors_Stop();
    waitms(300);

    Motors_Backward();
    waitms(400);

    arc_right_back(600);
    waitms(300);

    Motors_Backward();
    waitms(1200);

    arc_left_back(600);
    waitms(300);

    Motors_Forward();
    waitms(400);
    Motors_Stop();

    printf("Parked.\n");
    while (1);
}

void main (void)
{
    unsigned char success;
    int range = 0;

    float v_L, v_R, v_I, diff, duty;
    const float Kp                  = 50.0;
    const float threshold           = 0.3;
    const float line_detect         = 0.8;
    const float intersection_detect = 1.5;

    unsigned int  tof_count         = 0;
    unsigned int  ir_count          = 0;
    unsigned int  hcsr04_count      = 0;
    bit           obstacle_detected = 0;
    int           distance;
    unsigned char cmd               = 0;
    bit           mode_toggled      = 0;

    unsigned int hcsr04_distance = 0;

    bit driving_forward = 0;
    bit parked          = 0;

    #define HCSR04_DRIVE_INTERVAL  200
    #define HCSR04_IDLE_INTERVAL  1000

    LED_PATH1 = 1; LED_PATH2 = 1; LED_PATH3 = 1;
    LED_MODE  = 0;
    LED_RIGHT = 1; LED_LEFT  = 1;
    speaker = 0;

    waitms(1000);
    printf("\x1b[2J\x1b[1;1H");
    printf("\n\nVL53L0x test\n");

    Restore_Vars();
    validate_I2C_interface();

    success = vl53l0x_init();
    if (success) printf("VL53L0x initialization succeeded.\n");
    else         printf("VL53L0x initialization failed.\n");

    InitPinADC(2, 3);
    InitPinADC(2, 4);
    InitPinADC(2, 5);
    InitADC();
    Motors_Stop();
    printf("manual\n");

    while (1)
    {
        hcsr04_count++;
        if ( (driving_forward  && hcsr04_count >= HCSR04_DRIVE_INTERVAL) ||
             (!driving_forward && hcsr04_count >= HCSR04_IDLE_INTERVAL) )
        {
            hcsr04_count    = 0;
            hcsr04_distance = HCSR04_read();
            printf("Sonar: %u mm\n", hcsr04_distance);

            if (driving_forward && hcsr04_distance > 0 && hcsr04_distance <= 50)
            {
                driving_forward   = 0;
                parked            = 1;
                obstacle_detected = 0;
                do_parallel_park();
            }
        }

        if (!parked)
        {
            tof_count++;
            if (tof_count >= 400)
            {
                tof_count = 0;

                success = vl53l0x_read_range_single(&range);
                if (success)
                {
                    distance = range - offset;
                    if (distance < 100) obstacle_detected = 1;
                    else                obstacle_detected = 0;
                }

                if (P3_7 == 0)
                {
                    printf("\nPut a surface 25cm away from the sensor.\n");
                    while (P3_7 == 0);
                    success = vl53l0x_read_range_single(&range);
                    if (success)
                    {
                        offset = range - 250;
                        Save_Vars();
                        printf("Calibration saved to non-volatile memory.\n");
                    }
                    else
                    {
                        printf("Calibration failed.\n");
                    }
                }
            }

            if (obstacle_detected)
            {
            	unsigned int b;
                driving_forward = 0;
                Motors_Stop();
                waitms(200);
                LED_RIGHT = 0; LED_LEFT = 0; waitms(200);
                LED_RIGHT = 1; LED_LEFT = 1; waitms(200);
				for (b = 0; b < 200; b++) 
				{
				    speaker = 1; Timer3us(500);
				    speaker = 0; Timer3us(500);
				}	
                LED_RIGHT = 0; LED_LEFT = 0; waitms(200);
                LED_RIGHT = 1; LED_LEFT = 1;
                for (b = 0; b < 200; b++) 
				{
				    speaker = 1; Timer3us(500);
				    speaker = 0; Timer3us(500);
				}	
                
                servo_move(2500, 700);
                servo_move(1500, 300);
                obstacle_detected = 0;
                waitms(500);
            }
        }

        ir_count++;
        if (ir_count >= 500)
        {
            ir_count = 0;
            if (!IR_receiver)
            {
                cmd = IR_read();
                CKCON0 |= 0b_0000_0100;
                TMOD   &= 0xf0;
                TMOD   |= 0x02;
                TL0 = TH0 = 256-(SYSCLK/SMB_FREQUENCY/3);
                TR0 = 1;
            }
            else
            {
                cmd = 0;
            }
        }


        if (cmd == 'X')
        {
            mode_toggled = !mode_toggled;
            if (mode_toggled == 1)
            {
                LED_MODE             = 1;
                path_complete        = 0;
                intersection_count   = 0;
                intersection_latched = 0;
                in_intersection      = 0;
                driving_forward      = 0;
                printf("automatic\n");
            }
            else
            {
                LED_MODE        = 0;
                driving_forward = 0;
                Motors_Stop();
                printf("manual\n");
            }
            waitms(500);
            cmd = 0;
        }


        else if (mode_toggled == 0)
        {
            if (cmd != 0)
            {
                switch (cmd)
                {
                    case '0':
                        selected_path = path1;
                        LED_PATH1 = 0; LED_PATH2 = 1; LED_PATH3 = 1;
                        printf("path1 selected\n");
                        break;

                    case '5':
                        selected_path = path2;
                        LED_PATH1 = 1; LED_PATH2 = 0; LED_PATH3 = 1;
                        printf("path2 selected\n");
                        break;

                    case '8':
                        selected_path = path3;
                        LED_PATH1 = 1; LED_PATH2 = 1; LED_PATH3 = 0;
                        printf("path3 selected\n");
                        break;

                    case '?':
                        driving_forward = 0;
                        Motors_Stop();
                        break;

                    case '$':
                        driving_forward = 0;
                        Motors_Left();
                        waitms(1100);
                        Motors_Stop();
                        break;

                    case 'F':
                        driving_forward = 1;
                        parked          = 0;
                        Motors_Forward();
                        break;

                    case 'D':
                        driving_forward = 0;
                        Motors_Backward();
                        break;

                    case 'r':
                        driving_forward = 0;
                        Motors_Right();
                        waitms(500);
                        Motors_Stop();
                        break;

                    case 'l':
                        driving_forward = 0;
                        Motors_Left();
                        waitms(500);
                        Motors_Stop();
                        break;

                    case 0:
                        break;

                    default:
                        break;
                }
                cmd = 0;
            }
        }

        else
        {
            if (path_complete)
            {
                Motors_Stop();
            }
            else
            {
                v_L = Volts_at_Pin(QFP32_MUX_P2_5);
                v_R = Volts_at_Pin(QFP32_MUX_P2_4);
                v_I = Volts_at_Pin(QFP32_MUX_P2_3);

                if ((v_I >= intersection_detect) && (intersection_latched == 0))
                {
                    intersection_latched = 1;
                    in_intersection      = 1;
                    execute_intersection_command(get_next_command());
                    intersection_count++;
                }
                else if ((v_I < intersection_detect) && (in_intersection == 0))
                {
                    intersection_latched = 0;
                }

                if (!in_intersection)
                {
                    if (v_L >= line_detect || v_R >= line_detect)
                    {
                        diff = absolute(v_L - v_R);
                        if (diff < threshold)
                        {
                            Motors_Forward();
                        }
                        else if (v_L > v_R)
                        {
                            duty = 100.0 - (diff * Kp);
                            if (duty < 20.0) duty = 20.0;
                            PWM_Steer(10, (unsigned char)duty, 100);
                        }
                        else
                        {
                            duty = 100.0 - (diff * Kp);
                            if (duty < 20.0) duty = 20.0;
                            PWM_Steer(10, 100, (unsigned char)duty);
                        }
                    }
                    else
                    {
                        PWM_Steer(10, 50, 50);
                    }
                }
            }
        }

    }
}