#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "soc/rtc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "driver/adc.h"
#include <math.h>
#define N_MOT 4

#define M_AVDX 0
#define M_DIDX 1
#define M_AVSX 2
#define M_DISX 3

#define PIN_ENB_AVDX GPIO_NUM_26
#define PIN_ENB_DIDX GPIO_NUM_2
#define PIN_ENB_AVSX GPIO_NUM_22
#define PIN_ENB_DISX GPIO_NUM_18

#define STCPpin GPIO_NUM_16
#define SHCPpin GPIO_NUM_17
#define DSpin GPIO_NUM_4

#define CSEpin GPIO_NUM_13

#define Freq_pwm 500//500
#define kp 150.0
#define ki 2.0
#define kd 7.0    //7.0
#define limI 0.15 //0.15
#define limP 200
#define limD 500 //500

static mcpwm_dev_t *MCPWM[2] = {&MCPWM0, &MCPWM1};
#define CAP0_INT_EN BIT(27) //Capture 0 interrupt bit
#define CAP1_INT_EN BIT(28) //Capture 1 interrupt bit
#define CAP2_INT_EN BIT(29) //Capture 2 interrupt bit

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

static xQueueHandle val_mot[4] = {NULL, NULL, NULL, NULL};
static xQueueHandle q_giri_voluti = NULL;
static xQueueHandle q_distanza = NULL;
static xQueueHandle r_distanza = NULL;
static xQueueHandle q_diametro = NULL;
static xQueueHandle q_avvio = NULL;

static volatile uint32_t old_mot1, old_mot2, old_mot3, old_mot0;
static volatile uint32_t att_mot1, att_mot2, att_mot3, att_mot0;
static volatile uint8_t sta_mot1, sta_mot2, sta_mot3, sta_mot0;

double ABS(double x) { return (x < 0) ? -x : x; }

#define NOP() asm volatile ("nop")

typedef struct
{ //passa_voluti
    double giri_voluti[N_MOT];
} passa_voluti;

typedef struct
{ //def_str_setMot
    uint8_t UNIT;
    uint8_t CAP;
    uint8_t NA;
    uint8_t TIMER;
    uint8_t SELECT_CAP;
    uint8_t POS;
} def_str_setMot;

typedef struct
{ //def_str_Mot
    uint8_t PIN_DR0;
    uint8_t PIN_DR1;
    uint8_t PIN_PWM;
    uint8_t PIN_ENA;
    uint8_t PIN_ENB;
    uint8_t E_PID;
    uint8_t E_MIS;
    int8_t COR_MIS;
    def_str_setMot M_SET;
} def_str_Mot;

typedef struct
{ //def_mot
    def_str_Mot M[N_MOT];
} def_mot;

typedef struct
{ //capture
    uint32_t valore;
    uint8_t stato;
} capture;

typedef struct
{ //passa_shift
    uint8_t valore;
    uint8_t mask;
} passa_shift;

void set_mot(def_mot *ptr)
{
    ptr->M[M_AVDX].E_MIS = 1; //abilita la misurazione
    ptr->M[M_DIDX].E_MIS = 1;
    ptr->M[M_AVSX].E_MIS = 1;
    ptr->M[M_DISX].E_MIS = 1;

    ptr->M[M_AVDX].E_PID = 1;  //abilita il PID
    ptr->M[M_DIDX].E_PID = 1;
    ptr->M[M_AVSX].E_PID = 1;
    ptr->M[M_DISX].E_PID = 1;

    ptr->M[M_AVDX].COR_MIS = 1;  //corregge la misurazione (1 o -1)
    ptr->M[M_DIDX].COR_MIS = -1;
    ptr->M[M_AVSX].COR_MIS = -1;
    ptr->M[M_DISX].COR_MIS = -1;

    ptr->M[M_AVDX].PIN_DR0 = 0;
    ptr->M[M_AVDX].PIN_DR1 = 1;
    ptr->M[M_AVDX].PIN_ENA = GPIO_NUM_25;
    ptr->M[M_AVDX].PIN_ENB = PIN_ENB_AVDX;
    ptr->M[M_AVDX].PIN_PWM = GPIO_NUM_27;

    ptr->M[M_DIDX].PIN_DR0 = 3;
    ptr->M[M_DIDX].PIN_DR1 = 2;
    ptr->M[M_DIDX].PIN_ENA = GPIO_NUM_15;
    ptr->M[M_DIDX].PIN_ENB = PIN_ENB_DIDX;
    ptr->M[M_DIDX].PIN_PWM = GPIO_NUM_14;

    ptr->M[M_AVSX].PIN_DR0 = 5;
    ptr->M[M_AVSX].PIN_DR1 = 4;
    ptr->M[M_AVSX].PIN_ENA = GPIO_NUM_21;
    ptr->M[M_AVSX].PIN_ENB = PIN_ENB_AVSX;
    ptr->M[M_AVSX].PIN_PWM = GPIO_NUM_23;

    ptr->M[M_DISX].PIN_DR0 = 6;
    ptr->M[M_DISX].PIN_DR1 = 7;
    ptr->M[M_DISX].PIN_ENA = GPIO_NUM_5;
    ptr->M[M_DISX].PIN_ENB = PIN_ENB_DISX;
    ptr->M[M_DISX].PIN_PWM = GPIO_NUM_19;

    ptr->M[M_AVDX].M_SET.UNIT = MCPWM_UNIT_0;
    ptr->M[M_DIDX].M_SET.UNIT = MCPWM_UNIT_0;
    ptr->M[M_AVSX].M_SET.UNIT = MCPWM_UNIT_1;
    ptr->M[M_DISX].M_SET.UNIT = MCPWM_UNIT_1;

    ptr->M[M_AVDX].M_SET.TIMER = MCPWM_TIMER_0;
    ptr->M[M_DIDX].M_SET.TIMER = MCPWM_TIMER_1;
    ptr->M[M_AVSX].M_SET.TIMER = MCPWM_TIMER_0;
    ptr->M[M_DISX].M_SET.TIMER = MCPWM_TIMER_1;

    ptr->M[M_AVDX].M_SET.CAP = MCPWM_CAP_0;
    ptr->M[M_DIDX].M_SET.CAP = MCPWM_CAP_1;
    ptr->M[M_AVSX].M_SET.CAP = MCPWM_CAP_0;
    ptr->M[M_DISX].M_SET.CAP = MCPWM_CAP_1;

    ptr->M[M_AVDX].M_SET.NA = MCPWM0A;
    ptr->M[M_DIDX].M_SET.NA = MCPWM1A;
    ptr->M[M_AVSX].M_SET.NA = MCPWM0A;
    ptr->M[M_DISX].M_SET.NA = MCPWM1A;

    ptr->M[M_AVDX].M_SET.POS = MCPWM_POS_EDGE;
    ptr->M[M_DIDX].M_SET.POS = MCPWM_POS_EDGE;
    ptr->M[M_AVSX].M_SET.POS = MCPWM_POS_EDGE;
    ptr->M[M_DISX].M_SET.POS = MCPWM_POS_EDGE;

    ptr->M[M_AVDX].M_SET.SELECT_CAP = MCPWM_SELECT_CAP0;
    ptr->M[M_DIDX].M_SET.SELECT_CAP = MCPWM_SELECT_CAP1;
    ptr->M[M_AVSX].M_SET.SELECT_CAP = MCPWM_SELECT_CAP0;
    ptr->M[M_DISX].M_SET.SELECT_CAP = MCPWM_SELECT_CAP1;
}

static void IRAM_ATTR ISR_CAP_UNIT0(void *a)
{
    uint32_t mcpwm_intr_status = MCPWM[MCPWM_UNIT_0]->int_st.val;  //ricevo informazioni di interruzione
    if (mcpwm_intr_status & CAP0_INT_EN) //Verifico se è stato causato dal CAP0
    {
        sta_mot0 = gpio_get_level(PIN_ENB_AVDX); //Rilevo livello logico seconda fase
        att_mot0 = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP0); //Ricavo tempo in tick della precisa avvenuta dell'impulso 
        capture evt_mot0; //creo variabile struttura per trasferimento dai
        evt_mot0.stato = sta_mot0; //indico di quale motore è la misura
        if (att_mot0 > old_mot0) //se il nuovo valore è maggiore di quello precendente (la differnza è positiva)
        {
            evt_mot0.valore = att_mot0 - old_mot0; //calcolo differenza tra due impulsi in tick 
            xQueueOverwriteFromISR(val_mot[0], &evt_mot0, NULL); //invio informazioni sulla queue apposita
        }
        old_mot0 = att_mot0; //aggiorno valore precenete con quello attuale
        MCPWM[MCPWM_UNIT_0]->int_ena.cap0_int_ena = 1; //riattivo interrupt
    }
    if (mcpwm_intr_status & CAP1_INT_EN)
    {
        sta_mot1 = gpio_get_level(PIN_ENB_DIDX);
        att_mot1 = mcpwm_capture_signal_get_value(MCPWM_UNIT_0, MCPWM_SELECT_CAP1);
        capture evt_mot1;
        evt_mot1.stato = sta_mot1;
        if (att_mot1 > old_mot1)
        {
            evt_mot1.valore = att_mot1 - old_mot1;
            xQueueOverwriteFromISR(val_mot[1], &evt_mot1, NULL);
        }
        old_mot1 = att_mot1;
        MCPWM[MCPWM_UNIT_0]->int_ena.cap1_int_ena = 1;
    }
    MCPWM[MCPWM_UNIT_0]->int_clr.val = mcpwm_intr_status;
}

static void IRAM_ATTR ISR_CAP_UNIT1(void *b)
{
    uint32_t mcpwm_intr_status = MCPWM[MCPWM_UNIT_1]->int_st.val;
    if (mcpwm_intr_status & CAP0_INT_EN)
    {
        sta_mot2 = gpio_get_level(PIN_ENB_AVSX);
        att_mot2 = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP0);
        capture evt_mot2;
        evt_mot2.stato = sta_mot2;
        if (att_mot2 > old_mot2)
        {
            evt_mot2.valore = att_mot2 - old_mot2;
            //xQueueSendFromISR(val_mot[2], &evt_mot2, NULL);
            xQueueOverwriteFromISR(val_mot[2], &evt_mot2, NULL);
        }
        old_mot2 = att_mot2;
        MCPWM[MCPWM_UNIT_1]->int_ena.cap0_int_ena = 1;
    }
    if (mcpwm_intr_status & CAP1_INT_EN)
    {
        sta_mot3 = gpio_get_level(PIN_ENB_DISX);
        att_mot3 = mcpwm_capture_signal_get_value(MCPWM_UNIT_1, MCPWM_SELECT_CAP1);
        capture evt_mot3;
        evt_mot3.stato = sta_mot3;
        if (att_mot3 > old_mot3)
        {
            evt_mot3.valore = att_mot3 - old_mot3;
            xQueueOverwriteFromISR(val_mot[3], &evt_mot3, NULL);
        }
        old_mot3 = att_mot3;
        MCPWM[MCPWM_UNIT_1]->int_ena.cap1_int_ena = 1;
    }
    MCPWM[MCPWM_UNIT_1]->int_clr.val = mcpwm_intr_status;
}

void set_shift(uint8_t BYTE)
{
    gpio_set_level(STCPpin, 0);
    for (int i = 0; i < 8; i++)
    {
        gpio_set_level(SHCPpin, 0);
        gpio_set_level(DSpin, (BYTE >> i) & 1);
        gpio_set_level(SHCPpin, 1);
    }
    gpio_set_level(STCPpin, 1);
    gpio_set_level(STCPpin, 0);
    gpio_set_level(SHCPpin, 0);
}

static void serialRasp(void *arg)
{
    uint8_t in_byte[6], err = 0, D1 = 0;
    int  in_mot[4];
    double Distanza = 0;
    uint16_t I1, I2;
    uint8_t I0;
    double diam;
    int in_pair[2];

    uart_flush(UART_NUM_1);
    uint8_t pair_type =0 ;
    uint8_t inv_adc = 0 ,inv_dist = 0;
    uint8_t set[8], mod, ricevi,somma;

    while (1)
    {

        //uart_read_bytes(UART_NUM_1, &in_byte[0], 1, portMAX_DELAY);
        if(uart_read_bytes(UART_NUM_1, &in_byte[0], 1, 1000 / portTICK_RATE_MS)<=0)
        {
            //printf("reset x tempo\n");
            passa_voluti voluti;
            voluti.giri_voluti[0] = 0;
            voluti.giri_voluti[1] = 0;
            voluti.giri_voluti[2] = 0;
            voluti.giri_voluti[3] = 0;
            xQueueSend(q_giri_voluti, &voluti, 0);
        }
        else
        {
            //printf("%d "BYTE_TO_BINARY_PATTERN,in_byte[0],BYTE_TO_BINARY(ikn_byte[0]));
           // printf("\n");
            if(( in_byte[0] & 0b10000000)!=0 ){
                //printf("ok1\n");
                for (int i =0; i<8;i++)
                {
                    set[i]= (in_byte[0]>>i) & 1;
                }

                if(set[6]==0 && set[5]==0 && set[2]==0 && set[1]==0)
                { //TUTTI UGUALI
                    mod = 1;
                    ricevi = 2;
                }else if(set[6]==0 && set[5]==0)
                { //COPPIA
                    mod = 2;
                    ricevi = 3;
                }else if(set[6]==0 && set[5]==1)
                { //TUTTI DIVERSI
                    mod = 3;
                    ricevi = 6;
                }else if(set[6]==1 && set[5]==0)
                { //DIAMETRO
                    mod = 4;
                    ricevi = 1;
                }else if(set[6]==1 && set[5]==1)
                { //RICHIESTE
                    mod = 5;
                    ricevi = 0;
                }else
                {
                    mod = 0;
                    ricevi = 0;
                }

                err = 0;

                for ( int i=0;i<(ricevi+1);i++){
                    if (uart_read_bytes(UART_NUM_1, &in_byte[i+1], 1, 20 / portTICK_RATE_MS) <= 0)
                    {
                        err = 1;
                        break;
                    }
                    if((in_byte[i+1] & 0b10000000)!=0 ){
                        err = 1;
                        break;
                    }
                }
                //printf("err %d, mod%d  \n",err,mod);
                if(err==0 && mod!=0){
                    //printf("ok3\n");
                    somma = 0;
                    for ( int i=0;i<(ricevi+1);i++){
                        for (int a=0; a<8;a++){
                            somma +=(in_byte[i]>>a)%2;
                        }                
                    }

                    if(somma==in_byte[ricevi+1]){
                        inv_adc = set[4];
                        inv_dist = set[3];

                        if(inv_adc!=0 && inv_dist!=0){
                            I1 = adc1_get_raw(ADC1_CHANNEL_4);
                            xQueueReceive(q_distanza, &Distanza, 0);
                            I2 = (uint16_t)((int)(Distanza * 10) + 32768);
                            somma = 0;
                            for (int a=0; a<16;a++) somma +=(I1>>a)%2; 
                            for (int a=0; a<16;a++) somma +=(I2>>a)%2;

                            uart_write_bytes(UART_NUM_1, (const char *)&I1, 2);
                            uart_write_bytes(UART_NUM_1, (const char *)&I2, 2);
                            uart_write_bytes(UART_NUM_1, (const char *)&somma, 1);
                        }else if(inv_adc!=0){
                            I1 = adc1_get_raw(ADC1_CHANNEL_4);
                            somma=0;
                            for (int a=0; a<16;a++) somma +=(I1>>a)%2;
                            uart_write_bytes(UART_NUM_1, (const char *)&I1, 2);
                            uart_write_bytes(UART_NUM_1, (const char *)&somma, 1);
                        }else if(inv_dist!=0){
                            xQueueReceive(q_distanza, &Distanza, 0);
                            I1 = (uint16_t)((int)(Distanza * 10) + 32768);
                            somma=0;
                            for (int a=0; a<16;a++) somma +=(I1>>a)%2;
                            uart_write_bytes(UART_NUM_1, (const char *)&I1, 2);
                            uart_write_bytes(UART_NUM_1, (const char *)&somma, 1);
                        }else{
                            I0 = somma; //'O'+'k'
                            uart_write_bytes(UART_NUM_1, (const char *)&I0, 1);
                        }

                        //printf("ok4, s:%d\n",somma);


                        if(mod==1){
                            in_mot[0] = (in_byte[1] << 7) | in_byte[2];
                            passa_voluti vol;
                            for (int i = 0; i < 4; i++)
                            {
                                vol.giri_voluti[i] = (((double)in_mot[0]) * (4.0 / 16384.0)) - 2;
                            }
                            xQueueSend(q_giri_voluti, &vol, 0);
                            //printf("mod 1 %f \n", vol.giri_voluti[0]);
                            printf("ok giri mod1\n");
                        }
                        if(mod==2){
                            in_pair[0] =  (in_byte[1] << 3) | (in_byte[2]>>4);
                            in_pair[1] =  ((in_byte[2] & 0b1111) << 6) | (in_byte[3]>>1);

                            pair_type = (in_byte[0] & 0b110)>>1;

                            if(pair_type == 1){
                                in_mot[0] = in_pair[0];
                                in_mot[1] = in_pair[0];
                                in_mot[2] = in_pair[1];
                                in_mot[3] = in_pair[1];
                            }
                            if(pair_type == 2){
                                in_mot[0] = in_pair[0];
                                in_mot[1] = in_pair[1];
                                in_mot[2] = in_pair[0];
                                in_mot[3] = in_pair[1];
                            }
                            if(pair_type == 3){
                                in_mot[0] = in_pair[0];
                                in_mot[1] = in_pair[1];
                                in_mot[2] = in_pair[1];
                                in_mot[3] = in_pair[0];
                            }
                        
                            passa_voluti vol;
                            for (int i = 0; i < 4; i++)
                            {
                                vol.giri_voluti[i] = (((double)in_mot[i]) * (4.0 / 1024.0)) - 2;
                            }
                            xQueueSend(q_giri_voluti, &vol, 0);
                            printf("ok giri mod2\n");
                            //printf("mod 2 %f %f \n", vol.giri_voluti[0],vol.giri_voluti[2] );
                        }
                        if(mod==3){
                            in_mot[0] = (in_byte[1]<<3) | (in_byte[2]>>4);
                            in_mot[1] = ((in_byte[2] & 0b111)<<7 ) | in_byte[3];
                            in_mot[2] = (in_byte[4]<<3) | (in_byte[5]>>4);
                            in_mot[3] = ((in_byte[5] & 0b111)<<7 ) | in_byte[6];
                            passa_voluti vol;
                            for (int i = 0; i < 4; i++)
                            {
                                vol.giri_voluti[i] = (((double)in_mot[i]) * (4.0 / 1024.0)) - 2;
                            }
                            xQueueSend(q_giri_voluti, &vol, 0);
                            printf("ok giri mod3\n");
                            //printf("mod 3 %f %f %f %f \n", vol.giri_voluti[0],vol.giri_voluti[1],vol.giri_voluti[2],vol.giri_voluti[3] );
                        }
                        if(mod==4){
                            diam = ((double)in_byte[1] *(20.0/128.0))+55;
                            xQueueSend(q_diametro, &diam, 0);
                            printf("diam %f \n", diam);
                        }
                        if(mod==5 && inv_dist==0 && inv_adc==0){
                            printf("ok reset dist\n");
                            Distanza = 0;
                            xQueueOverwrite(r_distanza, &D1);
                            xQueueOverwrite(q_distanza, &Distanza);
                        }
                    }else{
                        printf("NO4");
                    }
                            
                }else{
                    printf("NO3\n");
                }
                



                                
            }


        }
        
    }
    
    
}

void init_serial()
{
    uart_config_t uart_config = {
        .baud_rate = 1000000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN, //UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1, //UART_STOP_BITS_1
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, 12, 0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, (1024) * 2, 0, 0, NULL, 0);
    xTaskCreate(serialRasp, "serialRASP", 8192, NULL, 3, NULL);
    q_distanza = xQueueCreate(1, sizeof(double));
    r_distanza = xQueueCreate(1, sizeof(uint8_t));
    q_diametro = xQueueCreate(1, sizeof(double));
    
}

void init_gpio()
{
    def_mot mot;
    set_mot(&mot);
    uint64_t mask2 = 0, mask1 = 0;

    for (int i = 0; i < N_MOT; i++)
    {
        mask1 |= 1 << mot.M[i].PIN_ENA;
        mask1 |= 1 << mot.M[i].PIN_ENB;
    }
    mask2 |= (1 << STCPpin) | (1 << SHCPpin) | (1 << DSpin) | (1 << CSEpin);

    gpio_config_t io_conf_1;
    io_conf_1.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf_1.mode = GPIO_MODE_OUTPUT;
    io_conf_1.pull_down_en = 0;
    io_conf_1.pull_up_en = 0;
    io_conf_1.pin_bit_mask = mask2;
    gpio_config(&io_conf_1);

    io_conf_1.mode = GPIO_MODE_INPUT;
    io_conf_1.pull_down_en = 1;
    io_conf_1.pin_bit_mask = mask1;
    gpio_config(&io_conf_1);

    gpio_set_level(STCPpin, 0);
    gpio_set_level(SHCPpin, 0);
}

void init_pid()
{
    def_mot mot;
    set_mot(&mot);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = Freq_pwm; //frequency = 500Hz,
    pwm_config.cmpr_a = 0;           //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;           //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    for (int i = 0; i < N_MOT; i++)
    {
        mcpwm_gpio_init(mot.M[i].M_SET.UNIT, mot.M[i].M_SET.CAP, mot.M[i].PIN_ENA);
        mcpwm_gpio_init(mot.M[i].M_SET.UNIT, mot.M[i].M_SET.NA, mot.M[i].PIN_PWM);
        mcpwm_init(mot.M[i].M_SET.UNIT, mot.M[i].M_SET.TIMER, &pwm_config);
    }

    for (int i = 0; i < N_MOT; i++)
    {
        mcpwm_capture_enable(mot.M[i].M_SET.UNIT, mot.M[i].M_SET.SELECT_CAP, mot.M[i].M_SET.POS, 0);
    }

    mcpwm_isr_register(MCPWM_UNIT_0, ISR_CAP_UNIT0, NULL, ESP_INTR_FLAG_IRAM, NULL); //Set ISR Handler
    mcpwm_isr_register(MCPWM_UNIT_1, ISR_CAP_UNIT1, NULL, ESP_INTR_FLAG_IRAM, NULL); //Set ISR Handler

    MCPWM[MCPWM_UNIT_0]->int_ena.cap0_int_ena = 1;
    MCPWM[MCPWM_UNIT_0]->int_ena.cap1_int_ena = 1;
    MCPWM[MCPWM_UNIT_1]->int_ena.cap0_int_ena = 1;
    MCPWM[MCPWM_UNIT_1]->int_ena.cap1_int_ena = 1;

    for (int i = 0; i < 4; i++)
        val_mot[i] = xQueueCreate(1, sizeof(capture));

    q_giri_voluti = xQueueCreate(1, sizeof(passa_voluti));

    //xTaskCreate (task_pid, "task_pid" , 8192 , NULL , 1 | portPRIVILEGE_BIT  , NULL );
}

uint8_t qav = 0;
static void periodic_timer_callback(void* arg)
{
    /*
    int64_t time_since_boot = esp_timer_get_time();
    printf("%lld \n", time_since_boot);
    qav=1;
    */
    xQueueSendFromISR(q_avvio, &qav, 0);


}

void app_main()
{
    uart_set_baudrate(UART_NUM_0, 500000);
    esp_timer_init();
    gpio_set_level(CSEpin, 0);
    init_gpio();
    init_serial();
    init_pid();

    passa_voluti voluti;
    voluti.giri_voluti[0] = 0;
    voluti.giri_voluti[1] = 0;
    voluti.giri_voluti[2] = 0;
    voluti.giri_voluti[3] = 0;
    xQueueSend(q_giri_voluti, &voluti, 0);
    //uint8_t x=0;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    ////////////////////////////////////////////////////////////////////////////////PID
    double giri_attuali[N_MOT] = {0}, giri_voluti = 0, errore, olderrore[N_MOT] = {0}, proporzionale, integrale[N_MOT] = {0}, derivata, correzione, attuali[N_MOT] = {0};
    double kit = ki * (0.002), kdt = kd / (0.002), k_rid = 100.0 / 1023.0, k_dist =  0.002 * 3.14159;
    double giri_temp[4][2];
    double distanza[4], min_distanza, direzione;
    double diametro = 70;
    //int count[4];
    capture in[4];
    uint8_t ctr = 0, M_at = 0, BYTE = 0, r_dis;
    //int64_t old_time = 0;
    passa_voluti pass_v;
    for (int i = 0; i < 4; i++)
    {
        in[i].valore = 0;
        in[i].stato = 0;
    }
    //passa_shift pass_s;
    def_mot mot;
    set_mot(&mot);
    uint8_t avv;


    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        .name = "periodic"
    };

    q_avvio = xQueueCreate(1, sizeof(uint8_t));
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    esp_timer_start_periodic(periodic_timer, 500);


    gpio_set_level(CSEpin, 1);
    //long counttime = 0;
    
    while (1)
    {

        xQueueReceive(q_avvio, &(avv), 1000);  //recezione bloccante, il programma viene eseguito quando riceve il valore di avvio, questo è generato ogni 500us da un timer ad alta precisione     
        M_at++; //aumento il numero di motore (motore che su cui eseguirè il PID in questo ciclo)
        if (M_at >= 4) // ci sono al massimo 4 motori
            M_at = 0;
        ctr = 0; //controllo esatta recezione nuova velocità istantanea
        if (mot.M[M_at].E_MIS==1 && xQueueReceive(val_mot[M_at], &(in[M_at]), 0) == pdTRUE)// ricevo dalla queue dei motori, la recezione è avvenuta solo se la funzione di recezione ritorna pdTRUE
        {
            if (in[M_at].valore != 0) 
            {
                giri_temp[M_at][1] = (((double)80000000.0) / (((double)in[M_at].valore) * 990.0)); //calcolo velocità ruota (fclock = 80MHz) (trasmissione 990:1 )
                if (giri_temp[M_at][1] < 3) //se la ruota stà andando a meno di 3 giri/s
                {
                    giri_temp[M_at][2] = giri_temp[M_at][1] * ((double)(in[M_at].stato * 2) - 1) * (double)(mot.M[M_at].COR_MIS); //aggiusto la mia velocità di segno con una correzzione costante e con il valore della seconda fase
                    //if(ABS(giri_temp[M_at][2]-giri_attuali[M_at])<0.25 || count[M_at]>=4){
                    //count[M_at]=0;
                    giri_attuali[M_at] = giri_temp[M_at][2]; //aggiorno la velocità
                    ctr = 1; //misura andata a buon fine
                    //}else count[M_at]++;
                }
            }
        }
        if (ctr != 1) //se la misura non è stata effettuata
        {
            if (ABS(giri_attuali[M_at]) < 0.01) //sotto una certa velocità viene considerato 0, altrimenti la funzione sarebbe asintotica rispetto alle ascisse
                giri_attuali[M_at] = 0;
            else
                giri_attuali[M_at] += giri_attuali[M_at] * -0.01; //correggo la velocità ipotizzandone una diminuzione esponenziale della velocità, questo si verifica a basse velocità dove le misure sono meno frequenti, e questa correzzione aiuta il pid a correggere proprio in queste zone
        }
        
        
        xQueueReceive(q_giri_voluti, &pass_v, 0); //ricevo le velocità volute dalla seriale se queste sono disponibili, altrimeni rimangono quelle precedenti
        giri_voluti = pass_v.giri_voluti[M_at]; //ricavo la velocità del singolo motore
        //if(counttime==2500) giri_voluti = 1;
        //if(counttime==3500) giri_voluti = 0;
        //if(M_at==0){
        //    printf("%f %f %f %f %f\n",giri_voluti*1000,giri_attuali[0]*1000,giri_attuali[1]*1000, giri_attuali[2]*1000,giri_attuali[3]*1000); //Debug principale
        //   //counttime++;
        //}
       
        //printf("%f\n",giri_voluti);

        ////PID
        proporzionale=0;
        derivata = 0;

        if ((giri_voluti != 0 || (giri_voluti == 0 && giri_attuali[M_at] != 0)) && mot.M[M_at].E_PID==1)  //agisco se la velocità voluta è diversa da zero, oppure se è zero ma la velocità del motore non è zero
        {
            errore = giri_voluti - giri_attuali[M_at]; //calcolo errore
            proporzionale = errore * kp; //calcolo correzione proporzionale
            integrale[M_at] += kit * errore; //calcolo correzione integrale
            derivata = (errore - olderrore[M_at]) * kdt; //calcolo correzione derivativa
            olderrore[M_at] = errore; //aggiorno errore precedente
            integrale[M_at] = integrale[M_at] > limI ? limI : integrale[M_at] < -limI ? -limI : integrale[M_at]; //limitazioni intergale
            proporzionale = proporzionale > limP ? limP : proporzionale < -limP ? -limP : proporzionale; //limitazioni proporzionale
            derivata = derivata > limD ? limD : derivata < -limD ? -limD : derivata; //limitazioni derivata
            correzione = proporzionale + derivata + integrale[M_at] + attuali[M_at]; //calcolo correzzione con l'aggiunta della velocità precendente (tutte le correzzioni devono tendere a 0)
            correzione = correzione > 1023 ? 1023 : correzione < -1023 ? -1023 : correzione; //limitazioni correzione
            
        }
        else
            correzione = 0; //altrimenti imposto correzione a zero

        //if(M_at==0){
        //    printf("%f  %f  %f  %f  %f %f \n", giri_attuali[M_at]*1000, giri_voluti*1000, proporzionale*10, integrale[M_at]*1000, derivata, correzione);
        //}

        attuali[M_at] = correzione; //imposo la correzzione
        correzione *= k_rid;  //sistemazione correzzione costante
        BYTE &= ~((1 << mot.M[M_at].PIN_DR0) | (1 << mot.M[M_at].PIN_DR1));  //imposto entrambe le uscite del motore a 0 sull'immagine delle uscite digiali dello shiftregister
        if (correzione < 0) //se la correzione è negativa
        {
            correzione *= -1; //valore assoluto correzione
            BYTE |= (1 << mot.M[M_at].PIN_DR0) /* | (0<<mot.M[M_at].PIN_DR1)*/; // Imposto un verso di rotazione (INDIETRO)
        }
        else if (correzione == 0)
        {
            BYTE |= (1 << mot.M[M_at].PIN_DR0) | (1 << mot.M[M_at].PIN_DR1); // impostando entrabe le uscite del motore a livello alto (freno motore)
        }
        else
        {
            BYTE |= /*(0<<mot.M[M_at].PIN_DR0) |*/ (1 << mot.M[M_at].PIN_DR1); // Imposto un verso di rotazione (AVANTI)
        }
        set_shift(BYTE);  //scrivo le uscite digitali dello shiftregister
        //if(M_at==M_DISX)correzione=50;
        mcpwm_set_duty(mot.M[M_at].M_SET.UNIT, mot.M[M_at].M_SET.TIMER, MCPWM_OPR_A, correzione); //imposto correzzione PWM

        if (xQueueReceive(r_distanza, &r_dis, 0) == pdTRUE) //ricevo comando di resettare la distanza compiuta
            for (uint8_t i = 0; i < 4; i++)
                distanza[i] = 0; //la imposto a 0 per tutti i motori

        xQueueReceive(q_diametro, &diametro, 0); //aggiorno il diametro delle ruote se diponibile

        distanza[M_at] += giri_attuali[M_at] * k_dist* diametro; //calcolo distanza compita dal motore (dist = distPrecedente + (velocità  * diametro *  0.002 * 3.14159 ))

        //calcolo distanza complessiva del robot, in base alle distanze singole
        direzione = distanza[0] + distanza[1] + distanza[2] + distanza[3];  //calcolo una direzione prevalente
        min_distanza = distanza[3];
        if(direzione>=0){ // se la direzione è avanti prendo come distanza il motore che ne ha percorsa meno
            for (uint8_t i = 0; i < 3; i++)
                if (distanza[i] < min_distanza)
                    min_distanza = distanza[i];
        }else{     // se la direzione è indietro prendo come distanza il motore che ne ha percorsa di più
            for (uint8_t i = 0; i < 3; i++)
                if (distanza[i] > min_distanza)
                    min_distanza = distanza[i];
        }
        //if(M_at==0) printf("%f %f %f %f %f\n", distanza[0] ,distanza[1] ,distanza[2] , distanza[3],min_distanza); 

        xQueueOverwrite(q_distanza, &min_distanza); //invio la distanza alla comunicazione, così da poter essere usata se richiesta
    }
}
