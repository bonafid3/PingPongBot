#include "AT91SAM7S256.h"
#include "lib_AT91SAM7S256.h"
#include <ctl_api.h>

#include <__cross_studio_io.h>

#include <stdio.h>
#include <string.h>

void AT91F_TC_Open ( AT91PS_TC TC_pt, unsigned int Mode, unsigned int TimerId);
int pressed(unsigned int btn);

void msleep(const unsigned int);
int waitFor(const unsigned int delay, const char *data);

volatile int gCritSection = 0;

void step0();
void step1();
void step2();
void step3();

void setLedValue(const int val);

#define PA0 (1<<0) // pwm0
#define PA1 (1<<1) // pwm1
#define PA2 (1<<2) // pwm2

#define PA5 (1<<5)
#define PA6 (1<<6)

#define PA8 (1<<8) // pwm2

#define PA13 (1<<13) // gpio13
#define PA15 (1<<15) // gpio15
#define PA17 (1<<17) // gpio17
#define PA19 (1<<19) // gpio19

#define PA21 (1<<21) // led0
#define PA23 (1<<23) // led1
#define PA25 (1<<25) // led2

#define PA27 (1<<27) // button main
#define PA29 (1<<29) // button up
#define PA31 (1<<31) // button down

#define PA30 (1<<30) // reset wifi

#define BTN_MAIN (PA21)
#define BTN_UP (PA23)
#define BTN_DOWN (PA25)

//#define EXT_OC                 18432000
#define MCK (48054860)

#define MCKDIV1  (0x00)
#define MCKDIV2 (0x01)
#define MCKDIV4 (0x02)
#define MCKDIV8 (0x03)
#define MCKDIV16 (0x04)
#define MCKDIV32 (0x05)
#define MCKDIV64 (0x06)
#define MCKDIV128 (0x07)
#define MCKDIV256 (0x08)
#define MCKDIV512 (0x09)
#define MCKDIV1024 (0x0A)

#define USART0_INTERRUPT_LEVEL		1
#define TIMER0_INTERRUPT_LEVEL		1
#define TIMER1_INTERRUPT_LEVEL		1
#define PIOA_INTERRUPT_LEVEL		1

#define RECV_BUFF_SIZE    8192
#define MAX_CONNECTIONS   4

typedef struct {
char data[RECV_BUFF_SIZE];
int size;
} sBuffer;


void usart0_c_irq_handler(void);
void timer0_c_irq_handler(void);
void timer1_c_irq_handler(void);
void pioa_c_irq_handler(void);
void PIT_ISR(void);

typedef enum { eSTATE_1ST, eSTATE_2ND, eSTATE_3RD, eSTATE_ALL } tSelectedMotorState;

volatile tSelectedMotorState gSelectedMotorState = eSTATE_ALL;

typedef enum {
  eUS_STATE_ERROR,
  eUS_STATE_CIFSR,
  eUS_STATE_CIPMUX,
  eUS_STATE_CIPSTART,
//  eUS_STATE_CIPSERVER,
  eUS_STATE_DATA
  } tUartState;

volatile int gUartState = eUS_STATE_CIFSR;

#define MIN_DUTY 3000
#define MAX_DUTY 4500
#define PWM_CHANNELS 3

volatile int duties[PWM_CHANNELS] = { MIN_DUTY,MIN_DUTY,MIN_DUTY };

volatile int step;
volatile unsigned int cnt=0, gTickCount=0;

volatile int gTimeout = 10;
volatile int gBtnMainPressed = 0;
volatile int gBtnUpPressed = 0;
volatile int gBtnDownPressed = 0;

volatile int gReset = 0;
volatile int gPause = 0;

volatile unsigned int rPos = 0;
volatile unsigned int wPos = 0;

void clearBuffer(volatile sBuffer *);

volatile sBuffer gData[MAX_CONNECTIONS];

volatile sBuffer buffer;

volatile char * gPos = 0;

int endsWith(const sBuffer *b, const char *data);

// MUST RUN THE CODE WITH "GO" IN ORDER TO HAVE THE INTERRUPT RUNNING!!!

void PIT_ISR(void)              /* PIT interrupt service routine */   
{   
  AT91F_PITGetPIVR(AT91C_BASE_PITC);        // clear PITS   
  gTickCount++; // 1ms
}

void usart0_c_irq_handler()
{
  if(gReset)
  {
    gReset = 0;
    buffer.size = 0;
    gPos = 0;
  }
  if(buffer.size+1 < RECV_BUFF_SIZE)
  {
    buffer.data[buffer.size++] = AT91C_BASE_US0->US_RHR;
    buffer.data[buffer.size] = 0;
  }
}

int endsWith(const sBuffer *b, const char *data)
{
  int i = b->size - 1;
  int j = strlen(data)-1;
  
  while(b->data[i--] == data[j--] && j >= 0);

  if(j < 0)
    return 1;

  return 0;
}

int main()
{ 
  // enable Parallel IO controller and PWM Controller
  AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, (1 << AT91C_ID_PIOA) | (1 << AT91C_ID_PWMC) | (1 << AT91C_ID_SYS) | (1 << AT91C_ID_US0) ) ;

  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA0 ) ; // pwm0
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA1 ) ; // pwm1a
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA2 ) ; // pwm2

  AT91F_PIO_CfgInput( AT91C_BASE_PIOA, PA5 ) ; // rxd0
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA6 ) ; // txd0

  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA8 ) ;

  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA13 ) ; // gpio
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA15 ) ; // gpio
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA17 ) ; // gpio
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA19 ) ; // gpio

  AT91F_PIO_CfgInput(AT91C_BASE_PIOA, PA21); // button main
  AT91F_PIO_CfgInput(AT91C_BASE_PIOA, PA23); // button up
  AT91F_PIO_CfgInput(AT91C_BASE_PIOA, PA25); // button down

  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA27 ) ; // led0
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA29 ) ; // led1
  AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA31 ) ; // led2

AT91F_PIO_CfgOutput( AT91C_BASE_PIOA, PA30 ) ; // RESET ESP8266 WIFI

  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA0 ); // pwm0
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA1 ); // pwm1
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA2 ); // pwm2

  AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA8 ); // led off

  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA13 ); // gpio
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA15 ); // gpio
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA17 ); // gpio
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA19 ); // gpio

  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA27 ); // led0
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA29 ); // led1
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA31 ); // led2

  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA30 ); // do the reset

  ctl_global_interrupts_enable();

  AT91F_PITC_CfgPMC();
  AT91F_PITInit(AT91C_BASE_PITC, 1, 48);
  AT91F_PITSetPIV(AT91C_BASE_PITC,3000-1);
  AT91F_PITEnableInt(AT91C_BASE_PITC);
  AT91F_AIC_ConfigureIt(AT91C_BASE_AIC,AT91C_ID_SYS,1,AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL,PIT_ISR); 
  AT91F_AIC_EnableIt(AT91C_BASE_AIC,AT91C_ID_SYS);

  AT91F_TC_Open(AT91C_BASE_TC0, AT91C_TC_CLKS_TIMER_DIV1_CLOCK, AT91C_ID_TC0); // TIMER_DIV1 = MAIN_CLOCK_FREQ / 2
  AT91F_TC_Open(AT91C_BASE_TC1, AT91C_TC_CLKS_TIMER_DIV1_CLOCK, AT91C_ID_TC1);

//AT91F_US0_CfgPIO();

	AT91F_PIO_CfgPeriph(
		AT91C_BASE_PIOA, // PIO controller base address
		((unsigned int) AT91C_PA5_RXD0    ) |
		((unsigned int) AT91C_PA6_TXD0    ) |
		((unsigned int) AT91C_PA7_RTS0    ),
		((unsigned int) AT91C_PA2_SCK0    )); // Peripheral B

AT91F_US_Configure(AT91C_BASE_US0, MCK, AT91C_US_ASYNC_MODE, 115200, 0);
AT91F_US_EnableRx(AT91C_BASE_US0);
AT91F_US_EnableTx(AT91C_BASE_US0);

  // configure interrupt
  AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_TC0);
  AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_TC1);

  AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_US0);

  //AT91F_AIC_DisableIt(AT91C_BASE_AIC, AT91C_ID_PIOA);

  AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_US0, USART0_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, usart0_c_irq_handler);

  // timer interrupt that will let us send pulses to the stepper's pins right in time
  AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_TC0, TIMER0_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, timer0_c_irq_handler);

  // timer for system time count
  AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_TC1, TIMER1_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, timer1_c_irq_handler);

  // button interrupt
  //AT91F_AIC_ConfigureIt ( AT91C_BASE_AIC, AT91C_ID_PIOA, PIOA_INTERRUPT_LEVEL, AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL, pioa_c_irq_handler);

  AT91C_BASE_US0->US_IER = AT91C_US_RXRDY; // this is the interrupt event we want

  AT91C_BASE_TC0->TC_IER = AT91C_TC_CPCS;  //  IRQ enable CPC
  //AT91C_BASE_TC1->TC_IER = AT91C_TC_CPCS;  //  IRQ enable CPC

  AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_US0); // enable usart0 interrupt

  // by default the timer 0 is disabled, dont want the dispenser start on reset
  //AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_TC0); // enable timer0 interrupt
  //AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_TC1); // enable timer1 interrupt

  // start timer 0
  AT91C_BASE_TC0->TC_CCR = AT91C_TC_SWTRG;

  // start timer 1
  AT91C_BASE_TC1->TC_CCR = AT91C_TC_SWTRG;

  // --- PWM ---
  // http://read.pudn.com/downloads159/sourcecode/embed/716413/Xtest13_2.c__.htm

  //AT91F_PWMC_CH0_CfgPIO(); <-- configures all PWM0 including PA0, not good! use instead:
  AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA, AT91C_PA0_PWM0, 0); // peripheral A!
  AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA, AT91C_PA1_PWM1, 0); // peripheral A!
  AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA, AT91C_PA2_PWM2, 0); // peripheral A!
  //AT91F_PIO_CfgPeriph( AT91C_BASE_PIOA, 0, AT91C_PA11_PWM0); // peripheral B!

  AT91F_PWMC_CfgPMC(); // <- already enabled see AT91F_PMC_EnablePeriphClock

  AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);  //Disable PWM Interrupt on channel
  AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);  //Disable PWM Interrupt on channel
  AT91F_PWMC_InterruptDisable(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);  //Disable PWM Interrupt on channel

  // main clock: 48000000 Hz / 16 / divider = result in hz
  // period = 48000000 / 16 / 50Hz = 60000 is the divider above
  
  AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 0, (MCKDIV16 | AT91C_PWMC_CPOL), 60000, 3000);
  AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 1, (MCKDIV16 | AT91C_PWMC_CPOL), 60000, 3000);
  AT91F_PWMC_CfgChannel(AT91C_BASE_PWMC, 2, (MCKDIV16 | AT91C_PWMC_CPOL), 60000, 3000);

  AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);      //Enable channel
  AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID1);      //Enable channel
  AT91F_PWMC_StartChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID2);      //Enable channel

  //AT91F_PWMC_StopChannel(AT91C_BASE_PWMC, AT91C_PWMC_CHID0);      //Disable channel

//AT91PS_MC pMC;
//pMC->MC_FMR;
//AT91F_MC_EFC_PerformCmd(pMC, (48<<16));

  volatile unsigned int btn_main_tstamp = 0, btn_up_tstamp = 0, btn_down_tstamp = 0;

  AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA30 ); // do the reset
  // wait for UART
  //waitFor(10000, "WIFI");

  msleep(3000);

  //char atcmd[] = "AT+RESTORE\r\n"; //soft ap mode 192.168.4.1
  //char atcmd[] = "AT+CWSAP_DEF=\"PingPongBot\",\"\",6,0\r\n";
  //AT91F_PDC_SendFrame(AT91C_BASE_PDC_US0, atcmd, strlen(atcmd), (char *) 0, 0);
  //msleep(3000);

  clearBuffer(&buffer);

  while(1)
  {
    setLedValue(gUartState);

    int d = MIN_DUTY;
    for(int i=0; i<PWM_CHANNELS; i++)
    {
      if(!gPause)
      {
        d = duties[i];
      }
      AT91C_BASE_PWMC->PWMC_CH[i].PWMC_CDTYR = d;
    }

    switch(gUartState)
    {
      case eUS_STATE_CIFSR:
      {
        char atcmd[] = "AT+CIFSR\r\n";
        AT91F_PDC_SendFrame(AT91C_BASE_PDC_US0, atcmd, strlen(atcmd), (char *) 0, 0);
        if(!waitFor(2000, 0))
        {
          gUartState = eUS_STATE_ERROR;
          continue;
        }
        //debug_printf("%s\r\n",buffer.data);
        //clearBuffer(&buffer);
        gUartState++;
      }
      break;

      case eUS_STATE_CIPMUX:
      {
        char atcmd[] = "AT+CIPMUX=0\r\n";
        AT91F_PDC_SendFrame(AT91C_BASE_PDC_US0, atcmd, strlen(atcmd), (char *) 0, 0);
        if(!waitFor(2000, 0))
        {
          gUartState = eUS_STATE_ERROR;
          continue;
        }
        //clearBuffer(&buffer);
        gUartState++;
      }
      break;

      case eUS_STATE_CIPSTART:
      {
        char atcmd[] = "AT+CIPSTART=\"UDP\",\"0\",0,80,2\r\n";
        AT91F_PDC_SendFrame(AT91C_BASE_PDC_US0, atcmd, strlen(atcmd), (char *) 0, 0);
        if(!waitFor(2000, 0))
        {
          gUartState = eUS_STATE_ERROR;
          continue;
        }
        //clearBuffer(&buffer);
        gUartState++;
      }
      break;
/*
      case eUS_STATE_CIPSERVER:
      {
        char atcmd[] = "AT+CIPSERVER=1,80\r\n";
        AT91F_PDC_SendFrame(AT91C_BASE_PDC_US0, atcmd, strlen(atcmd), (char *) 0, 0);
        if(!waitFor(2000, 0))
        {
          gUartState = eUS_STATE_ERROR;
          continue;
        }
        //clearBuffer(&buffer);
        gUartState++;
      }
      break;
*/
      default:      
      case eUS_STATE_DATA:
      {
        if(gPos == 0)
          gPos = buffer.data;

        char *pos, *pose;

        if( (pos = strnstr(gPos, "+IPD,", buffer.size - (gPos-buffer.data))) !=0 &&
            (pose = strnstr(pos, ":", buffer.size - (pos-buffer.data))) != 0 )
        {
          
          gPos = pos;
          int conn=0, len=0, disp=0;
          if(sscanf(gPos, "+IPD,%i:", &len) == 1)
          {
            gPos = pose + 1 + len;
          
            while(buffer.data+buffer.size < gPos)
            {
              // keep on reading...
            }

            int pwm=0, duty=0;

            if(sscanf(pose + 1, "DISPENSER/%i\n", &disp) == 1)
            {
              if(disp == 1)
                AT91F_AIC_EnableIt (AT91C_BASE_AIC, AT91C_ID_TC0);
              else
                AT91F_AIC_DisableIt (AT91C_BASE_AIC, AT91C_ID_TC0);
            }
            else if(sscanf(pose + 1, "PWM%i/%i\n", &pwm, &duty) == 2)
            {
              //AT91C_BASE_PWMC->PWMC_CH[pwm].PWMC_CDTYR = MIN_DUTY + ((MAX_DUTY-MIN_DUTY)/100) * duty;
              duties[pwm] = MIN_DUTY + ((MAX_DUTY-MIN_DUTY)/100) * duty;
            }
            else if(sscanf(pose + 1, "PAUSE/%i\n", &gPause) == 1)
            {
              // this will be applied later
            }
          }
          gReset = 1;
        }
      }
      break;
    }

  }
}

int waitFor(const unsigned int delay, const char *data)
{
  if(gPos == 0)
    gPos = buffer.data;

  char *pos = 0;

  unsigned int tstamp = gTickCount;
  while(tstamp + delay > gTickCount)
  {
    int len = buffer.size - (gPos-buffer.data);
    if(data != 0 && (pos=strnstr(gPos, data, len))!=0)
    {
      gPos = pos+strlen(data);
      return 1;
    }
    if((pos=strnstr(gPos, "OK\r\n", len))!=0)
    {
      gPos = pos+4;
      return 1;
    }
    else if((pos=strnstr(gPos, "busy", len))!=0)
    {
      gPos = pos+4;
      return 0;
    }
    else if((pos=strnstr(gPos, "ERROR\r\n", len))!=0)
    {
      gPos = pos+7;
      return 0;
    }
  }
  //debug_printf(buffer.data);
  return 0;
}

void msleep(const unsigned int delay)
{
  unsigned int tstamp = gTickCount;
  while(tstamp + delay > gTickCount)
  {
    AT91PS_PWMC pPWM = AT91C_BASE_PWMC;
    pPWM->PWMC_CH[0].PWMC_CDTYR = 3000;
  }
}

void setLedValue(const int val)
{
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA27 ) ;
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA29 ) ;
  AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA31 ) ;

  AT91PS_PWMC pPWM = AT91C_BASE_PWMC;

  if(val == 0)
  {
    //pPWM->PWMC_CH[0].PWMC_CDTYR = 3300;
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA8 ) ;
  }
  else
  {
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA8 ) ;

    if(val & (1<<0))
    {
      AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA27 ) ;
    }
    if(val & (1<<1))
    {
      AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA29 ) ;
    }
    if(val & (1<<2))
    {
      AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA31 ) ;
    }
  }
}

void clearBuffer(volatile sBuffer *b)
{
  memset(b, 0, sizeof(sBuffer));
}


int pressed(unsigned int btn)
{
  int res = AT91F_PIO_GetInput( AT91C_BASE_PIOA );
  return !(res & btn); // pulled to ground
}

void AT91F_TC_Open ( AT91PS_TC TC_pt, unsigned int Mode, unsigned int TimerId)
{
  volatile unsigned int dummy;

  //* First, enable the clock of the TIMER
  AT91F_PMC_EnablePeriphClock ( AT91C_BASE_PMC, 1<< TimerId ) ;

  //* Disable the clock and the interrupts
  TC_pt->TC_CCR = AT91C_TC_CLKDIS ;
  TC_pt->TC_IDR = 0xFFFFFFFF ; // interrupt disable register

  //* Clear status bit
  dummy = TC_pt->TC_SR;
  //* Suppress warning variable "dummy" was set but never used

  (void)dummy;

  TC_pt->TC_CMR = Mode ;

  //* Enable the clock
  TC_pt->TC_CCR = AT91C_TC_CLKEN ;
}

void pioa_c_irq_handler(void)
{
  volatile unsigned int dummy;

  AT91PS_PIO PIO_pt = AT91C_BASE_PIOA;
  

  dummy = PIO_pt->PIO_ASR;
  (void)dummy;

}

// TIMER 1 interrupt routine
// this will be called when the 16 bit timer overflows
// the fastest tick is main clock / 2

// runs in every 366 micro seconds
void timer1_c_irq_handler(void)
{
  volatile unsigned int dummy;
  AT91PS_TC TC_pt = AT91C_BASE_TC1;
  dummy = TC_pt->TC_SR;
  (void)dummy;
  return;

/*
  cnt++;
  if(cnt >= ((MCK / 2) / 65535)) // 1 sec
  {
    cnt = 0;
  }*/
}

/// TIMER 0

void timer0_c_irq_handler(void)
{
  volatile unsigned int dummy;
  dummy = AT91C_BASE_TC0->TC_SR;
  (void)dummy;

  switch(step)
  {
    case 0:
      step0();
      step++;
    break;
    case 1:
      step1();
      step++;
    break;
    case 2:
      step2();
      step++;
    break;
    case 3:
      step3();
      step=0;
    break;

    default:
    step=0;
    break;
  }
}

void step0()
{
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA19 ) ;
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA17 ) ;
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA15 );
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA13 );
}

void step1()
{
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA19 );
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA17 ) ;
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA15 ) ;
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA13 );
}

void step2()
{
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA19 );
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA17 );
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA15 ) ;
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA13 ) ;
}

void step3()
{
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA19 ) ;
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA17 );
    AT91F_PIO_ClearOutput( AT91C_BASE_PIOA, PA15 );
    AT91F_PIO_SetOutput( AT91C_BASE_PIOA, PA13 ) ;
}
