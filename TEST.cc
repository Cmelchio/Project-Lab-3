/***********************************************************************************************
 *                          MSP432P401
 *                     ---------------------
 *                     |                   |
 *                     |DVCC           DVSS|
 *                 <---|P1.0           P2.6|---> LCD DB4
 *                 <---|P1.1           P2.7|--->
 *                 <---|P1.2           TEST|--->
 *      ENC A      <---|P1.3            RST|--->
 *      ENC B      <---|P1.4           P1.7|---> UCB0SDA : SDA I2C Data
 *      ENC PB     <---|P1.5           P1.6|---> UCB0SCL : SCL I2C clock
 *      LCD ENABLE <---|P2.0           P2.5|---> LCD DB2
 *      LCD R/W    <---|P2.1           P2.4|---> LCD DB1
 *      LCD RS     <---|P2.2           P2.3|---> LCD DB0
 *                     |                   |
 *                     ---------------------
 *                     */

//*********************************************************************************************
#include <msp430g2553.h>
#include <stdint.h>

// LCD DISPLAY:
#define RS_DR P2OUT = P2OUT | BIT2 // define RS high
#define RS_CWR P2OUT = P2OUT & (~BIT2) // define RS low
#define RW_READ P2OUT = P2OUT | BIT1 // define Read signal R/W = 1 for reading
#define RW_WRITE P2OUT = P2OUT & (~BIT1) // define Write signal R/W = 0 for writing
#define ENABLE_HIGH P2OUT = P2OUT | BIT0 // define Enable high signal
#define ENABLE_LOW P2OUT = P2OUT & (~BIT0) // define Enable Low signal
#define LCD_DB = BIT3 | BIT4 | BIT5 | BIT6

void EnableNybble();
void Delay_mS(int);
void Delay_uS(int);



void CheckBusy();
void SendCommand(unsigned char);
void SendData(unsigned char);
void SendString(char*);
void LCDInit();

static const unsigned char FOUR_BITS_TWO_LINES = 0x28;
static const unsigned char SET_CURSOR = 0x10;
static const unsigned char SET_BLINKING_CURSOR = 0x0F;
static const unsigned char SET_ENTRY_MODE = 0x06;


//
//*********************************************************************************************
// QUADRATURE ENCODER SETUP
//


#define TIMER_A0_COUNT  65535;
#define TIMER_ENABLE_COUNT 1000;
#define ENC_CW  BIT3
#define ENC_CCW BIT4
#define ENC_PB  BIT5
#define ENC_PINS P1IN |= (ENC_CW + ENC_CCW + ENC_PB);

int  EncoderRead();
void EncoderSetup();
void UnusedPorts();
void EncoderLoop();


static bool ENC_CW_FLAG;
static bool ENC_CCW_FLAG;
static bool ENC_PB_FLAG;
static int  COUNT_uS;
static int  COUNT_mS;


//*********************************************************************************************
int main(void){

//Calibrated Clock Setup
    WDTCTL = WDTPW + WDTHOLD;   //Stop WDT
    DCOCTL = CALDCO_16MHZ;      //Set DCO to 16 MHz
    BCSCTL1 = CALBC1_16MHZ;     //Set DCO Step & Modulation

//TIMER A0 Setup:
    TA0CCR0 = 0;       // Load count value into Timer A0.
    TA0CCTL0 = CCIE;                // Enable Interrupts for Timer A0.
    TACTL = TASSEL_2 + MC_1 + ID_3; // Select SMCLK, /8 Divisor, Upmode.

    __bis_SR_register(GIE);         //Enable General Interrupts Bit

//General Setup:
    UnusedPorts();
    LCDInit();
    EncoderSetup();

    while(1)
        __bis_SR_register(LPM0 + GIE);



}
// END OF MAIN LOOP.
//*******************************************************************************************

//-------------------------------------ENCODER-----------------------------------------------------------
void EncoderSetup() {

    ENC_CW_FLAG = 0;
    ENC_CCW_FLAG = 0;
    ENC_PB_FLAG = 0;

    P1DIR   |= ~(ENC_CW + ENC_CCW + ENC_PB);
    P1REN   |= ENC_CW + ENC_CCW + ENC_PB;                // Enable internal pull-up/down resistors
    P1OUT   |= ENC_CW + ENC_CCW + ENC_PB;                   //Select pull-up mode for P1.3
    P1IE    |= ENC_CW + ENC_CCW + ENC_PB;                       // P1.3 interrupt enabled
    P1IES   |= ENC_CW + ENC_CCW + ENC_PB;                    // P1.3 Hi/lo edge
    P1IFG   &= ~(ENC_CW + ENC_CCW + ENC_PB);               // P1.3 IFG cleared

}

int EncoderRead() {
    static int8_t   ENC_STATES[] =  {0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0};
    uint8_t PREV_STATE = 0;
    PREV_STATE <<= 2;
    PREV_STATE = ENC_PINS;

    return (ENC_STATES[PREV_STATE & 0x0f]);
}


//-------------------------------------LCD DISPLAY-----------------------------------------------------------

/*Function for initializing the LCD pins and resetting the display*/
void LCDInit()
{
    P2DIR |= 0xE0;
    P2OUT |= 0x00;
    SendCommand(FOUR_BITS_TWO_LINES);
    SendCommand(SET_CURSOR);
    SendCommand(SET_BLINKING_CURSOR);
    SendCommand(SET_ENTRY_MODE);

}

void EnableNybble() {

    ENABLE_HIGH;
    //Delay(2);
    ENABLE_LOW;
}

/*Function used to check the state of the LCD*/
void CheckBusy(void)
{
    P2DIR &= ~(BIT7);   // make P2.7 as input
    while((P2IN&BIT7)==1)
        {
            EnableNybble();
        }
    P2DIR |= BIT7;  // make P2.7 as output
}

/*Function for giveing specific commands to the LCD*/
void SendCommand(unsigned char cmd)
{

    //CheckBusy();
    RW_WRITE;
    RS_CWR;
    P2OUT = (P2OUT & 0x00)|(cmd);
    EnableNybble();   // give enable trigge
}

/*Function for writing a char value to the LCD*/
void SendData(unsigned char data)
{
    CheckBusy();
    RW_WRITE;
    RS_DR;
    P2OUT = (P2OUT & 0x00)|(data);
    EnableNybble();

}



//-------------------------------------GENERAL FUNCTIONS-----------------------------------------------------------
void UnusedPorts() {
    P3DIR = 0;
    P3OUT = 0;

}

void Delay_uS(int n){

    const int ONE_uS = 2;
    COUNT_uS = n * ONE_uS;
    TA0CCR0 = COUNT_uS;

    return;

}

void Delay_mS(int n){

    const int ONE_mS = 2000;
    COUNT_mS = n * ONE_mS;
    TA0CCR0 = COUNT_mS;

    return;
}


//*********************************************************************************************************************
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
{

    if(P1IFG & ENC_CW)
        ENC_CW_FLAG = 1;
    else if (P1IFG & ENC_CCW)
        ENC_CCW_FLAG = 1;
    else if (P1IFG & ENC_PB)
        ENC_PB_FLAG = 1;
    else
        P1IFG &= 0x00;

}
