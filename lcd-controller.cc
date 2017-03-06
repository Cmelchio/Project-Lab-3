#include "lcd-controller.h"
#include <msp430g2553.h>
#include <stdint.h>


LcdController::LcdController(double initial_frequency) {
  this->Update(initial_frequency);
}

void LcdController::Update(double new_frequency) {
  // TODO: Update the LCD.
}

void LcdController::Init()
{

    LCD_DIR |= LCD_DB | LCD_EN | LCD_RS | LCD_RW;
    LCD_PORT &= ~(LCD_DB | LCD_EN | LCD_RS);

}

void LcdController::Delay(unsigned int delay_time)
{
 //TODO: Calculate delay time based upon Timer.

}



void LcdController::Clock()
{
    TA0CTL |= TASSEL_2 + ID_0 + MC_1;
    TA0CCTL0 |= CCIE;
    TA0CCR0 |= TA0_COUNT;


}
