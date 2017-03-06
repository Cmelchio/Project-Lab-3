#ifndef RF_GENERATOR_LCD_CONTROLLER_H_
#define RF_GENERATOR_LCD_CONTROLLER_H_

#define LCD_PORT P2OUT
#define LCD_DIR P2DIR

#define LCD_EN BIT0
#define LCD_RW BIT1
#define LCD_RS BIT2
#define LCD_DB BIT4 | BIT5 | BIT6 | BIT7

#define ROW 2
#define COL 16

#define LCD_HOME 0x80
#define delay_mS 16000

#define TA0_COUNT 1000



#include <msp430g2553.h>


class LcdController {
  public:
    // Initialized the LCD's displayed frequency with 'initial_frequency'.
    LcdController(double initial_frequency);

    // Updates the LCD's displayed frequency with 'new_frequency'.
    void Update(double new_frequency);

    void Cursor(unsigned char cursor_addr);

    unsigned char DataParse(double new_frequecy);

    void SendData(unsigned char data);

    void SendCommand(unsigned char cmd);

    static void Init();

    static void Clock();


  private:



    static void Delay(unsigned int delay_time);

    static void CheckBusy(void);

    static void setWrite(void);

    static void setRead(void);


};

#endif /* RF_GENERATOR_LCD_CONTROLLER_H_ */
