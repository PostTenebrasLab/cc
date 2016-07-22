/*
    CC -- NTSC closed captioning decoder

    Copyright 2016 magnustron

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

/* NTSC TIMING */
#define TCAPFIELD 200
#define VSYNCTHR  800
#define T0        465
#define NSAMPLES  280

/* DISPLAY */
#define LCD_I2C_ADDR 0x27
#define LED_PIN  3
#define En_PIN   2
#define Rw_PIN   1
#define Rs_PIN   0
#define D4_PIN   4
#define D5_PIN   5
#define D6_PIN   6
#define D7_PIN   7
#define LED_POL POSITIVE

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(LCD_I2C_ADDR,En_PIN,Rw_PIN,Rs_PIN,D4_PIN,D5_PIN,D6_PIN,D7_PIN,LED_PIN,LED_POL);

volatile uint8_t captured=0;
volatile uint8_t byte1;
volatile uint8_t byte2;
volatile uint32_t captured_frame;
ISR(TIMER1_CAPT_vect) {
  static uint16_t lastt;
  static uint32_t frame=0;
  static uint8_t nlong=0;
  static uint8_t nshort=0;
  static uint8_t field=0;
  static uint8_t line=0;
  uint16_t t=ICR1;
  uint16_t dt=t-lastt;
  PORTB|=0x01;
  if (dt<VSYNCTHR) {
    line=0;
    PORTB|=0x02;
    if (nlong) {
      nlong =0;
      nshort=0;
    }
    ++nshort;
    if (nshort==6) {
      uint8_t tmp;
      uint16_t t0=t+TCAPFIELD;
      asm volatile(
        "  lds   r24,%[tcnt1l]         \n" /* +2    */
        "  lds   r25,%[tcnt1h]         \n" /* +2    */
        "  sub   r24,%[t0l]            \n" /* +1    */
        "  sbc   r25,%[t0h]            \n" /* +1    */
        /* bit 0 */                        /* x0 x1 */
        "  sbrc  r24,0                 \n" /* +2/+1 */
        "  adiw  r24,1                 \n" /* +0/+2 */
                                           /*E+2/+3 */
        /* bit 1 */                        /* 0x 1x */
        "  sbrs  r24,1                 \n" /* +1/+2 */
        "  rjmp  timed_capture_wait_%= \n" /* +2/+0 */
        "  adiw  r24,2                 \n" /* +0/+2 */
        "  nop                         \n" /* +0/+1 */
                                           /*E+3/+5 */
        "timed_capture_wait_%=:        \n" /* +2/+0 */
        "  adiw  r24,4                 \n" /* +2    */
        "  brne  timed_capture_wait_%= \n" /* +2/+1 */
        "  sbi   %[portb],4            \n" /* +2    */
        "  in    %[data],%[acsr]       \n" /* +1    */
        "  cbi   %[portb],4            \n" /* +2    */
        :[data]  "=r" (tmp)
        :[t0l]    "r" ((t0)>>0&0xFF)
        ,[t0h]    "r" ((t0)>>8&0xFF)
        ,[acsr]   "I" (_SFR_IO_ADDR(ACSR))
        ,[portb]  "I" (_SFR_IO_ADDR(PORTB))
        ,[tcnt1l] "M" (_SFR_MEM_ADDR(TCNT1L))
        ,[tcnt1h] "M" (_SFR_MEM_ADDR(TCNT1H))
        :"r24","r25"
      );
      field=(tmp&0x20)?2:1;
      if (field==1) ++frame;
      if (field==1)
        PORTB|=0x08;
      else
        PORTB&=~0x08;
    }
  }
  else {
    PORTB|=0x04;
    if (nshort) {
      nshort= 0; 
      nlong = 0;
      line  =10;
    }
    if (line<255)
      ++line;
    ++nlong;
  }
  if (field==1 && line==21 && !captured) {
    ADMUX=3; // move threshold to slice bits
    uint16_t t0=t+T0;
    asm volatile(
      "  lds   r24,%[tcnt1l]         \n" /* +2    */
      "  lds   r25,%[tcnt1h]         \n" /* +2    */
      "  sub   r24,%[t0l]            \n" /* +1    */
      "  sbc   r25,%[t0h]            \n" /* +1    */
      /* bit 0 */                        /* x0 x1 */
      "  sbrc  r24,0                 \n" /* +2/+1 */
      "  adiw  r24,1                 \n" /* +0/+2 */
                                         /*E+2/+3 */
      /* bit 1 */                        /* 0x 1x */
      "  sbrs  r24,1                 \n" /* +1/+2 */
      "  rjmp  timed_capture_wait_%= \n" /* +2/+0 */
      "  adiw  r24,2                 \n" /* +0/+2 */
      "  nop                         \n" /* +0/+1 */
                                         /*E+3/+5 */
      "timed_capture_wait_%=:        \n" /* +2/+0 */
      "  adiw  r24,4                 \n" /* +2    */
      "  brne  timed_capture_wait_%= \n" /* +2/+1 */
      ".set i,0\n"
      ".rept 8\n"
      "  sbi   %[portb],5            \n" /* +2     */
      "  in    r24,%[acsr]           \n" /* +1     */
      "  cbi   %[portb],5            \n" /* +2     */
      "  bst   r24,5                 \n" /* +1     */
      "  bld   %[byte1],i            \n" /* +1     */
      ".rept 25\n"
      "  nop\n"
      ".endr   \n" 
      ".set i,i+1\n"
      ".endr\n"
      ".set i,0\n"
      ".rept 8\n"
      "  sbi   %[portb],5            \n" /* +2     */
      "  in    r24,%[acsr]           \n" /* +1     */
      "  cbi   %[portb],5            \n" /* +2     */
      "  bst   r24,5                 \n" /* +1     */
      "  bld   %[byte2],i            \n" /* +1     */
      ".rept 25\n"
      "  nop\n"
      ".endr   \n" 
      ".set i,i+1\n"
      ".endr\n"
      :[byte1] "=r" (byte1)
      ,[byte2] "=r" (byte2)
      :[t0l]    "r" ((t0)>>0&0xFF)
      ,[t0h]    "r" ((t0)>>8&0xFF)
      ,[acsr]   "I" (_SFR_IO_ADDR(ACSR))
      ,[portb]  "I" (_SFR_IO_ADDR(PORTB))
      ,[tcnt1l] "M" (_SFR_MEM_ADDR(TCNT1L))
      ,[tcnt1h] "M" (_SFR_MEM_ADDR(TCNT1H))
      :"r24","r25"
    );
    ADMUX=0; // move threshold back for syncs
    captured=1;
    captured_frame=frame;
    TIFR1|=1<<ICF1; // drain capture event
  }
  lastt=t;
  PORTB&=~0x07;
}

void setup() {
  // Timer/Counter 0 (default time base):
  TIMSK0=0x00; // disable interrupts
  // ADC:
  ADCSRA=0x00; // disable
  ADMUX =0; // select input 0
  // Analog comparator:
  ADCSRB=0x40; // use ADC multiplexer for negative input
  ACSR  =0x04; // use to capture Timer/Counter 1, no interrupts
  // Timer/Counter 1:
  TIMSK1=0x20; // interrupts on capture
  TCCR1A=0x00; // disable any PWM out
  TCCR1B=0x01; // falling edge, no noise canceling, full speed
  // I/O:
  Serial.begin(115200);
//  UCSR0B&=0x1F; // disable interrups
  interrupts();
  DDRB=0xFF; // PORT B is for debugging
  Serial.print("Scenarist_SCC V1.0");
  lcd.begin(16,2);               // initialize the lcd 
}

void loop() {
  static uint32_t lastframe=-1;
  char msg[50];
  if (captured) {
    if (!(byte1==0x80 && byte2==0x80)) {
      if (captured_frame==lastframe+1) {
        int j=sprintf(msg  ," %02X",byte1);
              sprintf(msg+j,"%02X" ,byte2);
        Serial.print(msg);
      }
      else {
        int i=sprintf(msg    ,"\n\n%d:\t",captured_frame);
        int j=sprintf(msg+i  ,"%02X",byte1);
              sprintf(msg+i+j,"%02X",byte2);
        Serial.print(msg);
      }
      lastframe=captured_frame;
    }
    int i=sprintf(msg    ,"%d: ",captured_frame);
    int j=sprintf(msg+i  ,"%02X",byte1);
          sprintf(msg+i+j,"%02X",byte2);
    uint8_t char1=byte1&0x7F;
    uint8_t char2=byte2&0x7F;
    if (0x20<=char1 && 0x20<=char2) {
      msg[0]=char1;
      msg[1]=char2;
      msg[2]=0;
      lcd.print(msg);
    }
    else if (byte1==0x94 && byte2==0x20) {
      lcd.clear();
      lcd.home();
    }
    captured=0;
  }
}

