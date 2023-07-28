//#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <iwdg.h>
#include <RotaryEncoder.h>
//====u8x8=====
//#include <U8x8lib.h>
//U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(/* clock=*/ PA5, /* data=*/ PA7, /* cs=*/ PB1, /* dc=*/ PA6, /* reset=*/ PB0);
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ PA5, /* data=*/ PA7, /* cs=*/ PB1, /* dc=*/ PA6, /* reset=*/ PB0);
//====u8x8=====

#define BEEPER PB10
#define LED_LEVEL PB11

HardwareSerial Serial1(PA10, PA9);
//HardwareSerial Serial2(PA3, PA2);
//HardwareSerial Serial3(PB11, PB10);

uint8_t bms_smart_request_msg[]  = {0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D};
uint8_t bms_jbd_request_msg[] = {0x01, 0x03, 0x00, 0x2E, 0x00, 0x01, 0xE4, 0x03};

enum BMS_TYPE {
	BMS_NONE = 0,
	BMS_SMART,
	BMS_JBD,
	BMS_MAX,
};

unsigned long bms_req_time = 0;
uint8_t settDataIn[50];
uint8_t bms_detected = 0;
uint8_t smart_bms = 0;
uint8_t bms_err = 0;
uint16_t capacity_percent = 0;
uint8_t bms_stype = 0;
uint8_t cap_min_level = 0;


unsigned long enc_timer = 0;
int16_t pos = 0;
int dir = 0;

#define ENC_CLK PB10
#define ENC_DATA PB11

volatile uint32_t encoderCount;

void encoder1_read(void)
{
  volatile static uint8_t ABs = 0;
  ABs = (ABs << 2) & 0x0f; //left 2 bits now contain the previous AB key read-out;
  ABs |= (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DATA);
  switch (ABs)
  {
    case 0x0d:
      encoderCount++;
      break;
    case 0x0e:
      encoderCount--;
      break;
  }
}

/*void HAL_SYSTICK_Callback()
{
  encoder1_read();
}
*/
void setup(void) {
  Serial1.begin(9600);
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, LOW);
  pinMode(BEEPER, OUTPUT);
  digitalWrite(BEEPER, LOW);
  pinMode(LED_LEVEL, OUTPUT);
  digitalWrite(LED_LEVEL, LOW);

  pinMode(PB12, INPUT);
  pinMode(PB13, INPUT);
  pinMode(PB14, INPUT);
  pinMode(PB15, INPUT);
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFlipMode(0);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_10x20_t_cyrillic);
  u8g2.setCursor(25, 60);
  u8g2.print("Загрузка");
  u8g2.drawFrame(14, 20, 106, 16);
  u8g2.sendBuffer();
  delay(100);
  for (int i = 10; i < 119; i+=5)
  {
    u8g2.drawBox(i, 20, 5, 16);
    delay(1);
    u8g2.sendBuffer();
  }
  iwdg_init(IWDG_PRE_256, 625); //156Hz - 4s
  delay(100);
  if(!digitalRead(PB12))
  {
    cap_min_level = 5;
  }
  if(!digitalRead(PB13))
  {
    cap_min_level = 10;
  }
  if(!digitalRead(PB14))
  {
    cap_min_level = 15;
  }
  if(!digitalRead(PB15))
  {
    cap_min_level = 20;
  }
  digitalWrite(BEEPER, HIGH);
  digitalWrite(LED_LEVEL, HIGH);
  delay(300);
  digitalWrite(BEEPER, LOW);
  digitalWrite(LED_LEVEL, LOW);

  pinMode(ENC_CLK, INPUT);
  pinMode(ENC_DATA, INPUT);
  encoderCount = 10000;

  u8g2.setDrawColor(1);
}

void oledDisplayUpdate()
{
  if (capacity_percent < 0) capacity_percent = 0;
  else if (capacity_percent > 100) capacity_percent = 100;
  static uint8_t prev_capasity = 1;
  if (prev_capasity != capacity_percent)
  {
    prev_capasity = capacity_percent;
    u8g2.clearBuffer();
    u8g2.setDrawColor(1);
    u8g2.setFont(u8g2_font_inb63_mn);
    if (bms_stype == 0)
    {
      u8g2.setCursor(10, 64);
      u8g2.print("--");
    }
    else
    {
      if (capacity_percent < 10)
      {
        u8g2.setCursor(40, 64);
        u8g2.print(String(capacity_percent));
      }
      else if (capacity_percent >= 10 && capacity_percent < 100)
      {
        u8g2.setCursor(15, 64);
        u8g2.print(String(capacity_percent));
      }
      else
      {
        u8g2.setCursor(0, 64);
        u8g2.print("1");
        u8g2.setCursor(34, 64);
        u8g2.print("0");
        u8g2.setCursor(82, 64);
        u8g2.print("0");
      }
    }
    u8g2.sendBuffer();
  }
}

void check_low_level()
{
  if (capacity_percent < cap_min_level)
  {
    static unsigned long blink_time = 0;
    static unsigned long beep_time = 0;
    if (millis() - beep_time > 8000)
    {
      beep_time = millis();
      digitalWrite(BEEPER, HIGH);
      delay(100);
      digitalWrite(BEEPER, LOW);
      delay(100);
      digitalWrite(BEEPER, HIGH);
      delay(100);
      digitalWrite(BEEPER, LOW);
      delay(100);
      digitalWrite(BEEPER, HIGH);
      delay(100);
      digitalWrite(BEEPER, LOW);
      digitalWrite(LED_LEVEL, !digitalRead(LED_LEVEL));
    }
    if (millis() - blink_time > 500)
    {
      blink_time = millis();
      digitalWrite(LED_LEVEL, !digitalRead(LED_LEVEL));
    }    
  }
  else {
    digitalWrite(BEEPER, LOW);
    digitalWrite(LED_LEVEL, LOW);
  }
}
uint8_t count_inbyte = 0;
void getSerialData()
{  
  while (Serial1.available() > 0)
  {
    count_inbyte = Serial1.readBytes(settDataIn, 13);
    if (count_inbyte > 40) count_inbyte = 0;
  }
  if (settDataIn[0] == 0xA5 && settDataIn[1] == 0x01 && count_inbyte > 10) {
    smart_bms = 1;
    bms_stype = BMS_SMART;
    capacity_percent = ((settDataIn[10] << 8) | settDataIn[11])/10;
    bms_err = 0;
  }
  else if (settDataIn[0] == 0xDD && count_inbyte > 23) {
    smart_bms = 0;
    bms_stype = BMS_JBD;
    capacity_percent = (uint16_t)settDataIn[23];
    bms_err = 0;
  }
}

void getEnc()
{
  u8g2.clearBuffer();
  u8g2.setCursor(0, 30);
  u8g2.print("pos: "+ (String)pos);
  //u8g2.setCursor(0, 60);
  //u8g2.print("dir: "+ (String)dir);
  u8g2.sendBuffer();
}



void loop(void) {
  iwdg_feed();
  /*getSerialData();

  if (HAL_GetTick() - bms_req_time > 1000)
  {
    if (!bms_detected)
    {
      count_inbyte = 0;
      digitalWrite(PC13, HIGH);
      delay(1);
      Serial1.write(bms_smart_request_msg, sizeof(bms_smart_request_msg));
      delay(1);
      Serial1.flush();
      digitalWrite(PC13, LOW);
      delay(1);
      bms_detected = 1;
    }
    else
    {
      if (smart_bms)
      {
        count_inbyte = 0;
        digitalWrite(PC13, HIGH);
        delay(1);
        Serial1.write(bms_smart_request_msg, sizeof(bms_smart_request_msg));               
        delay(1);
        Serial1.flush();
        digitalWrite(PC13, LOW);
        delay(1);
      }
      else
      {
        count_inbyte = 0;
        digitalWrite(PC13, HIGH);
        delay(1);
        Serial1.write(bms_jbd_request_msg, sizeof(bms_jbd_request_msg));
        delay(1);
        Serial1.flush();
        digitalWrite(PC13, LOW);
        delay(1);
      }
    }
    bms_req_time = HAL_GetTick();
    bms_err++;
    if (bms_err > 5)
    {
      capacity_percent = 0; //uncomment for realise
      bms_detected = 0;
      bms_stype = BMS_NONE;
    }
    //////////////////test/////////////////////
    //capacity_percent++;
    //if (capacity_percent > 100) capacity_percent = 0;
    //bms_stype = BMS_SMART;    
    ///////////////////////////////////////////
    oledDisplayUpdate();
  }
  if (cap_min_level > 0) check_low_level();
*/

  pos = encoderCount;
  encoder1_read();

  if (HAL_GetTick() - enc_timer > 1000)
  {
    enc_timer = HAL_GetTick();
    getEnc();
  }
}