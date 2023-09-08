//#include <Arduino.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <iwdg.h>
//#include <RotaryEncoder.h>
//====u8x8=====
//#include <U8x8lib.h>
//U8X8_SSD1306_128X64_NONAME_4W_SW_SPI u8x8(/* clock=*/ PA5, /* data=*/ PA7, /* cs=*/ PB1, /* dc=*/ PA6, /* reset=*/ PB0);
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_4W_SW_SPI u8g2(U8G2_R0, /* clock=*/ PA5, /* data=*/ PA7, /* cs=*/ PB1, /* dc=*/ PA6, /* reset=*/ PB0);
//====u8x8=====

//#define NEW_SCREEN

#ifdef NEW_SCREEN
  HardwareSerial Serial1(PA10, PA9);
#endif
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

#ifdef NEW_SCREEN
  #define ENC_CLK PB10
  #define ENC_DATA PB11
#else
  #define ENC_CLK PA2
  #define ENC_DATA PA3
#endif

volatile int32_t encoderCount;
volatile int e_c = 0;
volatile int e_d = 0;

void encoder1_read(void)
{
  volatile static uint8_t ABs = 0;
  ABs = (ABs << 2) & 0x0f; //left 2 bits now contain the previous AB key read-out;
  //ABs |= (digitalRead(ENC_CLK) << 1) | digitalRead(ENC_DATA);
  //ABs |= ((GPIOB->IDR & (1 << 10)) << 1) | (GPIOB->IDR & (1 << 11));
  GPIOB->IDR & (1 << 10) ? (e_c = 0) : (e_c = 1);
  GPIOB->IDR & (1 << 11) ? (e_d = 0) : (e_d = 1);
  ABs |= (e_c << 1) | e_d;
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

void pinAInterrupt()
{
	//When pin A's wave is detected...

	if (digitalRead(ENC_DATA) == 0) //if B is LOW, it means that pin A's wave occured first -> CW rotation occured
	{
		encoderCount++; //increase value
		//Serial.println(numberofclicks); //do not use delays or prints in the final code, use it only for debugging/developing
	}
	else //if B is HIGH, it means that pin B's wave occured first. So, when pin A has a rising edge, pin B is alreadi high -> CCW rotation
	{
		encoderCount--; //decrease value
		//Serial.println(numberofclicks);
	}
}

void setup(void) {
  #ifdef NEW_SCREEN
    Serial1.begin(9600);
    pinMode(PC13, OUTPUT);
    digitalWrite(PC13, LOW);

    pinMode(PB12, INPUT);
    pinMode(PB13, INPUT);
    pinMode(PB14, INPUT);
    pinMode(PB15, INPUT);
  #endif
  u8g2.begin();
  u8g2.enableUTF8Print();
  u8g2.setFlipMode(0);
  u8g2.clearBuffer();
  //u8g2.setFont(u8g2_font_10x20_t_cyrillic);
  u8g2.setFont(u8g2_font_profont29_mr);
  u8g2.setCursor(5, 60);
  u8g2.print("Loading");
  u8g2.drawFrame(14, 20, 106, 16);
  u8g2.sendBuffer();
  delay(5);
  /*for (int i = 10; i < 119; i+=5)
  {
    u8g2.drawBox(i, 20, 5, 16);
    delay(1);
    u8g2.sendBuffer();
  }*/
  u8g2.drawBox(10, 20, 110, 16);
  u8g2.sendBuffer();
  //iwdg_init(IWDG_PRE_256, 625); //156Hz - 4s
  delay(30);

  //pinMode(ENC_CLK, INPUT);
  //pinMode(ENC_DATA, INPUT);

  //pinMode(ENC_CLK, INPUT_PULLDOWN);
  //pinMode(ENC_DATA, INPUT_PULLDOWN);
  encoderCount = 0;

  u8g2.setDrawColor(1);
  
  pinMode(ENC_CLK, INPUT_PULLUP);
	pinMode(ENC_DATA, INPUT_PULLUP);

	attachInterrupt(digitalPinToInterrupt(ENC_CLK), pinAInterrupt, RISING);
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
  u8g2.setCursor(10, 40);
  u8g2.print((String)encoderCount);
  //u8g2.setCursor(0, 60);
  //u8g2.print("dir: "+ (String)dir);
  u8g2.sendBuffer();
}



void loop(void) {
  //iwdg_feed();
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

  //pos = encoderCount;
  //encoder1_read();

  if (HAL_GetTick() - enc_timer > 50)
  {
    enc_timer = HAL_GetTick();
    getEnc();
  }
}
