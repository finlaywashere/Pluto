#include "communication.h"
#include <LiquidCrystal_I2C.h>

//#define BATTERY_ENABLE

#define BATTERY_NOMINAL 4.2
#define BATTERY_THRESHOLD 3.3

#define SD_CS 6

#define RADIO_CE 8
#define RADIO_CSN A2

#define BATTV A3

#define SW1 2
#define SW2 3
#define SW3 4
#define SW4 5
#define SW6 7 // Buzzer
#define SW7 9 // Power
#define SW8 10 // Button 1
#define SW9 A6 // Input only
#define SW10 A7 // Input only
#define SW11 A1

#define IRQ1 A0

#ifdef SD_ENABLE
  #include <SdFat.h>
  boolean sd_error = false;
  SdFat card;
  SdFile data_file;
#endif

boolean sw7_pressed = false;
boolean sw8_pressed = false;
boolean sw9_pressed = false;
boolean sw10_pressed = false;
boolean sw11_pressed = false;

long rocket_last_rec = 0;
long last_query = 0;

boolean rocket_ready = false;

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  Serial.begin(115200);
  #ifdef SD_ENABLE
    if(!card.begin(SD_CS)){
      sd_error = true;
      boolean crit = false;
      #ifdef SD_CRITICAL
        crit = true;
      #endif
      error("SD card error",crit);
    }
    data_file.open("control.dat",O_CREAT | O_EXCL | O_WRITE);
    if(!data_file){
      sd_error = true;
      boolean crit = false;
      #ifdef SD_CRITICAL
        crit = true;
      #endif
      error("SD file error!",crit);
    }
  #endif
  // Outputs drive LEDs, inputs are for push buttons
  pinMode(SW1,OUTPUT);
  pinMode(SW2,OUTPUT);
  pinMode(SW3,OUTPUT);
  pinMode(SW4,OUTPUT);
  pinMode(SW6,OUTPUT);
  digitalWrite(SW6,HIGH);
  pinMode(SW7,OUTPUT);
  digitalWrite(SW7,HIGH);
  pinMode(SW8,INPUT_PULLUP);
  pinMode(SW9,INPUT_PULLUP);
  pinMode(SW10,INPUT_PULLUP);
  pinMode(SW11,INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0,0);
  lcd.print("!NOT CONNECTED! ");
}

void loop() {
  #ifdef BATTERY_ENABLED
    int value = analogRead(BATTV);
    float voltage = value / 1023 * BATTERY_NOMINAL; // Calculate voltage on pin
    if(voltage <= BATTERY_THRESHOLD){
      error("Low battery voltage!",true);
    }
  #endif
  if(digitalRead(SW8) == LOW){
    if(!sw8_pressed){
      sw8_pressed = true;
      press(8);
    }
  }else if(sw8_pressed){
    sw8_pressed = false;
  }
  if(digitalRead(SW9) == LOW){
    if(!sw9_pressed){
      sw9_pressed = true;
      press(9);
    }
  }else if(sw9_pressed){
    sw9_pressed = false;
  }
  if(digitalRead(SW10) == LOW){
    if(!sw10_pressed){
      sw10_pressed = true;
      press(10);
    }
  }else if(sw10_pressed){
    sw10_pressed = false;
  }
  if(digitalRead(SW11) == LOW){
    if(!sw11_pressed){
      sw11_pressed = true;
      press(11);
    }
  }else if(sw11_pressed){
    sw11_pressed = false;
  }
  byte message[17];
  if(rocket_last_rec != 0 && millis() - rocket_last_rec > 5000){
    lcd.setCursor(0,0);
    lcd.print("!TIMEOUT!       ");
  }
  if(message_retrieve(message)){
    rocket_last_rec = millis();
    lcd.setCursor(0,0);
    lcd.print("CONNECTED       ");
    if(message[1] == CRITICAL){
      rocket_ready = false;
      lcd.setCursor(0,0);
      lcd.print("!!ROCKET ERROR!!");
      while(1);
    }else if(message[1] == INFO){
      if(message[2] == QUERY_LAUNCH){
        rocket_ready = message[3] == STATUS_READY;
      }
    }
  }
  if(millis() - last_query > 2500){
    last_query = millis();
    message_query(SOURCE_SC,QUERY_LAUNCH);
  }
  delay(50);
}
void error(String error, boolean critical){
  Serial.println(error); // TODO: implement sd card logging?
  
  while(critical){
    digitalWrite(SW6,!digitalRead(SW6));
    delay(1000);
  }
}
void press(int btn){
  if(btn == 8){
    // Launch
    launch();
  }
}
void launch(){
  if(!rocket_ready)
    return;
  // Rocket is ready for launch
  message_status(STATUS,STATUS_LAUNCH); // This will disable comms with the rocket
  delay(50); // Wait to ensure its received
  digitalWrite(SW6,LOW);
  delay(1000);
  digitalWrite(SW7,LOW);
  delay(20000);
  digitalWrite(SW6,HIGH);
  digitalWrite(SW7,HIGH);
  rocket_ready = false;
}
void message_status(byte code, byte reason){
  byte data[17];
  for(int i = 0; i < 17; i++)
    data[i] = 0;
  data[1] = code;
  data[2] = reason;
  write_data(data);
}
void write_data(byte *data){
  write_data(data,true,true);
}
void write_data(byte *data, boolean ser, boolean lau){
  data[0] = 0xFF;
  data[15] = SOURCE_LC; // Mark source
  // Calculate checksum
  int checksum = 0;
  for(int i = 0; i < 16; i++){
    checksum += data[i]; // Checksum is sum of bytes
  }
  checksum %= 255;
  data[16] = checksum;
  if(ser)
    Serial.write(data,17);
  #ifdef LAUNCHER_COMMS
    if(lau)
      launcher_serial.write(data,17);
  #endif
}
boolean message_available(){
  while(Serial.available() && Serial.peek() != 0xFF){
    Serial.read(); // Clear garbage data
  }
  return Serial.available() >= 17; // 16 byte data frame
}
boolean message_retrieve(byte *buffer){
  byte checksum = 0;
  for(int i = 0; i < 17; i++){
    buffer[i] = Serial.read();
    if(i != 16)
      checksum += buffer[i];
  }
  checksum %= 255;
  if(checksum == buffer[16])
    return true;
  return false;
}
void message_query(int target, int code){
  byte data[17];
  for(int i = 0; i < 17; i++)
    data[i] = 0;
  data[1] = QUERY;
  data[2] = target;
  data[3] = code;
  write_data(data);
}
