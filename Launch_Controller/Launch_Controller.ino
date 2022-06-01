#define SD_ENABLE
#define SD_CRITICAL

#define BATTERY_ENABLE

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
#define SW6 7
#define SW7 9
#define SW8 10
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
  
  pinMode(SW7,INPUT_PULLUP);
  pinMode(SW8,INPUT_PULLUP);
  pinMode(SW9,INPUT_PULLUP);
  pinMode(SW10,INPUT_PULLUP);
  pinMode(SW11,INPUT_PULLUP);

  digitalWrite(SW1,HIGH);
}

void loop() {
  #ifdef BATTERY_ENABLED
    int value = analogRead(BATTV);
    float voltage = value / 1023 * BATTERY_NOMINAL; // Calculate voltage on pin
    if(voltage <= BATTERY_THRESHOLD){
      error("Low battery voltage!",true);
    }
  #endif
  if(digitalRead(SW7) == HIGH){
    if(!sw7_pressed){
      sw7_pressed = true;
      press(7);
    }
  }else if(sw7_pressed){
    sw7_pressed = false;
  }
  if(digitalRead(SW8) == HIGH){
    if(!sw8_pressed){
      sw8_pressed = true;
      press(8);
    }
  }else if(sw8_pressed){
    sw8_pressed = false;
  }
  if(digitalRead(SW9) == HIGH){
    if(!sw9_pressed){
      sw9_pressed = true;
      press(9);
    }
  }else if(sw9_pressed){
    sw9_pressed = false;
  }
  if(digitalRead(SW10) == HIGH){
    if(!sw10_pressed){
      sw10_pressed = true;
      press(10);
    }
  }else if(sw10_pressed){
    sw10_pressed = false;
  }
  if(digitalRead(SW11) == HIGH){
    if(!sw11_pressed){
      sw11_pressed = true;
      press(11);
    }
  }else if(sw11_pressed){
    sw11_pressed = false;
  }
}
void error(String error, boolean critical){
  Serial.println(error); // TODO: implement sd card logging?
  
  while(critical){
    digitalWrite(SW6,!digitalRead(SW6));
    delay(1000);
  }
}
void press(int btn){
  //TODO: Implement menus
}
