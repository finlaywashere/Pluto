//#define BME_ENABLE
//#define BME_CRITICAL
#define PARACHUTE_ENABLE

#define BATTERY_ENABLE

#define BATTERY_NOMINAL 4.2
#define BATTERY_THRESHOLD 3.3

#define PARACHUTE_CLOSED_POS 180
#define PARACHUTE_OPEN_POS 90

#ifdef PARACHUTE_ENABLE
  #include <Servo.h>
#endif
#ifdef BME_ENABLE
  #include <Adafruit_BME280.h>
#endif

#define SEA_LEVEL_PRESSURE 1015 // Sea level pressure in hPa

// Include all our libraries
#include "communication.h"

#define AU1 2
#define AU2 7
#define AU3 4
#define AU4 A7 // This was a dumb design issue and this pin is input only :/
#define AU5 A1

#define SERVO1 3
#define SERVO2 5
#define SERVO3 6
#define SERVO4 9
#define SERVO5 10
#define SERVO6 11

#define MOTOR_ENABLE A3
#define BATTV A6

// Radio pins
#define RCE 8
#define RCSN A2
#define IRQ1 A0

#define MESSAGE_RETRANSMIT_TIME 1000

#define BME_ADDR 0x76

#ifdef BME_ENABLE
  boolean bme_error = false;
  Adafruit_BME280 bme;
  float base_altitude = -10000;
#endif

#ifdef PARACHUTE_ENABLE
  Servo parachute_servo;
  boolean parachute_arm = false;
#endif

boolean setup_mode = true;
int loop_count = 0;
long last_rec_ping = 0;
long last_ping = 0;

void setup() {
  // Initialize serial connection with sister board
  Serial.begin(115200);

  // Setup safety authorization pins
  pinMode(AU1,OUTPUT);
  digitalWrite(AU1,LOW);
  pinMode(AU2,OUTPUT);
  digitalWrite(AU2,LOW);
  pinMode(AU3,OUTPUT);
  digitalWrite(AU3,LOW);
  //pinMode(AU4,OUTPUT); // This pin will not work, hardware issue
  //digitalWrite(AU4,LOW);
  pinMode(AU5,OUTPUT);
  digitalWrite(AU5,LOW);

  pinMode(SERVO1,OUTPUT);
  pinMode(SERVO2,OUTPUT);
  pinMode(SERVO3,OUTPUT);
  pinMode(SERVO4,OUTPUT);
  pinMode(SERVO5,OUTPUT);
  pinMode(SERVO6,OUTPUT);
  
  pinMode(MOTOR_ENABLE,OUTPUT);
  digitalWrite(MOTOR_ENABLE,LOW);

  pinMode(BATTV,INPUT);

  pinMode(RCE,OUTPUT);
  pinMode(RCSN,OUTPUT);
  pinMode(IRQ1,INPUT);
  #ifdef BME_ENABLE
    if(!bme.begin(BME_ADDR)){
      bme_error = true;
      #ifdef BME_CRITICAL
        message_status(CRITICAL,CODE_HW_BME);
        while(1){
          delay(MESSAGE_RETRANSMIT_TIME);
          message_status(CRITICAL,CODE_HW_BME);
        }
      #else
        message_status(ERROR,CODE_HW_BME);
      #endif
    }
  #endif
  #ifdef PARACHUTE_ENABLE
    delay(2000); // Give 2s before locking
    parachute_servo.attach(SERVO1);
    lock_parachute();
  #endif
}

void loop() {
  #ifdef BATTERY_ENABLE
    int value = analogRead(BATTV);
    float voltage = value / 1023 * BATTERY_NOMINAL; // Calculate voltage on pin
    if(voltage <= BATTERY_THRESHOLD){
      message_status(CRITICAL,CODE_BATTERY);
      #ifdef PARACHUTE_ENABLED
        eject_parachute();
      #endif
      while(1){
        delay(MESSAGE_RETRANSMIT_TIME);
        message_status(CRITICAL,CODE_BATTERY);
      }
    }
  #endif
  if(millis() - last_ping > 1000){
    message_status(PING,0);
    last_ping = millis();
  }
  if(last_rec_ping != 0 && millis() - last_rec_ping > 3000){
    message_status(CRITICAL,CODE_COMMS);
    #ifdef PARACHUTE_ENABLED
      eject_parachute();
    #endif
    while(1){
      delay(MESSAGE_RETRANSMIT_TIME);
      message_status(CRITICAL,CODE_COMMS);
    }
  }
  if(last_rec_ping == 0 && millis() > 10000){
    // Flight controller has not been initialized and we've been on for 10s
    message_status(CRITICAL,CODE_COMMS);
    while(1){
      delay(MESSAGE_RETRANSMIT_TIME);
      message_status(CRITICAL,CODE_COMMS);
    }
    while(1);
  }
  if(!setup_mode){
    #ifdef BME_ENABLE
      if(!bme_error){
        float temp = bme.readTemperature();
        float pres = bme.readPressure();
        float alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
        float humid = bme.readHumidity();
        if(temp == NAN || pres == NAN || pres == 0 || alt == NAN || humid == NAN){
          bme_error = true;
          #ifdef BME_CRITICAL
            message_status(CRITICAL,CODE_HW_BME);
            last_ping = millis();
            #ifdef PARACHUTE_ENABLED
              eject_parachute();
            #endif
            while(1){
              delay(MESSAGE_RETRANSMIT_TIME);
              message_status(CRITICAL,CODE_HW_BME);
            }
          #else
            message_status(ERROR,CODE_HW_BME);
            last_ping = millis();
          #endif
        }else{
          message_bme(temp,pres,alt,humid);
          last_ping = millis();
        }
        float rel_alt = alt-base_altitude;
        #ifdef PARACHUTE_ENABLED
          if(!parachute_arm && rel_alt > PARACHUTE_ARM){
            // Arm parachute ejection
            parachute_arm = true;
          }else if(parachute_arm && rel_alt < PARACHUTE_ALT){
            // Eject parachute message
            message_status(ACTION,CODE_EJECT_PARACHUTE);
            eject_parachute();
          }
        #endif
      }
    #endif
  }else{
    if(loop_count > 10){
      setup_mode = false;
    }
    #ifdef BME_ENABLE
      float alt = bme.readAltitude(SEA_LEVEL_PRESSURE);
      if(alt == NAN){
        bme_error = true;
        #ifdef BME_CRITICAL
          message_status(CRITICAL,CODE_HW_BME);
          while(1){
              delay(MESSAGE_RETRANSMIT_TIME);
              message_status(CRITICAL,CODE_HW_BME);
            }
        #else
          message_status(ERROR,CODE_HW_BME);
        #endif
      }else{
        if(base_altitude < alt){
          base_altitude = alt; // Use max detected altitude as base altitude to be safe
        }
      }
    #endif
    loop_count++;
    delay(750);
  }
  if(message_available()){
      byte message[16];
      if(message_retrieve(message)){ // Make sure checksum is valid
        last_rec_ping = millis();
        if(message[0] == CRITICAL){
          // An error has occurred with the flight computer, enter safe mode
          // Eject parachute immediately upon error
          #ifdef PARACHUTE_ENABLED
            eject_parachute();
          #endif
        }else if(message[0] == ACTION){
          #ifdef PARACHUTE_ENABLED
            if(message[1] == CODE_EJECT_PARACHUTE)
              eject_parachute();
          #endif
        }
      }
    }
    delay(250);
}
#ifdef PARACHUTE_ENABLE
  void eject_parachute(){
    digitalWrite(MOTOR_ENABLE,HIGH);
    parachute_servo.write(PARACHUTE_OPEN_POS);
    delay(500);
    digitalWrite(MOTOR_ENABLE,LOW);
  }
  void lock_parachute(){
    digitalWrite(MOTOR_ENABLE,HIGH);
    parachute_servo.write(PARACHUTE_CLOSED_POS);
    delay(500);
    digitalWrite(MOTOR_ENABLE,LOW);
  }
#endif
void message_status(byte code, byte reason){
  byte data[16];
  data[0] = code;
  data[1] = reason;
  write_data(data);
}
void write_data(byte *data){
  data[14] = SOURCE_SC; // Mark source
  // Calculate checksum
  for(int i = 0; i < 15; i++){
    data[15] += data[i]; // Checksum is sum of bytes
  }
  Serial.write(data,16);
}
boolean message_available(){
  return Serial.available() >= 16; // 16 byte data frame
}
boolean message_retrieve(byte *buffer){
  byte checksum = 0;
  for(int i = 0; i < 16; i++){
    buffer[i] = Serial.read();
    if(i != 15)
      checksum += buffer[i];
  }
  if(checksum == buffer[15])
    return true;
  return false;
}
#ifdef BME_ENABLE
  void message_bme(float temp, float pres, float alt, float humid){
    // Floats are made of 4 bytes
    byte *tempB = (byte*) &temp;
    byte *presB = (byte*) &pres;
    byte *altB = (byte*) &alt;
    
    byte data[16];
    for(int i = 0; i < 16; i++)
      data[i] = 0;
    data[0] = DATA;
    data[1] = DATA_BME;
    for(int i = 0; i < 12; i++){
      int index = i % 4;
      int var = i / 4;
      if(var == 0)
        data[i+2] = tempB[index];
      else if(var == 1)
        data[i+2] = presB[index];
      else
        data[i+2] = altB[index];
    }
    write_data(data);
    data[1] = DATA_BME_EXT;
    byte *humidB = (byte*) &humid;
    for(int i = 0; i < 12; i++){
      if(i < 4)
        data[i+2] = humidB[i];
      else
        data[i+2] = 0;
    }
    write_data(data);
  }
#endif
