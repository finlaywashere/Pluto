#define BME_ENABLE
#define BME_CRITICAL
#define SD_ENABLE
#define SD_CRITICAL
#define MPU_ENABLE
#define MPU_CRITICAL

#define PARACHUTE_ALT 50
#define PARACHUTE_ARM 75

#define BME_ADDR 0x76

#include <SPI.h>
#include <Wire.h>

// Include all the libraries we need
#ifdef BME_ENABLE
  #include <Adafruit_BME280.h>
#endif

#ifdef SD_ENABLE
  #include <SdFat.h>
#endif

#ifdef MPU_ENABLE
  //#include "I2Cdev.h"
  //#include "MPU6050.h"
  #include "MPU6050_6Axis_MotionApps20.h"
#endif

// These will be different for different MPU6050's
#define MPU_ACCEL_X -1865
#define MPU_ACCEL_Y 883
#define MPU_ACCEL_Z 5511
#define MPU_GYRO_X 74
#define MPU_GYRO_Y 74
#define MPU_GYRO_Z 35

#define SEA_LEVEL_PRESSURE 1015 // Sea level pressure in hPa

#include "communication.h"

#define SERVO1 3
#define SERVO2 5
#define SERVO3 6
#define SERVO4 9
#define SERVO5 10
#define SERVO6 11 // Same as MOSI pin

#define MOTOR_ENABLE 7
#define LED1 4
#define MPU_INT_PIN 8
#define SD_CS A3

#define INT2 A0
#define INT3 A1
#define INT4 A2


#define MESSAGE_RETRANSMIT_TIME 1000

#ifdef BME_ENABLE
  boolean bme_error = false;
  Adafruit_BME280 bme;
  float base_altitude = -10000;
  boolean parachute_arm = false;
#endif

#ifdef SD_ENABLE
  boolean sd_error = false;
  SdFat card;
  SdFile data_file;
#endif

#ifdef MPU_ENABLE
  boolean mpu_error = false;
  MPU6050 mpu;
  uint16_t mpu_packet_size;
  boolean dmpReady = false;
  volatile boolean mpuInterrupt = false;
  uint16_t fifoCount;
  uint8_t fifoBuffer[64];
  Quaternion q;
  VectorFloat gravity;
  float ypr[3];
  VectorInt16 aa;
  VectorInt16 aaReal;
  VectorInt16 aaWorld;
  uint8_t mpuIntStatus;
#endif

boolean setup_mode = true;
int loop_count = 0;

long last_ping = 0;
long last_rec_ping = 0;
long errors = 0; // Communication errors

void setup() {
  // Initialize serial connection with sister board
  Serial.begin(115200);
  Wire.begin();

  pinMode(SERVO1,OUTPUT);
  pinMode(SERVO2,OUTPUT);
  pinMode(SERVO3,OUTPUT);
  pinMode(SERVO4,OUTPUT);
  pinMode(SERVO5,OUTPUT);
  pinMode(SERVO6,OUTPUT);

  pinMode(MOTOR_ENABLE,OUTPUT);
  digitalWrite(MOTOR_ENABLE,LOW);

  pinMode(LED1,OUTPUT);
  pinMode(INT1,INPUT);
  pinMode(SD_CS,OUTPUT);
  
  #ifdef SD_ENABLE
    if(!card.begin(SD_CS)){
      sd_error = true;
      digitalWrite(LED1,HIGH);
      #ifdef SD_CRITICAL
        message_status(CRITICAL,CODE_HW_SD);
        while(1){
          delay(MESSAGE_RETRANSMIT_TIME);
          message_status(CRITICAL,CODE_HW_SD);
        }
      #else
        message_status(ERROR,CODE_HW_SD);
      #endif
    }
    data_file.open("flight.dat",O_CREAT | O_EXCL | O_WRITE);
    if(!data_file){
      sd_error = true;
      digitalWrite(LED1,HIGH);
      #ifdef SD_CRITICAL
        message_status(CRITICAL,CODE_HW_SD);
        while(1){
          delay(MESSAGE_RETRANSMIT_TIME);
          message_status(CRITICAL,CODE_HW_SD);
        }
      #else
        message_status(ERROR,CODE_HW_SD);
      #endif
    }
  #endif
  #ifdef BME_ENABLE
    if(!bme.begin(BME_ADDR)){
      digitalWrite(LED1,HIGH);
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
  #ifdef MPU_ENABLE
    mpu.initialize();
    if(!mpu.testConnection()){
      digitalWrite(LED1,HIGH);
      mpu_error = true;
      #ifdef MPU_CRITICAL
        message_status(CRITICAL,CODE_HW_MPU);
        while(1){
          delay(MESSAGE_RETRANSMIT_TIME);
          message_status(CRITICAL,CODE_HW_MPU);
        }
      #else
        message_status(ERROR,CODE_HW_MPU);
      #endif
    }
    uint8_t status = mpu.dmpInitialize();
    mpu.setXAccelOffset(MPU_ACCEL_X);
    mpu.setYAccelOffset(MPU_ACCEL_Y);
    mpu.setZAccelOffset(MPU_ACCEL_Z);
    mpu.setXGyroOffset(MPU_GYRO_X);
    mpu.setYGyroOffset(MPU_GYRO_Y);
    mpu.setZGyroOffset(MPU_GYRO_Z);
    if(status == 0){
      mpu.setDMPEnabled(true);
      attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady ,RISING);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
      mpu_packet_size = mpu.dmpGetFIFOPacketSize();
    }else{
      // Error
      digitalWrite(LED1,HIGH);
      mpu_error = true;
      #ifdef MPU_CRITICAL
        message_status(CRITICAL,CODE_HW_MPU);
        while(1){
          delay(MESSAGE_RETRANSMIT_TIME);
          message_status(CRITICAL,CODE_HW_MPU);
        }
      #else
        message_status(ERROR,CODE_HW_MPU);
      #endif
    }
  #endif
  message_status(STARTUP,0);
}
#ifdef MPU_ENABLE
  void dmpDataReady(){
    mpuInterrupt = true;
  }
#endif

void loop() {
  if(millis() - last_ping > 1000){
    message_status(PING,0);
    last_ping = millis();
  }
  if(last_rec_ping != 0 && millis() - last_rec_ping > 3000){
    digitalWrite(LED1,HIGH);
    message_status(CRITICAL,CODE_COMMS);
    while(1){
      delay(MESSAGE_RETRANSMIT_TIME);
      message_status(CRITICAL,CODE_COMMS);
    }
  }
  if(last_rec_ping == 0 && millis() > 10000){
    // Safety controller has not been initialized and we've been on for 10s
    digitalWrite(LED1,HIGH);
    message_status(CRITICAL,CODE_COMMS);
    while(1){
      delay(MESSAGE_RETRANSMIT_TIME);
      message_status(CRITICAL,CODE_COMMS);
    }
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
          digitalWrite(LED1,HIGH);
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
          message_bme(temp,pres,alt,humid);
        }
        float rel_alt = alt-base_altitude;
        if(!parachute_arm && rel_alt > PARACHUTE_ARM){
          // Arm parachute ejection
          parachute_arm = true;
        }else if(parachute_arm && rel_alt < PARACHUTE_ALT){
          // Eject parachute
          message_status(ACTION,CODE_EJECT_PARACHUTE);
        }
      }
    #endif
    #ifdef MPU_ENABLE
      if(mpuInterrupt && !mpu_error && mpuIntStatus & 0x02){
        while(fifoCount < mpu_packet_size) // Should be a short wait
          fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer,mpu_packet_size);
        fifoCount -= mpu_packet_size;
        
        mpu.dmpGetQuaternion(&q,fifoBuffer);
        mpu.dmpGetGravity(&gravity,&q);
        mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);

        float yaw = ypr[0] * 180/M_PI;
        float pitch = ypr[1] * 180/M_PI;
        float roll = ypr[2] * 180/M_PI;

        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        
        float x = aaReal.x;
        float y = aaReal.y;
        float z = aaReal.z;
        
        message_mpu(yaw,pitch,roll,x,y,z);
        mpu.resetFIFO();
        mpuInterrupt = false;
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
        digitalWrite(LED1,HIGH);
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
        #ifdef SD_ENABLE
          write_sd_data(message);
        #endif
        if(message[0] == CRITICAL){
          digitalWrite(LED1,HIGH);
          // An error has occurred with the safety computer, enter safe mode
          // TODO: Figure out what to do
          while(1); // Just lock up?
        }else if(message[0] == CHECKSUM){
          errors++;
        }
      }else{
        errors++;
        message_status(CHECKSUM,0);
      }
    }
    delay(250);
}
void message_status(byte code, byte reason){
  byte data[16];
  data[0] = code;
  data[1] = reason;
  for(int i = 2; i < 16; i++){
    data[i] = 0;
  }
  write_data(data);
}
#ifdef SD_ENABLE
  void write_sd_data(byte *data){
    if(!sd_error){
      if(data_file.write(data,16) != 16){
        sd_error = true;
        digitalWrite(LED1,HIGH);
        #ifdef SD_CRITICAL
          message_status(CRITICAL,CODE_HW_SD);
          while(1){
            delay(MESSAGE_RETRANSMIT_TIME);
            message_status(CRITICAL,CODE_HW_SD);
          }
        #else
          message_status(ERROR,CODE_HW_SD);
        #endif
      }
    }
  }
#endif
void write_data(byte *data){
  data[14] = SOURCE_FC; // Mark source
  // Calculate checksum
  for(int i = 0; i < 15; i++){
    data[15] += data[i]; // Checksum is sum of bytes
  }
  #ifdef SD_ENABLE
    write_sd_data(data);
  #endif
  last_ping = millis();
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
#ifdef MPU_ENABLE
  void message_mpu(float yaw, float pitch, float roll, float x, float y, float z){
    byte data[16];
    for(int i = 0; i < 16; i++)
      data[i] = 0;
    data[0] = DATA;
    data[1] = DATA_MPU;
    byte *yawB = (byte*) &yaw;
    byte *pitchB = (byte*) &pitch;
    byte *rollB = (byte*) &roll;
    byte *xB = (byte*) &x;
    byte *yB = (byte*) &y;
    byte *zB = (byte*) &z;
    for(int i = 0; i < 12; i++){
      int index = i + 2;
      int var = i / 4;
      int elem = i % 4;
      if(var == 0)
        data[index] = yawB[elem];
      else if(var == 1)
        data[index] = pitchB[elem];
      else
        data[index] = rollB[elem];
    }
    write_data(data);
    data[1] = DATA_MPU_EXT;
    for(int i = 0; i < 12; i++){
      int index = i + 2;
      int var = i / 4;
      int elem = i % 4;
      if(var == 0)
        data[index] = xB[elem];
      else if(var == 1)
        data[index] = yB[elem];
      else
        data[index] = zB[elem];
    }
    write_data(data);
  }
#endif
