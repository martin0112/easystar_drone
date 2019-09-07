#include <HardwareSerial.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <MPU9250_asukiaaa.h>
#include <I2CEEPROM.h>

/*Pinout
RX2 16
TX2 17
SDA 32
SCL 33
Servo_v 14
Servo_h 15
Throttle 13
dummy servo 18,19
*/
//cycle time
long cycle_time=8; 
float cycle_time_filter_ratio=0.1;

//Debugging
bool debug=false;

//IMU accel
#define SDA_PIN 32
#define SCL_PIN 33
MPU9250 mySensor;
double yaw,pitch,roll,dir;
int dpitch_dt, droll_dt,dv_dt;
float filter_coeff=0.1;

//Flugschreiber
#define CHIP_ADDRESS 0x50 // Address of EEPROM chip (24LC256->0x50)
#define EEPROM_BYTES 32768 // Number of bytes in EEPROM chip
#define t_cycle_recorder 0.1 
#define amount_data 9
int last_record=0;
int16_t no_records=0;
String betrieb="nichts";
I2CEEPROM i2c_eeprom(CHIP_ADDRESS); // Create I2C EEPROM instance
unsigned int current_address = 0;
unsigned int first_write_time=0;

//HC12 radio
#define RXD2 16
#define TXD2 17

//Aztopilot
String autopilot_mode="MAN";

//FBW
bool switch_to_fbw=false;

float P_roll= 0;
float Ti_roll=99999;
float D_roll=0;
float CV_roll_neutral=90;
float CV_roll;

int roll_sp=0;
float error_roll_int=0;
float derror_roll_dt=0;
int error_roll_alt;

float P_pitch= 1;
float Ti_pitch=999999999;
float D_pitch=0;
float CV_pitch_neutral=90;
float CV_pitch;

int pitch_sp=10;
float error_pitch_int=0;
float derror_pitch_dt=0;
int error_pitch_alt;

//servos
Servo dummy_1; //Summyservo wegen timern
#define dum_pin_1 18
Servo dummy_2; //Summyservo wegen timern
#define dum_pin_2 19

Servo myservo_v;  // Höhenruder
#define min_ver 10
#define max_ver 170
#define servoPin_v  15
int servo_angle_vertical=0;


Servo myservo_h;  // Seitenruder
#define min_hor 10
#define max_hor 170
#define servoPin_h  14
int servo_angle_horiz=0;

//Schub
#define min_throttle 0
#define max_throttle 100
#define throttle_pin  13
#define freq  50
#define ledChannel  0
#define resolution  8



//KONTROLLE HC12
int control_radio[4]={0,0,50,50}; //button, thrust, horizontal , vertical 
HardwareSerial  HC12(2);   



void setup() {

  //I2c
    Wire.begin(SDA_PIN,SCL_PIN); //SDA SCL


  //Accel
  mySensor.setWire(&Wire);
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag();


  
  //Serial
  Serial.begin(115200); //Debugging
  HC12.begin(9600, SERIAL_8N1, RXD2, TXD2); //HC12
  Serial.println("Flugzeug Start");
  // Servo
  dummy_1.attach(dum_pin_1);
  dummy_2.attach(dum_pin_2);

  myservo_v.setPeriodHertz(50);    // standard 50 hz servo
  myservo_v.attach(servoPin_v); // attaches the servo on pin 18 to the servo object
  myservo_h.setPeriodHertz(50);    // standard 50 hz servo
  myservo_h.attach(servoPin_h); // attaches the servo on pin 18 to the servo object
  //Servo_engine_control();



  //Trottle
  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(throttle_pin, ledChannel);
 

}

void loop() {

long start_time=micros();

receive_hc12();
Servo_engine_control();
imu_read();
flugschreiber();
debugging();
yield();
cycle_time=(micros()-start_time)*cycle_time_filter_ratio+cycle_time-cycle_time_filter_ratio*cycle_time;
//Serial.print(cycle_time);
//Serial.print(";");
}

void imu_read()
{
  mySensor.magUpdate();
  int mX = mySensor.magX();
  int mY = mySensor.magY();
  int mZ = mySensor.magZ();
  dir = mySensor.magHorizDirection();
/*
 Serial.print(mX);
 Serial.print(";");
 Serial.print(mY);
 Serial.print(";");
 Serial.println(mZ);
*/ 
  
  mySensor.gyroUpdate();
  int gX = mySensor.gyroX();
  int gY = mySensor.gyroY();
  int gZ = mySensor.gyroZ();
  
  mySensor.accelUpdate();
  float accelX = mySensor.accelX();
  float accelY = mySensor.accelY();
  float accelZ = mySensor.accelZ();
  
  float pitch_temp =-atan2 (accelY ,( sqrt ((accelX * accelX) + (accelZ * accelZ))))/3.14*180;
  pitch_temp=accelZ<0?pitch_temp:180-pitch_temp;
  pitch_temp=pitch_temp<180?pitch_temp:pitch_temp-360;
  
  
  float roll_temp = atan2(-accelX ,( sqrt((accelY * accelY) + (accelZ * accelZ))))/3.14*180;
  roll_temp=accelZ<0?roll_temp:180-roll_temp;
  roll_temp=roll_temp<180?roll_temp:roll_temp-360;
  if (debug)
  {
    Serial.print(" Accel X: ");
    Serial.print(accelX);
    Serial.print(" Accel Y: ");
    Serial.print(accelX);
    Serial.print(" Accel Z: ");
    Serial.print(accelZ);

    }



  
  pitch=filter_coeff*(gX*cycle_time/1000.0/1000.0+pitch)+(1-filter_coeff)*pitch_temp;
  roll=filter_coeff*(gY*cycle_time/1000.0/1000.0+roll)+(1-filter_coeff)*roll_temp;
  dpitch_dt=gX; 
  droll_dt=gY;
  dv_dt=gZ;
  //Serial.println(roll); //Drehung um X Achse
  

  }

void flugschreiber()
{
 if (Serial.available())
 {
  char oper=Serial.read();
  if (oper=='L')
  {
    betrieb="lesen";
   }
   if (oper=='S')
  {
    betrieb="schreiben";
   }
   if (oper=='N')
  {
    betrieb="nichts";
    debug=false;
   }
  if (oper=='D') 
  {
    debug=true;
   }
   

   
 }
  
  if (betrieb=="lesen")
  {
    Serial.print("Read Memory ");

    
    byte big_byte=i2c_eeprom.read(0);
    byte small_byte=i2c_eeprom.read(1);
    int16_t number_records = int16_t(big_byte << 8) + int16_t(small_byte);

    Serial.println(number_records);

    for (int i=0;i<number_records;i++)
    {
      for (int k=0;k<amount_data;k++)
      {
          byte big_byte=i2c_eeprom.read(amount_data*i*2+k*2+2);
          byte small_byte=i2c_eeprom.read(amount_data*i*2+k*2+1+2);
          int16_t recorded_data = int16_t(big_byte << 8) + int16_t(small_byte);
          Serial.print(recorded_data);
          Serial.print(";");
      
          
          
      }
    Serial.println();

          
    }
    
    
    delay(1000);    
}
  
  
  

  
  if (betrieb=="schreiben")
  {
    
    int time_since_record=millis()-last_record;
    if (time_since_record>t_cycle_recorder*1000)
    {
    
    byte payload[amount_data*2];

    
    payload[0] = highByte(int16_t (time_since_record));
    payload[1] = lowByte(int16_t (time_since_record));
        
    payload[2] = highByte(int16_t (pitch));
    payload[3] = lowByte(int16_t (pitch));
    
    payload[4] = highByte(int16_t (roll));
    payload[5] = lowByte(int16_t (roll));

    payload[6] = highByte(int16_t (dpitch_dt));
    payload[7] = lowByte(int16_t (dpitch_dt));

    payload[8] = highByte(int16_t (droll_dt));
    payload[9] = lowByte(int16_t (droll_dt));

    
    payload[10] = highByte(int16_t (dv_dt));
    payload[11] = lowByte(int16_t (dv_dt));

    payload[12] = highByte(int16_t (control_radio[1]/100.0*(max_throttle-min_throttle)+min_throttle));
    payload[13] = lowByte(int16_t (control_radio[1]/100.0*(max_throttle-min_throttle)+min_throttle));
    
    payload[14] = highByte(int16_t (servo_angle_vertical));
    payload[15] = lowByte(int16_t (servo_angle_vertical));
    
    payload[16]= highByte(int16_t (servo_angle_horiz));
    payload[17]= lowByte(int16_t (servo_angle_horiz));

    if ((no_records*amount_data*2+2)>EEPROM_BYTES)
    {
    no_records=0;
    current_address=0;
    
    }
    Serial.print("Writing...");
    
    for (int i=0;i<amount_data*2;i++)
    {
    
    
    i2c_eeprom.write(current_address+2, payload[i]);    
    current_address++;
    
    
    }
    no_records++;
    i2c_eeprom.write(0, highByte(no_records));    
    i2c_eeprom.write(1, lowByte(no_records));
    
    
    
    Serial.println("Gespeichert.");

    
    last_record=millis();
    }
  
  }
  
  
}

void Servo_engine_control()
{
  if (autopilot_mode=="MAN") //Manuell
  {
    switch_to_fbw=true;
    //Servo pos 0 bis 100
    servo_angle_vertical=control_radio[3]/100.0*(max_ver-min_ver)+min_ver;
    servo_angle_horiz=control_radio[2]/100.0*(max_hor-min_hor)+min_hor;
    
    myservo_v.write(servo_angle_vertical);    // tell servo to go to position in variable 'pos'
    myservo_h.write(servo_angle_horiz);    // tell servo to go to position in variable 'pos'
    
   int dutyCycle=(control_radio[1]/100.0*(max_throttle-min_throttle)+min_throttle)/100.0*255.0;
  ledcWrite(ledChannel, dutyCycle);
  
  }

  if (autopilot_mode=="FBW") //Kaskadierter Regler Level 1- Rollrate Level 2-Rollwinkel
  {   if (switch_to_fbw==true) //Zurücksetzen regler beim umstellen
      {
        error_pitch_int=0;
        error_roll_int=0;

        switch_to_fbw=false;
      
      }
      //Reglung Anstellwinkel
      float error_pitch=pitch_sp-pitch;
      error_pitch_int=error_pitch_int+error_pitch*cycle_time;
      derror_pitch_dt=(error_pitch-error_pitch_alt)/cycle_time;
      CV_pitch=P_pitch*(error_pitch+1/Ti_pitch*error_pitch_int+D_pitch*derror_pitch_dt)+CV_pitch_neutral;
      CV_pitch=CV_pitch<max_ver?CV_pitch:max_ver;
      CV_pitch=CV_pitch>min_ver?CV_pitch:min_ver;
      if (debug)
      {
      Serial.print("Error pitch:");
      Serial.print(error_pitch);
      
      Serial.print(" CV pitch:");
      Serial.print(CV_pitch,2);
      
      }
      myservo_v.write(CV_pitch);    // tell servo to go to position in variable 'pos'      
      error_pitch_alt=error_pitch;

      //Reglung Rollwinkel
      float error_roll=roll_sp-roll;
      error_roll_int=error_roll_int+error_pitch*cycle_time;
      derror_roll_dt=(error_roll-error_roll_alt)/cycle_time;
      float CV_roll=P_roll*(error_roll+1/Ti_roll*error_roll_int+D_pitch*derror_roll_dt)+CV_roll_neutral;
      CV_roll=CV_pitch<max_ver?CV_pitch:max_ver;
      CV_roll=CV_pitch>min_ver?CV_pitch:min_ver;
      myservo_h.write(CV_roll);    // tell servo to go to position in variable 'pos'      
      error_roll_alt=error_roll;

      //

  }

  
}


void receive_hc12()
{
int control[5]; //button, thrust, horizontal , vertical, control_character

while (HC12.available()){  
  char temp=HC12.read(); 
  if (temp=='S')
  {
  Serial.print(temp);
  temp=(char)HC12.read();
  control[0]=temp-'0';
  HC12.read();
  control[1]=((char)HC12.read()-'0')*100;
  HC12.read();
  control[1]=((char)HC12.read()-'0')*10+control[1];
  control[1]=((char)HC12.read()-'0')+control[1];
  HC12.read();
  control[2]=((char)HC12.read()-'0')*100;
  HC12.read();
  control[2]=((char)HC12.read()-'0')*10+control[2];
  control[2]=((char)HC12.read()-'0')+control[2];
  HC12.read();
  control[3]=((char)HC12.read()-'0')*100;
  HC12.read();
  control[3]=((char)HC12.read()-'0')*10+control[3];
  control[3]=((char)HC12.read()-'0')+control[3];
  control[4]=HC12.read();
  

  //Konsistenzprüfung
  if ((control[0]>=0)&&(control[0]<=1)&&(control[1]>=0)&&(control[1]<=100)&&(control[2]>=0)&&(control[2]<=100)&&(control[3]>=0)&&(control[3]<=100)&&(control[4]=='E'))
  {

   control_radio[0]=control[0];
   control_radio[1]=control[1];
   control_radio[2]=control[2];
   control_radio[3]=control[3];
   switch_to_fbw=false;
   
   if (debug)
   {
   Serial.print("Empfangen");
   Serial.print(control_radio[0]);
   Serial.print(control_radio[1]);
   Serial.print(control_radio[2]);
   Serial.println(control_radio[3]);
   }
      
   }
   else
   {
    //Serial.println("Fehler");
    }

  }
  if (temp=='F')
  {
    
    //Neueinschalten von Fly by wire reinitialisiert regler
   if (switch_to_fbw==false)
    {
    error_pitch_int=0;
    switch_to_fbw=true;
    }
  
  }
}

}


void debugging()
{

 
  
  
  if (debug)
  {
    Serial.print(" Pitch SP: ");
    Serial.print(pitch_sp);
    Serial.print(" Pitch: PV");
    Serial.print(pitch);
    Serial.print(" Autopilot Mode");
    Serial.print(autopilot_mode);
    Serial.println();
    
    }
  
   
 }
 
  
   
