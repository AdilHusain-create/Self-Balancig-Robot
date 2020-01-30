#include <Wire.h>
////////////////VARIABLE DEFINATION///////////////
int rmotor1 = 9;
int rmotor2 = 10;
int lmotor1 = 6;
int lmotor2 = 5;
int mspeed = 10;
int turnspeed=50;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];
float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;
float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
////////////////////////PID CONSTANST/////////////////////
float kp=25;
float ki=0;
float kd=0.8;
float desired_angle = 0;//////////////TARGET ANGLE/////////////
void setup() 
{
  Wire.begin(); /////////////TO BEGIN I2C COMMUNICATIONS///////////////
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  ////////////////PIN MODE DEFINATIONS//////////////////////
  pinMode(rmotor1,OUTPUT);
  pinMode(lmotor1,OUTPUT);
  pinMode(rmotor2,OUTPUT);
  pinMode(lmotor2,OUTPUT);
  Serial.begin(9600);
  time = millis(); ///////////////STARTS COUNTING TIME IN MILLISECONDS/////////////
}
void loop() 
{
  /*////////////////////////WARNING//////////////////////
   * DO NOT USE ANY DELAYS INSIDE THE LOOP OTHERWISE THE BOT WON'T BE 
   * ABLE TO CORRECT THE BALANCE FAST ENOUGH
   * ALSO, DONT USE ANY SERIAL PRINTS. BASICALLY DONT SLOW DOWN THE LOOP SPEED.
  */
    timePrev = time;  
    time = millis();  
    elapsedTime = (time - timePrev) / 1000; 
    Wire.beginTransmission(0x68);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,6,true);
    ////////////////////PULLING RAW ACCELEROMETER DATA FROM IMU///////////////// 
    Acc_rawX=Wire.read()<<8|Wire.read(); 
    Acc_rawY=Wire.read()<<8|Wire.read();
    Acc_rawZ=Wire.read()<<8|Wire.read(); 
    /////////////////////CONVERTING RAW DATA TO ANGLES/////////////////////
    Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
    Wire.beginTransmission(0x68);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(0x68,4,true); 
    //////////////////PULLING RAW GYRO DATA FROM IMU/////////////////////////
    Gyr_rawX=Wire.read()<<8|Wire.read(); 
    Gyr_rawY=Wire.read()<<8|Wire.read(); 
    ////////////////////CONVERTING RAW DATA TO ANGLES///////////////////////
    Gyro_angle[0] = Gyr_rawX/131.0; 
    Gyro_angle[1] = Gyr_rawY/131.0;
    //////////////////////////////COMBINING BOTH ANGLES USING COMPLIMENTARY FILTER////////////////////////
    Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
    Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
    ////TOTAL_ANGLE[0] IS THE PITCH ANGLE WHICH WE NEED////////////
    error = Total_angle[0] - desired_angle; /////////////////ERROR CALCULATION////////////////////
    ///////////////////////PROPORTIONAL ERROR//////////////
    pid_p = kp*error;
    ///////////////////////INTERGRAL ERROR/////////////////
    pid_i = pid_i+(ki*error);  
    ///////////////////////DIFFERENTIAL ERROR//////////////
    pid_d = kd*((error - previous_error)/elapsedTime);
    ///////////////////////TOTAL PID VALUE/////////////////
    PID = pid_p + pid_d;
    ///////////////////////UPDATING THE ERROR VALUE////////
    previous_error = error;
    //Serial.println(PID);                     //////////UNCOMMENT FOR DDEBUGGING//////////////
    //delay(60);                               //////////UNCOMMENT FOR DDEBUGGING//////////////
    //Serial.println(Total_angle[0]);          //////////UNCOMMENT FOR DDEBUGGING//////////////
    /////////////////CONVERTING PID VALUES TO ABSOLUTE VALUES//////////////////////////////////
    mspeed = abs(PID);
    //Serial.println(mspeed);                  //////////UNCOMMENT FOR DDEBUGGING//////////////
    ///////////////SELF EXPLANATORY///////////////
    if(Total_angle[0]<0)
      {
       anti();
      }
    if(Total_angle[0]>0)
      {
       clockw();
      }
    if(Total_angle[0]>45)
    halt();
    if(Total_angle[0]<-45)
    halt();
    
}
//////////////MOVEMENT FUNCTION///////////////////
void clockw()
{
  analogWrite(rmotor1,mspeed);
  analogWrite(rmotor2,0);
  analogWrite(lmotor1,mspeed);
  analogWrite(lmotor2,0); 
}
void anti()
{

  analogWrite(rmotor2,mspeed);
  analogWrite(rmotor1,0);
  analogWrite(lmotor2,mspeed);
  analogWrite(lmotor1,0);
}
void halt()
{
  
  analogWrite(rmotor1,0);
  analogWrite(rmotor2,0);
  analogWrite(lmotor1,0);
  analogWrite(lmotor2,0);
  
}

       