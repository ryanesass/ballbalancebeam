//#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();


#define trigPin1 41 //near sensor (near distance sensor)
#define trigPin2 42 //far sensor (near fan)
#define trigPin3 40 //distance sensor

#define echoPin1 6 //near sensor (near distance sensor)
#define echoPin2 5 //far sensor (near fan)
#define echoPin3 7 //distance sensor

#define SERVOMIN 150
#define SERVOMAX 600

//Servo esc;
//Servo choke;

unsigned long strt1, strt2;
uint16_t pulselength;
int throttlePin,ii,duration,sp,out,degreesm,tme,flag,x;
float distance1,distance2,distance3,tegral1,tegral2,tegral1LIM,tegral2LIM,deriv1,deriv2;
float oldl,oldd,err_l,err_d,outl,outd,Kpl,Kil,Kdl,Kpd,Kid,Kdd,deriv_l,deriv_d,d1;

char buffer[5];
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {

  strt1 = 0;
  throttlePin = A8;
  out = 0;
  outd = 0;
  sp = 75;
  ii = 1;
  
  //leveling gains
  Kpl = 0.75;
  //Kpl = 0;
  //Kil = .0005;
  Kil = 0;
  Kdl = 30; // worked best at 0.7 and 70
  //Kdl = 0;

  //distance/ball gains
  Kpd = .25;
//  Kpd = 0;
 // Kil = 0.001;
  Kil = 0;
  Kdd = 20;
  //Kdd = 0;

  
  tegral1LIM = 7000;
  tegral2LIM = 7000;
  distance1 = 50;
  distance2= 100;  
  tegral1 =0;
  tegral2 =0;
  deriv1 =0;
  deriv2 = 0;
  oldd = 0;
  oldl = 0;
  err_l = 0;
  err_d =0;
  x=0;

  Serial.begin(9600);
  //esc.attach(9);
  //choke.attach(10);
  pwm.begin();
  
  pwm.setPWMFreq(60);
  
  pinMode(trigPin1, OUTPUT);
  pinMode(trigPin2, OUTPUT);
  pinMode(trigPin3, OUTPUT);
  
  pinMode(echoPin1, INPUT);
  pinMode(echoPin2, INPUT);
  pinMode(echoPin3, INPUT);
  
  degreesm = 170;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0,0,pulselength);
  //choke.write(180);
  /*degreesm = 24
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm2.setPWM(1,0,pulselength);*/
   degreesm = 7;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(3,0,pulselength);
  //esc.write(24);
  Serial.println("Calibration Delay");
  delay (8000);  
  //delay(1000);

  degreesm = 143;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0,0,pulselength);
  //choke.write(148);
   degreesm = 24;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(3,0,pulselength);
  //esc.write(44);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
  sen_read(); 
  deadband_err();
  deriv_avg();
  
  diag();
  
  PID_level();   
  PID_distance();

  //perform PI(D) control for level sensors
  ///level PI(D) should control to force the platform level
  ///set limits to I command
  ///tilt should not exceed a given saturated value
  
  //perform PID control for distance sensor
  ///distance PID should over-ride level command to force ball rolling in certain direction
  ///set limits to I command

  choke_set();
  /*degreesm = 170;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0,0,pulselength);
  delay(1000);
  Serial.println(pulselength);
  degreesm = 132;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0,0,pulselength);
  Serial.println(pulselength);
  delay(1000);*/
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void diag(){
  /*Serial.print("Distance 1    ");  
  Serial.println(distance1);
  Serial.print("Distance 2    ");  
  Serial.println(distance2);
  Serial.print("Distance 3    ");  
  Serial.println(distance3);*/

  //Serial.println("");
  
  //Serial.print("Level Difference    ");  
    /*Serial.println("");
    Serial.print(distance1);
    Serial.print("    ");
    Serial.print(distance2);*/
    tme=millis()/10;
    Serial.print("#S|LOGTEST|[");
    Serial.print(itoa((tme),buffer,10));
    Serial.print(",");
    Serial.print(itoa((err_d*10),buffer,10));
    Serial.print(",");
    Serial.print(itoa((tegral2),buffer,10));/**/
    Serial.println("]#");
    delay(5);
  /* Serial.print(itoa((deriv_l*Kdl),buffer,10));
    Serial.print(",");
    Serial.print(itoa(out,buffer,10));
    */

  //Serial.print("Distance Difference    ");
  //Serial.println(err_d);
  //delay(200);  
}
//SENSOR READING
void sen_read(){
  
  //trigPin1
  digitalWrite(trigPin1, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin1, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(trigPin1, LOW);
  duration = pulseIn(echoPin1, HIGH);
  distance1 = (duration/2) / 29.1;

  //trigPin2
  digitalWrite(trigPin2, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin2, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(trigPin2, LOW);
  duration = pulseIn(echoPin2, HIGH);
  distance2 = (duration/2) / 29.1;

  //trigPin3
  digitalWrite(trigPin3, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin3, HIGH);
  delayMicroseconds(10);
  
  digitalWrite(trigPin3, LOW);
  duration = pulseIn(echoPin3, HIGH);
  distance3 = (duration/2) / 29.1;
  
}

//DEADBANDING AND DEFINTION OF ERROR
void deadband_err(){
    if((abs(distance1 - distance2) <= .75)){
      err_l = 0;
    }
    else{
      err_l = distance1 - distance2;
    }
    if(abs(distance3 - sp) <= 1){
      err_d = 0;
    }
    else{
      err_d = distance3 - sp;
    }
    if((millis()-strt1) > 50){
      deriv1 = err_l - oldl;
      deriv2 = err_d - oldd;
      if (abs(deriv1) >= 2) deriv1 = 0;
      if (abs(deriv1) <= 0.5) deriv1 = 0;
      if (abs(deriv2) >= 2) deriv2 = 0;
      if (abs(deriv2) <= 0.5) deriv2 = 0;
      oldl = err_l;
      oldd = err_d;
      flag=1;
      strt1 = millis();
    }
}

void deriv_avg(){
  if(flag == 1){
    if(x < 20){
      deriv_l += deriv1;
      deriv_d += deriv2;
      x++;
    }else{
      deriv_l = deriv1/20;
      deriv_d = deriv2/20;      
      x = 0;      
    }    
  }
  flag = 0;
}


//PID FOR LEVEL CONTROL
void PID_level(){
  //INTEGRAL 
  tegral1 = tegral1 + err_l;
  if(abs(tegral1) > tegral1LIM){
    if(tegral1 > tegral1LIM){
      tegral1 = tegral1LIM;
    }else{
      tegral1 = -tegral1LIM;
    }
  } 
  if(abs(deriv_l) < .2){
    deriv_l = 0;
  } 
  /*Serial.print("#S|LOGTEST|[");
  Serial.print(itoa((deriv_l*100),buffer,10));
  Serial.print(",");
  Serial.print(itoa(out,buffer,10))d;
  Serial.println("]#");*/
  //delay(200);
  //PID LOOP FOR LEVEL CONTROL
  //Serial.println(tegral1);
  outl = err_l*Kpl + tegral1*Kil + Kdl*(deriv_l); // used to be 164 And that works well Changing to 158 for new battery
  deriv_l = 0;  
  outl = (int) outl;
}

void PID_distance(){
  //INTEGRAL 
  if((millis()-strt2) > 200){
    tegral2 = tegral2 + err_d;
    if(abs(tegral2) > tegral2LIM){
      if(tegral2 > tegral2LIM){    
        tegral2 = tegral2LIM;
        }else{
          tegral2 = -tegral2LIM;
          }
        } 
      strt2 = millis();
    }

  if(abs(deriv_d) < .2){
    deriv_d = 0;
  } 
  /*Serial.print("#S|LOGTEST|[");
  Serial.print(itoa((deriv_l*100),buffer,10));
  Serial.print(",");
  Serial.print(itoa(out,buffer,10))d;
  Serial.println("]#");*/
  //delay(200);
  //PID LOOP FOR LEVEL CONTROL
  //Serial.println(tegral1);
  outd = err_d*Kpd + tegral2*Kid + Kdd*(deriv_d); // used to be 164 And that works well Changing to 158 for new battery
  deriv_d = 0;  
  outd = (int) outd;
}

//SETTING CHOKE SERVO TO CONTROL AIRFLOW
void choke_set(){
  out = outl + outd + 149;
  if(out > 170){
    out = 170; 
  }
  if(out < 132){
    out = 132;
  } 
  degreesm = out;
  pulselength = map(degreesm, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(0,0,pulselength);
  //choke.write(out);
  //Serial.println(out);
}

