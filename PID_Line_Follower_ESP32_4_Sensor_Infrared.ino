#define setpoint 0

const int pinIR1 = 34;
const int pinIR2 = 35;
const int pinIR3 = 32;
const int pinIR4 = 33;

const int IN1 = 4;
const int IN2 = 2;
const int IN3 = 5;
const int IN4 = 18;
const int enA = 19;
const int enB = 15;

//kecepatan motor
int rightspeed = 100;
int leftspeed = 100;

//Deklarasi PID
float Kp = 25;
float Ki = 0.5;
float Kd = 2.5;

int error, sensor, derivative, integral, PID, errorTerakhir, totalkiri, totalkanan;

void setup() 
{
  Serial.begin (9600);

    Serial.begin(9600);
  pinMode (pinIR1, INPUT);
  pinMode (pinIR2, INPUT);
  pinMode (pinIR3, INPUT);
  pinMode (pinIR4, INPUT);
  
  pinMode (IN1, OUTPUT);
  pinMode (IN2, OUTPUT);
  pinMode (IN3, OUTPUT);
  pinMode (IN4, OUTPUT);

  digitalWrite (IN1, HIGH);
  digitalWrite (IN2, LOW);
  digitalWrite (IN3, HIGH);
  digitalWrite (IN4, LOW);
}

void loop() 
{

  int sensorState4 = digitalRead(pinIR4); 
  int sensorState3 = digitalRead(pinIR3); 
  int sensorState2 = digitalRead(pinIR2); 
  int sensorState1 = digitalRead(pinIR1);


  /////////////////////////////////////////////// kiri 1
  if (sensorState1 == HIGH && sensorState2 == HIGH && sensorState3 == HIGH && sensorState4 == LOW)
    {
  sensor = 2;
    }
  /////////////////////////////////////////////// kiri 2
  if (sensorState1 == HIGH && sensorState2 == HIGH && sensorState3 == LOW && sensorState4 == LOW)
    {
  sensor = 1;
    }
    /////////////////////////////////////////////// Depan
  if (sensorState1 == HIGH && sensorState2 == LOW && sensorState3 == LOW && sensorState4 == HIGH)
    {
  sensor = 0;
    }
       /////////////////////////////////////////////// Kanan 1
  if (sensorState1 == LOW && sensorState2 == LOW && sensorState3 == HIGH && sensorState4 == HIGH)
    {
  sensor = -1;
    }
         /////////////////////////////////////////////// Kanan 2
  if (sensorState1 == LOW && sensorState2 == HIGH && sensorState3 == HIGH && sensorState4 == HIGH)
    {
  sensor = -2;
    }

  error = setpoint - sensor;

   integral = error + integral;
  if (integral >=5){
    integral = 5;    
  }
  if (integral <=-5){
    integral =-5;    
  }    
  derivative = (error - errorTerakhir);

 //penghitungan PID 
  PID = (Kp * error + Kd * derivative + Ki * integral);
  
  errorTerakhir = error;

  if (PID >=255){
    PID = 255;    
  }
  if (PID <=-255){
    PID =-255;    
  }  

    totalkiri = leftspeed - (PID);
    totalkanan = rightspeed + PID;
    forward(totalkanan, totalkiri); 
  
 }
 void forward(int speed1, int speed2){

  // Menentukan nilaikecepatan maju; max =  nilai min = 0
  if ( speed1 < 0 ) {
    speed1 = 0;   
  }
  if ( speed1 > 512 ) {  
    speed1 = 512;
  }
  if (speed2 < 0) {
    speed2 = 0;  
  }
  if (speed2 > 512) {
    speed2 = 512;
  }
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(enA, speed1);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(enB, speed2);
}
