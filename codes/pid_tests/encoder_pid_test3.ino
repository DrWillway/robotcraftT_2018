#include <Encoder.h>
#define AIN2 8
#define BIN1 5
#define AIN1 7
#define BIN2 6
#define PWMA 4
#define PWMB 9
#define FRONT_S A5
#define BACK_S A6
#define LS A2
#define FS A3
#define RS A4
#define NORMALIZE(z) atan2(sin(z), cos(z))  // auxiliary function to normalize angle to the -pi, pi domain

//for sensors
const int WhiteThreshold = 700;
const int BlackThreshold = 900;
bool sensFront = false;
bool sensBack = false;
int rightDist, leftDist, frontDist;
float mmLeft, mmFront, mmRight;

//for encoders
Encoder encL(3, 2);
Encoder encR(19, 18);
long int currentNL, currentNR, NL, NR, previousNL=0, previousNR=0;

//speed and position
float r = 0.016;
float b = 0.093;
float C = 8364.0; //3576.0;
float v = 0.05;
float w = 0.0;
float angle=0;
float x;
float y;
float wleft;
float wright;
float vreal;
float wreal;
float wleftreal;
float wrightreal;
float period = 0.1;
float current_error_left;
float current_error_right;
float sum_error_left;
float sum_error_right;
float diff_error_left;
float diff_error_right;
float previous_error_left = 0 ;
float previous_error_right = 0 ;
float gain_left;
float gain_right;
//float Kpl = 110.0, Kll = 1.02, Kdl = 0.5, Kpr = Kpl, Klr = Kll, Kdr = Kdl;
//float Kpl = 110.0, Kll = 80.0, Kdl = 0.4, Kpr = Kpl, Klr = Kll, Kdr = Kdl;
float Kpl = 60.0, Kll = 170.0, Kdl = 0.4, Kpr = Kpl, Klr = Kll, Kdr = Kdl;
float speed_right;
float speed_left;
float DL,DR,D;

//for mesuring time
unsigned long oldTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(FRONT_S, INPUT);
  pinMode(BACK_S, INPUT);
}

//*************IR distance sensors*******************//
float sensR(int averagecounter) {
  float sum = 0;
  float average;
  for (int i = 0; i < averagecounter; i++)
  {
    sum = sum + pow(3027.4 / analogRead(RS), 1.2134);
  }
  average = sum / averagecounter;
  //Serial.println(average);
  return average;
}

float sensL(int averagecounter) {
  float sum = 0;
  float average;
  for (int i = 0; i < averagecounter; i++)
  {
    sum = sum + pow(3027.4 / analogRead(LS), 1.2134);
  }
  average = sum / averagecounter;
  //Serial.println(average);
  return average;
}

float sensF(int averagecounter) {
  float sum = 0;
  float average;
  for (int i = 0; i < averagecounter; i++)
  {
    sum = sum + pow(3027.4 / analogRead(FS), 1.2134);
  }
  average = sum / averagecounter;
  Serial.println(average);
  return average;
}

//*************Line sensors*******************//
//front sensor
bool sensDF() {
  int sensor = analogRead(FRONT_S);
  if (sensor < WhiteThreshold) { //white
    sensFront = false;
  } else if (sensor > BlackThreshold) { //black
    sensFront = true;
  }
  return sensFront;
}

//back sensor
bool sensBF() {
  int sensor = analogRead(BACK_S);
  if (sensor < WhiteThreshold) { //white
    sensBack = false;
  } else if (sensor > BlackThreshold) { //black
    sensBack = true;
  }
  return sensBack;
}

//*************Wheels control*******************//
void moveLWheel(float mSpeed) {
  if (mSpeed >= 0) {
    /*Serial.print("my print: ");*/
   // Serial.println((int)mSpeed);
    analogWrite(PWMA, (int)mSpeed);
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
  }
  else {
    analogWrite(PWMA, (int)mSpeed * -1);
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
  }
}

void moveRWheel(float mSpeed) {
  if (mSpeed >= 0) {
    analogWrite(PWMB, (int)mSpeed);
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
  }
  else {
    analogWrite(PWMB, (int)mSpeed * -1);
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
  }
}

void moveTwoWheels(float mSpeed) {
  moveRWheel(mSpeed);
  moveLWheel(mSpeed);
}

void stopL() {
  analogWrite(PWMB, 0);
  digitalWrite(BIN1, 0);
  digitalWrite(BIN2, 0);
}

void stopR() {
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, 0);
  digitalWrite(AIN2, 0);
}

void stopTwoWheels() {
  stopL();
  stopR();
}

//*************Encoder and Count Time*******************//
void distEnc() {
  currentNL = encL.read();
  currentNR = encR.read() * -1;
  NL = currentNL - previousNL;
  NR = currentNR - previousNR;
  previousNL = currentNL;
  previousNR = currentNR;
}

void poseUpdate() {
  distEnc();
  DL = ((2.0 * PI * (r) / C) * (float)(NL));
  DR = ((2.0 * PI * (r) / C) * (float)(NR));
  angle = atan2(sin(angle+wreal),cos(angle+wreal));
 // x += x + vreal*cos(angle);
 // y += y + vreal*sin(angle);
 // angle += (DR - DL) / b;
 // angle = NORMALIZE(angle);
  D = (DL + DR) / 2.0;
  x += D * cos(angle);
  y += D * sin(angle);
  Serial.print("New coordinates:  ");
  Serial.print(x);
  Serial.print("  ");
  Serial.print(y);
  Serial.print("  ");
  Serial.println(angle);

}

void cmd_realvel() {
  vreal = ((2.0 * PI * r) / C) * ((NR + NL) / 2.0) * (1.0 / period);
  wreal = ((2.0 * PI * r) / C) * ((NR - NL) / 2.0) * (1.0 / period);
}

void cmd_desiredvel2wheels() {
  wleft = (v - (b / 2.0) * w) / r;
  wright = (v + (b / 2.0) * w) / r;
}

void cmd_realvel2wheels() {
  wleftreal = (vreal - (b / 2.0) * wreal) / r;
  wrightreal = (vreal + (b / 2.0) * wreal) / r;
}

void pid() {
  current_error_left = wleft - wleftreal;
  current_error_right = wright - wrightreal;
  
  sum_error_left += current_error_left;
  sum_error_right += current_error_right;
  
  diff_error_left = current_error_left - previous_error_left;
  diff_error_right = current_error_right - previous_error_right;

  gain_left = Kpl * current_error_left + Kll * sum_error_left * period + (Kdl) * (diff_error_left / period);
  gain_right = Kpr * current_error_right + Klr * sum_error_right * period + (Kdr) * (diff_error_right / period);
  
  previous_error_left = current_error_left;
  previous_error_right = current_error_right;
   
  if(gain_left > 255) gain_left = 255;
  else if (gain_left<-255) gain_left=-255;
  if(gain_right > 255) gain_right = 255;
  else if (gain_right<-255) gain_right=-255;
  
  speed_left = gain_left ;
  speed_right = gain_right ;
}

char c;

void count(){
      poseUpdate();
      cmd_realvel();
      cmd_desiredvel2wheels();
      cmd_realvel2wheels();
      pid();
}
void loop() {
    if (millis() - oldTime >= 100) {
      oldTime = millis();
      
      if(Serial.available()){
        c=Serial.read();
      }
      Serial.println(c);
     switch(c){
        case 'w': 
                count();
                moveLWheel(gain_left);
                moveRWheel(gain_right);
                break;
        case 'd': 
                count();
                moveLWheel(gain_left);
                moveRWheel(gain_right*-1);
                break;
        case 'a': 
                count();
                moveLWheel(gain_left*-1);
                moveRWheel(gain_right);
                break;
        case 's':
                moveLWheel(0);
                moveRWheel(0);
                break;
        case 'z': 
                count();
                moveLWheel(gain_left*-1);
                moveRWheel(gain_right*-1);
                break;
      }
    }
}
