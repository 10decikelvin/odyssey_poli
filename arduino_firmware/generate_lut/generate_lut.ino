#include <math.h>
#include <digitalWriteFast.h>

const int MotorRLEN = 6;
const int MotorRREN = 7;
const int MotorRLPWM = 8;
const int MotorRRPWM = 9;
const int MotorLLEN = 10;
const int MotorLREN = 11;
const int MotorLLPWM = 12;
const int MotorLRPWM = 13;

const int encoderRPinA = 2;
const int encoderRPinB = 4;
const int encoderLPinA = 3; 
const int encoderLPinB = 5;

const int MAX_PWM = 250;

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

int dataRX[4] = {0,0,0,0};

bool newData = false;

long int encoderPosition[2] = {0, 0}; // Stores last encoder positions L and R

//ODO calcs=========================

const float WHEEL_RATIO = 500; // encoder ticks per metre (0.00188496m/t)
const float WHEELBASE = 0.37; // metres
const float HALF_VEHICLE_WIDTH = WHEELBASE / 2; // metres
const float MAX_SPEED = 1300 / WHEEL_RATIO;
int offValues[3] = {0,0,1};

double leftDelta = 0;
double rightDelta = 0;

double newX = 0;
double newY = 0;
double x = 0;
double y = 0;
double dx = 0;
double dy = 0;

float dist = 0;

long prevLeft = 0;
long prevRight = 0;
unsigned long prevTime = 0;
unsigned long currentTime = 0;

float heading = 0;
float newHeading = 0;
float deltaHeading = 0;


struct vec
{
  double x;
  double y;
};

vec currPos = {0, 0};
vec newPos = {0, 0};
vec dPos = {0, 0};

vec targPos = {0, 0};
vec targVel = {0, 0};
vec headingVec = {1, 0};


//====================================

/////// PID CONTROL ///////
// DESIRED SPEEDS
// float desired_L = 0;
// float desired_R = 0;
double current_L = 0; // IN TICKS PER SECOND => 1m/s = 531 ticks/s
double current_R = 0;
double prev_Vel_L = 0;
double prev_Vel_R = 0;

const int MIN_SPEED = 4; // 8 mm/s


// ===================================
// ===================================
// ========== PID constants ==========
const double KpL = 0.9; // 0.9;
const double KiL = 0.8;//0.8;
const double KdL = 0;//0.02;
const double biasL = 0;

const double KpR = 0.9;//0.8;
const double KiR = 0.8;//0.75;
const double KdR = 0;//0.02;
const double biasR = 0;
// const double KpR = ;
// const double KiR = 0.01;
// const double KdR = 0.000;

const long int TIME = 100000; // us/microseconds

const float VELOCITY_ALPHA = 0.1;
const float SETPOINT_ALPHA = 0.05;

const int RESET_THRESHOLD = 40;
const float DEST_THRESHOLD = 0.05;
// ===================================
// ===================================
// ===================================


// Error variables
double e_prev_L = 0;  // Previous left speed error
double e_prev_R = 0;  // Previous right speed error
double integral_L = 0;  // Integral term for left wheel
double integral_R = 0;  // Integral term for right wheel

float output_L = 0;
float output_R = 0;

long long int prev_L = 0;
long long int prev_R = 0;

// time variables
unsigned long prevTimePID = 0;
// unsigned long prevTimePIDL = 0;
// unsigned long prevTimePIDR = 0;
unsigned long currTimePID = 0;

int driveData[3] = {0,0,0};
double dt;
long int dtCircle;
long int prevCircleTime = 0;
float dtL;
float dtR;

int des_L = 0;
int des_R = 0;

//====================================

bool driving = false;
long int driveTime;
const long int measureDelay = 2000000; // us
long int driveDuration = 2000000; // us

void setup() {
  Serial.begin(500000);
  Serial.println("Enter data in this form <LeftSpeed, RightSpeed, mode, resetEnc>");
  Serial.println("Speed values in encoder ticks/s (531 t/s = 1 m/s)");
  //Serial.println("Modes: 0 = normal drive | 1 = coast | 2 = brake");
  Serial.println("ResetEnc: 0 = normal | 1 = reset encoder distance");
  Serial.println();

  pinMode(MotorRLEN, OUTPUT);
  pinMode(MotorRREN, OUTPUT);
  pinMode(MotorRLPWM, OUTPUT);
  pinMode(MotorRRPWM, OUTPUT);
  pinMode(MotorLLEN, OUTPUT);
  pinMode(MotorLREN, OUTPUT);
  pinMode(MotorLLPWM, OUTPUT);
  pinMode(MotorLRPWM, OUTPUT);

  pinMode(encoderRPinA, INPUT_PULLUP);
  pinMode(encoderRPinB, INPUT_PULLUP);
  pinMode(encoderLPinA, INPUT_PULLUP);
  pinMode(encoderLPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderRPinA), readEncoderR, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderLPinA), readEncoderL, RISING);
}

//====================================

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseDataPID();
      //showParsedData(); // s l o w

      driveTime = micros();
      
      driveData[0] = dataRX[0];
      driveData[1] = dataRX[1];
      driveData[2] = dataRX[2];

      drive(driveData);
      driving = true;

      newData = false;
  }
  currentTime = millis();
  
  while(driving == true){
    measureSpeed();
  }


  //sendEncoder();
  //updateOdometry();
}

//====================================

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

//====================================

void parseDataPID() {      // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(tempChars, ",");      // get the first part - int1
  dataRX[0] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // get the second part - int2
  dataRX[1] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // get the third part - int3
  dataRX[2] = atoi(strtokIndx);     // convert this part to an integer

  strtokIndx = strtok(NULL, ","); // get the third part - int3
  dataRX[3] = atoi(strtokIndx);     // convert this part to an integer

  // Check if dataRX[0] is within -255 and 255
  if (dataRX[0] < -MAX_PWM) {
      dataRX[0] = -MAX_PWM;
  } else if (dataRX[0] > MAX_PWM) {
      dataRX[0] = MAX_PWM;
  }

  // Check if dataRX[1] is within -255 and 255
  if (dataRX[1] < -MAX_PWM) {
      dataRX[1] = -MAX_PWM;
  } 
  else if (dataRX[1] > MAX_PWM) {
      dataRX[1] = MAX_PWM;
  }

  // Check if dataRX[2] is within 0 and 2
  if (dataRX[2] < 0 || dataRX[2] > 2) {
      dataRX[2] = 1;
  }

  // Reset distance if dataRX[3] is 1
  if (dataRX[3] == 1) {
    // encoderPosition[0] = 0;
    // encoderPosition[1] = 0;
    dPos.x = 0;
    dPos.y = 0;
    currPos.x = 0;
    currPos.y = 0;
    heading = 0;
    deltaHeading = 0;
  }
}

//====================================
void showParsedData() {
    Serial.print("speedL: ");
    Serial.println(dataRX[0]);
    Serial.print("speedR: ");
    Serial.println(dataRX[1]);
    Serial.print("mode: ");
    Serial.println(dataRX[2]);
    Serial.print("reset: ");
    Serial.println(dataRX[3]);
}

void sendEncoder() {
  // Calculate and print encoder distances
  Serial.print("<");
  Serial.print(encoderPosition[0]);
  Serial.print(",");
  Serial.print(encoderPosition[1]);
  Serial.println(">");
}

void readEncoderR() {
  if (digitalRead(encoderRPinB) == LOW) {
    encoderPosition[0]--;
  } else {
    encoderPosition[0]++;
  }
}

void readEncoderL() {
  if (digitalRead(encoderLPinB) == LOW) {
    encoderPosition[1]++;
  } else {
    encoderPosition[1]--;
  }
}

float boundAngle(float angle) {
  while (angle < 0) {
    angle += (float)2 * M_PI;
  }
  while (angle >= 2 * M_PI) {
    angle -= (float)2 * M_PI;
  }
  return (float)angle;
}

//Low pass filter for velocity
float lowPassFilter(float input, float prev, float alpha) {
  return alpha * input + (1 - alpha) * prev;
}


//function to check if target speed is positive, clamp input to positive values, vice versa for negative
//also check to see if is more than MAX_PWM
float clampSpeed(float target, float input) {
  if (target > 0) {
    if (input < 0) {
      return 0;
    }
    if (input > MAX_PWM) {
      return MAX_PWM;
    }
  } else if (target < 0) {
    if (input > 0) {
      return 0;
    }
    if (input < -MAX_PWM) {
      return -MAX_PWM;
    }
  }
  return input;
}

//==============================
// Calculate PID control
void PID(float desired_L, float desired_R) {
  // Calculate time difference
  currTimePID = micros();

  dt = (float)(currTimePID - prevTimePID);

  if(dt<TIME){return;}
  prevTimePID = currTimePID;


  prev_Vel_L = current_L;
  prev_Vel_R = current_R;

  current_L = 1000000*(encoderPosition[0] - prev_L)/dt;
  current_R = 1000000*(encoderPosition[1] - prev_R)/dt;

  current_L = lowPassFilter(current_L, prev_Vel_L, VELOCITY_ALPHA);
  current_R = lowPassFilter(current_R, prev_Vel_R, VELOCITY_ALPHA);

  if(dataRX[2] == 2) {
    current_L = 0;
    current_R = 0;
    integral_L = 0;
    integral_R = 0;
  }
  if(dataRX[2] == 1) {
    integral_L = 0;
    integral_R = 0;
  }


  // Calculate error
  float e_L = desired_L - current_L;
  float e_R = desired_R - current_R;

  // Calculate integral term
  integral_L += e_L * dt / 1000000;
  integral_R += e_R * dt / 1000000;

  // Calculate derivative term
  float derivative_L = 1000000*(e_L - e_prev_L) / dt;
  float derivative_R = 1000000*(e_R - e_prev_R) / dt;

    // Update previous encoder values
  prev_L = encoderPosition[0];
  prev_R = encoderPosition[1];
  
  e_prev_L = e_L;
  e_prev_R = e_R;

  output_L = KpL * e_L + KiL * integral_L + KdL * derivative_L + biasL;
  output_R = KpR * e_R + KiR * integral_R + KdR * derivative_R + biasR;

  output_L = clampSpeed(desired_L, output_L);
  output_R = clampSpeed(desired_R, output_R);


  driveData[0] = output_L;
  driveData[1] = output_R;
  driveData[2] = dataRX[2];
}

void printVel() {
  Serial.print(current_L);
  Serial.print(" ");
  Serial.print(current_R);
  Serial.print(" ");
  Serial.print(des_L);
  Serial.print(" ");
  Serial.print(des_R);
  Serial.print(" ");
  Serial.print("0");
  Serial.print(" ");
  Serial.println("500");
}


void measureSpeed() {
  // Calculate time difference
  currTimePID = micros();

  dt = (float)(currTimePID - prevTimePID);

  if(dt<TIME){return;}
  prevTimePID = currTimePID;

  current_L = (double)1000000*(encoderPosition[0] - prev_L)/dt;
  current_R = (double)1000000*(encoderPosition[1] - prev_R)/dt;

  Serial.print("<");
  Serial.print(current_L);
  Serial.print(", ");
  Serial.print(current_R);
  Serial.println(">");
  
  // Update previous encoder values
  prev_L = encoderPosition[0];
  prev_R = encoderPosition[1];

  if((currTimePID - driveTime) > measureDelay){
    driving = false;
    drive(offValues);
  }
}


// MY ONE (working)
void updateOdometry() {
  // Update encoder deltas
  leftDelta = (encoderPosition[0] - prevLeft) / WHEEL_RATIO; // left side speed
  rightDelta = (encoderPosition[1] - prevRight) / WHEEL_RATIO; // right side speed

  prevLeft = encoderPosition[0];
  prevRight = encoderPosition[1];

  dist = (leftDelta + rightDelta) / 2;

  if (abs(leftDelta - rightDelta) == 0) { // basically going straight
    newPos.x = currPos.x + leftDelta * cos(heading); // current pos + horizontal velocity
    newPos.y = currPos.y + rightDelta * sin(heading); // current pos + vertical velocity
    newHeading = heading; // no change in heading
  }
  else {
    float R = (leftDelta + rightDelta) / (rightDelta - leftDelta) * WHEELBASE / 2;
    deltaHeading = (float)(rightDelta - leftDelta) / WHEELBASE;

    newPos.x = currPos.x + dist * cos(heading + deltaHeading / 2);
    newPos.y = currPos.y + dist * sin(heading + deltaHeading / 2);
    newHeading = boundAngle(heading + deltaHeading);
  }
  // update previous values to current
  dPos.x = newPos.x - currPos.x;
  dPos.y = newPos.y - currPos.y;
  currPos.x = newPos.x;
  currPos.y = newPos.y;
  heading = newHeading;
  headingVec.x = cos(heading);
  headingVec.y = sin(heading);
  // gives x, y, θ, dx, dy, dθ
}


void sendOdometry() {
  Serial.print("<");
  Serial.print(currPos.x);
  Serial.print(",");
  Serial.print(currPos.y);
  Serial.print(",");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(encoderPosition[0]);
  Serial.print(",");
  Serial.print(encoderPosition[1]);
  Serial.print(",");
  Serial.print(deltaHeading);
  Serial.println(">");
}


//eg. <50,50,0,0>
//3 element array, left, right, mode
void drive(int data[]) {
  // LPWM = data[0];
  // RPWM = data[1];
  // mode = data[2];
  
  //NORMAL
  if(data[2] == 0) {
    //LEFT SIDE DRIVE
    if(data[0] > 0) {
      digitalWriteFast(MotorLLEN,1);
      digitalWriteFast(MotorLREN,1);
      analogWrite(MotorLLPWM,0);
      analogWrite(MotorLRPWM,data[0]);
    }
    else if(data[0] < 0) {
      digitalWriteFast(MotorLLEN,1);
      digitalWriteFast(MotorLREN,1);
      analogWrite(MotorLLPWM,-1*data[0]);
      analogWrite(MotorLRPWM,0);
    }
    else if(data[0] == 0) {
      digitalWriteFast(MotorLLEN,1);
      digitalWriteFast(MotorLREN,1);
      analogWrite(MotorLLPWM,0);
      analogWrite(MotorLRPWM,0);
    }

    //RIGHT SIDE DRIVE
    if(data[1] > 0) {
      digitalWriteFast(MotorRLEN,1);
      digitalWriteFast(MotorRREN,1);
      analogWrite(MotorRLPWM,0);
      analogWrite(MotorRRPWM,data[1]);
    }
    else if(data[1] < 0) {
      digitalWriteFast(MotorRLEN,1);
      digitalWriteFast(MotorRREN,1);
      analogWrite(MotorRLPWM, -1*data[1]);
      analogWrite(MotorRRPWM,0);
    }
    else if(data[1] == 0) {
      digitalWriteFast(MotorRLEN,1);
      digitalWriteFast(MotorRREN,1);
      analogWrite(MotorRLPWM,0);
      analogWrite(MotorRRPWM,0);
    }

    //BOTH 0
    if(data[0] == 0 && data[1] == 0) {
      digitalWriteFast(MotorLLEN,0);
      digitalWriteFast(MotorLREN,0);
      analogWrite(MotorLLPWM,0);
      analogWrite(MotorLRPWM,0);
      digitalWriteFast(MotorRLEN,0);
      digitalWriteFast(MotorRREN,0);
      analogWrite(MotorRLPWM,0);
      analogWrite(MotorRRPWM,0);
    }
  }

  //OFF
  else if (data[2] == 1) {
    digitalWriteFast(MotorLLEN,0);
    digitalWriteFast(MotorLREN,0);
    analogWrite(MotorLLPWM,0);
    analogWrite(MotorLRPWM,0);
    digitalWriteFast(MotorRLEN,0);
    digitalWriteFast(MotorRREN,0);
    analogWrite(MotorRLPWM,0);
    analogWrite(MotorRRPWM,0);
  }

  //BRAKE (avoid this, causes new spikes)
  else if (data[2] == 2) {
    digitalWriteFast(MotorLLEN,1);
    digitalWriteFast(MotorLREN,1);
    analogWrite(MotorLLPWM,0);
    analogWrite(MotorLRPWM,0);
    digitalWriteFast(MotorRLEN,1);
    digitalWriteFast(MotorRREN,1);
    analogWrite(MotorRLPWM,0);
    analogWrite(MotorRRPWM,0);
  }
  else {
    digitalWriteFast(MotorLLEN,0);
    digitalWriteFast(MotorLREN,0);
    analogWrite(MotorLLPWM,0);
    analogWrite(MotorLRPWM,0);
    digitalWriteFast(MotorRLEN,0);
    digitalWriteFast(MotorRREN,0);
    analogWrite(MotorRLPWM,0);
    analogWrite(MotorRRPWM,0);
  }
}


