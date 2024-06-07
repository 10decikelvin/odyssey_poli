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

const float WHEEL_RATIO = 555; // encoder ticks per metre (0.00188496m/t)
const float WHEELBASE = 0.37; // metres
const float HALF_VEHICLE_WIDTH = WHEELBASE / 2; // metres

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
const double KpL = 0.9;
const double KiL = 0.8;
const double KdL = 0.03;
const double biasL = 0;

const double KpR = 0.9;
const double KiR = 0.8;
const double KdR = 0.03;
const double biasR = 0;
// const double KpR = ;
// const double KiR = 0.01;
// const double KdR = 0.000;

const long int TIME = 20000; // us/microseconds

const float VELOCITY_ALPHA = 0.1;
const float SETPOINT_ALPHA = 0.4;

const int RESET_THRESHOLD = 40;
const float DEST_THRESHOLD = 0.03;
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


/// CIRCLE TRACKING VARIABLES
long int dt;
long int dtCircle;
long int prevCircleTime = 0;

const float T = 3; // Time to get to target in seconds
const long int CIRCLE_TIME = 500000;

float dtL;
float dtR;

int des_L = 0;
int des_R = 0;

//====================================

byte dataTX[24]; 
int dataIndex = 0;


// Dumb controller constants
const float KdpL = 0.32;
const float KdpR = 0.29;
const long int dumbTime = 500000;

//const int ERROR_TOLERANCE = 2;

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

      //setpoint prefiltering
      // des_L = lowPassFilter(dataRX[0], des_L, SETPOINT_ALPHA);
      // des_R = lowPassFilter(dataRX[1], des_R, SETPOINT_ALPHA);

      //with circle tracking thingy
      targPos.x = (float) dataRX[0] / 100;
      targPos.y = (float) dataRX[1] / 100;
      targVel = circle_track(currPos, targPos, headingVec);
      des_L = lowPassFilter(targVel.x * WHEEL_RATIO, des_L, SETPOINT_ALPHA);
      des_R = lowPassFilter(targVel.y * WHEEL_RATIO, des_R, SETPOINT_ALPHA);

      //des_L = dataRX[0];
      //des_R = dataRX[1];

      PID(des_L, des_R);
      
      //printPID();
      //printVel();
      drive(driveData);
      //sendEncoder(); // dont need anymore
      //sendOdometry();
      sendVecs();
      newData = false;
  }
  currentTime = millis();
  //sendEncoder();
  //writeOdometry(); 
  
  updateOdometry();
  //sendOdometry();
  sendVecs(); 

  PID(des_L, des_R);

  if (abs(targPos.x - currPos.x) < DEST_THRESHOLD && abs(targPos.y - currPos.y) < DEST_THRESHOLD) {
    driveData[0] = 0;
    driveData[1] = 0;
    driveData[2] = 1;
    integral_L = 0;
    integral_R = 0;
  }

  dtCircle = (long)(micros() - prevCircleTime);
  
  // if (dtCircle > CIRCLE_TIME) {
  //   targVel = circle_track(currPos, targPos, headingVec);
  // }
  
  if(dt > TIME) {
    //printPID();
    //printVel();
    
    // des_L = lowPassFilter(dataRX[0], des_L, SETPOINT_ALPHA);
    // des_R = lowPassFilter(dataRX[1], des_R, SETPOINT_ALPHA);

    //targVel = circle_track(currPos, targPos, headingVec);
    des_L = lowPassFilter(targVel.x * WHEEL_RATIO, des_L, SETPOINT_ALPHA);
    des_R = lowPassFilter(targVel.y * WHEEL_RATIO, des_R, SETPOINT_ALPHA);


    drive(driveData);
  }
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

  // // Check if dataRX[0] is within -255 and 255
  // if (dataRX[0] < -maxPWM) {
  //     dataRX[0] = -maxPWM;
  // } else if (dataRX[0] > maxPWM) {
  //     dataRX[0] = maxPWM;
  // }

  // // Check if dataRX[1] is within -255 and 255
  // if (dataRX[1] < -maxPWM) {
  //     dataRX[1] = -maxPWM;
  // } 
  // else if (dataRX[1] > maxPWM) {
  //     dataRX[1] = maxPWM;
  // }

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
  // dtL = (float)(currTimePID - prevTimePIDL);
  // dtR = (float)(currTimePID - prevTimePIDR);
  if(dt<TIME){return;}
  prevTimePID = currTimePID;

  // only calculate if encoder position has changed
  // if (encoderPosition[0] != prev_L) {
  //   // Calculate current speed
  //   current_L = 1000000*(encoderPosition[0] - prev_L)/dtL;
  //   prevTimePIDL = currTimePID;
  // }

  // if (encoderPosition[1] != prev_R) {
  //   current_R = 1000000*(encoderPosition[1] - prev_R)/dtR;
  //   prevTimePIDR = currTimePID;
  // }

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

  // Serial.print("[");
  // Serial.print("i_L: ");
  // Serial.print(integral_L);
  // Serial.print(", ");
  // Serial.print("i_R: ");
  // Serial.print(integral_R);
  // //print derivatives
  // Serial.print(", ");
  // Serial.print("d_L: ");
  // Serial.print(derivative_L);
  // Serial.print(", ");
  // Serial.print("d_R: ");
  // Serial.print(derivative_R);
  // Serial.println("]");

  
  // if (abs(desired_L) < MIN_SPEED && abs(desired_R) < MIN_SPEED) {
  //   output_L = 0;
  //   output_R = 0;
  //   //Serial.println("override 1");
  //   return;
  // }
  // if (abs(desired_L) < MIN_SPEED) {
  //   output_L = 0;
  //   //Serial.println("override 2");
  //   return;
  // } 
  // if (abs(desired_R) < MIN_SPEED) {
  //   output_R = 0;
  //   //Serial.println("override 3");
  //   return;
  // } 
  

  // if (-ERROR_TOLERANCE < e_L && e_L < ERROR_TOLERANCE && -ERROR_TOLERANCE < e_R && e_R < ERROR_TOLERANCE) {
  //   //integral_L = 0;
  //   e_L = 0;
  //   //integral_R = 0;
  //   e_R = 0;
  //   return; 
  // }
  // if(-ERROR_TOLERANCE < e_L && e_L < ERROR_TOLERANCE) {
  //   //integral_L = 0;
  //   e_L = 0;
  //   output_R = KpR * e_R + KiR * integral_R + KdR * derivative_R;
  //   if (output_R < -MAX_PWM) {
  //     output_R = -MAX_PWM;
  //   } 
  //   else if (output_R > MAX_PWM) {
  //     output_R = MAX_PWM;
  //   }    
  //   return;
  // }
  // if(-ERROR_TOLERANCE < e_R && e_R < ERROR_TOLERANCE) {
  //   //integral_R = 0;
  //   e_R = 0;
  //   output_L = KpL * e_L + KiL * integral_L + KdL * derivative_L;
  //   if (output_L < -MAX_PWM) {
  //     output_L = -MAX_PWM;
  //   } else if (output_L > MAX_PWM) {
  //     output_L = MAX_PWM;
  //   }
  //   return;
  // }

  // Calculate output
  // output_L = lowPassFilter(KpL * e_L + KiL * integral_L + KdL * derivative_L + biasL, output_L, OUTPUT_ALPHA);
  // output_R = lowPassFilter(KpR * e_R + KiR * integral_R + KdR * derivative_R + biasR, output_R, OUTPUT_ALPHA);

  output_L = KpL * e_L + KiL * integral_L + KdL * derivative_L + biasL;
  output_R = KpR * e_R + KiR * integral_R + KdR * derivative_R + biasR;

  output_L = clampSpeed(desired_L, output_L);
  output_R = clampSpeed(desired_R, output_R);


  // if (output_L < -MAX_PWM) {
  //   output_L = -MAX_PWM;
  // } else if (output_L > MAX_PWM) {
  //   output_L = MAX_PWM;
  // }
  // if (output_R < -MAX_PWM) {
  //   output_R = -MAX_PWM;
  // } 
  // else if (output_R > MAX_PWM) {
  //   output_R = MAX_PWM;
  // }    

  // Serial.print("desired_L: ");
  // Serial.println(desired_L);
  // Serial.print("desired_R: ");
  // Serial.println(desired_R);
  // Serial.print("integral_L: ");
  // Serial.println(integral_L);
  // Serial.print("integral_R: ");
  // Serial.println(integral_R);

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

void printPID() {
  Serial.print("{");

  Serial.print("current_L: ");
  Serial.print(current_L);
  Serial.print(", ");
  Serial.print("current_R: ");
  Serial.print(current_R);
  Serial.print(", ");

  Serial.print("e_prev_L: ");
  Serial.print(e_prev_L);
  Serial.print(", ");
  Serial.print("e_prev_R: ");
  Serial.print(e_prev_R);
  Serial.print(", ");

  Serial.print("desired: ");
  Serial.print(des_L);
  Serial.print(" ");
  Serial.print(des_R);
  Serial.print(", ");

  Serial.print("output_L: ");
  Serial.print(output_L);
  Serial.print(", ");
  Serial.print("output_R: ");
  Serial.print(output_R);
  Serial.print(", ");

  Serial.print("dt: ");
  Serial.print(dt);
  Serial.println("} ");

  // Serial.print("e_prev_L: ");
  // Serial.print(e_prev_L);
  // Serial.print(", ");
  // Serial.print("e_prev_R: ");
  // Serial.print(e_prev_R);
  // Serial.print(", ");

  // Serial.print("[");
  // Serial.print("i_L: ");
  // Serial.print(integral_L);
  // Serial.print(", ");

  // Serial.print("i_R: ");
  // Serial.print(integral_R);
  // Serial.println("]");
}

void dumbController(float desired_L, float desired_R) {
  // Calculate time difference
  currTimePID = micros();

  dt = (float)(currTimePID - prevTimePID);

  if(dt < dumbTime){return;}
  prevTimePID = currTimePID;


  current_L = 1000000*(encoderPosition[0] - prev_L)/dt;
  current_R = 1000000*(encoderPosition[1] - prev_R)/dt;

  // Calculate output
  output_L = KdpL * desired_L;
  output_R = KdpR * desired_R;

  if (desired_L < MIN_SPEED && desired_R < MIN_SPEED) {
    output_L = 0;
    output_R = 0;
    //Serial.println("override 1");
  }
  if (desired_L < MIN_SPEED) {
    output_L = 0;
    //Serial.println("override 2");
  } 
  if (desired_R < MIN_SPEED) {
    output_R = 0;
    //Serial.println("override 3");
  } 

  if (output_L < -MAX_PWM) {
    output_L = -MAX_PWM;
  } else if (output_L > MAX_PWM) {
    output_L = MAX_PWM;
  }

  // Check if dataRX[1] is within -255 and 255
  if (output_R < -MAX_PWM) {
    output_R = -MAX_PWM;
  } 
  else if (output_R > MAX_PWM) {
    output_R = MAX_PWM;
  }    

  // Update previous encoder values
  prev_L = encoderPosition[0];
  prev_R = encoderPosition[1];
  
  // Serial.print("desired_L: ");
  // Serial.println(desired_L);
  // Serial.print("desired_R: ");
  // Serial.println(desired_R);
  // Serial.print("integral_L: ");
  // Serial.println(integral_L);
  // Serial.print("integral_R: ");
  // Serial.println(integral_R);

  driveData[0] = output_L;
  driveData[1] = output_R;
  driveData[2] = dataRX[2];
}


vec circle_track1(vec s, vec t, vec h){
  vec motorSpeeds; //the x is the "left side", the y is the "right side"
  vec d;
  d.x = t.x - s.x;
  d.y = t.y - s.y;
  float d_mag_sq = (d.x * d.x + d.y * d.y);

  if(d_mag_sq < 0.05){
    //within tolerance, return;
    motorSpeeds.x = 0;
    motorSpeeds.y = 0;
    return motorSpeeds;
  }
  //calculate position of the turning center (again in meters)
  vec r;
  r.x = d_mag_sq / (2 * d.y * ( 1 + h.y / h.x));
  r.y = d_mag_sq / (2 * d.x * ( 1 + h.x / h.y));
  float r_mag = min(10, sqrt(r.x * r.x + r.y * r.y)); //prevent some divide by 0 shenanigans just in case the radius of curivture is insane
  
  //rough speed that we should be targeting, based on the heureustic that we want to arrive in ~1s. NOTE THAT THE ROUNDING IS _MANDATORY_. DO NOT! REMOVE!!
  float speed = round(sqrt(d_mag_sq) * 5) / (5 * T);

  //compute cross product betweeen heading vector and displacement vector to see if you should turn left or right
  double cp = (double) h.x * d.y - h.y * d.x;
  if(cp > 0){
    motorSpeeds.x = speed / r_mag * (r_mag - (WHEELBASE / 2));
    motorSpeeds.y = speed / r_mag * (r_mag + (WHEELBASE / 2));
  } else if(cp < 0){
    motorSpeeds.x = speed / r_mag * (r_mag + (WHEELBASE / 2));
    motorSpeeds.y = speed / r_mag * (r_mag - (WHEELBASE / 2));
  } else{
    //weird
    motorSpeeds.x = speed;
    motorSpeeds.y = speed;
  }
  
  return motorSpeeds;
}

vec circle_track(vec s, vec t, vec h){
  prevCircleTime = micros();

  vec motorSpeeds; //the x is the "left side", the y is the "right side"
  vec d;
  d.x = t.x - s.x;
  d.y = t.y - s.y;
  float d_mag_sq = (d.x * d.x + d.y * d.y);

  if(d_mag_sq < 0.05){
    //within tolerance, return;
    motorSpeeds.x = 0;
    motorSpeeds.y = 0;
    return motorSpeeds;
  }
  //rough speed that we should be targeting, based on the heureustic that we want to arrive in ~1s. NOTE THAT THE ROUNDING IS _MANDATORY_. DO NOT! REMOVE!!
  float speed = round(sqrt(d_mag_sq) * 5) / (5 * T);
  //cross product. if 0, straight line. if positive, turn left. if negative, turn right.
  double cp = (double) h.x * d.y - h.y * d.x;
  if(cp == 0){
    motorSpeeds.x = speed;
    motorSpeeds.y = speed;
    return motorSpeeds;
  }
  //calculate position of the turning center (again in meters)
  vec r;
  //division is expensive so we save it and only use it once lol
  float multiplier = d_mag_sq / (2 * cp);
  r.x = - multiplier * h.y;
  r.y = multiplier * h.x;
  float r_mag = min(15, sqrt(r.x * r.x + r.y * r.y));
  //decide turn left or right
  if(cp > 0){
    motorSpeeds.x = speed / r_mag * (r_mag - HALF_VEHICLE_WIDTH);
    motorSpeeds.y = speed / r_mag * (r_mag + HALF_VEHICLE_WIDTH);
  }else if(cp < 0){
    motorSpeeds.x = speed / r_mag * (r_mag + HALF_VEHICLE_WIDTH);
    motorSpeeds.y = speed / r_mag * (r_mag - HALF_VEHICLE_WIDTH);
  }
  return motorSpeeds;
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

void sendVecs() {
  Serial.print("<");
  Serial.print(currPos.x);
  Serial.print(",");
  Serial.print(currPos.y);
  Serial.print(",");
  Serial.print(heading);
  Serial.print(",");
  Serial.print(targPos.x);
  Serial.print(",");
  Serial.print(targPos.y);
  Serial.print(",");
  Serial.print(targVel.x);
  Serial.print(",");
  Serial.print(targVel.y);
  Serial.println(">");
}

// void writeOdometry() {
//   // Convert floats to byte arrays and store in data array
//   memcpy(&dataTX[dataIndex], &x, sizeof(float));
//   dataIndex += sizeof(float);
//   memcpy(&dataTX[dataIndex], &y, sizeof(float));
//   dataIndex += sizeof(float);
//   memcpy(&dataTX[dataIndex], &heading, sizeof(float));
//   dataIndex += sizeof(float);
//   // Convert longs to byte arrays and store in data array
//   memcpy(&dataTX[dataIndex], &encoderPosition[0], sizeof(long int));
//   dataIndex += sizeof(long int);
//   memcpy(&dataTX[dataIndex], &encoderPosition[1], sizeof(long int));
//   dataIndex += sizeof(long int);
//   memcpy(&dataTX[dataIndex], &deltaHeading, sizeof(float));
//   dataIndex += sizeof(float);
//   Serial.write(dataTX, dataIndex);
// }

//eg. <50,50,0,0>
//MODE 0 = normal, 1 = off, 2 = brake
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


