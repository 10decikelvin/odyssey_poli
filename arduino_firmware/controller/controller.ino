#include <math.h>
#include <digitalWriteFast.h>
#include "lut.h"

/************************************************
 *  \brief Motor and Encoder Pin Definitions
 * @ingroup vehicle
 ***********************************************/
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



/*************************************************
 * \brief Variables for receiving and parsing data
 ************************************************/
/**
 * @addtogroup comm Serial Communication
 * @{
 */
const byte numChars = 32;        ///< number of characters in the array
char receivedChars[numChars];    ///< array to store received data   
char tempChars[numChars];        ///< temporary array for use when parsing
int dataRX[4] = {0,0,0,0};       ///< array to store parsed data
bool newData = false;            ///< flag to indicate if new data is available
int prevResetEnc = 0;            ///< previous reset encoder value
/** @} */

/*************************************************
 * \brief Vehicle constants
 ************************************************/
/**
 * @addtogroup vehicle Vehicle
 * @{
 */
const float WHEEL_RATIO = 555;                    ///< encoder ticks per metre (~0.00188496m/t)
const float WHEELBASE = 0.36;                     ///< distance between the two wheels in metres
const float HALF_VEHICLE_WIDTH = WHEELBASE / 2;   ///< half the wheelbase, in metres
const float MAX_SPEED = 1300 / WHEEL_RATIO;       ///< max speed in ticks per second
const int MAX_PWM = 250;                          ///< max PWM value for the motors

const int MIN_SPEED = 4; ///< minimum speed to move the robot in ticks per second (4 t/s ~= 8mm/s)
/** @} */

/*************************************************
 * \brief Variables for odometry calculations
 ************************************************/
/**
 * @addtogroup odo Odometry Calculations
 * @{
 */
long int encoderPosition[2] = {0, 0}; ///< Stores last encoder positions L and R

double leftDelta = 0;     ///< change in left encoder ticks
double rightDelta = 0;    ///< change in right encoder ticks

double newX = 0;        ///< new x position
double newY = 0;        ///< new y position
double x = 0;           ///< current x position
double y = 0;           ///< current y position
double dx = 0;          ///< change in x position
double dy = 0;          ///< change in y position 

float dist = 0;         ///< average distance travelled

long prevLeft = 0;      ///< previous left encoder position
long prevRight = 0;     ///< previous right encoder position
unsigned long prevTime = 0;     ///< previous time since last odometry update
unsigned long currentTime = 0;  ///< current time

float heading = 0;      ///< current heading in radians
float newHeading = 0;   ///< new heading in radians
float deltaHeading = 0; ///< change in heading


/**
 * @struct vec
 * @brief The struct `vec` represents a 2D vector with `x` and `y` components.
 * @var vec::x 
 * The property `x` in the `vec` struct represents the x-coordinate of a 2D vector.
 * @var vec::y
 * The `y` property in the `vec` struct represents the y-coordinate of a 2D vector.
 */
struct vec {
  double x;
  double y;
};

vec currPos = {0,0};    ///< current position of the robot
vec newPos = {0,0};     ///< new position of the robot
vec dPos = {0,0};       ///< change in position of the robot

vec targPos = {0,0};    ///< target position of the robot
vec targVel = {0,0};    ///< target velocity of the robot, where the x component is the left wheel and the y component is the right wheel
vec headingVec = {1,0}; ///< heading vector of the robot



double current_L = 0; ///< current left side velocity IN TICKS PER SECOND => 1m/s = 531 ticks/s
double current_R = 0; ///< current right side velocity IN TICKS PER SECOND => 1m/s = 531 ticks/s
double prev_Vel_L = 0; ///< previous left side velocity
double prev_Vel_R = 0; ///< previous right side velocity

/** @} */

// ===================================
// ===================================
// ========== PID constants ==========
/** \addtogroup PID PID Controller
 * @{
 */
const double Kp = 0.9; ///< Proportional constant
const double Ki = 0.8; ///< Integral constant
const double Kd = 0;   ///< Derivative constant | Keep this 0 
const double bias = 0; ///< Bias constant | Keep this 0

const long int PID_TIME = 20000; ///< μs | PID Controller update time

const float VELOCITY_ALPHA = 0.1; ///< alpha value for low pass filter for measured velocity
const float SETPOINT_ALPHA = 0.1; ///< alpha value for low pass filter for target setpoint velocity

const float DEST_THRESHOLD = 0.15; ///< threshold for destination reached | in metres
float dt; ///< time difference for PID update

// ===================================
// ===================================
// ===================================



// Error variables
double e_prev_L = 0;    ///< Previous left speed error
double e_prev_R = 0;    ///< Previous right speed error
double integral_L = 0;  ///< Integral term for left wheel
double integral_R = 0;  ///< Integral term for right wheel

float output_L = 0;    ///< Output for left wheel
float output_R = 0;    ///< Output for right wheel

long long int prev_L = 0; ///< Previous left encoder position
long long int prev_R = 0; ///< Previous right encoder position

// time variables
unsigned long prevTimePID = 0; ///< Previous time for PID update
unsigned long currTimePID = 0; ///< Current time for PID update
int driveData[3] = {0,0,0}; ///< Array to store drive data for the motors

/** @} */


/** \addtogroup circle Circle Tracking
 * @{
 */
long int dtCircle; ///< time difference for circle tracking
long int prevCircleTime = 0; ///< previous time for circle tracking

const long int CIRCLE_TIME = 500000; ///< μs | Circle tracking update time
const int CIRCLE_LOOPS = CIRCLE_TIME/PID_TIME; ///< number of loops for circle tracking
float AVERAGE_TARGET_VELOCITY = 0.5; ///<  m/s  | average target velocity for circle tracking

long int loop_no = 0; ///< loop number for circle tracking


int des_L = 0; ///< calculated desired left speed
int des_R = 0; ///< calculated desired right speed
/** @} */


//====================================

/**
 * @brief Main setup function
 */
void setup() {
  Serial.begin(500000);
  Serial.println("Enter data in this form <target_x, target_y, mode, resetEnc>");
  Serial.println("Coordinates in cm, mode: 0 = normal | 1 = off | 2 = brake");
  Serial.println("ResetEnc: when changed from 0 to 1, resets encoder positions");
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
/** 
 * @brief Main loop function
 */
void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
      strcpy(tempChars, receivedChars);
      parseDataPID();
      integral_L = 0;
      integral_R = 0;

      //with circle tracking thingy
      targPos.x = (double) dataRX[0] / 100;
      targPos.y = (double) dataRX[1] / 100;
      targVel = circle_track(currPos, targPos, headingVec, AVERAGE_TARGET_VELOCITY);
      des_L = lowPassFilter(targVel.x * WHEEL_RATIO, des_L, SETPOINT_ALPHA);
      des_R = lowPassFilter(targVel.y * WHEEL_RATIO, des_R, SETPOINT_ALPHA);

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
  
  if(dt > PID_TIME) {
    //printPID();
    //printVel();
    
    loop_no += 1;
    if(loop_no % 20 == 0){
      targVel = circle_track(currPos, targPos, headingVec, AVERAGE_TARGET_VELOCITY);
    }
    des_L = lowPassFilter(targVel.x * WHEEL_RATIO, des_L, SETPOINT_ALPHA);
    des_R = lowPassFilter(targVel.y * WHEEL_RATIO, des_R, SETPOINT_ALPHA);

    drive(driveData);
  }
}

//====================================

/**
 * @brief Receives a string of characters via Serial until a specific end marker is found.
 * @ingroup comm
 * This function listens for characters coming in via Serial communication, assembling them into a string.
 * The string assembly starts when a predefined start marker character is detected and ends when an end marker
 * character is received. The assembled string is stored in `receivedChars` and is made available once the end marker
 * is detected. This function uses static variables to maintain state between calls.
 * 
 * @note This function relies on global variables: `newData`, a boolean indicating if new data has been fully received,
 * and `receivedChars`, an array where the assembled string is stored. It also uses `numChars` to prevent buffer overflow,
 * ensuring that the assembled string does not exceed the size of `receivedChars`.
 * 
 * @note The function is designed to be called repeatedly; it processes one character per call until the end marker is detected.
 * 
 * @warning This function assumes that `Serial` has been initialized and is ready for reading.
 * 
 * Global Variables:
 * - `newData`: Set to true when a complete string is received, until it is reset externally.
 * - `receivedChars[]`: Array where the received string is stored.
 * - `numChars`: The maximum size of `receivedChars[]`.
 * 
 * Static Variables:
 * - `recvInProgress`: Tracks whether the reception of a string is currently in progress.
 * - `ndx`: The current index in `receivedChars[]` where the next character will be stored.
 * 
 * @see Serial.read()
 */
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

/**
 * @brief Parses the received data into its individual parts.
 * @ingroup comm
 * This function uses the `strtok()` function to split the received string into its individual parts.
 * The parts are then converted to integers and stored in the `dataRX[]` array.
 * 
 * @note This function assumes that the received string is in the format: "<int1,int2,int3,int4>"
 * 
 * Global Variables:
 * - `receivedChars[]`: The array containing the received string.
 * - `tempChars[]`: A temporary array used for parsing.
 * - `dataRX[]`: The array where the parsed data is stored.
 * - `newData`: A boolean flag indicating if new data is available.
 * 
 * Static Variables:
 * - `strtokIndx`: An index used by `strtok()` to keep track of the current position in the string.
 * 
 * @see strtok()
 */
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


  // Check if dataRX[2] is within 0 and 2
  if (dataRX[2] < 0 || dataRX[2] > 2) {
      dataRX[2] = 1;
  }

  // Reset distance if dataRX[3] is 1
  if (dataRX[3] != prevResetEnc) {
    // encoderPosition[0] = 0;
    // encoderPosition[1] = 0;
    dPos.x = 0;
    dPos.y = 0;
    currPos.x = 0;
    currPos.y = 0;
    heading = 0;
    deltaHeading = 0;
    targPos.x = 0;
    targPos.y = 0;
    des_L = 0;
    des_R = 0;
    prevResetEnc = dataRX[3];
  }
}


/**
 * @brief sends raw encoder data to the serial monitor
 */
void sendEncoder() {
  // Calculate and print encoder distances
  Serial.print("<");
  Serial.print(encoderPosition[0]);
  Serial.print(",");
  Serial.print(encoderPosition[1]);
  Serial.println(">");
}


/**
 * @brief Interrupt service routine for reading the right encoder
 * @ingroup odo
 */
void readEncoderR() {
  if (digitalRead(encoderRPinB) == LOW) {
    encoderPosition[0]--;
  } else {
    encoderPosition[0]++;
  }
}

/**
 * @brief Interrupt service routine for reading the left encoder
 * @ingroup odo
 */
void readEncoderL() {
  if (digitalRead(encoderLPinB) == LOW) {
    encoderPosition[1]++;
  } else {
    encoderPosition[1]--;
  }
}

/**
 * @brief Normalizes an angle to the range [0, 2π).
 * @ingroup odo
 * This function ensures that any given angle in radians is converted to an equivalent angle within the range [0, 2π).
 * It is useful for angle comparisons or operations where the angle needs to be within a single, continuous 360-degree cycle.
 * The normalization is done by adding or subtracting 2π from the angle until it falls within the desired range.
 * 
 * @param angle The input angle in radians to be normalized.
 * @return The normalized angle in radians, guaranteed to be between 0 (inclusive) and 2π (exclusive).
 */
float boundAngle(float angle) {
  while (angle < 0) {
    angle += (float)2 * M_PI;
  }
  while (angle >= 2 * M_PI) {
    angle -= (float)2 * M_PI;
  }
  return (float)angle;
}


/**
 * @brief Applies a low-pass filter to the input signal.
 * @ingroup PID
 * This function implements a simple low-pass filter, which is useful for smoothing out
 * high-frequency noise in a signal. The filter is applied to the current input value
 * based on the previous filtered value and a smoothing factor (alpha).
 * 
 * @param input The current raw input value to be filtered.
 * @param prev The previous filtered value.
 * @param alpha The smoothing factor used in the filter, between 0 and 1. A higher alpha
 *              value gives more weight to the current input, while a lower alpha value
 *              gives more weight to the previous values, resulting in stronger smoothing.
 * @return The filtered value.
 */
float lowPassFilter(float input, float prev, float alpha) {
  return alpha * input + (1 - alpha) * prev;
}


/**
 * @brief Clamps the input speed based on the target speed direction and limits.
 * @ingroup PID
 * This function adjusts the input speed to ensure it is in the correct direction relative to the target speed
 * and within the allowable range of speed values. If the target speed is positive, the input speed is clamped to
 * a minimum of 0 and a maximum of MAX_PWM. For a negative target speed, the input speed is clamped to a minimum of
 * -MAX_PWM and a maximum of 0. This ensures the input speed does not exceed the maximum allowed speed (MAX_PWM) and
 * is in the correct direction (positive or negative) as the target speed.
 * 
 * @param target The target speed, which determines the allowed direction of the input speed.
 * @param input The current input speed to be clamped based on the target speed and MAX_PWM.
 * @return The clamped speed, adjusted to be within the allowable range and direction.
 */
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

//====================================
// lookup feedforward values
/**
 * @brief Finds the motor configuration closest to the desired speed.
 * @ingroup PID
 * This function searches through a lookup table (lut) of motor configurations to find the one whose speed
 * is closest to a desired speed. It iterates through the lut, calculating the absolute difference between
 * the desired speed and each motor speed in the lut. The motor configuration with the smallest difference
 * is considered the closest match and is returned.
 * @note The LUT array must be defined and contain motor structures with speed values.
 * @note This function assumes that the LUT is sorted in ascending order of speed values.
 * @param desiredSpeed The target speed to match.
 * @return The motor configuration from the lut that is closest to the desired speed.
 */
motor lookupFF(float desiredSpeed) {
    motor closestMotor = lut[0];
    int smallestDifference = abs(desiredSpeed - lut[0].speed);

    for (int i = 1; i < sizeof(lut) / sizeof(lut[0]); i++) {
        int difference = abs(desiredSpeed - lut[i].speed);
        if (difference < smallestDifference) {
            smallestDifference = difference;
            closestMotor = lut[i];
        }
    }

    return closestMotor;
}


//==============================
// Calculate PID control
/**
 * @brief Implements a PID controller for velocity control.
 * @ingroup PID
 * This function calculates the PID control output based on the current and desired velocities for the left and right wheels.
 * It computes the error between the desired and current velocities, updates the integral term, and calculates the derivative term.
 * The PID control output is then determined using the proportional, integral, and derivative gains (Kp, Ki, Kd) along with a bias term and feedforward lookup table.
 * The output is clamped to ensure it does not exceed the maximum speed and is set to 0 if the desired speed is below a minimum threshold.
 * 
 * @note This function requires the `lookupFF()` function to find the feedforward values for the desired speeds.
 * @note The PID constants (Kp, Ki, Kd) and bias term can be adjusted to tune the controller's performance.
 * @note The function uses a low-pass filter to smooth the measured velocities and the desired setpoints.
 * 
 * @param desired_L The desired velocity for the left wheel.
 * @param desired_R The desired velocity for the right wheel.
 * 
 * @see lookupFF()
 * @see lowPassFilter()
 */
void PID(float desired_L, float desired_R) {
  // Calculate time difference
  currTimePID = micros();

  dt = (float)(currTimePID - prevTimePID);

  if(dt<PID_TIME){return;}
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

  float leftFF = lookupFF(desired_L).pwm;
  float rightFF = lookupFF(desired_R).pwm;
  //Serial.println(leftFF);
  //Serial.println(lookupFF(desired_L).speed);

  output_L = Kp * e_L + Ki * integral_L + Kd * derivative_L + bias + leftFF;
  output_R = Kp * e_R + Ki * integral_R + Kd * derivative_R + bias + rightFF;
  
  output_L = clampSpeed(desired_L, output_L);
  output_R = clampSpeed(desired_R, output_R);

  // if desired is less than MIN_SPEED, set output to 0
  output_L = (abs(desired_L) < MIN_SPEED) ? 0 : output_L;
  output_R = (abs(desired_R) < MIN_SPEED) ? 0 : output_R;

  
  driveData[0] = output_L;
  driveData[1] = output_R;
  driveData[2] = dataRX[2];
}


/**
 * @brief prints the current and desired velocities for the left and right wheels.
 * @ingroup PID
 */
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


/**
 * @brief prints the current and desired velocities for the left and right wheels along with PID values.
 * @ingroup PID
 */
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


/**
 * @brief Calculates motor speeds for tracking a circular path towards a target point.
 * @ingroup circle
 * This function computes the speeds for the left and right motors of a vehicle to make it follow a circular path
 * towards a target point. It uses the current position (s), target position (t), and heading vector (h) to determine
 * the direction and magnitude of the required turn. The function calculates the distance to the target and uses it
 * along with the desired velocity (vel) to compute the appropriate motor speeds. If the target is within a small
 * tolerance, it stops the motors. Otherwise, it determines whether to turn left or right based on the cross product
 * of the heading and the direction to the target. It then calculates the turning radius and adjusts the motor speeds
 * accordingly to achieve the desired circular path. The function ensures that the vehicle turns in the most efficient
 * manner to reach the target point.
 * 
 * @note This function uses the `HALF_VEHICLE_WIDTH` constant to determine the turning radius based on the vehicle's dimensions.
 * 
 * @param s The current position of the vehicle as a vector.
 * @param t The target position as a vector.
 * @param h The current heading of the vehicle as a vector.
 * @param vel The desired velocity towards the target.
 * @return A vector representing the speeds for the left (x component) and right (y component) motors.
 */
vec circle_track(vec s, vec t, vec h, float vel){
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
  float calcTime = sqrt(d_mag_sq) / vel; 
  //rough speed that we should be targeting, based on the heureustic that we want to arrive in ~1s. NOTE THAT THE ROUNDING IS _MANDATORY_. DO NOT! REMOVE!!
  float speed = round(sqrt(d_mag_sq) * 5) / (5 * calcTime);
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
/**
 * @brief Updates the odometry information of the robot.
 * @ingroup odo
 * This function calculates the new position and heading of the robot based on the encoder readings. It updates
 * the robot's current position (currPos), heading, and the change in position (dPos) and heading (deltaHeading)
 * since the last update. The calculations differentiate between straight movement and turning by checking the
 * difference in encoder values for the left and right wheels. For straight movement, the position update is
 * straightforward. For turning, it calculates the turning radius and adjusts the position and heading accordingly.
 * The function ensures that the heading is always normalized to the range [0, 2π) using the boundAngle function.
 * 
 * @note This function uses the `WHEEL_RATIO` constant to convert encoder ticks to metres and the `WHEELBASE` constant
 * to determine the distance between the two wheels. It also uses the `headingVec` vector to represent the heading
 * 
 * @see boundAngle()
 */
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

/**
 * @brief sends the odometry information to the serial monitor.
 * @ingroup odo
 */
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

/**
 * @brief sends the current position, target position, and target velocity to the serial monitor.
 * @ingroup circle
 */
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


//eg. <50,50,0,0>
//MODE 0 = normal, 1 = off, 2 = brake
/**
 * @brief Controls the driving mechanism of a robot based on input commands.
 * @ingroup vehicle
 * This function interprets an array of integers to control the driving mechanism of a robot, adjusting the speed and
 * direction of its motors. The input array contains values for left and right motor PWM (Pulse Width Modulation) signals
 * and a mode selector. The function supports three modes: normal operation, off, and brake. In normal operation, the
 * PWM values directly control the motor speeds, allowing for forward and reverse motion. The off mode stops all motor
 * activity, and the brake mode actively halts the motors, which can cause voltage spikes and should be used cautiously.
 * 
 * @warning The brake mode can cause voltage spikes and should be used with caution to avoid damaging the motors or other components.
 * 
 * The function uses `digitalWriteFast` to set the direction of the motors and `analogWrite` to control the speed. It
 * ensures that the motors are stopped if both PWM values are zero, regardless of the selected mode. This function is
 * designed for robots with differential drive systems, where the speed and direction of each side (left and right) can
 * be controlled independently.
 * 
 * @param data An array of integers where:
 *        - data[0] is the PWM value for the left motor (positive for forward, negative for reverse | range: -255 to 255),
 *        - data[1] is the PWM value for the right motor (positive for forward, negative for reverse | range: -255 to 255),
 *        - data[2] is the mode selector (0 for normal operation, 1 for off, 2 for brake).
 */
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


