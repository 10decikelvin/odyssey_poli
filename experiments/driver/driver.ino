//safety
const int MAX_SAFE_PWM_OUTPUT = 230; //technically this is 255, but we want to care about safety

//serial
const byte maxCharLength = 32;
char _incomingChars[maxCharLength];
boolean newData = false;
// Command
char command[maxCharLength] = {0};
float param0 = 0.0;
float param1 = 0.0;
//motor pin definitions
const int motorLA = 12;
const int motorLB = 11;
const int motorPWMLA = 10;
const int motorPWMLB = 9;

const int motorRA = 7;
const int motorRB = 4;
const int motorPWMRA = 6;
const int motorPWMRB = 5;
void setup() {
    Serial.begin(115200);
    Serial.println("AUTOPOWERCORRECT: ENABLED. Command format: <string, float, float>");

    //set pin mode
    pinMode(motorLA, OUTPUT);
    pinMode(motorLB, OUTPUT);
    pinMode(motorRA, OUTPUT);
    pinMode(motorRB, OUTPUT);
    pinMode(motorPWMLA, OUTPUT);
    pinMode(motorPWMLB, OUTPUT);
    pinMode(motorPWMRA, OUTPUT);
    pinMode(motorPWMRB, OUTPUT);
}

//============

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        newData = false;
        char * strtokIndx;

        //Parse Command
        strtokIndx = strtok(_incomingChars,",");
        strcpy(command, strtokIndx);
        strtokIndx = strtok(NULL, ","); 
        param0 = atof(strtokIndx);
        strtokIndx = strtok(NULL, ",");
        param1 = atof(strtokIndx);

        //process commands
        
        if(strcmp("brake", command) == 0){
          brake();
          Serial.println("BRAKE: OK");
        }else if(strcmp("off", command) == 0){
          off();
          Serial.println("BRAKE: OK");
        }else if(strcmp("move", command) == 0){
          move(param0, param1);
        }else if(strcmp("right", command) == 0){
          right(param0, param1);
          Serial.println("RIGHT: OK");
        }else if(strcmp("left", command) == 0){
          left(param0, param1);
          Serial.println("LEFT: OK");
        }else{
          Serial.print("ERR: Unknown Command ");
          Serial.println(command);
        }
    }
}

//============

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
                _incomingChars[ndx] = rc;
                ndx++;
                if (ndx >= maxCharLength) {
                    ndx = maxCharLength - 1;
                }
            }
            else {
                _incomingChars[ndx] = '\0'; // terminate the string
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

//motor functions
void off(){
    // Enable pins are low during off() to coast
    analogWrite(motorPWMLA,0);
    analogWrite(motorPWMLB,0);
    analogWrite(motorPWMRA,0);
    analogWrite(motorPWMRB,0);

    digitalWrite(motorLA,0);
    digitalWrite(motorLB,0);
    digitalWrite(motorRA,0);
    digitalWrite(motorRB,0);
}

void brake(){
    //enable pins are high when braking
    digitalWrite(motorLA,1);
    digitalWrite(motorLB,1);
    digitalWrite(motorRA,1);
    digitalWrite(motorRB,1);

    analogWrite(motorPWMLA,0);
    analogWrite(motorPWMLB,0);
    analogWrite(motorPWMRA,0);
    analogWrite(motorPWMRB,0);
}

bool move(float lspd, float rspd){
    if(lspd > 1 || lspd < -1) return false;
    if(rspd > 1 || rspd < -1) return false;

    float corrected_lspd = speed2pwm1(lspd);
    float corrected_rspd = speed2pwm2(rspd);
    Serial.print("MOVE: OK; ");
    Serial.print(lspd);
    Serial.print(" ");
    Serial.print(rspd);
    Serial.print(" > ");
    Serial.print(corrected_lspd);
    Serial.print(" ");
    Serial.println(corrected_rspd);

    left(1.0, corrected_lspd);
    right(1.0, corrected_rspd);
    return true;
}

void left(float enablePin, float spd){
  int pwm = abs(int(spd * MAX_SAFE_PWM_OUTPUT));
  if(enablePin == 0.0){
    digitalWrite(motorRA,0);
    digitalWrite(motorRB,0);
  }else if(spd < 0){
    digitalWrite(motorRA,1);
    digitalWrite(motorRB,1);
    analogWrite(motorPWMRA, pwm);
    analogWrite(motorPWMRB,0);
  }else{
    digitalWrite(motorRA,1);
    digitalWrite(motorRB, 1);
    analogWrite(motorPWMRA,0);
    analogWrite(motorPWMRB, pwm);
  }
}

void right(float enablePin, float spd){
  int pwm = abs(int(spd * MAX_SAFE_PWM_OUTPUT));
  if(enablePin == 0.0){
    digitalWrite(motorLA,0);
    digitalWrite(motorLB,0);
  }else if(spd < 0){
    digitalWrite(motorLA,1);
    digitalWrite(motorLB,1);
    analogWrite(motorPWMLA, pwm);
    analogWrite(motorPWMLB,0);
  }else{
    digitalWrite(motorLA,1);
    digitalWrite(motorLB,1);
    analogWrite(motorPWMLA,0);
    analogWrite(motorPWMLB, pwm);
  }
}

float clip(float x){
  if(x>1){
    return 1;
  }
  if(x<0){
    return 0;
  }
  return x;
}

// converts a speed (0-1) to a corrected power rate (0-1) for motor 1
float speed2pwm1(float desiredSpeed){
  if(desiredSpeed > 0){
    return clip(abs(desiredSpeed));
  }else{
    return -clip(abs(desiredSpeed));
  }
}

float speed2pwm2(float desiredSpeed){
  if(desiredSpeed > 0){
    return clip(abs(desiredSpeed) * 0.878 - 0.094);
  }else{
    return -clip(abs(desiredSpeed) * 0.878 - 0.094);
  }
}