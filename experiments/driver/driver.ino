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
const int motorLA = 13;
const int motorLB = 12;
const int motorRA = 8;
const int motorRB = 9;

const int motorPWMLA = 11;
const int motorPWMLB = 6;
const int motorPWMRA = 10;
const int motorPWMRB = 5;
const int Isense = 7;
void setup() {
    Serial.begin(9600);
    Serial.println("Command format: <string, float, float>");

    //set pin mode
    pinMode(motorLA, OUTPUT);
    pinMode(motorLB, OUTPUT);
    pinMode(motorRA, OUTPUT);
    pinMode(motorRB, OUTPUT);
    pinMode(motorPWMLA, OUTPUT);
    pinMode(motorPWMLB, OUTPUT);
    pinMode(motorPWMRA, OUTPUT);
    pinMode(motorPWMRB, OUTPUT);

    pinMode(Isense, INPUT);

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
          Serial.println("MOVE: OK");
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
    left(1.0, lspd);
    right(1.0, rspd);
    return true;
}

void left(float enablePin, float spd){
  int pwm = abs(int(spd * MAX_SAFE_PWM_OUTPUT));
  if(enablePin == 1.0){
    digitalWrite(motorRA,1);
    digitalWrite(motorRB,1);
  }else{
    digitalWrite(motorRA,0);
    digitalWrite(motorRB,0);
  }
  if(spd > 0){
    analogWrite(motorPWMRA, pwm);
    analogWrite(motorPWMRB,0);
  }else{
    analogWrite(motorPWMRA,0);
    analogWrite(motorPWMRB, pwm);
  }
}

void right(float enablePin, float spd){
  int pwm = abs(int(spd * MAX_SAFE_PWM_OUTPUT));
  if(enablePin == 1.0){
    digitalWrite(motorLA,1);
    digitalWrite(motorLB,1);
  }else{
    digitalWrite(motorLA,0);
    digitalWrite(motorLB,0);
  }
  if(spd > 0){
    analogWrite(motorPWMLA, pwm);
    analogWrite(motorPWMLB,0);
  }else{
    analogWrite(motorPWMLA,0);
    analogWrite(motorPWMLB, pwm);
  }
}