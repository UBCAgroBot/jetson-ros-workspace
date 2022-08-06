/* 
 *  Code for controlling the robot using the keyboard
 *  DOCS for Gearbox Motor Driver: https://images-na.ssl-images-amazon.com/images/I/A1TemgvjKjL.pdf
 *  DOCS for Stepper Motor Driver: https://www.steppermotorcanada.ca/dm860h.pdf
 *  
 *  PWM: Analog
 *  DIR: Digital
 *  PULSE: Digital
 */

// Define pins for motors
// G: Gearbox Motor
// S: Stepper Motor
// F: Front
// B: Back
// L: Left
// R: Right
const int GFL_DIR = 32;
const int GFL_PWM = 8;

const int GFR_DIR = 32;
const int GFR_PWM = 8;

const int GBL_DIR = 32;
const int GBL_PWM = 8;

const int GBR_DIR = 32;
const int GBR_PWM = 8;

const int SL_DIR = 49;
const int SL_PULSE = 51;

const int SR_DIR = 48;
const int SR_PULSE = 50;

void setup() {
  // setup pin modes
  pinMode(GFL_DIR, OUTPUT);
  pinMode(GFL_PWM, OUTPUT);

  pinMode(GFR_DIR, OUTPUT);
  pinMode(GFR_PWM, OUTPUT);

  pinMode(GBL_DIR, OUTPUT);
  pinMode(GBL_PWM, OUTPUT);

  pinMode(GBR_DIR, OUTPUT);
  pinMode(GBR_PWM, OUTPUT);

  pinMode(SL_DIR, OUTPUT);
  pinMode(SL_PULSE, OUTPUT);

  pinMode(SR_DIR, OUTPUT);
  pinMode(SR_PULSE, OUTPUT);

  // start logger
  Serial.begin(9600);
}

// enum for directions
enum directions {
  undefined,
  forward,
  left,
  backward,
  right
};

// variable for stepper direction [left, right]
enum directions angular_dir = undefined;

// variable for gearbox direction [forward, backward]
enum directions movement_dir = undefined;

// LEFT and RIGHT output values for stepper motor
const int DIR_RIGHT = 1;
const int DIR_LEFT = 0;

// FORWARD and BACKWARD output values for gearbox motor
const int DIR_FORWARD = 1;
const int DIR_BACKWARD = 0;

// Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
const int ANGULAR_SPEED = 7;

// Strength of PWM, between 0 and 255, higher is faster
const int PWM_SPEED = 100;

void loop() {
  bool is_valid_input = true;
  
  // to send serial data, use the Serial Monitor tool
  if (Serial.available()) {
    is_valid_input = update_dir_enums(Serial.read());
  }
  if (is_valid_input) {
    run();
  }
}

// update direction enums
// returns true if input is a valid value, 
// othwerise returns false
int update_dir_enums(int input) {
  switch (input) {
    case 'w':
      setForward();
      break;
    case 'a':
      setLeft();
      break;
    case 's':
      setBackward();
      break;
    case 'd':
      setRight();
      break;
    case 'h':
      halt();
      break;
    default:
      return false;
  }
  return true;
}

// sets direction value to desired value if it is not already set to that value,
// otherwise, interpret action as an instruction to stop, and set to undefined

void setForward() {
  movement_dir = movement_dir == forward ? undefined : forward;
}

void setLeft() {
  angular_dir = angular_dir == left ? undefined : left;
}

void setBackward() {
  movement_dir = movement_dir == backward ? undefined : backward;
}

void setRight() {
  angular_dir = angular_dir == right ? undefined : right;
}

// halt any actions
void halt() {
  movement_dir = undefined;
  angular_dir = undefined;
}

// manipulate motors based on angular_dir and movement_dir values
void run() {
  if (angular_dir == left) {
    Serial.println("Turning left");
    turn_wheels(DIR_LEFT);
  } else if (angular_dir == right) {
    Serial.println("Turning right");
    turn_wheels(DIR_RIGHT);
  }

  if (movement_dir == forward) {
    Serial.println("Moving forward");
    rotate_wheels(DIR_FORWARD);
  } else if (movement_dir == backward) {
    Serial.println("Moving backward");
    rotate_wheels(DIR_BACKWARD);
  }
}

// turns stepper motor in desired direction
void turn_wheels(int dir) {
  // set direction of stepper motors
  digitalWrite(SL_DIR, dir);
  digitalWrite(SR_DIR, dir);

  // generate pulse
  digitalWrite(SL_PULSE, HIGH);
  digitalWrite(SR_PULSE, HIGH);
  delay(ANGULAR_SPEED);
  digitalWrite(SL_PULSE, LOW);
  digitalWrite(SR_PULSE, LOW);
  delay(ANGULAR_SPEED);
}

// turns gearbox motor in desired direction
void rotate_wheels(int dir) {
  // set direction of gearbox motors
  digitalWrite(GFL_DIR, dir);
  digitalWrite(GFR_DIR, dir);
  digitalWrite(GBL_DIR, dir);
  digitalWrite(GBR_DIR, dir);

  // generate pwm
  analogWrite(GFL_PWM, PWM_SPEED);
  analogWrite(GFR_PWM, PWM_SPEED);
  analogWrite(GBL_PWM, PWM_SPEED);
  analogWrite(GBR_PWM, PWM_SPEED);
}
