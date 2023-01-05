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
const int GFL_DIR = 30;
const int GFL_PWM = 10;

const int GFR_DIR = 42;
const int GFR_PWM = 11;

const int GBL_DIR = 32;
const int GBL_PWM = 8;

const int GBR_DIR = 26;
const int GBR_PWM = 9;

const int SL_DIR = 48;
const int SL_PULSE = 50;

const int SR_DIR = 49;
const int SR_PULSE = 51;

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

  // start serial communication, logger
  Serial.begin(9600);
}

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

// 1 pulse to angle
// 400 revs per 360 degrees
const float REV_ANGLE = 360.0 / 400.0;

// max turning angle
const float MAX_ANGLE = 90.0;

// enum for vertical directions
enum v_directions {
  halted,
  forward,
  backward
};

// enum for horizontal directions
enum h_directions {
  straight,
  left,
  right
};

// variable for stepper direction [left, right]
enum h_directions angular_dir = straight;

// variable for gearbox direction [forward, backward]
enum v_directions movement_dir = halted;

// current_angle of stepper motor
float current_angle = 0.0;

// input string buffer
char buf[80];

void loop() {
  bool is_valid_input = true;
  
  // to send serial data, use the Serial Monitor tool
  if (readline(Serial.read(), buf, 80) > 0) {
    is_valid_input = update_dir_enums(buf);
  }
  if (is_valid_input) {
    run();
  }
}

// Read a line of text from serial input
int readline(int readch, char *buffer, int len) {
    static int pos = 0;
    int rpos;

    if (readch > 0) {
        switch (readch) {
            case '\r': // Ignore CR
                break;
            case '\n': // Return on new-line
                rpos = pos;
                pos = 0;  // Reset position index ready for next time
                return rpos;
            default:
                if (pos < len-1 && readch > 0 && readch < 128) {
                    //Serial.println(readch);
                    buffer[pos++] = readch;
                    buffer[pos] = 0;
                }
        }
    }
    return 0;
}

// update direction enums
// returns true if input is a valid value, 
// othwerise returns false
int update_dir_enums(String input) {
  switch (input.charAt(0)) {
    case 'H':
      halt();
      break;
    case 'F':
      set_forward();
      break;
    case 'B':
      set_backward();
      break;
    default:
      return false;
  }
      
  switch (input.charAt(1)) {
    case 'L':
      set_left();
      break;
    case 'S':
      reset_angle();
      break;
    case 'R':
      set_right();
      break;
    default:
      return false;
  }
  return true;
}

// sets direction value to desired value if it is not already set to that value,
// otherwise, interpret action as an instruction to stop, and set to undefined
void set_forward() {
  movement_dir = forward;
}

void set_left() {
  angular_dir = left;
}

void set_backward() {
  movement_dir = backward;
}

void set_right() {
  angular_dir = right;
}

// resets the stepper motor angle to 0 degrees
void reset_angle() {
  int dir;
  float inc_value;

  // find direction to turn to
  if (current_angle < 0) {
    dir = DIR_RIGHT;
    inc_value = REV_ANGLE;
  } else {
    dir = DIR_LEFT;
    inc_value = -REV_ANGLE;
  }

  // halt angular directions
  angular_dir = straight;

  // TODO: investigate error with this line
  // Serial.println("Straight");
  Serial.println("Reseting angle");
  const float turn_angle = abs(current_angle) / REV_ANGLE;
  for (int i = 0; i < turn_angle; i++) {
    turn_wheels(dir);
    current_angle += inc_value;
  }
  Serial.println("Reset complete");
}

// halt any actions
void halt() {
  movement_dir = halted;
  Serial.println("Halt");
}

// manipulate motors based on angular_dir and movement_dir values
void run() {
  if (angular_dir == left) {
    if (current_angle <= -MAX_ANGLE) {
      Serial.println("Max angle reached");
    } else {
      Serial.println("Turning left");
      current_angle -= REV_ANGLE;
      turn_wheels(DIR_LEFT);
    }
  } else if (angular_dir == right) {
    if (current_angle >= MAX_ANGLE) {
      Serial.println("Max angle reached");
    } else {
      Serial.println("Turning right");
      current_angle += REV_ANGLE;
      turn_wheels(DIR_RIGHT);
    }
  }

  if (movement_dir == forward) {
    Serial.println("Moving forward");
    rotate_wheels(DIR_FORWARD);
  } else if (movement_dir == backward) {
    Serial.println("Moving backward");
    rotate_wheels(DIR_BACKWARD);
  }

  // stop pwm
  if (movement_dir == halted) {
    generatePWM(0);
  }
}

// turns stepper motor in desired direction
void turn_wheels(int dir) {
  // set direction of stepper motors
  digitalWrite(SL_DIR, dir);
  digitalWrite(SR_DIR, dir);

  // generate pulse
  generatePulse();
}

// generates pulse for stepper motors
void generatePulse() {
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
  // the motors on the right side are flipped, hence !dir
  digitalWrite(GFL_DIR, dir);
  digitalWrite(GFR_DIR, !dir);
  digitalWrite(GBL_DIR, dir);
  digitalWrite(GBR_DIR, !dir);

  // generate pwm
  generatePWM(PWM_SPEED);
}

// generate pwm for gearbox motors
void generatePWM(int spd) {
  analogWrite(GFL_PWM, spd);
  analogWrite(GFR_PWM, spd);
  analogWrite(GBL_PWM, spd);
  analogWrite(GBR_PWM, spd);
}
