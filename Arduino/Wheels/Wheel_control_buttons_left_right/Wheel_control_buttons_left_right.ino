/*Test code for the stepper motor, 2 Button control for left and right
 * Pin 5 on arduino -> PUL+ on the controller
 * Pin 4 on arduino -> DIR+ on the controller
 * ENA is disconnected
 * DIR- and PUL- is connected to ground
 * Arduino should be conntected to ground
 * Left Button -> Pin 34
 * Right Button -> Pin 35
*/

// PUL+ and DIR+ pins
const int pulse1 = 51;
const int dir1 = 49;
const int pulse2 = 50;
const int dir2 = 48;

// Angular Speed, recommended setting 7, DO NOT DECREASE TO < 3
const int spd = 7;

// LEFT and RIGHT pins
const int turn_left = 34;
const int turn_right = 35;

// LEFT and RIGHT output values to motor
const int leftDirection = 0;
const int rightDirection = 1;

// Bools for If button is pressed
int isLeftOn = 0;
int isRightOn = 0;

void setup() {
  // put your setup code here, to run once:

  // Wheel setup
  pinMode(pulse1, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(pulse2, OUTPUT);
  pinMode(dir2, OUTPUT);

  // button setup
  pinMode(turn_left, INPUT);
  pinMode(turn_right, INPUT);
  Serial.begin(9600);
}

void loop() {
  // read the state of the pushbutton value:
  isLeftOn = digitalRead(turn_left);
  isRightOn = digitalRead(turn_right);

  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
  if (isLeftOn == HIGH) {
    turnDirection(leftDirection);
  } else if(isRightOn == HIGH){
    turnDirection(rightDirection);
  }
}

void turnDirection(int dir) {
   digitalWrite(dir1, dir);
   digitalWrite(dir2, dir);
   pulseGen(spd);
}

// Needed for Stepping Up and Down
void pulseGen(long del){
  digitalWrite(pulse1, HIGH);
  digitalWrite(pulse2, HIGH);
  delay(del);
  digitalWrite(pulse1, LOW);
  digitalWrite(pulse2, LOW);
  delay(del);
}