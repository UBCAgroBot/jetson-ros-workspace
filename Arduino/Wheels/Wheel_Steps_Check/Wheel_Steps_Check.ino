/*Test code for the stepper motor
 *Pin 5 on arduino -> PUL+ on the controller
 *Pin 4 on arduino -> DIR+ on the controller
 *ENA is disconnected
 * DIR- and PUL- is connected to ground
 * Arduino should be conntected to ground
*/

const int pulse1 = 51;
const int dir1 = 49;
const int pulse2 = 50;
const int dir2 = 48;

const int spd = 3;

const int turn_left = 34;
const int turn_right = 35;

const int leftDirection = 0;
const int rightDirection = 1;

int isLeftOn = 0;
int isRightOn = 0;

//testing with LED
const int ledPin1 = 3;
const int ledPin2 = 5;

int counter = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(pulse1, OUTPUT);
  pinMode(dir1, OUTPUT); 
  pinMode(pulse2, OUTPUT);
  pinMode(dir2, OUTPUT);

  pinMode(turn_left, INPUT);
  pinMode(turn_right, INPUT);
  Serial.begin(9600);
}

void loop() {
//  // read the state of the pushbutton value:
//  isLeftOn = digitalRead(turn_left);
//  isRightOn = digitalRead(turn_right);
//
//  // check if the pushbutton is pressed. If it is, the buttonState is HIGH:
//  if (isLeftOn == HIGH) {
//   turnDirection(leftDirection);
//    digitalWrite(ledPin1, HIGH);
//    digitalWrite(ledPin2, LOW);
//  } else if(isRightOn == HIGH){
//    turnDirection(rightDirection);
//    digitalWrite(ledPin1, LOW);
//    digitalWrite(ledPin2, HIGH);
//  }
//
//  if(loops < 10){
//    if(counter < 400){
//     turnDirection(leftDirection);
//     counter++;
//    }
//    else{
//      counter = 0;
//      loops++;
//    }
//  }
//  Serial.print("Counter: ");
//  Serial.println(counter);
//  Serial.print("Loops: ");
//  Serial.println(loops);

  if(counter < 400){
    turnDirection(leftDirection);
  }
  counter++;
  Serial.println(counter);
}

void turnDirection(int dir) {
   digitalWrite(dir1, dir);
   digitalWrite(dir2, dir);
   pulseGen(spd);
}

void pulseGen(long del){
  digitalWrite(pulse1, HIGH);
  digitalWrite(pulse2, HIGH);
  delay(del);
  digitalWrite(pulse1, LOW);
  digitalWrite(pulse2, LOW);
  delay(del);
}
