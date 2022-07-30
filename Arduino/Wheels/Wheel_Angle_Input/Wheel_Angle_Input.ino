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

const int spd = 7;

const int turn_left = 34;
const int turn_right = 35;

const int leftDirection = 0;
const int rightDirection = 1;

int isLeftOn = 0;
int isRightOn = 0;

//testing with LED
const int ledPin1 = 3;
const int ledPin2 = 5;

float myF;

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
  if(Serial.available()){
    myF = Serial.parseFloat();
    Serial.println(myF);
    int steps = int(map(abs(myF), 0, 360, 0, 400));
    for (int i = 0; i < steps; i++) {
      if(myF>0){
        turnDirection(leftDirection);
      } else {
        turnDirection(rightDirection);
      }
      Serial.println(myF);
      Serial.println(i);
    }
  }
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
