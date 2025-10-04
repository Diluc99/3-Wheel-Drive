int M1_Dir = 4;
int M1_Pwm = 2;
int M2_Dir = 5;
int M2_Pwm = 3;
int M3_Dir = 21;
int M3_Pwm = 23;
void setup() {
  // put your setup code here, to run once:
  pinMode(M1_Pwm, OUTPUT);
  analogWrite(M1_Pwm, 0);
  pinMode(M1_Dir, OUTPUT);
  pinMode(M2_Pwm, OUTPUT);
  analogWrite(M2_Pwm, 0);
  pinMode(M2_Dir, OUTPUT);
  pinMode(M3_Pwm, OUTPUT);
  analogWrite(M3_Pwm, 0);
  pinMode(M3_Dir, OUTPUT);
  Serial.begin(115200);
  Serial.println("Enter Pwm value: ");
}

void loop() {
  // put your main code here, to run repeatedly:
  int speed = Serial.parseInt();
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(M3_Dir, HIGH);
    analogWrite(M3_Pwm, speed);
  } else if(speed<0) {
    digitalWrite(M3_Dir, LOW);
    analogWrite(M3_Pwm, abs(speed));
  }else{
    digitalWrite(M3_Dir, LOW);
    analogWrite(M3_Pwm,0);
  }
}

