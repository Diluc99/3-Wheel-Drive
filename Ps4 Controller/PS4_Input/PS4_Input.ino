#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
BluetoothController bluetooth(myusb);
JoystickController joystick1(myusb);

// Joystick values
int leftX = 0, leftY = 0;
int rightX = 0, rightY = 0;
bool controllerConnected = false;

float Vx=0.0,Vy=0.0,omega=0.0;
float V1=0.0,V2=0.0,V3=0.0;
//Motor Pins
int M1_Dir = 4;
int M1_Pwm = 2;
int M2_Dir = 5;
int M2_Pwm = 3;
int M3_Dir = 21;
int M3_Pwm = 23;
const float MaxSpeed=0.05; // 80% max speed for safety

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {}

  Serial.println("Waiting for controller...");
  
  myusb.begin();
  pinMode(M1_Pwm, OUTPUT);
  pinMode(M1_Dir, OUTPUT);
  pinMode(M2_Pwm, OUTPUT);
  pinMode(M2_Dir, OUTPUT);
  pinMode(M3_Pwm, OUTPUT);
  pinMode(M3_Dir, OUTPUT);
  
}

void loop() {
  myusb.Task();
  
  if (joystick1.available()) {
    if (!controllerConnected) {
      controllerConnected = true;
      Serial.println(" PS4 Controller Connected via Bluetooth!");
    }
    // Read joystick values
    readJoysticks();
    calcSpeed(); 
    // Print values
    printJoystickValues(); // only for degugging
    
    joystick1.joystickDataClear();
  } else {
    if (controllerConnected) {
      controllerConnected = false;
      Serial.println(" Controller disconnected");
      MotorDirSetting(M1_Pwm, M1_Dir, 0);
      MotorDirSetting(M2_Pwm, M2_Dir, 0);
      MotorDirSetting(M3_Pwm, M3_Dir, 0);
      
    }
  }
  
  delay(50);
}
void MotorDirSetting(int pwmPin, int dirPin, float speed) {
  speed = constrain(speed, -1, 1);
  if (speed > 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, (int)(speed * 255));
  } else if (speed < 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, (int)(-speed * 255));
  }
  else{
    analogWrite(pwmPin, 0);
  }
}
void MotorDirSettingM3(int pwmPin, int dirPin, float speed) {
  speed = constrain(speed, -1, 1);
  if (speed > 0) {
    digitalWrite(dirPin, LOW);
    analogWrite(pwmPin, (int)(speed * 255));
  } else if (speed < 0) {
    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, (int)(-speed * 255));
  }
  else{
    analogWrite(pwmPin, 0);
  }
}
void readJoysticks() {
  // Read analog sticks
  // Axis mapping for PS4 controller:
  leftX = joystick1.getAxis(0)-128;   // Left stick X
  leftY = joystick1.getAxis(1)-128;   // Left stick Y
  rightX = joystick1.getAxis(2)-128;  // Right stick X
  //rightY = joystick1.getAxis(4);  // Right stick Y
  Vx = constrain(-leftY / 128.0, -1.0, 1.0);  // Forward/backward (invert if needed)
  Vy = constrain(leftX / 128.0, -1.0, 1.0);   // Left/right
  omega = constrain(rightX / 128.0, -1.0, 1.0); // Rotation
 
  if (abs(Vx) < 0.05) Vx = 0;
  if (abs(Vy) < 0.05) Vy = 0;
  if (abs(omega) < 0.05) omega = 0;

}

 void calcSpeed(){  // formula derived using IK
V1 = 0.0 * Vx + 1.0 * Vy + omega;      // M1 at 0°
V2 = -0.866 * Vx - 0.5 * Vy + omega;   // M2 at 120°  
V3 = 0.866 * Vx - 0.5 * Vy + omega;    // M3 at 240°

 /*float maxVal = max(max(abs(V1), abs(V2)), abs(V3));
  if (maxVal > 1.0) {
    V1 /= maxVal;
    V2 /= maxVal;
    V3 /= maxVal;
  }*/
   // Apply additional % limit to final wheel speeds (redundant safety)
  V1 *= MaxSpeed;
  V2 *= MaxSpeed;
  V3 *= MaxSpeed;
  MotorDirSetting(M1_Pwm, M1_Dir, V1);
  MotorDirSetting(M2_Pwm, M2_Dir, V2);
  MotorDirSettingM3(M3_Pwm, M3_Dir, V3);

}
void printJoystickValues() {
  static unsigned long lastPrint = 0;
  
  // Print every 200ms to avoid spam
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    
   /* Serial.print("LEFT: (");
    Serial.print(leftX);
    Serial.print(", ");
    Serial.print(leftY);
    Serial.print(") ");*/
    Serial.print("Vx: (");
    Serial.print(Vx);
    Serial.print(", ");
    Serial.print(") ");
    
    /*Serial.print("RIGHT: (");
    Serial.print(rightX);
    Serial.print(", ");
    Serial.print(rightY);
    Serial.print(")");*/
    Serial.print("Vy: (");
    Serial.print(Vy);
    Serial.print(", ");
    Serial.println(") ");
    Serial.print("V1: (");
    Serial.print(V1);
    Serial.print(", ");
    Serial.print(") ");
    Serial.print("V2: (");
    Serial.print(V2);
    Serial.print(", ");
    Serial.print(") ");
    Serial.print("V3: (");
    Serial.print(V3);
    Serial.print(", ");
    Serial.print(") ");
  }
}