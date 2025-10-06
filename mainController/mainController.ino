#include <USBHost_t36.h>
// USB Host and device declarations
USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
BluetoothController bluetooth(myusb); // connection mode
JoystickController joystick1(myusb);

// Right analog stick coordinates
int leftX = 0, leftY = 0; // Left analog stick coordinates
int rightX = 0, rightY = 0; // Right analog stick coordinates
bool controllerConnected = false;

float Vx=0.0,Vy=0.0,omega=0.0;  // // Robot velocities: Vx(forward/back), Vy(strafing), omega(rotation)
float V1=0.0,V2=0.0,V3=0.0; // Individual wheel speeds
//Motor Pins
int M1_Dir = 4;
int M1_Pwm = 2;
int M2_Dir = 5;
int M2_Pwm = 3;
int M3_Dir = 21;
int M3_Pwm = 23;
const float MaxSpeed=0.35; // Maximum speed limit (35% of full speed for safety)

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {}  // Wait for serial connection

  Serial.println("Waiting for controller...");
  
  myusb.begin();   // Initialize USB host
  // Initialize motor control pins
  pinMode(M1_Pwm, OUTPUT);
  analogWrite(M1_Pwm,0); // Start with motors stopped
  pinMode(M1_Dir, OUTPUT);
  pinMode(M2_Pwm, OUTPUT);
  analogWrite(M2_Pwm,0);// Start with motors stopped
  pinMode(M2_Dir, OUTPUT);
  pinMode(M3_Pwm, OUTPUT);
  analogWrite(M3_Pwm,0);// Start with motors stopped
  pinMode(M3_Dir, OUTPUT);
  
}
void loop() {
  myusb.Task();  // Process USB events
  
  if (joystick1.available()) {
    if (!controllerConnected) {
      controllerConnected = true;
      Serial.println(" PS4 Controller Connected via Bluetooth!");
    }
  
  // Read joystick values and process robot movement
    readJoysticks();      // Get controller input
    calcSpeed();          // Calculate wheel speeds using Inverse Kinematics
    printJoystickValues(); // Display debug information
    
    joystick1.joystickDataClear();  // Clear joystick data for next reading
  } else {
    if (controllerConnected) {
      controllerConnected = false;
      Serial.println(" Controller disconnected");
       // Safety: Stop all motors when controller disconnects
      MotorDirSetting(M1_Pwm, M1_Dir, 0);
      MotorDirSetting(M2_Pwm, M2_Dir, 0);
      MotorDirSetting(M3_Pwm, M3_Dir, 0);
      
    }
  }
  
  delay(50); // Main loop delay (20Hz control frequency)
}
void readJoysticks() {
// Read analog sticks from PS4 controller
// -128 to center at 0
  leftX = joystick1.getAxis(0) - 128;   // Left stick X: -128 to 127
  leftY = joystick1.getAxis(1) - 128;   // Left stick Y: -128 to 127
  rightX = joystick1.getAxis(2) - 128;  // Right stick X: -128 to 127

  // Map joystick inputs to robot velocity vectors
  Vx = constrain(-leftY, -128, 128);   // Forward/backward motion (inverted Y-axis)
  Vy = constrain(leftX, -128, 128);    // Left/right strafing motion
  omega = constrain(rightX, -128, 128); // Rotational motion
 
  // Apply deadzone to prevent drift from small joystick movements
  if (abs(Vx) < 15) Vx = 0;
  if (abs(Vy) < 15) Vy = 0;
  if (abs(omega) < 15) omega = 0;

}
void MotorDirSetting(int pwmPin, int dirPin, float speed) { // Motor control function with direction and speed
  if (speed > 0) {
    digitalWrite(dirPin, LOW);  // Set direction forward
    analogWrite(pwmPin, (int)(speed)); // Set PWM speed 
  } else if (speed < 0) {
    digitalWrite(dirPin, HIGH);  // Set direction reverse
    analogWrite(pwmPin, (int)(-speed)); // Set PWM speed 
  }
  else{
    analogWrite(pwmPin, 0); // Stop motor
  }
}
void calcSpeed(){  // formula derived using IK
// INVERSE KINEMATICS CALCULATION for 3-wheel holonomic drive
  // Wheel configuration: M1 at 0°, M2 at 120°, M3 at 240°

// formula used:  V_wheel = -sin(θ) * Vx + cos(θ) * Vy + omega
// Trigonometric inverse kinematics formulas:
V1 = 0.0 * Vx + 1.0 * Vy + omega;      
V2 = -0.866 * Vx - 0.5 * Vy + omega;     
V3 = 0.866 * Vx - 0.5 * Vy + omega;   

 // Apply global speed limit for safety
  V1 *= MaxSpeed;  
  V2 *= MaxSpeed;
  V3 *= MaxSpeed;

  // Set motor speeds
  MotorDirSetting(M1_Pwm, M1_Dir, V1);
  MotorDirSetting(M2_Pwm, M2_Dir, V2);
  MotorDirSetting(M3_Pwm, M3_Dir, -V3);  //Motor 3 direction is inverted

}
void printJoystickValues() {
  static unsigned long lastPrint = 0;
  
  // Print every 200ms to avoid spam
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    
 // Display robot velocity vectors
    Serial.print("Vx: (");
    Serial.print(Vx);
    Serial.print(", ");
    Serial.print(") ");
    
    Serial.print("Vy: (");
    Serial.print(Vy);
    Serial.print(", ");
    Serial.print(") ");

     Serial.print("omega: (");
    Serial.print(omega);
    Serial.print(", ");
    Serial.println(") ");

  // Display individual wheel speeds
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