#include <USBHost_t36.h>

USBHost myusb;
USBHub hub1(myusb);
USBHIDParser hid1(myusb);
USBHIDParser hid2(myusb);
USBHIDParser hid3(myusb);
BluetoothController bluetooth(myusb);
JoystickController joystick1(myusb);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000) {}
  
  Serial.println("PS4 Controller - Raw Joystick Test");
  myusb.begin();
}

void loop() {
  myusb.Task();
  
  if (joystick1.available()) {
    // Read only joystick axes (raw 0-255 values)
    int leftX = joystick1.getAxis(0);
    int leftY = joystick1.getAxis(1);
    int rightX = joystick1.getAxis(2);
    int rightY = joystick1.getAxis(5);
    
    Serial.println("=== RAW JOYSTICK DATA ===");
    Serial.print("Left Stick: X=");
    Serial.print(leftX);
    Serial.print(" Y=");
    Serial.println(leftY);
    
    Serial.print("Right Stick: X=");
    Serial.print(rightX);
    Serial.print(" Y=");
    Serial.println(rightY);
    Serial.println("=========================");
    
    joystick1.joystickDataClear();
  }
  
  delay(100); // Update every 100ms
}