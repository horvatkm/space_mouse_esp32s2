// This code is the combination of multiple works by others:
// 1. Original code for the Space Mushroom by Shiura on Thingiverse: https://www.thingiverse.com/thing:5739462
//    The next two from the comments on the instructables page: https://www.instructables.com/Space-Mushroom-Full-6-DOFs-Controller-for-CAD-Appl/
//    and the comments of Thingiverse: https://www.thingiverse.com/thing:5739462/comments
// 2. Code to emulate a 3DConnexion Space Mouse by jfedor: https://pastebin.com/gQxUrScV
// 3. This code was then remixed by BennyBWalker to include the above two sketches: https://pastebin.com/erhTgRBH
// 4. Four joystick remix code by fdmakara: https://www.thingiverse.com/thing:5817728
// My work invloves mixing all of these. The basis is fdmakara's four joystick movement logic, with jfedor/BennyBWalker's HID SpaceMouse emulation.
// The four joystick logic sketch was setup for the joystick library instead of HID, so elements of this were omitted where not needed.
// The outputs were jumbled no matter how I plugged them in, so I spent a lot of time adding debugging code to track exactly what was happening.
// On top of this, I have added more control of speed/direction and comments/links to informative resources to try and explain what is happening in each phase.

// Spacemouse emulation
// I followed the instructions here from nebhead: https://gist.github.com/nebhead/c92da8f1a8b476f7c36c032a0ac2592a
// with two key differences:
// 1. I changed the word 'DaemonBite' to 'Spacemouse' in all references.
// 2. I changed the VID and PID values as per jfedor's instructions: vid=0x256f, pid=0xc631 (SpaceMouse Pro Wireless (cabled))
// When compiling and uploading, I select Arduino AVR boards (in Sketchbook) > Spacemouse and then the serial port.
// You will also need to download and install the 3DConnexion software: https://3dconnexion.com/us/drivers-application/3dxware-10/
// If all goes well, the 3DConnexion software will show a SpaceMouse Pro wireless when the Arduino is connected.

// Include inbuilt Arduino HID library by NicoHood: https://github.com/NicoHood/HID 
#include "HID.h"

// Debugging
// 0: Debugging off. Set to this once everything is working.
// 1: Output raw joystick values. 0-1023 raw ADC 10-bit values
// 2: Output centered joystick values. Values should be approx -500 to +500, jitter around 0 at idle.
// 3: Output centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle.
// 4: Output translation and rotation values. Approx -800 to 800 depending on the parameter.
// 5: Output debug 4 and 5 side by side for direct cause and effect reference.
int debug = 0;

// Direction
// Modify the direction of translation/rotation depending on preference. This can also be done per application in the 3DConnexion software.
// Switch between true/false as desired.
bool invX = false; // pan left/right
bool invY = false; // pan up/down
bool invZ = false; // zoom in/out
bool invRX = false; // Rotate around X axis (tilt front/back)
bool invRY = true; // Rotate around Y axis (tilt left/right)
bool invRZ = true; // Rotate around Z axis (twist left/right)

// Speed
// Modify to change sensitibity/speed. Default and maximum 100. Works like a percentage ie. 50 is half as fast as default. This can also be done per application in the 3DConnexion software.
int16_t speed = 80;

// Default Assembly when looking from above:
//    C           Y+
//    |           .
// B--+--D   X-...Z+...X+
//    |           .
//    A           Y-
//
// Wiring. Matches the first eight analogue pins of the Arduino Pro Micro (atmega32u4)
int PINLIST[8] = { // The positions of the reads
  A0, // X-axis A
  A1, // Y-axis A
  A2, // X-axis B
  A3, // Y-axis B
  A6, // X-axis C
  A7, // Y-axis C
  A8, // X-axis D
  A9  // Y-axis D
};
// Deadzone to filter out unintended movements. Increase if the mouse has small movements when it should be idle or the mouse is too senstive to subtle movements.
int DEADZONE = 40;

// This portion sets up the communication with the 3DConnexion software. The communication protocol is created here.
// hidReportDescriptor webpage can be found here: https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/ 
static const uint8_t _hidReportDescriptor[] PROGMEM = {
  0x05, 0x01,           //  Usage Page (Generic Desktop)
  0x09, 0x08,           //  0x08: Usage (Multi-Axis)
  0xa1, 0x01,           //  Collection (Application)
  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x01,           //  Report ID
  0x16, 0x00, 0x80,     //logical minimum (-500)
  0x26, 0xff, 0x7f,     //logical maximum (500)
  0x36, 0x00, 0x80,     //Physical Minimum (-32768)
  0x46, 0xff, 0x7f,     //Physical Maximum (32767)
  0x09, 0x30,           //    Usage (X)
  0x09, 0x31,           //    Usage (Y)
  0x09, 0x32,           //    Usage (Z)
  0x75, 0x10,           //    Report Size (16)
  0x95, 0x03,           //    Report Count (3)
  0x81, 0x02,           //    Input (variable,absolute)
  0xC0,                 //  End Collection
  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x02,           //  Report ID
  0x16, 0x00, 0x80,     //logical minimum (-500)
  0x26, 0xff, 0x7f,     //logical maximum (500)
  0x36, 0x00, 0x80,     //Physical Minimum (-32768)
  0x46, 0xff, 0x7f,     //Physical Maximum (32767)
  0x09, 0x33,           //    Usage (RX)
  0x09, 0x34,           //    Usage (RY)
  0x09, 0x35,           //    Usage (RZ)
  0x75, 0x10,           //    Report Size (16)
  0x95, 0x03,           //    Report Count (3)
  0x81, 0x02,           //    Input (variable,absolute)
  0xC0,                 //  End Collection
 
  0xa1, 0x00,           // Collection (Physical)
  0x85, 0x03,           //  Report ID
  0x15, 0x00,           //   Logical Minimum (0)
  0x25, 0x01,           //    Logical Maximum (1)
  0x75, 0x01,           //    Report Size (1)
  0x95, 32,             //    Report Count (24)
  0x05, 0x09,           //    Usage Page (Button)
  0x19, 1,              //    Usage Minimum (Button #1)
  0x29, 32,             //    Usage Maximum (Button #24)
  0x81, 0x02,           //    Input (variable,absolute)
  0xC0,
  0xC0
};

// Axes are matched to pin order.
#define AX 0
#define AY 1
#define BX 2
#define BY 3
#define CX 4
#define CY 5
#define DX 6
#define DY 7

// Centerpoint variable to be populated during setup routine.
int centerPoints[8];

// Function to read and store analogue voltages for each joystick axis.
void readAllFromJoystick(int *rawReads){
  for(int i=0; i<8; i++){
    rawReads[i] = analogRead(PINLIST[i]);
  }
}

void setup() {
  // HID protocol is set.
  static HIDSubDescriptor node(_hidReportDescriptor, sizeof(_hidReportDescriptor));
  HID().AppendDescriptor(&node);
  // Begin Seral for debugging
  Serial.begin(250000);
  delay(100);
  // Read idle/centre positions for joysticks.
  readAllFromJoystick(centerPoints);
  readAllFromJoystick(centerPoints);
}

// Function to send translation and rotation data to the 3DConnexion software using the HID protocol outlined earlier. Two sets of data are sent: translation and then rotation.
// For each, a 16bit integer is split into two using bit shifting. The first is mangitude and the second is direction.
void send_command(int16_t rx, int16_t ry, int16_t rz, int16_t x, int16_t y, int16_t z) {
  uint8_t trans[6] = { x & 0xFF, x >> 8, y & 0xFF, y >> 8, z & 0xFF, z >> 8 };
  HID().SendReport(1, trans, 6);
  uint8_t rot[6] = { rx & 0xFF, rx >> 8, ry & 0xFF, ry >> 8, rz & 0xFF, rz >> 8 };
  HID().SendReport(2, rot, 6);
}

void loop() {
  int rawReads[8], centered[8];
  // Joystick values are read. 0-1023
  readAllFromJoystick(rawReads);
  // Report back 0-1023 raw ADC 10-bit values if enabled
  if(debug == 1){ 
    Serial.print("AX:");
    Serial.print(rawReads[0]);
    Serial.print(",");
    Serial.print("AY:");
    Serial.print(rawReads[1]);
    Serial.print(",");
    Serial.print("BX:");
    Serial.print(rawReads[2]);
    Serial.print(",");
    Serial.print("BY:");
    Serial.print(rawReads[3]);
    Serial.print(",");
    Serial.print("CX:");
    Serial.print(rawReads[4]);
    Serial.print(",");
    Serial.print("CY:");
    Serial.print(rawReads[5]);
    Serial.print(",");
    Serial.print("DX:");
    Serial.print(rawReads[6]);
    Serial.print(",");
    Serial.print("DY:");
    Serial.println(rawReads[7]);
  }

  // Subtract centre position from measured position to determine movement.
  for(int i=0; i<8; i++) centered[i] = rawReads[i] - centerPoints[i]; // 
  // Report centered joystick values if enabled. Values should be approx -500 to +500, jitter around 0 at idle.
  if(debug == 2){
    Serial.print("AX:");
    Serial.print(centered[0]);
    Serial.print(",");
    Serial.print("AY:");
    Serial.print(centered[1]);
    Serial.print(",");
    Serial.print("BX:");
    Serial.print(centered[2]);
    Serial.print(",");
    Serial.print("BY:");
    Serial.print(centered[3]);
    Serial.print(",");
    Serial.print("CX:");
    Serial.print(centered[4]);
    Serial.print(",");
    Serial.print("CY:");
    Serial.print(centered[5]);
    Serial.print(",");
    Serial.print("DX:");
    Serial.print(centered[6]);
    Serial.print(",");
    Serial.print("DY:");
    Serial.println(centered[7]);
  }
  // Filter movement values. Set to zero if movement is below deadzone threshold.
  for(int i=0; i<8; i++){
    if(centered[i]<DEADZONE && centered[i]>-DEADZONE) centered[i] = 0;
  }
  // Report centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle
  if(debug == 3){
    Serial.print("AX:");
    Serial.print(centered[0]);
    Serial.print(",");
    Serial.print("AY:");
    Serial.print(centered[1]);
    Serial.print(",");
    Serial.print("BX:");
    Serial.print(centered[2]);
    Serial.print(",");
    Serial.print("BY:");
    Serial.print(centered[3]);
    Serial.print(",");
    Serial.print("CX:");
    Serial.print(centered[4]);
    Serial.print(",");
    Serial.print("CY:");
    Serial.print(centered[5]);
    Serial.print(",");
    Serial.print("DX:");
    Serial.print(centered[6]);
    Serial.print(",");
    Serial.print("DY:");
    Serial.println(centered[7]);
  }

  // Doing all through arithmetic contribution by fdmakara
  // Integer has been changed to 16 bit int16_t to match what the HID protocol expects.
  int16_t transX, transY, transZ, rotX, rotY, rotZ; // Declare movement variables at 16 bit integers
  // Original fdmakara calculations
  //transX = (-centered[AX] +centered[CX])/1;
  //transY = (-centered[BX] +centered[DX])/1;
  //transZ = (-centered[AY] -centered[BY] -centered[CY] -centered[DY])/2;
  //rotX = (-centered[AY] +centered[CY])/2;
  //rotY = (+centered[BY] -centered[DY])/2;
  //rotZ = (+centered[AX] +centered[BX] +centered[CX] +centered[DX])/4;
  // My altered calculations based on debug output. Final divisor can be changed to alter sensitivity for each axis.
  transX = -(-centered[CY] +centered[AY])/2;  
  transY = (-centered[BY]+centered[DY])/2;  
  transZ = (-centered[AX] -centered[BX] -centered[CX] -centered[DX])/2;
  rotX = (-centered[AX] +centered[CX])/2;
  rotY = (+centered[BX] -centered[DX])/2;
  rotZ = (+centered[AY] +centered[BY] +centered[CY] +centered[DY])/4;

// Alter speed to suit user preference
  transX = transX/100*speed;
  transY = transY/100*speed;
  transZ = transZ/100*speed;
  rotX = rotX/100*speed;
  rotY = rotY/100*speed;
  rotZ = rotZ/100*speed;
// Invert directions if needed
  if(invX == true){ transX = transX*-1;};
  if(invY == true){ transY = transY*-1;};
  if(invZ == true){ transZ = transZ*-1;};
  if(invRX == true){ rotX = rotX*-1;};
  if(invRY == true){ rotY = rotY*-1;};
  if(invRZ == true){ rotZ = rotZ*-1;};

// Report translation and rotation values if enabled. Approx -800 to 800 depending on the parameter.
  if(debug == 4){
    Serial.print("TX:");
    Serial.print(transX);
    Serial.print(",");
    Serial.print("TY:");
    Serial.print(transY);
    Serial.print(",");
    Serial.print("TZ:");
    Serial.print(transZ);
    Serial.print(",");
    Serial.print("RX:");
    Serial.print(rotX);
    Serial.print(",");
    Serial.print("RY:");
    Serial.print(rotY);
    Serial.print(",");
    Serial.print("RZ:");
    Serial.println(rotZ);
  }
// Report debug 4 and 5 info side by side for direct reference if enabled. Very useful if you need to alter which inputs are used in th arithmatic above.
  if(debug == 5){
    Serial.print("AX:");
    Serial.print(centered[0]);
    Serial.print(",");
    Serial.print("AY:");
    Serial.print(centered[1]);
    Serial.print(",");
    Serial.print("BX:");
    Serial.print(centered[2]);
    Serial.print(",");
    Serial.print("BY:");
    Serial.print(centered[3]);
    Serial.print(",");
    Serial.print("CX:");
    Serial.print(centered[4]);
    Serial.print(",");
    Serial.print("CY:");
    Serial.print(centered[5]);
    Serial.print(",");
    Serial.print("DX:");
    Serial.print(centered[6]);
    Serial.print(",");
    Serial.print("DY:");
    Serial.print(centered[7]);
    Serial.print("||");
    Serial.print("TX:");
    Serial.print(transX);
    Serial.print(",");
    Serial.print("TY:");
    Serial.print(transY);
    Serial.print(",");
    Serial.print("TZ:");
    Serial.print(transZ);
    Serial.print(",");
    Serial.print("RX:");
    Serial.print(rotX);
    Serial.print(",");
    Serial.print("RY:");
    Serial.print(rotY);
    Serial.print(",");
    Serial.print("RZ:");
    Serial.println(rotZ);
  }

// Send data to the 3DConnexion software.
// The correct order for me was determined after trial and error
  send_command(rotX, rotY, rotZ, transX, transZ, transY);
}
