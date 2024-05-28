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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include <math.h>

const static char *TAG = "SM";

// bitwidth for continuous mode
// SOC_ADC_DIGI_MAX_BITWIDTH=12
// bitwidth for oneshot mode
// #define SOC_ADC_RTC_MAX_BITWIDTH                (13)


// Debugging
// 0: Debugging off. Set to this once everything is working.
// 1: Output raw joystick values. 0-8192 raw ADC 10-bit values
// 2: Output centered joystick values. Values should be approx -500 to +500, jitter around 0 at idle.
// 3: Output centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle.
// 4: Output translation and rotation values. Approx -800 to 800 depending on the parameter.
// 5: Output debug 4 and 5 side by side for direct cause and effect reference.
int DEBUG=1;

// Direction
// Modify the direction of translation/rotation depending on preference. This can also be done per application in the 3DConnexion software.
// Switch between true/false as desired.
bool invX = false; // pan left/right
bool invY = false; // pan up/down
bool invZ = true; // zoom in/out
bool invRX = true; // Rotate around X axis (tilt front/back)
bool invRY = false; // Rotate around Y axis (tilt left/right)
bool invRZ = true; // Rotate around Z axis (twist left/right)

// Axes are matched to pin order.
#define AX 0
#define AY 1
#define BX 2
#define BY 3
#define CX 4
#define CY 5
#define DX 6
#define DY 7

// Speed - Had to be removed when the V2 shorter range of motion. The logic reduced sensitivity on small movements. Use 3DConnexion slider instead for V2.
// Modify to change sensitibity/speed. Default and maximum 100. Works like a percentage ie. 50 is half as fast as default. This can also be done per application in the 3DConnexion software.
//int16_t speed = 100; 

// Default Assembly when looking from above:
//    C           Y+
//    |           .
// B--+--D   X-...Z+...X+
//    |           .
//    A           Y-
//
// Wiring. Matches the first eight ADC pins of ESP32-S2 on Wemos S2 mini
int PINLIST_ADC1[] = { // The positions of the reads
  3, // X-axis A
  5, // Y-axis A
  7, // X-axis B
  9, // Y-axis B
};

int PINLIST_ADC2[] = { // The positions of the reads
  11, // X-axis C
  12, // Y-axis C
  16, // X-axis D
  18  // Y-axis D
};
#define PIN_CNT (sizeof(PINLIST_ADC1)/sizeof(int))

// Deadzone to filter out unintended movements. Increase if the mouse has small movements when it should be idle or the mouse is too senstive to subtle movements.
int DEADZONE = 5; // Recommended to have this as small as possible for V2 to allow smaller knob range of motion.

// ADC related handles and data
class ADCData {
    adc_oneshot_unit_handle_t adc1_handle, adc2_handle;
    adc_channel_t adc1_chans[PIN_CNT], adc2_chans[PIN_CNT];
    int rawReads[8];
    // Centerpoint variable to be populated during setup routine.
    int centerPoints[8];
    int centered[8];
    int centeredDZ[8];
    int16_t transX, transY, transZ, rotX, rotY, rotZ; // Declare movement variables at 16 bit integers

public:
  ADCData(){
      for(int i = 0;i<2*PIN_CNT;i++) {
        rawReads[i] = 0;
        centerPoints[i] = 0;
        centered[i] = 0;
        centeredDZ[i] = 0;
      }

  }
  // Function to read and store analogue voltages for each joystick axis.
  void readAllFromJoystick(int nSamples){

      int32_t lRaw[2*PIN_CNT] = {0, 0, 0, 0, 0, 0, 0, 0};
      for(int j = 0;j<nSamples;j++) {
        for(int i = 0;i<PIN_CNT;i++) {
          int v1, v2;
          ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, adc1_chans[i], &v1));
          ESP_ERROR_CHECK(adc_oneshot_read(adc2_handle, adc2_chans[i], &v2));
          lRaw[i] += v1;
          lRaw[i + PIN_CNT] += v2;
        }
      }

      for(int i = 0;i<2*PIN_CNT;i++) {
        rawReads[i] = lRaw[i]/nSamples;
      }
  }

  void initCenterPoints() {
    readAllFromJoystick(100);
    for(int i = 0;i<2*PIN_CNT;i++) {
        centerPoints[i] = rawReads[i];
    }
  }

  /**
   * subtract center values and interpolate into range 0-1024 for compat with Arduino
  */
  void interpolateTo1024() {
      for(int i = 0;i<2*PIN_CNT;i++) {
        int r = rawReads[i];
        int c = centerPoints[i];
        int d = r - c;
        int pv = 0;
        if (d<0) {
          pv = round(d*512.0/c);
        } else {
          pv = round(d*512.0/(8192-c));
        }
        centered[i] = pv;
      }
  }

  void filterDeadZone() {
      for(int i = 0;i<2*PIN_CNT;i++) {
        int v = centered[i];
        if (abs(v)<DEADZONE) {
          v = 0;
        }
        centeredDZ[i] = v;
      }
  }

  void calcRotTrans() {
    int* centered = this->centeredDZ;
    // Doing all through arithmetic contribution by fdmakara
    // Integer has been changed to 16 bit int16_t to match what the HID protocol expects.
    // Original fdmakara calculations
    //transX = (-centered[AX] +centered[CX])/1;
    //transY = (-centered[BX] +centered[DX])/1;
    //transZ = (-centered[AY] -centered[BY] -centered[CY] -centered[DY])/2;
    //rotX = (-centered[AY] +centered[CY])/2;
    //rotY = (+centered[BY] -centered[DY])/2;
    //rotZ = (+centered[AX] +centered[BX] +centered[CX] +centered[DX])/4;
    // My altered calculations based on debug output. Final divisor can be changed to alter sensitivity for each axis.
    transX = -(-centered[CY] +centered[AY])/1;  
    transY = (-centered[BY]+centered[DY])/1;
    if((abs(centered[AX])>DEADZONE)&&(abs(centered[BX])>DEADZONE)&&(abs(centered[CX])>DEADZONE)&&(abs(centered[DX])>DEADZONE)){
      transZ = (-centered[AX] -centered[BX] -centered[CX] -centered[DX])/1;
      transX = 0;
      transY = 0;
    } else {
      transZ = 0;
    } 
    rotX = (-centered[AX] +centered[CX])/1;
    rotY = (+centered[BX] -centered[DX])/1;
    if((abs(centered[AY])>DEADZONE)&&(abs(centered[BY])>DEADZONE)&&(abs(centered[CY])>DEADZONE)&&(abs(centered[DY])>DEADZONE)){
      rotZ = (+centered[AY] +centered[BY] +centered[CY] +centered[DY])/2;
      rotX = 0;
      rotY = 0;
    } else {
      rotZ = 0;
    }

  // Alter speed to suit user preference - Use 3DConnexion slider instead for V2.
    //transX = transX/100*speed;
    //transY = transY/100*speed;
    //transZ = transZ/100*speed;
    //rotX = rotX/100*speed;
    //rotY = rotY/100*speed;
    //rotZ = rotZ/100*speed;
  // Invert directions if needed
    if(invX == true){ transX = transX*-1;};
    if(invY == true){ transY = transY*-1;};
    if(invZ == true){ transZ = transZ*-1;};
    if(invRX == true){ rotX = rotX*-1;};
    if(invRY == true){ rotY = rotY*-1;};
    if(invRZ == true){ rotZ = rotZ*-1;};
  }

  void adc_init() {
      adc_oneshot_unit_init_cfg_t init_config1 = {
          .unit_id = ADC_UNIT_1,
          .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
          .ulp_mode = ADC_ULP_MODE_DISABLE
      };
      ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
      adc_oneshot_unit_init_cfg_t init_config2 = {
          .unit_id = ADC_UNIT_2,
          .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
          .ulp_mode = ADC_ULP_MODE_DISABLE
      };
      ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config2, &adc2_handle));

      adc_unit_t adc1 = ADC_UNIT_1, adc2 = ADC_UNIT_2;
      for(int i = 0;i<PIN_CNT;i++) {
          ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PINLIST_ADC1[i], &adc1, &adc1_chans[i]));
          ESP_ERROR_CHECK(adc_oneshot_io_to_channel(PINLIST_ADC2[i], &adc2, &adc2_chans[i]));
      }

      //-------------ADC Config---------------//
      adc_oneshot_chan_cfg_t config = {
          .atten = ADC_ATTEN_DB_12,
          .bitwidth = ADC_BITWIDTH_DEFAULT,
      };
      for(int i = 0;i<PIN_CNT;i++) {
          ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc1_chans[i], &config));
          ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, adc2_chans[i], &config));
      }
  }

void dbg_prints() {
    if (DEBUG == 1) {
        ESP_LOGI(TAG, "AX:%4d AY:%4d BX:%4d BY:%4d CX:%4d CY:%4d DX:%4d DY:%4d ", rawReads[0], 
        rawReads[1], rawReads[2], rawReads[3], rawReads[4], 
        rawReads[5], rawReads[6], rawReads[7] );
    }

  // Subtract centre position from measured position to determine movement.
  int* centered = this->centered;

  // Report centered joystick values if enabled. Values should be approx -500 to +500, jitter around 0 at idle.
  if(DEBUG == 2){
        ESP_LOGI(TAG, "AX:%4d AY:%4d BX:%4d BY:%4d CX:%4d CY:%4d DX:%4d DY:%4d ", centered[0], 
        centered[1], centered[2], centered[3], centered[4], centered[5], centered[6], centered[7] );
  }    

  // Filter movement values. Set to zero if movement is below deadzone threshold.
  filterDeadZone();

  // Report centered joystick values. Filtered for deadzone. Approx -500 to +500, locked to zero at idle
  if(DEBUG == 3){
        ESP_LOGI(TAG, "CDZ AX:%4d AY:%4d BX:%4d BY:%4d CX:%4d CY:%4d DX:%4d DY:%4d ", centered[0], 
        centered[1], centered[2], centered[3], centered[4], centered[5], centered[6], centered[7] );
  }  

  calcRotTrans();

// Report translation and rotation values if enabled. Approx -800 to 800 depending on the parameter.
  if(DEBUG == 4){
    ESP_LOGI(TAG, "TX:%2d TY:%2d TZ:%2d RX:%2d RY:%2d RZ:%2d ", transX, transY, transZ, rotX, rotY, rotZ );
  }
// Report debug 3 and 4 info side by side for direct reference if enabled. Very useful if you need to alter which inputs are used in the arithmatic above.
  if(DEBUG == 5){
    ESP_LOGI(TAG, "AX:%4d AY:%4d BX:%4d BY:%4d CX:%4d CY:%4d DX:%4d DY:%4d || TX:%2d TY:%2d TZ:%2d RX:%2d RY:%2d RZ:%2d ", centered[0], 
        centered[1], centered[2], centered[3], centered[4], centered[5], centered[6], centered[7], transX, transY, transZ, rotX, rotY, rotZ );
  }  

}  

  void adc_done() {
      ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
      ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
  }  

};








extern "C" void app_main(void)
{
    esp_timer_early_init();

    //-------------ADC Init---------------//
    ADCData adcData;
    adcData.adc_init();

    // Read idle/centre positions for joysticks.
    adcData.initCenterPoints();

    while (1) {


        // int64_t before = esp_timer_get_time();
        adcData.readAllFromJoystick(10);
        // int64_t elapsed = esp_timer_get_time() - before;

        adcData.interpolateTo1024();
        adcData.filterDeadZone();
        adcData.calcRotTrans();
        if (DEBUG>0) {
          adcData.dbg_prints();
        }
        // int64_t elapsed2 = esp_timer_get_time() - before;
        // ESP_LOGI(TAG, "loop: %d %d", (int)elapsed, (int)elapsed2);

        vTaskDelay(pdMS_TO_TICKS(100));

    }

    //Tear Down
    adcData.adc_done();
}


