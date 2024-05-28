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
#include "const.h"

//(sizeof(PINLIST_ADC1)/sizeof(int))
#define PIN_CNT 4 

class ADCData {
    adc_oneshot_unit_handle_t adc1_handle, adc2_handle;
    adc_channel_t adc1_chans[PIN_CNT], adc2_chans[PIN_CNT];
    int rawReads[8];
    // Centerpoint variable to be populated during setup routine.
    int centerPoints[8];
    int centered[8];
    int centeredDZ[8];

public:
  int16_t transX, transY, transZ, rotX, rotY, rotZ; // Declare movement variables at 16 bit integers

  ADCData();
  
  // Function to read and store analogue voltages for each joystick axis.
  void readAllFromJoystick(int nSamples);

  void initCenterPoints();

  /**
   * subtract center values and interpolate into range 0-1024 for compat with Arduino
  */
  void interpolateTo1024();

  void filterDeadZone();

  void calcRotTrans();

  void adc_init();

  void dbg_prints();

  void adc_done();

};

