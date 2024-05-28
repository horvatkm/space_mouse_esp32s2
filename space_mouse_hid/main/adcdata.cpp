#include "adcdata.h"


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



ADCData::ADCData(){
    for(int i = 0;i<2*PIN_CNT;i++) {
    rawReads[i] = 0;
    centerPoints[i] = 0;
    centered[i] = 0;
    centeredDZ[i] = 0;
    }
}

  // Function to read and store analogue voltages for each joystick axis.
  void ADCData::readAllFromJoystick(int nSamples){

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

  void ADCData::initCenterPoints() {
    readAllFromJoystick(100);
    for(int i = 0;i<2*PIN_CNT;i++) {
        centerPoints[i] = rawReads[i];
    }
  }

  /**
   * subtract center values and interpolate into range 0-1024 for compat with Arduino
  */
  void ADCData::interpolateTo1024() {
      for(int i = 0;i<2*PIN_CNT;i++) {
        int r = rawReads[i];
        int c = centerPoints[i];
        int d = r - c;
        int pv = 0;
        if (d<0) {
          pv = round(d*250.0/c);
        } else {
          pv = round(d*250.0/(8192-c));
        }
        centered[i] = pv;
      }
  }

  void ADCData::filterDeadZone() {
      for(int i = 0;i<2*PIN_CNT;i++) {
        int v = centered[i];
        if (abs(v)<DEADZONE) {
          v = 0;
        }
        centeredDZ[i] = v;
      }
  }

  void ADCData::calcRotTrans() {
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
    if(INVX == true){ transX = transX*-1;};
    if(INVY == true){ transY = transY*-1;};
    if(INVZ == true){ transZ = transZ*-1;};
    if(INVRX == true){ rotX = rotX*-1;};
    if(INVRY == true){ rotY = rotY*-1;};
    if(INVRZ == true){ rotZ = rotZ*-1;};
  }

  void ADCData::adc_init() {
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

const static char *TAG = "SM";

void ADCData::dbg_prints() {
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

void ADCData::adc_done() {
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc1_handle));
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc2_handle));
}  
