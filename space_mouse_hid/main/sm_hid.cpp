/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tinyusb.h"
#include "soc/soc_caps.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "class/hid/hid_device.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "adcdata.h"
#include "hal/wdt_hal.h"

#define SPACE_MOUSE_PRO 1 
#define SPACE_MOUSE_WIRELESS 2
#define SPACE_MOUSE_ENTERPRISE 3

// two events for translation and rotation
#define SM_DEVICE  SPACE_MOUSE_PRO

// single event for both translation and rotation
// #define SM_DEVICE SPACE_MOUSE_WIRELESS

// single event for both translation and rotation
#define SM_DEVICE SPACE_MOUSE_ENTERPRISE

#if SM_DEVICE == SPACE_MOUSE_PRO  || SM_DEVICE == SPACE_MOUSE_WIRELESS
#define DEVICE_TYPE 66
#else
#define DEVICE_TYPE 12
#endif

static const char *TAG = "SM";

/************* TinyUSB descriptors ****************/
// This portion sets up the communication with the 3DConnexion software. The communication protocol is created here.
// hidReportDescriptor webpage can be found here: https://eleccelerator.com/tutorial-about-usb-hid-report-descriptors/ 
#if DEVICE_TYPE == 66
#define TUD_HID_REPORT_DESC_SPACE_MOUSE \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER  )   ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
  HID_COLLECTION ( HID_COLLECTION_PHYSICAL )                    ,\
  HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)                       \
  HID_LOGICAL_MIN_N  ( -32768, 2                              ) ,\
  HID_LOGICAL_MAX_N  ( 32767, 2                               ) ,\
  HID_PHYSICAL_MIN_N  ( -32768, 2                             ) ,\
  HID_PHYSICAL_MAX_N  ( 32767, 2                              ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
  HID_REPORT_SIZE    ( 16                                     ) ,\
  HID_REPORT_COUNT   ( 3                                      ) ,\
  HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END                                            ,\
  HID_COLLECTION ( HID_COLLECTION_PHYSICAL )                    ,\
  HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE)                          \
  HID_LOGICAL_MIN_N  ( -32768, 2                              ) ,\
  HID_LOGICAL_MAX_N  ( 32767, 2                               ) ,\
  HID_PHYSICAL_MIN_N  ( -32768, 2                             ) ,\
  HID_PHYSICAL_MAX_N  ( 32767, 2                              ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
  HID_REPORT_SIZE    ( 16                                     ) ,\
  HID_REPORT_COUNT   ( 3                                      ) ,\
  HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END                                            ,\
  HID_COLLECTION ( HID_COLLECTION_PHYSICAL )                    ,\
  HID_REPORT_ID(3)                                               \
  HID_LOGICAL_MIN  ( 0                                        ) ,\
  HID_LOGICAL_MAX  ( 1                                        ) ,\
  HID_REPORT_SIZE    ( 1                                      ) ,\
  HID_REPORT_COUNT   ( 32                                     ) ,\
  HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
  HID_USAGE_MIN      (1                                       ) ,\
  HID_USAGE_MAX      (32                                      ) ,\
  HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END                                            ,\
  HID_COLLECTION_END
#else
#define TUD_HID_REPORT_DESC_SPACE_MOUSE \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_MULTI_AXIS_CONTROLLER  )   ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
  HID_COLLECTION ( HID_COLLECTION_PHYSICAL )                    ,\
  HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)                       \
  HID_LOGICAL_MIN_N  ( -32768, 2                              ) ,\
  HID_LOGICAL_MAX_N  ( 32767, 2                               ) ,\
  HID_PHYSICAL_MIN_N  ( -32768, 2                             ) ,\
  HID_PHYSICAL_MAX_N  ( 32767, 2                              ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
  HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
  HID_REPORT_SIZE    ( 16                                     ) ,\
  HID_REPORT_COUNT   ( 6                                      ) ,\
  HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END                                            ,\
  HID_COLLECTION ( HID_COLLECTION_PHYSICAL )                    ,\
  HID_REPORT_ID(3)                                               \
  HID_LOGICAL_MIN  ( 0                                        ) ,\
  HID_LOGICAL_MAX  ( 1                                        ) ,\
  HID_REPORT_SIZE    ( 1                                      ) ,\
  HID_REPORT_COUNT   ( 32                                     ) ,\
  HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
  HID_USAGE_MIN      (1                                       ) ,\
  HID_USAGE_MAX      (32                                      ) ,\
  HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END                                            ,\
  HID_COLLECTION_END
#endif

#define TUSB_DESC_TOTAL_LEN      (TUD_CONFIG_DESC_LEN + CFG_TUD_HID * TUD_HID_DESC_LEN)

/**
 * @brief HID report descriptor
 *
 * In this example we implement Keyboard + Mouse HID device,
 * so we must define both report descriptors
 */
const uint8_t hid_report_descriptor[] = {
    // TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(HID_ITF_PROTOCOL_KEYBOARD)),
    // TUD_HID_REPORT_DESC_MOUSE(HID_REPORT_ID(HID_ITF_PROTOCOL_MOUSE)),
    TUD_HID_REPORT_DESC_SPACE_MOUSE
};

/**
 * @brief String descriptor
 */
const char* hid_string_descriptor[5] = {
    // array of pointer to string descriptors
    (char[]){0x09, 0x04},  // 0: is supported language is English (0x0409)
    "3Dconnexion",                // 1: Manufacturer
    "Spacemouse",  // 2: Product
    "K2343432345348",             // 3: Serials, should use chip ID
    "Space mouse HID interface",  // 4: HID
};

/**
 * @brief Configuration descriptor
 *
 * This is a simple configuration descriptor that defines 1 configuration and 1 HID interface
 */
static const uint8_t hid_configuration_descriptor[] = {
    // Configuration number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, boot protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

/********* TinyUSB HID callbacks ***************/

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application return pointer to descriptor, whose contents must exist long enough for transfer to complete
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    // We use only one interface and one HID report descriptor, so we can ignore parameter 'instance'
    return hid_report_descriptor;
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
    (void) instance;
    (void) report_id;
    (void) report_type;
    (void) buffer;
    (void) reqlen;
      ESP_LOGI(TAG, "tud_hid_get_report_cb: instance:%d report_id:%d reporttype:%d reqlen:%d", instance, report_id, report_type, reqlen);
    return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
      ESP_LOGI(TAG, "tud_hid_set_report_cb: instance:%d report_id:%d reporttype:%d bufsize:%d", instance, report_id, report_type, bufsize);
}

/********* Application ***************/

void sendHidReport(int rx, int ry, int rz, int x, int y, int z) {
#if DEVICE_TYPE == 66
    uint8_t trans[6] = { static_cast<uint8_t>(x & 0xFF) , static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(z & 0xFF), static_cast<uint8_t>(z >> 8) };
    tud_hid_report(1, trans, 6);

    uint8_t rot[6] = { static_cast<uint8_t>(rx & 0xFF), static_cast<uint8_t>(rx >> 8), static_cast<uint8_t>(ry & 0xFF), static_cast<uint8_t>(ry >> 8), static_cast<uint8_t>(rz & 0xFF), static_cast<uint8_t>(rz >> 8) };
    tud_hid_report(2, rot, 6);
#else                                      
    uint8_t trans[12] = { static_cast<uint8_t>(x & 0xFF) , static_cast<uint8_t>(x >> 8), static_cast<uint8_t>(y & 0xFF), static_cast<uint8_t>(y >> 8), static_cast<uint8_t>(z & 0xFF), static_cast<uint8_t>(z >> 8),
        static_cast<uint8_t>(rx & 0xFF), static_cast<uint8_t>(rx >> 8),                      static_cast<uint8_t>(ry & 0xFF), static_cast<uint8_t>(ry >> 8), static_cast<uint8_t>(rz & 0xFF), static_cast<uint8_t>(rz >> 8) };
    tud_hid_report(1, trans, 12);
#endif

}                                     
                                                                                                                   
extern "C" void app_main(void)
{                                                                                                                                                                                        
    //-------------ADC Init---------------//
    ADCData adcData;
    adcData.adc_init();

    // Read idle/centre positions for joysticks.
    adcData.initCenterPoints();

    // Initialize button that will trigger HID reports
    // const gpio_config_t boot_button_config = {
    //     .pin_bit_mask = BIT64(APP_BUTTON),
    //     .mode = GPIO_MODE_INPUT,
    //     .intr_type = GPIO_INTR_DISABLE,
    //     .pull_up_en = true,
    //     .pull_down_en = false,
    // };
    // ESP_ERROR_CHECK(gpio_config(&boot_button_config));

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_HID,
    .bDeviceSubClass = 0,//MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = 0,//MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

#if SM_DEVICE == SPACE_MOUSE_PRO
    .idVendor = 0x046d, 
    .idProduct = 0xc62b,
#elif SM_DEVICE == SPACE_MOUSE_WIRELESS
    .idVendor = 0x256f, 
    .idProduct = 0xc631,
#elif SM_DEVICE == SPACE_MOUSE_ENTERPRISE
    .idVendor = 0x256f, 
    .idProduct = 0xc633,
#endif

    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};


    ESP_LOGI(TAG, "USB initialization");
    const tinyusb_config_t tusb_cfg = {
        .device_descriptor = &descriptor_config,
        .string_descriptor = hid_string_descriptor,
        .string_descriptor_count = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
        .external_phy = false,
        .configuration_descriptor = hid_configuration_descriptor,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB initialization DONE");

    while (1) {
        bool mounted = tud_mounted();
        // ESP_LOGI(TAG, "loop mounted: %d", mounted);
        if (mounted) {

            // int64_t before = esp_timer_get_time();
            adcData.readAllFromJoystick(5);
            // int64_t elapsed = esp_timer_get_time() - before;

            adcData.interpolateTo1024();
            adcData.filterDeadZone();
            adcData.calcRotTrans();
            if (DEBUG>0) {
                adcData.dbg_prints();
            }

            sendHidReport(adcData.rotX, adcData.rotY, adcData.rotZ, adcData.transX, adcData.transY, adcData.transZ);
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
