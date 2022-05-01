#ifndef DEVICE_NAME
    #ifdef EMAX_HAS_OLED
    #define DEVICE_NAME         "EMAX_2400TX"
    #else
    #define DEVICE_NAME         "EMAX_NANO_2400TX"
    #endif
#endif

// // There is some special handling for this target
#ifndef EMAX_HAS_OLED
#define TARGET_TX_EMAX_2400_V1
#define GPIO_PIN_LED_BLUE       32
#define GPIO_PIN_LED_GREEN      33
#else
#define USE_OLED_I2C
#define OLED_REVERSED
#define HAS_FIVE_WAY_BUTTON

#define GPIO_PIN_OLED_SDA       33
#define GPIO_PIN_OLED_SCK       32
#define GPIO_PIN_OLED_RST       U8X8_PIN_NONE
#define GPIO_PIN_JOYSTICK       35

/* Joystick values              {UP, DOWN, LEFT, RIGHT, ENTER, IDLE}*/
#define JOY_ADC_VALUES          {1905, 1160, 580, 2580, 0, 4095}
#endif

#define USE_TX_BACKPACK
#define PASSTHROUGH_BAUD        115200
#define GPIO_PIN_DEBUG_RX       16
#define GPIO_PIN_DEBUG_TX       17
#define GPIO_PIN_BACKPACK_EN    25
#define GPIO_PIN_BACKPACK_BOOT  15

// GPIO pin definitions
#define GPIO_PIN_NSS            5
#define GPIO_PIN_BUSY           21
#define GPIO_PIN_DIO0           -1
#define GPIO_PIN_DIO1           4
#define GPIO_PIN_MOSI           23
#define GPIO_PIN_MISO           19
#define GPIO_PIN_SCK            18
#define GPIO_PIN_RST            14
#define GPIO_PIN_RX_ENABLE      27
#define GPIO_PIN_TX_ENABLE      26
#define GPIO_PIN_RCSIGNAL_RX    13
#define GPIO_PIN_RCSIGNAL_TX    13
#define GPIO_PIN_FAN_EN         22

// Output Power
#define MinPower PWR_10mW
#define MaxPower PWR_1000mW
#define POWER_OUTPUT_VALUES {-19,-14,-10,-7,-3,0,5}

#define Regulatory_Domain_ISM_2400 1