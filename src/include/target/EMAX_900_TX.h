#ifndef DEVICE_NAME
    #ifdef EMAX_HAS_OLED
    #define DEVICE_NAME         "EMAX_900TX"
    #else
    #define DEVICE_NAME         "EMAX_NANO_900TX"
    #endif
#endif

#ifdef EMAX_HAS_OLED
#define USE_OLED_I2C
#define OLED_REVERSED
#define HAS_FIVE_WAY_BUTTON


#define GPIO_PIN_OLED_SDA       22
#define GPIO_PIN_OLED_SCK       21
#define GPIO_PIN_OLED_RST       U8X8_PIN_NONE

#define GPIO_PIN_JOYSTICK       33
/* Joystick values              {UP, DOWN, LEFT, RIGHT, ENTER, IDLE}*/
#define JOY_ADC_VALUES          {2010, 1230, 635, 2730, 0, 4095}
#else
#define WS2812_IS_GRB
#define GPIO_PIN_LED_WS2812     27
#endif

#define USE_TX_BACKPACK
#define PASSTHROUGH_BAUD        230400
#define GPIO_PIN_DEBUG_RX       16
#define GPIO_PIN_DEBUG_TX       17
#define GPIO_PIN_BACKPACK_EN    25
#define GPIO_PIN_BACKPACK_BOOT  15

// GPIO pin definitions
#define GPIO_PIN_NSS            5
#define GPIO_PIN_DIO0           4
#define GPIO_PIN_MOSI           23
#define GPIO_PIN_MISO           19
#define GPIO_PIN_SCK            18
#define GPIO_PIN_RST            14
#define GPIO_PIN_RX_ENABLE      12
#define GPIO_PIN_RCSIGNAL_RX    13
#define GPIO_PIN_RCSIGNAL_TX    13
#define GPIO_PIN_FAN_EN         32
#define GPIO_PIN_RFamp_APC2     26

// Output Power
#define MinPower                PWR_10mW
#define MaxPower                PWR_2000mW
#define POWER_OUTPUT_DACWRITE
#define POWER_OUTPUT_VALUES     {8,18,25,35,50,70,105,225}
