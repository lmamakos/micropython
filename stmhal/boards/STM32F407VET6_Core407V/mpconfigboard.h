#define STM32F407COREV

#define MICROPY_HW_BOARD_NAME       "Black STM32F407CORE (8MHz HSE)"
#define MICROPY_HW_MCU_NAME         "STM32F407"

#define MICROPY_HW_HAS_SWITCH       (1)
#define MICROPY_HW_HAS_FLASH        (1)
#define MICROPY_HW_HAS_SDCARD       (1)   /* XXX */
#define MICROPY_HW_HAS_MMA7660      (0)
#define MICROPY_HW_HAS_LIS3DSH      (0)
#define MICROPY_HW_HAS_LCD          (0)
#define MICROPY_HW_ENABLE_RNG       (1)
#define MICROPY_HW_ENABLE_RTC       (1)
#define MICROPY_HW_ENABLE_TIMER     (1)
#define MICROPY_HW_ENABLE_SERVO     (1)
#define MICROPY_HW_ENABLE_DAC       (1)
#define MICROPY_HW_ENABLE_CAN       (1)

// HSE is 8MHz
#define MICROPY_HW_CLK_PLLM (8)
#define MICROPY_HW_CLK_PLLN (336)
#define MICROPY_HW_CLK_PLLP (RCC_PLLP_DIV2)
#define MICROPY_HW_CLK_PLLQ (7)


// UART config
#define MICROPY_HW_UART1_TX     (pin_A9)   // PA9,PB6
#define MICROPY_HW_UART1_RX     (pin_A10)  // PA10,PB7
#define MICROPY_HW_UART2_TX     (pin_A2)  // PA2,PD5
#define MICROPY_HW_UART2_RX     (pin_A3)  // PA3,PD6
#define MICROPY_HW_UART2_RTS    (pin_A1)  // PA1,PD4
#define MICROPY_HW_UART2_CTS    (pin_D3)  // PA0,PD3 - A0 is wake-up switch

// #define MICROPY_HW_UART3_TX     (pin_D8)  // PB10,PC10,PD8  - D8 also used for FSMC/LCD
// #define MICROPY_HW_UART3_RX     (pin_D9)  // PB11,PC11,PD9  - D9 also used for FSMC/LCD
// #define MICROPY_HW_UART3_RTS    (pin_D12) // PB14,PD12
// #define MICROPY_HW_UART3_CTS    (pin_D11) // PB13,PD11

// conflicts with SDIO (PC10) or wake-up function (PA0)
// #define MICROPY_HW_UART4_TX     (pin_A0)  // PA0,PC10 
// #define MICROPY_HW_UART4_RX     (pin_A1)  // PA1,PC11

// conflicts with SDIO
//#define MICROPY_HW_UART5_TX     (pin_C12) // PC12
//#define MICROPY_HW_UART5_TX     (pin_D2)  // PD2

#define MICROPY_HW_UART6_TX     (pin_C6) // PC6,PG14
#define MICROPY_HW_UART6_RX     (pin_C7) // PC7,PG9

#if 0
// CMSIS support seems missing..
#define MICROPY_HW_UART7_TX     (pin_E8) // PE8,PF7
#define MICROPY_HW_UART7_RX     (pin_E7) // PE7,PF6

#define MICROPY_HW_UART8_TX     (pin_E1) // PE1
#define MICROPY_HW_UART8_RX     (pin_E0) // PE0
#endif

// I2S busses
// I2S2_CK  PB13
// I2S2_MCK PC6
// I2S2_SD  PB15
// I2S2_WS  PB12
// I2S3_CK  PB3
// I2S3_MCK PC7
// I2S3_SD  PB5
// I2S3_WS  PA15

// I2C busses
#define MICROPY_HW_I2C1_SCL (pin_B6)
#define MICROPY_HW_I2C1_SDA (pin_B7)
#define MICROPY_HW_I2C2_SCL (pin_B10)
#define MICROPY_HW_I2C2_SDA (pin_B11)

// SPI busses
#define MICROPY_HW_SPI1_NSS  (pin_A4)  // PA4
#define MICROPY_HW_SPI1_SCK  (pin_A5)  // PA5,PB3
#define MICROPY_HW_SPI1_MISO (pin_A6)  // PA6,PB4
#define MICROPY_HW_SPI1_MOSI (pin_A7)  // PA7,PB5

#define MICROPY_HW_SPI2_NSS  (pin_B12) // PB12
#define MICROPY_HW_SPI2_SCK  (pin_B13) // PB13
#define MICROPY_HW_SPI2_MISO (pin_B14) // PB14
#define MICROPY_HW_SPI2_MOSI (pin_B15) // PB15

#define MICROPY_HW_SPI3_NSS  (pin_A15) // PA15
#define MICROPY_HW_SPI3_SCK  (pin_B3)  // PB3  - pin wired to nRF24L01+ connector on board
#define MICROPY_HW_SPI3_MISO (pin_B4)  // PB4  - pin wired to nRF24L01+ connector on board
#define MICROPY_HW_SPI3_MOSI (pin_B5)  // PB5  - pin wired to nRF24L01+ connector on board
                                       // PB6 - NRF_CE
                                       // PB7 - NRF_CS
                                       // PB8 - NRF_IRQ
// SPI Flash CS on PB0
// nRF24L01 CE  on PB6
// nRF24L01 CS  on PB7

#define MICROPY_HW_USRSW_PIN        (pin_E4)
#define MICROPY_HW_USRSW_PULL       (GPIO_PULLUP)
#define MICROPY_HW_USRSW_EXTI_MODE  (GPIO_MODE_IT_FALLING)
#define MICROPY_HW_USRSW_PRESSED    (0)

#define MICROPY_HW_LED1             (pin_A6)      // conflicts with use of SPI1 MISO
#define MICROPY_HW_LED2             (pin_A7)      // conflicts with use of SPI1 MOSI
#define MICROPY_HW_LED_ON(pin)      (mp_hal_pin_low(pin))
#define MICROPY_HW_LED_OFF(pin)     (mp_hal_pin_high(pin))

// SD card detect switch
// disabled until workaround for lack of MICROPY_HW_SDCARD_DETECT_PIN thing is implemented in driver
//#define MICROPY_HW_SDCARD_DETECT_PIN        (pin_A8)
//#define MICROPY_HW_SDCARD_DETECT_PULL       (GPIO_PULLUP)
//#define MICROPY_HW_SDCARD_DETECT_PRESENT    (GPIO_PIN_RESET)
// 1      - PC10 - DAT2/RES
// 2      - PC11 - CD/DAT3/CS
// 3      - PD2  - CMD/DI
// 4      - VCC  - VDD
// 5      - PC12 - CLK/SCLK
// 6      - GND  - VSS
// 7      - PC8  - DAT0/D0
// 8      - PC9  - DAT1/RES
// 9  SW2 - GND
// 10 SW1 - PA8


// USB config
// #define MICROPY_HW_USB_VBUS_DETECT_PIN (pin_A9)
// #define MICROPY_HW_USB_OTG_ID_PIN      (pin_A10)

//#define MICROPY_HW_UART_REPL    PYB_UART_1
//#define MICROPY_HW_UART_REPL_BAUD  (115200)

