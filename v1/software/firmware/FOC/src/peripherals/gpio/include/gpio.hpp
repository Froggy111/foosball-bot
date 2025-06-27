#pragma once

#include <stdint.h>
#include <stm32g4xx_hal.h>

namespace gpio {

enum class Pin : uint16_t;
enum class Mode : uint32_t;
enum class Pull : uint32_t;
enum class Speed : uint32_t;
enum class AF : uint8_t;

constexpr bool LOW = false;
constexpr bool HIGH = true;

struct PinConfig {
    GPIO_TypeDef *port;
    Pin pin;
    AF alternate_function;
};

using InterruptFn = void (*)(void *);

void init(const PinConfig &pin_config, Mode mode, Pull pull, Speed speed);

void attach_interrupt(const PinConfig &pin_config, Mode mode, Pull pull,
                      Speed speed, InterruptFn callback, void *args);

inline bool read(const PinConfig &pin_config) {
    HAL_GPIO_ReadPin(pin_config.port, (uint16_t)pin_config.pin);
}
inline void write(const PinConfig &pin_config, bool state) {
    HAL_GPIO_WritePin(pin_config.port, (uint16_t)pin_config.pin,
                      (GPIO_PinState)state);
}

enum class Pin : uint16_t {
    PIN0 = GPIO_PIN_0,
    PIN1 = GPIO_PIN_1,
    PIN2 = GPIO_PIN_2,
    PIN3 = GPIO_PIN_3,
    PIN4 = GPIO_PIN_4,
    PIN5 = GPIO_PIN_5,
    PIN6 = GPIO_PIN_6,
    PIN7 = GPIO_PIN_7,
    PIN8 = GPIO_PIN_8,
    PIN9 = GPIO_PIN_9,
    PIN10 = GPIO_PIN_10,
    PIN11 = GPIO_PIN_11,
    PIN12 = GPIO_PIN_12,
    PIN13 = GPIO_PIN_13,
    PIN14 = GPIO_PIN_14,
    PIN15 = GPIO_PIN_15,
    ALL = GPIO_PIN_All
};

enum class Mode : uint32_t {
    INPUT = GPIO_MODE_INPUT, /*!< Input Floating Mode                   */
    OUTPUT_PP = GPIO_MODE_OUTPUT_PP, /*!< Output Push Pull Mode */
    OUTPUT_OD = GPIO_MODE_OUTPUT_OD, /*!< Output Open Drain Mode */
    AF_PP = GPIO_MODE_AF_PP,   /*!< Alternate Function Push Pull Mode     */
    AF_OD = GPIO_MODE_AF_OD,   /*!< Alternate Function Open Drain Mode    */
    ANALOG = GPIO_MODE_ANALOG, /*!< Analog Mode  */
    INT_RISING = GPIO_MODE_IT_RISING,   /*!< External Interrupt Mode with
                                           Rising   edge trigger detection   */
    INT_FALLING = GPIO_MODE_IT_FALLING, /*!< External Interrupt Mode with
                                           Falling edge trigger detection */
    INT_RISING_FALLING =
        GPIO_MODE_IT_RISING_FALLING,     /*!< External Interrupt Mode with
                                            Rising/Falling edge trigger
                                            detection   */
    EVT_RISING = GPIO_MODE_EVT_RISING,   /*!< External Event Mode with Rising
                                            edge trigger detection */
    EVT_FALLING = GPIO_MODE_EVT_FALLING, /*!< External Event Mode with Falling
                                            edge trigger detection */
    EVT_RISING_FALLING =
        GPIO_MODE_EVT_RISING_FALLING /*!< External Event Mode with
                                        Rising/Falling edge trigger
                                        detection */
};

enum class Pull : uint32_t {
    DOWN = GPIO_PULLDOWN,
    UP = GPIO_PULLUP,
    NOPULL = GPIO_NOPULL
};

enum class Speed : uint32_t {
    LOW = GPIO_SPEED_FREQ_LOW,
    MEDIUM = GPIO_SPEED_FREQ_MEDIUM,
    HIGH = GPIO_SPEED_FREQ_HIGH,
#ifdef GPIO_SPEED_FREQ_VERY_HIGH
    VERY_HIGH = GPIO_SPEED_FREQ_VERY_HIGH,
#else
#error GPIO_SPEED_FREQ_VERY_HIGH not available.
#endif
};

// STM32G4xx specific Alternate Function mappings
#ifdef STM32G4
enum class AF : uint8_t {
    NONE = 0xFF,
    /**
     * @brief   AF 0 selection
     */
    AF0_RTC_50Hz = 0x00, /* RTC_50Hz Alternate Function mapping */
    AF0_MCO = 0x00,   /* MCO (MCO1 and MCO2) Alternate Function mapping      */
    AF0_SWJ = 0x00,   /* SWJ (SWD and JTAG) Alternate Function mapping      */
    AF0_TRACE = 0x00, /* TRACE Alternate Function mapping    */

    /**
     * @brief   AF 1 selection
     */
    AF1_TIM2 = 0x01, /* TIM2 Alternate Function mapping   */
#ifdef TIM5
    AF1_TIM5 = 0x01,  /* TIM5 Alternate Function mapping   */
#endif                /* TIM5 */
    AF1_TIM16 = 0x01, /* TIM16 Alternate Function mapping  */
    AF1_TIM17 = 0x01, /* TIM17 Alternate Function mapping  */
    AF1_TIM17_COMP1 =
        0x01,          /* TIM17/COMP1 Break in Alternate Function mapping  */
    AF1_TIM15 = 0x01,  /* TIM15 Alternate Function mapping  */
    AF1_LPTIM1 = 0x01, /* LPTIM1 Alternate Function mapping */
    AF1_IR = 0x01,     /* IR Alternate Function mapping     */

    /**
     * @brief   AF 2 selection
     */
    AF2_TIM1 = 0x02, /* TIM1 Alternate Function mapping  */
    AF2_TIM2 = 0x02, /* TIM2 Alternate Function mapping  */
    AF2_TIM3 = 0x02, /* TIM3 Alternate Function mapping  */
    AF2_TIM4 = 0x02, /* TIM4 Alternate Function mapping  */
#ifdef TIM5
    AF2_TIM5 = 0x02,  /* TIM5 Alternate Function mapping  */
#endif                /* TIM5 */
    AF2_TIM8 = 0x02,  /* TIM8 Alternate Function mapping  */
    AF2_TIM15 = 0x02, /* TIM15 Alternate Function mapping */
    AF2_TIM16 = 0x02, /* TIM16 Alternate Function mapping */
#ifdef TIM20
    AF2_TIM20 = 0x02,      /* TIM20 Alternate Function mapping */
#endif                     /* TIM20 */
    AF2_TIM1_COMP1 = 0x02, /* TIM1/COMP1 Break in Alternate Function mapping */
    AF2_TIM15_COMP1 =
        0x02, /* TIM15/COMP1 Break in Alternate Function mapping  */
    AF2_TIM16_COMP1 =
        0x02, /* TIM16/COMP1 Break in Alternate Function mapping  */
#ifdef TIM20
    AF2_TIM20_COMP1 =
        0x02, /* TIM20/COMP1 Break in Alternate Function mapping  */
    AF2_TIM20_COMP2 =
        0x02,         /* TIM20/COMP2 Break in Alternate Function mapping  */
#endif                /* TIM20 */
    AF2_I2C3 = 0x02,  /* I2C3 Alternate Function mapping  */
    AF2_COMP1 = 0x02, /* COMP1 Alternate Function mapping */

    /**
     * @brief   AF 3 selection
     */
    AF3_TIM15 = 0x03, /* TIM15 Alternate Function mapping   */
#ifdef TIM20
    AF3_TIM20 = 0x03, /* TIM20 Alternate Function mapping   */
#endif                /* TIM20 */
    AF3_UCPD1 = 0x03, /* UCPD1 Alternate Function mapping   */
    AF3_I2C3 = 0x03,  /* I2C3 Alternate Function mapping    */
#ifdef I2C4
    AF3_I2C4 = 0x03, /* I2C4 Alternate Function mapping    */
#endif               /* I2C4 */
#ifdef HRTIM1
    AF3_HRTIM1 = 0x03, /* HRTIM1 Alternate Function mapping  */
#endif                 /* HRTIM1 */
#ifdef QUADSPI
    AF3_QUADSPI = 0x03, /* QUADSPI Alternate Function mapping */
#endif                  /* QUADSPI */
    AF3_TIM8 = 0x03,    /* TIM8 Alternate Function mapping    */
    AF3_SAI1 = 0x03,    /* SAI1 Alternate Function mapping  */
    AF3_COMP3 = 0x03,   /* COMP3 Alternate Function mapping */

    /**
     * @brief   AF 4 selection
     */
    AF4_TIM1 = 0x04,       /* TIM1 Alternate Function mapping    */
    AF4_TIM8 = 0x04,       /* TIM8 Alternate Function mapping    */
    AF4_TIM16 = 0x04,      /* TIM16 Alternate Function mapping   */
    AF4_TIM17 = 0x04,      /* TIM17 Alternate Function mapping   */
    AF4_TIM8_COMP1 = 0x04, /* TIM8/COMP1 Break in Alternate Function mapping  */
    AF4_I2C1 = 0x04,       /* I2C1 Alternate Function mapping    */
    AF4_I2C2 = 0x04,       /* I2C2 Alternate Function mapping    */
    AF4_I2C3 = 0x04,       /* I2C3 Alternate Function mapping    */
#ifdef I2C4
    AF4_I2C4 = 0x04, /* I2C4 Alternate Function mapping    */
#endif               /* I2C4 */

    /**
     * @brief   AF 5 selection
     */
    AF5_SPI1 = 0x05, /* SPI1 Alternate Function mapping       */
    AF5_SPI2 = 0x05, /* SPI2 Alternate Function mapping       */
#ifdef SPI4
    AF5_SPI4 = 0x05,       /* SPI4 Alternate Function mapping       */
#endif                     /* SPI4 */
    AF5_IR = 0x05,         /* IR Alternate Function mapping         */
    AF5_TIM8 = 0x05,       /* TIM8 Alternate Function mapping       */
    AF5_TIM8_COMP1 = 0x05, /* TIM8/COMP1 Break in Alternate Function mapping  */
    AF5_UART4 = 0x05,      /* UART4 Alternate Function mapping      */
#ifdef UART5
    AF5_UART5 = 0x05,   /* UART5 Alternate Function mapping      */
#endif                  /* UART5 */
    AF5_I2S2ext = 0x05, /* I2S2ext_SD Alternate Function mapping */

    /**
     * @brief   AF 6 selection
     */
    AF6_SPI2 = 0x06, /* SPI2 Alternate Function mapping       */
    AF6_SPI3 = 0x06, /* SPI3 Alternate Function mapping       */
    AF6_TIM1 = 0x06, /* TIM1 Alternate Function mapping       */
#ifdef TIM5
    AF6_TIM5 = 0x06, /* TIM5 Alternate Function mapping       */
#endif               /* TIM5 */
    AF6_TIM8 = 0x06, /* TIM8 Alternate Function mapping       */
#ifdef TIM20
    AF6_TIM20 = 0x06,      /* TIM20 Alternate Function mapping      */
#endif                     /* TIM20 */
    AF6_TIM1_COMP1 = 0x06, /* TIM1/COMP1 Break in Alternate Function mapping  */
    AF6_TIM1_COMP2 = 0x06, /* TIM1/COMP2 Break in Alternate Function mapping  */
    AF6_TIM8_COMP2 = 0x06, /* TIM8/COMP2 Break in Alternate Function mapping  */
    AF6_IR = 0x06,         /* IR Alternate Function mapping         */
    AF6_I2S3ext = 0x06,    /* I2S3ext_SD Alternate Function mapping */

    /**
     * @brief   AF 7 selection
     */
    AF7_USART1 = 0x07, /* USART1 Alternate Function mapping  */
    AF7_USART2 = 0x07, /* USART2 Alternate Function mapping  */
    AF7_USART3 = 0x07, /* USART3 Alternate Function mapping  */
#ifdef COMP5
    AF7_COMP5 = 0x07, /* COMP5 Alternate Function mapping   */
#endif                /* COMP5 */
#ifdef COMP6
    AF7_COMP6 = 0x07, /* COMP6 Alternate Function mapping   */
#endif                /* COMP6 */
#ifdef COMP7
    AF7_COMP7 = 0x07, /* COMP7 Alternate Function mapping   */
#endif                /* COMP7 */

    /**
     * @brief   AF 8 selection
     */
    AF8_COMP1 = 0x08, /* COMP1 Alternate Function mapping   */
    AF8_COMP2 = 0x08, /* COMP2 Alternate Function mapping   */
    AF8_COMP3 = 0x08, /* COMP3 Alternate Function mapping   */
    AF8_COMP4 = 0x08, /* COMP4 Alternate Function mapping   */
#ifdef COMP5
    AF8_COMP5 = 0x08, /* COMP5 Alternate Function mapping   */
#endif                /* COMP5 */
#ifdef COMP6
    AF8_COMP6 = 0x08, /* COMP6 Alternate Function mapping   */
#endif                /* COMP6 */
#ifdef COMP7
    AF8_COMP7 = 0x08, /* COMP7 Alternate Function mapping   */
#endif                /* COMP7 */
    AF8_I2C3 = 0x08,  /* I2C3 Alternate Function mapping    */
#ifdef I2C4
    AF8_I2C4 = 0x08,    /* I2C4 Alternate Function mapping    */
#endif                  /* I2C4 */
    AF8_LPUART1 = 0x08, /* LPUART1 Alternate Function mapping */
    AF8_UART4 = 0x08,   /* UART4 Alternate Function mapping   */
#ifdef UART5
    AF8_UART5 = 0x08, /* UART5 Alternate Function mapping   */
#endif                /* UART5 */

    /**
     * @brief   AF 9 selection
     */
    AF9_TIM1 = 0x09,       /* TIM1 Alternate Function mapping    */
    AF9_TIM8 = 0x09,       /* TIM8 Alternate Function mapping    */
    AF9_TIM15 = 0x09,      /* TIM15 Alternate Function mapping   */
    AF9_TIM1_COMP1 = 0x09, /* TIM1/COMP1 Break in Alternate Function mapping */
    AF9_TIM8_COMP1 = 0x09, /* TIM8/COMP1 Break in Alternate Function mapping */
    AF9_TIM15_COMP1 =
        0x09,          /* TIM15/COMP1 Break in Alternate Function mapping  */
    AF9_FDCAN1 = 0x09, /* FDCAN1 Alternate Function mapping  */
#ifdef FDCAN2
    AF9_FDCAN2 = 0x09, /* FDCAN2 Alternate Function mapping  */
#endif                 /* FDCAN2 */

    /**
     * @brief   AF 10 selection
     */
    AF10_TIM2 = 0x0A,       /* TIM2 Alternate Function mapping    */
    AF10_TIM3 = 0x0A,       /* TIM3 Alternate Function mapping    */
    AF10_TIM4 = 0x0A,       /* TIM4 Alternate Function mapping    */
    AF10_TIM8 = 0x0A,       /* TIM8 Alternate Function mapping    */
    AF10_TIM17 = 0x0A,      /* TIM17 Alternate Function mapping   */
    AF10_TIM8_COMP2 = 0x0A, /* TIM8/COMP2 Break in Alternate Function mapping */
    AF10_TIM17_COMP1 =
        0x0A, /* TIM17/COMP1 Break in Alternate Function mapping   */
#ifdef QUADSPI
    AF10_QUADSPI = 0x0A, /* OctoSPI Manager Port 1 Alternate Function mapping */
#endif                   /* QUADSPI */

    /**
     * @brief   AF 11 selection
     */
    AF11_FDCAN1 = 0x0B, /* FDCAN1 Alternate Function mapping  */
#ifdef FDCAN3
    AF11_FDCAN3 = 0x0B,     /* FDCAN3 Alternate Function mapping  */
#endif                      /* FDCAN3 */
    AF11_TIM1 = 0x0B,       /* TIM1 Alternate Function mapping    */
    AF11_TIM8 = 0x0B,       /* TIM8 Alternate Function mapping    */
    AF11_TIM8_COMP1 = 0x0B, /* TIM8/COMP1 Break in Alternate Function mapping */
    AF11_LPTIM1 = 0x0B,     /* LPTIM1 Alternate Function mapping  */

    /**
     * @brief   AF 12 selection
     */
    AF12_LPUART1 = 0x0C,    /* LPUART1 Alternate Function mapping */
    AF12_TIM1 = 0x0C,       /* TIM1 Alternate Function mapping    */
    AF12_TIM1_COMP1 = 0x0C, /* TIM1/COMP1 Break in Alternate Function mapping */
    AF12_TIM1_COMP2 = 0x0C, /* TIM1/COMP2 Break in Alternate Function mapping */
#ifdef HRTIM1
    AF12_HRTIM1 = 0x0C, /* HRTIM1 Alternate Function mapping  */
#endif                  /* HRTIM1 */
#ifdef FMC_BANK1
    AF12_FMC = 0x0C,  /* FMC Alternate Function mapping     */
#endif                /* FMC_BANK1 */
    AF12_SAI1 = 0x0C, /* SAI1 Alternate Function mapping  */

/**
 * @brief   AF 13 selection
 */
#ifdef HRTIM1
    AF13_HRTIM1 = 0x0D, /* HRTIM1 Alternate Function mapping  */
#endif                  /* HRTIM1 */
    AF13_SAI1 = 0x0D,   /* SAI1 Alternate Function mapping  */

    /**
     * @brief   AF 14 selection
     */
    AF14_TIM2 = 0x0E,  /* TIM2 Alternate Function mapping   */
    AF14_TIM15 = 0x0E, /* TIM15 Alternate Function mapping   */
    AF14_UCPD1 = 0x0E, /* UCPD1 Alternate Function mapping  */
    AF14_SAI1 = 0x0E,  /* SAI1 Alternate Function mapping  */
    AF14_UART4 = 0x0E, /* UART4 Alternate Function mapping      */
#ifdef UART5
    AF14_UART5 = 0x0E, /* UART5 Alternate Function mapping      */
#endif                 /* UART5 */

    /**
     * @brief   AF 15 selection
     */
    AF15_EVENTOUT = 0x0F /* EVENTOUT Alternate Function mapping */
};
#endif

}  // namespace gpio
