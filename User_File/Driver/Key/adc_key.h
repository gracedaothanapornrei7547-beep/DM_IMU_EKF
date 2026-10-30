#ifndef DM02_ADC_KEY_H
#define DM02_ADC_KEY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * DM-MC02 five-way key ADC ranges (ADC1 channel 19 on PA5, 16-bit).
 * Idle is normally near 65535.
 */
#define ADC_KEY_CENTER_MAX       2000U
#define ADC_KEY_DOWN_MIN         8000U
#define ADC_KEY_DOWN_MAX        18000U
#define ADC_KEY_UP_MIN          21000U
#define ADC_KEY_UP_MAX          31000U
#define ADC_KEY_LEFT_MIN        33000U
#define ADC_KEY_LEFT_MAX        43000U
#define ADC_KEY_RIGHT_MIN       46000U
#define ADC_KEY_RIGHT_MAX       56000U
#define ADC_KEY_DEBOUNCE_COUNT      3U

typedef enum
{
    ADC_KEY_NONE = 0,
    ADC_KEY_UP,
    ADC_KEY_DOWN,
    ADC_KEY_LEFT,
    ADC_KEY_RIGHT,
    ADC_KEY_CENTER
} adc_key_t;

/* Configure PA5 and ADC1 channel 19 for 16-bit software-triggered sampling. */
bool ADC_Key_Init(void);

/* Read one raw ADC sample. Returns UINT16_MAX when conversion fails. */
uint16_t ADC_Key_Read(void);

/* Decode one raw sample without debounce. */
adc_key_t ADC_Key_Decode(uint16_t adc);

/* Return one event after three stable samples; holding a key does not repeat. */
adc_key_t ADC_Key_Scan(uint16_t adc);

/* Human-readable name used by diagnostics/UI. */
const char *ADC_Key_Name(adc_key_t key);

#ifdef __cplusplus
}
#endif

#endif /* DM02_ADC_KEY_H */
