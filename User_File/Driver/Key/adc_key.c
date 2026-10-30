#include "adc_key.h"

#include "main.h"

static ADC_HandleTypeDef s_key_adc;
static adc_key_t s_stable = ADC_KEY_NONE;
static adc_key_t s_candidate = ADC_KEY_NONE;
static uint8_t s_stable_count;

bool ADC_Key_Init(void)
{
    GPIO_InitTypeDef gpio = {0};
    RCC_PeriphCLKInitTypeDef clock = {0};
    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef channel = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &gpio);

    clock.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    clock.PLL2.PLL2M = 2;
    clock.PLL2.PLL2N = 16;
    clock.PLL2.PLL2P = 2;
    clock.PLL2.PLL2Q = 2;
    clock.PLL2.PLL2R = 2;
    clock.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
    clock.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    clock.PLL2.PLL2FRACN = 0;
    clock.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    if (HAL_RCCEx_PeriphCLKConfig(&clock) != HAL_OK)
    {
        return false;
    }

    __HAL_RCC_ADC12_CLK_ENABLE();

    s_key_adc.Instance = ADC1;
    s_key_adc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
    s_key_adc.Init.Resolution = ADC_RESOLUTION_16B;
    s_key_adc.Init.ScanConvMode = ADC_SCAN_DISABLE;
    s_key_adc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    s_key_adc.Init.LowPowerAutoWait = DISABLE;
    s_key_adc.Init.ContinuousConvMode = DISABLE;
    s_key_adc.Init.NbrOfConversion = 1;
    s_key_adc.Init.DiscontinuousConvMode = DISABLE;
    s_key_adc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    s_key_adc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    s_key_adc.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
    s_key_adc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    s_key_adc.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
    s_key_adc.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&s_key_adc) != HAL_OK)
    {
        return false;
    }

    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&s_key_adc, &multimode) != HAL_OK)
    {
        return false;
    }

    channel.Channel = ADC_CHANNEL_19;
    channel.Rank = ADC_REGULAR_RANK_1;
    channel.SamplingTime = ADC_SAMPLETIME_32CYCLES_5;
    channel.SingleDiff = ADC_SINGLE_ENDED;
    channel.OffsetNumber = ADC_OFFSET_NONE;
    channel.Offset = 0;
    channel.OffsetSignedSaturation = DISABLE;
    if (HAL_ADC_ConfigChannel(&s_key_adc, &channel) != HAL_OK)
    {
        return false;
    }

    s_stable = ADC_KEY_NONE;
    s_candidate = ADC_KEY_NONE;
    s_stable_count = 0U;

    return HAL_ADCEx_Calibration_Start(
               &s_key_adc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) == HAL_OK;
}

uint16_t ADC_Key_Read(void)
{
    uint16_t value = UINT16_MAX;

    if (HAL_ADC_Start(&s_key_adc) == HAL_OK)
    {
        if (HAL_ADC_PollForConversion(&s_key_adc, 2U) == HAL_OK)
        {
            value = (uint16_t)HAL_ADC_GetValue(&s_key_adc);
        }
        (void)HAL_ADC_Stop(&s_key_adc);
    }
    return value;
}

adc_key_t ADC_Key_Decode(uint16_t adc)
{
    if (adc < ADC_KEY_CENTER_MAX)
    {
        return ADC_KEY_CENTER;
    }
    if (adc > ADC_KEY_DOWN_MIN && adc < ADC_KEY_DOWN_MAX)
    {
        return ADC_KEY_DOWN;
    }
    if (adc > ADC_KEY_UP_MIN && adc < ADC_KEY_UP_MAX)
    {
        return ADC_KEY_UP;
    }
    if (adc > ADC_KEY_LEFT_MIN && adc < ADC_KEY_LEFT_MAX)
    {
        return ADC_KEY_LEFT;
    }
    if (adc > ADC_KEY_RIGHT_MIN && adc < ADC_KEY_RIGHT_MAX)
    {
        return ADC_KEY_RIGHT;
    }
    return ADC_KEY_NONE;
}

adc_key_t ADC_Key_Scan(uint16_t adc)
{
    adc_key_t current = ADC_Key_Decode(adc);

    if (current == s_candidate)
    {
        if (s_stable_count < ADC_KEY_DEBOUNCE_COUNT)
        {
            s_stable_count++;
        }
    }
    else
    {
        s_candidate = current;
        s_stable_count = 1U;
    }

    if (s_stable_count >= ADC_KEY_DEBOUNCE_COUNT && s_candidate != s_stable)
    {
        adc_key_t previous = s_stable;
        s_stable = s_candidate;
        if (previous == ADC_KEY_NONE && s_stable != ADC_KEY_NONE)
        {
            return s_stable;
        }
    }
    return ADC_KEY_NONE;
}

const char *ADC_Key_Name(adc_key_t key)
{
    switch (key)
    {
        case ADC_KEY_UP:     return "UP";
        case ADC_KEY_DOWN:   return "DOWN";
        case ADC_KEY_LEFT:   return "LEFT";
        case ADC_KEY_RIGHT:  return "RIGHT";
        case ADC_KEY_CENTER: return "CENTER";
        default:             return "NONE";
    }
}
