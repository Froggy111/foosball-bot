#ifndef __ADC_H__
#define __ADC_H__

#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;

extern ADC_HandleTypeDef hadc2;

void MX_ADC1_Init(void);
void MX_ADC2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */
