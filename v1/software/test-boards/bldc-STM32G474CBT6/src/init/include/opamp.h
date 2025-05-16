#ifndef __OPAMP_H__
#define __OPAMP_H__

#ifdef __cplusplus
extern "C" {
#endif

extern OPAMP_HandleTypeDef hopamp1;

extern OPAMP_HandleTypeDef hopamp2;

extern OPAMP_HandleTypeDef hopamp3;

void MX_OPAMP1_Init(void);
void MX_OPAMP2_Init(void);
void MX_OPAMP3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __OPAMP_H__ */
