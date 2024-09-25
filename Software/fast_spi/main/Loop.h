#ifndef __LOOP_H__
#define __LOOP_H__

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

#include <stdint.h>
#include <stdbool.h>

void Loop_Init();
uint32_t Loop_dequeue();
bool Loop_is_vehicle_detected(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif