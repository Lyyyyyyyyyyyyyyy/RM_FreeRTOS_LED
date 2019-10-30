#ifndef __M6020_H
#define __M6020_H

#include "user_common.h"

#define M6020_READID_START	0x205
#define M6020_READID_END	  0x207
#define M6020_SENDID		    0x1FF

#define M6020_CurrentRatio	819.2f	//16384/20A = 819.2->1A

extern M6020s_t M6020s[2];

void M6020_setCurrent(int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void M6020_getInfo(CanRxMsg RxMessage);

#endif /* __M6020_H */
