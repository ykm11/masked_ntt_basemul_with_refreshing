/*
    This file is part of the ChipWhisperer Example Targets
    Copyright (C) 2012-2017 NewAE Technology Inc.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "hal.h"
#include "simpleserial.h"
#include <stdint.h>
#include <stdlib.h>

#include <arm_acle.h>
#include "stm32f3xx.h" // firmware/mcu/hal/stm32f3/CMSIS/device/stm32f3xx.h

#define MONT -1044 // 2^16 mod q
#define QINV -3327 // q^-1 mod 2^16
#define KYBER_Q 3329


#define PACK16(hi, lo)   __PKHBT(hi, lo, 16)

void masked_basemul(int16_t *r,
                    const int16_t *a0, const int16_t *a1,
                    const int16_t *b, int16_t zeta,
                    const int16_t *rand) {
    int32_t acc;
    int16_t zeta_b = montgomery_reduce((int32_t)b[1] * zeta);
    
    // r[0] = a0_0*b0 + r01     + a1_0*(b1*zeta) + r23
    acc  = __SMUAD( PACK16(a0[0],    rand[0]),
                    PACK16(b[0],     rand[1])
                    );
    acc  = __SMLAD( PACK16(a1[0],    rand[2]),
                    PACK16(zeta_b,   rand[3]),
                    acc );
    r[0] = montgomery_reduce(acc);
    
    // r[1] = a0_1*b0 - r01     + a1_1*(b1*zeta) - r23
    acc  = __SMUAD( PACK16(a0[1],   -rand[0]),
                    PACK16(b[0],     rand[1])
                    );
    acc  = __SMLAD( PACK16(a1[1],   -rand[2]),
                    PACK16(zeta_b,   rand[3]),
                    acc );
    r[1] = montgomery_reduce(acc);
    
    // r[2] = a0_0*b1 + r45     + a1_0*b0     + r67
    acc  = __SMUAD( PACK16(a0[0],    rand[4]),
                    PACK16(b[1],     rand[5])
                    );
    acc  = __SMLAD( PACK16(a1[0],    rand[6]),
                    PACK16(b[0],     rand[7]),
                    acc );
    r[2] = montgomery_reduce(acc);

    // r[3] = a0_1*b1 - r45     + a1_1*b0     - r67
    acc  = __SMUAD( PACK16(a0[1],   -rand[4]),
                    PACK16(b[1],    rand[5])
                    );
    acc += __SMLAD( PACK16(a1[1],   -rand[6]),
                    PACK16(b[0],    rand[7]),
                    acc );
    r[3] = montgomery_reduce(acc);
}

uint8_t masked_mul(uint8_t* pt, uint8_t len)
{
    // a0_0, a0_1, a1_0, a1_1, b0, b1, r0, r1, r2, r3, r4, r5, r6, r7;

    int16_t a0[2];
    int16_t a1[2];
    int16_t b[2];
    int16_t rand[8];
    int16_t d[8];

    
    int16_t zeta = -1044;

    a0[0] = pt[0];
    a0[0] |= ((int16_t)pt[1] << 8);
    a0[1] = pt[2];
    a0[1] |= ((int16_t)pt[3] << 8);

    a1[0] = pt[4];
    a1[0] |= ((int16_t)pt[5] << 8);
    a1[1] = pt[6];
    a1[1] |= ((int16_t)pt[7] << 8);
    
    b[0] = pt[8];
    b[0] |= ((int16_t)pt[9] << 8);
    b[1] = pt[10];
    b[1] |= ((int16_t)pt[11] << 8);

    for (int16_t i = 0; i < 8; i++) {
        rand[i] = pt[12 + 2*i];
        rand[i] |= ((int16_t)pt[12 + 2*i + 1] << 8);
    }

    trigger_high();

  #ifdef ADD_JITTER
    for (volatile uint8_t k = 0; k < (*pt & 0x0F); k++);
  #endif
    
    // int16_t *result, uint16_t *a0 uint16_t *a1, uint16_t *b, int16_t zeta
    masked_basemul(d, a0, a1, b, zeta, rand);
	  trigger_low();

	simpleserial_put('r', 8, (uint8_t*)d);
	return 0x00;
}


uint8_t mul(uint8_t* pt, uint8_t len) {

    int16_t a[2];
    int16_t b[2];
    int16_t d[2];
    
    int16_t zeta = -1044;

    a[0] = pt[0];
    a[0] |= ((int16_t)pt[1] << 8);
    a[1] = pt[2];
    a[1] |= ((int16_t)pt[3] << 8);

    b[0] = pt[4];
    b[0] |= ((int16_t)pt[5] << 8);
    b[1] = pt[6];
    b[1] |= ((int16_t)pt[7] << 8);
    
    trigger_high();

  #ifdef ADD_JITTER
    for (volatile uint8_t k = 0; k < (*pt & 0x0F); k++);
  #endif
    basemul(d, a, b, zeta);    
	  trigger_low();

	  simpleserial_put('r', 4, (uint8_t*)d);
	  return 0x00;
}


int main(void) {

    platform_init();
    init_uart();
    trigger_setup();

	  simpleserial_init();

    simpleserial_addcmd('t', 28, masked_mul);
    simpleserial_addcmd('p', 8, mul);
    
    while(1)
        simpleserial_get();
}
