#pragma once
#include "fec.h"
#ifdef ESP_PLATFORM
#include "esp_heap_caps.h"
#endif

#define SWAP(a,b,t) {t tmp; tmp=a; a=b; b=tmp;}
#define NEW_GF_MATRIX(rows, cols) (gf*)malloc(rows * cols)

#define gf_mul(x,y) (*gf_mul_table)[x][y]

#define USE_GF_MULC register const gf * __gf_mulc_

#define GF_MULC0(c) __gf_mulc_ = (const gf*)&((*gf_mul_table)[c])
#define GF_ADDMULC(dst, x) dst ^= __gf_mulc_[x]

#define GF_ADDMULC4(dst, src)  \
    temp1 = *((uint32_t*)dst);  \
    temp1 ^= __gf_mulc_[*src++];  \
    temp1 ^= ((uint32_t)__gf_mulc_[*src++]) << 8;   \
    temp1 ^= ((uint32_t)__gf_mulc_[*src++]) << 16;  \
    temp1 ^= ((uint32_t)__gf_mulc_[*src++]) << 24;  \
    *((uint32_t*)dst) = temp1;   \
    dst+=4;   

/*
 * addmul() computes dst[] = dst[] + c * src[]
 * This is used often, so better optimize it! Currently the loop is
 * unrolled 16 times, a good value for 486 and pentium-class machines.
 * The case c=0 is also optimized, whereas c=1 is not. These
 * calls are unfrequent in my typical apps so I did not bother.
 */
#define addmul(dst, src, c, sz)                 \
    if (c != 0) _addmul1(dst, src, c, sz)

#define UNROLL 16               /* 1, 4, 8, 16 */

/*
 * This section contains the proper FEC encoding/decoding routines.
 * The encoding matrix is computed starting with a Vandermonde matrix,
 * and then transforming it into a systematic matrix.
 */
#define FEC_MAGIC	0xFECC0DEC

#if defined(_MSC_VER)
// actually, some of the flavors (i.e. Enterprise) do support restrict
#define restrict
#endif
#define restrict __restrict

ZFE_FEC::gf ZFE_FEC::modnn(int x) {
    while (x >= 255) {
      x -= 255;
      x = (x >> 8) + (x & 255);
    }
    return x;
  }

void ZFE_FEC::generate_gf(void){
    int i;
    gf mask;

    mask = 1;                     /* x ** 0 = 1 */
    gf_exp[8] = 0;          /* will be updated at the end of the 1st loop */

    /*
    * first, generate the (polynomial representation of) powers of \alpha,
    * which are stored in gf_exp[i] = \alpha ** i .
    * At the same time build gf_log[gf_exp[i]] = i .
    * The first 8 powers are simply bits shifted to the left.
    */
    for (i = 0; i < 8; i++, mask <<= 1) {
      gf_exp[i] = mask;
      gf_log[gf_exp[i]] = i;
      /*
      * If Pp[i] == 1 then \alpha ** i occurs in poly-repr
      if (ZFE_FEC::Pp[i] == '1')
      */
      if (Pp[i] == '1')
        gf_exp[8] ^= mask;
    }
    /*
    * now gf_exp[8] = \alpha ** 8 is complete, so can also
    * compute its inverse.
    */
    gf_log[gf_exp[8]] = 8;
    /*
    * Poly-repr of \alpha ** (i+1) is given by poly-repr of
    * \alpha ** i shifted left one-bit and accounting for any
    * \alpha ** 8 term that may occur when poly-repr of
    * \alpha ** i is shifted.
    */
    mask = 1 << 7;
    for (i = 9; i < 255; i++) {
      if (gf_exp[i - 1] >= mask)
        gf_exp[i] = gf_exp[8] ^ ((gf_exp[i - 1] ^ mask) << 1);
      else
        gf_exp[i] = gf_exp[i - 1] << 1;
      gf_log[gf_exp[i]] = i;
    }
    /*
    * log(0) is not defined, so use a special value
    */
    gf_log[0] = 255;
    /* set the extended gf_exp values for fast multiply */
    for (i = 0; i < 255; i++)
      gf_exp[i + 255] = gf_exp[i];

    /*
    * again special cases. 0 has no inverse. This used to
    * be initialized to 255, but it should make no difference
    * since noone is supposed to read from here.
    */
    inverse[0] = 0;
    inverse[1] = 1;
    for (i = 2; i <= 255; i++)
      inverse[i] = gf_exp[255 - gf_log[i]];
  }

void ZFE_FEC::_init_mul_table(void){
    int i, j;

#ifdef ESP_PLATFORM      
    //performance is 20-30% slower with PSRAM, but we need internal memory desperately for the wifi output buffer
    //review: precalculate and place in flash? (4 byte access?)
    gf_mul_table = (gf_mul_table_p)heap_caps_malloc(256*256, MALLOC_CAP_SPIRAM);
    //gf_mul_table = (gf_mul_table_p)heap_caps_malloc(256*256, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
#else
    gf_mul_table = (gf_mul_table_p)malloc( 256*256 );
#endif

    for (i = 0; i < 256; i++){
      for (j = 0; j < 256; j++)
        (*gf_mul_table)[i][j] = gf_exp[modnn (gf_log[i] + gf_log[j])];
    }

    for (j = 0; j < 256; j++)
      (*gf_mul_table)[0][j] = (*gf_mul_table)[j][0] = 0;
        
  }

void ZFE_FEC::_addmul1(register gf*restrict dst, const register gf*restrict src, gf c, size_t sz)
    
