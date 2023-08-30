/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2021, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file
 * \brief Implementation of IbcHashMap class
 */

#include "CommonDefX86.h"
#include "../IntraPrediction.h"

#ifdef TARGET_SIMD_X86

#include <nmmintrin.h>

#if ENABLE_SIMD_TMP
#if JVET_AD0086_ENHANCED_INTRA_TMP
// Calculation with step of 4
inline uint32_t calcDiff4(const short* pSrc1, const short* pSrc2, const int start, const int end)
{
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 4)
  {
    __m128i vsrc1 = _mm_loadl_epi64((const __m128i*) & pSrc1[iX]);
    __m128i vsrc2 = _mm_loadl_epi64((const __m128i*) & pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
  }
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01 00 11 10
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10 11 00 01
  return _mm_cvtsi128_si32(vsum32);
}
// Calculation with step of 8
inline uint32_t calcDiff8(const short* pSrc1, const short* pSrc2, const int start, const int end)
{
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    __m128i vsrc1 = _mm_loadu_si128((const __m128i*) & pSrc1[iX]);
    __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) & pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
  }
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01 00 11 10
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10 11 00 01
  return _mm_cvtsi128_si32(vsum32);
}
template<X86_VEXT vext>
void calcTemplateDiffSIMD(Pel *ref, unsigned int uiStride, Pel **tarPatch, unsigned int uiPatchWidth,
                          unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType tempType,
                          int requiredTemplate)
{
  int diffSum  = 0;
  int topDiff  = MAX_INT;
  int leftDiff = MAX_INT;
  int iY;
#if JVET_W0069_TMP_BOUNDARY
  Pel *refPatchRow;
  if (tempType == L_SHAPE_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride;
  }
#else
  Pel *refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
#endif
  Pel *    tarPatchRow;
  uint32_t uiSum;

  // horizontal difference
#if JVET_W0069_TMP_BOUNDARY
  if (tempType == L_SHAPE_TEMPLATE)
  {
#endif
    topDiff  = 0;
    leftDiff = 0;
    if (requiredTemplate == 3)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        int iCols = uiPatchWidth;

        // TopLeft
        uiSum = calcDiff4(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
        diffSum += uiSum;

        if (((iCols - TMP_TEMPLATE_SIZE) & 7) == 0)
        {
          uiSum = calcDiff8(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
        else
        {
          uiSum = calcDiff4(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
        diffSum += uiSum;
        topDiff += uiSum;

        if (diffSum > iMax[0] && topDiff > iMax[1])
        {
          break;
        }

        // update location
        refPatchRow += uiStride;
      }
      // TopDiff = top_diff;
      refPatchRow = ref - TMP_TEMPLATE_SIZE;

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;

      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);

        diffSum += uiSum;
        leftDiff += uiSum;

        if (diffSum > iMax[0] && leftDiff > iMax[2])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
    }
    else if (requiredTemplate == 0)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        // int  iRows = uiPatchHeight;
        int iCols = uiPatchWidth;
        if ((iCols & 7) == 0)
        {
          uiSum = calcDiff8(pSrc1, pSrc2, 0, iCols);
        }
        else
        {
          uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);
        }
        diffSum += uiSum;

        if (diffSum > iMax[0])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;

      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);

        diffSum += uiSum;

        if (diffSum > iMax[0])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
    }
    else if (requiredTemplate == 1)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        int iCols = uiPatchWidth;

        if (((iCols - TMP_TEMPLATE_SIZE) & 7) == 0)
        {
          uiSum = calcDiff8(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
        else
        {
          uiSum = calcDiff4(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, iCols);
        }
        topDiff += uiSum;

        if (topDiff > iMax[1])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
    }
    else   // L
    {
      refPatchRow = ref - TMP_TEMPLATE_SIZE;

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;

      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference

        uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);

        leftDiff += uiSum;

        if (leftDiff > iMax[2])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
    }

#if JVET_W0069_TMP_BOUNDARY
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    // horizontal difference
    for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference
      // int  iRows = uiPatchHeight;
      int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
      if ((iCols & 7) == 0)
      {
        uiSum = calcDiff8(pSrc1, pSrc2, 0, iCols);
      }
      else
      {
        uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);
      }
      diffSum += uiSum;

      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }

      // update location
      refPatchRow += uiStride;
    }
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    // vertical difference
    int iCols = TMP_TEMPLATE_SIZE;

    for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference

      uiSum = calcDiff4(pSrc1, pSrc2, 0, iCols);

      diffSum += uiSum;

      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }

      refPatchRow += uiStride;
    }
  }
#endif

  diff[0] = diffSum;
  diff[1] = topDiff;
  diff[2] = leftDiff;
}
#else
template< X86_VEXT vext >
#if JVET_W0069_TMP_BOUNDARY
int calcTemplateDiffSIMD(Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType tempType)
#else
int calcTemplateDiffSIMD( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax )
#endif
{
  int diffSum = 0;
  int iY;
#if JVET_W0069_TMP_BOUNDARY
  Pel* refPatchRow;
  if( tempType == L_SHAPE_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
  }
  else if( tempType == LEFT_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
  }
  else if( tempType == ABOVE_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride;
  }
#else
  Pel* refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
#endif
  Pel* tarPatchRow;
  uint32_t uiSum;

  // horizontal difference
#if JVET_W0069_TMP_BOUNDARY
  if (tempType == L_SHAPE_TEMPLATE)
  {
#endif
    for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference
      // int  iRows = uiPatchHeight;
      int iCols = uiPatchWidth;
      if ((iCols & 7) == 0)
      {
        // Do with step of 8
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 8)
        {
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_lddqu_si128((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      else
      {
        // Do with step of 4
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 4)
        {
          __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }

    // vertical difference
    int iCols = TMP_TEMPLATE_SIZE;

    for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference

      // Do with step of 4
      __m128i vzero  = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      __m128i vsum16 = vzero;
      for (int iX = 0; iX < iCols; iX += 4)
      {
        __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
        __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
        vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
      uiSum            = _mm_cvtsi128_si32(vsum32);

      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }
#if JVET_W0069_TMP_BOUNDARY
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    // horizontal difference
    for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference
      // int  iRows = uiPatchHeight;
      int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
      if ((iCols & 7) == 0)
      {
        // Do with step of 8
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 8)
        {
          __m128i vsrc1 = _mm_loadu_si128((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_lddqu_si128((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      else
      {
        // Do with step of 4
        __m128i vzero  = _mm_setzero_si128();
        __m128i vsum32 = vzero;
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 4)
        {
          __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
          __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
          vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32           = _mm_add_epi32(vsum32, vsumtemp);
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
        vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
        uiSum            = _mm_cvtsi128_si32(vsum32);
      }
      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    // vertical difference
    int iCols = TMP_TEMPLATE_SIZE;

    for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      tarPatchRow        = tarPatch[iY];
      const short *pSrc1 = (const short *) tarPatchRow;
      const short *pSrc2 = (const short *) refPatchRow;

      // SIMD difference

      // Do with step of 4
      __m128i vzero  = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      __m128i vsum16 = vzero;
      for (int iX = 0; iX < iCols; iX += 4)
      {
        __m128i vsrc1 = _mm_loadl_epi64((const __m128i *) &pSrc1[iX]);
        __m128i vsrc2 = _mm_loadl_epi64((const __m128i *) &pSrc2[iX]);
        vsum16        = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
      vsum32           = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
      uiSum            = _mm_cvtsi128_si32(vsum32);

      diffSum += uiSum;

      if (diffSum > iMax)   // for speeding up
      {
        return diffSum;
      }
      // update location
      refPatchRow += uiStride;
    }
  }
#endif

  return diffSum;
}
#endif 
#endif

#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void dimdBlendingSIMD( Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height )
{
  CHECK( (width % 4) != 0, "width should be multiple of 4" );
  __m128i vw0 = _mm_set1_epi32( w0 );
  __m128i vw1 = _mm_set1_epi32( w1 );
  __m128i vw2 = _mm_set1_epi32( w2 );
  const int shift = 6;

  for( int i = 0; i < height; i++ )
  {
    for( int j = 0; j < width; j += 4 )
    {
      __m128i vdst = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pDst + j) ) );
      __m128i vsrc0 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc0 + j) ) );
      __m128i vsrc1 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc1 + j) ) );

      vdst = _mm_mullo_epi32( vdst, vw0 );
      vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc0, vw1 ) );
      vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc1, vw2 ) );

      vdst = _mm_srai_epi32( vdst, shift );
      vdst = _mm_packs_epi32( vdst, vdst );
      _mm_storel_epi64( (__m128i*)(pDst + j), vdst );
    }
    pDst += strideDst;
    pSrc0 += strideSrc0;
    pSrc1 += strideSrc1;
  }
}
#endif

#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void timdBlendingSIMD( Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height )
{
  CHECK( (width % 4) != 0, "width should be multiple of 4" );
  __m128i vw0 = _mm_set1_epi32( w0 );
  __m128i vw1 = _mm_set1_epi32( w1 );
  const int shift = 6;

  for( int i = 0; i < height; i++ )
  {
    for( int j = 0; j < width; j += 4 )
    {
      __m128i vdst = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pDst + j) ) );
      __m128i vsrc = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc + j) ) );

      vdst = _mm_mullo_epi32( vdst, vw0 );
      vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc, vw1 ) );

      vdst = _mm_srai_epi32( vdst, shift );
      vdst = _mm_packs_epi32( vdst, vdst );
      _mm_storel_epi64( (__m128i*)(pDst + j), vdst );
    }
    pDst += strideDst;
    pSrc += strideSrc;
  }
}
#endif

#if JVET_AC0112_IBC_CIIP && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
void ibcCiipBlendingSIMD( Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height )
{
#if USE_AVX2
  if ((vext >= AVX2) && (width & 0x7) == 0)
  {
    const int offset = 1 << (shift - 1);
    __m256i mw = _mm256_unpacklo_epi16(_mm256_set1_epi16(w0), _mm256_set1_epi16(w1));
    __m256i voffset = _mm256_set1_epi32(offset);
    __m256i msrc0, msrc1, msum0, msum1;

    for (int row = 0; row < height; row++)
    {
      for (int col = 0; col < width; col += 8)
      {
        msrc0 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i*)(&pSrc0[col])));
        msrc1 = _mm256_castsi128_si256(_mm_lddqu_si128((__m128i*)(&pSrc1[col])));
        msum0 = _mm256_unpacklo_epi16(msrc0, msrc1);
        msum1 = _mm256_unpackhi_epi16(msrc0, msrc1);
        msum0 = _mm256_madd_epi16(msum0, mw);
        msum1 = _mm256_madd_epi16(msum1, mw);
        msum0 = _mm256_add_epi32(msum0, voffset);
        msum1 = _mm256_add_epi32(msum1, voffset);
        msum0 = _mm256_srai_epi32(msum0, shift);
        msum1 = _mm256_srai_epi32(msum1, shift);
        msum0 = _mm256_packs_epi32(msum0, msum1);
        _mm_storeu_si128((__m128i *)&pDst[col], _mm256_castsi256_si128(msum0));
      }
      pSrc0 += strideSrc0;
      pSrc1 += strideSrc1;
      pDst += strideDst;
    }
  }
  else
  {
#endif
    __m128i vw0 = _mm_set1_epi32( w0 );
    __m128i vw1 = _mm_set1_epi32( w1 );
    const int offset = 1 << (shift - 1);
    __m128i voffset  = _mm_set1_epi32( offset );

    for( int i = 0; i < height; i++ )
    {
      for( int j = 0; j < width; j += 4 )
      {
        __m128i vdst = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pDst + j) ) );
        __m128i vsrc0 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc0 + j) ) );
        __m128i vsrc1 = _mm_cvtepi16_epi32( _mm_loadl_epi64( (__m128i*)(pSrc1 + j) ) );

        vdst = _mm_mullo_epi32( vsrc0, vw0 );
        vdst = _mm_add_epi32( vdst, _mm_mullo_epi32( vsrc1, vw1 ) );

        vdst = _mm_add_epi32( vdst, voffset );
        vdst = _mm_srai_epi32( vdst, shift );
        vdst = _mm_packs_epi32( vdst, vdst );
        _mm_storel_epi64( (__m128i*)(pDst + j), vdst );
      }
      pDst += strideDst;
      pSrc0 += strideSrc0;
      pSrc1 += strideSrc1;
    }
#if USE_AVX2
  }
#endif
}
#endif

template <X86_VEXT vext>
void IntraPrediction::_initIntraX86()
{
#if ENABLE_SIMD_TMP
  m_calcTemplateDiff = calcTemplateDiffSIMD<vext>;
#endif
#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
  m_dimdBlending = dimdBlendingSIMD<vext>;
#endif
#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
  m_timdBlending = timdBlendingSIMD<vext>;
#endif
#if JVET_AC0112_IBC_CIIP && INTRA_TRANS_ENC_OPT
  m_ibcCiipBlending = ibcCiipBlendingSIMD<vext>;
#endif
}

template void IntraPrediction::_initIntraX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}
