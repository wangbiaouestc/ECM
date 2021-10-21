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
  if( tempType == L_SHAPE_TEMPLATE )
  {
#endif
  for( iY = 0; iY < TMP_TEMPLATE_SIZE; iY++ )
  {
    tarPatchRow = tarPatch[iY];
    const short* pSrc1 = ( const short* ) tarPatchRow;
    const short* pSrc2 = ( const short* ) refPatchRow;

    // SIMD difference
    //int  iRows = uiPatchHeight;
    int  iCols = uiPatchWidth;
    if( (iCols & 7) == 0 )
    {
      // Do with step of 8
      __m128i vzero = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      //for (int iY = 0; iY < iRows; iY += iSubStep)
      {
        __m128i vsum16 = vzero;
        for( int iX = 0; iX < iCols; iX += 8 )
        {
          __m128i vsrc1 = _mm_loadu_si128( (const __m128i*)(&pSrc1[iX]) );
          __m128i vsrc2 = _mm_lddqu_si128( (const __m128i*)(&pSrc2[iX]) );
          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }
        __m128i vsumtemp = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );
        vsum32 = _mm_add_epi32( vsum32, vsumtemp );
        //pSrc1 += iStrideSrc1;
        //pSrc2 += iStrideSrc2;
      }
      vsum32 = _mm_add_epi32( vsum32, _mm_shuffle_epi32( vsum32, 0x4e ) );   // 01001110
      vsum32 = _mm_add_epi32( vsum32, _mm_shuffle_epi32( vsum32, 0xb1 ) );   // 10110001
      uiSum = _mm_cvtsi128_si32( vsum32 );
    }
    else
    {
      // Do with step of 4
      __m128i vzero = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      //for (int iY = 0; iY < iRows; iY += iSubStep)
      {
        __m128i vsum16 = vzero;
        for( int iX = 0; iX < iCols; iX += 4 )
        {
          __m128i vsrc1 = _mm_loadl_epi64( (const __m128i*) & pSrc1[iX] );
          __m128i vsrc2 = _mm_loadl_epi64( (const __m128i*) & pSrc2[iX] );
          vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
        }
        __m128i vsumtemp = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );
        vsum32 = _mm_add_epi32( vsum32, vsumtemp );
        //pSrc1 += iStrideSrc1;
        //pSrc2 += iStrideSrc2;
      }
      vsum32 = _mm_add_epi32( vsum32, _mm_shuffle_epi32( vsum32, 0x4e ) );   // 01001110
      vsum32 = _mm_add_epi32( vsum32, _mm_shuffle_epi32( vsum32, 0xb1 ) );   // 10110001
      uiSum = _mm_cvtsi128_si32( vsum32 );
    }
    diffSum += uiSum;

    if( diffSum > iMax ) //for speeding up
    {
      return diffSum;
    }
    // update location
    refPatchRow += uiStride;
  }

  // vertical difference
  int  iCols = TMP_TEMPLATE_SIZE;

  for( iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++ )
  {
    tarPatchRow = tarPatch[iY];
    const short* pSrc1 = ( const short* ) tarPatchRow;
    const short* pSrc2 = ( const short* ) refPatchRow;

    // SIMD difference

    // Do with step of 4
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    //for (int iY = 0; iY < iRows; iY += iSubStep)
    {
      __m128i vsum16 = vzero;
      for( int iX = 0; iX < iCols; iX += 4 )
      {
        __m128i vsrc1 = _mm_loadl_epi64( (const __m128i*) & pSrc1[iX] );
        __m128i vsrc2 = _mm_loadl_epi64( (const __m128i*) & pSrc2[iX] );
        vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
      }
      __m128i vsumtemp = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );
      vsum32 = _mm_add_epi32( vsum32, vsumtemp );
      //pSrc1 += iStrideSrc1;
      //pSrc2 += iStrideSrc2;
    }
    vsum32 = _mm_add_epi32( vsum32, _mm_shuffle_epi32( vsum32, 0x4e ) );   // 01001110
    vsum32 = _mm_add_epi32( vsum32, _mm_shuffle_epi32( vsum32, 0xb1 ) );   // 10110001
    uiSum = _mm_cvtsi128_si32( vsum32 );

    diffSum += uiSum;

    if( diffSum > iMax ) //for speeding up
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
    tarPatchRow = tarPatch[iY];
    const short* pSrc1 = (const short*)tarPatchRow;
    const short* pSrc2 = (const short*)refPatchRow;

    // SIMD difference
    //int  iRows = uiPatchHeight;
    int  iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
    if ((iCols & 7) == 0)
    {
      // Do with step of 8
      __m128i vzero = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      //for (int iY = 0; iY < iRows; iY += iSubStep)
      {
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 8)
        {
          __m128i vsrc1 = _mm_loadu_si128((const __m128i*)(&pSrc1[iX]));
          __m128i vsrc2 = _mm_lddqu_si128((const __m128i*)(&pSrc2[iX]));
          vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32 = _mm_add_epi32(vsum32, vsumtemp);
        //pSrc1 += iStrideSrc1;
        //pSrc2 += iStrideSrc2;
      }
      vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
      vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
      uiSum = _mm_cvtsi128_si32(vsum32);
    }
    else
    {
      // Do with step of 4
      __m128i vzero = _mm_setzero_si128();
      __m128i vsum32 = vzero;
      //for (int iY = 0; iY < iRows; iY += iSubStep)
      {
        __m128i vsum16 = vzero;
        for (int iX = 0; iX < iCols; iX += 4)
        {
          __m128i vsrc1 = _mm_loadl_epi64((const __m128i*) & pSrc1[iX]);
          __m128i vsrc2 = _mm_loadl_epi64((const __m128i*) & pSrc2[iX]);
          vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
        }
        __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
        vsum32 = _mm_add_epi32(vsum32, vsumtemp);
        //pSrc1 += iStrideSrc1;
        //pSrc2 += iStrideSrc2;
      }
      vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
      vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
      uiSum = _mm_cvtsi128_si32(vsum32);
    }
    diffSum += uiSum;

    if (diffSum > iMax) //for speeding up
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
  int  iCols = TMP_TEMPLATE_SIZE;

  for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
  {
    tarPatchRow = tarPatch[iY];
    const short* pSrc1 = (const short*)tarPatchRow;
    const short* pSrc2 = (const short*)refPatchRow;

    // SIMD difference

    // Do with step of 4
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    //for (int iY = 0; iY < iRows; iY += iSubStep)
    {
      __m128i vsum16 = vzero;
      for (int iX = 0; iX < iCols; iX += 4)
      {
        __m128i vsrc1 = _mm_loadl_epi64((const __m128i*) & pSrc1[iX]);
        __m128i vsrc2 = _mm_loadl_epi64((const __m128i*) & pSrc2[iX]);
        vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32 = _mm_add_epi32(vsum32, vsumtemp);
      //pSrc1 += iStrideSrc1;
      //pSrc2 += iStrideSrc2;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    uiSum = _mm_cvtsi128_si32(vsum32);

    diffSum += uiSum;

    if (diffSum > iMax) //for speeding up
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

template <X86_VEXT vext>
void IntraPrediction::_initIntraX86()
{
#if ENABLE_SIMD_TMP
  m_calcTemplateDiff = calcTemplateDiffSIMD<vext>;
#endif
}

template void IntraPrediction::_initIntraX86<SIMDX86>();
#endif

#endif //#ifdef TARGET_SIMD_X86
//! \}
