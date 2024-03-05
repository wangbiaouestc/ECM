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
#if JVET_AG0136_INTRA_TMP_LIC
template<X86_VEXT vext>
#endif
inline uint32_t calcDiff4(const short* pSrc1, const short* pSrc2, const int start, const int end)
{
#if JVET_AG0136_INTRA_TMP_LIC
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start])));
  const __m128i vzero = _mm_setzero_si128();
#else
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 4)
  {
    __m128i vsrc1 = _mm_loadl_epi64((const __m128i*) & pSrc1[iX]);
    __m128i vsrc2 = _mm_loadl_epi64((const __m128i*) & pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
  }
#endif
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01 00 11 10
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10 11 00 01
  return _mm_cvtsi128_si32(vsum32);
}
// Calculation with step of 8
#if JVET_AG0136_INTRA_TMP_LIC
template<X86_VEXT vext>
#endif
inline uint32_t calcDiff8(const short* pSrc1, const short* pSrc2, const int start, const int end)
{
#if JVET_AG0136_INTRA_TMP_LIC && USE_AVX2
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), _mm_lddqu_si128((const __m128i*) &pSrc2[start])));
  const __m128i vzero = _mm_setzero_si128();
#else
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    __m128i vsrc1 = _mm_loadu_si128((const __m128i*) & pSrc1[iX]);
    __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) & pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(vsrc1, vsrc2)));
  }
#endif
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01 00 11 10
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10 11 00 01
  return _mm_cvtsi128_si32(vsum32);
}
#if JVET_AG0136_INTRA_TMP_LIC
#if USE_AVX2
template<X86_VEXT vext>
inline int calcDiff16(const short* pSrc1, const short* pSrc2, const int start, const int end)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc1 = _mm256_loadu_si256((const __m256i*) &pSrc1[iX]);
    const __m256i vsrc2 = _mm256_lddqu_si256((const __m256i*) &pSrc2[iX]);
    vsum16 = _mm256_add_epi16(vsum16, _mm256_abs_epi16(_mm256_sub_epi16(vsrc1, vsrc2)));
  }
  __m256i vsum32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0xb1));
  return _mm256_extract_epi32(vsum32, 0) + _mm256_extract_epi32(vsum32, 4);
}
#endif

template<X86_VEXT vext>
inline int summation4(const short* pSrc, const int start, const int end)
{
  const __m128i vsum16 = _mm_loadl_epi64((const __m128i*) &pSrc[start]);
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

template<X86_VEXT vext>
inline int summation8(const short* pSrc, const int start, const int end)
{
#if USE_AVX2
  const __m128i vsum16 = _mm_loadu_si128((const __m128i*) &pSrc[start]);
  const __m128i vzero = _mm_setzero_si128();
#else
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    const __m128i vsrc = _mm_loadu_si128((const __m128i*) &pSrc[iX]);
    vsum16 = _mm_add_epi16(vsum16, vsrc);
  }
#endif
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

#if USE_AVX2
template<X86_VEXT vext>
inline int summation16(const short* pSrc, const int start, const int end)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc = _mm256_loadu_si256((const __m256i*) &pSrc[iX]);
    vsum16 = _mm256_add_epi16(vsum16, vsrc);
  }
  __m256i vsum32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0xb1));
  return _mm256_extract_epi32(vsum32, 0) + _mm256_extract_epi32(vsum32, 4);
}
#endif

template<X86_VEXT vext>
inline void calcDiffDelta4Joint(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end, int& sad, int& mrsad)
{
  const __m128i difference = _mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start]));
  const __m128i vsumSad16 = _mm_abs_epi16(difference);
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(difference, delta));
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsumSad32 = _mm_unpacklo_epi16(vsumSad16, vzero);
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0x4e));
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0xb1));
  __m128i vsumMrsad32 = _mm_unpacklo_epi16(vsumMrsad16, vzero);
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0x4e));
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm_cvtsi128_si32(vsumSad32);
  mrsad = _mm_cvtsi128_si32(vsumMrsad32);
}

template<X86_VEXT vext>
inline int calcDiffDelta4(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end)
{
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(_mm_loadl_epi64((const __m128i*) &pSrc1[start]), _mm_loadl_epi64((const __m128i*) &pSrc2[start])), delta));
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = _mm_unpacklo_epi16(vsum16, vzero);
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

template<X86_VEXT vext>
inline void calcDiffDelta8Joint(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end, int& sad, int& mrsad)
{
#if USE_AVX2
  const __m128i difference = _mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), _mm_lddqu_si128((const __m128i*) &pSrc2[start]));
  const __m128i vsumSad16 = _mm_abs_epi16(difference);
  const __m128i vsumMrsad16 = _mm_abs_epi16(_mm_sub_epi16(difference, delta));
  const __m128i vzero = _mm_setzero_si128();
#else
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsumSad16 = vzero;
  __m128i vsumMrsad16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    const __m128i vsrc1 = _mm_loadu_si128((const __m128i*) &pSrc1[iX]);
    const __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) &pSrc2[iX]);
    const __m128i difference = _mm_sub_epi16(vsrc1, vsrc2);
    vsumSad16 = _mm_add_epi16(vsumSad16, _mm_abs_epi16(difference));
    vsumMrsad16 = _mm_add_epi16(vsumMrsad16, _mm_abs_epi16(_mm_sub_epi16(difference, delta)));
  }
#endif
  __m128i vsumSad32 = _mm_add_epi32(_mm_unpacklo_epi16(vsumSad16, vzero), _mm_unpackhi_epi16(vsumSad16, vzero));
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0x4e));
  vsumSad32 = _mm_add_epi32(vsumSad32, _mm_shuffle_epi32(vsumSad32, 0xb1));
  __m128i vsumMrsad32 = _mm_add_epi32(_mm_unpacklo_epi16(vsumMrsad16, vzero), _mm_unpackhi_epi16(vsumMrsad16, vzero));
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0x4e));
  vsumMrsad32 = _mm_add_epi32(vsumMrsad32, _mm_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm_cvtsi128_si32(vsumSad32);
  mrsad = _mm_cvtsi128_si32(vsumMrsad32);
}

template<X86_VEXT vext>
inline int calcDiffDelta8(const short* const pSrc1, const short* const pSrc2, const __m128i delta, const int start, const int end)
{
#if USE_AVX2
  const __m128i vsum16 = _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(_mm_loadu_si128((const __m128i*) &pSrc1[start]), _mm_lddqu_si128((const __m128i*) &pSrc2[start])), delta));
  const __m128i vzero = _mm_setzero_si128();
#else
  const __m128i vzero = _mm_setzero_si128();
  __m128i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 8)
  {
    const __m128i vsrc1 = _mm_loadu_si128((const __m128i*) &pSrc1[iX]);
    const __m128i vsrc2 = _mm_lddqu_si128((const __m128i*) &pSrc2[iX]);
    vsum16 = _mm_add_epi16(vsum16, _mm_abs_epi16(_mm_sub_epi16(_mm_sub_epi16(vsrc1, vsrc2), delta)));
  }
#endif
  __m128i vsum32 = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));
  return _mm_cvtsi128_si32(vsum32);
}

#if USE_AVX2
template<X86_VEXT vext>
inline void calcDiffDelta16Joint(const short* pSrc1, const short* pSrc2, const __m256i delta, const int start, const int end, int& sad, int& mrsad)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsumSad16 = vzero;
  __m256i vsumMrsad16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc1 = _mm256_loadu_si256((const __m256i*) &pSrc1[iX]);
    const __m256i vsrc2 = _mm256_lddqu_si256((const __m256i*) &pSrc2[iX]);
    const __m256i difference = _mm256_sub_epi16(vsrc1, vsrc2);
    vsumSad16 = _mm256_add_epi16(vsumSad16, _mm256_abs_epi16(difference));
    vsumMrsad16 = _mm256_add_epi16(vsumMrsad16, _mm256_abs_epi16(_mm256_sub_epi16(difference, delta)));
  }
  __m256i vsumSad32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsumSad16, vzero), _mm256_unpackhi_epi16(vsumSad16, vzero));
  vsumSad32 = _mm256_add_epi32(vsumSad32, _mm256_shuffle_epi32(vsumSad32, 0x4e));
  vsumSad32 = _mm256_add_epi32(vsumSad32, _mm256_shuffle_epi32(vsumSad32, 0xb1));
  __m256i vsumMrsad32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsumMrsad16, vzero), _mm256_unpackhi_epi16(vsumMrsad16, vzero));
  vsumMrsad32 = _mm256_add_epi32(vsumMrsad32, _mm256_shuffle_epi32(vsumMrsad32, 0x4e));
  vsumMrsad32 = _mm256_add_epi32(vsumMrsad32, _mm256_shuffle_epi32(vsumMrsad32, 0xb1));
  sad = _mm256_extract_epi32(vsumSad32, 0) + _mm256_extract_epi32(vsumSad32, 4);
  mrsad = _mm256_extract_epi32(vsumMrsad32, 0) + _mm256_extract_epi32(vsumMrsad32, 4);
}

template<X86_VEXT vext>
inline int calcDiffDelta16(const short* pSrc1, const short* pSrc2, const __m256i delta, const int start, const int end)
{
  const __m256i vzero = _mm256_setzero_si256();
  __m256i vsum16 = vzero;
  for (int iX = start; iX < end; iX += 16)
  {
    const __m256i vsrc1 = _mm256_loadu_si256((const __m256i*) &pSrc1[iX]);
    const __m256i vsrc2 = _mm256_lddqu_si256((const __m256i*) &pSrc2[iX]);
    vsum16 = _mm256_add_epi16(vsum16, _mm256_abs_epi16(_mm256_sub_epi16(_mm256_sub_epi16(vsrc1, vsrc2), delta)));
  }
  __m256i vsum32 = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0x4e));
  vsum32 = _mm256_add_epi32(vsum32, _mm256_shuffle_epi32(vsum32, 0xb1));
  return _mm256_extract_epi32(vsum32, 0) + _mm256_extract_epi32(vsum32, 4);
}
#endif

template<X86_VEXT vext>
void calcTargetMeanSIMD(Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, const RefTemplateType tempType, const int requiredTemplate, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, int& topTargetMean, int& leftTargetMean)
{
  topTargetMean = 0;
  leftTargetMean = 0;
  if (tempType == L_SHAPE_TEMPLATE)
  {
    if (requiredTemplate == 3 || requiredTemplate == 0 || requiredTemplate == 1)
    {
#if  USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
        {
          topTargetMean += summation16<vext>((const short*) tarPatch[iY], TMP_TEMPLATE_SIZE, uiPatchWidth);
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
        {
          topTargetMean += summation8<vext>((const short*) tarPatch[iY], TMP_TEMPLATE_SIZE, uiPatchWidth);
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
        {
          topTargetMean += summation4<vext>((const short*) tarPatch[iY], TMP_TEMPLATE_SIZE, uiPatchWidth);
        }
      }
      topTargetMean >>= log2SizeTop;
    }
    if (requiredTemplate == 3 || requiredTemplate == 0 || requiredTemplate == 2)
    {
      for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        leftTargetMean += summation4<vext>((const short*) tarPatch[iY], 0, TMP_TEMPLATE_SIZE);
      }
      leftTargetMean >>= log2SizeLeft;
    }
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        topTargetMean += summation16<vext>((const short*) tarPatch[iY], 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        topTargetMean += summation8<vext>((const short*) tarPatch[iY], 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        topTargetMean += summation4<vext>((const short*) tarPatch[iY], 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    topTargetMean >>= log2SizeTop;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      leftTargetMean += summation4<vext>((const short*) tarPatch[iY], 0, TMP_TEMPLATE_SIZE);
    }
    leftTargetMean >>= log2SizeLeft;
  }
}

template<X86_VEXT vext>
inline __m128i calcMeanRefLeftSIMD(const Pel* const ref, const unsigned int uiPatchHeight, const unsigned int uiStride, const int log2SizeLeft, const int leftTargetMean)
{
  int leftMeanRef = 0;
  const Pel* refPatchRowTemp = ref - TMP_TEMPLATE_SIZE;
  for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRowTemp += uiStride)
  {
    leftMeanRef += summation4<vext>((const short*) refPatchRowTemp, 0, TMP_TEMPLATE_SIZE);
  }
  leftMeanRef >>= log2SizeLeft;
  return _mm_set1_epi16(leftMeanRef - leftTargetMean);
}

template<X86_VEXT vext>
void calcTemplateDiffJointSadMrsadSIMD(const Pel* const ref, const unsigned int uiStride, Pel** tarPatch, const unsigned int uiPatchWidth, const unsigned int uiPatchHeight, int* diffSad, int* diffMrsad, int* iMaxSad, int* iMaxMrsad, const RefTemplateType tempType, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean)
{
  int diffSumSad = 0;
  int diffSumMrsad = 0;
  int topDiffSad = MAX_INT;
  int topDiffMrsad = MAX_INT;
  int leftDiffSad = MAX_INT;
  int leftDiffMrsad = MAX_INT;
  const Pel* refPatchRow = tempType == L_SHAPE_TEMPLATE ? ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE : (tempType == ABOVE_TEMPLATE ? ref - TMP_TEMPLATE_SIZE * uiStride : ref - TMP_TEMPLATE_SIZE);
  int topMeanRef = 0;
  __m128i topMeanDelta = _mm_setzero_si128();
#if USE_AVX2
  __m256i topMeanDelta256 = _mm256_setzero_si256();
#endif
  if (tempType == L_SHAPE_TEMPLATE)
  {
    const Pel* refPatchRowTemp = refPatchRow;
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation16<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation8<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation4<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
      }
    }
    topMeanRef >>= log2SizeTop;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    const Pel* refPatchRowTemp = refPatchRow;
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation16<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation8<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        topMeanRef += summation4<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
      }
    }
    topMeanRef >>= log2SizeTop;
  }
  if (tempType == L_SHAPE_TEMPLATE || tempType == ABOVE_TEMPLATE)
  {
    topMeanDelta = _mm_set1_epi16(topMeanRef - topTargetMean);
#if USE_AVX2
    topMeanDelta256 = _mm256_set1_epi16(topMeanRef - topTargetMean);
#endif
  }
  int iSumSad = 0;
  int iSumMrsad = 0;
  if (tempType == L_SHAPE_TEMPLATE)
  {
    topDiffSad = 0;
    topDiffMrsad = 0;
    leftDiffSad = 0;
    leftDiffMrsad = 0;
#if USE_AVX2
    if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
    {
      const Pel* refPatchRowTemp = refPatchRow;
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        const short* const pSrc1 = (const short*) refPatchRowTemp;
        const short* const pSrc2 = (const short*) tarPatch[iY];
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        calcDiffDelta16Joint<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && topDiffSad > iMaxSad[1] && diffSumMrsad > iMaxMrsad[0] && topDiffMrsad > iMaxMrsad[1])
        {
          break;
        }
      }
    }
    else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
    if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
    {
      const Pel* refPatchRowTemp = refPatchRow;
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        const short* const pSrc1 = (const short*) refPatchRowTemp;
        const short* const pSrc2 = (const short*) tarPatch[iY];
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        calcDiffDelta8Joint<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && topDiffSad > iMaxSad[1] && diffSumMrsad > iMaxMrsad[0] && topDiffMrsad > iMaxMrsad[1])
        {
          break;
        }
      }
    }
    else if (uiPatchWidth == 8)
    {
      const Pel* refPatchRowTemp = refPatchRow;
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
      {
        const short* const pSrc1 = (const short*) refPatchRowTemp;
        const short* const pSrc2 = (const short*) tarPatch[iY];
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        calcDiffDelta4Joint<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        topDiffSad += iSumSad;
        topDiffMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && topDiffSad > iMaxSad[1] && diffSumMrsad > iMaxMrsad[0] && topDiffMrsad > iMaxMrsad[1])
        {
          break;
        }
      }
    }
    const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
    {
      calcDiffDelta4Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
      diffSumSad += iSumSad;
      diffSumMrsad += iSumMrsad;
      leftDiffSad += iSumSad;
      leftDiffMrsad += iSumMrsad;
      if (diffSumSad > iMaxSad[0] && leftDiffSad > iMaxSad[2] && diffSumMrsad > iMaxMrsad[0] && leftDiffMrsad > iMaxMrsad[2])
      {
        break;
      }
    }
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    const int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
#if USE_AVX2
    if (vext >= AVX2 && (iCols & 15) == 0)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
        calcDiffDelta16Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, 0, iCols, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
        {
          break;
        }
      }
    }
    else if ((iCols & 7) == 0)
#else
    if ((iCols & 7) == 0)
#endif
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
        calcDiffDelta8Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
        {
          break;
        }
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
        calcDiffDelta4Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols, iSumSad, iSumMrsad);
        diffSumSad += iSumSad;
        diffSumMrsad += iSumMrsad;
        if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
        {
          break;
        }
      }
    }
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
    {
      calcDiffDelta4Joint<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE, iSumSad, iSumMrsad);
      diffSumSad += iSumSad;
      diffSumMrsad += iSumMrsad;
      if (diffSumSad > iMaxSad[0] && diffSumMrsad > iMaxMrsad[0])
      {
        break;
      }
    }
  }
  diffSad[0] = diffSumSad;
  diffSad[1] = topDiffSad;
  diffSad[2] = leftDiffSad;
  diffMrsad[0] = diffSumMrsad;
  diffMrsad[1] = topDiffMrsad;
  diffMrsad[2] = leftDiffMrsad;
}
#endif
template<X86_VEXT vext>
void calcTemplateDiffSIMD(Pel *ref, unsigned int uiStride, Pel **tarPatch, unsigned int uiPatchWidth,
                          unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType tempType,
                          int requiredTemplate
#if JVET_AG0136_INTRA_TMP_LIC
                          , const bool isMrSad, const int log2SizeTop, const int log2SizeLeft, const int sizeTopLeft, const int topTargetMean, const int leftTargetMean
#endif
                          )
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
#if JVET_AG0136_INTRA_TMP_LIC
  if (isMrSad)
  {
    __m128i topMeanDelta = _mm_setzero_si128();
#if USE_AVX2
    __m256i topMeanDelta256 = _mm256_setzero_si256();
#endif
    int topMeanRef = 0;
    if (tempType == L_SHAPE_TEMPLATE)
    {
      if (requiredTemplate == 3 || requiredTemplate == 0 || requiredTemplate == 1)
      {
        const Pel* refPatchRowTemp = refPatchRow;
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            topMeanRef += summation16<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            topMeanRef += summation8<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
          }
        }
        else if (uiPatchWidth == 8)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            topMeanRef += summation4<vext>((const short*) refPatchRowTemp, TMP_TEMPLATE_SIZE, uiPatchWidth);
          }
        }
        topMeanRef >>= log2SizeTop;
      }
    }
    else if (tempType == ABOVE_TEMPLATE)
    {
      const Pel* refPatchRowTemp = refPatchRow;
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
        {
          topMeanRef += summation16<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
        {
          topMeanRef += summation8<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
        {
          topMeanRef += summation4<vext>((const short*) refPatchRowTemp, 0, uiPatchWidth - TMP_TEMPLATE_SIZE);
        }
      }
      topMeanRef >>= log2SizeTop;
    }
    if ((tempType == L_SHAPE_TEMPLATE && requiredTemplate != 2) || tempType == ABOVE_TEMPLATE)
    {
      topMeanDelta = _mm_set1_epi16(topMeanRef - topTargetMean);
#if USE_AVX2
      topMeanDelta256 = _mm256_set1_epi16(topMeanRef - topTargetMean);
#endif
    }
    if (tempType == L_SHAPE_TEMPLATE)
    {
      topDiff = 0;
      leftDiff = 0;
      if (requiredTemplate == 3)
      {
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          const Pel* refPatchRowTemp = refPatchRow;
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRowTemp;
            const short* const pSrc2 = (const short*) tarPatch[iY];
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            uiSum = calcDiffDelta16<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
            diffSum += uiSum;
            topDiff += uiSum;
            if (diffSum > iMax[0] && topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          const Pel* refPatchRowTemp = refPatchRow;
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRowTemp;
            const short* const pSrc2 = (const short*) tarPatch[iY];
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            uiSum = calcDiffDelta8<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            diffSum += uiSum;
            topDiff += uiSum;
            if (diffSum > iMax[0] && topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (uiPatchWidth == 8)
        {
          const Pel* refPatchRowTemp = refPatchRow;
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRowTemp += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRowTemp;
            const short* const pSrc2 = (const short*) tarPatch[iY];
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            uiSum = calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            diffSum += uiSum;
            topDiff += uiSum;
            if (diffSum > iMax[0] && topDiff > iMax[1])
            {
              break;
            }
          }
        }
        const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
        refPatchRow = ref - TMP_TEMPLATE_SIZE;
        for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
        {
          uiSum = calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
          diffSum += uiSum;
          leftDiff += uiSum;
          if (diffSum > iMax[0] && leftDiff > iMax[2])
          {
            break;
          }
        }
      }
      else if (requiredTemplate == 0)
      {
        bool isAboveLeftSkipped = false;
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRow;
            const short* const pSrc2 = (const short*) tarPatch[iY];
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            diffSum += calcDiffDelta16<vext>(pSrc1, pSrc2, topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (diffSum > iMax[0])
            {
              isAboveLeftSkipped = true;
              break;
            }
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            const short* const pSrc1 = (const short*) refPatchRow;
            const short* const pSrc2 = (const short*) tarPatch[iY];
            diffSum += calcDiffDelta4<vext>(pSrc1, pSrc2, topMeanDelta, 0, TMP_TEMPLATE_SIZE);
            diffSum += calcDiffDelta8<vext>(pSrc1, pSrc2, topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (diffSum > iMax[0])
            {
              isAboveLeftSkipped = true;
              break;
            }
          }
        }
        else if (uiPatchWidth == 8)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            diffSum += calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, uiPatchWidth);
            if (diffSum > iMax[0])
            {
              isAboveLeftSkipped = true;
              break;
            }
          }
        }
        if (!isAboveLeftSkipped)
        {
          const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
          refPatchRow = ref - TMP_TEMPLATE_SIZE;
          for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
          {
            diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
            if (diffSum > iMax[0])
            {
              break;
            }
          }
        }
      }
      else if (requiredTemplate == 1)
      {
#if USE_AVX2
        if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            topDiff += calcDiffDelta16<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
        if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            topDiff += calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (topDiff > iMax[1])
            {
              break;
            }
          }
        }
        else if (uiPatchWidth == 8)
        {
          for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
          {
            topDiff += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, TMP_TEMPLATE_SIZE, uiPatchWidth);
            if (topDiff > iMax[1])
            {
              break;
            }
          }
        }
      }
      else
      {
        const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
        refPatchRow = ref - TMP_TEMPLATE_SIZE;
        for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
        {
          leftDiff += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
          if (leftDiff > iMax[2])
          {
            break;
          }
        }
      }
    }
    else if (tempType == ABOVE_TEMPLATE)
    {
      const int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
#if USE_AVX2
      if (vext >= AVX2 && (iCols & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          diffSum += calcDiffDelta16<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta256, 0, iCols);
          if (diffSum > iMax[0])
          {
            break;
          }
        }
      }
      else if ((iCols & 7) == 0)
#else
      if ((iCols & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          diffSum += calcDiffDelta8<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols);
          if (diffSum > iMax[0])
          {
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], topMeanDelta, 0, iCols);
          if (diffSum > iMax[0])
          {
            break;
          }
        }
      }
    }
    else if (tempType == LEFT_TEMPLATE)
    {
      const __m128i leftMeanDelta = calcMeanRefLeftSIMD<vext>(ref, uiPatchHeight, uiStride, log2SizeLeft, leftTargetMean);
      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++, refPatchRow += uiStride)
      {
        diffSum += calcDiffDelta4<vext>((const short*) refPatchRow, (const short*) tarPatch[iY], leftMeanDelta, 0, TMP_TEMPLATE_SIZE);
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
  }
  else
  {
#endif

  // horizontal difference
#if JVET_W0069_TMP_BOUNDARY
  if (tempType == L_SHAPE_TEMPLATE)
  {
#endif
    topDiff  = 0;
    leftDiff = 0;
    if (requiredTemplate == 3)
    {
#if JVET_AG0136_INTRA_TMP_LIC 
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          const short* pSrc1 = (const short*) tarPatch[iY];
          const short* pSrc2 = (const short*) refPatchRow;
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          uiSum = calcDiff16<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          diffSum += uiSum;
          topDiff += uiSum;
          if (diffSum > iMax[0] && topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          const short* pSrc1 = (const short*) tarPatch[iY];
          const short* pSrc2 = (const short*) refPatchRow;
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          uiSum = calcDiff8<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          diffSum += uiSum;
          topDiff += uiSum;
          if (diffSum > iMax[0] && topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          const short* pSrc1 = (const short*) tarPatch[iY];
          const short* pSrc2 = (const short*) refPatchRow;
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          uiSum = calcDiff4<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          diffSum += uiSum;
          topDiff += uiSum;
          if (diffSum > iMax[0] && topDiff > iMax[1])
          {
            break;
          }
        }
      }
#else
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
#endif
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
        uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                         <vext>
#endif
                         (pSrc1, pSrc2, 0, iCols);

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
#if JVET_AG0136_INTRA_TMP_LIC 
      bool isAboveLeftSkipped = false;
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          const short* pSrc1 = (const short*) tarPatch[iY];
          const short* pSrc2 = (const short*) refPatchRow;
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          diffSum += calcDiff16<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (diffSum > iMax[0])
          {
            isAboveLeftSkipped = true;
            break;
          }
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          const short* pSrc1 = (const short*) tarPatch[iY];
          const short* pSrc2 = (const short*) refPatchRow;
          diffSum += calcDiff4<vext>(pSrc1, pSrc2, 0, TMP_TEMPLATE_SIZE);
          diffSum += calcDiff8<vext>(pSrc1, pSrc2, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (diffSum > iMax[0])
          {
            isAboveLeftSkipped = true;
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          diffSum += calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, uiPatchWidth);
          if (diffSum > iMax[0])
          {
            isAboveLeftSkipped = true;
            break;
          }
        }
      }
      if (!isAboveLeftSkipped)
      {
#else
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
#endif

      // vertical difference
      int iCols = TMP_TEMPLATE_SIZE;

      for (iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow        = tarPatch[iY];
        const short *pSrc1 = (const short *) tarPatchRow;
        const short *pSrc2 = (const short *) refPatchRow;

        // SIMD difference
        uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                         <vext>
#endif
                         (pSrc1, pSrc2, 0, iCols);

        diffSum += uiSum;

        if (diffSum > iMax[0])
        {
          break;
        }
        // update location
        refPatchRow += uiStride;
      }
#if JVET_AG0136_INTRA_TMP_LIC 
      }
#endif
    }
    else if (requiredTemplate == 1)
    {
#if JVET_AG0136_INTRA_TMP_LIC 
#if USE_AVX2
      if (vext >= AVX2 && ((uiPatchWidth - TMP_TEMPLATE_SIZE) & 15) == 0)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          topDiff += calcDiff16<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#else
      if (((uiPatchWidth - TMP_TEMPLATE_SIZE) & 7) == 0)
#endif
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          topDiff += calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (topDiff > iMax[1])
          {
            break;
          }
        }
      }
      else if (uiPatchWidth == 8)
      {
        for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
        {
          topDiff += calcDiff4<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, TMP_TEMPLATE_SIZE, uiPatchWidth);
          if (topDiff > iMax[1])
          {
            break;
          }
        }
      }
#else
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
#endif
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

        uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                         <vext>
#endif
                         (pSrc1, pSrc2, 0, iCols);

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
#if JVET_AG0136_INTRA_TMP_LIC
    const int iCols = uiPatchWidth - TMP_TEMPLATE_SIZE;
#if USE_AVX2
    if (vext >= AVX2 && (iCols & 15) == 0)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
        diffSum += calcDiff16<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
    else if ((iCols & 7) == 0)
#else
    if ((iCols & 7) == 0)
#endif
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
        diffSum += calcDiff8<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
    else if (uiPatchWidth == 8)
    {
      for (iY = 0; iY < TMP_TEMPLATE_SIZE; iY++, refPatchRow += uiStride)
      {
        diffSum += calcDiff4<vext>((const short*) tarPatch[iY], (const short*) refPatchRow, 0, iCols);
        if (diffSum > iMax[0])
        {
          break;
        }
      }
    }
#else
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
#endif
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

      uiSum = calcDiff4
#if JVET_AG0136_INTRA_TMP_LIC
                        <vext>
#endif
                        (pSrc1, pSrc2, 0, iCols);

      diffSum += uiSum;

      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }

      refPatchRow += uiStride;
    }
  }
#endif

#if JVET_AG0136_INTRA_TMP_LIC
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

#if JVET_AG0058_EIP && INTRA_TRANS_ENC_OPT
template< X86_VEXT vext >
int64_t calcAeipGroupSumSIMD(const Pel* src1, const Pel* src2, const int numSamples)
{
  int i = 0;
#if USE_AVX2
  __m256i vzero = _mm256_setzero_si256();
  __m256i vsum32 = vzero;
  const int samplesBySIMD = (numSamples >> 4) << 4;
  for (; i < samplesBySIMD; i += 16)
  {
    __m256i vsrc1 = _mm256_lddqu_si256((__m256i*)(&src1[i]));
    __m256i vsrc2 = _mm256_lddqu_si256((__m256i*)(&src2[i]));
    __m256i vsumtemp = _mm256_madd_epi16(vsrc1, vsrc2);
    vsum32 = _mm256_add_epi32(vsum32, vsumtemp);
  }
  vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
  vsum32 = _mm256_hadd_epi32(vsum32, vsum32);
  int64_t sum = (_mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32)) + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11))));
#else
  __m128i vzero = _mm_setzero_si128();
  __m128i vsum32 = vzero;
  const int samplesBySIMD = (numSamples >> 4) << 4;
  for (; i < samplesBySIMD; i += 8)
  {
    __m128i vsrc1 = _mm_loadu_si128((__m128i*)(&src1[i]));
    __m128i vsrc2 = _mm_loadu_si128((__m128i*)(&src2[i]));
    __m128i vsumtemp = _mm_madd_epi16(vsrc1, vsrc2);
    vsum32 = _mm_add_epi32(vsum32, vsumtemp);
  }
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
  vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
  int64_t sum = _mm_cvtsi128_si32(vsum32);
#endif
  for (; i < numSamples; i++)
  {
    sum += src1[i] * src2[i];
  }

  return sum;
}
#endif

template <X86_VEXT vext>
void IntraPrediction::_initIntraX86()
{
#if ENABLE_SIMD_TMP
  m_calcTemplateDiff = calcTemplateDiffSIMD<vext>;
#if JVET_AG0136_INTRA_TMP_LIC
  m_calcTemplateDiffJointSadMrsad = calcTemplateDiffJointSadMrsadSIMD<vext>;
  m_calcTargetMean = calcTargetMeanSIMD<vext>;
#endif
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
#if JVET_AG0058_EIP && INTRA_TRANS_ENC_OPT
  m_calcAeipGroupSum = calcAeipGroupSumSIMD<vext>;
#endif
}

template void IntraPrediction::_initIntraX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}
