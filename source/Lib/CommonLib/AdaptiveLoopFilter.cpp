/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
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

/** \file     AdaptiveLoopFilter.cpp
    \brief    adaptive loop filter class
*/

#include "AdaptiveLoopFilter.h"
#if ALF_IMPROVEMENT
#include "AlfFixedFilters.h"
#endif
#include "CodingStructure.h"
#include "Picture.h"
#include <array>
#include <cmath>

constexpr int AdaptiveLoopFilter::AlfNumClippingValues[];

AdaptiveLoopFilter::AdaptiveLoopFilter()
#if !JVET_X0071_ALF_BAND_CLASSIFIER
  : m_classifier( nullptr )
#endif
{
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  for( int i = 0; i < ALF_NUM_CLASSIFIER + 1; i++ )
#else
  for( int i = 0; i < ALF_NUM_CLASSIFIER; i++ )
#endif
  {
    m_classifier[i] = nullptr;
  }
#endif
  for (size_t i = 0; i < NUM_DIRECTIONS; i++)
  {
    m_laplacian[i] = m_laplacianPtr[i];
    for (size_t j = 0; j < sizeof(m_laplacianPtr[i]) / sizeof(m_laplacianPtr[i][0]); j++)
    {
      m_laplacianPtr[i][j] = m_laplacianData[i][j];
    }
  }

#if ALF_IMPROVEMENT
  int ind = 0;
  memset(m_mappingDir, 0, sizeof(m_mappingDir));
  for (int i = 0; i < NUM_DIR_FIX; i++)
  {
    for (int j = 0; j <= i; j++)
    {
      m_mappingDir[i][j] = ind;
      ind++;
    }
  }
#endif

  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = nullptr;
    m_ctuAlternative[compIdx] = nullptr;
  }
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  m_ctuEnableOnlineLumaFlag = nullptr;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  m_gaussFiltering = gaussFiltering;
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_ctuPadFlag = nullptr;
  m_fixFilter13x13Db9Blk = fixedFilterBlk<ALF_FIXED_FILTER_13_DB_9>;
  m_fixFilter9x9Db9Blk = fixedFilterBlk<ALF_FIXED_FILTER_9_DB_9>;
#endif
  m_filterCcAlf = filterBlkCcAlf<CC_ALF>;
  m_filter5x5Blk = filterBlk<ALF_FILTER_5>;
  m_filter7x7Blk = filterBlk<ALF_FILTER_7>;
#if ALF_IMPROVEMENT
  m_filter9x9Blk = filterBlk<ALF_FILTER_9>;
  m_filter9x9BlkExt = filterBlk<ALF_FILTER_9_EXT>;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
  m_filter13x13BlkExtDb = filterBlk<ALF_FILTER_13_EXT_DB>;
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  m_filter9x9BlkExtDb = filterBlk<ALF_FILTER_9_EXT_DB>;
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
  m_filter13x13BlkExt = filterBlk<ALF_FILTER_13_EXT>;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_filter13x13BlkExtDbResiDirect = filterBlk<ALF_FILTER_13_EXT_DB_RESI_DIRECT>;
  m_filter13x13BlkExtDbResi       = filterBlk<ALF_FILTER_13_EXT_DB_RESI>;
#endif
  m_filterBlkExt = filterBlk<ALF_FILTER_EXT>;
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_filter13x13Blk = fixedFiltering;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_filterResi9x9Blk = fixedFilteringResi;
#else
  m_filterResi13x13Blk = fixedFilteringResi;
#endif
#endif
  m_deriveClassificationLaplacian = deriveClassificationLaplacian;
  m_deriveClassificationLaplacianBig = deriveClassificationLaplacianBig;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_deriveVariance = deriveVariance;
  m_calcClass0 = calcClass0Var;
#else
  m_calcClass0 = calcClass;
#endif
  m_calcClass1 = calcClass;
#if JVET_X0071_ALF_BAND_CLASSIFIER
  m_calcClass2 = calcClassNew;
#endif
  m_fixFilterResult = nullptr;
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_fixFilterResiResult = nullptr;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  m_fixedFilterResultPerCtu = nullptr;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  m_gaussPic = nullptr;
  m_gaussCtu = nullptr;
#endif
#else
  m_deriveClassificationBlk = deriveClassificationBlk;
#endif

#if ENABLE_SIMD_OPT_ALF
#ifdef TARGET_SIMD_X86
  initAdaptiveLoopFilterX86();
#endif
#endif
}

bool AdaptiveLoopFilter::isCrossedByVirtualBoundaries( const CodingStructure& cs, const int xPos, const int yPos, const int width, const int height, bool& clipTop, bool& clipBottom, bool& clipLeft, bool& clipRight, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], int& rasterSliceAlfPad )
{
  clipTop = false; clipBottom = false; clipLeft = false; clipRight = false;
  numHorVirBndry = 0; numVerVirBndry = 0;
  const PPS*   pps = cs.pps;
  const PicHeader* picHeader = cs.picHeader;

  if( picHeader->getVirtualBoundariesPresentFlag() )
  {
    for( int i = 0; i < picHeader->getNumHorVirtualBoundaries(); i++ )
    {
      if( picHeader->getVirtualBoundariesPosY(i) == yPos )
      {
        clipTop = true;
      }
      else if( picHeader->getVirtualBoundariesPosY(i) == yPos + height )
      {
        clipBottom = true;
      }
      else if( yPos < picHeader->getVirtualBoundariesPosY(i) && picHeader->getVirtualBoundariesPosY(i) < yPos + height )
      {
        horVirBndryPos[numHorVirBndry++] = picHeader->getVirtualBoundariesPosY(i);
      }
    }
    for( int i = 0; i < picHeader->getNumVerVirtualBoundaries(); i++ )
    {
      if( picHeader->getVirtualBoundariesPosX(i) == xPos )
      {
        clipLeft = true;
      }
      else if( picHeader->getVirtualBoundariesPosX(i) == xPos + width )
      {
        clipRight = true;
      }
      else if( xPos < picHeader->getVirtualBoundariesPosX(i) && picHeader->getVirtualBoundariesPosX(i) < xPos + width )
      {
        verVirBndryPos[numVerVirBndry++] = picHeader->getVirtualBoundariesPosX(i);
      }
    }
  }

  const Slice& slice = *(cs.slice);
  int   ctuSize = slice.getSPS()->getCTUSize();
  const Position currCtuPos(xPos, yPos);
  const CodingUnit *currCtu = cs.getCU(currCtuPos, CHANNEL_TYPE_LUMA);
  const SubPic& curSubPic = slice.getPPS()->getSubPicFromPos(currCtuPos);
  bool loopFilterAcrossSubPicEnabledFlag = curSubPic.getloopFilterAcrossEnabledFlag();
  //top
  if (yPos >= ctuSize && clipTop == false)
  {
    const Position prevCtuPos(xPos, yPos - ctuSize);
    const CodingUnit *prevCtu = cs.getCU(prevCtuPos, CHANNEL_TYPE_LUMA);
    if ((!pps->getLoopFilterAcrossSlicesEnabledFlag() && !CU::isSameSlice(*currCtu, *prevCtu)) ||
        (!pps->getLoopFilterAcrossTilesEnabledFlag()  && !CU::isSameTile(*currCtu,  *prevCtu))
      || (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *prevCtu))
      )
    {
      clipTop = true;
    }
  }

  //bottom
  if (yPos + ctuSize < cs.pcv->lumaHeight && clipBottom == false)
  {
    const Position nextCtuPos(xPos, yPos + ctuSize);
    const CodingUnit *nextCtu = cs.getCU(nextCtuPos, CHANNEL_TYPE_LUMA);
    if ((!pps->getLoopFilterAcrossSlicesEnabledFlag() && !CU::isSameSlice(*currCtu, *nextCtu)) ||
        (!pps->getLoopFilterAcrossTilesEnabledFlag()  && !CU::isSameTile(*currCtu,  *nextCtu))
      || (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *nextCtu))
      )
    {
      clipBottom = true;
    }
  }

  //left
  if (xPos >= ctuSize && clipLeft == false)
  {
    const Position prevCtuPos(xPos - ctuSize, yPos);
    const CodingUnit *prevCtu = cs.getCU(prevCtuPos, CHANNEL_TYPE_LUMA);
    if ((!pps->getLoopFilterAcrossSlicesEnabledFlag() && !CU::isSameSlice(*currCtu, *prevCtu)) ||
        (!pps->getLoopFilterAcrossTilesEnabledFlag()  && !CU::isSameTile(*currCtu,  *prevCtu))
      || (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *prevCtu))
      )
    {
      clipLeft = true;
    }
  }

  //right
  if (xPos + ctuSize < cs.pcv->lumaWidth && clipRight == false)
  {
    const Position nextCtuPos(xPos + ctuSize, yPos);
    const CodingUnit *nextCtu = cs.getCU(nextCtuPos, CHANNEL_TYPE_LUMA);
    if ((!pps->getLoopFilterAcrossSlicesEnabledFlag() && !CU::isSameSlice(*currCtu, *nextCtu)) ||
        (!pps->getLoopFilterAcrossTilesEnabledFlag()  && !CU::isSameTile(*currCtu,  *nextCtu))
      || (!loopFilterAcrossSubPicEnabledFlag && !CU::isSameSubPic(*currCtu, *nextCtu))
      )
    {
      clipRight = true;
    }
  }

  rasterSliceAlfPad = 0;
  if ( !clipTop && !clipLeft )
  {
    //top-left CTU
    if ( xPos >= ctuSize && yPos >= ctuSize )
    {
      const Position prevCtuPos( xPos - ctuSize, yPos - ctuSize );
      const CodingUnit *prevCtu = cs.getCU( prevCtuPos, CHANNEL_TYPE_LUMA );
      if ( !pps->getLoopFilterAcrossSlicesEnabledFlag() && !CU::isSameSlice( *currCtu, *prevCtu ) )
      {
        rasterSliceAlfPad = 1;
      }
    }
  }

  if ( !clipBottom && !clipRight )
  {
    //bottom-right CTU
    if ( xPos + ctuSize < cs.pcv->lumaWidth && yPos + ctuSize < cs.pcv->lumaHeight )
    {
      const Position nextCtuPos( xPos + ctuSize, yPos + ctuSize );
      const CodingUnit *nextCtu = cs.getCU( nextCtuPos, CHANNEL_TYPE_LUMA );
      if ( !pps->getLoopFilterAcrossSlicesEnabledFlag() && !CU::isSameSlice( *currCtu, *nextCtu ) )
      {
        rasterSliceAlfPad += 2;
      }
    }
  }

  return numHorVirBndry > 0 || numVerVirBndry > 0 || clipTop || clipBottom || clipLeft || clipRight || rasterSliceAlfPad;
}
#if !ALF_IMPROVEMENT
const int AdaptiveLoopFilter::m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF] =
{
  { 0,   0,   2,  -3,   1,  -4,   1,   7,  -1,   1,  -1,   5, 0 },
  { 0,   0,   0,   0,   0,  -1,   0,   1,   0,   0,  -1,   2, 0 },
  { 0,   0,   0,   0,   0,   0,   0,   1,   0,   0,   0,   0, 0 },
  { 0,   0,   0,   0,   0,   0,   0,   0,   0,   0,  -1,   1, 0 },
  { 2,   2,  -7,  -3,   0,  -5,  13,  22,  12,  -3,  -3,  17,  0 },
  { -1,   0,   6,  -8,   1,  -5,   1,  23,   0,   2,  -5,  10,  0 },
  { 0,   0,  -1,  -1,   0,  -1,   2,   1,   0,   0,  -1,   4, 0 },
  { 0,   0,   3, -11,   1,   0,  -1,  35,   5,   2,  -9,   9,  0 },
  { 0,   0,   8,  -8,  -2,  -7,   4,   4,   2,   1,  -1,  25,  0 },
  { 0,   0,   1,  -1,   0,  -3,   1,   3,  -1,   1,  -1,   3, 0 },
  { 0,   0,   3,  -3,   0,  -6,   5,  -1,   2,   1,  -4,  21,  0 },
  { -7,   1,   5,   4,  -3,   5,  11,  13,  12,  -8,  11,  12,  0 },
  { -5,  -3,   6,  -2,  -3,   8,  14,  15,   2,  -7,  11,  16,  0 },
  { 2,  -1,  -6,  -5,  -2,  -2,  20,  14,  -4,   0,  -3,  25,  0 },
  { 3,   1,  -8,  -4,   0,  -8,  22,   5,  -3,   2, -10,  29,  0 },
  { 2,   1,  -7,  -1,   2, -11,  23,  -5,   0,   2, -10,  29,  0 },
  { -6,  -3,   8,   9,  -4,   8,   9,   7,  14,  -2,   8,   9,  0 },
  { 2,   1,  -4,  -7,   0,  -8,  17,  22,   1,  -1,  -4,  23,  0 },
  { 3,   0,  -5,  -7,   0,  -7,  15,  18,  -5,   0,  -5,  27,  0 },
  { 2,   0,   0,  -7,   1, -10,  13,  13,  -4,   2,  -7,  24,  0 },
  { 3,   3, -13,   4,  -2,  -5,   9,  21,  25,  -2,  -3,  12,  0 },
  { -5,  -2,   7,  -3,  -7,   9,   8,   9,  16,  -2,  15,  12,  0 },
  { 0,  -1,   0,  -7,  -5,   4,  11,  11,   8,  -6,  12,  21,  0 },
  { 3,  -2,  -3,  -8,  -4,  -1,  16,  15,  -2,  -3,   3,  26,  0 },
  { 2,   1,  -5,  -4,  -1,  -8,  16,   4,  -2,   1,  -7,  33,  0 },
  { 2,   1,  -4,  -2,   1, -10,  17,  -2,   0,   2, -11,  33,  0 },
  { 1,  -2,   7, -15, -16,  10,   8,   8,  20,  11,  14,  11,  0 },
  { 2,   2,   3, -13, -13,   4,   8,  12,   2,  -3,  16,  24,  0 },
  { 1,   4,   0,  -7,  -8,  -4,   9,   9,  -2,  -2,   8,  29,  0 },
  { 1,   1,   2,  -4,  -1,  -6,   6,   3,  -1,  -1,  -3,  30,  0 },
  { -7,   3,   2,  10,  -2,   3,   7,  11,  19,  -7,   8,  10, 0 },
  { 0,  -2,  -5,  -3,  -2,   4,  20,  15,  -1,  -3,  -1,  22,  0 },
  { 3,  -1,  -8,  -4,  -1,  -4,  22,   8,  -4,   2,  -8,  28,  0 },
  { 0,   3, -14,   3,   0,   1,  19,  17,   8,  -3,  -7,  20,  0 },
  { 0,   2,  -1,  -8,   3,  -6,   5,  21,   1,   1,  -9,  13,  0 },
  { -4,  -2,   8,  20,  -2,   2,   3,   5,  21,   4,   6,   1, 0 },
  { 2,  -2,  -3,  -9,  -4,   2,  14,  16,   3,  -6,   8,  24,  0 },
  { 2,   1,   5, -16,  -7,   2,   3,  11,  15,  -3,  11,  22,  0 },
  { 1,   2,   3, -11,  -2,  -5,   4,   8,   9,  -3,  -2,  26,  0 },
  { 0,  -1,  10,  -9,  -1,  -8,   2,   3,   4,   0,   0,  29,  0 },
  { 1,   2,   0,  -5,   1,  -9,   9,   3,   0,   1,  -7,  20,  0 },
  { -2,   8,  -6,  -4,   3,  -9,  -8,  45,  14,   2, -13,   7, 0 },
  { 1,  -1,  16, -19,  -8,  -4,  -3,   2,  19,   0,   4,  30,  0 },
  { 1,   1,  -3,   0,   2, -11,  15,  -5,   1,   2,  -9,  24,  0 },
  { 0,   1,  -2,   0,   1,  -4,   4,   0,   0,   1,  -4,   7,  0 },
  { 0,   1,   2,  -5,   1,  -6,   4,  10,  -2,   1,  -4,  10,  0 },
  { 3,   0,  -3,  -6,  -2,  -6,  14,   8,  -1,  -1,  -3,  31,  0 },
  { 0,   1,   0,  -2,   1,  -6,   5,   1,   0,   1,  -5,  13,  0 },
  { 3,   1,   9, -19, -21,   9,   7,   6,  13,   5,  15,  21,  0 },
  { 2,   4,   3, -12, -13,   1,   7,   8,   3,   0,  12,  26,  0 },
  { 3,   1,  -8,  -2,   0,  -6,  18,   2,  -2,   3, -10,  23,  0 },
  { 1,   1,  -4,  -1,   1,  -5,   8,   1,  -1,   2,  -5,  10,  0 },
  { 0,   1,  -1,   0,   0,  -2,   2,   0,   0,   1,  -2,   3,  0 },
  { 1,   1,  -2,  -7,   1,  -7,  14,  18,   0,   0,  -7,  21,  0 },
  { 0,   1,   0,  -2,   0,  -7,   8,   1,  -2,   0,  -3,  24,  0 },
  { 0,   1,   1,  -2,   2, -10,  10,   0,  -2,   1,  -7,  23,  0 },
  { 0,   2,   2, -11,   2,  -4,  -3,  39,   7,   1, -10,   9,  0 },
  { 1,   0,  13, -16,  -5,  -6,  -1,   8,   6,   0,   6,  29,  0 },
  { 1,   3,   1,  -6,  -4,  -7,   9,   6,  -3,  -2,   3,  33,  0 },
  { 4,   0, -17,  -1,  -1,   5,  26,   8,  -2,   3, -15,  30,  0 },
  { 0,   1,  -2,   0,   2,  -8,  12,  -6,   1,   1,  -6,  16,  0 },
  { 0,   0,   0,  -1,   1,  -4,   4,   0,   0,   0,  -3,  11,  0 },
  { 0,   1,   2,  -8,   2,  -6,   5,  15,   0,   2,  -7,   9,  0 },
  { 1,  -1,  12, -15,  -7,  -2,   3,   6,   6,  -1,   7,  30,  0 },
};
const int AdaptiveLoopFilter::m_classToFilterMapping[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES] =
{
  { 8,   2,   2,   2,   3,   4,  53,   9,   9,  52,   4,   4,   5,   9,   2,   8,  10,   9,   1,   3,  39,  39,  10,   9,  52 },
  { 11,  12,  13,  14,  15,  30,  11,  17,  18,  19,  16,  20,  20,   4,  53,  21,  22,  23,  14,  25,  26,  26,  27,  28,  10 },
  { 16,  12,  31,  32,  14,  16,  30,  33,  53,  34,  35,  16,  20,   4,   7,  16,  21,  36,  18,  19,  21,  26,  37,  38,  39 },
  { 35,  11,  13,  14,  43,  35,  16,   4,  34,  62,  35,  35,  30,  56,   7,  35,  21,  38,  24,  40,  16,  21,  48,  57,  39 },
  { 11,  31,  32,  43,  44,  16,   4,  17,  34,  45,  30,  20,  20,   7,   5,  21,  22,  46,  40,  47,  26,  48,  63,  58,  10 },
  { 12,  13,  50,  51,  52,  11,  17,  53,  45,   9,  30,   4,  53,  19,   0,  22,  23,  25,  43,  44,  37,  27,  28,  10,  55 },
  { 30,  33,  62,  51,  44,  20,  41,  56,  34,  45,  20,  41,  41,  56,   5,  30,  56,  38,  40,  47,  11,  37,  42,  57,   8 },
  { 35,  11,  23,  32,  14,  35,  20,   4,  17,  18,  21,  20,  20,  20,   4,  16,  21,  36,  46,  25,  41,  26,  48,  49,  58 },
  { 12,  31,  59,  59,   3,  33,  33,  59,  59,  52,   4,  33,  17,  59,  55,  22,  36,  59,  59,  60,  22,  36,  59,  25,  55 },
  { 31,  25,  15,  60,  60,  22,  17,  19,  55,  55,  20,  20,  53,  19,  55,  22,  46,  25,  43,  60,  37,  28,  10,  55,  52 },
  { 12,  31,  32,  50,  51,  11,  33,  53,  19,  45,  16,   4,   4,  53,   5,  22,  36,  18,  25,  43,  26,  27,  27,  28,  10 },
  { 5,   2,  44,  52,   3,   4,  53,  45,   9,   3,   4,  56,   5,   0,   2,   5,  10,  47,  52,   3,  63,  39,  10,   9,  52 },
  { 12,  34,  44,  44,   3,  56,  56,  62,  45,   9,  56,  56,   7,   5,   0,  22,  38,  40,  47,  52,  48,  57,  39,  10,   9 },
  { 35,  11,  23,  14,  51,  35,  20,  41,  56,  62,  16,  20,  41,  56,   7,  16,  21,  38,  24,  40,  26,  26,  42,  57,  39 },
  { 33,  34,  51,  51,  52,  41,  41,  34,  62,   0,  41,  41,  56,   7,   5,  56,  38,  38,  40,  44,  37,  42,  57,  39,  10 },
  { 16,  31,  32,  15,  60,  30,   4,  17,  19,  25,  22,  20,   4,  53,  19,  21,  22,  46,  25,  55,  26,  48,  63,  58,  55 },
};
#endif
void AdaptiveLoopFilter::applyCcAlfFilter(CodingStructure &cs, ComponentID compID, const PelBuf &dstBuf,
                                          const PelUnitBuf &recYuvExt, uint8_t *filterControl,
                                          const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF],
                                          const int   selectedFilterIdx)
{
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int  numHorVirBndry = 0, numVerVirBndry = 0;
  int  horVirBndryPos[] = { 0, 0, 0 };
  int  verVirBndryPos[] = { 0, 0, 0 };

  int ctuIdx = 0;

  const int chromaScaleX = getComponentScaleX( compID, m_chromaFormat );
  const int chromaScaleY = getComponentScaleY( compID, m_chromaFormat );

  for( int yPos = 0; yPos < m_picHeight; yPos += m_maxCUHeight )
  {
    for( int xPos = 0; xPos < m_picWidth; xPos += m_maxCUWidth )
    {
      int filterIdx =
        (filterControl == nullptr)
          ? selectedFilterIdx
          : filterControl[(yPos >> cs.pcv->maxCUHeightLog2) * cs.pcv->widthInCtus + (xPos >> cs.pcv->maxCUWidthLog2)];
      bool skipFiltering = (filterControl != nullptr && filterIdx == 0) ? true : false;
      if (!skipFiltering)
      {
        if (filterControl != nullptr)
        {
          filterIdx--;
        }

        const int16_t *filterCoeff = filterSet[filterIdx];

        const int width        = (xPos + m_maxCUWidth > m_picWidth) ? (m_picWidth - xPos) : m_maxCUWidth;
        const int height       = (yPos + m_maxCUHeight > m_picHeight) ? (m_picHeight - yPos) : m_maxCUHeight;

        int rasterSliceAlfPad = 0;
        if (isCrossedByVirtualBoundaries(cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight,
                                         numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos,
                                         rasterSliceAlfPad))
        {
          int yStart = yPos;
          for (int i = 0; i <= numHorVirBndry; i++)
          {
            const int  yEnd   = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
            const int  h      = yEnd - yStart;
            const bool clipT  = (i == 0 && clipTop) || (i > 0) || (yStart == 0);
            const bool clipB  = (i == numHorVirBndry && clipBottom) || (i < numHorVirBndry) || (yEnd == m_picHeight);
            int        xStart = xPos;
            for (int j = 0; j <= numVerVirBndry; j++)
            {
              const int  xEnd  = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
              const int  w     = xEnd - xStart;
              const bool clipL = (j == 0 && clipLeft) || (j > 0) || (xStart == 0);
              const bool clipR = (j == numVerVirBndry && clipRight) || (j < numVerVirBndry) || (xEnd == m_picWidth);
              const int  wBuf  = w + (clipL ? 0 : MAX_ALF_PADDING_SIZE) + (clipR ? 0 : MAX_ALF_PADDING_SIZE);
              const int  hBuf  = h + (clipT ? 0 : MAX_ALF_PADDING_SIZE) + (clipB ? 0 : MAX_ALF_PADDING_SIZE);
              PelUnitBuf buf   = m_tempBuf2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
              buf.copyFrom(recYuvExt.subBuf(
                UnitArea(cs.area.chromaFormat, Area(xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE),
                                                    yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
              // pad top-left unavailable samples for raster slice
              if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
              {
                buf.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
              }

              // pad bottom-right unavailable samples for raster slice
              if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
              {
                buf.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
              }
#if JVET_AA0095_ALF_LONGER_FILTER 
              mirroredPaddingForAlf(cs, buf, MAX_ALF_PADDING_SIZE, true, true);
#else
              buf.extendBorderPel(MAX_ALF_PADDING_SIZE);
#endif
              buf = buf.subBuf(UnitArea(
                cs.area.chromaFormat, Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h)));

              const Area blkSrc(0, 0, w, h);

              const Area blkDst(xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY);
#if ALF_IMPROVEMENT
              m_filterCcAlf( dstBuf, buf, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs );
#else
              m_filterCcAlf(dstBuf, buf, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos);
#endif

              xStart = xEnd;
            }

            yStart = yEnd;
          }
        }
        else
        {
          const UnitArea area(m_chromaFormat, Area(xPos, yPos, width, height));

          Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
          Area blkSrc(xPos, yPos, width, height);
#if ALF_IMPROVEMENT
          m_filterCcAlf( dstBuf, recYuvExt, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs );
#else
          m_filterCcAlf(dstBuf, recYuvExt, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos);
#endif
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilter::ALFProcess(CodingStructure& cs)
{

  // set clipping range
  m_clpRngs = cs.slice->getClpRngs();

  // set CTU enable flags
  for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
  {
    m_ctuEnableFlag[compIdx] = cs.picture->getAlfCtuEnableFlag( compIdx );
    m_ctuAlternative[compIdx] = cs.picture->getAlfCtuAlternativeData( compIdx );
  }
  short* alfCtuFilterIndex = nullptr;
  uint32_t lastSliceIdx = 0xFFFFFFFF;

  PelUnitBuf recYuv = cs.getRecoBuf();
  m_tempBuf.copyFrom( recYuv );
  PelUnitBuf tmpYuv = m_tempBuf.getBuf( cs.area );
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_LONGER_FILTER 
  mirroredPaddingForAlf(cs, tmpYuv, MAX_FILTER_LENGTH_FIXED >> 1, true, true);
#else
  tmpYuv.extendBorderPel( MAX_FILTER_LENGTH_FIXED >> 1 );
#endif
#else
#if JVET_AA0095_ALF_LONGER_FILTER 
  mirroredPaddingForAlf(cs, tmpYuv, MAX_ALF_FILTER_LENGTH >> 1, true, true);
#else
  tmpYuv.extendBorderPel( MAX_ALF_FILTER_LENGTH >> 1 );
#endif
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  PelUnitBuf tmpYuvBeforeDb = m_tempBufBeforeDb.getBuf(cs.area);
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  mirroredPaddingForAlf(cs, tmpYuvBeforeDb, NUM_DB_PAD, true, false);
#else
  tmpYuvBeforeDb.extendBorderPel(NUM_DB_PAD);
#endif
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  PelUnitBuf tmpYuvResi = m_tempBufResi.getBuf(cs.area);
  mirroredPaddingForAlf(cs, tmpYuvResi, MAX_FILTER_LENGTH_FIXED >> 1, true, false);
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  m_isFixedFilterPaddedPerCtu = cs.picHeader->getVirtualBoundariesPresentFlag() || cs.slice->getCuQpDeltaSubdiv();
  memset(m_ctuEnableOnlineLumaFlag, 0, sizeof(uint8_t) * m_numCTUsInPic);
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  memset(m_ctuPadFlag, 0, sizeof(uint8_t) * m_numCTUsInPic);
#endif
  const PreCalcValues& pcv = *cs.pcv;

  int ctuIdx = 0;
  bool clipTop = false, clipBottom = false, clipLeft = false, clipRight = false;
  int numHorVirBndry = 0, numVerVirBndry = 0;
  int horVirBndryPos[] = { 0, 0, 0 };
  int verVirBndryPos[] = { 0, 0, 0 };

#if ALF_IMPROVEMENT
  int fixedFilterSetIdx = cs.slice->getTileGroupAlfFixedFilterSetIdx();
#endif

  for( int yPos = 0; yPos < pcv.lumaHeight; yPos += pcv.maxCUHeight )
  {
    for( int xPos = 0; xPos < pcv.lumaWidth; xPos += pcv.maxCUWidth )
    {
      // get first CU in CTU
      const CodingUnit *cu = cs.getCU( Position(xPos, yPos), CHANNEL_TYPE_LUMA );

      // skip this CTU if ALF is disabled
      if (!cu->slice->getTileGroupAlfEnabledFlag(COMPONENT_Y) && !cu->slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) && !cu->slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr))
      {
        ctuIdx++;
        continue;
      }

      // reload ALF APS each time the slice changes during raster scan filtering
      if(ctuIdx == 0 || lastSliceIdx != cu->slice->getSliceID() || alfCtuFilterIndex==nullptr)
      {
        cs.slice = cu->slice;
        reconstructCoeffAPSs(cs, true, cu->slice->getTileGroupAlfEnabledFlag(COMPONENT_Cb) || cu->slice->getTileGroupAlfEnabledFlag(COMPONENT_Cr), false);
        alfCtuFilterIndex = cu->slice->getPic()->getAlfCtbFilterIndex();
        m_ccAlfFilterParam = cu->slice->m_ccAlfFilterParam;
      }
      lastSliceIdx = cu->slice->getSliceID();

      const int width = ( xPos + pcv.maxCUWidth > pcv.lumaWidth ) ? ( pcv.lumaWidth - xPos ) : pcv.maxCUWidth;
      const int height = ( yPos + pcv.maxCUHeight > pcv.lumaHeight ) ? ( pcv.lumaHeight - yPos ) : pcv.maxCUHeight;
      bool ctuEnableFlag = m_ctuEnableFlag[COMPONENT_Y][ctuIdx];
      for( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
      {
        ctuEnableFlag |= m_ctuEnableFlag[compIdx][ctuIdx] > 0;
        if (cu->slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
        {
          ctuEnableFlag |= m_ccAlfFilterControl[compIdx - 1][ctuIdx] > 0;
        }
      }
      int rasterSliceAlfPad = 0;
      if( ctuEnableFlag && isCrossedByVirtualBoundaries( cs, xPos, yPos, width, height, clipTop, clipBottom, clipLeft, clipRight, numHorVirBndry, numVerVirBndry, horVirBndryPos, verVirBndryPos, rasterSliceAlfPad ) )
      {
        int yStart = yPos;
        for( int i = 0; i <= numHorVirBndry; i++ )
        {
          const int yEnd = i == numHorVirBndry ? yPos + height : horVirBndryPos[i];
          const int h = yEnd - yStart;
          const bool clipT = ( i == 0 && clipTop ) || ( i > 0 ) || ( yStart == 0 );
          const bool clipB = ( i == numHorVirBndry && clipBottom ) || ( i < numHorVirBndry ) || ( yEnd == pcv.lumaHeight );
          int xStart = xPos;
          for( int j = 0; j <= numVerVirBndry; j++ )
          {
            const int xEnd = j == numVerVirBndry ? xPos + width : verVirBndryPos[j];
            const int w = xEnd - xStart;
            const bool clipL = ( j == 0 && clipLeft ) || ( j > 0 ) || ( xStart == 0 );
            const bool clipR = ( j == numVerVirBndry && clipRight ) || ( j < numVerVirBndry ) || ( xEnd == pcv.lumaWidth );
            const int wBuf = w + ( clipL ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipR ? 0 : MAX_ALF_PADDING_SIZE );
            const int hBuf = h + ( clipT ? 0 : MAX_ALF_PADDING_SIZE ) + ( clipB ? 0 : MAX_ALF_PADDING_SIZE );
            PelUnitBuf buf = m_tempBuf2.subBuf( UnitArea( cs.area.chromaFormat, Area( 0, 0, wBuf, hBuf ) ) );
            buf.copyFrom( tmpYuv.subBuf( UnitArea( cs.area.chromaFormat, Area( xStart - ( clipL ? 0 : MAX_ALF_PADDING_SIZE ), yStart - ( clipT ? 0 : MAX_ALF_PADDING_SIZE ), wBuf, hBuf ) ) ) );
            // pad top-left unavailable samples for raster slice
            if( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
            {
              buf.padBorderPel( MAX_ALF_PADDING_SIZE, 1 );
            }

            // pad bottom-right unavailable samples for raster slice
            if( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
            {
              buf.padBorderPel( MAX_ALF_PADDING_SIZE, 2 );
            }
#if JVET_AA0095_ALF_LONGER_FILTER 
            mirroredPaddingForAlf(cs, buf, MAX_ALF_PADDING_SIZE, true, true);
#else
            buf.extendBorderPel( MAX_ALF_PADDING_SIZE );
#endif
            buf = buf.subBuf( UnitArea( cs.area.chromaFormat, Area( clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h ) ) );

#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
            PelUnitBuf bufResi = m_tempBufResi2.subBuf(UnitArea(cs.area.chromaFormat, Area(0, 0, wBuf, hBuf)));
            bufResi.copyFrom(tmpYuvResi.subBuf(
              UnitArea(cs.area.chromaFormat, Area(xStart - (clipL ? 0 : MAX_ALF_PADDING_SIZE),
                                                  yStart - (clipT ? 0 : MAX_ALF_PADDING_SIZE), wBuf, hBuf))));
            // pad top-left unavailable samples for raster slice
            if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
            {
              bufResi.padBorderPel(MAX_ALF_PADDING_SIZE, 1);
            }

            // pad bottom-right unavailable samples for raster slice
            if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
            {
              bufResi.padBorderPel(MAX_ALF_PADDING_SIZE, 2);
            }
#if JVET_AA0095_ALF_LONGER_FILTER
            mirroredPaddingForAlf(cs, bufResi, MAX_ALF_PADDING_SIZE, true, false);
#else
            bufResi.extendBorderPel(MAX_ALF_PADDING_SIZE);
#endif
            bufResi = bufResi.subBuf(UnitArea(cs.area.chromaFormat, Area(clipL ? 0 : MAX_ALF_PADDING_SIZE, clipT ? 0 : MAX_ALF_PADDING_SIZE, w, h)));
#endif

            if( m_ctuEnableFlag[COMPONENT_Y][ctuIdx] )
            {
              const Area blkSrc( 0, 0, w, h );
              const Area blkDst( xStart, yStart, w, h );
              short filterSetIndex = alfCtuFilterIndex[ctuIdx];
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
              PelUnitBuf bufDb = m_tempBufBeforeDb2.subBuf(UnitArea(CHROMA_400, Area(0, 0, wBuf, hBuf)));
              bufDb.copyFrom(m_tempBufBeforeDb.subBuf(UnitArea(CHROMA_400, Area(xStart - (clipL ? 0 : NUM_DB_PAD), yStart - (clipT ? 0 : NUM_DB_PAD), wBuf, hBuf))));
              // pad top-left unavailable samples for raster slice
              if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
              {
                bufDb.padBorderPel(NUM_DB_PAD, 1);
              }
              // pad bottom-right unavailable samples for raster slice
              if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
              {
                bufDb.padBorderPel(NUM_DB_PAD, 2);
              }
              bufDb.extendBorderPel(NUM_DB_PAD);
              bufDb = bufDb.subBuf(UnitArea(CHROMA_400, Area(clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h)));
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
              deriveClassification( m_classifier, buf.get(COMPONENT_Y), 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                m_filterTypeApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS] == ALF_FILTER_13_EXT_DB_RESI, bufResi.get(COMPONENT_Y),
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
                bufDb.get(COMPONENT_Y), 0,
#endif
                blkDst, blkSrc, cs, 
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
                -1,
#else
                filterSetIndex < NUM_FIXED_FILTER_SETS ? filterSetIndex : -1, 
#endif
                filterSetIndex < NUM_FIXED_FILTER_SETS ? -1 : m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][m_ctuAlternative[COMPONENT_Y][ctuIdx]] );
#else
              deriveClassification( m_classifier, buf.get( COMPONENT_Y ), blkDst, blkSrc 
#if ALF_IMPROVEMENT
              , cs, filterSetIndex < NUM_FIXED_FILTER_SETS ? filterSetIndex : -1
#endif
              );
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
              paddingFixedFilterResultsCtu( m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blkDst, 0 );
              if( filterSetIndex != 0 )
              {
                deriveFixedFilterResults( m_classifier, buf.get(COMPONENT_Y), bufDb.get(COMPONENT_Y), blkDst, blkSrc, cs, 1, fixedFilterSetIdx );
                paddingFixedFilterResultsCtu( m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blkDst, 1 );
              }
#endif
              short *coeff;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
              Pel *clip;
#else
              short *clip;
#endif
#if ALF_IMPROVEMENT
              if( filterSetIndex < NUM_FIXED_FILTER_SETS )
              {
                copyFixedFilterResults(recYuv, blkDst, COMPONENT_Y, m_fixFilterResult, fixedFilterSetIdx, filterSetIndex);
              }
              else
              {
                uint8_t alt_num = m_ctuAlternative[COMPONENT_Y][ctuIdx];
                coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
                clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
                AlfFilterType filterTypeCtb = m_filterTypeApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
                if( m_isFixedFilterPaddedPerCtu )
                {
                  paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blkDst);
                }
#endif
                m_ctuEnableOnlineLumaFlag[ctuIdx] = numFixedFilters(filterTypeCtb) > 1 ? true : false;
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
                int classifierIdx = m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
                PelUnitBuf bufDb = m_tempBufBeforeDb2.subBuf( UnitArea( CHROMA_400, Area( 0, 0, wBuf, hBuf ) ) );
#if JVET_AA0095_ALF_LONGER_FILTER
                if( filterTypeCtb == ALF_FILTER_9_EXT_DB || filterTypeCtb == ALF_FILTER_13_EXT_DB
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                    || filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI || filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI_DIRECT
#endif
                  )
#else
                if( filterTypeCtb == ALF_FILTER_9_EXT_DB )
#endif
                {
                  bufDb.copyFrom(m_tempBufBeforeDb.subBuf( UnitArea( CHROMA_400, Area( xStart - ( clipL ? 0 : NUM_DB_PAD ), yStart - ( clipT ? 0 : NUM_DB_PAD ), wBuf, hBuf ) ) ) );
                  // pad top-left unavailable samples for raster slice
                  if ( xStart == xPos && yStart == yPos && ( rasterSliceAlfPad & 1 ) )
                  {
                    bufDb.padBorderPel( NUM_DB_PAD, 1 );
                  }
                  // pad bottom-right unavailable samples for raster slice
                  if ( xEnd == xPos + width && yEnd == yPos + height && ( rasterSliceAlfPad & 2 ) )
                  {
                    bufDb.padBorderPel( NUM_DB_PAD, 2 );
                  }
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                  mirroredPaddingForAlf( cs, bufDb, NUM_DB_PAD, true, false );
#else
                  bufDb.extendBorderPel( NUM_DB_PAD );
#endif
                  bufDb = bufDb.subBuf( UnitArea( CHROMA_400, Area( clipL ? 0 : NUM_DB_PAD, clipT ? 0 : NUM_DB_PAD, w, h ) ) );
                }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                PelUnitBuf bufResi = m_tempBufResi2.subBuf(UnitArea(CHROMA_400, Area(0, 0, wBuf, hBuf)));
                if (filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI || filterTypeCtb == ALF_FILTER_13_EXT_DB_RESI_DIRECT
#if JVET_AD0222_ALF_RESI_CLASS
                    || classifierIdx == 2
#endif
                  )
                {
                  bufResi.copyFrom( m_tempBufResi.subBuf(UnitArea(CHROMA_400, Area(xStart - (clipL ? 0 : NUM_RESI_PAD), yStart - (clipT ? 0 : NUM_RESI_PAD), wBuf, hBuf))));
                  // pad top-left unavailable samples for raster slice
                  if (xStart == xPos && yStart == yPos && (rasterSliceAlfPad & 1))
                  {
                    bufResi.padBorderPel(NUM_RESI_PAD, 1);
                  }
                  // pad bottom-right unavailable samples for raster slice
                  if (xEnd == xPos + width && yEnd == yPos + height && (rasterSliceAlfPad & 2))
                  {
                    bufResi.padBorderPel(NUM_RESI_PAD, 2);
                  }
                  bufResi.extendBorderPel(NUM_RESI_PAD);
                  bufResi = bufResi.subBuf(UnitArea(CHROMA_400, Area(clipL ? 0 : NUM_RESI_PAD, clipT ? 0 : NUM_RESI_PAD, w, h)));
                }
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                deriveGaussResults( bufDb.get(COMPONENT_Y), blkDst, blkSrc, cs, 0, 0 );
                if( m_isFixedFilterPaddedPerCtu )
                {
                  for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
                  {
                    paddingGaussResultsCtu(m_gaussPic, m_gaussCtu, gaussIdx, blkDst);
                  }
                }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                alfFiltering(m_classifier[classifierIdx], recYuv, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#else
                alfFiltering(m_classifier[classifierIdx], recYuv, bufDb, bufResi, buf, blkDst, blkSrc, COMPONENT_Y,  coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[classifierIdx], recYuv, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier[classifierIdx], recYuv, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[classifierIdx], recYuv, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier[classifierIdx], recYuv, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recYuv, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier, recYuv, bufDb, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recYuv, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
                alfFiltering( m_classifier, recYuv, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#endif
              }
#else
              if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
              {
                coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
                clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
              }
              else
              {
                coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
                clip = m_clipDefault;
              }
              m_filter7x7Blk( m_classifier, recYuv, buf, blkDst, blkSrc, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
#endif
          }

            for( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
            {
              ComponentID compID = ComponentID( compIdx );
              const int chromaScaleX = getComponentScaleX( compID, tmpYuv.chromaFormat );
              const int chromaScaleY = getComponentScaleY( compID, tmpYuv.chromaFormat );

              if( m_ctuEnableFlag[compIdx][ctuIdx] )
              {
                const Area blkSrc( 0, 0, w >> chromaScaleX, h >> chromaScaleY );
                const Area blkDst( xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY );
                uint8_t alt_num = m_ctuAlternative[compIdx][ctuIdx];
#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                alfFiltering(m_classifier[0], recYuv, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false, m_gaussPic, m_gaussCtu);
#else
                alfFiltering(m_classifier[0], recYuv, tmpYuvBeforeDb, tmpYuvResi, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[0], recYuv, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier[0], recYuv, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier[0], recYuv, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier[0], recYuv, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recYuv, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier, recYuv, tmpYuvBeforeDb, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                alfFiltering( m_classifier, recYuv, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1, nullptr, false );
#else
                alfFiltering( m_classifier, recYuv, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, -1 );
#endif
#endif
#endif
#else
                m_filter5x5Blk(m_classifier, recYuv, buf, blkDst, blkSrc, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_alfVBChmaCTUHeight, m_alfVBChmaPos );
#endif
              }
              if (cu->slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
              {
                const int filterIdx = m_ccAlfFilterControl[compIdx - 1][ctuIdx];

                if (filterIdx != 0)
                {
                  const Area blkSrc(0, 0, w, h);
                  Area blkDst(xStart >> chromaScaleX, yStart >> chromaScaleY, w >> chromaScaleX, h >> chromaScaleY);

                  const int16_t *filterCoeff = m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1][filterIdx - 1];
#if ALF_IMPROVEMENT
                  m_filterCcAlf( recYuv.get( compID ), buf, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs );
#else
                  m_filterCcAlf(recYuv.get(compID), buf, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos);
#endif
                }
              }
            }

            xStart = xEnd;
          }

          yStart = yEnd;
        }
      }
      else
      {
        const UnitArea area( cs.area.chromaFormat, Area( xPos, yPos, width, height ) );
        if( m_ctuEnableFlag[COMPONENT_Y][ctuIdx] )
        {
          Area blk( xPos, yPos, width, height );
          short filterSetIndex = alfCtuFilterIndex[ctuIdx];
#if JVET_X0071_ALF_BAND_CLASSIFIER
          deriveClassification( m_classifier, tmpYuv.get(COMPONENT_Y), 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
            m_filterTypeApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS] == ALF_FILTER_13_EXT_DB_RESI, tmpYuvResi.get(COMPONENT_Y),
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            tmpYuvBeforeDb.get( COMPONENT_Y ), m_ctuPadFlag[ctuIdx],
#endif
            blk, blk, cs,
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            - 1,
#else
            filterSetIndex < NUM_FIXED_FILTER_SETS ? filterSetIndex : -1, 
#endif
            filterSetIndex < NUM_FIXED_FILTER_SETS ? -1 : m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][m_ctuAlternative[COMPONENT_Y][ctuIdx]] );
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          if( filterSetIndex != 0 )
          {
            deriveFixedFilterResultsCtuBoundary( m_classifier, m_fixFilterResult, tmpYuv.get( COMPONENT_Y ), tmpYuvBeforeDb.get( COMPONENT_Y ), blk, m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), fixedFilterSetIdx, m_mappingDir, m_laplacian, m_ctuEnableFlag[COMPONENT_Y], m_ctuEnableOnlineLumaFlag, ctuIdx, 0 );

            deriveFixedFilterResults( m_classifier, tmpYuv.get( COMPONENT_Y ), m_tempBufBeforeDb.get( COMPONENT_Y ), blk, blk, cs, 1, fixedFilterSetIdx );

            deriveFixedFilterResultsCtuBoundary( m_classifier, m_fixFilterResult, tmpYuv.get( COMPONENT_Y ), tmpYuvBeforeDb.get( COMPONENT_Y ), blk, m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), fixedFilterSetIdx, m_mappingDir, m_laplacian, m_ctuEnableFlag[COMPONENT_Y], m_ctuEnableOnlineLumaFlag, ctuIdx, 1 );
        }
#endif
#else
          deriveClassification( m_classifier, tmpYuv.get( COMPONENT_Y ), blk, blk 
#if ALF_IMPROVEMENT
            , cs, filterSetIndex < NUM_FIXED_FILTER_SETS ? filterSetIndex : -1
#endif
          );      
#endif
          short *coeff;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
          Pel *clip;
#else
          short *clip;
#endif
#if ALF_IMPROVEMENT
          if( filterSetIndex < NUM_FIXED_FILTER_SETS )
          {
            copyFixedFilterResults( recYuv, blk, COMPONENT_Y, m_fixFilterResult, fixedFilterSetIdx, filterSetIndex );
          }
          else
          {
            uint8_t alt_num = m_ctuAlternative[COMPONENT_Y][ctuIdx];
            coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
            clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
            AlfFilterType filterTypeCtb = m_filterTypeApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            if( m_isFixedFilterPaddedPerCtu )
            {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
              paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blk, 1);
#else
              paddingFixedFilterResultsCtu(m_fixFilterResult, m_fixedFilterResultPerCtu, fixedFilterSetIdx, blk);
#endif
            }
            else
            {
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
              deriveFixedFilterResultsCtuBoundary(m_classifier[0], m_fixFilterResult, tmpYuv.get(COMPONENT_Y), blk, m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), fixedFilterSetIdx, m_mappingDir, m_laplacian, m_ctuEnableFlag[COMPONENT_Y], m_ctuEnableOnlineLumaFlag, ctuIdx);
#endif
#else
              deriveFixedFilterResultsCtuBoundary(m_classifier, m_fixFilterResult, tmpYuv.get(COMPONENT_Y), blk, m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), fixedFilterSetIdx, m_mappingDir, m_laplacian, m_ctuEnableFlag[COMPONENT_Y], m_ctuEnableOnlineLumaFlag, ctuIdx);
#endif
            }
            m_ctuEnableOnlineLumaFlag[ctuIdx] = numFixedFilters(filterTypeCtb) > 1 ? true : false;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            deriveGaussResults( tmpYuvBeforeDb.get(COMPONENT_Y), blk, blk, cs, 0, 0 );
            if( m_isFixedFilterPaddedPerCtu )
            {
              for(int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
              {
                paddingGaussResultsCtu(m_gaussPic, m_gaussCtu, gaussIdx, blk);
              }
            }
            else
            {
              deriveGaussResultsCtuBoundary(m_gaussPic, tmpYuvBeforeDb.get(COMPONENT_Y), blk, cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], m_ctuEnableFlag[COMPONENT_Y], m_ctuEnableOnlineLumaFlag, ctuIdx, 0, 0 );
            }
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
            int classifierIdx = m_classifierIdxApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS][alt_num];
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            alfFiltering(m_classifier[classifierIdx], recYuv, tmpYuvBeforeDb, tmpYuvResi, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu, m_gaussPic, m_gaussCtu);
#else
            alfFiltering(m_classifier[classifierIdx], recYuv, tmpYuvBeforeDb, tmpYuvResi, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, m_fixFilterResiResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier[classifierIdx], recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
            alfFiltering( m_classifier[classifierIdx], recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier[classifierIdx], recYuv, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
            alfFiltering( m_classifier[classifierIdx], recYuv, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier, recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
            alfFiltering( m_classifier, recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier, recYuv, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx, m_fixedFilterResultPerCtu, m_isFixedFilterPaddedPerCtu );
#else
            alfFiltering( m_classifier, recYuv, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, filterTypeCtb, m_fixFilterResult, fixedFilterSetIdx );
#endif
#endif
#endif
          }
#else
          if( filterSetIndex >= NUM_FIXED_FILTER_SETS )
          {
            coeff = m_coeffApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
            clip = m_clippApsLuma[filterSetIndex - NUM_FIXED_FILTER_SETS];
          }
          else
          {
            coeff = m_fixedFilterSetCoeffDec[filterSetIndex];
            clip = m_clipDefault;
          }
          m_filter7x7Blk( m_classifier, recYuv, tmpYuv, blk, blk, COMPONENT_Y, coeff, clip, m_clpRngs.comp[COMPONENT_Y], cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos );
#endif
        }

        for( int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++ )
        {
          ComponentID compID = ComponentID( compIdx );
          const int chromaScaleX = getComponentScaleX( compID, tmpYuv.chromaFormat );
          const int chromaScaleY = getComponentScaleY( compID, tmpYuv.chromaFormat );

          if (m_ctuEnableFlag[compIdx][ctuIdx])
          {
            Area    blk(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
            uint8_t alt_num = m_ctuAlternative[compIdx][ctuIdx];

#if ALF_IMPROVEMENT
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            alfFiltering(m_classifier[0], recYuv, tmpYuvBeforeDb, tmpYuvResi, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false, m_gaussPic, m_gaussCtu);
#else
            alfFiltering(m_classifier[0], recYuv, tmpYuvBeforeDb, tmpYuvResi, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma, nullptr, nullptr, -1, nullptr, false);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier[0], recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1, nullptr, false );
#else
            alfFiltering( m_classifier[0], recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1 );
#endif
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier[0], recYuv, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1, nullptr, false );
#else
            alfFiltering( m_classifier[0], recYuv, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1 );
#endif
#endif
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier, recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1, nullptr, false );
#else
            alfFiltering( m_classifier, recYuv, tmpYuvBeforeDb, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1 );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            alfFiltering( m_classifier, recYuv, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1, nullptr, false );
#else
            alfFiltering( m_classifier, recYuv, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_filterTypeApsChroma  , nullptr, -1 );
#endif
#endif
#endif
#else
            m_filter5x5Blk( m_classifier, recYuv, tmpYuv, blk, blk, compID, m_chromaCoeffFinal[alt_num], m_chromaClippFinal[alt_num], m_clpRngs.comp[compIdx], cs, m_alfVBChmaCTUHeight, m_alfVBChmaPos );
#endif
          }
          if (cu->slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
          {
            const int filterIdx = m_ccAlfFilterControl[compIdx - 1][ctuIdx];

            if (filterIdx != 0)
            {
              Area blkDst(xPos >> chromaScaleX, yPos >> chromaScaleY, width >> chromaScaleX, height >> chromaScaleY);
              Area blkSrc(xPos, yPos, width, height);

              const int16_t *filterCoeff = m_ccAlfFilterParam.ccAlfCoeff[compIdx - 1][filterIdx - 1];

#if ALF_IMPROVEMENT
              m_filterCcAlf( recYuv.get( compID ), tmpYuv, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs );
#else
              m_filterCcAlf(recYuv.get(compID), tmpYuv, blkDst, blkSrc, compID, filterCoeff, m_clpRngs, cs, m_alfVBLumaCTUHeight, m_alfVBLumaPos);
#endif
            }
          }
        }
      }
      ctuIdx++;
    }
  }
}

void AdaptiveLoopFilter::reconstructCoeffAPSs(CodingStructure& cs, bool luma, bool chroma, bool isRdo)
{
  //luma
  APS** aps = cs.slice->getAlfAPSs();
  AlfParam alfParamTmp;
  APS* curAPS;
  if (luma)
  {
    for (int i = 0; i < cs.slice->getTileGroupNumAps(); i++)
    {
      int apsIdx = cs.slice->getTileGroupApsIdLuma()[i];
      curAPS = aps[apsIdx];
      CHECK(curAPS == NULL, "invalid APS");
      alfParamTmp = curAPS->getAlfAPSParam();
      reconstructCoeff(alfParamTmp, CHANNEL_TYPE_LUMA, isRdo, true);
      memcpy(m_coeffApsLuma[i], m_coeffFinal, sizeof(m_coeffFinal));
      memcpy(m_clippApsLuma[i], m_clippFinal, sizeof(m_clippFinal));
#if JVET_X0071_ALF_BAND_CLASSIFIER
      memcpy(m_classifierIdxApsLuma[i], m_classifierFinal, sizeof(m_classifierFinal));
#endif
#if ALF_IMPROVEMENT
      m_filterTypeApsLuma[i] = alfParamTmp.filterType[CHANNEL_TYPE_LUMA];
      m_numLumaAltAps[i] = alfParamTmp.numAlternativesLuma;
#endif
    }
  }

  //chroma
  if (chroma)
  {
    int apsIdxChroma = cs.slice->getTileGroupApsIdChroma();
    curAPS = aps[apsIdxChroma];
    m_alfParamChroma = &curAPS->getAlfAPSParam();
    alfParamTmp = *m_alfParamChroma;
    reconstructCoeff(alfParamTmp, CHANNEL_TYPE_CHROMA, isRdo, true);
#if ALF_IMPROVEMENT
    m_filterTypeApsChroma = alfParamTmp.filterType[CHANNEL_TYPE_CHROMA];
#endif
  }
}

void AdaptiveLoopFilter::reconstructCoeff( AlfParam& alfParam, ChannelType channel, const bool isRdo, const bool isRedo )
{
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  int factor = 0;
#else
  int factor = isRdo ? 0 : (1 << (m_NUM_BITS - 1));
#endif
#if ALF_IMPROVEMENT
  AlfFilterType filterType = alfParam.filterType[channel];
  int numClasses = isLuma(channel) ? MAX_NUM_ALF_CLASSES : 1;
  int numCoeff = m_filterShapes[channel][m_filterTypeToStatIndex[channel][filterType]].numCoeff;
#else
  AlfFilterType filterType = isLuma( channel ) ? ALF_FILTER_7 : ALF_FILTER_5;
  int numClasses = isLuma( channel ) ? MAX_NUM_ALF_CLASSES : 1;
  int numCoeff = filterType == ALF_FILTER_5 ? 7 : 13;
#endif
  int numCoeffMinus1 = numCoeff - 1;
#if ALF_IMPROVEMENT
  const int numAlts = isLuma( channel ) ? alfParam.numAlternativesLuma : alfParam.numAlternativesChroma;
#else
  const int numAlts = isLuma( channel ) ? 1 : alfParam.numAlternativesChroma;
#endif

  for( int altIdx = 0; altIdx < numAlts; ++ altIdx )
  {
#if ALF_IMPROVEMENT
    int numFilters = isLuma( channel ) ? alfParam.numLumaFilters[altIdx] : 1;
    short* coeff = isLuma( channel ) ? alfParam.lumaCoeff[altIdx] : alfParam.chromaCoeff[altIdx];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    Pel* clipp = isLuma( channel ) ? alfParam.lumaClipp[altIdx] : alfParam.chromaClipp[altIdx];
#else
    short* clipp = isLuma( channel ) ? alfParam.lumaClipp[altIdx] : alfParam.chromaClipp[altIdx];
#endif
#else
    int numFilters = isLuma( channel ) ? alfParam.numLumaFilters : 1;
    short* coeff = isLuma( channel ) ? alfParam.lumaCoeff : alfParam.chromaCoeff[altIdx];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    Pel* clipp = isLuma( channel ) ? alfParam.lumaClipp : alfParam.chromaClipp[altIdx];
#else
    short* clipp = isLuma( channel ) ? alfParam.lumaClipp : alfParam.chromaClipp[altIdx];
#endif
#endif

    for( int filterIdx = 0; filterIdx < numFilters; filterIdx++ )
    {
      coeff[filterIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor;
    }

    if( isChroma( channel ) )
    {
      for( int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx )
      {
        m_chromaCoeffFinal[altIdx][coeffIdx] = coeff[coeffIdx];
#if ALF_IMPROVEMENT
        int clipIdx = alfParam.nonLinearFlag[channel][altIdx] ? clipp[coeffIdx] : 0;
#else
        int clipIdx = alfParam.nonLinearFlag[channel] ? clipp[coeffIdx] : 0;
#endif
        m_chromaClippFinal[altIdx][coeffIdx] = isRdo ? clipIdx : m_alfClippingValues[channel][clipIdx];
      }
      m_chromaCoeffFinal[altIdx][numCoeffMinus1] = factor;
      m_chromaClippFinal[altIdx][numCoeffMinus1] = isRdo ? 0 : m_alfClippingValues[channel][0];
      continue;
    }
#if JVET_X0071_ALF_BAND_CLASSIFIER
    m_classifierFinal[altIdx] = alfParam.lumaClassifierIdx[altIdx];
#endif
    for( int classIdx = 0; classIdx < numClasses; classIdx++ )
    {
#if ALF_IMPROVEMENT
      int filterIdx = alfParam.filterCoeffDeltaIdx[altIdx][classIdx];
      CHECK(!(filterIdx >= 0 && filterIdx < alfParam.numLumaFilters[altIdx]), "Bad coeff delta idx in ALF");
      for (int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx)
      {
        m_coeffFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] = coeff[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx];
      }
      m_coeffFinal[altIdx][classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor;
      m_clippFinal[altIdx][classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = isRdo ? 0 : m_alfClippingValues[channel][0];
      for (int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx)
      {
        int clipIdx = alfParam.nonLinearFlag[channel][altIdx] ? clipp[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] : 0;
        CHECK(!(clipIdx >= 0 && clipIdx < MaxAlfNumClippingValues), "Bad clip idx in ALF");
        m_clippFinal[altIdx][classIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] = isRdo ? clipIdx : m_alfClippingValues[channel][clipIdx];
      }
      m_clippFinal[altIdx][classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = isRdo ? 0 : m_alfClippingValues[channel][0];
#else
      int filterIdx = alfParam.filterCoeffDeltaIdx[classIdx];

      CHECK(!(filterIdx >= 0 && filterIdx < alfParam.numLumaFilters), "Bad coeff delta idx in ALF");
      for (int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx)
      {
        m_coeffFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] = coeff[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx];
      }
      m_coeffFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = factor;
      m_clippFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = isRdo ? 0 : m_alfClippingValues[channel][0];
      for( int coeffIdx = 0; coeffIdx < numCoeffMinus1; ++coeffIdx )
      {
        int clipIdx = alfParam.nonLinearFlag[channel] ? clipp[filterIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] : 0;
        CHECK(!(clipIdx >= 0 && clipIdx < MaxAlfNumClippingValues), "Bad clip idx in ALF");
        m_clippFinal[classIdx * MAX_NUM_ALF_LUMA_COEFF + coeffIdx] = isRdo ? clipIdx : m_alfClippingValues[channel][clipIdx];
      }
      m_clippFinal[classIdx* MAX_NUM_ALF_LUMA_COEFF + numCoeffMinus1] = isRdo ? 0 : m_alfClippingValues[channel][0];
#endif
    }
  }
}

void AdaptiveLoopFilter::create(const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE])
{
  destroy();
  std::memcpy(m_inputBitDepth, inputBitDepth, sizeof(m_inputBitDepth));
  m_picWidth = picWidth;
  m_picHeight = picHeight;
  m_maxCUWidth = maxCUWidth;
  m_maxCUHeight = maxCUHeight;
  m_maxCUDepth = maxCUDepth;
  m_chromaFormat = format;

  m_numCTUsInWidth = (m_picWidth / m_maxCUWidth) + ((m_picWidth % m_maxCUWidth) ? 1 : 0);
  m_numCTUsInHeight = (m_picHeight / m_maxCUHeight) + ((m_picHeight % m_maxCUHeight) ? 1 : 0);
  m_numCTUsInPic = m_numCTUsInHeight * m_numCTUsInWidth;
  m_filterShapesCcAlf.push_back(AlfFilterShape(size_CC_ALF));

#if ALF_IMPROVEMENT
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(5));
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(7));
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(9));
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_9_EXT));
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_EXT));
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_13_EXT_DB));
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_9_EXT_DB));
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_13_EXT));
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_13_EXT_DB_RESI_DIRECT));
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(size_ALF_FILTER_13_EXT_DB_RESI));
#endif
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back(AlfFilterShape(5));
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back(AlfFilterShape(7));
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back(AlfFilterShape(9));
  memset(m_filterTypeTest, 0, sizeof(m_filterTypeTest));
  memset(m_filterTypeToStatIndex, 0, sizeof(m_filterTypeToStatIndex));
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_5] = 0;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_7] = 1;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_9] = 2;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_9_EXT] = 3;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_EXT] = 4;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB] = 5;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_9_EXT_DB] = 6;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT] = 7;
#elif JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_9_EXT_DB] = 5;
#elif JVET_AA0095_ALF_LONGER_FILTER
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT] = 5;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = 8;
  m_filterTypeToStatIndex[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI]        = 9;
#endif
  m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][ALF_FILTER_5] = 0;
  m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][ALF_FILTER_7] = 1;
  m_filterTypeToStatIndex[CHANNEL_TYPE_CHROMA][ALF_FILTER_9] = 2;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF && JVET_AA0095_ALF_LONGER_FILTER
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB] = false;
#else
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB] = true;
#endif
#elif JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_9_EXT_DB] = true;
#elif JVET_AA0095_ALF_LONGER_FILTER
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT] = true;
#else
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_9_EXT] = true;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI_DIRECT] = true;
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_13_EXT_DB_RESI]        = true;
#endif
  m_filterTypeTest[CHANNEL_TYPE_LUMA][ALF_FILTER_EXT] = false;
  m_filterTypeTest[CHANNEL_TYPE_CHROMA][ALF_FILTER_9] = true;
#else
  m_filterShapes[CHANNEL_TYPE_LUMA].push_back(AlfFilterShape(7));
  m_filterShapes[CHANNEL_TYPE_CHROMA].push_back(AlfFilterShape(5));
  m_alfVBLumaPos = m_maxCUHeight - ALF_VB_POS_ABOVE_CTUROW_LUMA;
  m_alfVBChmaPos = (m_maxCUHeight >> ((m_chromaFormat == CHROMA_420) ? 1 : 0)) - ALF_VB_POS_ABOVE_CTUROW_CHMA;

  m_alfVBLumaCTUHeight = m_maxCUHeight;
  m_alfVBChmaCTUHeight = (m_maxCUHeight >> ((m_chromaFormat == CHROMA_420) ? 1 : 0));
#endif

  static_assert(AlfNumClippingValues[CHANNEL_TYPE_LUMA] > 0, "AlfNumClippingValues[CHANNEL_TYPE_LUMA] must be at least one");
  m_alfClippingValues[CHANNEL_TYPE_LUMA][0] = 1 << m_inputBitDepth[CHANNEL_TYPE_LUMA];
  int shiftLuma = m_inputBitDepth[CHANNEL_TYPE_LUMA] - 8;
  for (int i = 1; i < AlfNumClippingValues[CHANNEL_TYPE_LUMA]; ++i)
  {
    m_alfClippingValues[CHANNEL_TYPE_LUMA][i] = 1 << (7 - 2 * i + shiftLuma);
  }
  static_assert(AlfNumClippingValues[CHANNEL_TYPE_CHROMA] > 0, "AlfNumClippingValues[CHANNEL_TYPE_CHROMA] must be at least one");
  m_alfClippingValues[CHANNEL_TYPE_CHROMA][0] = 1 << m_inputBitDepth[CHANNEL_TYPE_CHROMA];
  int shiftChroma = m_inputBitDepth[CHANNEL_TYPE_CHROMA] - 8;
  for (int i = 1; i < AlfNumClippingValues[CHANNEL_TYPE_CHROMA]; ++i)
  {
    m_alfClippingValues[CHANNEL_TYPE_CHROMA][i] = 1 << (7 - 2 * i + shiftChroma);
  }
  if (m_created)
  {
    return;
  }

  m_tempBuf.destroy();
  // NOTE: make border 1 sample wider to avoid out-of-bounds memory access in SIMD code (simdDeriveClassificationBlk
  // function)
#if ALF_IMPROVEMENT
  m_tempBuf.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, MAX_FILTER_LENGTH_FIXED, 0, false);
#else
  m_tempBuf.create(format, Area(0, 0, picWidth, picHeight), maxCUWidth, (MAX_ALF_FILTER_LENGTH + 1) >> 1, 0, false);
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  m_tempBufBeforeDb.destroy();
  m_tempBufBeforeDb.create(CHROMA_400, Area(0, 0, picWidth, picHeight), maxCUWidth, NUM_DB_PAD, 0, false);
  m_tempBufBeforeDb2.destroy();
  m_tempBufBeforeDb2.create(CHROMA_400, Area(0, 0, maxCUWidth + (NUM_DB_PAD << 1), maxCUHeight + (NUM_DB_PAD << 1)), maxCUWidth, NUM_DB_PAD, 0, false);
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_tempBufResi.destroy();
  m_tempBufResi.create(CHROMA_400, Area(0, 0, picWidth, picHeight), maxCUWidth, MAX_FILTER_LENGTH_FIXED, 0, false);
  m_tempBufResi2.destroy();
  m_tempBufResi2.create(CHROMA_400, Area(0, 0, maxCUWidth + (MAX_ALF_PADDING_SIZE << 1), maxCUHeight + (MAX_ALF_PADDING_SIZE << 1)), maxCUWidth, MAX_ALF_PADDING_SIZE, 0, false);
#endif
  m_tempBuf2.destroy();
  m_tempBuf2.create( format, Area( 0, 0, maxCUWidth + (MAX_ALF_PADDING_SIZE << 1), maxCUHeight + (MAX_ALF_PADDING_SIZE << 1) ), maxCUWidth, MAX_ALF_PADDING_SIZE, 0, false );

#if ALF_IMPROVEMENT
  int numFixedFilters = EXT_LENGTH << 1;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  m_isFixedFilterPaddedPerCtu = true;
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS << 1;
#endif
  //destroy
  if (m_fixFilterResult)
  {
    for (int i = 0; i < numFixedFilters; i++)
    {
      if (m_fixFilterResult[i])
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        for (int j = 0; j < m_picHeight + padSize; j++)
#else
        for (int j = 0; j < m_picHeight; j++)
#endif
        {
          if (m_fixFilterResult[i][j])
          {
            delete[] m_fixFilterResult[i][j];
            m_fixFilterResult[i][j] = nullptr;
          }
        }
        delete[] m_fixFilterResult[i];
        m_fixFilterResult[i] = nullptr;
      }
    }
    delete[] m_fixFilterResult;
    m_fixFilterResult = nullptr;
  }
  //create
  m_fixFilterResult = new Pel**[numFixedFilters];
  for (int i = 0; i < numFixedFilters; i++)
  {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    m_fixFilterResult[i] = new Pel*[picHeight + padSize];
    for (int j = 0; j < picHeight + padSize; j++)
    {
      m_fixFilterResult[i][j] = new Pel[picWidth + padSize];
    }
#else
    m_fixFilterResult[i] = new Pel*[picHeight];
    for (int j = 0; j < picHeight; j++)
    {
      m_fixFilterResult[i][j] = new Pel[picWidth];
    }
#endif
  }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  int numResiFixedFilters = EXT_LENGTH;
  // destroy
  if (m_fixFilterResiResult)
  {
    for (int i = 0; i < numResiFixedFilters; i++)
    {
      if (m_fixFilterResiResult[i])
      {
        // for( int j = 0; j < m_picWidth; j++ )
        for (int j = 0; j < m_picHeight; j++)
        {
          if (m_fixFilterResiResult[i][j])
          {
            delete[] m_fixFilterResiResult[i][j];
            m_fixFilterResiResult[i][j] = nullptr;
          }
        }
        delete[] m_fixFilterResiResult[i];
        m_fixFilterResiResult[i] = nullptr;
      }
    }
    delete[] m_fixFilterResiResult;
    m_fixFilterResiResult = nullptr;
  }
  // create
  m_fixFilterResiResult = new Pel **[numResiFixedFilters];
  for (int i = 0; i < numResiFixedFilters; i++)
  {
    m_fixFilterResiResult[i] = new Pel *[picHeight];
    for (int j = 0; j < picHeight; j++)
    {
      m_fixFilterResiResult[i][j] = new Pel[picWidth];
    }
  }
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  //destroy
  if (m_fixedFilterResultPerCtu)
  {
    for (int i = 0; i < numFixedFilters; i++)
    {
      if (m_fixedFilterResultPerCtu[i])
      {
        for (int j = 0; j < m_maxCUHeight + padSize; j++)
        {
          if (m_fixedFilterResultPerCtu[i][j])
          {
            delete[] m_fixedFilterResultPerCtu[i][j];
            m_fixedFilterResultPerCtu[i][j] = nullptr;
          }
        }
        delete[] m_fixedFilterResultPerCtu[i];
        m_fixedFilterResultPerCtu[i] = nullptr;
      }
    }
    delete[] m_fixedFilterResultPerCtu;
    m_fixedFilterResultPerCtu = nullptr;
  }
    //create
  m_fixedFilterResultPerCtu = new Pel**[numFixedFilters];
  for (int i = 0; i < numFixedFilters; i++)
  {
    m_fixedFilterResultPerCtu[i] = new Pel*[m_maxCUHeight + padSize];
    for (int j = 0; j < m_maxCUHeight + padSize; j++)
    {
      m_fixedFilterResultPerCtu[i][j] = new Pel[m_maxCUWidth + padSize];
    }
  }
  //destory
  if( m_ctuEnableOnlineLumaFlag )
  {
    delete[] m_ctuEnableOnlineLumaFlag;
    m_ctuEnableOnlineLumaFlag = nullptr;
  }
  //create
  m_ctuEnableOnlineLumaFlag = new uint8_t[m_numCTUsInPic];
  memset(m_ctuEnableOnlineLumaFlag, 0, sizeof(uint8_t) * m_numCTUsInPic);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  if( m_ctuPadFlag )
  {
    delete[] m_ctuPadFlag;
    m_ctuPadFlag = nullptr;
  }
  //create
  m_ctuPadFlag = new uint8_t[m_numCTUsInPic];
  memset( m_ctuPadFlag, 0, sizeof( uint8_t ) * m_numCTUsInPic );
#endif
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS << 1;
  //Delete
  if ( m_gaussPic )
  {
    for (int i = 0; i < NUM_GAUSS_FILTERED_SOURCE; i++)
    {
      if (m_gaussPic[i])
      {
        for (int j = 0; j < m_picHeight + padSizeGauss; j++)
        {
          if (m_gaussPic[i][j])
          {
            delete[] m_gaussPic[i][j];
            m_gaussPic[i][j] = nullptr;
          }
        }
        delete[] m_gaussPic[i];
        m_gaussPic[i] = nullptr;
      }
    }
    delete[] m_gaussPic;
    m_gaussPic = nullptr;
  }

  if ( m_gaussCtu )
  {
    for (int i = 0; i < NUM_GAUSS_FILTERED_SOURCE; i++)
    {
      if (m_gaussCtu[i])
      {
        for (int j = 0; j < m_maxCUHeight + padSizeGauss; j++)
        {
          if (m_gaussCtu[i][j])
          {
            delete[] m_gaussCtu[i][j];
            m_gaussCtu[i][j] = nullptr;
          }
        }
        delete[] m_gaussCtu[i];
        m_gaussCtu[i] = nullptr;
      }
    }
    delete[] m_gaussCtu;
    m_gaussCtu = nullptr;
  }
  //Create
  m_gaussPic = new Pel**[NUM_GAUSS_FILTERED_SOURCE];
  for (int i = 0; i < NUM_GAUSS_FILTERED_SOURCE; i++)
  {
    m_gaussPic[i] = new Pel*[picHeight + padSizeGauss];
    for (int j = 0; j < picHeight + padSizeGauss; j++)
    {
      m_gaussPic[i][j] = new Pel[picWidth + padSizeGauss];
    }
  }

  m_gaussCtu = new Pel**[NUM_GAUSS_FILTERED_SOURCE];
  for (int i = 0; i < NUM_GAUSS_FILTERED_SOURCE; i++)
  {
    m_gaussCtu[i] = new Pel*[m_maxCUHeight + padSizeGauss];
    for (int j = 0; j < m_maxCUHeight + padSizeGauss; j++)
    {
      m_gaussCtu[i][j] = new Pel[m_maxCUWidth + padSizeGauss];
    }
  }
#endif
#endif

  // Classification
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  for( int classifier = 0; classifier < ALF_NUM_CLASSIFIER + 1; classifier++ )
#else
  for( int classifier = 0; classifier < ALF_NUM_CLASSIFIER; classifier++ )
#endif
  {
    if( m_classifier[classifier] == nullptr )
    {
      m_classifier[classifier] = new AlfClassifier*[picHeight];
      m_classifier[classifier][0] = new AlfClassifier[picWidth * picHeight];

      for( int i = 1; i < picHeight; i++ )
      {
        m_classifier[classifier][i] = m_classifier[classifier][0] + i * picWidth;
      }
    }
  }
#else
  if ( m_classifier == nullptr )
  {
    m_classifier = new AlfClassifier*[picHeight];
    m_classifier[0] = new AlfClassifier[picWidth * picHeight];

    for (int i = 1; i < picHeight; i++)
    {
      m_classifier[i] = m_classifier[0] + i * picWidth;
    }
  }
#endif
#if !ALF_IMPROVEMENT
  for (int filterSetIndex = 0; filterSetIndex < NUM_FIXED_FILTER_SETS; filterSetIndex++)
  {
    for (int classIdx = 0; classIdx < MAX_NUM_ALF_CLASSES; classIdx++)
    {
      int fixedFilterIdx = m_classToFilterMapping[filterSetIndex][classIdx];
      for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF - 1; i++)
      {
       m_fixedFilterSetCoeffDec[filterSetIndex][classIdx * MAX_NUM_ALF_LUMA_COEFF + i] = m_fixedFilterSetCoeff[fixedFilterIdx][i];
      }
      m_fixedFilterSetCoeffDec[filterSetIndex][classIdx * MAX_NUM_ALF_LUMA_COEFF + MAX_NUM_ALF_LUMA_COEFF - 1] = (1 << (m_NUM_BITS - 1));
    }
  }
  for (int i = 0; i < MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES; i++)
  {
    m_clipDefault[i] = m_alfClippingValues[CHANNEL_TYPE_LUMA][0];
  }
#endif
  m_created = true;

  m_ccAlfFilterControl[0] = new uint8_t[m_numCTUsInPic];
  m_ccAlfFilterControl[1] = new uint8_t[m_numCTUsInPic];
}

void AdaptiveLoopFilter::destroy()
{
  if (!m_created)
  {
    return;
  }

#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  for (int classifier = 0; classifier < ALF_NUM_CLASSIFIER + 1; classifier++)
#else
  for (int classifier = 0; classifier < ALF_NUM_CLASSIFIER; classifier++)
#endif
  {
    if (m_classifier[classifier])
    {
      delete[] m_classifier[classifier][0];
      delete[] m_classifier[classifier];
      m_classifier[classifier] = nullptr;
    }
  }
#else
  if( m_classifier )
  {
    delete[] m_classifier[0];
    delete[] m_classifier;
    m_classifier = nullptr;
  }
#endif

#if ALF_IMPROVEMENT
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS << 1;
#endif
  if( m_fixFilterResult )
  {
    int numFixedFilters = EXT_LENGTH << 1;
    //for( int i = 0; i < m_picHeight; i++ )
    for( int i = 0; i < numFixedFilters; i++ )
    {
      if( m_fixFilterResult[i] )
      {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
        for( int j = 0; j < m_picHeight + padSize; j++ )
#else
        for( int j = 0; j < m_picHeight; j++ )
#endif
        {
          if( m_fixFilterResult[i][j] )
          {
            delete[] m_fixFilterResult[i][j];
            m_fixFilterResult[i][j] = nullptr;
          }
        }
        delete[] m_fixFilterResult[i];
        m_fixFilterResult[i] = nullptr;
      }
    }
    delete[] m_fixFilterResult;
    m_fixFilterResult = nullptr;
  }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  if (m_fixFilterResiResult)
  {
    int numResiFixedFilters = EXT_LENGTH;
    // for( int i = 0; i < m_picHeight; i++ )
    for (int i = 0; i < numResiFixedFilters; i++)
    {
      if (m_fixFilterResiResult[i])
      {
        // for( int j = 0; j < m_picWidth; j++ )
        for (int j = 0; j < m_picHeight; j++)
        {
          if (m_fixFilterResiResult[i][j])
          {
            delete[] m_fixFilterResiResult[i][j];
            m_fixFilterResiResult[i][j] = nullptr;
          }
        }
        delete[] m_fixFilterResiResult[i];
        m_fixFilterResiResult[i] = nullptr;
      }
    }
    delete[] m_fixFilterResiResult;
    m_fixFilterResiResult = nullptr;
  }
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  if( m_fixedFilterResultPerCtu )
  {
    int numFixedFilters = EXT_LENGTH << 1;
    for( int i = 0; i < numFixedFilters; i++ )
    {
      if( m_fixedFilterResultPerCtu[i] )
      {
        for( int j = 0; j < m_maxCUHeight + padSize; j++ )
        {
          if( m_fixedFilterResultPerCtu[i][j] )
          {
            delete[] m_fixedFilterResultPerCtu[i][j];
            m_fixedFilterResultPerCtu[i][j] = nullptr;
          }
        }
        delete[] m_fixedFilterResultPerCtu[i];
        m_fixedFilterResultPerCtu[i] = nullptr;
      }
    }
    delete[] m_fixedFilterResultPerCtu;
    m_fixedFilterResultPerCtu = nullptr;
  }

  if( m_ctuEnableOnlineLumaFlag )
  {
    delete[] m_ctuEnableOnlineLumaFlag;
    m_ctuEnableOnlineLumaFlag = nullptr;
  }
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  if( m_ctuPadFlag )
  {
    delete[] m_ctuPadFlag;
    m_ctuPadFlag = nullptr;
  }
#endif
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS << 1;
  if( m_gaussPic )
  {
    for( int i = 0; i < NUM_GAUSS_FILTERED_SOURCE; i++ )
    {
      if( m_gaussPic[i] )
      {
        for( int j = 0; j < m_picHeight + padSizeGauss; j++ )
        {
          if( m_gaussPic[i][j] )
          {
            delete[] m_gaussPic[i][j];
            m_gaussPic[i][j] = nullptr;
          }
        }
        delete[] m_gaussPic[i];
        m_gaussPic[i] = nullptr;
      }
    }
    delete[] m_gaussPic;
    m_gaussPic = nullptr;
  }

  if( m_gaussCtu )
  {
    for( int i = 0; i < NUM_GAUSS_FILTERED_SOURCE; i++ )
    {
      if( m_gaussCtu[i] )
      {
        for( int j = 0; j < m_maxCUHeight + padSizeGauss; j++ )
        {
          if( m_gaussCtu[i][j] )
          {
            delete[] m_gaussCtu[i][j];
            m_gaussCtu[i][j] = nullptr;
          }
        }
        delete[] m_gaussCtu[i];
        m_gaussCtu[i] = nullptr;
      }
    }
    delete[] m_gaussCtu;
    m_gaussCtu = nullptr;
  }
#endif
#endif

  m_tempBuf.destroy();
  m_tempBuf2.destroy();
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  m_tempBufBeforeDb.destroy();
  m_tempBufBeforeDb2.destroy();
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  m_tempBufResi.destroy();
  m_tempBufResi2.destroy();
#endif
  m_filterShapes[CHANNEL_TYPE_LUMA].clear();
  m_filterShapes[CHANNEL_TYPE_CHROMA].clear();
  m_created = false;

  m_filterShapesCcAlf.clear();
  if ( m_ccAlfFilterControl[0] )
  {
    delete [] m_ccAlfFilterControl[0];
    m_ccAlfFilterControl[0] = nullptr;
  }

  if ( m_ccAlfFilterControl[1] )
  {
    delete [] m_ccAlfFilterControl[1];
    m_ccAlfFilterControl[1] = nullptr;
  }
}

#if ALF_IMPROVEMENT
void  AdaptiveLoopFilter::copyFixedFilterResults( const PelUnitBuf &recDst, const Area &blkDst, ComponentID compId, Pel*** fixedFilterResults, const int fixedFilterSetIdx, const int classifierIdx )
{
  PelBuf dstLuma = recDst.get( compId );
  const int dstStride = dstLuma.stride;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride + blkDst.x;
  int filterIndex = classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
  for( int y = 0; y < blkDst.height; y++ )
  {
    for( int x = 0; x < blkDst.width; x++ )
    {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
      dst[x] = fixedFilterResults[filterIndex][y + blkDst.y + padSize][x + blkDst.x + padSize];
#else
      dst[x] = fixedFilterResults[filterIndex][y + blkDst.y][x + blkDst.x];
#endif
    }
    dst += dstStride;
  }
}

void  AdaptiveLoopFilter::alfFiltering( AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *fClipSet,
#else
  const short *fClipSet, 
#endif
  const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel*** fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  , Pel*** fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  , Pel*** gaussPic, Pel*** gaussCtu
#endif
    )
{
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  if (filterType == ALF_FILTER_9)
  {
    m_filter9x9Blk(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet,  fClipSet, clpRng, cs, nullptr, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                   , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                   , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_7)
  {
    m_filter7x7Blk(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                   , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                   , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_5)
  {
    m_filter5x5Blk(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                   , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                   , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_9_EXT)
  {
    m_filter9x9BlkExt(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                  , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                  , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_EXT)
  {
    m_filterBlkExt(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                 , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                 , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_9_EXT_DB)
  {
    m_filter9x9BlkExtDb(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                , gaussPic, gaussCtu
#endif
    );
  }
#if JVET_AA0095_ALF_LONGER_FILTER
  else if (filterType == ALF_FILTER_13_EXT)
  {
    m_filter13x13BlkExt(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_13_EXT_DB)
  {
    m_filter13x13BlkExtDb(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                , gaussPic, gaussCtu
#endif
    );
  }
#endif
  else if (filterType == ALF_FILTER_13_EXT_DB_RESI_DIRECT)
  {
    m_filter13x13BlkExtDbResiDirect(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
               , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
               , gaussPic, gaussCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_13_EXT_DB_RESI)
  {
    m_filter13x13BlkExtDbResi(classifier, recDst, recBeforeDb, resi, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterResiResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
              , gaussPic, gaussCtu
#endif
    );
  }
#else
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  if (filterType == ALF_FILTER_9)
  {
    m_filter9x9Blk( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_7)
  {
    m_filter7x7Blk( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_5)
  {
    m_filter5x5Blk( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_9_EXT)
  {
    m_filter9x9BlkExt( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_EXT)
  {
    m_filterBlkExt( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_9_EXT_DB)
  {
    m_filter9x9BlkExtDb( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
#if JVET_AA0095_ALF_LONGER_FILTER
  else if (filterType == ALF_FILTER_13_EXT)
  {
    m_filter13x13BlkExt( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_13_EXT_DB)
  {
    m_filter13x13BlkExtDb( classifier, recDst, recBeforeDb, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
#endif
#else
  if (filterType == ALF_FILTER_9)
  {
    m_filter9x9Blk( classifier, recDst, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_7)
  {
    m_filter7x7Blk( classifier, recDst, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_5)
  {
    m_filter5x5Blk( classifier, recDst, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, nullptr, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_9_EXT)
  {
    m_filter9x9BlkExt( classifier, recDst, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
  else if (filterType == ALF_FILTER_EXT)
  {
    m_filterBlkExt( classifier, recDst, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
#if JVET_AA0095_ALF_LONGER_FILTER
  else if (filterType == ALF_FILTER_13_EXT)
  {
    m_filter13x13BlkExt( classifier, recDst, recSrc, blkDst, blk, compId, filterSet, fClipSet, clpRng, cs, fixedFilterResults, fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
    , fixedFilterResultsPerCtu, isFixedFilterPaddedPerCtu
#endif
    );
  }
#endif
#endif
#endif
  else
  {
    CHECK(1, "unsupported filter type");
  }
}
#endif

#if JVET_X0071_ALF_BAND_CLASSIFIER
void AdaptiveLoopFilter::deriveClassification( AlfClassifier*** classifier, const CPelBuf& srcLuma, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                                              const bool bResiFixed, const CPelBuf &srcResiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  const CPelBuf& srcLumaBeforeDb, const uint8_t ctuPadFlag,
#endif
  const Area& blkDst, const Area& blk, CodingStructure &cs, const int classifierIdx, const int multipleClassifierIdx )
#else
void AdaptiveLoopFilter::deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk 
#if ALF_IMPROVEMENT
  , CodingStructure &cs, const int classifierIdx
#endif
)
#endif
{
#if ALF_IMPROVEMENT
  if( cs.slice->getCuQpDeltaSubdiv() )
  {
    UnitArea curArea( cs.area.chromaFormat, blkDst );
    for( auto &currCU : cs.traverseCUs( curArea, CH_L ) )
    {
#if JVET_X0071_ALF_BAND_CLASSIFIER
      deriveClassificationAndFixFilterResultsBlk( classifier, m_fixFilterResult, srcLuma, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
        bResiFixed, m_fixFilterResiResult, srcResiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
        srcLumaBeforeDb, 0,
#endif
        Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], currCU.qp, cs.slice->getTileGroupAlfFixedFilterSetIdx(), m_mappingDir, m_laplacian, classifierIdx, multipleClassifierIdx );
#else
      deriveClassificationAndFixFilterResultsBlk( classifier, m_fixFilterResult, srcLuma, Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], currCU.qp, cs.slice->getTileGroupAlfFixedFilterSetIdx(), m_mappingDir, m_laplacian, classifierIdx );
#endif
    }
  }
  else
  {
    int height = blk.pos().y + blk.height;
    int width = blk.pos().x + blk.width;
    for( int i = blk.pos().y; i < height; i += m_CLASSIFICATION_BLK_SIZE )
    {
      int nHeight = std::min( i + m_CLASSIFICATION_BLK_SIZE, height ) - i;

      for( int j = blk.pos().x; j < width; j += m_CLASSIFICATION_BLK_SIZE )
      {
        int nWidth = std::min( j + m_CLASSIFICATION_BLK_SIZE, width ) - j;
#if JVET_X0071_ALF_BAND_CLASSIFIER
        deriveClassificationAndFixFilterResultsBlk(classifier, m_fixFilterResult, srcLuma, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
          bResiFixed, m_fixFilterResiResult, srcResiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          srcLumaBeforeDb, ( j == blk.pos().x && i == blk.pos().y ) ? ctuPadFlag : 0,
#endif
          Area(j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight), Area(j, i, nWidth, nHeight), m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), cs.slice->getTileGroupAlfFixedFilterSetIdx(), m_mappingDir, m_laplacian, classifierIdx, multipleClassifierIdx );
#else
        deriveClassificationAndFixFilterResultsBlk( classifier, m_fixFilterResult, srcLuma, Area(j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight), Area(j, i, nWidth, nHeight), m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), cs.slice->getTileGroupAlfFixedFilterSetIdx(), m_mappingDir, m_laplacian, classifierIdx );
#endif
      }
    }
  }
#else
  int height = blk.pos().y + blk.height;
  int width = blk.pos().x + blk.width;

  for( int i = blk.pos().y; i < height; i += m_CLASSIFICATION_BLK_SIZE )
  {
    int nHeight = std::min( i + m_CLASSIFICATION_BLK_SIZE, height ) - i;

    for( int j = blk.pos().x; j < width; j += m_CLASSIFICATION_BLK_SIZE )
    {
      int nWidth = std::min( j + m_CLASSIFICATION_BLK_SIZE, width ) - j;
      m_deriveClassificationBlk(classifier, m_laplacian, srcLuma, Area( j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight ), Area(j, i, nWidth, nHeight), m_inputBitDepth[CHANNEL_TYPE_LUMA] + 4, m_alfVBLumaCTUHeight, m_alfVBLumaPos);
    }
  }
#endif
}

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
void AdaptiveLoopFilter::calcClass0Var( AlfClassifier **classifier, const Area &blkDst, const Area &curBlk, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS] )
{
  int shift = 9 + bitDepth;
  const int multTab[] = { 5628, 1407, 624, 351, 225, 156 };
  int mult = multTab[dirWindSize];

  const int divShift2[16] = { 2, 2, 4, 4, 6, 6, 6, 6,  8,  8,  8,  8, 10, 10, 10, 10 };
  int noDirAll = NUM_DIR_FIX * ( NUM_DIR_FIX + 1 );
  int noDirActAll = noDirAll * NUM_ACT_FIX;
  const int sqrtSum[50] = { 0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7 };

  for( int i = 0; i < curBlk.height; i += subBlkSize )
  {
    for( int j = 0; j < curBlk.width; j += subBlkSize )
    {
      int sum = 0, sumV = 0, sumH = 0, sumD0 = 0, sumD1 = 0;
      if( dirWindSize == 1 )
      {
        int yOffset = ( i >> 1 ) + 2;
        int xOffset = ( j >> 1 ) + 2;
        sumV = laplacian[VER][yOffset][xOffset];
        sumH = laplacian[HOR][yOffset][xOffset];
        sumD0 = laplacian[DIAG0][yOffset][xOffset];
        sumD1 = laplacian[DIAG1][yOffset][xOffset];
      }
      else if( dirWindSize == 5 )
      {
        int yOffset = i >> 1;
        int xOffset = j >> 1;
        sumV = laplacian[VER][yOffset][xOffset];
        sumH = laplacian[HOR][yOffset][xOffset];
        sumD0 = laplacian[DIAG0][yOffset][xOffset];
        sumD1 = laplacian[DIAG1][yOffset][xOffset];
      }
      else
      {
        CHECK( 1, "other window sizes are not supported" );
      }
      sum += sumV + sumH;
      int direction = 0, mainDirection, secondaryDirection, dirTempHV = 0, dirTempD = 0, hv1, hv0, d1, d0, hvd1, hvd0;
      if( sumV > sumH )
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }
      if( sumD0 > sumD1 )
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }

      if( ( uint64_t )d1 * hv0 > ( uint64_t )d0 * hv1 )
      {
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
        hvd1 = d1;
        hvd0 = d0;
      }
      else
      {
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
        hvd1 = hv1;
        hvd0 = hv0;
      }
      //activity
      int activity = 0;
      if( noAct == 5 )
      {
        const int th[] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
#if JVET_X0071_ALF_BAND_CLASSIFIER
        activity = th[std::min( ( mult * 3 * sum ) >> shift, 15 )];
#else
        activity = th[std::min( (mult * sum ) >> shift, 15 )];
#endif
      }
      else
      {
        const int th[193] = { 0,  1,  2,  3,  4,  4,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15 };
        activity = th[std::min( ( mult * sum ) >> ( shift - 4 ), 192 )];
      }

      //direction
      if( classDir == 0 )
      {
        int directionStrength = 0;
        if( hvd1 * 2 > 9 * hvd0 )
        {
          directionStrength = 2;
        }
        else if( hvd1 > 2 * hvd0 )
        {
          directionStrength = 1;
        }

        if( directionStrength )
        {
          direction = ( ( mainDirection & 0x1 ) << 1 ) + directionStrength;
        }
      }
      else
      {
        int edgeStrengthHV = 0, edgeStrengthD = 0;
        if( hv1 > 8 * hv0 )
        {
          edgeStrengthHV = 6;
        }
        else if( hv1 * 2 > 9 * hv0 )
        {
          edgeStrengthHV = 5;
        }
        else if( hv1 > 3 * hv0 )
        {
          edgeStrengthHV = 4;
        }
        else if (hv1 > 2 * hv0)
        {
          edgeStrengthHV = 3;
        }
        else if( hv1 * 2 > 3 * hv0 )
        {
          edgeStrengthHV = 2;
        }
        else if( hv1 * 4 > 5 * hv0 )
        {
          edgeStrengthHV = 1;
        }

        if( d1 > 8 * d0 )
        {
          edgeStrengthD = 6;
        }
        else if( d1 * 2 > 9 * d0 )
        {
          edgeStrengthD = 5;
        }
        else if( d1 > 3 * d0 )
        {
          edgeStrengthD = 4;
        }
        else if( d1 > 2 * d0 )
        {
          edgeStrengthD = 3;
        }
        else if( d1 * 2 > 3 * d0 )
        {
          edgeStrengthD = 2;
        }
        else if( d1 * 4 > 5 * d0 )
        {
          edgeStrengthD = 1;
        }

        if( ( uint64_t )hv1 * d0 >  ( uint64_t )hv0 * d1 )
        {
          direction = mappingDir[edgeStrengthHV][edgeStrengthD];
        }
        else
        {
          direction = 28 + mappingDir[edgeStrengthD][edgeStrengthHV];
        }
      }

      int actDirInd = 0;
      if( classDir == 0 )
      {
        actDirInd = noDir * activity + direction;
      }
      else
      {
        actDirInd = noDir * ( noDir + 1 ) * activity + direction;
      }

      uint32_t sum2 = laplacian[VARIANCE][i >> 1][j >> 1] >> divShift2[activity];
      if( sum2 > 49 )
      {
        sum2 = 49;
      }

      actDirInd = sqrtSum[sum2] * noDirActAll + actDirInd;

      const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + ( secondaryDirection >> 1 )];

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int posYDst = blkDst.pos().y;
      int posXDst = blkDst.pos().x;

      for( int ii = posYDst + i; ii < posYDst + i + subBlkSize; ii++ )
      {
        for( int jj = posXDst + j; jj < posXDst + j + subBlkSize; jj++ )
        {
#else
      for( int ii = curBlk.y + i; ii < curBlk.y + i + subBlkSize; ii++ )
      {
        for( int jj = curBlk.x + j; jj < curBlk.x + j + subBlkSize; jj++ )
        {
#endif
          classifier[ii][jj] = (actDirInd << 2) + transposeIdx;
        }
      }
    }
  }
}

void AdaptiveLoopFilter::alfFixedFilterBlkNonSimd( AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4] )
{
  if( dirWindSize == 0 )
  { 
    fixedFilterBlk<ALF_FIXED_FILTER_9_DB_9>( classifier, srcLuma, curBlk, blkDst, srcLumaBeforeDb, fixedFilterResults, picWidth, fixedFiltInd, m_classIdnFixedFilter9Db9[fixedFiltQpInd], fixedFiltQpInd, dirWindSize, clpRng, clippingValues );
  }
  else
  {
    fixedFilterBlk<ALF_FIXED_FILTER_13_DB_9>( classifier, srcLuma, curBlk, blkDst, srcLumaBeforeDb, fixedFilterResults, picWidth, fixedFiltInd, m_classIdnFixedFilter13Db9[fixedFiltQpInd], fixedFiltQpInd, dirWindSize, clpRng, clippingValues );
  }
}
void AdaptiveLoopFilter::alfFixedFilterBlk( AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4] )
{
  if( dirWindSize == 0 )
  { 
    m_fixFilter9x9Db9Blk( classifier, srcLuma, curBlk, blkDst, srcLumaBeforeDb, fixedFilterResults, picWidth, fixedFiltInd, m_classIdnFixedFilter9Db9[fixedFiltQpInd], fixedFiltQpInd, dirWindSize, clpRng, clippingValues );
  }
  else
  {
    m_fixFilter13x13Db9Blk( classifier, srcLuma, curBlk, blkDst, srcLumaBeforeDb, fixedFilterResults, picWidth, fixedFiltInd, m_classIdnFixedFilter13Db9[fixedFiltQpInd], fixedFiltQpInd, dirWindSize, clpRng, clippingValues );
  }
}

template<AlfFixedFilterType filtType>
void AdaptiveLoopFilter::fixedFilterBlk( AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk, const Area &blkDst, const CPelBuf &srcLumaBeforeDb, Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd, const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4] )
{
  const int shift = m_NUM_BITS_FIXED_FILTER - 1;
  const int offset = 1 << (shift - 1);
  const int srcStride = srcLuma.stride;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int posYDst = blkDst.y;
  int posXDst = blkDst.x;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif

  int posY = curBlk.y;
  int posX = curBlk.x;
  const Pel *pImgYPad0 = srcLuma.buf + posY * srcStride + posX;

  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10, *pImg11, *pImg12;
  const int clsSizeY = 2;
  const int clsSizeX = 2;
  const int srcStride2 = srcStride * clsSizeY;
  const Pel *pImg0BeforeDb, *pImg1BeforeDb, *pImg2BeforeDb, *pImg3BeforeDb, *pImg4BeforeDb, *pImg5BeforeDb, *pImg6BeforeDb, *pImg7BeforeDb, *pImg8BeforeDb;
  const int srcBeforeDbStride = srcLumaBeforeDb.stride;
  const Pel *pImgYBeforeDbPad0 = srcLumaBeforeDb.buf + posY * srcBeforeDbStride + posX;
  const int srcBeforeDbStride2 = srcBeforeDbStride * clsSizeY;
  int fixedFiltIndF0 = -1;

  int numCoeff;
  if( filtType == ALF_FIXED_FILTER_13_DB_9 )
  {
    numCoeff = FIX_FILTER_NUM_COEFF_13_DB_9;
    fixedFiltIndF0 = fixedFiltInd > 1 ? fixedFiltInd - EXT_LENGTH : -1;
  }
  else if( filtType == ALF_FIXED_FILTER_9_DB_9 )
  {
    numCoeff = FIX_FILTER_NUM_COEFF_9_DB_9;
  }
  else
  {
    CHECK( 1, "not supported" );
  }

  std::vector<short> filterCoeff( numCoeff, 0 );
  std::vector<short> filterClipp( numCoeff, 0 );
    
  for( int i = 0; i < curBlk.height; i += clsSizeY )
  {
    for( int j = 0; j < curBlk.width; j += clsSizeX )
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int classIdx = classifier[posYDst + i][posXDst + j] >> 2;
      int transposeIdx = classifier[posYDst + i][posXDst + j] & 0x3;
#else
      int classIdx = classifier[posY + i][posX + j] >> 2;
      int transposeIdx = classifier[posY + i][posX + j] & 0x3;
#endif
      int filterIdx = classIndFixed[classIdx];      

      const short* coeff;
      const short* clipp;

      if( filtType == ALF_FIXED_FILTER_13_DB_9 )
      {
        coeff = m_filterCoeffFixed13Db9[fixedFiltQpInd][filterIdx];
        clipp = m_clippingFixed13Db9[fixedFiltQpInd][filterIdx];
        if( transposeIdx == 0 )
        {
          filterCoeff = { coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8], coeff[9], coeff[10], coeff[11], coeff[12], coeff[13], coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27], coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34], coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40], coeff[41], coeff[42], coeff[43], coeff[44], coeff[45], coeff[46], coeff[47], coeff[48], coeff[49], coeff[50], coeff[51], coeff[52], coeff[53], coeff[54], coeff[55], coeff[56], coeff[57], coeff[58], coeff[59], coeff[60], coeff[61], coeff[62], coeff[63] };
          filterClipp = { clipp[0], clipp[1], clipp[2], clipp[3], clipp[4], clipp[5], clipp[6], clipp[7], clipp[8], clipp[9], clipp[10], clipp[11], clipp[12], clipp[13], clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27], clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34], clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40], clipp[41], clipp[42], clipp[43], clipp[44], clipp[45], clipp[46], clipp[47], clipp[48], clipp[49], clipp[50], clipp[51], clipp[52], clipp[53], clipp[54], clipp[55], clipp[56], clipp[57], clipp[58], clipp[59], clipp[60], clipp[61], clipp[62], clipp[63] };
        }
        else if (transposeIdx == 1)
        {
          filterCoeff = { coeff[36], coeff[25], coeff[37], coeff[35], coeff[16], coeff[26], coeff[38], coeff[34], coeff[24], coeff[9], coeff[17], coeff[27], coeff[39], coeff[33], coeff[23], coeff[15], coeff[4], coeff[10], coeff[18], coeff[28], coeff[40], coeff[32], coeff[22], coeff[14], coeff[8], coeff[1], coeff[5], coeff[11], coeff[19], coeff[29], coeff[41], coeff[31], coeff[21], coeff[13], coeff[7], coeff[3], coeff[0], coeff[2], coeff[6], coeff[12], coeff[20], coeff[30], coeff[58], coeff[51], coeff[59], coeff[57], coeff[46], coeff[52], coeff[60], coeff[56], coeff[50], coeff[43], coeff[47], coeff[53], coeff[61], coeff[55], coeff[49], coeff[45], coeff[42], coeff[44], coeff[48], coeff[54], coeff[62], coeff[63] };
          filterClipp = { clipp[36], clipp[25], clipp[37], clipp[35], clipp[16], clipp[26], clipp[38], clipp[34], clipp[24], clipp[9], clipp[17], clipp[27], clipp[39], clipp[33], clipp[23], clipp[15], clipp[4], clipp[10], clipp[18], clipp[28], clipp[40], clipp[32], clipp[22], clipp[14], clipp[8], clipp[1], clipp[5], clipp[11], clipp[19], clipp[29], clipp[41], clipp[31], clipp[21], clipp[13], clipp[7], clipp[3], clipp[0], clipp[2], clipp[6], clipp[12], clipp[20], clipp[30], clipp[58], clipp[51], clipp[59], clipp[57], clipp[46], clipp[52], clipp[60], clipp[56], clipp[50], clipp[43], clipp[47], clipp[53], clipp[61], clipp[55], clipp[49], clipp[45], clipp[42], clipp[44], clipp[48], clipp[54], clipp[62], clipp[63] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coeff[0], coeff[3], coeff[2], coeff[1], coeff[8], coeff[7], coeff[6], coeff[5], coeff[4], coeff[15], coeff[14], coeff[13], coeff[12], coeff[11], coeff[10], coeff[9], coeff[24], coeff[23], coeff[22], coeff[21], coeff[20], coeff[19], coeff[18], coeff[17], coeff[16], coeff[35], coeff[34], coeff[33], coeff[32], coeff[31], coeff[30], coeff[29], coeff[28], coeff[27], coeff[26], coeff[25], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40], coeff[41], coeff[42], coeff[45], coeff[44], coeff[43], coeff[50], coeff[49], coeff[48], coeff[47], coeff[46], coeff[57], coeff[56], coeff[55], coeff[54], coeff[53], coeff[52], coeff[51], coeff[58], coeff[59], coeff[60], coeff[61], coeff[62], coeff[63] };
          filterClipp = { clipp[0], clipp[3], clipp[2], clipp[1], clipp[8], clipp[7], clipp[6], clipp[5], clipp[4], clipp[15], clipp[14], clipp[13], clipp[12], clipp[11], clipp[10], clipp[9], clipp[24], clipp[23], clipp[22], clipp[21], clipp[20], clipp[19], clipp[18], clipp[17], clipp[16], clipp[35], clipp[34], clipp[33], clipp[32], clipp[31], clipp[30], clipp[29], clipp[28], clipp[27], clipp[26], clipp[25], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40], clipp[41], clipp[42], clipp[45], clipp[44], clipp[43], clipp[50], clipp[49], clipp[48], clipp[47], clipp[46], clipp[57], clipp[56], clipp[55], clipp[54], clipp[53], clipp[52], clipp[51], clipp[58], clipp[59], clipp[60], clipp[61], clipp[62], clipp[63] };
        }
        else
        {
          filterCoeff = { coeff[36], coeff[35], coeff[37], coeff[25], coeff[24], coeff[34], coeff[38], coeff[26], coeff[16], coeff[15], coeff[23], coeff[33], coeff[39], coeff[27], coeff[17], coeff[9], coeff[8], coeff[14], coeff[22], coeff[32], coeff[40], coeff[28], coeff[18], coeff[10], coeff[4], coeff[3], coeff[7], coeff[13], coeff[21], coeff[31], coeff[41], coeff[29], coeff[19], coeff[11], coeff[5], coeff[1], coeff[0], coeff[2], coeff[6], coeff[12], coeff[20], coeff[30], coeff[58], coeff[57], coeff[59], coeff[51], coeff[50], coeff[56], coeff[60], coeff[52], coeff[46], coeff[45], coeff[49], coeff[55], coeff[61], coeff[53], coeff[47], coeff[43], coeff[42], coeff[44], coeff[48], coeff[54], coeff[62], coeff[63] };
          filterClipp = { clipp[36], clipp[35], clipp[37], clipp[25], clipp[24], clipp[34], clipp[38], clipp[26], clipp[16], clipp[15], clipp[23], clipp[33], clipp[39], clipp[27], clipp[17], clipp[9], clipp[8], clipp[14], clipp[22], clipp[32], clipp[40], clipp[28], clipp[18], clipp[10], clipp[4], clipp[3], clipp[7], clipp[13], clipp[21], clipp[31], clipp[41], clipp[29], clipp[19], clipp[11], clipp[5], clipp[1], clipp[0], clipp[2], clipp[6], clipp[12], clipp[20], clipp[30], clipp[58], clipp[57], clipp[59], clipp[51], clipp[50], clipp[56], clipp[60], clipp[52], clipp[46], clipp[45], clipp[49], clipp[55], clipp[61], clipp[53], clipp[47], clipp[43], clipp[42], clipp[44], clipp[48], clipp[54], clipp[62], clipp[63] };
        }
      }
      else if( filtType == ALF_FIXED_FILTER_9_DB_9 )
      {
        coeff = m_filterCoeffFixed9Db9[fixedFiltQpInd][filterIdx];
        clipp = m_clippingFixed9Db9[fixedFiltQpInd][filterIdx];
        if( transposeIdx == 0 )
        {
          filterCoeff = { coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8], coeff[9], coeff[10], coeff[11], coeff[12], coeff[13], coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27], coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34], coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
          filterClipp = { clipp[0], clipp[1], clipp[2], clipp[3], clipp[4], clipp[5], clipp[6], clipp[7], clipp[8], clipp[9], clipp[10], clipp[11], clipp[12], clipp[13], clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27], clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34], clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
        }
        else if( transposeIdx == 1 )
        {
          filterCoeff = { coeff[16], coeff[9], coeff[17], coeff[15], coeff[4], coeff[10], coeff[18], coeff[14], coeff[8], coeff[1], coeff[5], coeff[11], coeff[19], coeff[13], coeff[7], coeff[3], coeff[0], coeff[2], coeff[6], coeff[12], coeff[36], coeff[29], coeff[37], coeff[35], coeff[24], coeff[30], coeff[38], coeff[34], coeff[28], coeff[21], coeff[25], coeff[31], coeff[39], coeff[33], coeff[27], coeff[23], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
          filterClipp = { clipp[16], clipp[9], clipp[17], clipp[15], clipp[4], clipp[10], clipp[18], clipp[14], clipp[8], clipp[1], clipp[5], clipp[11], clipp[19], clipp[13], clipp[7], clipp[3], clipp[0], clipp[2], clipp[6], clipp[12],  clipp[36], clipp[29], clipp[37], clipp[35], clipp[24], clipp[30], clipp[38], clipp[34], clipp[28], clipp[21], clipp[25], clipp[31], clipp[39], clipp[33], clipp[27], clipp[23], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coeff[0], coeff[3], coeff[2], coeff[1], coeff[8], coeff[7], coeff[6], coeff[5], coeff[4], coeff[15], coeff[14] , coeff[13], coeff[12], coeff[11], coeff[10] , coeff[9], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[23], coeff[22], coeff[21], coeff[28], coeff[27], coeff[26], coeff[25], coeff[24], coeff[35], coeff[34] , coeff[33], coeff[32], coeff[31], coeff[30] , coeff[29], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
          filterClipp = { clipp[0], clipp[3], clipp[2], clipp[1], clipp[8], clipp[7], clipp[6], clipp[5], clipp[4], clipp[15], clipp[14] , clipp[13], clipp[12], clipp[11], clipp[10] , clipp[9], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[23], clipp[22], clipp[21], clipp[28], clipp[27], clipp[26], clipp[25], clipp[24], clipp[35], clipp[34] , clipp[33], clipp[32], clipp[31], clipp[30] , clipp[29], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
        }
        else
        {
          filterCoeff = { coeff[16], coeff[15], coeff[17], coeff[9], coeff[8], coeff[14], coeff[18], coeff[10], coeff[4], coeff[3], coeff[7], coeff[13], coeff[19], coeff[11], coeff[5], coeff[1], coeff[0], coeff[2], coeff[6], coeff[12], coeff[36], coeff[35], coeff[37], coeff[29], coeff[28], coeff[34], coeff[38], coeff[30], coeff[24], coeff[23], coeff[27], coeff[33], coeff[39], coeff[31], coeff[25], coeff[21], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
          filterClipp = { clipp[16], clipp[15], clipp[17], clipp[9], clipp[8], clipp[14], clipp[18], clipp[10], clipp[4], clipp[3], clipp[7], clipp[13], clipp[19], clipp[11], clipp[5], clipp[1], clipp[0], clipp[2], clipp[6], clipp[12], clipp[36], clipp[35], clipp[37], clipp[29], clipp[28], clipp[34], clipp[38], clipp[30], clipp[24], clipp[23], clipp[27], clipp[33], clipp[39], clipp[31], clipp[25], clipp[21], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
        }                  
      }
      else
      {
        CHECK( 1, "not supported" );
      }


      for( int k = 0; k < numCoeff; k++ )
      {
        filterClipp[k] = clippingValues[filterClipp[k]];
      }

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        const Pel* pImg0Cur = pImgYPad0 + ii * srcStride + j;
        pImg0BeforeDb = pImgYBeforeDbPad0 + ii * srcBeforeDbStride + j;
        for( int jj = 0; jj < clsSizeX; jj++ )
        {
          Pel curr = pImg0Cur[0];
          if( fixedFiltIndF0 >= 0 )
          {
            int curPosY = posYDst + ii + i + padSize;
            int curPosX = posXDst + jj + j + padSize;

            pImg0 = &fixedFilterResults[fixedFiltIndF0][curPosY][curPosX];
            pImg1 = &fixedFilterResults[fixedFiltIndF0][curPosY - 1][curPosX];
            pImg3 = &fixedFilterResults[fixedFiltIndF0][curPosY - 2][curPosX];
            pImg5 = &fixedFilterResults[fixedFiltIndF0][curPosY - 3][curPosX];
            pImg7 = &fixedFilterResults[fixedFiltIndF0][curPosY - 4][curPosX];
            pImg9 = &fixedFilterResults[fixedFiltIndF0][curPosY - 5][curPosX];
            pImg11 = &fixedFilterResults[fixedFiltIndF0][curPosY - 6][curPosX];
            pImg2 = &fixedFilterResults[fixedFiltIndF0][curPosY + 1][curPosX];
            pImg4 = &fixedFilterResults[fixedFiltIndF0][curPosY + 2][curPosX];
            pImg6 = &fixedFilterResults[fixedFiltIndF0][curPosY + 3][curPosX];
            pImg8 = &fixedFilterResults[fixedFiltIndF0][curPosY + 4][curPosX];
            pImg10 = &fixedFilterResults[fixedFiltIndF0][curPosY + 5][curPosX];
            pImg12 = &fixedFilterResults[fixedFiltIndF0][curPosY + 6][curPosX];
          }
          else
          {
            pImg0 = pImg0Cur;
            pImg1 = pImg0 - srcStride;
            pImg3 = pImg1 - srcStride;
            pImg5 = pImg3 - srcStride;
            pImg7 = pImg5 - srcStride;
            pImg9 = pImg7 - srcStride;
            pImg11 = pImg9 - srcStride;
            pImg2 = pImg0 + srcStride;
            pImg4 = pImg2 + srcStride;
            pImg6 = pImg4 + srcStride;
            pImg8 = pImg6 + srcStride;
            pImg10 = pImg8 + srcStride;
            pImg12 = pImg10 + srcStride;
          }

          pImg1BeforeDb = pImg0BeforeDb - srcBeforeDbStride;
          pImg3BeforeDb = pImg1BeforeDb - srcBeforeDbStride;
          pImg5BeforeDb = pImg3BeforeDb - srcBeforeDbStride;
          pImg7BeforeDb = pImg5BeforeDb - srcBeforeDbStride;
          pImg2BeforeDb = pImg0BeforeDb + srcBeforeDbStride;
          pImg4BeforeDb = pImg2BeforeDb + srcBeforeDbStride;
          pImg6BeforeDb = pImg4BeforeDb + srcBeforeDbStride;
          pImg8BeforeDb = pImg6BeforeDb + srcBeforeDbStride;

          int sum = 0;
          if( filtType == ALF_FIXED_FILTER_13_DB_9 )
          {
            sum += filterCoeff[0] * ( clipALF( filterClipp[0], curr, pImg11[+0], pImg12[+0] ) );

            sum += filterCoeff[1] * ( clipALF( filterClipp[1], curr, pImg9[-1], pImg10[+1] ) );
            sum += filterCoeff[2] * ( clipALF( filterClipp[2], curr, pImg9[+0], pImg10[+0] ) );
            sum += filterCoeff[3] * ( clipALF( filterClipp[3], curr, pImg9[+1], pImg10[-1] ) );

            sum += filterCoeff[4] * ( clipALF( filterClipp[4], curr, pImg7[-2], pImg8[+2] ) );
            sum += filterCoeff[5] * ( clipALF( filterClipp[5], curr, pImg7[-1], pImg8[+1] ) );
            sum += filterCoeff[6] * ( clipALF( filterClipp[6], curr, pImg7[+0], pImg8[+0] ) );
            sum += filterCoeff[7] * ( clipALF( filterClipp[7], curr, pImg7[+1], pImg8[-1] ) );
            sum += filterCoeff[8] * ( clipALF( filterClipp[8], curr, pImg7[+2], pImg8[-2] ) );

            sum += filterCoeff[9]  * ( clipALF( filterClipp[9] , curr, pImg5[-3], pImg6[+3] ) );
            sum += filterCoeff[10] * ( clipALF( filterClipp[10], curr, pImg5[-2], pImg6[+2] ) );
            sum += filterCoeff[11] * ( clipALF( filterClipp[11], curr, pImg5[-1], pImg6[+1] ) );
            sum += filterCoeff[12] * ( clipALF( filterClipp[12], curr, pImg5[+0], pImg6[+0] ) );
            sum += filterCoeff[13] * ( clipALF( filterClipp[13], curr, pImg5[+1], pImg6[-1] ) );
            sum += filterCoeff[14] * ( clipALF( filterClipp[14], curr, pImg5[+2], pImg6[-2] ) );
            sum += filterCoeff[15] * ( clipALF( filterClipp[15], curr, pImg5[+3], pImg6[-3] ) );

            sum += filterCoeff[16] * ( clipALF( filterClipp[16], curr, pImg3[-4], pImg4[+4] ) );
            sum += filterCoeff[17] * ( clipALF( filterClipp[17], curr, pImg3[-3], pImg4[+3] ) );
            sum += filterCoeff[18] * ( clipALF( filterClipp[18], curr, pImg3[-2], pImg4[+2] ) );
            sum += filterCoeff[19] * ( clipALF( filterClipp[19], curr, pImg3[-1], pImg4[+1] ) );
            sum += filterCoeff[20] * ( clipALF( filterClipp[20], curr, pImg3[+0], pImg4[+0] ) );
            sum += filterCoeff[21] * ( clipALF( filterClipp[21], curr, pImg3[+1], pImg4[-1] ) );
            sum += filterCoeff[22] * ( clipALF( filterClipp[22], curr, pImg3[+2], pImg4[-2] ) );
            sum += filterCoeff[23] * ( clipALF( filterClipp[23], curr, pImg3[+3], pImg4[-3] ) );
            sum += filterCoeff[24] * ( clipALF( filterClipp[24], curr, pImg3[+4], pImg4[-4] ) );

            sum += filterCoeff[25] * ( clipALF( filterClipp[25], curr, pImg1[-5], pImg2[+5] ) );
            sum += filterCoeff[26] * ( clipALF( filterClipp[26], curr, pImg1[-4], pImg2[+4] ) );
            sum += filterCoeff[27] * ( clipALF( filterClipp[27], curr, pImg1[-3], pImg2[+3] ) );
            sum += filterCoeff[28] * ( clipALF( filterClipp[28], curr, pImg1[-2], pImg2[+2] ) );
            sum += filterCoeff[29] * ( clipALF( filterClipp[29], curr, pImg1[-1], pImg2[+1] ) );
            sum += filterCoeff[30] * ( clipALF( filterClipp[30], curr, pImg1[+0], pImg2[+0] ) );
            sum += filterCoeff[31] * ( clipALF( filterClipp[31], curr, pImg1[+1], pImg2[-1] ) );
            sum += filterCoeff[32] * ( clipALF( filterClipp[32], curr, pImg1[+2], pImg2[-2] ) );
            sum += filterCoeff[33] * ( clipALF( filterClipp[33], curr, pImg1[+3], pImg2[-3] ) );
            sum += filterCoeff[34] * ( clipALF( filterClipp[34], curr, pImg1[+4], pImg2[-4] ) );
            sum += filterCoeff[35] * ( clipALF( filterClipp[35], curr, pImg1[+5], pImg2[-5] ) );

            sum += filterCoeff[36] * ( clipALF( filterClipp[36], curr, pImg0[-6], pImg0[+6] ) );
            sum += filterCoeff[37] * ( clipALF( filterClipp[37], curr, pImg0[-5], pImg0[+5] ) );
            sum += filterCoeff[38] * ( clipALF( filterClipp[38], curr, pImg0[-4], pImg0[+4] ) );
            sum += filterCoeff[39] * ( clipALF( filterClipp[39], curr, pImg0[-3], pImg0[+3] ) );
            sum += filterCoeff[40] * ( clipALF( filterClipp[40], curr, pImg0[-2], pImg0[+2] ) );
            sum += filterCoeff[41] * ( clipALF( filterClipp[41], curr, pImg0[-1], pImg0[+1] ) );

            sum += filterCoeff[42] * ( clipALF( filterClipp[42], curr, pImg7BeforeDb[+0], pImg8BeforeDb[+0] ) );

            sum += filterCoeff[43] * ( clipALF( filterClipp[43], curr, pImg5BeforeDb[-1], pImg6BeforeDb[+1] ) );
            sum += filterCoeff[44] * ( clipALF( filterClipp[44], curr, pImg5BeforeDb[+0], pImg6BeforeDb[+0] ) );
            sum += filterCoeff[45] * ( clipALF( filterClipp[45], curr, pImg5BeforeDb[+1], pImg6BeforeDb[-1] ) );

            sum += filterCoeff[46] * ( clipALF( filterClipp[46], curr, pImg3BeforeDb[-2], pImg4BeforeDb[+2] ) );
            sum += filterCoeff[47] * ( clipALF( filterClipp[47], curr, pImg3BeforeDb[-1], pImg4BeforeDb[+1] ) );
            sum += filterCoeff[48] * ( clipALF( filterClipp[48], curr, pImg3BeforeDb[+0], pImg4BeforeDb[+0] ) );
            sum += filterCoeff[49] * ( clipALF( filterClipp[49], curr, pImg3BeforeDb[+1], pImg4BeforeDb[-1] ) );
            sum += filterCoeff[50] * ( clipALF( filterClipp[50], curr, pImg3BeforeDb[+2], pImg4BeforeDb[-2] ) );

            sum += filterCoeff[51] * ( clipALF( filterClipp[51], curr, pImg1BeforeDb[-3], pImg2BeforeDb[+3] ) );
            sum += filterCoeff[52] * ( clipALF( filterClipp[52], curr, pImg1BeforeDb[-2], pImg2BeforeDb[+2] ) );
            sum += filterCoeff[53] * ( clipALF( filterClipp[53], curr, pImg1BeforeDb[-1], pImg2BeforeDb[+1] ) );
            sum += filterCoeff[54] * ( clipALF( filterClipp[54], curr, pImg1BeforeDb[+0], pImg2BeforeDb[+0] ) );
            sum += filterCoeff[55] * ( clipALF( filterClipp[55], curr, pImg1BeforeDb[+1], pImg2BeforeDb[-1] ) );
            sum += filterCoeff[56] * ( clipALF( filterClipp[56], curr, pImg1BeforeDb[+2], pImg2BeforeDb[-2] ) );
            sum += filterCoeff[57] * ( clipALF( filterClipp[57], curr, pImg1BeforeDb[+3], pImg2BeforeDb[-3] ) );

            sum += filterCoeff[58] * ( clipALF( filterClipp[58], curr, pImg0BeforeDb[-4], pImg0BeforeDb[+4] ) );
            sum += filterCoeff[59] * ( clipALF( filterClipp[59], curr, pImg0BeforeDb[-3], pImg0BeforeDb[+3] ) );
            sum += filterCoeff[60] * ( clipALF( filterClipp[60], curr, pImg0BeforeDb[-2], pImg0BeforeDb[+2] ) );
            sum += filterCoeff[61] * ( clipALF( filterClipp[61], curr, pImg0BeforeDb[-1], pImg0BeforeDb[+1] ) );
            sum += filterCoeff[62] * ( clipALF( filterClipp[62], curr, pImg0BeforeDb[+0] ) );
            sum += filterCoeff[63] * ( clipALF( filterClipp[63], curr, pImg0[+0] ) );
          }
          else if( filtType == ALF_FIXED_FILTER_9_DB_9 )
          {
            sum += filterCoeff[0] * ( clipALF( filterClipp[0], curr, pImg7[+0], pImg8[+0] ) );

            sum += filterCoeff[1] * ( clipALF( filterClipp[1], curr, pImg5[-1], pImg6[+1] ) );
            sum += filterCoeff[2] * ( clipALF( filterClipp[2], curr, pImg5[+0], pImg6[+0] ) );
            sum += filterCoeff[3] * ( clipALF( filterClipp[3], curr, pImg5[+1], pImg6[-1] ) );

            sum += filterCoeff[4] * ( clipALF( filterClipp[4], curr, pImg3[-2], pImg4[+2] ) );
            sum += filterCoeff[5] * ( clipALF( filterClipp[5], curr, pImg3[-1], pImg4[+1] ) );
            sum += filterCoeff[6] * ( clipALF( filterClipp[6], curr, pImg3[+0], pImg4[+0] ) );
            sum += filterCoeff[7] * ( clipALF( filterClipp[7], curr, pImg3[+1], pImg4[-1] ) );
            sum += filterCoeff[8] * ( clipALF( filterClipp[8], curr, pImg3[+2], pImg4[-2] ) );

            sum += filterCoeff[9]  * ( clipALF( filterClipp[9] , curr, pImg1[-3], pImg2[+3] ) );
            sum += filterCoeff[10] * ( clipALF( filterClipp[10], curr, pImg1[-2], pImg2[+2] ) );
            sum += filterCoeff[11] * ( clipALF( filterClipp[11], curr, pImg1[-1], pImg2[+1] ) );
            sum += filterCoeff[12] * ( clipALF( filterClipp[12], curr, pImg1[+0], pImg2[+0] ) );
            sum += filterCoeff[13] * ( clipALF( filterClipp[13], curr, pImg1[+1], pImg2[-1] ) );
            sum += filterCoeff[14] * ( clipALF( filterClipp[14], curr, pImg1[+2], pImg2[-2] ) );
            sum += filterCoeff[15] * ( clipALF( filterClipp[15], curr, pImg1[+3], pImg2[-3] ) );

            sum += filterCoeff[16] * ( clipALF( filterClipp[16], curr, pImg0[-4], pImg0[+4] ) );
            sum += filterCoeff[17] * ( clipALF( filterClipp[17], curr, pImg0[-3], pImg0[+3] ) );
            sum += filterCoeff[18] * ( clipALF( filterClipp[18], curr, pImg0[-2], pImg0[+2] ) );
            sum += filterCoeff[19] * ( clipALF( filterClipp[19], curr, pImg0[-1], pImg0[+1] ) );

            sum += filterCoeff[20] * ( clipALF( filterClipp[20], curr, pImg7BeforeDb[+0], pImg8BeforeDb[+0] ) );

            sum += filterCoeff[21] * ( clipALF( filterClipp[21], curr, pImg5BeforeDb[-1], pImg6BeforeDb[+1] ) );
            sum += filterCoeff[22] * ( clipALF( filterClipp[22], curr, pImg5BeforeDb[+0], pImg6BeforeDb[+0] ) );
            sum += filterCoeff[23] * ( clipALF( filterClipp[23], curr, pImg5BeforeDb[+1], pImg6BeforeDb[-1] ) );

            sum += filterCoeff[24] * ( clipALF( filterClipp[24], curr, pImg3BeforeDb[-2], pImg4BeforeDb[+2] ) );
            sum += filterCoeff[25] * ( clipALF( filterClipp[25], curr, pImg3BeforeDb[-1], pImg4BeforeDb[+1] ) );
            sum += filterCoeff[26] * ( clipALF( filterClipp[26], curr, pImg3BeforeDb[+0], pImg4BeforeDb[+0] ) );
            sum += filterCoeff[27] * ( clipALF( filterClipp[27], curr, pImg3BeforeDb[+1], pImg4BeforeDb[-1] ) );
            sum += filterCoeff[28] * ( clipALF( filterClipp[28], curr, pImg3BeforeDb[+2], pImg4BeforeDb[-2] ) );

            sum += filterCoeff[29] * ( clipALF( filterClipp[29], curr, pImg1BeforeDb[-3], pImg2BeforeDb[+3] ) );
            sum += filterCoeff[30] * ( clipALF( filterClipp[30], curr, pImg1BeforeDb[-2], pImg2BeforeDb[+2] ) );
            sum += filterCoeff[31] * ( clipALF( filterClipp[31], curr, pImg1BeforeDb[-1], pImg2BeforeDb[+1] ) );
            sum += filterCoeff[32] * ( clipALF( filterClipp[32], curr, pImg1BeforeDb[+0], pImg2BeforeDb[+0] ) );
            sum += filterCoeff[33] * ( clipALF( filterClipp[33], curr, pImg1BeforeDb[+1], pImg2BeforeDb[-1] ) );
            sum += filterCoeff[34] * ( clipALF( filterClipp[34], curr, pImg1BeforeDb[+2], pImg2BeforeDb[-2] ) );
            sum += filterCoeff[35] * ( clipALF( filterClipp[35], curr, pImg1BeforeDb[+3], pImg2BeforeDb[-3] ) );

            sum += filterCoeff[36] * ( clipALF( filterClipp[36], curr, pImg0BeforeDb[-4], pImg0BeforeDb[+4] ) );
            sum += filterCoeff[37] * ( clipALF( filterClipp[37], curr, pImg0BeforeDb[-3], pImg0BeforeDb[+3] ) );
            sum += filterCoeff[38] * ( clipALF( filterClipp[38], curr, pImg0BeforeDb[-2], pImg0BeforeDb[+2] ) );
            sum += filterCoeff[39] * ( clipALF( filterClipp[39], curr, pImg0BeforeDb[-1], pImg0BeforeDb[+1] ) );
            sum += filterCoeff[40] * ( clipALF( filterClipp[40], curr, pImg0BeforeDb[+0] ) );
          }
          else
          {
            CHECK( 1, "not supported" );
          }
          sum = ( sum + offset ) >> shift;
          sum += curr;
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          fixedFilterResults[fixedFiltInd][posYDst + ii + i + padSize][posXDst + jj + j + padSize] = ClipPel( sum, clpRng );
#else
          fixedFilterResults[fixedFiltInd][posYDst + ii + i][posXDst + jj + j] = ClipPel( sum, clpRng );
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          fixedFilterResults[fixedFiltInd][posY + ii + i + padSize][posX + jj + j + padSize] = ClipPel( sum, clpRng );
#else
          fixedFilterResults[fixedFiltInd][posY + ii + i][posX + jj + j] = ClipPel( sum, clpRng );
#endif
#endif 
          pImg0Cur++;
          pImg0BeforeDb++;
        }//jj
      }//ii
    }//j
    pImgYBeforeDbPad0 += srcBeforeDbStride2;
    pImgYPad0 += srcStride2;
  }
}
#endif

#if JVET_X0071_ALF_BAND_CLASSIFIER
void AdaptiveLoopFilter::calcClassNew( AlfClassifier **classifier, const Area &blkDst, const Area &curBlk, const CPelBuf& srcLuma, int subBlkSize, AlfClassifier **classifier0, int classifierIdx, int bitDepth
#if JVET_AD0222_ALF_RESI_CLASS
  , const CPelBuf& srcLumaResi, uint32_t **buffer
#endif
  )
{
#if JVET_AD0222_ALF_RESI_CLASS
  if (classifierIdx == 1)
  {
#endif
  const Pel* src = srcLuma.buf;
  int stride = srcLuma.stride;
  int yOffset = blkDst.pos().y * stride;
  const Pel *src0 = &src[yOffset];
  const Pel *src1 = &src[yOffset + stride];
  int stride2 = 2 * stride;
  for (int i = 0; i < blkDst.height; i += subBlkSize)
  {
    for (int j = 0; j < blkDst.width; j += subBlkSize)
    {
      int xOffset = blkDst.pos().x + j;
      const Pel *pY0 = src0 + xOffset;
      const Pel *pY1 = src1 + xOffset;
      int sum = pY0[0] + pY0[1] + pY1[0] + pY1[1];
      int classIdx = (sum * ALF_NUM_CLASSES_CLASSIFIER[classifierIdx]) >> (bitDepth + 2);
      for (int ii = curBlk.y + i; ii < curBlk.y + i + subBlkSize; ii++)
      {
        for (int jj = curBlk.x + j; jj < curBlk.x + j + subBlkSize; jj++)
        {
          classifier[ii][jj] = classIdx << 2;
        }
      }
    }
    src0 += stride2;
    src1 += stride2;
  }
#if JVET_AD0222_ALF_RESI_CLASS
  }
  else
  {
    const Pel *src = srcLumaResi.buf;
    int        stride = srcLumaResi.stride;
    int        yOffset = blkDst.pos().y * stride;
    const Pel *src0 = &src[yOffset];
    int stride2 = stride * 2;
    const Pel *srcUp = src0 - ALF_PADDING_SIZE_PRED * stride + blkDst.pos().x - ALF_PADDING_SIZE_PRED;
    const Pel *srcDn = srcUp + stride;
    //2x2 sum
    for (int i = 0; i < blkDst.height + ALF_PADDING_SIZE_PRED * 2; i += 2)
    {
      for (int j = 0; j < blkDst.width + ALF_PADDING_SIZE_PRED * 2; j += 2)
      {
        buffer[i >> 1][j >> 1] = abs(srcUp[j]) + abs(srcUp[j + 1]) + abs(srcDn[j]) + abs(srcDn[j + 1]);
      }
      srcUp += stride2;
      srcDn += stride2;
    }
    //2x4 sum
    for (int i = 0; i < (blkDst.height + ALF_PADDING_SIZE_PRED * 2) >> 1; i++)
    {
      for (int j = 0; j < ((blkDst.width + ALF_PADDING_SIZE_PRED * 2) >> 1) - 1; j++)
      {
        buffer[i][j] = buffer[i][j] + buffer[i][j + 1];
      }
    }
    //4x4 sum
    for (int i = 0; i < ((blkDst.height + ALF_PADDING_SIZE_PRED * 2) >> 1) - 1; i++)
    {
      for (int j = 0; j < ((blkDst.width + ALF_PADDING_SIZE_PRED * 2) >> 1) - 1; j++)
      {
        buffer[i][j] = buffer[i][j] + buffer[i + 1][j];
      }
    }
    for (int i = 0; i < blkDst.height; i += subBlkSize)
    {
      for (int j = 0; j < blkDst.width; j += subBlkSize)
      {
        int i2 = i >> 1;
        int j2 = j >> 1;
        int sum = buffer[i2][j2] + buffer[i2][j2 + 2] + buffer[i2 + 2][j2] + buffer[i2 + 2][j2 + 2];
        int shiftOffset = ALF_RESI_SHIFT_OFFSET;
        int classIdx = sum >> (bitDepth - shiftOffset);
        if (classIdx > 24)
        {
          classIdx = 24;
        }
        for (int ii = curBlk.y + i; ii < curBlk.y + i + subBlkSize; ii++)
        {
          for (int jj = curBlk.x + j; jj < curBlk.x + j + subBlkSize; jj++)
          {
            classifier[ii][jj] = classIdx << 2;
          }
        }
      }
    }
  }
#endif
}
#endif
#if ALF_IMPROVEMENT
int AdaptiveLoopFilter::assignAct( int avg_varPrec, int shift, int noAct )
{
  if (noAct == 5)
  {
    int th[] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
    return th[std::min(avg_varPrec >> shift, 15)];
  }
  else
  {
    int avg_var = avg_varPrec >> (shift - 4);
    const int th[193] = { 0,  1,  2,  3,  4,  4,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15 };
    return th[std::min(avg_var, 192)];
  }
}

void AdaptiveLoopFilter::calcClass(AlfClassifier **classifier, const Area &blkDst, const Area &curBlk, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS])
{
  int shift = 9 + bitDepth;
  const int multTab[] = { 5628, 1407, 624, 351, 225, 156 };
  int mult = multTab[dirWindSize];

  for( int i = 0; i < curBlk.height; i += subBlkSize)
  {
    for( int j = 0; j < curBlk.width; j += subBlkSize)
    {
      int sum = 0, sumV = 0, sumH = 0, sumD0 = 0, sumD1 = 0;
      if( dirWindSize == 1 )
      {
        int yOffset = (i >> 1) + 2;
        int xOffset = (j >> 1) + 2;
        sumV = laplacian[VER][yOffset][xOffset];
        sumH = laplacian[HOR][yOffset][xOffset];
        sumD0 = laplacian[DIAG0][yOffset][xOffset];
        sumD1 = laplacian[DIAG1][yOffset][xOffset];
      }
      else if (dirWindSize == 5)
      {
        int yOffset = i >> 1;
        int xOffset = j >> 1;
        sumV = laplacian[VER][yOffset][xOffset];
        sumH = laplacian[HOR][yOffset][xOffset];
        sumD0 = laplacian[DIAG0][yOffset][xOffset];
        sumD1 = laplacian[DIAG1][yOffset][xOffset];
      }
      else
      {
        CHECK(1, "other window sizes are not supported");
      }
      sum += sumV + sumH;
      int direction = 0, mainDirection, secondaryDirection, dirTempHV = 0, dirTempD = 0, hv1, hv0, d1, d0, hvd1, hvd0;
      if (sumV > sumH)
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }
      if (sumD0 > sumD1)
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }

      if ( ( uint64_t )d1 * hv0 > ( uint64_t )d0 * hv1 )
      {
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
        hvd1 = d1;
        hvd0 = d0;
      }
      else
      {
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
        hvd1 = hv1;
        hvd0 = hv0;
      }
      //activity
      int activity = 0;
      if (noAct == 5)
      {
        const int th[] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
#if JVET_X0071_ALF_BAND_CLASSIFIER
        activity = th[std::min((mult * 3 * sum) >> shift, 15)];
#else
        activity = th[std::min((mult * sum) >> shift, 15)];
#endif
      }
      else
      {
        const int th[193] = { 0,  1,  2,  3,  4,  4,  5,  5,  6,  6,  6,  6,  7,  7,  7,  7,  8,  8,  8,  8,  8,  8,  8,  8,  9,  9,  9,  9,  9,  9,  9,  9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15 };
        activity = th[std::min((mult * sum) >> (shift - 4), 192)];
      }

      //direction
      if (classDir == 0)
      {
        int directionStrength = 0;
        if (hvd1 * 2 > 9 * hvd0)
        {
          directionStrength = 2;
        }
        else if (hvd1 > 2 * hvd0)
        {
          directionStrength = 1;
        }

        if (directionStrength)
        {
          direction = ((mainDirection & 0x1) << 1) + directionStrength;
        }
      }
      else
      {
        int edgeStrengthHV = 0, edgeStrengthD = 0;
        if (hv1 > 8 * hv0)
        {
          edgeStrengthHV = 6;
        }
        else if (hv1 * 2 > 9 * hv0)
        {
          edgeStrengthHV = 5;
        }
        else if (hv1 > 3 * hv0)
        {
          edgeStrengthHV = 4;
        }
        else if (hv1 > 2 * hv0)
        {
          edgeStrengthHV = 3;
        }
        else if (hv1 * 2 > 3 * hv0)
        {
          edgeStrengthHV = 2;
        }
        else if (hv1 * 4 > 5 * hv0)
        {
          edgeStrengthHV = 1;
        }

        if (d1 > 8 * d0)
        {
          edgeStrengthD = 6;
        }
        else if (d1 * 2 > 9 * d0)
        {
          edgeStrengthD = 5;
        }
        else if (d1 > 3 * d0)
        {
          edgeStrengthD = 4;
        }
        else if (d1 > 2 * d0)
        {
          edgeStrengthD = 3;
        }
        else if (d1 * 2 > 3 * d0)
        {
          edgeStrengthD = 2;
        }
        else if (d1 * 4 > 5 * d0)
        {
          edgeStrengthD = 1;
        }

        if ( ( uint64_t )hv1 * d0 >  ( uint64_t )hv0 * d1)
        {
          direction = mappingDir[edgeStrengthHV][edgeStrengthD];
        }
        else
        {
          direction = 28 + mappingDir[edgeStrengthD][edgeStrengthHV];
        }
      }

      int actDirInd = 0;
      if ( classDir == 0 )
      {
        actDirInd = noDir * activity + direction;
      }
      else
      {
        actDirInd = noDir * ( noDir + 1 ) * activity + direction;
      }

      const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + (secondaryDirection >> 1)];

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int posYDst = blkDst.pos().y;
      int posXDst = blkDst.pos().x;

      for (int ii = posYDst + i; ii < posYDst + i + subBlkSize; ii++)
      {
        for (int jj = posXDst + j; jj < posXDst + j + subBlkSize; jj++)
        {
#else
      for (int ii = curBlk.y + i; ii < curBlk.y + i + subBlkSize; ii++)
      {
        for (int jj = curBlk.x + j; jj < curBlk.x + j + subBlkSize; jj++)
        {
#endif
          classifier[ii][jj] = (actDirInd << 2) + transposeIdx;
        }
      }
    }
  }
}
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
void AdaptiveLoopFilter::fixedFiltering(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                        const Area &blkDst,
#endif
                                        Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd,
                                        const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                                        const ClpRng &clpRng, const Pel clippingValues[4])
{
  const int shift = m_NUM_BITS_FIXED_FILTER - 1;
  const int offset = 1 << (shift - 1);
  const int srcStride = srcLuma.stride;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int posYDst = blkDst.y;
  int posXDst = blkDst.x;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif

  int posY = curBlk.y;
  int posX = curBlk.x;
  const Pel *pImgYPad0 = srcLuma.buf + posY * srcStride + posX;

  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10, *pImg11, *pImg12;
  const int clsSizeY = 2;
  const int clsSizeX = 2;
  const int srcStride2 = srcStride * clsSizeY;

  for( int i = 0; i < curBlk.height; i+= clsSizeY)
  {
    for (int j = 0; j < curBlk.width; j += clsSizeX)
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int classIdx     = classifier[posYDst + i][posXDst + j] >> 2;
      int transposeIdx = classifier[posYDst + i][posXDst + j] & 0x3;
#else
      int classIdx = classifier[posY + i][posX + j] >> 2;
      int transposeIdx = classifier[posY + i][posX + j] & 0x3;
#endif
      int filterIdx = classIndFixed[classIdx];

      std::array<short, FIX_FILTER_NUM_COEFF> filterCoeff;
      std::array<short, FIX_FILTER_NUM_COEFF> filterClipp;

      const short* coeff = m_filterCoeffFixed[fixedFiltQpInd][dirWindSize][filterIdx];
      const short* clipp = m_clippingFixed[fixedFiltQpInd][dirWindSize][filterIdx];

      if (transposeIdx == 0)
      {
        filterCoeff = { coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8], coeff[9], coeff[10], coeff[11], coeff[12], coeff[13], coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27], coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34], coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40], coeff[41] };
        filterClipp = { clipp[0], clipp[1], clipp[2], clipp[3], clipp[4], clipp[5], clipp[6], clipp[7], clipp[8], clipp[9], clipp[10], clipp[11], clipp[12], clipp[13], clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27], clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34], clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40], clipp[41] };
      }
      else if (transposeIdx == 1)
      {
        filterCoeff = { coeff[36], coeff[25], coeff[37], coeff[35], coeff[16], coeff[26], coeff[38], coeff[34], coeff[24], coeff[9], coeff[17], coeff[27], coeff[39], coeff[33], coeff[23], coeff[15], coeff[4], coeff[10], coeff[18], coeff[28], coeff[40], coeff[32], coeff[22], coeff[14], coeff[8], coeff[1], coeff[5], coeff[11], coeff[19], coeff[29], coeff[41], coeff[31], coeff[21], coeff[13], coeff[7], coeff[3], coeff[0], coeff[2], coeff[6], coeff[12], coeff[20], coeff[30] };
        filterClipp = { clipp[36], clipp[25], clipp[37], clipp[35], clipp[16], clipp[26], clipp[38], clipp[34], clipp[24], clipp[9], clipp[17], clipp[27], clipp[39], clipp[33], clipp[23], clipp[15], clipp[4], clipp[10], clipp[18], clipp[28], clipp[40], clipp[32], clipp[22], clipp[14], clipp[8], clipp[1], clipp[5], clipp[11], clipp[19], clipp[29], clipp[41], clipp[31], clipp[21], clipp[13], clipp[7], clipp[3], clipp[0], clipp[2], clipp[6], clipp[12], clipp[20], clipp[30] };
      }
      else if (transposeIdx == 2)
      {
        filterCoeff = { coeff[0], coeff[3], coeff[2], coeff[1], coeff[8], coeff[7], coeff[6], coeff[5], coeff[4], coeff[15], coeff[14], coeff[13], coeff[12], coeff[11], coeff[10], coeff[9], coeff[24], coeff[23], coeff[22], coeff[21], coeff[20], coeff[19], coeff[18], coeff[17], coeff[16], coeff[35], coeff[34], coeff[33], coeff[32], coeff[31], coeff[30], coeff[29], coeff[28], coeff[27], coeff[26], coeff[25], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40], coeff[41] };
        filterClipp = { clipp[0], clipp[3], clipp[2], clipp[1], clipp[8], clipp[7], clipp[6], clipp[5], clipp[4], clipp[15], clipp[14], clipp[13], clipp[12], clipp[11], clipp[10], clipp[9], clipp[24], clipp[23], clipp[22], clipp[21], clipp[20], clipp[19], clipp[18], clipp[17], clipp[16], clipp[35], clipp[34], clipp[33], clipp[32], clipp[31], clipp[30], clipp[29], clipp[28], clipp[27], clipp[26], clipp[25], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40], clipp[41] };
      }
      else
      {
        filterCoeff = { coeff[36], coeff[35], coeff[37], coeff[25], coeff[24], coeff[34], coeff[38], coeff[26], coeff[16], coeff[15], coeff[23], coeff[33], coeff[39], coeff[27], coeff[17], coeff[9], coeff[8], coeff[14], coeff[22], coeff[32], coeff[40], coeff[28], coeff[18], coeff[10], coeff[4], coeff[3], coeff[7], coeff[13], coeff[21], coeff[31], coeff[41], coeff[29], coeff[19], coeff[11], coeff[5], coeff[1], coeff[0], coeff[2], coeff[6], coeff[12], coeff[20], coeff[30] };
        filterClipp = { clipp[36], clipp[35], clipp[37], clipp[25], clipp[24], clipp[34], clipp[38], clipp[26], clipp[16], clipp[15], clipp[23], clipp[33], clipp[39], clipp[27], clipp[17], clipp[9], clipp[8], clipp[14], clipp[22], clipp[32], clipp[40], clipp[28], clipp[18], clipp[10], clipp[4], clipp[3], clipp[7], clipp[13], clipp[21], clipp[31], clipp[41], clipp[29], clipp[19], clipp[11], clipp[5], clipp[1], clipp[0], clipp[2], clipp[6], clipp[12], clipp[20], clipp[30] };
      }

      for (int k = 0; k < 42; k++)
      {
        filterClipp[k] = clippingValues[filterClipp[k]];
      }

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        pImg0 = pImgYPad0 + ii * srcStride + j;
        for (int jj = 0; jj < clsSizeX; jj++)
        {
          pImg1 = pImg0 - srcStride;
          pImg3 = pImg1 - srcStride;
          pImg5 = pImg3 - srcStride;
          pImg7 = pImg5 - srcStride;
          pImg9 = pImg7 - srcStride;
          pImg11 = pImg9 - srcStride;
          pImg2 = pImg0 + srcStride;
          pImg4 = pImg2 + srcStride;
          pImg6 = pImg4 + srcStride;
          pImg8 = pImg6 + srcStride;
          pImg10 = pImg8 + srcStride;
          pImg12 = pImg10 + srcStride;
          Pel curr = pImg0[0];

          int sum = 0;
          sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pImg11[+0], pImg12[+0]));

          sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pImg9[-1], pImg10[+1]));
          sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pImg9[+0], pImg10[+0]));
          sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pImg9[+1], pImg10[-1]));

          sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pImg7[-2], pImg8[+2]));
          sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pImg7[-1], pImg8[+1]));
          sum += filterCoeff[6] * (clipALF(filterClipp[6], curr, pImg7[+0], pImg8[+0]));
          sum += filterCoeff[7] * (clipALF(filterClipp[7], curr, pImg7[+1], pImg8[-1]));
          sum += filterCoeff[8] * (clipALF(filterClipp[8], curr, pImg7[+2], pImg8[-2]));

          sum += filterCoeff[9] * (clipALF(filterClipp[9], curr, pImg5[-3], pImg6[+3]));
          sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImg5[-2], pImg6[+2]));
          sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImg5[-1], pImg6[+1]));
          sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImg5[+0], pImg6[+0]));
          sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImg5[+1], pImg6[-1]));
          sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImg5[+2], pImg6[-2]));
          sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImg5[+3], pImg6[-3]));

          sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImg3[-4], pImg4[+4]));
          sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImg3[-3], pImg4[+3]));
          sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImg3[-2], pImg4[+2]));
          sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImg3[-1], pImg4[+1]));
          sum += filterCoeff[20] * (clipALF(filterClipp[20], curr, pImg3[+0], pImg4[+0]));
          sum += filterCoeff[21] * (clipALF(filterClipp[21], curr, pImg3[+1], pImg4[-1]));
          sum += filterCoeff[22] * (clipALF(filterClipp[22], curr, pImg3[+2], pImg4[-2]));
          sum += filterCoeff[23] * (clipALF(filterClipp[23], curr, pImg3[+3], pImg4[-3]));
          sum += filterCoeff[24] * (clipALF(filterClipp[24], curr, pImg3[+4], pImg4[-4]));

          sum += filterCoeff[25] * (clipALF(filterClipp[25], curr, pImg1[-5], pImg2[+5]));
          sum += filterCoeff[26] * (clipALF(filterClipp[26], curr, pImg1[-4], pImg2[+4]));
          sum += filterCoeff[27] * (clipALF(filterClipp[27], curr, pImg1[-3], pImg2[+3]));
          sum += filterCoeff[28] * (clipALF(filterClipp[28], curr, pImg1[-2], pImg2[+2]));
          sum += filterCoeff[29] * (clipALF(filterClipp[29], curr, pImg1[-1], pImg2[+1]));
          sum += filterCoeff[30] * (clipALF(filterClipp[30], curr, pImg1[+0], pImg2[+0]));
          sum += filterCoeff[31] * (clipALF(filterClipp[31], curr, pImg1[+1], pImg2[-1]));
          sum += filterCoeff[32] * (clipALF(filterClipp[32], curr, pImg1[+2], pImg2[-2]));
          sum += filterCoeff[33] * (clipALF(filterClipp[33], curr, pImg1[+3], pImg2[-3]));
          sum += filterCoeff[34] * (clipALF(filterClipp[34], curr, pImg1[+4], pImg2[-4]));
          sum += filterCoeff[35] * (clipALF(filterClipp[35], curr, pImg1[+5], pImg2[-5]));

          sum += filterCoeff[36] * (clipALF(filterClipp[36], curr, pImg0[-6], pImg0[+6]));
          sum += filterCoeff[37] * (clipALF(filterClipp[37], curr, pImg0[-5], pImg0[+5]));
          sum += filterCoeff[38] * (clipALF(filterClipp[38], curr, pImg0[-4], pImg0[+4]));
          sum += filterCoeff[39] * (clipALF(filterClipp[39], curr, pImg0[-3], pImg0[+3]));
          sum += filterCoeff[40] * (clipALF(filterClipp[40], curr, pImg0[-2], pImg0[+2]));
          sum += filterCoeff[41] * (clipALF(filterClipp[41], curr, pImg0[-1], pImg0[+1]));

          sum = (sum + offset) >> shift;
          sum += curr;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          fixedFilterResults[fixedFiltInd][posYDst + ii + i + padSize][posXDst + jj + j + padSize] = ClipPel(sum, clpRng);
#else
          fixedFilterResults[fixedFiltInd][posYDst + ii + i][posXDst + jj + j] = ClipPel(sum, clpRng);
#endif
#else
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
          fixedFilterResults[fixedFiltInd][posY + ii + i + padSize][posX + jj + j + padSize] = ClipPel(sum, clpRng);
#else
          fixedFilterResults[fixedFiltInd][posY + ii + i][posX + jj + j] = ClipPel(sum, clpRng);
#endif
#endif 

          pImg0++;
        }//jj
      }//ii
    }//j
    pImgYPad0 += srcStride2;
  }
}
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
void AdaptiveLoopFilter::fixedFilteringResi(AlfClassifier **classifier, const CPelBuf &srcResiLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                            const Area &blkDst,
#endif
                                            Pel ***fixedFilterResiResults, int picWidth, const int fixedFiltInd,
                                            const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd,
                                            int dirWindSize, const ClpRng &clpRng, const Pel clippingValues[4])
{
  const int shift     = m_NUM_BITS_FIXED_FILTER - 1;
  const int offset    = 1 << (shift - 1);
  const int srcStride = srcResiLuma.stride;

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  int posYDst = blkDst.y;
  int posXDst = blkDst.x;
#endif
  int        posY      = curBlk.y;
  int        posX      = curBlk.x;
  const Pel *pImgYPad0 = srcResiLuma.buf + posY * srcStride + posX;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#else
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8, *pImg9, *pImg10, *pImg11, *pImg12;
#endif
  const int  clsSizeY   = 2;
  const int  clsSizeX   = 2;
  const int  srcStride2 = srcStride * clsSizeY;

  for (int i = 0; i < curBlk.height; i += clsSizeY)
  {
    for (int j = 0; j < curBlk.width; j += clsSizeX)
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      int classIdx     = classifier[posYDst + i][posXDst + j] >> 2;
      int transposeIdx = classifier[posYDst + i][posXDst + j] & 0x3;
#else
      int classIdx = classifier[posY + i][posX + j] >> 2;
      int transposeIdx = classifier[posY + i][posX + j] & 0x3;
#endif
      int filterIdx = classIndFixed[classIdx];
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      std::array<short, FIX_FILTER_NUM_COEFF_9_DB_9> filterCoeff;
      std::array<short, FIX_FILTER_NUM_COEFF_9_DB_9> filterClipp;
      const short *coeff = m_filterCoeffFixed9Db9[fixedFiltQpInd][filterIdx];
      const short *clipp = m_clippingFixed9Db9[fixedFiltQpInd][filterIdx];

      if( transposeIdx == 0 )
      {
        filterCoeff = { coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6], coeff[7], coeff[8], coeff[9], coeff[10], coeff[11], coeff[12], coeff[13], coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27], coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34], coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
        filterClipp = { clipp[0], clipp[1], clipp[2], clipp[3], clipp[4], clipp[5], clipp[6], clipp[7], clipp[8], clipp[9], clipp[10], clipp[11], clipp[12], clipp[13], clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27], clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34], clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
      }
      else if( transposeIdx == 1 )
      {
        filterCoeff = { coeff[16], coeff[9], coeff[17], coeff[15], coeff[4], coeff[10], coeff[18], coeff[14], coeff[8], coeff[1], coeff[5], coeff[11], coeff[19], coeff[13], coeff[7], coeff[3], coeff[0], coeff[2], coeff[6], coeff[12], coeff[36], coeff[29], coeff[37], coeff[35], coeff[24], coeff[30], coeff[38], coeff[34], coeff[28], coeff[21], coeff[25], coeff[31], coeff[39], coeff[33], coeff[27], coeff[23], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
        filterClipp = { clipp[16], clipp[9], clipp[17], clipp[15], clipp[4], clipp[10], clipp[18], clipp[14], clipp[8], clipp[1], clipp[5], clipp[11], clipp[19], clipp[13], clipp[7], clipp[3], clipp[0], clipp[2], clipp[6], clipp[12],  clipp[36], clipp[29], clipp[37], clipp[35], clipp[24], clipp[30], clipp[38], clipp[34], clipp[28], clipp[21], clipp[25], clipp[31], clipp[39], clipp[33], clipp[27], clipp[23], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
      }
      else if (transposeIdx == 2)
      {
        filterCoeff = { coeff[0], coeff[3], coeff[2], coeff[1], coeff[8], coeff[7], coeff[6], coeff[5], coeff[4], coeff[15], coeff[14] , coeff[13], coeff[12], coeff[11], coeff[10] , coeff[9], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20], coeff[23], coeff[22], coeff[21], coeff[28], coeff[27], coeff[26], coeff[25], coeff[24], coeff[35], coeff[34] , coeff[33], coeff[32], coeff[31], coeff[30] , coeff[29], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40] };
        filterClipp = { clipp[0], clipp[3], clipp[2], clipp[1], clipp[8], clipp[7], clipp[6], clipp[5], clipp[4], clipp[15], clipp[14] , clipp[13], clipp[12], clipp[11], clipp[10] , clipp[9], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20], clipp[23], clipp[22], clipp[21], clipp[28], clipp[27], clipp[26], clipp[25], clipp[24], clipp[35], clipp[34] , clipp[33], clipp[32], clipp[31], clipp[30] , clipp[29], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40] };
      }
      else
      {
        filterCoeff = { coeff[16], coeff[15], coeff[17], coeff[9], coeff[8], coeff[14], coeff[18], coeff[10], coeff[4], coeff[3], coeff[7], coeff[13], coeff[19], coeff[11], coeff[5], coeff[1], coeff[0], coeff[2], coeff[6], coeff[12], coeff[36], coeff[35], coeff[37], coeff[29], coeff[28], coeff[34], coeff[38], coeff[30], coeff[24], coeff[23], coeff[27], coeff[33], coeff[39], coeff[31], coeff[25], coeff[21], coeff[20], coeff[22], coeff[26], coeff[32], coeff[40] };
        filterClipp = { clipp[16], clipp[15], clipp[17], clipp[9], clipp[8], clipp[14], clipp[18], clipp[10], clipp[4], clipp[3], clipp[7], clipp[13], clipp[19], clipp[11], clipp[5], clipp[1], clipp[0], clipp[2], clipp[6], clipp[12], clipp[36], clipp[35], clipp[37], clipp[29], clipp[28], clipp[34], clipp[38], clipp[30], clipp[24], clipp[23], clipp[27], clipp[33], clipp[39], clipp[31], clipp[25], clipp[21], clipp[20], clipp[22], clipp[26], clipp[32], clipp[40] };
      }
      for( int k = 0; k < 20; k++ )
      {
        filterCoeff[k] = filterCoeff[k] + filterCoeff[k + 20];
        filterClipp[k] = (filterClipp[k] + filterClipp[k + 20] + 1) >> 1;
      }
      for( int k = 0; k < FIX_FILTER_NUM_COEFF_9_DB_9; k++ )
      {
        filterClipp[k] = clippingValues[filterClipp[k]];
      }
#else
      std::array<short, FIX_FILTER_NUM_COEFF> filterCoeff;
      std::array<short, FIX_FILTER_NUM_COEFF> filterClipp;
      const short *coeff = m_filterCoeffFixed[fixedFiltQpInd][dirWindSize][filterIdx];
      const short *clipp = m_clippingFixed[fixedFiltQpInd][dirWindSize][filterIdx];

      if (transposeIdx == 0)
      {
        filterCoeff = { coeff[0],  coeff[1],  coeff[2],  coeff[3],  coeff[4],  coeff[5],  coeff[6],
                        coeff[7],  coeff[8],  coeff[9],  coeff[10], coeff[11], coeff[12], coeff[13],
                        coeff[14], coeff[15], coeff[16], coeff[17], coeff[18], coeff[19], coeff[20],
                        coeff[21], coeff[22], coeff[23], coeff[24], coeff[25], coeff[26], coeff[27],
                        coeff[28], coeff[29], coeff[30], coeff[31], coeff[32], coeff[33], coeff[34],
                        coeff[35], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40], coeff[41] };
        filterClipp = { clipp[0],  clipp[1],  clipp[2],  clipp[3],  clipp[4],  clipp[5],  clipp[6],
                        clipp[7],  clipp[8],  clipp[9],  clipp[10], clipp[11], clipp[12], clipp[13],
                        clipp[14], clipp[15], clipp[16], clipp[17], clipp[18], clipp[19], clipp[20],
                        clipp[21], clipp[22], clipp[23], clipp[24], clipp[25], clipp[26], clipp[27],
                        clipp[28], clipp[29], clipp[30], clipp[31], clipp[32], clipp[33], clipp[34],
                        clipp[35], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40], clipp[41] };
      }
      else if (transposeIdx == 1)
      {
        filterCoeff = { coeff[36], coeff[25], coeff[37], coeff[35], coeff[16], coeff[26], coeff[38],
                        coeff[34], coeff[24], coeff[9],  coeff[17], coeff[27], coeff[39], coeff[33],
                        coeff[23], coeff[15], coeff[4],  coeff[10], coeff[18], coeff[28], coeff[40],
                        coeff[32], coeff[22], coeff[14], coeff[8],  coeff[1],  coeff[5],  coeff[11],
                        coeff[19], coeff[29], coeff[41], coeff[31], coeff[21], coeff[13], coeff[7],
                        coeff[3],  coeff[0],  coeff[2],  coeff[6],  coeff[12], coeff[20], coeff[30] };
        filterClipp = { clipp[36], clipp[25], clipp[37], clipp[35], clipp[16], clipp[26], clipp[38],
                        clipp[34], clipp[24], clipp[9],  clipp[17], clipp[27], clipp[39], clipp[33],
                        clipp[23], clipp[15], clipp[4],  clipp[10], clipp[18], clipp[28], clipp[40],
                        clipp[32], clipp[22], clipp[14], clipp[8],  clipp[1],  clipp[5],  clipp[11],
                        clipp[19], clipp[29], clipp[41], clipp[31], clipp[21], clipp[13], clipp[7],
                        clipp[3],  clipp[0],  clipp[2],  clipp[6],  clipp[12], clipp[20], clipp[30] };
      }
      else if (transposeIdx == 2)
      {
        filterCoeff = { coeff[0],  coeff[3],  coeff[2],  coeff[1],  coeff[8],  coeff[7],  coeff[6],
                        coeff[5],  coeff[4],  coeff[15], coeff[14], coeff[13], coeff[12], coeff[11],
                        coeff[10], coeff[9],  coeff[24], coeff[23], coeff[22], coeff[21], coeff[20],
                        coeff[19], coeff[18], coeff[17], coeff[16], coeff[35], coeff[34], coeff[33],
                        coeff[32], coeff[31], coeff[30], coeff[29], coeff[28], coeff[27], coeff[26],
                        coeff[25], coeff[36], coeff[37], coeff[38], coeff[39], coeff[40], coeff[41] };
        filterClipp = { clipp[0],  clipp[3],  clipp[2],  clipp[1],  clipp[8],  clipp[7],  clipp[6],
                        clipp[5],  clipp[4],  clipp[15], clipp[14], clipp[13], clipp[12], clipp[11],
                        clipp[10], clipp[9],  clipp[24], clipp[23], clipp[22], clipp[21], clipp[20],
                        clipp[19], clipp[18], clipp[17], clipp[16], clipp[35], clipp[34], clipp[33],
                        clipp[32], clipp[31], clipp[30], clipp[29], clipp[28], clipp[27], clipp[26],
                        clipp[25], clipp[36], clipp[37], clipp[38], clipp[39], clipp[40], clipp[41] };
      }
      else
      {
        filterCoeff = { coeff[36], coeff[35], coeff[37], coeff[25], coeff[24], coeff[34], coeff[38],
                        coeff[26], coeff[16], coeff[15], coeff[23], coeff[33], coeff[39], coeff[27],
                        coeff[17], coeff[9],  coeff[8],  coeff[14], coeff[22], coeff[32], coeff[40],
                        coeff[28], coeff[18], coeff[10], coeff[4],  coeff[3],  coeff[7],  coeff[13],
                        coeff[21], coeff[31], coeff[41], coeff[29], coeff[19], coeff[11], coeff[5],
                        coeff[1],  coeff[0],  coeff[2],  coeff[6],  coeff[12], coeff[20], coeff[30] };
        filterClipp = { clipp[36], clipp[35], clipp[37], clipp[25], clipp[24], clipp[34], clipp[38],
                        clipp[26], clipp[16], clipp[15], clipp[23], clipp[33], clipp[39], clipp[27],
                        clipp[17], clipp[9],  clipp[8],  clipp[14], clipp[22], clipp[32], clipp[40],
                        clipp[28], clipp[18], clipp[10], clipp[4],  clipp[3],  clipp[7],  clipp[13],
                        clipp[21], clipp[31], clipp[41], clipp[29], clipp[19], clipp[11], clipp[5],
                        clipp[1],  clipp[0],  clipp[2],  clipp[6],  clipp[12], clipp[20], clipp[30] };
      }

      for (int k = 0; k < 42; k++)
      {
        filterClipp[k] = clippingValues[filterClipp[k]];
      }
#endif

      for (int ii = 0; ii < clsSizeY; ii++)
      {
        pImg0 = pImgYPad0 + ii * srcStride + j;
        for (int jj = 0; jj < clsSizeX; jj++)
        {
          pImg1    = pImg0 - srcStride;
          pImg3    = pImg1 - srcStride;
          pImg5    = pImg3 - srcStride;
          pImg7    = pImg5 - srcStride;
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
          pImg9    = pImg7 - srcStride;
          pImg11   = pImg9 - srcStride;
#endif
          pImg2    = pImg0 + srcStride;
          pImg4    = pImg2 + srcStride;
          pImg6    = pImg4 + srcStride;
          pImg8    = pImg6 + srcStride;
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
          pImg10   = pImg8 + srcStride;
          pImg12   = pImg10 + srcStride;
#endif
          Pel curr = pImg0[0];

          int sum = 0;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pImg7[+0], pImg8[+0]));

          sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pImg5[-1], pImg6[+1]));
          sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pImg5[+0], pImg6[+0]));
          sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pImg5[+1], pImg6[-1]));

          sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pImg3[-2], pImg4[+2]));
          sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pImg3[-1], pImg4[+1]));
          sum += filterCoeff[6] * (clipALF(filterClipp[6], curr, pImg3[+0], pImg4[+0]));
          sum += filterCoeff[7] * (clipALF(filterClipp[7], curr, pImg3[+1], pImg4[-1]));
          sum += filterCoeff[8] * (clipALF(filterClipp[8], curr, pImg3[+2], pImg4[-2]));

          sum += filterCoeff[9]  * (clipALF(filterClipp[9] , curr, pImg1[-3], pImg2[+3]));
          sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImg1[-2], pImg2[+2]));
          sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImg1[-1], pImg2[+1]));
          sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImg1[+0], pImg2[+0]));
          sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImg1[+1], pImg2[-1]));
          sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImg1[+2], pImg2[-2]));
          sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImg1[+3], pImg2[-3]));

          sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImg0[-4], pImg0[+4]));
          sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImg0[-3], pImg0[+3]));
          sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImg0[-2], pImg0[+2]));
          sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImg0[-1], pImg0[+1]));

          sum += filterCoeff[40] * (clipALF(filterClipp[40], curr, 0));
#else
          sum += filterCoeff[0] * (clipALF(filterClipp[0], curr, pImg11[+0], pImg12[+0]));

          sum += filterCoeff[1] * (clipALF(filterClipp[1], curr, pImg9[-1], pImg10[+1]));
          sum += filterCoeff[2] * (clipALF(filterClipp[2], curr, pImg9[+0], pImg10[+0]));
          sum += filterCoeff[3] * (clipALF(filterClipp[3], curr, pImg9[+1], pImg10[-1]));

          sum += filterCoeff[4] * (clipALF(filterClipp[4], curr, pImg7[-2], pImg8[+2]));
          sum += filterCoeff[5] * (clipALF(filterClipp[5], curr, pImg7[-1], pImg8[+1]));
          sum += filterCoeff[6] * (clipALF(filterClipp[6], curr, pImg7[+0], pImg8[+0]));
          sum += filterCoeff[7] * (clipALF(filterClipp[7], curr, pImg7[+1], pImg8[-1]));
          sum += filterCoeff[8] * (clipALF(filterClipp[8], curr, pImg7[+2], pImg8[-2]));

          sum += filterCoeff[9] * (clipALF(filterClipp[9], curr, pImg5[-3], pImg6[+3]));
          sum += filterCoeff[10] * (clipALF(filterClipp[10], curr, pImg5[-2], pImg6[+2]));
          sum += filterCoeff[11] * (clipALF(filterClipp[11], curr, pImg5[-1], pImg6[+1]));
          sum += filterCoeff[12] * (clipALF(filterClipp[12], curr, pImg5[+0], pImg6[+0]));
          sum += filterCoeff[13] * (clipALF(filterClipp[13], curr, pImg5[+1], pImg6[-1]));
          sum += filterCoeff[14] * (clipALF(filterClipp[14], curr, pImg5[+2], pImg6[-2]));
          sum += filterCoeff[15] * (clipALF(filterClipp[15], curr, pImg5[+3], pImg6[-3]));

          sum += filterCoeff[16] * (clipALF(filterClipp[16], curr, pImg3[-4], pImg4[+4]));
          sum += filterCoeff[17] * (clipALF(filterClipp[17], curr, pImg3[-3], pImg4[+3]));
          sum += filterCoeff[18] * (clipALF(filterClipp[18], curr, pImg3[-2], pImg4[+2]));
          sum += filterCoeff[19] * (clipALF(filterClipp[19], curr, pImg3[-1], pImg4[+1]));
          sum += filterCoeff[20] * (clipALF(filterClipp[20], curr, pImg3[+0], pImg4[+0]));
          sum += filterCoeff[21] * (clipALF(filterClipp[21], curr, pImg3[+1], pImg4[-1]));
          sum += filterCoeff[22] * (clipALF(filterClipp[22], curr, pImg3[+2], pImg4[-2]));
          sum += filterCoeff[23] * (clipALF(filterClipp[23], curr, pImg3[+3], pImg4[-3]));
          sum += filterCoeff[24] * (clipALF(filterClipp[24], curr, pImg3[+4], pImg4[-4]));

          sum += filterCoeff[25] * (clipALF(filterClipp[25], curr, pImg1[-5], pImg2[+5]));
          sum += filterCoeff[26] * (clipALF(filterClipp[26], curr, pImg1[-4], pImg2[+4]));
          sum += filterCoeff[27] * (clipALF(filterClipp[27], curr, pImg1[-3], pImg2[+3]));
          sum += filterCoeff[28] * (clipALF(filterClipp[28], curr, pImg1[-2], pImg2[+2]));
          sum += filterCoeff[29] * (clipALF(filterClipp[29], curr, pImg1[-1], pImg2[+1]));
          sum += filterCoeff[30] * (clipALF(filterClipp[30], curr, pImg1[+0], pImg2[+0]));
          sum += filterCoeff[31] * (clipALF(filterClipp[31], curr, pImg1[+1], pImg2[-1]));
          sum += filterCoeff[32] * (clipALF(filterClipp[32], curr, pImg1[+2], pImg2[-2]));
          sum += filterCoeff[33] * (clipALF(filterClipp[33], curr, pImg1[+3], pImg2[-3]));
          sum += filterCoeff[34] * (clipALF(filterClipp[34], curr, pImg1[+4], pImg2[-4]));
          sum += filterCoeff[35] * (clipALF(filterClipp[35], curr, pImg1[+5], pImg2[-5]));

          sum += filterCoeff[36] * (clipALF(filterClipp[36], curr, pImg0[-6], pImg0[+6]));
          sum += filterCoeff[37] * (clipALF(filterClipp[37], curr, pImg0[-5], pImg0[+5]));
          sum += filterCoeff[38] * (clipALF(filterClipp[38], curr, pImg0[-4], pImg0[+4]));
          sum += filterCoeff[39] * (clipALF(filterClipp[39], curr, pImg0[-3], pImg0[+3]));
          sum += filterCoeff[40] * (clipALF(filterClipp[40], curr, pImg0[-2], pImg0[+2]));
          sum += filterCoeff[41] * (clipALF(filterClipp[41], curr, pImg0[-1], pImg0[+1]));
#endif

          sum = (sum + offset) >> shift;
          sum += curr;

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          fixedFilterResiResults[fixedFiltInd][posYDst + ii + i][posXDst + jj + j] = Clip3<int>(-clpRng.max, clpRng.max, sum);
#else
          fixedFilterResiResults[fixedFiltInd][posY + ii + i][posX + jj + j] = Clip3<int>(-clpRng.max, clpRng.max, sum);
#endif
#else
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
          fixedFilterResiResults[fixedFiltInd][posYDst + ii + i][posXDst + jj + j] = Clip3<int>(-1024, +1024, sum);
#else
          fixedFilterResiResults[fixedFiltInd][posY + ii + i][posX + jj + j] = ClipPel(sum, clpRng);
#endif
#endif

          pImg0++;
        }   // jj
      }     // ii
    }       // j
    pImgYPad0 += srcStride2;
  }
}
#endif

void AdaptiveLoopFilter::deriveClassificationLaplacianBig(const Area &curBlk, uint32_t **laplacian[NUM_DIRECTIONS])
{
  int fl2 = ALF_CLASSIFIER_FL << 1;
  for (int i = 0; i < (curBlk.height + fl2) >> 1; i++)
  {
    for (int j = 4; j <= (curBlk.width + fl2 - 2) >> 1; j++)
    {
      int jM2 = j - 2;
      int jM4 = j - 4;
      laplacian[VER][i][jM4] += laplacian[VER][i][jM2] + laplacian[VER][i][j];
      laplacian[HOR][i][jM4] += laplacian[HOR][i][jM2] + laplacian[HOR][i][j];
      laplacian[DIAG0][i][jM4] += laplacian[DIAG0][i][jM2] + laplacian[DIAG0][i][j];
      laplacian[DIAG1][i][jM4] += laplacian[DIAG1][i][jM2] + laplacian[DIAG1][i][j];
    }
    if (i >= 4)
    {
      int iM4 = i - 4;
      int iM2 = i - 2;
      for (int j = 0; j <= (curBlk.width + fl2 - 11) >> 1; j++)
      {
        laplacian[VER][iM4][j] += laplacian[VER][iM2][j] + laplacian[VER][i][j];
        laplacian[HOR][iM4][j] += laplacian[HOR][iM2][j] + laplacian[HOR][i][j];
        laplacian[DIAG0][iM4][j] += laplacian[DIAG0][iM2][j] + laplacian[DIAG0][i][j];
        laplacian[DIAG1][iM4][j] += laplacian[DIAG1][iM2][j] + laplacian[DIAG1][i][j];
      }
    }
  }
}

void AdaptiveLoopFilter::deriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS])
{
  int fl = ALF_CLASSIFIER_FL;
  int fl2 = 2 * fl;
  const Pel* src = srcLuma.buf;
  int stride = srcLuma.stride;
  int yOffset = (blk.pos().y - fl) * stride;
  const Pel *src0 = &src[yOffset - stride];
  const Pel *src1 = &src[yOffset];
  const Pel *src2 = &src[yOffset + stride];
  int stride2 = 2 * stride;
  const Pel *src3 = &src[yOffset + stride2];
  for (int i = 0; i < blk.height + fl2; i += 2)
  {
    for (int j = 0; j < blk.width + fl2; j += 2)
    {
      int xOffset = blk.pos().x - fl + j;
      const Pel *pY = src1 + xOffset;
      const Pel *pYup = src0 + xOffset;
      const Pel *pYdn = src2 + xOffset;
      const Pel *pYdn2 = src3 + xOffset;
      Pel y0 = pY[0] << 1;
      Pel y01 = pY[1] << 1;
      Pel y1 = pYdn[0] << 1;
      Pel y11 = pYdn[1] << 1;
      laplacian[VER][i >> 1][j >> 1] = abs(y0 - pYup[0] - pYdn[0]) + abs(y01 - pYup[1] - pYdn[1]) + abs(y1 - pY[0] - pYdn2[0]) + abs(y11 - pY[1] - pYdn2[1]);
      laplacian[HOR][i >> 1][j >> 1] = abs(y0 - pY[-1] - pY[1]) + abs(y01 - pY[0] - pY[2]) + abs(y1 - pYdn[-1] - pYdn[1]) + abs(y11 - pYdn[0] - pYdn[2]);
      laplacian[DIAG0][i >> 1][j >> 1] = abs(y0 - pYup[-1] - pYdn[1]) + abs(y01 - pYup[0] - pYdn[2]) + abs(y1 - pY[-1] - pYdn2[1]) + abs(y11 - pY[0] - pYdn2[2]);
      laplacian[DIAG1][i >> 1][j >> 1] = abs(y0 - pYup[1] - pYdn[-1]) + abs(y01 - pYup[2] - pYdn[0]) + abs(y1 - pY[1] - pYdn2[-1]) + abs(y11 - pY[2] - pYdn2[0]);
    }
    src0 = src0 + stride2;
    src1 = src1 + stride2;
    src2 = src2 + stride2;
    src3 = src3 + stride2;
  }

  for (int i = 0; i < (blk.height + fl2) >> 1; i++)
  {
    for (int j = 1; j < (blk.width + fl2) >> 1; j++)
    {
      int jM1 = j - 1;
      laplacian[VER][i][jM1] = laplacian[VER][i][jM1] + laplacian[VER][i][j];
      laplacian[HOR][i][jM1] = laplacian[HOR][i][jM1] + laplacian[HOR][i][j];
      laplacian[DIAG0][i][jM1] = laplacian[DIAG0][i][jM1] + laplacian[DIAG0][i][j];
      laplacian[DIAG1][i][jM1] = laplacian[DIAG1][i][jM1] + laplacian[DIAG1][i][j];
    }
    if (i > 0)
    {
      int iM1 = i - 1;
      for (int j = 0; j < (blk.width + fl2) >> 1; j++)
      {
        laplacian[VER][iM1][j] = laplacian[VER][iM1][j] + laplacian[VER][i][j];
        laplacian[HOR][iM1][j] = laplacian[HOR][iM1][j] + laplacian[HOR][i][j];
        laplacian[DIAG0][iM1][j] = laplacian[DIAG0][iM1][j] + laplacian[DIAG0][i][j];
        laplacian[DIAG1][iM1][j] = laplacian[DIAG1][iM1][j] + laplacian[DIAG1][i][j];
      }
    }
  }
}

#if JVET_X0071_ALF_BAND_CLASSIFIER
void AdaptiveLoopFilter::deriveClassificationAndFixFilterResultsBlk( AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const bool bResiFixed, Pel ***fixedFilterResiResults, const CPelBuf &srcResiLuma,
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  const CPelBuf &srcLumaBeforeDb, const uint8_t ctuPadFlag,
#endif
  const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx, const int multipleClassifierIdx )
#else
void AdaptiveLoopFilter::deriveClassificationAndFixFilterResultsBlk( AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx )
#endif
{
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  m_deriveVariance(srcLuma, blkDst, blk, laplacian);
#endif
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  m_deriveClassificationLaplacian(srcLuma, blkDst, blk, laplacian);
#else
  m_deriveClassificationLaplacian(srcLuma, blk, blkDst, laplacian);
#endif 
  
  int fixedFiltSetIndMin = (qp - 22) / 4;
  int fixedFiltSetIndMax = (qp - 18) / 4;
  if (fixedFiltSetIndMin <= 0)
  {
    fixedFiltSetIndMin = 0;
    fixedFiltSetIndMax = 1;
  }

  if (fixedFiltSetIndMax > (NUM_SETS_FIXED_FILTERS - 1))
  {
    fixedFiltSetIndMin = NUM_SETS_FIXED_FILTERS - 2;
    fixedFiltSetIndMax = NUM_SETS_FIXED_FILTERS - 1;
  }

  int targetFixedFilterSetInd = fixedFilterSetIdx == 0 ? fixedFiltSetIndMin : (fixedFilterSetIdx == 1 ? fixedFiltSetIndMax : -1);
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  int targetFixedFilterSetIndReverse = fixedFilterSetIdx == 0 ? fixedFiltSetIndMax : (fixedFilterSetIdx == 1 ? fixedFiltSetIndMin : -1);
#endif
      
  int fixedFiltInd = 0;
  for (int dirWindSize = 0; dirWindSize < NUM_CLASSIFIER; dirWindSize++)
  {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    int storeIdx = dirWindSize == 0 ? 0 : ALF_NUM_CLASSIFIER;
#endif
    bool reuse = false;
    for (int fixedFiltSetInd = fixedFiltSetIndMin; fixedFiltSetInd <= fixedFiltSetIndMax; fixedFiltSetInd++)
    {
      if (dirWindSize == classifierIdx || classifierIdx == -1)
      {
        if (fixedFiltSetInd == targetFixedFilterSetInd || targetFixedFilterSetInd == -1)
        {
          if (!reuse)
          {
            if (dirWindSize > 0)
            {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              m_deriveClassificationLaplacianBig(blk, laplacian);
#else
              m_deriveClassificationLaplacianBig(blkDst, laplacian);
#endif 
            }
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            m_calcClass0(classifier[storeIdx], blkDst, blk, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian);
#else
            m_calcClass0(classifier[0], blkDst, blk, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2,
                         mappingDir, laplacian);
#endif
#else
            m_calcClass0(classifier[0], blkDst, blkDst, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits,
                         2, mappingDir, laplacian);
#endif
#else
            m_calcClass0(classifier, blkDst, blkDst, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2,
                         mappingDir, laplacian);
#endif

            reuse = true;
          }
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          int xShift = 0;
          int yShift = 0;
          if( ctuPadFlag > 0 )
          {
            if( dirWindSize == 0 )
            {
              if( ctuPadFlag & 0x01 )
              {
                xShift = 16;
              }
              if( ctuPadFlag & 0x02 )
              {
                yShift = 16;
              }
            }
            else
            {
              if( ctuPadFlag & 0x01 )
              {
                xShift = 8;
              }
              if( ctuPadFlag & 0x02 )
              {
                yShift = 8;
              }
            }
          }
          Area blkDstNew( blkDst.x + xShift, blkDst.y + yShift, blkDst.width - xShift, blkDst.height - yShift );
          Area blkNew( blk.x + xShift, blk.y + yShift, blk.width - xShift, blk.height - yShift );
          if( dirWindSize == 0 )
          {
#endif
          //fixed filtering
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            alfFixedFilterBlk( classifier[storeIdx], srcLuma, blkNew, blkDstNew, srcLumaBeforeDb, fixedFilterResults, m_picWidth, fixedFiltInd, fixedFiltSetInd, dirWindSize, clpRng, clippingValues );
#else
          m_filter13x13Blk(classifier[0], srcLuma,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                           blk,
#else
                           blkDst,
#endif

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                           blkDst,
#endif
                           fixedFilterResults, m_picWidth, fixedFiltInd, m_classIdnFixedFilter[fixedFiltSetInd][dirWindSize], fixedFiltSetInd, dirWindSize, clpRng, clippingValues);
#endif
#else
          m_filter13x13Blk(classifier, srcLuma,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                           blk,
#else
                           blkDst,
#endif
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                           blkDst,
#endif
                           fixedFilterResults, m_picWidth, fixedFiltInd, m_classIdnFixedFilter[fixedFiltSetInd][dirWindSize], fixedFiltSetInd, dirWindSize, clpRng, clippingValues);
#endif
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
          }
#endif
        }
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
        if (fixedFiltSetInd == targetFixedFilterSetIndReverse || targetFixedFilterSetIndReverse == -1)
        {
          if (!reuse)
          {
            if (dirWindSize > 0)
            {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
              m_deriveClassificationLaplacianBig(blk, laplacian);
#else
              m_deriveClassificationLaplacianBig(blkDst, laplacian);
#endif
            }
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            m_calcClass0(classifier[storeIdx], blkDst, blk, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian);
#else
            m_calcClass0(classifier[0], blkDst, blk, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2,
                         mappingDir, laplacian);
#endif
#else
            m_calcClass0(classifier[0], blkDst, blkDst, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits,
                         2, mappingDir, laplacian);
#endif
#else
            m_calcClass0(classifier, blkDst, blkDst, usedWindowIdx[dirWindSize], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2,
                         mappingDir, laplacian);
#endif

            reuse = true;
          }
          if (bResiFixed && dirWindSize == 0)
          {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            m_filterResi9x9Blk(classifier[0], srcResiLuma, blk, blkDst, fixedFilterResiResults, m_picWidth, fixedFiltInd, m_classIdnFixedFilter9Db9[fixedFiltSetInd], fixedFiltSetInd, dirWindSize, clpRng, clippingValues);
#else
            m_filterResi13x13Blk(classifier[0], srcResiLuma,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                 blk,
#else
                                 blkDst,
#endif

#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                                 blkDst,
#endif
                                 fixedFilterResiResults, m_picWidth, fixedFiltInd, m_classIdnFixedFilter[fixedFiltSetInd][dirWindSize], fixedFiltSetInd, dirWindSize, clpRng, clippingValues);
#endif
          }
        }
#endif
      }
      fixedFiltInd++;
    }
  }
  if( classifierIdx == -1 )
  {
#if JVET_X0071_ALF_BAND_CLASSIFIER
#if JVET_AD0222_ALF_RESI_CLASS
    if( multipleClassifierIdx == ALF_NUM_CLASSIFIER || multipleClassifierIdx == 0)
#else
    if( multipleClassifierIdx == ALF_NUM_CLASSIFIER || multipleClassifierIdx == 0 || multipleClassifierIdx == 1 )
#endif
    {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
      m_calcClass1(classifier[0], blkDst, Area(blk.pos().x, blk.pos().y, blk.width, blk.height), 5, 0, 5, 5, bits, 2,
                   mappingDir, laplacian);
#else
      m_calcClass1(classifier[0], blkDst, Area(blkDst.pos().x, blkDst.pos().y, blkDst.width, blkDst.height), 5, 0, 5, 5,
                   bits, 2, mappingDir, laplacian);
#endif
    }
    for( int curClassifierIdx = 1; curClassifierIdx < ALF_NUM_CLASSIFIER; curClassifierIdx++ )
    {
      if( multipleClassifierIdx == ALF_NUM_CLASSIFIER || curClassifierIdx == multipleClassifierIdx )
      {
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
#if JVET_AD0222_ALF_RESI_CLASS
        if( multipleClassifierIdx == ALF_NUM_CLASSIFIER && curClassifierIdx == 2 && cs.slice->isIntra() )
        {
          continue;
        }
        m_calcClass2( classifier[curClassifierIdx], blk, Area( blkDst.pos().x, blkDst.pos().y, blkDst.width, blkDst.height ), srcLuma, 2, classifier[0], curClassifierIdx, bits, srcResiLuma, laplacian[0] );
#else
        m_calcClass2( classifier[curClassifierIdx], blk, Area( blkDst.pos().x, blkDst.pos().y, blkDst.width, blkDst.height ), srcLuma, 2, classifier[0], curClassifierIdx, bits );
#endif
#else
        m_calcClass2( classifier[curClassifierIdx], blkDst, Area( blkDst.pos().x, blkDst.pos().y, blkDst.width, blkDst.height ), srcLuma, 2, classifier[0], curClassifierIdx, bits );
#endif
      }
    }
#else
    m_calcClass1( classifier, blkDst, Area(blkDst.pos().x, blkDst.pos().y, blkDst.width, blkDst.height), 5, 0, 5, 5, bits, 2, mappingDir, laplacian );
#endif
  }
}
#else
void AdaptiveLoopFilter::deriveClassificationBlk(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");
  static const int th[16] = { 0, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4 };
  const int stride = srcLuma.stride;
  const Pel* src = srcLuma.buf;
  const int maxActivity = 15;

  int fl = 2;
  int flP1 = fl + 1;
  int fl2 = 2 * fl;

  int mainDirection, secondaryDirection, dirTempHV, dirTempD;

  int pixY;
  int height = blk.height + fl2;
  int width = blk.width + fl2;
  int posX = blk.pos().x;
  int posY = blk.pos().y;
  int startHeight = posY - flP1;

  for( int i = 0; i < height; i += 2 )
  {
    int yoffset = ( i + 1 + startHeight ) * stride - flP1;
    const Pel *src0 = &src[yoffset - stride];
    const Pel *src1 = &src[yoffset];
    const Pel *src2 = &src[yoffset + stride];
    const Pel *src3 = &src[yoffset + stride * 2];

    const int y = blkDst.pos().y - 2 + i;
    if (y > 0 && (y & (vbCTUHeight - 1)) == vbPos - 2)
    {
      src3 = &src[yoffset + stride];
    }
    else if (y > 0 && (y & (vbCTUHeight - 1)) == vbPos)
    {
      src0 = &src[yoffset];
    }
    int* pYver = laplacian[VER][i];
    int* pYhor = laplacian[HOR][i];
    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig1 = laplacian[DIAG1][i];

    for( int j = 0; j < width; j += 2 )
    {
      pixY = j + 1 + posX;
      const Pel *pY = src1 + pixY;
      const Pel* pYdown = src0 + pixY;
      const Pel* pYup = src2 + pixY;
      const Pel* pYup2 = src3 + pixY;

      const Pel y0 = pY[0] << 1;
      const Pel yup1 = pYup[1] << 1;

      pYver[j] = abs( y0 - pYdown[0] - pYup[0] ) + abs( yup1 - pY[1] - pYup2[1] );
      pYhor[j] = abs( y0 - pY[1] - pY[-1] ) + abs( yup1 - pYup[2] - pYup[0] );
      pYdig0[j] = abs( y0 - pYdown[-1] - pYup[1] ) + abs( yup1 - pY[0] - pYup2[2] );
      pYdig1[j] = abs( y0 - pYup[-1] - pYdown[1] ) + abs( yup1 - pYup2[0] - pY[2] );

      if( j > 4 && ( j - 6 ) % 4 == 0 )
      {
        int jM6 = j - 6;
        int jM4 = j - 4;
        int jM2 = j - 2;
        pYver[jM6] += pYver[jM4] + pYver[jM2] + pYver[j];
        pYhor[jM6] += pYhor[jM4] + pYhor[jM2] + pYhor[j];
        pYdig0[jM6] += pYdig0[jM4] + pYdig0[jM2] + pYdig0[j];
        pYdig1[jM6] += pYdig1[jM4] + pYdig1[jM2] + pYdig1[j];
      }
    }
  }
  // classification block size
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  for( int i = 0; i < blk.height; i += clsSizeY )
  {
    int* pYver = laplacian[VER][i];
    int* pYver2 = laplacian[VER][i + 2];
    int* pYver4 = laplacian[VER][i + 4];
    int* pYver6 = laplacian[VER][i + 6];

    int* pYhor = laplacian[HOR][i];
    int* pYhor2 = laplacian[HOR][i + 2];
    int* pYhor4 = laplacian[HOR][i + 4];
    int* pYhor6 = laplacian[HOR][i + 6];

    int* pYdig0 = laplacian[DIAG0][i];
    int* pYdig02 = laplacian[DIAG0][i + 2];
    int* pYdig04 = laplacian[DIAG0][i + 4];
    int* pYdig06 = laplacian[DIAG0][i + 6];

    int* pYdig1 = laplacian[DIAG1][i];
    int* pYdig12 = laplacian[DIAG1][i + 2];
    int* pYdig14 = laplacian[DIAG1][i + 4];
    int* pYdig16 = laplacian[DIAG1][i + 6];

    for( int j = 0; j < blk.width; j += clsSizeX )
    {
      int sumV = 0; int sumH = 0; int sumD0 = 0; int sumD1 = 0;

      if (((i + blkDst.pos().y) % vbCTUHeight) == (vbPos - 4))
      {
        sumV = pYver[j] + pYver2[j] + pYver4[j];
        sumH = pYhor[j] + pYhor2[j] + pYhor4[j];
        sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j];
        sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j];
      }
      else if (((i + blkDst.pos().y) % vbCTUHeight) == vbPos)
      {
        sumV = pYver2[j] + pYver4[j] + pYver6[j];
        sumH = pYhor2[j] + pYhor4[j] + pYhor6[j];
        sumD0 = pYdig02[j] + pYdig04[j] + pYdig06[j];
        sumD1 = pYdig12[j] + pYdig14[j] + pYdig16[j];
      }
      else
      {
        sumV = pYver[j] + pYver2[j] + pYver4[j] + pYver6[j];
        sumH = pYhor[j] + pYhor2[j] + pYhor4[j] + pYhor6[j];
        sumD0 = pYdig0[j] + pYdig02[j] + pYdig04[j] + pYdig06[j];
        sumD1 = pYdig1[j] + pYdig12[j] + pYdig14[j] + pYdig16[j];
      }

      int tempAct = sumV + sumH;
      int activity = 0;

      const int y = (i + blkDst.pos().y) & (vbCTUHeight - 1);
      if (y == vbPos - 4 || y == vbPos)
      {
        activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 96) >> shift);
      }
      else
      {
        activity = (Pel)Clip3<int>(0, maxActivity, (tempAct * 64) >> shift);
      }

      int classIdx = th[activity];

      int hv1, hv0, d1, d0, hvd1, hvd0;

      if( sumV > sumH )
      {
        hv1 = sumV;
        hv0 = sumH;
        dirTempHV = 1;
      }
      else
      {
        hv1 = sumH;
        hv0 = sumV;
        dirTempHV = 3;
      }
      if( sumD0 > sumD1 )
      {
        d1 = sumD0;
        d0 = sumD1;
        dirTempD = 0;
      }
      else
      {
        d1 = sumD1;
        d0 = sumD0;
        dirTempD = 2;
      }
      if( (uint32_t)d1 * (uint32_t)hv0 > (uint32_t)hv1 * (uint32_t)d0 )
      {
        hvd1 = d1;
        hvd0 = d0;
        mainDirection = dirTempD;
        secondaryDirection = dirTempHV;
      }
      else
      {
        hvd1 = hv1;
        hvd0 = hv0;
        mainDirection = dirTempHV;
        secondaryDirection = dirTempD;
      }

      int directionStrength = 0;
      if( hvd1 > 2 * hvd0 )
      {
        directionStrength = 1;
      }
      if( hvd1 * 2 > 9 * hvd0 )
      {
        directionStrength = 2;
      }

      if( directionStrength )
      {
        classIdx += ( ( ( mainDirection & 0x1 ) << 1 ) + directionStrength ) * 5;
      }

      static const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx = transposeTable[mainDirection * 2 + ( secondaryDirection >> 1 )];
      int yOffset = i + blkDst.pos().y;
      int xOffset = j + blkDst.pos().x;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;
      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] = cl3[0] = cl3[1] = cl3[2] = cl3[3] = AlfClassifier( classIdx, transposeIdx );
    }
  }
}
#endif

template<AlfFilterType filtType>
void AdaptiveLoopFilter::filterBlk(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const PelUnitBuf &recBeforeDb,
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const PelUnitBuf &resi,
#endif
  const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
                                   const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng,
#else
                                   const short *filterSet, const short *fClipSet, const ClpRng &clpRng,
#endif
                                   CodingStructure &cs, 
#if ALF_IMPROVEMENT
                                   Pel ***fixedFilterResults, 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
                                   Pel ***fixedFilterResiResults,
#endif
  int fixedFilterSetIdx
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
                                 , Pel ***fixedFilterResultsPerCtu, bool isFixedFilterPaddedPerCtu
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
                                 , Pel ***gaussPic, Pel ***gaussCtu
#endif
   )
#else
                                   const int vbCTUHeight, int vbPos)
#endif
{
#if !ALF_IMPROVEMENT
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");
#endif

  const bool bChroma = isChroma( compId );
#if !ALF_IMPROVEMENT
  if( bChroma )
  {
    CHECK( filtType != 0, "Chroma needs to have filtType == 0" );
  }
#endif

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride;

#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  const int recDbStride = isLuma( compId ) ? recBeforeDb.get( compId ).stride : 0;
  Pel* recDb = isLuma( compId ) ? recBeforeDb.get( compId ).buf + blk.y * recDbStride : nullptr;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  const int resiStride = isLuma( compId ) ? resi.get(compId).stride : 0;
  Pel      *resiC      = isLuma( compId ) ? resi.get(compId).buf + blk.y * resiStride : nullptr;
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
  int padSizeGauss = ALF_PADDING_SIZE_GAUSS_RESULTS;
#endif

#if ALF_IMPROVEMENT
  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6, *pImgYPad7, *pImgYPad8;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6, *pImg7, *pImg8;
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER
  const Pel *pImgYPad9, *pImgYPad10, *pImgYPad11, *pImgYPad12;
  const Pel *pImg9, *pImg10, *pImg11, *pImg12;
#endif
#else
  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;
#endif

  const short *coef = filterSet;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const Pel *clip = fClipSet;
#else
  const short *clip = fClipSet;
#endif
  const int shift = m_NUM_BITS - 1;

  const int offset = 1 << ( shift - 1 );

  int transposeIdx = 0;
#if ALF_IMPROVEMENT
  const int clsSizeY = 2;
  const int clsSizeX = 2;
#else
  const int clsSizeY = 4;
  const int clsSizeX = 4;
#endif


  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  int recDbStride2 = recDbStride * clsSizeY;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  int resiStride2 = resiStride * clsSizeY;
#endif

  std::array<int, MAX_NUM_ALF_LUMA_COEFF> filterCoeff;
  std::array<int, MAX_NUM_ALF_LUMA_COEFF> filterClipp;

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;
#if ALF_IMPROVEMENT
  pImgYPad7 = pImgYPad5 + srcStride;
  pImgYPad8 = pImgYPad6 - srcStride;
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER
  pImgYPad9  = pImgYPad7 + srcStride;
  pImgYPad10 = pImgYPad8 - srcStride;
  pImgYPad11 = pImgYPad9 + srcStride;
  pImgYPad12 = pImgYPad10 - srcStride;
#endif
#endif

  Pel* pRec0 = dst + blkDst.x;
  Pel* pRec1 = pRec0 + dstStride;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  Pel* pRecDb0 = isLuma( compId ) ? recDb + blk.x : nullptr;
  Pel* pRecDb1 = isLuma( compId ) ? pRecDb0 + recDbStride : nullptr;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
  Pel *pResi0 = isLuma(compId) ? resiC + blk.x : nullptr;
  Pel *pResi1 = isLuma(compId) ? pResi0 + resiStride : nullptr;
#endif

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
    {
      if( !bChroma )
      {
        AlfClassifier& cl = pClass[j];
#if ALF_IMPROVEMENT
        transposeIdx = cl & 0x3;
        int classIdx = cl >> 2;
        coef = filterSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
        clip = fClipSet + classIdx * MAX_NUM_ALF_LUMA_COEFF;
#else
        transposeIdx = cl.transposeIdx;
        coef = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
        clip = fClipSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
#endif
      }

#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_LONGER_FILTER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
      if( filtType == ALF_FILTER_13_EXT || filtType == ALF_FILTER_13_EXT_DB 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
          || filtType == ALF_FILTER_13_EXT_DB_RESI || filtType == ALF_FILTER_13_EXT_DB_RESI_DIRECT
#endif
        )
#else
      if( filtType == ALF_FILTER_13_EXT )
#endif
      {
#if JVET_AD0222_ALF_LONG_FIXFILTER
        if (transposeIdx == 1)
        {
          filterCoeff = { coef[6], coef[7], coef[8], coef[3], coef[9], coef[5], coef[0], coef[1], coef[2], coef[4], coef[10] };
          filterClipp = { clip[6], clip[7], clip[8], clip[3], clip[9], clip[5], clip[0], clip[1], clip[2], clip[4], clip[10] };
        }
        else if (transposeIdx == 2)
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[5], coef[4], coef[3], coef[6], coef[7], coef[8], coef[9], coef[10] };
          filterClipp = { clip[0], clip[1], clip[2], clip[5], clip[4], clip[3], clip[6], clip[7], clip[8], clip[9], clip[10] };
        }
        else if (transposeIdx == 3)
        {
          filterCoeff = { coef[6], coef[7], coef[8], coef[5], coef[9], coef[3], coef[0], coef[1], coef[2], coef[4], coef[10] };
          filterClipp = { clip[6], clip[7], clip[8], clip[5], clip[9], clip[3], clip[0], clip[1], clip[2], clip[4], clip[10] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6], clip[7], clip[8], clip[9], clip[10] };
        }
#else
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[14], coef[15], coef[16], coef[17], coef[4], coef[9], coef[18], coef[13], coef[8], coef[5], coef[10], coef[19], coef[12], coef[7], coef[0], coef[1], coef[2], coef[3], coef[6], coef[11], coef[20] };
          filterClipp = { clip[14], clip[15], clip[16], clip[17], clip[4], clip[9], clip[18], clip[13], clip[8], clip[5], clip[10], clip[19], clip[12], clip[7], clip[0], clip[1], clip[2], clip[3], clip[6], clip[11], clip[20] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[8], coef[7], coef[6], coef[5], coef[4], coef[13], coef[12], coef[11], coef[10], coef[9], coef[14], coef[15], coef[16], coef[17], coef[18], coef[19], coef[20] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[8], clip[7], clip[6], clip[5], clip[4], clip[13], clip[12], clip[11], clip[10], clip[9], clip[14], clip[15], clip[16], clip[17], clip[18], clip[19], clip[20] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[14], coef[15], coef[16], coef[17], coef[8], coef[13], coef[18], coef[9], coef[4], coef[7], coef[12], coef[19], coef[10], coef[5], coef[0], coef[1], coef[2], coef[3], coef[6], coef[11], coef[20] };
          filterClipp = { clip[14], clip[15], clip[16], clip[17], clip[8], clip[13], clip[18], clip[9], clip[4], clip[7], clip[12], clip[19], clip[10], clip[5], clip[0], clip[1], clip[2], clip[3], clip[6], clip[11], clip[20] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10], coef[11], coef[12], coef[13], coef[14], coef[15], coef[16], coef[17], coef[18], coef[19], coef[20] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6], clip[7], clip[8], clip[9], clip[10], clip[11], clip[12], clip[13], clip[14], clip[15], clip[16], clip[17], clip[18], clip[19], clip[20] };
        }
#endif
      }
      else 
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
      if( filtType == ALF_FILTER_9 || filtType == ALF_FILTER_9_EXT || filtType == ALF_FILTER_9_EXT_DB )
#else
      if( filtType == ALF_FILTER_9 || filtType == ALF_FILTER_9_EXT )
#endif
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[16], coef[9], coef[17], coef[15], coef[4], coef[10], coef[18], coef[14], coef[8], coef[1], coef[5], coef[11], coef[19], coef[13], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12], coef[20] };
          filterClipp = { clip[16], clip[9], clip[17], clip[15], clip[4], clip[10], clip[18], clip[14], clip[8], clip[1], clip[5], clip[11], clip[19], clip[13], clip[7], clip[3], clip[0], clip[2], clip[6], clip[12], clip[20] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[15], coef[14] , coef[13], coef[12], coef[11], coef[10] , coef[9], coef[16], coef[17], coef[18], coef[19], coef[20] };
          filterClipp = { clip[0], clip[3], clip[2], clip[1], clip[8], clip[7], clip[6], clip[5], clip[4], clip[15], clip[14] , clip[13], clip[12], clip[11], clip[10] , clip[9], clip[16], clip[17], clip[18], clip[19], clip[20] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[16], coef[15], coef[17], coef[9], coef[8], coef[14], coef[18], coef[10], coef[4], coef[3], coef[7], coef[13], coef[19], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12], coef[20] };
          filterClipp = { clip[16], clip[15], clip[17], clip[9], clip[8], clip[14], clip[18], clip[10], clip[4], clip[3], clip[7], clip[13], clip[19], clip[11], clip[5], clip[1], clip[0], clip[2], clip[6], clip[12], clip[20] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10], coef[11], coef[12], coef[13], coef[14], coef[15], coef[16], coef[17], coef[18], coef[19], coef[20] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6], clip[7], clip[8], clip[9], clip[10], clip[11], clip[12], clip[13], clip[14], clip[15], clip[16], clip[17], clip[18], clip[19], clip[20] };
        }
      }
      else
#endif
      if( filtType == ALF_FILTER_7 )
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[9], coef[4], coef[10], coef[8], coef[1], coef[5], coef[11], coef[7], coef[3], coef[0], coef[2], coef[6], coef[12] };
          filterClipp = { clip[9], clip[4], clip[10], clip[8], clip[1], clip[5], clip[11], clip[7], clip[3], clip[0], clip[2], clip[6], clip[12] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[8], coef[7], coef[6], coef[5], coef[4], coef[9], coef[10], coef[11], coef[12] };
          filterClipp = { clip[0], clip[3], clip[2], clip[1], clip[8], clip[7], clip[6], clip[5], clip[4], clip[9], clip[10], clip[11], clip[12] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[9], coef[8], coef[10], coef[4], coef[3], coef[7], coef[11], coef[5], coef[1], coef[0], coef[2], coef[6], coef[12] };
          filterClipp = { clip[9], clip[8], clip[10], clip[4], clip[3], clip[7], clip[11], clip[5], clip[1], clip[0], clip[2], clip[6], clip[12] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6], coef[7], coef[8], coef[9], coef[10], coef[11], coef[12] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6], clip[7], clip[8], clip[9], clip[10], clip[11], clip[12] };
        }
      }
      else
      {
        if( transposeIdx == 1 )
        {
          filterCoeff = { coef[4], coef[1], coef[5], coef[3], coef[0], coef[2], coef[6] };
          filterClipp = { clip[4], clip[1], clip[5], clip[3], clip[0], clip[2], clip[6] };
        }
        else if( transposeIdx == 2 )
        {
          filterCoeff = { coef[0], coef[3], coef[2], coef[1], coef[4], coef[5], coef[6] };
          filterClipp = { clip[0], clip[3], clip[2], clip[1], clip[4], clip[5], clip[6] };
        }
        else if( transposeIdx == 3 )
        {
          filterCoeff = { coef[4], coef[3], coef[5], coef[1], coef[0], coef[2], coef[6] };
          filterClipp = { clip[4], clip[3], clip[5], clip[1], clip[0], clip[2], clip[6] };
        }
        else
        {
          filterCoeff = { coef[0], coef[1], coef[2], coef[3], coef[4], coef[5], coef[6] };
          filterClipp = { clip[0], clip[1], clip[2], clip[3], clip[4], clip[5], clip[6] };
        }
      }

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;
        pImg5 = pImgYPad5 + j + ii * srcStride;
        pImg6 = pImgYPad6 + j + ii * srcStride;
#if ALF_IMPROVEMENT
        pImg7 = pImgYPad7 + j + ii * srcStride;
        pImg8 = pImgYPad8 + j + ii * srcStride;
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER
        pImg9  = pImgYPad9  + j + ii * srcStride;
        pImg10 = pImgYPad10 + j + ii * srcStride;
        pImg11 = pImgYPad11 + j + ii * srcStride;
        pImg12 = pImgYPad12 + j + ii * srcStride;
#endif
#endif

        pRec1 = pRec0 + j + ii * dstStride;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
        pRecDb1 = pRecDb0 + j + ii * recDbStride;
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
        pResi1 = pResi0 + j + ii * resiStride;
#endif
#if !ALF_IMPROVEMENT
        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - (bChroma ? 2 : 4)))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;
          pImg5 = (yVb >= vbPos - 3) ? pImg3 : pImg5;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
          pImg6 = (yVb >= vbPos - 3) ? pImg4 : pImg6;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + (bChroma ? 1 : 3)))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;
          pImg6 = (yVb <= vbPos + 2) ? pImg4 : pImg6;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
          pImg5 = (yVb <= vbPos + 2) ? pImg3 : pImg5;
        }
        bool isNearVBabove = yVb < vbPos && (yVb >= vbPos - 1);
        bool isNearVBbelow = yVb >= vbPos && (yVb <= vbPos);
#endif

        for( int jj = 0; jj < clsSizeX; jj++ )
        {
          int32_t   sum  = 0;
          const Pel curr = pImg0[+0];
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_LONGER_FILTER
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
          if( filtType == ALF_FILTER_13_EXT || filtType == ALF_FILTER_13_EXT_DB 
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
              || filtType == ALF_FILTER_13_EXT_DB_RESI
              || filtType == ALF_FILTER_13_EXT_DB_RESI_DIRECT
#endif
            )
#else
          if( filtType == ALF_FILTER_13_EXT )
#endif
          {
#if JVET_AD0222_ALF_LONG_FIXFILTER
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg7[+0], pImg8[-0]) );
            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg5[+0], pImg6[-0]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg3[+0], pImg4[-0]) );

            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg1[+0], pImg2[-0]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg1[-1], pImg2[+1]) );

            sum += filterCoeff[6] * ( clipALF(filterClipp[6], curr, pImg0[+4], pImg0[-4]) );
            sum += filterCoeff[7] * ( clipALF(filterClipp[7], curr, pImg0[+3], pImg0[-3]) );
            sum += filterCoeff[8] * ( clipALF(filterClipp[8], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[9] * ( clipALF(filterClipp[9], curr, pImg0[+1], pImg0[-1]) );
#else
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg11[+0], pImg12[-0]) );
            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg9[+0], pImg10[-0]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg7[+0], pImg8[-0]) );
            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg5[+0], pImg6[-0]) );

            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg3[+2], pImg4[-2]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg3[+1], pImg4[-1]) );
            sum += filterCoeff[6] * ( clipALF(filterClipp[6], curr, pImg3[+0], pImg4[-0]) );
            sum += filterCoeff[7] * ( clipALF(filterClipp[7], curr, pImg3[-1], pImg4[+1]) );
            sum += filterCoeff[8] * ( clipALF(filterClipp[8], curr, pImg3[-2], pImg4[+2]) );

            sum += filterCoeff[9]  * ( clipALF(filterClipp[9],  curr, pImg1[+2], pImg2[-2]) );
            sum += filterCoeff[10] * ( clipALF(filterClipp[10], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[11] * ( clipALF(filterClipp[11], curr, pImg1[+0], pImg2[-0]) );
            sum += filterCoeff[12] * ( clipALF(filterClipp[12], curr, pImg1[-1], pImg2[+1]) );
            sum += filterCoeff[13] * ( clipALF(filterClipp[13], curr, pImg1[-2], pImg2[+2]) );

            sum += filterCoeff[14] * ( clipALF(filterClipp[14], curr, pImg0[+6], pImg0[-6]) );
            sum += filterCoeff[15] * ( clipALF(filterClipp[15], curr, pImg0[+5], pImg0[-5]) );
            sum += filterCoeff[16] * ( clipALF(filterClipp[16], curr, pImg0[+4], pImg0[-4]) );
            sum += filterCoeff[17] * ( clipALF(filterClipp[17], curr, pImg0[+3], pImg0[-3]) );
            sum += filterCoeff[18] * ( clipALF(filterClipp[18], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[19] * ( clipALF(filterClipp[19], curr, pImg0[+1], pImg0[-1]) );
#endif
          }
          else
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
          if( filtType == ALF_FILTER_9 || filtType == ALF_FILTER_9_EXT || filtType == ALF_FILTER_9_EXT_DB )
#else
          if( filtType == ALF_FILTER_9 || filtType == ALF_FILTER_9_EXT )
#endif
          {
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg7[+0], pImg8[+0]) );

            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg5[+1], pImg6[-1]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg5[+0], pImg6[+0]) );
            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg5[-1], pImg6[+1]) );

            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg3[+2], pImg4[-2]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg3[+1], pImg4[-1]) );
            sum += filterCoeff[6] * ( clipALF(filterClipp[6], curr, pImg3[+0], pImg4[+0]) );
            sum += filterCoeff[7] * ( clipALF(filterClipp[7], curr, pImg3[-1], pImg4[+1]) );
            sum += filterCoeff[8] * ( clipALF(filterClipp[8], curr, pImg3[-2], pImg4[+2]) );

            sum += filterCoeff[9] * ( clipALF(filterClipp[9], curr, pImg1[+3], pImg2[-3]) );
            sum += filterCoeff[10] * ( clipALF(filterClipp[10], curr, pImg1[+2], pImg2[-2]) );
            sum += filterCoeff[11] * ( clipALF(filterClipp[11], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[12] * ( clipALF(filterClipp[12], curr, pImg1[+0], pImg2[+0]) );
            sum += filterCoeff[13] * ( clipALF(filterClipp[13], curr, pImg1[-1], pImg2[+1]) );
            sum += filterCoeff[14] * ( clipALF(filterClipp[14], curr, pImg1[-2], pImg2[+2]) );
            sum += filterCoeff[15] * ( clipALF(filterClipp[15], curr, pImg1[-3], pImg2[+3]) );

            sum += filterCoeff[16] * ( clipALF(filterClipp[16], curr, pImg0[+4], pImg0[-4]) );
            sum += filterCoeff[17] * ( clipALF(filterClipp[17], curr, pImg0[+3], pImg0[-3]) );
            sum += filterCoeff[18] * ( clipALF(filterClipp[18], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[19] * ( clipALF(filterClipp[19], curr, pImg0[+1], pImg0[-1]) );
          }
          else
#endif
          if( filtType == ALF_FILTER_7 )
          {
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg5[+0], pImg6[+0]) );

            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg3[+1], pImg4[-1]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg3[+0], pImg4[+0]) );
            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg3[-1], pImg4[+1]) );

            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg1[+2], pImg2[-2]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[6] * ( clipALF(filterClipp[6], curr, pImg1[+0], pImg2[+0]) );
            sum += filterCoeff[7] * ( clipALF(filterClipp[7], curr, pImg1[-1], pImg2[+1]) );
            sum += filterCoeff[8] * ( clipALF(filterClipp[8], curr, pImg1[-2], pImg2[+2]) );

            sum += filterCoeff[9] * ( clipALF(filterClipp[9], curr, pImg0[+3], pImg0[-3]) );
            sum += filterCoeff[10] * ( clipALF(filterClipp[10], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[11] * ( clipALF(filterClipp[11], curr, pImg0[+1], pImg0[-1]) );
          }
          else
#if ALF_IMPROVEMENT
          if( filtType == ALF_FILTER_5 )
#endif
          {
            sum += filterCoeff[0] * ( clipALF(filterClipp[0], curr, pImg3[+0], pImg4[+0]) );

            sum += filterCoeff[1] * ( clipALF(filterClipp[1], curr, pImg1[+1], pImg2[-1]) );
            sum += filterCoeff[2] * ( clipALF(filterClipp[2], curr, pImg1[+0], pImg2[+0]) );
            sum += filterCoeff[3] * ( clipALF(filterClipp[3], curr, pImg1[-1], pImg2[+1]) );

            sum += filterCoeff[4] * ( clipALF(filterClipp[4], curr, pImg0[+2], pImg0[-2]) );
            sum += filterCoeff[5] * ( clipALF(filterClipp[5], curr, pImg0[+1], pImg0[-1]) );
          }
#if ALF_IMPROVEMENT
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          Pel *pImg0Gauss[NUM_GAUSS_FILTERED_SOURCE];
          Pel *pImg1Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg2Gauss[NUM_GAUSS_FILTERED_SOURCE];
          Pel *pImg3Gauss[NUM_GAUSS_FILTERED_SOURCE], *pImg4Gauss[NUM_GAUSS_FILTERED_SOURCE];

          for( int gaussIdx = 0; gaussIdx < NUM_GAUSS_FILTERED_SOURCE; gaussIdx++)
          {
            if( isFixedFilterPaddedPerCtu )
            {
              pImg0Gauss[gaussIdx] = &gaussCtu[gaussIdx][i + ii + padSizeGauss + 0][j + jj + padSizeGauss];
              pImg1Gauss[gaussIdx] = &gaussCtu[gaussIdx][i + ii + padSizeGauss + 1][j + jj + padSizeGauss];
              pImg2Gauss[gaussIdx] = &gaussCtu[gaussIdx][i + ii + padSizeGauss - 1][j + jj + padSizeGauss];
              pImg3Gauss[gaussIdx] = &gaussCtu[gaussIdx][i + ii + padSizeGauss + 2][j + jj + padSizeGauss];
              pImg4Gauss[gaussIdx] = &gaussCtu[gaussIdx][i + ii + padSizeGauss - 2][j + jj + padSizeGauss];
            }
            else
            {
              pImg0Gauss[gaussIdx] = &gaussPic[gaussIdx][blkDst.y + i + ii + padSizeGauss + 0][blkDst.x + j + jj + padSizeGauss];
              pImg1Gauss[gaussIdx] = &gaussPic[gaussIdx][blkDst.y + i + ii + padSizeGauss + 1][blkDst.x + j + jj + padSizeGauss];
              pImg2Gauss[gaussIdx] = &gaussPic[gaussIdx][blkDst.y + i + ii + padSizeGauss - 1][blkDst.x + j + jj + padSizeGauss];
              pImg3Gauss[gaussIdx] = &gaussPic[gaussIdx][blkDst.y + i + ii + padSizeGauss + 2][blkDst.x + j + jj + padSizeGauss];
              pImg4Gauss[gaussIdx] = &gaussPic[gaussIdx][blkDst.y + i + ii + padSizeGauss - 2][blkDst.x + j + jj + padSizeGauss];
            }
          }
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
#if JVET_AD0222_ALF_LONG_FIXFILTER
          if( filtType >= ALF_FILTER_13_EXT && filtType != ALF_FILTER_9_EXT_DB )
          {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            int filterSetIdx = 2 + fixedFilterSetIdx;
#else
            int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
            Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased, *pImg5FixedBased, *pImg6FixedBased, *pImg7FixedBased, *pImg8FixedBased, *pImg9FixedBased, *pImg10FixedBased, *pImg11FixedBased, *pImg12FixedBased;
            if( isFixedFilterPaddedPerCtu )
            {
              pImg0FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 0][j + jj + padSize];
              pImg1FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 1][j + jj + padSize];
              pImg2FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 1][j + jj + padSize];
              pImg3FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 2][j + jj + padSize];
              pImg4FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 2][j + jj + padSize];
              pImg5FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 3][j + jj + padSize];
              pImg6FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 3][j + jj + padSize];
              pImg7FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 4][j + jj + padSize];
              pImg8FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 4][j + jj + padSize];
              pImg9FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 5][j + jj + padSize];
              pImg10FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 5][j + jj + padSize];
              pImg11FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 6][j + jj + padSize];
              pImg12FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 6][j + jj + padSize];
            }
            else
            {
              pImg0FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 0][blkDst.x + j + jj + padSize];
              pImg1FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 1][blkDst.x + j + jj + padSize];
              pImg2FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 1][blkDst.x + j + jj + padSize];
              pImg3FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 2][blkDst.x + j + jj + padSize];
              pImg4FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 2][blkDst.x + j + jj + padSize];
              pImg5FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 3][blkDst.x + j + jj + padSize];
              pImg6FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 3][blkDst.x + j + jj + padSize];
              pImg7FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 4][blkDst.x + j + jj + padSize];
              pImg8FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 4][blkDst.x + j + jj + padSize];
              pImg9FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 5][blkDst.x + j + jj + padSize];
              pImg10FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 5][blkDst.x + j + jj + padSize];
              pImg11FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 6][blkDst.x + j + jj + padSize];
              pImg12FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 6][blkDst.x + j + jj + padSize];
            }

            if( transposeIdx == 1 )
            {
              sum += coef[22] * ( clipALF(clip[22], curr, pImg11FixedBased[-0], pImg12FixedBased[+0]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg9FixedBased[-0], pImg10FixedBased[+0]) );
              sum += coef[24] * ( clipALF(clip[24], curr, pImg7FixedBased[-0], pImg8FixedBased[+0]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg5FixedBased[-0], pImg6FixedBased[+0]) );
              sum += coef[17] * ( clipALF(clip[17], curr, pImg3FixedBased[+1], pImg4FixedBased[-1]) );
              sum += coef[26] * ( clipALF(clip[26], curr, pImg3FixedBased[+0], pImg4FixedBased[-0]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg3FixedBased[-1], pImg4FixedBased[+1]) );
              sum += coef[14] * ( clipALF(clip[14], curr, pImg1FixedBased[+2], pImg2FixedBased[-2]) );
              sum += coef[18] * ( clipALF(clip[18], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]) );
              sum += coef[27] * ( clipALF(clip[27], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]) );
              sum += coef[20] * ( clipALF(clip[20], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]) );
              sum += coef[16] * ( clipALF(clip[16], curr, pImg1FixedBased[-2], pImg2FixedBased[+2]) );
              sum += coef[10] * ( clipALF(clip[10], curr, pImg0FixedBased[-6], pImg0FixedBased[+6]) );
              sum += coef[11] * (clipALF(clip[11], curr, pImg0FixedBased[-5], pImg0FixedBased[+5]));
              sum += coef[12] * ( clipALF(clip[12], curr, pImg0FixedBased[-4], pImg0FixedBased[+4]) );
              sum += coef[13] * ( clipALF(clip[13], curr, pImg0FixedBased[-3], pImg0FixedBased[+3]) );
              sum += coef[15] * ( clipALF(clip[15], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[19] * ( clipALF(clip[19], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
            else if( transposeIdx == 2 )
            {
              sum += coef[10] * ( clipALF(clip[10], curr, pImg11FixedBased[-0], pImg12FixedBased[+0]) );
              sum += coef[11] * ( clipALF(clip[11], curr, pImg9FixedBased[-0], pImg10FixedBased[+0]) );
              sum += coef[12] * ( clipALF(clip[12], curr, pImg7FixedBased[-0], pImg8FixedBased[+0]) );
              sum += coef[13] * ( clipALF(clip[13], curr, pImg5FixedBased[-0], pImg6FixedBased[+0]) );
              sum += coef[16] * ( clipALF(clip[16], curr, pImg3FixedBased[+1], pImg4FixedBased[-1]) );
              sum += coef[15] * ( clipALF(clip[15], curr, pImg3FixedBased[+0], pImg4FixedBased[-0]) );
              sum += coef[14] * ( clipALF(clip[14], curr, pImg3FixedBased[-1], pImg4FixedBased[+1]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg1FixedBased[+2], pImg2FixedBased[-2]) );
              sum += coef[20] * ( clipALF(clip[20], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]) );
              sum += coef[19] * ( clipALF(clip[19], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]) );
              sum += coef[18] * ( clipALF(clip[18], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]) );
              sum += coef[17] * ( clipALF(clip[17], curr, pImg1FixedBased[-2], pImg2FixedBased[+2]) );
              sum += coef[22] * ( clipALF(clip[22], curr, pImg0FixedBased[-6], pImg0FixedBased[+6]) );
              sum += coef[23] * (clipALF(clip[23], curr, pImg0FixedBased[-5], pImg0FixedBased[+5]));
              sum += coef[24] * ( clipALF(clip[24], curr, pImg0FixedBased[-4], pImg0FixedBased[+4]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg0FixedBased[-3], pImg0FixedBased[+3]) );
              sum += coef[26] * ( clipALF(clip[26], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[27] * ( clipALF(clip[27], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
            else if( transposeIdx == 3 )
            {
              sum += coef[22] * ( clipALF(clip[22], curr, pImg11FixedBased[-0], pImg12FixedBased[+0]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg9FixedBased[-0], pImg10FixedBased[+0]) );
              sum += coef[24] * ( clipALF(clip[24], curr, pImg7FixedBased[-0], pImg8FixedBased[+0]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg5FixedBased[-0], pImg6FixedBased[+0]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg3FixedBased[+1], pImg4FixedBased[-1]) );
              sum += coef[26] * ( clipALF(clip[26], curr, pImg3FixedBased[+0], pImg4FixedBased[-0]) );
              sum += coef[17] * ( clipALF(clip[17], curr, pImg3FixedBased[-1], pImg4FixedBased[+1]) );
              sum += coef[16] * ( clipALF(clip[16], curr, pImg1FixedBased[+2], pImg2FixedBased[-2]) );
              sum += coef[20] * ( clipALF(clip[20], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]) );
              sum += coef[27] * ( clipALF(clip[27], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]) );
              sum += coef[18] * ( clipALF(clip[18], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]) );
              sum += coef[14] * ( clipALF(clip[14], curr, pImg1FixedBased[-2], pImg2FixedBased[+2]) );
              sum += coef[10] * ( clipALF(clip[10], curr, pImg0FixedBased[-6], pImg0FixedBased[+6]) );
              sum += coef[11] * ( clipALF(clip[11], curr, pImg0FixedBased[-5], pImg0FixedBased[+5]) );
              sum += coef[12] * ( clipALF(clip[12], curr, pImg0FixedBased[-4], pImg0FixedBased[+4]) );
              sum += coef[13] * ( clipALF(clip[13], curr, pImg0FixedBased[-3], pImg0FixedBased[+3]) );
              sum += coef[15] * ( clipALF(clip[15], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[19] * ( clipALF(clip[19], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
            else
            {
              sum += coef[10] * ( clipALF(clip[10], curr, pImg11FixedBased[-0], pImg12FixedBased[+0]) );
              sum += coef[11] * ( clipALF(clip[11], curr, pImg9FixedBased[-0], pImg10FixedBased[+0]) );
              sum += coef[12] * ( clipALF(clip[12], curr, pImg7FixedBased[-0], pImg8FixedBased[+0]) );
              sum += coef[13] * ( clipALF(clip[13], curr, pImg5FixedBased[-0], pImg6FixedBased[+0]) );
              sum += coef[14] * ( clipALF(clip[14], curr, pImg3FixedBased[+1], pImg4FixedBased[-1]) );
              sum += coef[15] * ( clipALF(clip[15], curr, pImg3FixedBased[+0], pImg4FixedBased[-0]) );
              sum += coef[16] * ( clipALF(clip[16], curr, pImg3FixedBased[-1], pImg4FixedBased[+1]) );
              sum += coef[17] * ( clipALF(clip[17], curr, pImg1FixedBased[+2], pImg2FixedBased[-2]) );
              sum += coef[18] * ( clipALF(clip[18], curr, pImg1FixedBased[+1], pImg2FixedBased[-1]) );
              sum += coef[19] * ( clipALF(clip[19], curr, pImg1FixedBased[+0], pImg2FixedBased[-0]) );
              sum += coef[20] * ( clipALF(clip[20], curr, pImg1FixedBased[-1], pImg2FixedBased[+1]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg1FixedBased[-2], pImg2FixedBased[+2]) );
              sum += coef[22] * ( clipALF(clip[22], curr, pImg0FixedBased[-6], pImg0FixedBased[+6]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg0FixedBased[-5], pImg0FixedBased[+5]) );
              sum += coef[24] * ( clipALF(clip[24], curr, pImg0FixedBased[-4], pImg0FixedBased[+4]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg0FixedBased[-3], pImg0FixedBased[+3]) );
              sum += coef[26] * ( clipALF(clip[26], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[27] * ( clipALF(clip[27], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
          }
          else
#endif
          if( filtType >= ALF_FILTER_9_EXT && filtType != ALF_FILTER_EXT )
          {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
            int filterSetIdx = 2 + fixedFilterSetIdx;
#else
            int filterSetIdx = 0 + fixedFilterSetIdx;
#endif
            Pel *pImg0FixedBased, *pImg1FixedBased, *pImg2FixedBased, *pImg3FixedBased, *pImg4FixedBased;

            if( isFixedFilterPaddedPerCtu )
            {
              pImg0FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 0][j + jj + padSize];
              pImg1FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 1][j + jj + padSize];
              pImg2FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 1][j + jj + padSize];
              pImg3FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize + 2][j + jj + padSize];
              pImg4FixedBased = &fixedFilterResultsPerCtu[filterSetIdx][i + ii + padSize - 2][j + jj + padSize];
            }
            else
            {
              pImg0FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 0][blkDst.x + j + jj + padSize];
              pImg1FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 1][blkDst.x + j + jj + padSize];
              pImg2FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 1][blkDst.x + j + jj + padSize];
              pImg3FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize + 2][blkDst.x + j + jj + padSize];
              pImg4FixedBased = &fixedFilterResults[filterSetIdx][blkDst.y + i + ii + padSize - 2][blkDst.x + j + jj + padSize];
            }

            if ( transposeIdx == 1 )
            {
              sum += coef[24] * ( clipALF(clip[24], curr, pImg4FixedBased[+0], pImg3FixedBased[+0]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg2FixedBased[+0], pImg1FixedBased[+0]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]) );
              sum += coef[20] * ( clipALF(clip[20], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[22] * ( clipALF(clip[22], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
            else if ( transposeIdx == 2 )
            {
              sum += coef[20] * ( clipALF(clip[20], curr, pImg4FixedBased[+0], pImg3FixedBased[+0]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]) );
              sum += coef[22] * ( clipALF(clip[22], curr, pImg2FixedBased[+0], pImg1FixedBased[+0]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]) );
              sum += coef[24] * ( clipALF(clip[24], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
            else if ( transposeIdx == 3 )
            {
              sum += coef[24] * ( clipALF(clip[24], curr, pImg4FixedBased[+0], pImg3FixedBased[+0]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg2FixedBased[+0], pImg1FixedBased[+0]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]) );
              sum += coef[20] * ( clipALF(clip[20], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[22] * ( clipALF(clip[22], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
            else
            {
              sum += coef[20] * ( clipALF(clip[20], curr, pImg4FixedBased[+0], pImg3FixedBased[+0]) );
              sum += coef[21] * ( clipALF(clip[21], curr, pImg2FixedBased[-1], pImg1FixedBased[+1]) );
              sum += coef[22] * ( clipALF(clip[22], curr, pImg2FixedBased[+0], pImg1FixedBased[+0]) );
              sum += coef[23] * ( clipALF(clip[23], curr, pImg2FixedBased[+1], pImg1FixedBased[-1]) );
              sum += coef[24] * ( clipALF(clip[24], curr, pImg0FixedBased[-2], pImg0FixedBased[+2]) );
              sum += coef[25] * ( clipALF(clip[25], curr, pImg0FixedBased[-1], pImg0FixedBased[+1]) );
            }
          }
#endif
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          if (filtType == ALF_FILTER_13_EXT)
          {
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[28] * ( clipALF(clip[28], curr, pImg4Gauss[0][+0], pImg3Gauss[0][+0]) );
            sum += coef[29] * ( clipALF(clip[29], curr, pImg2Gauss[0][+0], pImg1Gauss[0][+0]) );
            sum += coef[30] * ( clipALF(clip[30], curr, pImg0Gauss[0][-2], pImg0Gauss[0][+2]) );
            sum += coef[31] * ( clipALF(clip[31], curr, pImg0Gauss[0][-1], pImg0Gauss[0][+1]) );
            sum += coef[32] * ( clipALF(clip[32], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[33] * ( clipALF(clip[33], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[34] * ( clipALF(clip[34], curr, pImg0Gauss[0][+0]));
#elif JVET_AD0222_ALF_LONG_FIXFILTER
            sum += coef[28] * ( clipALF(clip[28], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[29] * ( clipALF(clip[29], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[26] * ( clipALF(clip[26], curr, pImg4Gauss[0][+0], pImg3Gauss[0][+0]) );
            sum += coef[27] * ( clipALF(clip[27], curr, pImg2Gauss[0][+0], pImg1Gauss[0][+0]) );
            sum += coef[28] * ( clipALF(clip[28], curr, pImg0Gauss[0][-2], pImg0Gauss[0][+2]) );
            sum += coef[29] * ( clipALF(clip[29], curr, pImg0Gauss[0][-1], pImg0Gauss[0][+1]) );
            sum += coef[30] * ( clipALF(clip[30], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[31] * ( clipALF(clip[31], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[32] * ( clipALF(clip[32], curr, pImg0Gauss[0][+0]));
#endif
          }
          else
#endif
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER && !JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          if( filtType == ALF_FILTER_9_EXT || filtType == ALF_FILTER_13_EXT )
#else
          if( filtType == ALF_FILTER_9_EXT )
#endif
          {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            sum += coef[26] * ( clipALF(clip[26], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[27] * ( clipALF(clip[27], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
#else
            for( int k = 0; k < EXT_LENGTH; k++ )
            {
              sum += coef[k + 20] * ( clipALF(clip[k + 20], curr, fixedFilterResults[k * EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj]) );
            }
#endif
          }
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          else if (filtType == ALF_FILTER_13_EXT_DB)
          {
            Pel* pRecDbTmp0 = pRecDb1 + jj;
            Pel* pRecDbTmp1 = pRecDbTmp0 + recDbStride;
            Pel* pRecDbTmp2 = pRecDbTmp0 - recDbStride;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[28] * ( clipALF(clip[28], curr, pImg4Gauss[0][+0], pImg3Gauss[0][+0]) );
            sum += coef[29] * ( clipALF(clip[29], curr, pImg2Gauss[0][+0], pImg1Gauss[0][+0]) );
            sum += coef[30] * ( clipALF(clip[30], curr, pImg0Gauss[0][-2], pImg0Gauss[0][+2]) );
            sum += coef[31] * ( clipALF(clip[31], curr, pImg0Gauss[0][-1], pImg0Gauss[0][+1]) );
            sum += coef[32] * ( clipALF(clip[32], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[33] * ( clipALF(clip[33], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[34] * ( clipALF(clip[34], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[35] * ( clipALF(clip[35], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[36] * ( clipALF(clip[36], curr, pRecDbTmp0[+0]));
            sum += coef[37] * ( clipALF(clip[37], curr, pImg0Gauss[0][+0]));
#elif JVET_AD0222_ALF_LONG_FIXFILTER
            sum += coef[28] * (clipALF(clip[28], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[29] * (clipALF(clip[29], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[30] * (clipALF(clip[30], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[31] * (clipALF(clip[31], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[32] * (clipALF(clip[32], curr, pRecDbTmp0[+0]));
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[26] * ( clipALF(clip[26], curr, pImg4Gauss[0][+0], pImg3Gauss[0][+0]) );
            sum += coef[27] * ( clipALF(clip[27], curr, pImg2Gauss[0][+0], pImg1Gauss[0][+0]) );
            sum += coef[28] * ( clipALF(clip[28], curr, pImg0Gauss[0][-2], pImg0Gauss[0][+2]) );
            sum += coef[29] * ( clipALF(clip[29], curr, pImg0Gauss[0][-1], pImg0Gauss[0][+1]) );
            sum += coef[30] * ( clipALF(clip[30], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[31] * ( clipALF(clip[31], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[32] * ( clipALF(clip[32], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[33] * ( clipALF(clip[33], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[34] * ( clipALF(clip[34], curr, pRecDbTmp0[+0]));
            sum += coef[35] * ( clipALF(clip[35], curr, pImg0Gauss[0][+0]));
#endif
          }
#endif
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER && !JVET_AD0222_ADDITONAL_ALF_FIXFILTER
          else if( filtType == ALF_FILTER_9_EXT_DB || filtType == ALF_FILTER_13_EXT_DB )
#else
          else if( filtType == ALF_FILTER_9_EXT_DB )
#endif
          {
            Pel* pRecDbTmp0 = pRecDb1 + jj;
            Pel* pRecDbTmp1 = pRecDbTmp0 + recDbStride;
            Pel* pRecDbTmp2 = pRecDbTmp0 - recDbStride;
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
            sum += coef[26] * ( clipALF(clip[26], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[27] * ( clipALF(clip[27], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[28] * ( clipALF(clip[28], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[29] * ( clipALF(clip[29], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[30] * ( clipALF(clip[30], curr, pRecDbTmp0[+0] ));
#else
            sum += coef[20] * ( clipALF(clip[20], curr, pRecDbTmp2[+0]) + clipALF(clip[20], curr, pRecDbTmp1[+0]) );
            sum += coef[21] * ( clipALF(clip[21], curr, pRecDbTmp0[-1]) + clipALF(clip[21], curr, pRecDbTmp0[+1]) );
            sum += coef[22] *   clipALF(clip[22], curr, pRecDbTmp0[+0]);
            for (int k = 0; k < EXT_LENGTH; k++)
            {
              sum += coef[k + 20 + NUM_DB] * (clipALF(clip[k + 20 + NUM_DB], curr, fixedFilterResults[k * EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj]));
            }
#endif
          }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
          else if (filtType == ALF_FILTER_13_EXT_DB_RESI_DIRECT)
          {
            Pel *pRecDbTmp0 = pRecDb1 + jj;
            Pel *pRecDbTmp1 = pRecDbTmp0 + recDbStride;
            Pel *pRecDbTmp2 = pRecDbTmp0 - recDbStride;

            Pel *pResiTmp0 = pResi1 + jj;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[28] * (clipALF(clip[28], curr, pImg4Gauss[0][+0], pImg3Gauss[0][+0]));
            sum += coef[29] * (clipALF(clip[29], curr, pImg2Gauss[0][+0], pImg1Gauss[0][+0]));
            sum += coef[30] * (clipALF(clip[30], curr, pImg0Gauss[0][-2], pImg0Gauss[0][+2]));
            sum += coef[31] * (clipALF(clip[31], curr, pImg0Gauss[0][-1], pImg0Gauss[0][+1]));

            sum += coef[32] * (clipALF(clip[32], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[33] * (clipALF(clip[33], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[34] * (clipALF(clip[34], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[35] * (clipALF(clip[35], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[36] * (clipALF(clip[36], curr, pRecDbTmp0[+0]));
            sum += coef[37] * (clipALF(clip[37], 0, pResiTmp0[+0]));
            sum += coef[38] * (clipALF(clip[38], curr, pImg0Gauss[0][+0]));
#elif JVET_AD0222_ALF_LONG_FIXFILTER
            sum += coef[28] * (clipALF(clip[28], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[29] * (clipALF(clip[29], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));
            sum += coef[30] * (clipALF(clip[30], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[31] * (clipALF(clip[31], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[32] * (clipALF(clip[32], curr, pRecDbTmp0[+0]));
            sum += coef[33] * (clipALF(clip[33], 0, pResiTmp0[+0]));
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[26] * (clipALF(clip[26], curr, pImg4Gauss[0][+0], pImg3Gauss[0][+0]));
            sum += coef[27] * (clipALF(clip[27], curr, pImg2Gauss[0][+0], pImg1Gauss[0][+0]));
            sum += coef[28] * (clipALF(clip[28], curr, pImg0Gauss[0][-2], pImg0Gauss[0][+2]));
            sum += coef[29] * (clipALF(clip[29], curr, pImg0Gauss[0][-1], pImg0Gauss[0][+1]));

            sum += coef[30] * (clipALF(clip[30], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[31] * (clipALF(clip[31], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[32] * (clipALF(clip[32], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[33] * (clipALF(clip[33], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[34] * (clipALF(clip[34], curr, pRecDbTmp0[+0]));
            sum += coef[35] * (clipALF(clip[35], 0, pResiTmp0[+0]));
            sum += coef[36] * (clipALF(clip[36], curr, pImg0Gauss[0][+0]));
#else
            sum += coef[26] * (clipALF(clip[26], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[27] * (clipALF(clip[27], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum +=
              coef[28]
              * (clipALF(
                clip[28], curr,
                fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum +=
              coef[29]
              * (clipALF(
                clip[29], curr,
                fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[30] * (clipALF(clip[30], curr, pRecDbTmp0[+0]));
            sum += coef[31] * (clipALF(clip[31], 0, pResiTmp0[+0]));
#endif
          }
          else if (filtType == ALF_FILTER_13_EXT_DB_RESI)
          {
            Pel *pRecDbTmp0 = pRecDb1 + jj;
            Pel *pRecDbTmp1 = pRecDbTmp0 + recDbStride;
            Pel *pRecDbTmp2 = pRecDbTmp0 - recDbStride;

            Pel *pResiTmp0 = pResi1 + jj;
#if JVET_AD0222_ALF_LONG_FIXFILTER && JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[28] * (clipALF(clip[28], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[29] * (clipALF(clip[29], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[30] * (clipALF(clip[30], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[31] * (clipALF(clip[31], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[32] * (clipALF(clip[32], 0, fixedFilterResiResults[0 + 1 - fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj]));
            sum += coef[33] * (clipALF(clip[33], curr, pRecDbTmp0[+0]));
            sum += coef[34] * (clipALF(clip[34], 0, pResiTmp0[+0]));
            sum += coef[35] * (clipALF(clip[35], curr, pImg0Gauss[0][+0]));
#elif JVET_AD0222_ALF_LONG_FIXFILTER
            sum += coef[28] * (clipALF(clip[28], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[29] * (clipALF(clip[29], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[30] * (clipALF(clip[30], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[31] * (clipALF(clip[31], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[32] * (clipALF(clip[32], 0, fixedFilterResiResults[0 + 1 - fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj]));
            sum += coef[33] * (clipALF(clip[33], curr, pRecDbTmp0[+0]));
            sum += coef[34] * (clipALF(clip[34], 0, pResiTmp0[+0]));
#elif JVET_AD0222_ADDITONAL_ALF_FIXFILTER
            sum += coef[26] * (clipALF(clip[26], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[27] * (clipALF(clip[27], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum += coef[28] * (clipALF(clip[28], curr, fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[29] * (clipALF(clip[29], curr, fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[30] * (clipALF(clip[30], 0, fixedFilterResiResults[0 + 1 - fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj]));
            sum += coef[31] * (clipALF(clip[31], curr, pRecDbTmp0[+0]));
            sum += coef[32] * (clipALF(clip[32], 0, pResiTmp0[+0]));
            sum += coef[33] * (clipALF(clip[33], curr, pImg0Gauss[0][+0]));
#else
            sum += coef[26] * (clipALF(clip[26], curr, pRecDbTmp2[+0], pRecDbTmp1[-0]));
            sum += coef[27] * (clipALF(clip[27], curr, pRecDbTmp0[-1], pRecDbTmp0[+1]));

            sum +=
              coef[28]
              * (clipALF(
                clip[28], curr,
                fixedFilterResults[0 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum +=
              coef[29]
              * (clipALF(
                clip[29], curr,
                fixedFilterResults[2 + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize]));
            sum += coef[30]
                   * (clipALF(clip[30], 0,
                              fixedFilterResiResults[0 + 1 - fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj]));
            sum += coef[31] * (clipALF(clip[31], curr, pRecDbTmp0[+0]));
            sum += coef[32] * (clipALF(clip[32], 0, pResiTmp0[+0]));
#endif
          }
#endif
          else if( filtType == ALF_FILTER_EXT )
          {
            for( int k = 0; k < EXT_LENGTH; k++ )
            {
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
              sum += coef[k] * ( clipALF(clip[k], curr, fixedFilterResults[k * EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i + ii + padSize][blkDst.x + j + jj + padSize] ) );
#else
              sum += coef[k] * ( clipALF(clip[k], curr, fixedFilterResults[k * EXT_LENGTH + fixedFilterSetIdx][blkDst.y + i + ii][blkDst.x + j + jj] ) );
#endif
            }
          }
          sum = ( sum + offset ) >> shift;
#else
          if (!(isNearVBabove || isNearVBbelow))
          {
            sum = (sum + offset) >> shift;
          }
          else
          {
            sum = (sum + (1 << ((shift + 3) - 1))) >> (shift + 3);
          }
#endif
          sum += curr;
          pRec1[jj] = ClipPel( sum, clpRng );

          pImg0++;
          pImg1++;
          pImg2++;
          pImg3++;
          pImg4++;
          pImg5++;
          pImg6++;
#if ALF_IMPROVEMENT
          pImg7++;
          pImg8++;
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER
          pImg9++;
          pImg10++;
          pImg11++;
          pImg12++;
#endif
#endif
        }
      }
    }

    pRec0 += dstStride2;
    pRec1 += dstStride2;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    if( isLuma( compId ) )
    {
      pRecDb0 += recDbStride2;
      pRecDb1 += recDbStride2;
    }
#endif
#if JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT
    if (isLuma(compId))
    {
      pResi0 += resiStride2;
      pResi1 += resiStride2;
    }
#endif
    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
#if ALF_IMPROVEMENT
    pImgYPad7 += srcStride2;
    pImgYPad8 += srcStride2;
#if JVET_AA0095_ALF_LONGER_FILTER && !JVET_AD0222_ALF_LONG_FIXFILTER
    pImgYPad9  += srcStride2;
    pImgYPad10 += srcStride2;
    pImgYPad11 += srcStride2;
    pImgYPad12 += srcStride2;
#endif
#endif
  }
}

template<AlfFilterType filtTypeCcAlf>
#if ALF_IMPROVEMENT
void AdaptiveLoopFilter::filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs)
#else
void AdaptiveLoopFilter::filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos)
#endif
{
#if !ALF_IMPROVEMENT
  CHECK(1 << floorLog2(vbCTUHeight) != vbCTUHeight, "Not a power of 2");
#endif

  CHECK(!isChroma(compId), "Must be chroma");

  const SPS*     sps           = cs.slice->getSPS();
  ChromaFormat nChromaFormat   = sps->getChromaFormatIdc();
  const int clsSizeY           = 4;
  const int clsSizeX           = 4;
  const int      startHeight        = blkDst.y;
  const int      endHeight          = blkDst.y + blkDst.height;
  const int      startWidth         = blkDst.x;
  const int      endWidth           = blkDst.x + blkDst.width;
  const int scaleX             = getComponentScaleX(compId, nChromaFormat);
  const int scaleY             = getComponentScaleY(compId, nChromaFormat);

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  CPelBuf     srcBuf     = recSrc.get(COMPONENT_Y);
  const int   lumaStride = srcBuf.stride;
  const Pel * lumaPtr    = srcBuf.buf + blkSrc.y * lumaStride + blkSrc.x;

  const int   chromaStride = dstBuf.stride;
  Pel *       chromaPtr    = dstBuf.buf + blkDst.y * chromaStride + blkDst.x;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    for( int j = 0; j < endWidth - startWidth; j += clsSizeX )
    {
      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        int row       = ii;
        int col       = j;
        Pel *srcSelf  = chromaPtr + col + row * chromaStride;

        int offset1 = lumaStride;
        int offset2 = -lumaStride;
        int offset3 = 2 * lumaStride;
#if JVET_X0071_LONGER_CCALF
        int offset4 = -2 * lumaStride;
        int offset5 =  3 * lumaStride;
        int offset6 = -3 * lumaStride;
        int offset7 = 4 * lumaStride;
        int offset8 = -4 * lumaStride;
#endif

        row <<= scaleY;
        col <<= scaleX;
        const Pel *srcCross = lumaPtr + col + row * lumaStride;

#if !ALF_IMPROVEMENT
        int pos = ((startHeight + i + ii) << scaleY) & (vbCTUHeight - 1);
        if (scaleY == 0 && (pos == vbPos || pos == vbPos + 1))
        {
          continue;
        }
        if (pos == (vbPos - 2) || pos == (vbPos + 1))
        {
          offset3 = offset1;
        }
        else if (pos == (vbPos - 1) || pos == vbPos)
        {
          offset1 = 0;
          offset2 = 0;
          offset3 = 0;
        }
#endif

        for (int jj = 0; jj < clsSizeX; jj++)
        {
          const int jj2     = (jj << scaleX);
          const int offset0 = 0;

          int sum = 0;
          const Pel currSrcCross = srcCross[offset0 + jj2];

#if JVET_X0071_LONGER_CCALF
          sum += filterCoeff[0] * (srcCross[offset8 + jj2] - currSrcCross);
          sum += filterCoeff[1] * (srcCross[offset6 + jj2] - currSrcCross);
          sum += filterCoeff[2] * (srcCross[offset4 + jj2] - currSrcCross);
          sum += filterCoeff[3] * (srcCross[offset2 + jj2] - currSrcCross);

          sum += filterCoeff[4] * (srcCross[offset0 + jj2 - 4] - currSrcCross);
          sum += filterCoeff[5] * (srcCross[offset0 + jj2 - 3] - currSrcCross);
          sum += filterCoeff[6] * (srcCross[offset0 + jj2 - 2] - currSrcCross);
          sum += filterCoeff[7] * (srcCross[offset0 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[8] * (srcCross[offset0 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[9] * (srcCross[offset0 + jj2 + 2] - currSrcCross);
          sum += filterCoeff[10] * (srcCross[offset0 + jj2 + 3] - currSrcCross);
          sum += filterCoeff[11] * (srcCross[offset0 + jj2 + 4] - currSrcCross);

          sum += filterCoeff[12] * (srcCross[offset1 + jj2 - 4] - currSrcCross);
          sum += filterCoeff[13] * (srcCross[offset1 + jj2 - 3] - currSrcCross);
          sum += filterCoeff[14] * (srcCross[offset1 + jj2 - 2] - currSrcCross);
          sum += filterCoeff[15] * (srcCross[offset1 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[16] * (srcCross[offset1 + jj2 - 0] - currSrcCross);
          sum += filterCoeff[17] * (srcCross[offset1 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[18] * (srcCross[offset1 + jj2 + 2] - currSrcCross);
          sum += filterCoeff[19] * (srcCross[offset1 + jj2 + 3] - currSrcCross);
          sum += filterCoeff[20] * (srcCross[offset1 + jj2 + 4] - currSrcCross);

          sum += filterCoeff[21] * (srcCross[offset3 + jj2] - currSrcCross);
          sum += filterCoeff[22] * (srcCross[offset5 + jj2] - currSrcCross);
          sum += filterCoeff[23] * (srcCross[offset7 + jj2] - currSrcCross);
#else
          sum += filterCoeff[0] * (srcCross[offset2 + jj2    ] - currSrcCross);
          sum += filterCoeff[1] * (srcCross[offset0 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[2] * (srcCross[offset0 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[3] * (srcCross[offset1 + jj2 - 1] - currSrcCross);
          sum += filterCoeff[4] * (srcCross[offset1 + jj2    ] - currSrcCross);
          sum += filterCoeff[5] * (srcCross[offset1 + jj2 + 1] - currSrcCross);
          sum += filterCoeff[6] * (srcCross[offset3 + jj2    ] - currSrcCross);
#endif
          sum = (sum + ((1 << m_scaleBits ) >> 1)) >> m_scaleBits;
          const int offset = 1 << clpRngs.comp[compId].bd >> 1;
          sum = ClipPel(sum + offset, clpRngs.comp[compId]) - offset;
          sum += srcSelf[jj];
          srcSelf[jj] = ClipPel(sum, clpRngs.comp[compId]);
        }
      }
    }

    chromaPtr += chromaStride * clsSizeY;

    lumaPtr += lumaStride * clsSizeY << getComponentScaleY(compId, nChromaFormat);
  }
}

#if JVET_AA0095_ALF_LONGER_FILTER
void AdaptiveLoopFilter::mirroredPaddingForAlf(CodingStructure &cs, const PelUnitBuf &src, int paddingSize, bool enableLuma, bool enableChroma)
{
  int padSize = paddingSize;
  int padSizeDou = padSize << 1;

  if( enableLuma )
  {
    int srcLumaStride = src.get(COMPONENT_Y).stride;
    Pel* srcLumaPel = src.get(COMPONENT_Y).buf;

    int widthLuma = src.get(COMPONENT_Y).width;
    int heightLuma = src.get(COMPONENT_Y).height;
    //perform mirror padding for ALF

    Pel* tmpSrcLuma = srcLumaPel;
    int xPos = 0;
    //left and right
    for(int yPos = 0; yPos < heightLuma; yPos++)
    {
      for(int i = 1; i < padSize + 1; i++)
      {
        xPos = 0;
        tmpSrcLuma[xPos - i] = tmpSrcLuma[xPos + i - 1];
        xPos = widthLuma - 1;
        tmpSrcLuma[xPos + i] = tmpSrcLuma[xPos - i + 1];
      }
      tmpSrcLuma += srcLumaStride;
    }
    //top and bottom
    tmpSrcLuma = srcLumaPel + srcLumaStride * (heightLuma - 1);
    int lineLength = widthLuma + padSizeDou;
    int yOffset = 0;

    for(int i = 1; i < padSize + 1; i++)
    {
      yOffset = i * srcLumaStride;
      memcpy( srcLumaPel - yOffset - padSize, srcLumaPel + yOffset - padSize - srcLumaStride, sizeof(Pel) * lineLength);
      memcpy( tmpSrcLuma + yOffset - padSize, tmpSrcLuma - yOffset - padSize + srcLumaStride, sizeof(Pel) * lineLength);
    }
  }

  if(m_chromaFormat == CHROMA_400)
  {
    return;
  }
  if( enableChroma )
  {
    for(int compIdx = 1; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      ComponentID compID = ComponentID(compIdx);

      int srcChromaStride = src.get(compID).stride;
      Pel* srcChromaPel = src.get(compID).buf;
      int xScale = getComponentScaleX(compID, m_chromaFormat);
      int yScale = getComponentScaleY(compID, m_chromaFormat);
      int widthChroma = src.get(COMPONENT_Y).width >> xScale;
      int heightChroma = src.get(COMPONENT_Y).height >> yScale;

      Pel* tmpSrcChroma = srcChromaPel;
      int xPos = 0;
      //left and right
      for(int yPos = 0; yPos < heightChroma; yPos++)
      {
        for(int i = 1; i < padSize + 1; i++)
        {
          xPos = 0;
          tmpSrcChroma[xPos - i] = tmpSrcChroma[xPos + i - 1];
          xPos = widthChroma - 1;
          tmpSrcChroma[xPos + i] = tmpSrcChroma[xPos - i + 1];
        }
        tmpSrcChroma += srcChromaStride;
      }
      //top and bottom
      tmpSrcChroma = srcChromaPel + srcChromaStride * (heightChroma - 1);
      int lineLength = widthChroma + padSizeDou;
      int yOffset = 0;

      for(int i = 1; i < padSize + 1; i++)
      {
        yOffset = i * srcChromaStride;
        memcpy( srcChromaPel - yOffset - padSize, srcChromaPel + yOffset - padSize - srcChromaStride, sizeof(Pel) * lineLength);
        memcpy( tmpSrcChroma + yOffset - padSize, tmpSrcChroma - yOffset - padSize + srcChromaStride, sizeof(Pel) * lineLength);
      }
    }
  }
}
#endif
#if JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS
void AdaptiveLoopFilter::paddingFixedFilterResultsPic(Pel*** fixedFilterResults, const int fixedFilterSetIdx)
{
  int classifierIdx = 0;
  int filterIndex = classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
  CHECK(ALF_PADDING_SIZE_FIXED_RESULTS != 2, "Wrong Padding Size in FixedFilterResults");
#endif
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
  int padSizeDou = padSize << 1;
  int lineWidth = padSizeDou + m_picWidth;

  for(int row = padSize; row < padSize + m_picHeight; row++)
  {
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
    for( int i = 0; i < padSize; i++ )
    {
      fixedFilterResults[filterIndex][row][i] = fixedFilterResults[filterIndex][row][padSizeDou - 1 - i];
      fixedFilterResults[filterIndex][row][padSize + m_picWidth + i] = fixedFilterResults[filterIndex][row][padSize + m_picWidth - 1 - i];
    }
#else
    fixedFilterResults[filterIndex][row][0] = fixedFilterResults[filterIndex][row][3];
    fixedFilterResults[filterIndex][row][1] = fixedFilterResults[filterIndex][row][2];

    fixedFilterResults[filterIndex][row][padSize + m_picWidth + 0] = fixedFilterResults[filterIndex][row][padSize + m_picWidth - 1];
    fixedFilterResults[filterIndex][row][padSize + m_picWidth + 1] = fixedFilterResults[filterIndex][row][padSize + m_picWidth - 2];
#endif
  }
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
  for( int i = 0; i < padSize; i++ )
  {
    memcpy(fixedFilterResults[filterIndex][i], fixedFilterResults[filterIndex][padSizeDou - 1 - i], sizeof(Pel) * lineWidth);
    memcpy(fixedFilterResults[filterIndex][padSize + m_picHeight + i], fixedFilterResults[filterIndex][padSize + m_picHeight - 1 - i], sizeof(Pel) * lineWidth);
  }
#else
  memcpy(fixedFilterResults[filterIndex][0], fixedFilterResults[filterIndex][3], sizeof(Pel) * lineWidth );
  memcpy(fixedFilterResults[filterIndex][1], fixedFilterResults[filterIndex][2], sizeof(Pel) * lineWidth );

  memcpy(fixedFilterResults[filterIndex][padSize + m_picHeight + 0], fixedFilterResults[filterIndex][padSize + m_picHeight - 1], sizeof(Pel) * lineWidth );
  memcpy(fixedFilterResults[filterIndex][padSize + m_picHeight + 1], fixedFilterResults[filterIndex][padSize + m_picHeight - 2], sizeof(Pel) * lineWidth );
#endif
}

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
void AdaptiveLoopFilter::paddingFixedFilterResultsCtu(Pel*** fixedFilterResultsPic, Pel*** fixedFilterResultsCtu, const int fixedFilterSetIdx, const Area &blk, const int classifierIdx)
#else
void AdaptiveLoopFilter::paddingFixedFilterResultsCtu(Pel*** fixedFilterResultsPic, Pel*** fixedFilterResultsCtu, const int fixedFilterSetIdx, const Area &blk)
#endif
{
  int xPos = blk.pos().x;
  int yPos = blk.pos().y;
  int width = blk.size().width;
  int height = blk.size().height;
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
  int classifierIdx = 0;
#endif
  int filterIndex = classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx;
#if !JVET_AD0222_ALF_LONG_FIXFILTER
  CHECK(ALF_PADDING_SIZE_FIXED_RESULTS != 2, "Wrong Padding Size in FixedFilterResults");
#endif
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
  int padSizeDou = padSize << 1;
  int lineWidth = padSizeDou + width;

  for(int row = padSize; row < height + padSize; row++)
  {
    memcpy(&fixedFilterResultsCtu[filterIndex][row][padSize], &fixedFilterResultsPic[filterIndex][row + yPos][padSize + xPos], sizeof(Pel) * width );
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
    for( int i = 0; i < padSize; i++ )
    {
      fixedFilterResultsCtu[filterIndex][row][i] = fixedFilterResultsCtu[filterIndex][row][padSizeDou - i - 1];
      fixedFilterResultsCtu[filterIndex][row][padSize + width + i] = fixedFilterResultsCtu[filterIndex][row][padSize + width - 1 - i];
    }
#else
    fixedFilterResultsCtu[filterIndex][row][0] = fixedFilterResultsCtu[filterIndex][row][3];
    fixedFilterResultsCtu[filterIndex][row][1] = fixedFilterResultsCtu[filterIndex][row][2];

    fixedFilterResultsCtu[filterIndex][row][padSize + width + 0] = fixedFilterResultsCtu[filterIndex][row][padSize + width - 1];
    fixedFilterResultsCtu[filterIndex][row][padSize + width + 1] = fixedFilterResultsCtu[filterIndex][row][padSize + width - 2];
#endif
  }
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
  for( int i = 0; i < padSize; i++ )
  {
    memcpy(fixedFilterResultsCtu[filterIndex][i], fixedFilterResultsCtu[filterIndex][padSizeDou - 1 - i], sizeof(Pel) * lineWidth);
    memcpy(fixedFilterResultsCtu[filterIndex][padSize + height + i], fixedFilterResultsCtu[filterIndex][padSize + height - 1 - i], sizeof(Pel) * lineWidth);
  }
#else
  memcpy(fixedFilterResultsCtu[filterIndex][0], fixedFilterResultsCtu[filterIndex][3], sizeof(Pel) * lineWidth );
  memcpy(fixedFilterResultsCtu[filterIndex][1], fixedFilterResultsCtu[filterIndex][2], sizeof(Pel) * lineWidth );

  memcpy(fixedFilterResultsCtu[filterIndex][padSize + height + 0], fixedFilterResultsCtu[filterIndex][padSize + height - 1], sizeof(Pel) * lineWidth );
  memcpy(fixedFilterResultsCtu[filterIndex][padSize + height + 1], fixedFilterResultsCtu[filterIndex][padSize + height - 2], sizeof(Pel) * lineWidth );
#endif
}

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
void AdaptiveLoopFilter::deriveVariance(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t ***laplacian)
{
  int fl = DIST_CLASS;
  int stride = srcLuma.stride;
  int stride2 = 2 * stride;
  const Pel* src = srcLuma.buf + (blk.pos().y - fl) * stride + blk.pos().x - fl;
  int numSample = (fl * 2 + 2) * (fl * 2 + 2);
  int numSample2 = 128 * 128;
  int offset = numSample2 >> 1;
  int fl2 = fl << 1;
  for (int i = 0; i < blk.height + fl2; i += 2)
  {
    const Pel* src1 = src + stride;
    int iOffset = i >> 1;
    for (int j = 0; j < blk.width + fl2; j += 2)
    {
      int jOffset = j >> 1;
      laplacian[0][iOffset][jOffset] = src[j] + src[j + 1] + src1[j] + src1[j + 1];
      laplacian[1][iOffset][jOffset] = src[j] * src[j] + src[j + 1] * src[j + 1] + src1[j] * src1[j] + src1[j + 1] * src1[j + 1];

      int iOffsetM4 = iOffset - 4;
      int jOffsetM4 = jOffset - 4;

      if (jOffsetM4 == 0)
      {
        laplacian[2][iOffset][jOffsetM4] = laplacian[0][iOffset][jOffset - 4] + laplacian[0][iOffset][jOffset - 3] + laplacian[0][iOffset][jOffset - 2] + laplacian[0][iOffset][jOffset - 1] + laplacian[0][iOffset][jOffset];
        laplacian[3][iOffset][jOffsetM4] = laplacian[1][iOffset][jOffset - 4] + laplacian[1][iOffset][jOffset - 3] + laplacian[1][iOffset][jOffset - 2] + laplacian[1][iOffset][jOffset - 1] + laplacian[1][iOffset][jOffset];
      }
      else if (jOffsetM4 > 0)
      {
        laplacian[2][iOffset][jOffsetM4] = laplacian[2][iOffset][jOffset - 5] - laplacian[0][iOffset][jOffset - 5] + laplacian[0][iOffset][jOffset];
        laplacian[3][iOffset][jOffsetM4] = laplacian[3][iOffset][jOffset - 5] - laplacian[1][iOffset][jOffset - 5] + laplacian[1][iOffset][jOffset];
      }

      if (iOffsetM4 >= 0)
      {
        if (iOffsetM4 == 0)
        {
          laplacian[0][iOffsetM4][jOffsetM4] = laplacian[2][iOffsetM4][jOffsetM4] + laplacian[2][iOffset - 3][jOffsetM4] + laplacian[2][iOffset - 2][jOffsetM4] + laplacian[2][iOffset - 1][jOffsetM4] + laplacian[2][iOffset][jOffsetM4];
          laplacian[1][iOffsetM4][jOffsetM4] = laplacian[3][iOffsetM4][jOffsetM4] + laplacian[3][iOffset - 3][jOffsetM4] + laplacian[3][iOffset - 2][jOffsetM4] + laplacian[3][iOffset - 1][jOffsetM4] + laplacian[3][iOffset][jOffsetM4];
        }
        else
        {
          laplacian[0][iOffsetM4][jOffsetM4] = laplacian[0][iOffsetM4 - 1][jOffsetM4] - laplacian[2][iOffsetM4 - 1][jOffsetM4] + laplacian[2][iOffset][jOffsetM4];
          laplacian[1][iOffsetM4][jOffsetM4] = laplacian[1][iOffsetM4 - 1][jOffsetM4] - laplacian[3][iOffsetM4 - 1][jOffsetM4] + laplacian[3][iOffset][jOffsetM4];
        }

        laplacian[VARIANCE][iOffsetM4][jOffsetM4] = (13 * ((numSample * laplacian[1][iOffsetM4][jOffsetM4] - laplacian[0][iOffsetM4][jOffsetM4] * laplacian[0][iOffsetM4][jOffsetM4] + offset) >> 3)) >> 14;
      }
    }
    src += stride2;
  }
}
void AdaptiveLoopFilter::deriveFixedFilterResultsBlk(AlfClassifier*** classifier, const CPelBuf& srcLuma, const CPelBuf& srcLumaBeforeDb, const Area& blkDst, const Area& blk, const int bits, CodingStructure &cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int winIdx, int fixedFilterSetIdx)
{
  int fixedFiltSetIndMin = (qp - 22) / 4;
  int fixedFiltSetIndMax = (qp - 18) / 4;

  if (fixedFiltSetIndMin <= 0)
  {
    fixedFiltSetIndMin = 0;
    fixedFiltSetIndMax = 1;
  }

  if (fixedFiltSetIndMax > (NUM_SETS_FIXED_FILTERS - 1))
  {
    fixedFiltSetIndMin = NUM_SETS_FIXED_FILTERS - 2;
    fixedFiltSetIndMax = NUM_SETS_FIXED_FILTERS - 1;
  }

  int targetFixedFilterSetInd = fixedFilterSetIdx == 0 ? fixedFiltSetIndMin : (fixedFilterSetIdx == 1 ? fixedFiltSetIndMax : -1);

  int fixedFiltInd = winIdx * EXT_LENGTH;
  for (int fixedFiltSetInd = fixedFiltSetIndMin; fixedFiltSetInd <= fixedFiltSetIndMax; fixedFiltSetInd++)
  {
    if (fixedFiltSetInd == targetFixedFilterSetInd || targetFixedFilterSetInd == -1)
    {
      alfFixedFilterBlk(winIdx == 0 ? classifier[0] : classifier[ALF_NUM_CLASSIFIER], srcLuma, blk, blkDst, srcLumaBeforeDb, m_fixFilterResult, m_picWidth, fixedFiltInd, fixedFiltSetInd, winIdx, clpRng, clippingValues);
    }
    fixedFiltInd++;
  }
}
void AdaptiveLoopFilter::deriveFixedFilterResults(AlfClassifier*** classifier, const CPelBuf& srcLuma, const CPelBuf& srcLumaBeforeDb, const Area& blkDst, const Area& blk, CodingStructure &cs, int winIdx, int fixedFilterSetIdx)
{
  if (cs.slice->getCuQpDeltaSubdiv())
  {
    UnitArea curArea(cs.area.chromaFormat, blkDst);
    for (auto &currCU : cs.traverseCUs(curArea, CH_L))
    {
      deriveFixedFilterResultsBlk(classifier, srcLuma, srcLumaBeforeDb, Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), Area(currCU.lumaPos().x, currCU.lumaPos().y, currCU.lwidth(), currCU.lheight()), m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], currCU.qp, winIdx, fixedFilterSetIdx);
    }
  }
  else
  {
    deriveFixedFilterResultsBlk(classifier, srcLuma, srcLumaBeforeDb, blkDst, blk, m_inputBitDepth[CHANNEL_TYPE_LUMA], cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], cs.slice->getSliceQp(), winIdx, fixedFilterSetIdx);
  }
}
void AdaptiveLoopFilter::deriveFixedFilterResultsCtuBoundary(AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &blkDst, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx, int classifierIdx)
#else
void AdaptiveLoopFilter::deriveFixedFilterResultsCtuBoundary(AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkDst, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx )
#endif
{
#if !JVET_AD0222_ALF_LONG_FIXFILTER
  CHECK(ALF_PADDING_SIZE_FIXED_RESULTS != 2, "Wrong Padding Size in FixedFilterResults");
#endif
  //Ctu Info
  int yPos = blkDst.pos().y;
  int xPos = blkDst.pos().x;
  int width = blkDst.size().width;
  int height = blkDst.size().height;
  int padSize = ALF_PADDING_SIZE_FIXED_RESULTS;
#if JVET_AD0222_ALF_LONG_FIXFILTER
  int padSize2 = padSize << 1;
#endif
  int maxBoundaryNum = 8;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  int numSampleForSIMD = classifierIdx == 0 ? 16 : 8;
#else
  int numSampleForSIMD = 8;
#endif
  int ctuStride = m_numCTUsInWidth;
  Area blkCur[8];
  bool isBoundaryValid[8] = {false}; //left, right, top, bottom, topleft, topright, bottomleft, bottomright
  bool isNeighborAvai[8] = {false}; //left, right, top, bottom, topleft, topright, bottomleft, bottomright

  isBoundaryValid[0] = xPos > 0 ? true : false;
  isBoundaryValid[1] = xPos + width < m_picWidth ? true : false;
  isBoundaryValid[2] = yPos > 0 ? true : false;
  isBoundaryValid[3] = yPos + height < m_picHeight ? true : false;
  isBoundaryValid[4] = isBoundaryValid[0] && isBoundaryValid[2];
  isBoundaryValid[5] = isBoundaryValid[1] && isBoundaryValid[2];
  isBoundaryValid[6] = isBoundaryValid[0] && isBoundaryValid[3];
  isBoundaryValid[7] = isBoundaryValid[1] && isBoundaryValid[3];

  Area blkZero = Area(xPos, yPos, 0, 0);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  int extraSize = numSampleForSIMD;
#else
  int extraSize = padSize;

  bool isLeftExtraAvai = isBoundaryValid[0] ? (xPos - numSampleForSIMD >= 0 ? true : false) : false;
  bool isRightExtraAvai = isBoundaryValid[1] ? (xPos + width + numSampleForSIMD - 1 < m_picWidth ? true : false) : false;
  extraSize = (isLeftExtraAvai && isRightExtraAvai) ? numSampleForSIMD : padSize;
#endif
  blkCur[0] = isBoundaryValid[0] ? Area(xPos - extraSize, yPos, extraSize, height) : blkZero;
  blkCur[1] = isBoundaryValid[1] ? Area(xPos + width    , yPos, extraSize, height) : blkZero;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  blkCur[2] = isBoundaryValid[2] ? Area(xPos, yPos - extraSize, width, extraSize) : blkZero;
  blkCur[3] = isBoundaryValid[3] ? Area(xPos, yPos + height, width, extraSize) : blkZero;
  blkCur[4] = isBoundaryValid[4] ? Area(xPos - extraSize, yPos - extraSize, extraSize, extraSize) : blkZero;
  blkCur[5] = isBoundaryValid[5] ? Area(xPos + width, yPos - extraSize, extraSize, extraSize) : blkZero;
  blkCur[6] = isBoundaryValid[6] ? Area(xPos - extraSize, yPos + height, extraSize, extraSize) : blkZero;
  blkCur[7] = isBoundaryValid[7] ? Area(xPos + width, yPos + height, extraSize, extraSize) : blkZero;
#else
  blkCur[2] = isBoundaryValid[2] ? Area(xPos, yPos - padSize, width, padSize) : blkZero;
  blkCur[3] = isBoundaryValid[3] ? Area(xPos, yPos + height,  width, padSize) : blkZero;
  blkCur[4] = isBoundaryValid[4] ? Area(xPos - extraSize, yPos - padSize, extraSize, padSize) : blkZero;
  blkCur[5] = isBoundaryValid[5] ? Area(xPos + width    , yPos - padSize, extraSize, padSize) : blkZero;
  blkCur[6] = isBoundaryValid[6] ? Area(xPos - extraSize, yPos + height, extraSize, padSize) : blkZero;
  blkCur[7] = isBoundaryValid[7] ? Area(xPos + width    , yPos + height, extraSize, padSize) : blkZero;
#endif

  isNeighborAvai[0] = isBoundaryValid[0] ? ctuEnableFlagLuma[ctuIdx - 1] && ctuEnableOnlineLuma[ctuIdx - 1] : false;
  isNeighborAvai[1] = isBoundaryValid[1] ? ctuEnableFlagLuma[ctuIdx + 1] && ctuEnableOnlineLuma[ctuIdx + 1] : false;
  isNeighborAvai[2] = isBoundaryValid[2] ? ctuEnableFlagLuma[ctuIdx - ctuStride] && ctuEnableOnlineLuma[ctuIdx - ctuStride] : false;
  isNeighborAvai[3] = isBoundaryValid[3] ? ctuEnableFlagLuma[ctuIdx + ctuStride] && ctuEnableOnlineLuma[ctuIdx + ctuStride] : false;
  isNeighborAvai[4] = isBoundaryValid[4] ? ctuEnableFlagLuma[ctuIdx - ctuStride - 1] && ctuEnableOnlineLuma[ctuIdx - ctuStride - 1] : false;
  isNeighborAvai[5] = isBoundaryValid[5] ? ctuEnableFlagLuma[ctuIdx - ctuStride + 1] && ctuEnableOnlineLuma[ctuIdx - ctuStride + 1] : false;
  isNeighborAvai[6] = isBoundaryValid[6] ? ctuEnableFlagLuma[ctuIdx + ctuStride - 1] && ctuEnableOnlineLuma[ctuIdx + ctuStride - 1] : false;
  isNeighborAvai[7] = isBoundaryValid[7] ? ctuEnableFlagLuma[ctuIdx + ctuStride + 1] && ctuEnableOnlineLuma[ctuIdx + ctuStride + 1] : false;
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
  int classifierIdx = 0;
#endif
  for(int boundaryIdx = 0; boundaryIdx < maxBoundaryNum; boundaryIdx++)
  {
    if(isBoundaryValid[boundaryIdx] && !isNeighborAvai[boundaryIdx])
    {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      if (boundaryIdx == 1)
      {
        m_ctuPadFlag[ctuIdx + 1] = m_ctuPadFlag[ctuIdx + 1] | 0x01;
      }
      else if (boundaryIdx == 3)
      {
        m_ctuPadFlag[ctuIdx + ctuStride] = m_ctuPadFlag[ctuIdx + ctuStride] | 0x02;
      }
      deriveFixedFilterResultsPerBlk(classifier, fixedFilterResults, srcLuma, srcLumaBeforeDb, blkCur[boundaryIdx], bits, cs, clpRng, clippingValues, qp, fixedFilterSetIdx, m_mappingDir, laplacian, classifierIdx);
#else
      deriveFixedFilterResultsPerBlk(classifier, fixedFilterResults, srcLuma, blkCur[boundaryIdx], bits, cs, clpRng, clippingValues, qp, fixedFilterSetIdx, m_mappingDir, laplacian, classifierIdx);
#endif
    }
  }

  int xPosTmp = xPos;
  int yPosTmp = yPos;
  int filterSetIdx = classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx;

  if( !isBoundaryValid[0] )
  {
    for(int y = yPos + padSize; y < yPos + padSize + height; y++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for(int x = 0; x < padSize; x++)
      {
        fixedFilterResults[filterSetIdx][y][x] = fixedFilterResults[filterSetIdx][y][padSize2 - 1 - x];
      }     
#else
      fixedFilterResults[filterSetIdx][y][0] = fixedFilterResults[filterSetIdx][y][3];
      fixedFilterResults[filterSetIdx][y][1] = fixedFilterResults[filterSetIdx][y][2];
#endif
    }
  }

  if( !isBoundaryValid[1] )
  {
    for(int y = yPos + padSize; y < yPos + padSize + height; y++)
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for(int x = 0; x < padSize; x++)
      {
        fixedFilterResults[filterSetIdx][y][padSize + m_picWidth + x] = fixedFilterResults[filterSetIdx][y][padSize + m_picWidth - 1 - x];
      }
#else
      fixedFilterResults[filterSetIdx][y][padSize + m_picWidth + 0] = fixedFilterResults[filterSetIdx][y][padSize + m_picWidth - 1];
      fixedFilterResults[filterSetIdx][y][padSize + m_picWidth + 1] = fixedFilterResults[filterSetIdx][y][padSize + m_picWidth - 2];
#endif
    }
  }

  if( !isBoundaryValid[2] )
  {
    xPosTmp = padSize + xPos + 0;
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
    for( int y = 0; y < padSize; y++ )
    {
      memcpy(&fixedFilterResults[filterSetIdx][y][xPosTmp], &fixedFilterResults[filterSetIdx][padSize2 - 1 - y][xPosTmp], sizeof(Pel) * width);
    }
#else
    memcpy(&fixedFilterResults[filterSetIdx][0][xPosTmp], &fixedFilterResults[filterSetIdx][3][xPosTmp], sizeof(Pel) * width );
    memcpy(&fixedFilterResults[filterSetIdx][1][xPosTmp], &fixedFilterResults[filterSetIdx][2][xPosTmp], sizeof(Pel) * width );
#endif
  }

  if( !isBoundaryValid[3] )
  {
    xPosTmp = padSize + xPos + 0;
#if JVET_AD0222_ALF_LONG_FIXFILTER || JVET_AE0139_ALF_IMPROVED_FIXFILTER
    for( int y = 0; y < padSize; y++ )
    {
      memcpy(&fixedFilterResults[filterSetIdx][padSize + m_picHeight + y][xPosTmp], &fixedFilterResults[filterSetIdx][padSize + m_picHeight - 1 - y][xPosTmp], sizeof(Pel) * width);
    }
#else
    memcpy(&fixedFilterResults[filterSetIdx][padSize + m_picHeight + 0][xPosTmp], &fixedFilterResults[filterSetIdx][padSize + m_picHeight - 1][xPosTmp], sizeof(Pel) * width );
    memcpy(&fixedFilterResults[filterSetIdx][padSize + m_picHeight + 1][xPosTmp], &fixedFilterResults[filterSetIdx][padSize + m_picHeight - 2][xPosTmp], sizeof(Pel) * width );
#endif
  }

  if( !isBoundaryValid[4] )
  {
    if( isBoundaryValid[2] )
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int y = 1; y <= extraSize; y++ )
#else
      for( int y = 1; y <= padSize; y++ )
#endif
      {
        yPosTmp = yPos + padSize - y;
        for( int x = 0; x < padSize; x++ )
        {
          fixedFilterResults[filterSetIdx][yPosTmp][x] = fixedFilterResults[filterSetIdx][yPosTmp][padSize2 - 1 - x];
        }
      }     
#else
      yPosTmp = yPos + padSize - 1;
      fixedFilterResults[filterSetIdx][yPosTmp][0] = fixedFilterResults[filterSetIdx][yPosTmp][3];
      fixedFilterResults[filterSetIdx][yPosTmp][1] = fixedFilterResults[filterSetIdx][yPosTmp][2];
      yPosTmp = yPos + padSize - 2;
      fixedFilterResults[filterSetIdx][yPosTmp][0] = fixedFilterResults[filterSetIdx][yPosTmp][3];
      fixedFilterResults[filterSetIdx][yPosTmp][1] = fixedFilterResults[filterSetIdx][yPosTmp][2];
#endif
    }
    else
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int x = 1; x <= ( isBoundaryValid[0] ? extraSize : padSize ); x++ )
#else
      for( int x = 1; x <= padSize; x++ )
#endif
      {
        xPosTmp = xPos + padSize - x;
        for( int y = 0; y < padSize; y++ )
        {
          fixedFilterResults[filterSetIdx][yPos + padSize - 1 - y][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + y][xPosTmp];
        }
      } 
#else
      xPosTmp = xPos + padSize - 1;
      fixedFilterResults[filterSetIdx][yPos + padSize - 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 0][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize - 2][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 1][xPosTmp];
      xPosTmp = xPos + padSize - 2;
      fixedFilterResults[filterSetIdx][yPos + padSize - 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 0][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize - 2][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 1][xPosTmp];
#endif
    }
  }

  if( !isBoundaryValid[5] )
  {
    if( isBoundaryValid[2] )
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int y = 1; y <= extraSize; y++ )
#else
      for( int y = 1; y <= padSize; y++ )
#endif
      {
        yPosTmp = yPos + padSize - y;
        for( int x = 0; x < padSize; x++ )
        {
          fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + x] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 1 - x];
        }
      }    
#else
      yPosTmp = yPos + padSize - 1;
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 0] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 1];
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 1] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 2];
      yPosTmp = yPos + padSize - 2;
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 0] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 1];
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 1] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 2];
#endif
    }
    else
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int x = 0; x < ( isBoundaryValid[1] ? extraSize : padSize ); x++ )
#else
      for( int x = 0; x < padSize; x++ )
#endif
      {
        xPosTmp = xPos + padSize + width + x;
        for( int y = 0; y < padSize; y++ )
        {
          fixedFilterResults[filterSetIdx][yPos + padSize - 1 - y][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + y][xPosTmp];
        }
      }      
#else
      xPosTmp = xPos + padSize + width + 0;
      fixedFilterResults[filterSetIdx][yPos + padSize - 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 0][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize - 2][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 1][xPosTmp];
      xPosTmp = xPos + padSize + width + 1;
      fixedFilterResults[filterSetIdx][yPos + padSize - 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 0][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize - 2][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + 1][xPosTmp];
#endif
    }
  }

  if( !isBoundaryValid[6] )
  {
    if( isBoundaryValid[3] )
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int y = 0; y < extraSize; y++ )
#else
      for( int y = 0; y < padSize; y++ )
#endif
      {
        yPosTmp = yPos + padSize + height + y;
        for( int x = 0; x < padSize; x++ )
        {
          fixedFilterResults[filterSetIdx][yPosTmp][x] = fixedFilterResults[filterSetIdx][yPosTmp][padSize2 - 1 - x];
        }
      }     
#else
      yPosTmp = yPos + padSize + height + 0;
      fixedFilterResults[filterSetIdx][yPosTmp][0] = fixedFilterResults[filterSetIdx][yPosTmp][3];
      fixedFilterResults[filterSetIdx][yPosTmp][1] = fixedFilterResults[filterSetIdx][yPosTmp][2];
      yPosTmp = yPos + padSize + height + 1;
      fixedFilterResults[filterSetIdx][yPosTmp][0] = fixedFilterResults[filterSetIdx][yPosTmp][3];
      fixedFilterResults[filterSetIdx][yPosTmp][1] = fixedFilterResults[filterSetIdx][yPosTmp][2];
#endif
    }
    else
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int x = 1; x <= ( isBoundaryValid[0] ? extraSize : padSize ); x++ )
#else
      for( int x = 1; x <= padSize; x++ )
#endif
      {
        xPosTmp = xPos + padSize - x;
        for( int y = 0; y < padSize; y++ )
        {
          fixedFilterResults[filterSetIdx][yPos + padSize + height + y][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 1 - y][xPosTmp];          
        }
      }       
#else
      xPosTmp = xPos + padSize - 1;
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 0][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 1][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 2][xPosTmp];
      xPosTmp = xPos + padSize - 2;
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 0][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 1][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 2][xPosTmp];
#endif
    }
  }

  if( !isBoundaryValid[7] )
  {
    if( isBoundaryValid[3] )
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int y = 0; y < extraSize; y++ )
#else
      for( int y = 0; y < padSize; y++ )
#endif
      {
        yPosTmp = yPos + padSize + height + y;
        for( int x = 0; x < padSize; x++ )
        {
          fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + x] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 1 - x];
        }
      }       
#else
      yPosTmp = yPos + padSize + height + 0;
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 0] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 1];
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 1] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 2];
      yPosTmp = yPos + padSize + height + 1;
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 0] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 1];
      fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width + 1] = fixedFilterResults[filterSetIdx][yPosTmp][xPos + padSize + width - 2];
#endif
    }
    else
    {
#if JVET_AD0222_ALF_LONG_FIXFILTER
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      for( int x = 0; x < ( isBoundaryValid[1] ? extraSize : padSize); x++ )
#else
      for( int x = 0; x < padSize; x++ )
#endif
      {
        xPosTmp = xPos + padSize + width + x;
        for( int y = 0; y < padSize; y++ )
        {
          fixedFilterResults[filterSetIdx][yPos + padSize + height + y][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 1 - y][xPosTmp];
        }
      }         
#else
      xPosTmp = xPos + padSize + width + 0;
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 0][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 1][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 2][xPosTmp];
      xPosTmp = xPos + padSize + width + 1;
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 0][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 1][xPosTmp];
      fixedFilterResults[filterSetIdx][yPos + padSize + height + 1][xPosTmp] = fixedFilterResults[filterSetIdx][yPos + padSize + height - 2][xPosTmp];
#endif
    }
  }
}

#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
void AdaptiveLoopFilter::deriveFixedFilterResultsPerBlk( AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const CPelBuf &srcLumaBeforeDb, const Area &blkCur, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx )
#else
void AdaptiveLoopFilter::deriveFixedFilterResultsPerBlk( AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkCur, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int fixedFilterSetIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx )
#endif
{
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  bool useSimd = (blkCur.size().width % 8 == 0) && (blkCur.size().height % 2 == 0) ? true : false;
#else
  bool useSimd = (blkCur.size().width % 8 == 0) && (blkCur.size().height % ALF_PADDING_SIZE_FIXED_RESULTS == 0) ? true : false;
#endif

  int fixedFiltSetIndMin = (qp - 22) / 4;
  int fixedFiltSetIndMax = (qp - 18) / 4;
  if (fixedFiltSetIndMin <= 0)
  {
    fixedFiltSetIndMin = 0;
    fixedFiltSetIndMax = 1;
  }

  if (fixedFiltSetIndMax > (NUM_SETS_FIXED_FILTERS - 1))
  {
    fixedFiltSetIndMin = NUM_SETS_FIXED_FILTERS - 2;
    fixedFiltSetIndMax = NUM_SETS_FIXED_FILTERS - 1;
  }

  int targetFixedFilterSetInd = fixedFilterSetIdx == 0 ? fixedFiltSetIndMin :  fixedFiltSetIndMax;
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  if( classifierIdx == 0 )
  {
#endif
  if(useSimd)
  {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    m_deriveVariance(srcLuma, blkCur, blkCur, laplacian);
#endif
    m_deriveClassificationLaplacian(srcLuma,blkCur, blkCur, laplacian);
  }
  else
  {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
    deriveVariance(srcLuma, blkCur, blkCur, laplacian);
#endif
    deriveClassificationLaplacian(srcLuma,blkCur, blkCur, laplacian);
  }
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
  }
  else
  {
    if( useSimd)
    {
      alfFixedFilterBlk( classifier[ALF_NUM_CLASSIFIER], srcLuma, blkCur, blkCur, srcLumaBeforeDb, fixedFilterResults, m_picWidth, classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx, targetFixedFilterSetInd, classifierIdx, clpRng, clippingValues );
    }
    else
    {
      alfFixedFilterBlkNonSimd( classifier[ALF_NUM_CLASSIFIER], srcLuma, blkCur, blkCur, srcLumaBeforeDb, fixedFilterResults, m_picWidth, classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx, targetFixedFilterSetInd, classifierIdx, clpRng, clippingValues );
    }
  }
  if( classifierIdx == 0 )
#else
  if( classifierIdx > 0 )
#endif
  {
    if(useSimd)
    {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      m_calcClass0( classifier[0], blkCur, blkCur, usedWindowIdx[0], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian);
      alfFixedFilterBlk(classifier[0], srcLuma, blkCur, blkCur, srcLumaBeforeDb, fixedFilterResults, m_picWidth, classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx, targetFixedFilterSetInd, classifierIdx, clpRng, clippingValues );
#endif
      m_deriveClassificationLaplacianBig(blkCur, laplacian);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      m_calcClass0( classifier[ALF_NUM_CLASSIFIER], blkCur, blkCur, usedWindowIdx[1], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian );
#endif
    }
    else
    {
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      calcClass0Var( classifier[0], blkCur, blkCur, usedWindowIdx[0], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian );
      alfFixedFilterBlkNonSimd( classifier[0], srcLuma, blkCur, blkCur, srcLumaBeforeDb, fixedFilterResults, m_picWidth, classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx, targetFixedFilterSetInd, classifierIdx, clpRng, clippingValues );
#endif
      deriveClassificationLaplacianBig(blkCur, laplacian);
#if JVET_AE0139_ALF_IMPROVED_FIXFILTER
      calcClass0Var( classifier[ALF_NUM_CLASSIFIER], blkCur, blkCur, usedWindowIdx[1], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian );
#endif
    }
  }
#if !JVET_AE0139_ALF_IMPROVED_FIXFILTER
  if(useSimd )
  {
    m_calcClass0(classifier, blkCur, blkCur, usedWindowIdx[classifierIdx], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian);
    m_filter13x13Blk(classifier, srcLuma, blkCur, blkCur, fixedFilterResults, m_picWidth, classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx, m_classIdnFixedFilter[targetFixedFilterSetInd][classifierIdx], targetFixedFilterSetInd, classifierIdx, clpRng, clippingValues);
  }
  else
  {
    calcClass(classifier, blkCur, blkCur, usedWindowIdx[classifierIdx], 1, NUM_DIR_FIX, NUM_ACT_FIX, bits, 2, mappingDir, laplacian);
    fixedFiltering(classifier, srcLuma, blkCur, blkCur, fixedFilterResults, m_picWidth, classifierIdx * NUM_FIXED_FILTER_SETS + fixedFilterSetIdx, m_classIdnFixedFilter[targetFixedFilterSetInd][classifierIdx], targetFixedFilterSetInd, classifierIdx, clpRng, clippingValues);
  }
#endif
}
#endif
#if JVET_AD0222_ADDITONAL_ALF_FIXFILTER
void AdaptiveLoopFilter::paddingGaussResultsPic(Pel*** gaussPic, const int storeIdx)
{
  int filterIndex = storeIdx;
  int padSize = ALF_PADDING_SIZE_GAUSS_RESULTS;
  int padSizeDou = padSize << 1;
  int lineWidth = padSizeDou + m_picWidth;
  //left and right
  for(int row = padSize; row < padSize + m_picHeight; row++)
  {
    for(int i = 0; i < padSize; i++)
    {
      gaussPic[filterIndex][row][i]                        = gaussPic[filterIndex][row][padSizeDou - 1 - i];
      gaussPic[filterIndex][row][padSize + m_picWidth + i] = gaussPic[filterIndex][row][padSize + m_picWidth - 1 - i];
    }
  }
  //top and bottom
  for(int i = 0; i < padSize; i++)
  {
    memcpy(gaussPic[filterIndex][i]                        , gaussPic[filterIndex][padSizeDou - 1 - i],            sizeof(Pel) * lineWidth );
    memcpy(gaussPic[filterIndex][padSize + m_picHeight + i], gaussPic[filterIndex][padSize + m_picHeight - 1 - i], sizeof(Pel) * lineWidth );
  }
}

void AdaptiveLoopFilter::paddingGaussResultsCtu(Pel*** gaussPic, Pel*** gaussCtu, const int storeIdx, const Area &blkDst)
{
  int xPos = blkDst.pos().x;
  int yPos = blkDst.pos().y;
  int width = blkDst.size().width;
  int height = blkDst.size().height;

  int filterIndex = storeIdx;
  int padSize = ALF_PADDING_SIZE_GAUSS_RESULTS;
  int padSizeDou = padSize << 1;
  int lineWidth = padSizeDou + width;
  //copy from picBuffer
  for(int row = padSize; row < height + padSize; row++)
  {
    memcpy(&gaussCtu[filterIndex][row][padSize], &gaussPic[filterIndex][row + yPos][padSize + xPos], sizeof(Pel) * width );
    for(int i = 0; i < padSize; i++)
    {
      gaussCtu[filterIndex][row][i]                   = gaussCtu[filterIndex][row][padSizeDou - 1 - i];
      gaussCtu[filterIndex][row][padSize + width + i] = gaussCtu[filterIndex][row][padSize + width - 1 - i];
    }
  }
  //top and bottom
  for(int i = 0; i < padSize; i++)
  {
    memcpy(gaussCtu[filterIndex][i]                   , gaussCtu[filterIndex][padSizeDou - 1 - i]      , sizeof(Pel) * lineWidth );
    memcpy(gaussCtu[filterIndex][padSize + height + i], gaussCtu[filterIndex][padSize + height - 1 - i], sizeof(Pel) * lineWidth );
  }
}

void AdaptiveLoopFilter::deriveGaussResultsCtuBoundary(Pel*** gaussPic, const CPelBuf &srcLuma, const Area &blkDst, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], uint8_t* ctuEnableFlagLuma, uint8_t* ctuEnableOnlineLuma, int ctuIdx, const int filterSetIdx, const int storeIdx )
{
  //Ctu Info
  int yPos = blkDst.pos().y;
  int xPos = blkDst.pos().x;
  int width = blkDst.size().width;
  int height = blkDst.size().height;

  int padSize = ALF_PADDING_SIZE_GAUSS_RESULTS;

  bool isBoundaryValid[8] = {false}; //left, right, top, bottom, topleft, topright, bottomleft, bottomright
  bool isNeighborAvai[8] = {false}; //left, right, top, bottom, topleft, topright, bottomleft, bottomright
  Area blkCur[8];
  int maxCtuBoundaryNum = 8;

  isBoundaryValid[0] = xPos > 0 ? true : false;
  isBoundaryValid[1] = xPos + width < m_picWidth ? true : false;
  isBoundaryValid[2] = yPos > 0 ? true : false;
  isBoundaryValid[3] = yPos + height < m_picHeight ? true : false;
  isBoundaryValid[4] = isBoundaryValid[0] && isBoundaryValid[2];
  isBoundaryValid[5] = isBoundaryValid[1] && isBoundaryValid[2];
  isBoundaryValid[6] = isBoundaryValid[0] && isBoundaryValid[3];
  isBoundaryValid[7] = isBoundaryValid[1] && isBoundaryValid[3];

  Area blkZero = Area(xPos, yPos, 0, 0);
  int extraSize = padSize;

  bool isLeftExtraAvai = isBoundaryValid[0] ? (xPos - 8 >= 0 ? true : false) : false;
  bool isRightExtraAvai = isBoundaryValid[1] ? (xPos + width + 8 - 1 < m_picWidth ? true : false) : false;
  extraSize = (isLeftExtraAvai && isRightExtraAvai) ? 8 : padSize;

  blkCur[0] = isBoundaryValid[0] ? Area(xPos - extraSize, yPos, extraSize, height) : blkZero;
  blkCur[1] = isBoundaryValid[1] ? Area(xPos + width    , yPos, extraSize, height) : blkZero;
  blkCur[2] = isBoundaryValid[2] ? Area(xPos, yPos - padSize, width, padSize) : blkZero;
  blkCur[3] = isBoundaryValid[3] ? Area(xPos, yPos + height,  width, padSize) : blkZero;
  blkCur[4] = isBoundaryValid[4] ? Area(xPos - extraSize, yPos - padSize, extraSize, padSize) : blkZero;
  blkCur[5] = isBoundaryValid[5] ? Area(xPos + width    , yPos - padSize, extraSize, padSize) : blkZero;
  blkCur[6] = isBoundaryValid[6] ? Area(xPos - extraSize, yPos + height, extraSize, padSize) : blkZero;
  blkCur[7] = isBoundaryValid[7] ? Area(xPos + width    , yPos + height, extraSize, padSize) : blkZero;

  int ctuStride = m_numCTUsInWidth;

  isNeighborAvai[0] = isBoundaryValid[0] ? ctuEnableFlagLuma[ctuIdx - 1] && ctuEnableOnlineLuma[ctuIdx - 1] : false;
  isNeighborAvai[1] = isBoundaryValid[1] ? ctuEnableFlagLuma[ctuIdx + 1] && ctuEnableOnlineLuma[ctuIdx + 1] : false;
  isNeighborAvai[2] = isBoundaryValid[2] ? ctuEnableFlagLuma[ctuIdx - ctuStride] && ctuEnableOnlineLuma[ctuIdx - ctuStride] : false;
  isNeighborAvai[3] = isBoundaryValid[3] ? ctuEnableFlagLuma[ctuIdx + ctuStride] && ctuEnableOnlineLuma[ctuIdx + ctuStride] : false;
  isNeighborAvai[4] = isBoundaryValid[4] ? ctuEnableFlagLuma[ctuIdx - ctuStride -1] && ctuEnableOnlineLuma[ctuIdx - ctuStride - 1] : false;
  isNeighborAvai[5] = isBoundaryValid[5] ? ctuEnableFlagLuma[ctuIdx - ctuStride +1] && ctuEnableOnlineLuma[ctuIdx - ctuStride + 1] : false;
  isNeighborAvai[6] = isBoundaryValid[6] ? ctuEnableFlagLuma[ctuIdx + ctuStride -1] && ctuEnableOnlineLuma[ctuIdx + ctuStride - 1] : false;
  isNeighborAvai[7] = isBoundaryValid[7] ? ctuEnableFlagLuma[ctuIdx + ctuStride +1] && ctuEnableOnlineLuma[ctuIdx + ctuStride + 1] : false;

  for(int boundaryIdx = 0; boundaryIdx < maxCtuBoundaryNum; boundaryIdx++)
  {
    if(isBoundaryValid[boundaryIdx] && !isNeighborAvai[boundaryIdx])
    {
      deriveGaussResultsBlk(gaussPic, srcLuma, blkCur[boundaryIdx], blkCur[boundaryIdx], cs, clpRng, clippingValues, filterSetIdx, storeIdx );
    }
  }

  int xPosTmp = xPos;
  int yPosTmp = yPos;

  //left
  if( !isBoundaryValid[0] )
  {
    for(int y = yPos + padSize; y < yPos + padSize + height; y++)
    {
      for(int i = 0; i < padSize; i++)
      {
        gaussPic[storeIdx][y][i] = gaussPic[storeIdx][y][padSize + padSize - 1 - i];
      }
    }
  }
  //right
  if( !isBoundaryValid[1] )
  {
    for(int y = yPos + padSize; y < yPos + padSize + height; y++)
    {
      for(int i = 0; i < padSize; i++)
      {
        gaussPic[storeIdx][y][padSize + m_picWidth + i] = gaussPic[storeIdx][y][padSize + m_picWidth - 1 - i];
      }
    }
  }
  //top
  if( !isBoundaryValid[2] )
  {
    xPosTmp = padSize + xPos + 0;
    for(int i = 0; i < padSize; i++)
    {
      memcpy(&gaussPic[storeIdx][i][xPosTmp], &gaussPic[storeIdx][padSize + padSize - 1 - i][xPosTmp], sizeof(Pel) * width );
    }
  }
  //bottom
  if( !isBoundaryValid[3] )
  {
    xPosTmp = padSize + xPos + 0;
    for(int i = 0; i < padSize; i++)
    {
      memcpy(&gaussPic[storeIdx][padSize + m_picHeight + i][xPosTmp], &gaussPic[storeIdx][padSize + m_picHeight - 1 - i][xPosTmp], sizeof(Pel) * width );
    }
  }
  //leftTop
  if( !isBoundaryValid[4] )
  {
    if( isBoundaryValid[2] )
    {
      //when top is avaiable, use horizontal padding
      for(int i = 1; i < padSize + 1; i++)
      {
        yPosTmp = yPos + padSize - i;
        for(int j = 0; j < padSize; j++)
        {
          gaussPic[storeIdx][yPosTmp][j] = gaussPic[storeIdx][yPosTmp][padSize + padSize - 1 - j];
        }
      }
    }
    else
    {
      //when top is unavaiable, use vertical padding
      xPosTmp = xPos + padSize - padSize;
      yPosTmp = yPos + padSize;
      for(int i = 1; i < padSize + 1; i++)
      {
        memcpy(&gaussPic[storeIdx][yPosTmp - i][xPosTmp], &gaussPic[storeIdx][yPosTmp + i - 1][xPosTmp], sizeof(Pel) * padSize);
      }
    }
  }
  //rightTop
  if( !isBoundaryValid[5] )
  {
    if( isBoundaryValid[2] )
    {
      //when top is avaiable, use horizontal padding
      for(int i = 1; i < padSize + 1; i++)
      {
        yPosTmp = yPos + padSize - i;
        for(int j = 0; j < padSize; j++)
        {
          gaussPic[storeIdx][yPosTmp][xPos + padSize + width + j] = gaussPic[storeIdx][yPosTmp][xPos + padSize + width - 1 - j];
        }
      }
    }
    else
    {
      //when top is unavaiable, use vertical padding
      xPosTmp = xPos + padSize + width + 0;
      yPosTmp = yPos + padSize;
      for(int i = 1; i < padSize + 1; i++)
      {
        memcpy(&gaussPic[storeIdx][yPosTmp - i][xPosTmp], &gaussPic[storeIdx][yPosTmp + i - 1][xPosTmp], sizeof(Pel) * padSize);
      }
    }
  }
  //leftBottom
  if( !isBoundaryValid[6] )
  {
    if( isBoundaryValid[3] )
    {
      //when bottom is avaiable, use horizontal padding
      for(int i = 0; i < padSize; i++)
      {
        yPosTmp = yPos + padSize + height + i;
        for(int j = 0; j < padSize; j++)
        {
          gaussPic[storeIdx][yPosTmp][j] = gaussPic[storeIdx][yPosTmp][padSize + padSize - 1 - j];
        }
      }
    }
    else
    {
      //when bottom is unavaiable, use vertical padding
      xPosTmp = xPos + padSize - padSize;
      yPosTmp = yPos + padSize + height;
      for(int i = 0; i < padSize; i++)
      {
        memcpy(&gaussPic[storeIdx][yPosTmp + i][xPosTmp], &gaussPic[storeIdx][yPosTmp - 1 - i][xPosTmp], sizeof(Pel) * padSize);
      }
    }
  }
  //rightBottom
  if( !isBoundaryValid[7] )
  {
    if( isBoundaryValid[3] )
    {
      //when bottom is avaiable, use horizontal padding
      for(int i = 0; i < padSize; i++)
      {
        yPosTmp = yPos + padSize + height + i;
        for(int j = 0; j < padSize; j++)
        {
          gaussPic[storeIdx][yPosTmp][xPos + padSize + width + j] = gaussPic[storeIdx][yPosTmp][xPos + padSize + width - 1 - j];
        }
      }
    }
    else
    {
      //when bottom is unavaiable, use vertical padding
      xPosTmp = xPos + padSize + width + 0;
      yPosTmp = yPos + padSize + height;
      for(int i = 0; i < padSize; i++)
      {
        memcpy(&gaussPic[storeIdx][yPosTmp + i][xPosTmp], &gaussPic[storeIdx][yPosTmp - 1 - i][xPosTmp], sizeof(Pel) * padSize );
      }
    }
  }
}

void AdaptiveLoopFilter::deriveGaussResults( const CPelBuf& srcLumaBeforeDb, const Area& blkDst, const Area& blk, CodingStructure &cs, const int filterSetIdx, const int storeIdx )
{
  int height = blk.pos().y + blk.height;
  int width = blk.pos().x + blk.width;

  for( int i = blk.pos().y; i < height; i += m_CLASSIFICATION_BLK_SIZE )
  {
    int nHeight = std::min( i + m_CLASSIFICATION_BLK_SIZE, height ) - i;

    for( int j = blk.pos().x; j < width; j += m_CLASSIFICATION_BLK_SIZE )
    {
      int nWidth = std::min( j + m_CLASSIFICATION_BLK_SIZE, width ) - j;
      deriveGaussResultsBlk(m_gaussPic, srcLumaBeforeDb, Area(j - blk.pos().x + blkDst.pos().x, i - blk.pos().y + blkDst.pos().y, nWidth, nHeight), Area(j, i, nWidth, nHeight), cs, m_clpRngs.comp[COMPONENT_Y], m_alfClippingValues[CHANNEL_TYPE_LUMA], filterSetIdx, storeIdx );
    }
  }
}

void AdaptiveLoopFilter::deriveGaussResultsBlk(Pel ***gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, const int storeIdx )
{
  bool useSimd = blkDst.size().width % 8 == 0 ? true : false;
  if( useSimd )
  {
    m_gaussFiltering(cs, gaussPic, srcLuma, blkDst, blk, clpRng, clippingValues, filterSetIdx, storeIdx);
  }
  else
  {
    gaussFiltering(cs, gaussPic, srcLuma, blkDst, blk, clpRng, clippingValues, filterSetIdx, storeIdx);
  }
}

void AdaptiveLoopFilter::gaussFiltering(CodingStructure &cs, Pel ***gaussPic, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const ClpRng &clpRng, const Pel clippingValues[4], int filterSetIdx, int storeIdx )
{
  int strideSrc = srcLuma.stride;
  int xPosSrc = blk.pos().x;
  int yPosSrc = blk.pos().y;
  int width = blk.size().width;
  int height = blk.size().height;
  int padSize = ALF_PADDING_SIZE_GAUSS_RESULTS;
  int shift = 10;
  const int numCoeff = 12;
  int diffTH = 32;

  int gaussTable[NUM_GAUSS_FILTERED_SOURCE][25] =
  {
    {
      8, 22, 30, 22, 22, 60, 85, 60, 22, 8, 30, 85, 119, 85, 30, 8, 22, 60, 85, 60, 22, 22, 30, 22, 8,
    },
  };

  int gaussClipTable[NUM_GAUSS_FILTERED_SOURCE][25] =
  {
    {
        3, 2, 1, 2, 2, 1, 0, 1, 2, 3, 1, 0, 0, 0, 1, 3, 2, 1, 0, 1, 2, 2, 1, 2, 3,
    },
  };

  Pel clipValueTable[numCoeff];
  for(int cIdx = 0; cIdx < numCoeff; cIdx++)
  {
    clipValueTable[cIdx] = clippingValues[gaussClipTable[filterSetIdx][cIdx]];
  }

  const Pel* srcPtr = srcLuma.buf + yPosSrc * strideSrc + xPosSrc;

  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

  for( int i = 0; i < height; i++)
  {
    for (int j = 0; j < width; j++)
    {
      int dstPosY = blkDst.y + i + padSize;
      int dstPosX = blkDst.x + j + padSize;

      pImg0 = srcPtr + i * strideSrc + j;

      pImg1 = pImg0 + strideSrc;
      pImg2 = pImg0 - strideSrc;
      pImg3 = pImg1 + strideSrc;
      pImg4 = pImg2 - strideSrc;
      pImg5 = pImg3 + strideSrc;
      pImg6 = pImg4 - strideSrc;

      int sum = 0;
      int curr = pImg0[+0];
      //Add Gauss Filter Process Here
      Pel refSample[numCoeff];

      refSample[0] = clipALF(clipValueTable[0], curr, pImg6[+0], pImg5[-0]);

      refSample[1] = clipALF(clipValueTable[1], curr, pImg4[-1], pImg3[+1]);
      refSample[2] = clipALF(clipValueTable[2], curr, pImg4[-0], pImg3[+0]);
      refSample[3] = clipALF(clipValueTable[3], curr, pImg4[+1], pImg3[-1]);

      refSample[4] = clipALF(clipValueTable[4], curr, pImg2[-2], pImg1[+2]);
      refSample[5] = clipALF(clipValueTable[5], curr, pImg2[-1], pImg1[+1]);
      refSample[6] = clipALF(clipValueTable[6], curr, pImg2[-0], pImg1[+0]);
      refSample[7] = clipALF(clipValueTable[7], curr, pImg2[+1], pImg1[-1]);
      refSample[8] = clipALF(clipValueTable[8], curr, pImg2[+2], pImg1[-2]);

      refSample[9]  = clipALF(clipValueTable[9] , curr, pImg0[-3], pImg0[+3]);
      refSample[10] = clipALF(clipValueTable[10], curr, pImg0[-2], pImg0[+2]);
      refSample[11] = clipALF(clipValueTable[11], curr, pImg0[-1], pImg0[+1]);

      for(int c = 0; c < numCoeff; c++)
      {
        sum += refSample[c] * gaussTable[filterSetIdx][c];
      }

      sum += 1 << (shift - 1);
      sum >>= shift;

      int diff = Clip3<int>(-diffTH, +diffTH, sum);
      sum = curr + diff;
      gaussPic[storeIdx][dstPosY][dstPosX] = ClipPel(sum, clpRng);

    }//width
  }//height
}
#endif