/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2017, ITU/ISO/IEC
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
#ifndef BILATERALFILTER_H
#define BILATERALFILTER_H

#include "CommonDef.h"

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER

#include "Unit.h"
#include "Buffer.h"
#ifdef TARGET_SIMD_X86
#include <tmmintrin.h>
#include <smmintrin.h>
#include <immintrin.h>
#endif
#if JVET_V0094_BILATERAL_FILTER
class BIFCabacEst
{
public:
  virtual ~BIFCabacEst() {};
  virtual uint64_t getBits( const ComponentID compID, const Slice& slice, const BifParams& htdfParams) = 0;
};
#endif
class BilateralFilter
{
private:
#ifdef TARGET_SIMD_X86
  __m128i tempblockSIMD[2320];
  __m128i tempblockFilteredSIMD[2320];
#else
  int64_t tempblockSIMD[2 * 2320];
  int64_t tempblockFilteredSIMD[2 * 2320];
#endif
  Pel *tempblock = (Pel*) tempblockSIMD;
  Pel *tempblockFilteredTemp = (Pel*) (&tempblockFilteredSIMD[1]);
  // SIMD method writes to tempblockFiltered + 4 so that address
  // must be 128-aligned. Hence tempblockFilteredSIMD is a bit bigger than
  // it otherwise would need to be and we don't use the first 14 bytes.
  // We pad 8 bytes so required sizes is (128+8)*(128+8)*2+16 bytes = 37008 bytes
  // = 2313 128-bit words which has been rounded up to 2320 above. 
  Pel *tempblockFiltered = &tempblockFilteredTemp[-2];

  void (*m_bilateralFilterDiamond5x5)( uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip);
  static void blockBilateralFilterDiamond5x5( uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip );

#if JVET_V0094_BILATERAL_FILTER
  char m_wBIF[26][16] = {
  {  0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {  0,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {  0,   2,   2,   2,   1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0, },
  {  0,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,  -1, },
  {  0,   3,   3,   3,   2,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,  -1, },
  {  0,   4,   4,   4,   3,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,  -1, },
  {  0,   5,   5,   5,   4,   3,   2,   2,   2,   2,   2,   1,   0,   1,   1,  -1, },
  {  0,   6,   7,   7,   5,   3,   3,   3,   3,   2,   2,   1,   1,   1,   1,  -1, },
  {  0,   6,   8,   8,   5,   4,   3,   3,   3,   3,   3,   2,   1,   2,   2,  -2, },
  {  0,   7,  10,  10,   6,   4,   4,   4,   4,   3,   3,   2,   2,   2,   2,  -2, },
  {  0,   8,  11,  11,   7,   5,   5,   4,   5,   4,   4,   2,   2,   2,   2,  -2, },
  {  0,   8,  12,  13,  10,   8,   8,   6,   6,   6,   5,   3,   3,   3,   3,  -2, },
  {  0,   8,  13,  14,  13,  12,  11,   8,   8,   7,   7,   5,   5,   4,   4,  -2, },
  {  0,   9,  14,  16,  16,  15,  14,  11,   9,   9,   8,   6,   6,   5,   6,  -3, },
  {  0,   9,  15,  17,  19,  19,  17,  13,  11,  10,  10,   8,   8,   6,   7,  -3, },
  {  0,   9,  16,  19,  22,  22,  20,  15,  12,  12,  11,   9,   9,   7,   8,  -3, },
  {  0,  10,  17,  21,  24,  25,  24,  20,  18,  17,  15,  12,  11,   9,   9,  -3, },
  {  0,  10,  18,  23,  26,  28,  28,  25,  23,  22,  18,  14,  13,  11,  11,  -3, },
  {  0,  11,  19,  24,  29,  30,  32,  30,  29,  26,  22,  17,  15,  13,  12,  -3, },
  {  0,  11,  20,  26,  31,  33,  36,  35,  34,  31,  25,  19,  17,  15,  14,  -3, },
  {  0,  12,  21,  28,  33,  36,  40,  40,  40,  36,  29,  22,  19,  17,  15,  -3, },
  {  0,  13,  21,  29,  34,  37,  41,  41,  41,  38,  32,  23,  20,  17,  15,  -3, },
  {  0,  14,  22,  30,  35,  38,  42,  42,  42,  39,  34,  24,  20,  17,  15,  -3, },
  {  0,  15,  22,  31,  35,  39,  42,  42,  43,  41,  37,  25,  21,  17,  15,  -3, },
  {  0,  16,  23,  32,  36,  40,  43,  43,  44,  42,  39,  26,  21,  17,  15,  -3, },
  {  0,  17,  23,  33,  37,  41,  44,  44,  45,  44,  42,  27,  22,  17,  15,  -3, },
  };
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  char m_wBIFChroma[26][16] = {
  {   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {   0,   1,   1,   1,   1,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0, },
  {   0,   2,   2,   2,   1,   1,   0,   1,   0,   0,   0,   0,   0,   0,   0,   0, },
  {   0,   2,   2,   2,   2,   1,   1,   1,   1,   1,   1,   1,   0,   1,   1,   -1, },
  {   0,   2,   2,   2,   2,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,   -1, },
  {   0,   3,   3,   3,   2,   2,   1,   2,   1,   1,   1,   1,   0,   1,   1,   -1, },
  {   0,   4,   4,   4,   3,   2,   2,   2,   2,   2,   2,   1,   0,   1,   1,   -1, },
  {   0,   5,   5,   5,   4,   2,   2,   2,   2,   2,   2,   1,   1,   1,   1,   -1, },
  {   0,   5,   6,   6,   4,   3,   2,   2,   2,   2,   2,   2,   1,   2,   2,   -2, },
  {   0,   5,   8,   8,   5,   3,   3,   3,   3,   2,   2,   2,   2,   2,   2,   -2, },
  {   0,   6,   9,   9,   5,   4,   4,   3,   4,   3,   3,   2,   2,   2,   2,   -2, },
  {   0,   6,   9,  10,   8,   6,   6,   5,   5,   5,   4,   2,   2,   2,   2,   -2, },
  {   0,   6,  10,  11,  10,   9,   9,   6,   6,   5,   5,   4,   4,   3,   3,   -2, },
  {   0,   7,  11,  12,  12,  12,  11,   9,   7,   7,   6,   5,   5,   4,   5,   -2, },
  {   0,   7,  12,  13,  15,  15,  13,  10,   9,   8,   8,   6,   6,   5,   5,   -2, },
  {   0,   7,  12,  15,  17,  17,  16,  12,   9,   9,   9,   7,   7,   5,   6,   -2, },
  {   0,   8,  13,  16,  19,  20,  19,  16,  14,  13,  12,   9,   9,   7,   7,   -2, },
  {   0,   8,  14,  18,  20,  22,  22,  20,  18,  17,  14,  11,  10,   9,   9,   -2, },
  {   0,   9,  15,  19,  23,  23,  25,  23,  23,  20,  17,  13,  12,  10,   9,   -2, },
  {   0,   9,  16,  20,  24,  26,  28,  27,  27,  24,  20,  15,  13,  12,  11,   -2, },
  {   0,   9,  16,  22,  26,  28,  31,  31,  31,  28,  23,  17,  15,  13,  12,   -2, },
  {   0,  10,  16,  23,  27,  29,  32,  32,  32,  30,  25,  18,  16,  13,  12,   -2, },
  {   0,  11,  17,  23,  27,  30,  33,  33,  33,  30,  27,  19,  16,  13,  12,   -2, },
  {   0,  12,  17,  24,  27,  30,  33,  33,  34,  32,  29,  20,  16,  13,  12,   -2, },
  {   0,  12,  18,  25,  28,  31,  34,  34,  34,  33,  30,  20,  16,  13,  12,   -2, },
  {   0,  13,  18,  26,  29,  32,  34,  34,  35,  34,  33,  21,  17,  13,  12,   -2, },
  };
#endif
public:
  BilateralFilter();
  ~BilateralFilter();

  void create();
  void destroy();
#if JVET_V0094_BILATERAL_FILTER
  void bilateralFilterRDOdiamond5x5(const ComponentID compID, PelBuf& resiBuf, const CPelBuf& predBuf, PelBuf& recoBuf, int32_t qp, const CPelBuf& recIPredBuf, const ClpRng& clpRng, TransformUnit & currTU, bool useReco, bool doReshape = false, std::vector<Pel>* pLUT = nullptr);
  void bilateralFilterPicRDOperCTU( const ComponentID compID, CodingStructure& cs, PelUnitBuf& src,BIFCabacEst* bifCABACEstimator);
  void bilateralFilterDiamond5x5( const ComponentID compID, const CPelUnitBuf& src, PelUnitBuf& rec, int32_t qp, const ClpRng& clpRng, TransformUnit & currTU, bool noClip
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
    , bool isCtuCrossedByVirtualBoundaries, int horVirBndryPos[], int verVirBndryPos[], int numHorVirBndry, int numVerVirBndry
    , bool clipTop, bool clipBottom, bool clipLeft, bool clipRight
#endif
  );

  const char* getFilterLutParameters( const int size, const PredMode predMode, const int qp, int& bfac );
  void clipNotBilaterallyFilteredBlocks( const ComponentID compID, const CPelUnitBuf& src, PelUnitBuf& rec, const ClpRng& clpRng, TransformUnit & currTU);
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  const char* getFilterLutParametersChroma( const int size, const PredMode predMode, const int qp, int& bfac, int widthForStrength, int heightForStrength, bool isLumaValid);
#endif
#endif


#if ENABLE_SIMD_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER_ENABLE_SIMD
#ifdef TARGET_SIMD_X86
  template<X86_VEXT vext>
  static void simdFilterDiamond5x5( uint32_t uiWidth, uint32_t uiHeight, int16_t block[], int16_t blkFilt[], const ClpRng& clpRng, Pel* recPtr, int recStride, int iWidthExtSIMD, int bfac, int bifRoundAdd, int bifRoundShift, bool isRDO, const char* lutRowPtr, bool noClip );

  void    initBilateralFilterX86();
  template <X86_VEXT vext>
  void    _initBilateralFilterX86();
#endif
#endif
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
  bool isCrossedByVirtualBoundaries(const CodingStructure &cs, const int xPos, const int yPos, const int width,
                                    const int height, bool &clipTop, bool &clipBottom, bool &clipLeft, bool &clipRight,
                                    int &numHorVirBndry, int &numVerVirBndry, int horVirBndryPos[],
                                    int verVirBndryPos[], bool isEncoderRDO = false);
#endif
};

#endif

#endif /* BILATERALFILTER_H */
