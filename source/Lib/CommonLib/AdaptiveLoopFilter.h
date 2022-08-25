/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
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

/** \file     AdaptiveLoopFilter.h
    \brief    adaptive loop filter class (header)
*/

#ifndef __ADAPTIVELOOPFILTER__
#define __ADAPTIVELOOPFILTER__

#include "CommonDef.h"

#include "Unit.h"
#include "UnitTools.h"

#if ALF_IMPROVEMENT
typedef       short           AlfClassifier;         
#else
struct AlfClassifier
{
  AlfClassifier() {}
  AlfClassifier( uint8_t cIdx, uint8_t tIdx )
    : classIdx( cIdx ), transposeIdx( tIdx )
  {
  }
  uint8_t classIdx;
  uint8_t transposeIdx;
};
#endif

enum Direction
{
  HOR,
  VER,
  DIAG0,
  DIAG1,
  NUM_DIRECTIONS
};

class AdaptiveLoopFilter
{
public:
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  static inline Pel clipALF(const Pel clip, const Pel ref, const Pel val0, const Pel val1)
  {
    return Clip3<Pel>(-clip, +clip, val0-ref) + Clip3<Pel>(-clip, +clip, val1-ref);
  }
#else
  static inline int clipALF(const int clip, const short ref, const short val0, const short val1)
  {
    return Clip3<int>(-clip, +clip, val0-ref) + Clip3<int>(-clip, +clip, val1-ref);
  }
#endif
#if ALF_IMPROVEMENT
  static inline int clipALF( const int clip, const short ref, const short val )
  {
    return Clip3<int>( -clip, +clip, val - ref );
  }
#endif
#if JVET_AA0095_ALF_LONGER_FILTER || JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  static inline int numFixedFilters( AlfFilterType filterType )
  {
    return ( filterType >= ALF_FILTER_9_EXT ) ? 2 : 1 ;
  }
#endif
#if JVET_AA0095_ALF_LONGER_FILTER
  void mirroredPaddingForAlf(CodingStructure& cs, const PelUnitBuf& src, int paddingSize, bool enableLuma, bool enableChroma);
#endif
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  void copyDbData( CodingStructure& cs ) { m_tempBufBeforeDb.bufs[COMPONENT_Y].copyFrom( cs.getRecoBuf().bufs[COMPONENT_Y] ); }
#endif

  static constexpr int AlfNumClippingValues[MAX_NUM_CHANNEL_TYPE] = { 4, 4 };
  static constexpr int MaxAlfNumClippingValues = 4;

  static constexpr int   m_NUM_BITS = 8;
#if ALF_IMPROVEMENT
  static constexpr int   m_NUM_BITS_FIXED_FILTER = 12;
  static constexpr int   m_CLASSIFICATION_BLK_SIZE = 256;  //non-normative, local buffer size
#else
  static constexpr int   m_CLASSIFICATION_BLK_SIZE = 32;  //non-normative, local buffer size
#endif
  static constexpr int m_ALF_UNUSED_CLASSIDX = 255;
  static constexpr int m_ALF_UNUSED_TRANSPOSIDX = 255;

  AdaptiveLoopFilter();
  virtual ~AdaptiveLoopFilter() {}
  void reconstructCoeffAPSs(CodingStructure& cs, bool luma, bool chroma, bool isRdo);
  void reconstructCoeff(AlfParam& alfParam, ChannelType channel, const bool isRdo, const bool isRedo = false);
  void ALFProcess(CodingStructure& cs);
  void create( const int picWidth, const int picHeight, const ChromaFormat format, const int maxCUWidth, const int maxCUHeight, const int maxCUDepth, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE] );
  void destroy();
#if RPR_ENABLE
  Size  getAlfSize() { return Size(m_tempBuf.get(COMPONENT_Y).width, m_tempBuf.get(COMPONENT_Y).height); }
#endif

#if ALF_IMPROVEMENT
  void  copyFixedFilterResults(const PelUnitBuf &recDst, const Area &blkDst, ComponentID COMPONENT_Y, Pel*** fixedFilterResults, const int fixedFilterSetIdx, const int classifierIdx);

  static void fixedFiltering(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &cu,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                             const Area &blkDst,
#endif
                             Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd,
                             const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                             const ClpRng &clpRng, const Pel clippingValues[4]);

  int assignAct(int avg_varPrec, int shift, int noAct);
  static void calcClass(AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
  static void deriveClassificationLaplacianBig(const Area &curBlk, uint32_t **laplacian[NUM_DIRECTIONS]);
  static void deriveClassificationLaplacian(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS]);
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void deriveClassificationAndFixFilterResultsBlk( AlfClassifier ***classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int qpIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx, const int multipleClassifierIdx );
  static void calcClassNew( AlfClassifier **classifier, const Area &blkDst, const Area &cu, const CPelBuf& srcLuma, int subBlkSize, AlfClassifier **classifier0, int classifierIdx, int bitDepth );
#else
  void deriveClassificationAndFixFilterResultsBlk( AlfClassifier **classifier, Pel ***fixedFilterResults, const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int bits, CodingStructure& cs, const ClpRng &clpRng, const Pel clippingValues[4], int qp, int qpIdx, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS], const int classifierIdx );
#endif
  template<AlfFilterType filtTypeCcAlf>
  static void filterBlkCcAlf(const PelBuf& dstBuf, const CPelUnitBuf& recSrc, const Area& blkDst, const Area& blkSrc, const ComponentID compId, const int16_t* filterCoeff, const ClpRngs& clpRngs, CodingStructure& cs);
#else
  static void deriveClassificationBlk(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos);
    template<AlfFilterType filtTypeCcAlf>
  static void filterBlkCcAlf(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos);
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void deriveClassification( AlfClassifier*** classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, CodingStructure &cs, const int classifierIdx, const int multipleClassifierIdx );
#else
  void deriveClassification( AlfClassifier** classifier, const CPelBuf& srcLuma, const Area& blkDst, const Area& blk
#if ALF_IMPROVEMENT
  , CodingStructure &cs, const int classifierIdx
#endif
  );
#endif

#if ALF_IMPROVEMENT
  void(*m_calcClass0)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
  void(*m_calcClass1)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, int dirWindSize, int classDir, int noDir, int noAct, int bitDepth, int subBlkSize, int mappingDir[NUM_DIR_FIX][NUM_DIR_FIX], uint32_t **laplacian[NUM_DIRECTIONS]);
#if JVET_X0071_ALF_BAND_CLASSIFIER
  void(*m_calcClass2)(AlfClassifier **classifier, const Area &blkDst, const Area &cu, const CPelBuf& srcLuma, int subBlkSize, AlfClassifier **classifier0, int classifierIdx, int bitDepth);
#endif
  void(*m_deriveClassificationLaplacianBig)(const Area &curBlk, uint32_t **laplacian[NUM_DIRECTIONS]);
  void(*m_deriveClassificationLaplacian)(const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, uint32_t **laplacian[NUM_DIRECTIONS]);
  void (*m_filter13x13Blk)(AlfClassifier **classifier, const CPelBuf &srcLuma, const Area &curBlk,
#if JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY
                           const Area &blkDst,
#endif
                           Pel ***fixedFilterResults, int picWidth, const int fixedFiltInd,
                           const short classIndFixed[NUM_CLASSES_FIX], int fixedFiltQpInd, int dirWindSize,
                           const ClpRng &clpRng, const Pel clippingValues[4]);
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs);
  template<AlfFilterType filtType>
  static void filterBlk(AlfClassifier **classifier, const PelUnitBuf &recDst,
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
    const PelUnitBuf &recBeforeDb,
#endif
    const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#else
    const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs
#endif
                        , Pel ***fixedFilterResults, int fixedFilterSetIdx);   
#else
  void(*m_filterCcAlf)(const PelBuf &dstBuf, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blkSrc, const ComponentID compId, const int16_t *filterCoeff, const ClpRngs &clpRngs, CodingStructure &cs, int vbCTUHeight, int vbPos);
  template<AlfFilterType filtType>
  static void filterBlk(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT                        
                        const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
#else
                        const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
#endif
                        int vbPos);
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
#endif
#else
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
#endif
#endif
#else
  void (*m_deriveClassificationBlk)(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos);
  void (*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
  void (*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const Pel *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
#endif
#else
#if ALF_IMPROVEMENT
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
  void(*m_filter9x9BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExtDb)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const PelUnitBuf &recBeforeDb, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
#endif
#else
  void(*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void  alfFiltering(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, AlfFilterType filterType, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults, int fixedFilterSetIdx);
  void(*m_filter9x9BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
  void(*m_filterBlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
#if JVET_AA0095_ALF_LONGER_FILTER
  void(*m_filter13x13BlkExt)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, Pel ***fixedFilterResults);
#endif
#endif
#else
  void (*m_deriveClassificationBlk)(AlfClassifier **classifier, int **laplacian[NUM_DIRECTIONS], const CPelBuf &srcLuma, const Area &blkDst, const Area &blk, const int shift, const int vbCTUHeight, int vbPos);
  void (*m_filter5x5Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
  void (*m_filter7x7Blk)(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc, const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet, const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight, int vbPos);
#endif
#endif
  void applyCcAlfFilter(CodingStructure &cs, ComponentID compID, const PelBuf &dstBuf, const PelUnitBuf &recYuvExt, uint8_t *   filterControl, const short filterSet[MAX_NUM_CC_ALF_FILTERS][MAX_NUM_CC_ALF_CHROMA_COEFF], const int   selectedFilterIdx);
  CcAlfFilterParam &getCcAlfFilterParam() { return m_ccAlfFilterParam; }
  uint8_t* getCcAlfControlIdc(const ComponentID compID)   { return m_ccAlfFilterControl[compID-1]; }

#ifdef TARGET_SIMD_X86  
  void initAdaptiveLoopFilterX86();
  template <X86_VEXT vext>
  void _initAdaptiveLoopFilterX86();
#endif

protected:
  bool isCrossedByVirtualBoundaries( const CodingStructure& cs, const int xPos, const int yPos, const int width, const int height, bool& clipTop, bool& clipBottom, bool& clipLeft, bool& clipRight, int& numHorVirBndry, int& numVerVirBndry, int horVirBndryPos[], int verVirBndryPos[], int& rasterSliceAlfPad );
  static constexpr int   m_scaleBits = 7; // 8-bits
  CcAlfFilterParam       m_ccAlfFilterParam;
  uint8_t*               m_ccAlfFilterControl[2];
#if !ALF_IMPROVEMENT
  static const int             m_classToFilterMapping[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES];
  static const int             m_fixedFilterSetCoeff[ALF_FIXED_FILTER_NUM][MAX_NUM_ALF_LUMA_COEFF];
  short                        m_fixedFilterSetCoeffDec[NUM_FIXED_FILTER_SETS][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
  short                        m_clipDefault[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
  bool                         m_created = false;
  short                        m_chromaCoeffFinal[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF];
  AlfParam*                    m_alfParamChroma;
  Pel                          m_alfClippingValues[MAX_NUM_CHANNEL_TYPE][MaxAlfNumClippingValues];
  std::vector<AlfFilterShape>  m_filterShapesCcAlf[2];
  std::vector<AlfFilterShape>  m_filterShapes[MAX_NUM_CHANNEL_TYPE];
#if ALF_IMPROVEMENT
  AlfFilterType                m_filterTypeApsLuma[ALF_CTB_MAX_NUM_APS];
  AlfFilterType                m_filterTypeApsChroma;
  static const int             alfNumCoeff[ALF_NUM_OF_FILTER_TYPES];
  bool                         m_filterTypeTest[MAX_NUM_CHANNEL_TYPE][ALF_NUM_OF_FILTER_TYPES];
  int                          m_filterTypeToStatIndex[MAX_NUM_CHANNEL_TYPE][ALF_NUM_OF_FILTER_TYPES];
#endif
#if JVET_X0071_ALF_BAND_CLASSIFIER
  AlfClassifier**              m_classifier[ALF_NUM_CLASSIFIER];
  char                         m_classifierIdxApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA];
  char                         m_classifierFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA];
#else
  AlfClassifier**              m_classifier;
#endif
#if ALF_IMPROVEMENT
  int                          m_numLumaAltAps[ALF_CTB_MAX_NUM_APS];
  short                        m_coeffApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#else  
  short                        m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#endif
  short                        m_coeffFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#else
  short                        m_clippFinal[MAX_NUM_ALF_ALTERNATIVES_LUMA][MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
#else
  short                        m_coeffApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#else
  short                        m_clippApsLuma[ALF_CTB_MAX_NUM_APS][MAX_NUM_ALF_LUMA_COEFF * MAX_NUM_ALF_CLASSES];
#endif
  short                        m_coeffFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  Pel                          m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#else
  short                        m_clippFinal[MAX_NUM_ALF_CLASSES * MAX_NUM_ALF_LUMA_COEFF];
#endif
#endif
  short                        m_chromaClippFinal[MAX_NUM_ALF_ALTERNATIVES_CHROMA][MAX_NUM_ALF_CHROMA_COEFF];
#if ALF_IMPROVEMENT
  Pel***                       m_fixFilterResult;
  const int                    usedWindowIdx[NUM_CLASSIFIER] = { 1, 5 };
  int                          m_mappingDir[NUM_DIR_FIX][NUM_DIR_FIX];
  uint32_t**                   m_laplacian[NUM_DIRECTIONS];
  uint32_t *                   m_laplacianPtr[NUM_DIRECTIONS][(m_CLASSIFICATION_BLK_SIZE + 10) >> 1];
  uint32_t                     m_laplacianData[NUM_DIRECTIONS][(m_CLASSIFICATION_BLK_SIZE + 10)>>1][((m_CLASSIFICATION_BLK_SIZE + 16)>>1) + 8];
#else
  int**                        m_laplacian[NUM_DIRECTIONS];
  int *                        m_laplacianPtr[NUM_DIRECTIONS][m_CLASSIFICATION_BLK_SIZE + 5];
  int                          m_laplacianData[NUM_DIRECTIONS][m_CLASSIFICATION_BLK_SIZE + 5][m_CLASSIFICATION_BLK_SIZE + 5];
#endif
  uint8_t*                     m_ctuEnableFlag[MAX_NUM_COMPONENT];
  uint8_t*                     m_ctuAlternative[MAX_NUM_COMPONENT];
  PelStorage                   m_tempBuf;
  PelStorage                   m_tempBuf2;
#if JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF
  PelStorage                   m_tempBufBeforeDb;
  PelStorage                   m_tempBufBeforeDb2;
#endif
  int                          m_inputBitDepth[MAX_NUM_CHANNEL_TYPE];
  int                          m_picWidth;
  int                          m_picHeight;
  int                          m_maxCUWidth;
  int                          m_maxCUHeight;
  int                          m_maxCUDepth;
  int                          m_numCTUsInWidth;
  int                          m_numCTUsInHeight;
  int                          m_numCTUsInPic;
#if !ALF_IMPROVEMENT
  int                          m_alfVBLumaPos;
  int                          m_alfVBChmaPos;
  int                          m_alfVBLumaCTUHeight;
  int                          m_alfVBChmaCTUHeight;
#endif
  ChromaFormat                 m_chromaFormat;
  ClpRngs                      m_clpRngs;
};

#endif
