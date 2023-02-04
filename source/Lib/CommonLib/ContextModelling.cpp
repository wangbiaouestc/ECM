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

/** \file     ContextModelling.cpp
    \brief    Classes providing probability descriptions and contexts
*/

#include "ContextModelling.h"
#include "UnitTools.h"
#include "CodingStructure.h"
#include "Picture.h"


CoeffCodingContext::CoeffCodingContext( const TransformUnit& tu, ComponentID component, bool signHide, bool bdpcm )
  : m_compID                    (component)
  , m_chType                    (toChannelType(m_compID))
  , m_width                     (tu.block(m_compID).width)
  , m_height                    (tu.block(m_compID).height)
  , m_log2CGWidth               ( g_log2SbbSize[ floorLog2(m_width) ][ floorLog2(m_height) ][0] )
  , m_log2CGHeight              ( g_log2SbbSize[ floorLog2(m_width) ][ floorLog2(m_height) ][1] )
  , m_log2CGSize                (m_log2CGWidth + m_log2CGHeight)
  , m_widthInGroups(std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) >> m_log2CGWidth)
  , m_heightInGroups(std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) >> m_log2CGHeight)
  , m_log2BlockWidth            ((unsigned)floorLog2(m_width))
  , m_log2BlockHeight           ((unsigned)floorLog2(m_height))
  , m_maxNumCoeff               (m_width * m_height)
  , m_signHiding                (signHide)
  , m_extendedPrecision         (tu.cs->sps->getSpsRangeExtension().getExtendedPrecisionProcessingFlag())
  , m_maxLog2TrDynamicRange     (tu.cs->sps->getMaxLog2TrDynamicRange(m_chType))
  , m_scanType                  (SCAN_DIAG)
  , m_scan                      (g_scanOrder     [SCAN_GROUPED_4x4][m_scanType][gp_sizeIdxInfo->idxFrom(m_width        )][gp_sizeIdxInfo->idxFrom(m_height        )])
  , m_scanCG                    (g_scanOrder     [SCAN_UNGROUPED  ][m_scanType][gp_sizeIdxInfo->idxFrom(m_widthInGroups)][gp_sizeIdxInfo->idxFrom(m_heightInGroups)])
  , m_CtxSetLastX               (Ctx::LastX[m_chType])
  , m_CtxSetLastY               (Ctx::LastY[m_chType])
  , m_maxLastPosX(g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_width) - 1])
  , m_maxLastPosY(g_uiGroupIdx[std::min<unsigned>(JVET_C0024_ZERO_OUT_TH, m_height) - 1])
  , m_lastOffsetX               (0)
  , m_lastOffsetY               (0)
  , m_lastShiftX                (0)
  , m_lastShiftY                (0)
  , m_TrafoBypass               (tu.cs->sps->getSpsRangeExtension().getTransformSkipContextEnabledFlag() && (tu.mtsIdx[m_compID] == MTS_SKIP))
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  , m_minCoeff                  (-(1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)))
  , m_maxCoeff                  ((1 << tu.cs->sps->getMaxLog2TrDynamicRange(m_chType)) - 1)
#endif
  , m_scanPosLast               (-1)
  , m_subSetId                  (-1)
  , m_subSetPos                 (-1)
  , m_subSetPosX                (-1)
  , m_subSetPosY                (-1)
  , m_minSubPos                 (-1)
  , m_maxSubPos                 (-1)
  , m_sigGroupCtxId             (-1)
  , m_tmplCpSum1                (-1)
  , m_tmplCpDiag                (-1)
#if TCQ_8STATES
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+4] }
#else
  , m_sigFlagCtxSet             { Ctx::SigFlag[m_chType], Ctx::SigFlag[m_chType+2], Ctx::SigFlag[m_chType+4] }
#endif
  , m_parFlagCtxSet             ( Ctx::ParFlag[m_chType] )
  , m_gtxFlagCtxSet             { Ctx::GtxFlag[m_chType], Ctx::GtxFlag[m_chType+2] }
  , m_sigGroupCtxIdTS           (-1)
  , m_tsSigFlagCtxSet           ( Ctx::TsSigFlag )
  , m_tsParFlagCtxSet           ( Ctx::TsParFlag )
  , m_tsGtxFlagCtxSet           ( Ctx::TsGtxFlag )
  , m_tsLrg1FlagCtxSet          (Ctx::TsLrg1Flag)
  , m_tsSignFlagCtxSet          (Ctx::TsResidualSign)
  , m_sigCoeffGroupFlag         ()
  , m_bdpcm                     (bdpcm)
#if SIGN_PREDICTION
  , m_bSignPredQualified        (TU::getDelayedSignCoding(tu, component))
#endif
{
  // LOGTODO
  unsigned log2sizeX = m_log2BlockWidth;
  unsigned log2sizeY = m_log2BlockHeight;
  if (m_chType == CHANNEL_TYPE_CHROMA)
  {
    const_cast<int&>(m_lastShiftX) = Clip3( 0, 2, int( m_width  >> 3) );
    const_cast<int&>(m_lastShiftY) = Clip3( 0, 2, int( m_height >> 3) );
  }
  else
  {
#if TU_256
    static const int prefix_ctx[]   = { 0, 0, 0, 3, 6, 10, 15, 21, 28 };
#else
    static const int prefix_ctx[8]  = { 0, 0, 0, 3, 6, 10, 15, 21 };
#endif
    const_cast<int&>(m_lastOffsetX) = prefix_ctx[ log2sizeX ];
    const_cast<int&>(m_lastOffsetY) = prefix_ctx[ log2sizeY ];
    const_cast<int&>(m_lastShiftX)  = (log2sizeX + 1) >> 2;
    const_cast<int&>(m_lastShiftY)  = (log2sizeY + 1) >> 2;
  }
}

void CoeffCodingContext::initSubblock( int SubsetId, bool sigGroupFlag )
{
  m_subSetId                = SubsetId;
  m_subSetPos               = m_scanCG[m_subSetId].idx;
  m_subSetPosY              = m_subSetPos / m_widthInGroups;
  m_subSetPosX              = m_subSetPos - ( m_subSetPosY * m_widthInGroups );
  m_minSubPos               = m_subSetId << m_log2CGSize;
  m_maxSubPos               = m_minSubPos + ( 1 << m_log2CGSize ) - 1;
  if( sigGroupFlag )
  {
    m_sigCoeffGroupFlag.set ( m_subSetPos );
  }
  unsigned  CGPosY    = m_subSetPosY;
  unsigned  CGPosX    = m_subSetPosX;
  unsigned  sigRight  = unsigned( ( CGPosX + 1 ) < m_widthInGroups  ? m_sigCoeffGroupFlag[ m_subSetPos + 1               ] : false );
  unsigned  sigLower  = unsigned( ( CGPosY + 1 ) < m_heightInGroups ? m_sigCoeffGroupFlag[ m_subSetPos + m_widthInGroups ] : false );
  m_sigGroupCtxId     = Ctx::SigCoeffGroup[m_chType]( sigRight | sigLower );
  unsigned  sigLeft   = unsigned( CGPosX > 0 ? m_sigCoeffGroupFlag[m_subSetPos - 1              ] : false );
  unsigned  sigAbove  = unsigned( CGPosY > 0 ? m_sigCoeffGroupFlag[m_subSetPos - m_widthInGroups] : false );
  m_sigGroupCtxIdTS   = Ctx::TsSigCoeffGroup( sigLeft  + sigAbove );
}

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
unsigned DeriveCtx::CtxModeConsFlag( const CodingStructure& cs, Partitioner& partitioner )
{
  assert( partitioner.chType == CHANNEL_TYPE_LUMA );
  const Position pos = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  unsigned ctxId = ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;
  return ctxId;
}
#endif

void DeriveCtx::CtxSplit( const CodingStructure& cs, Partitioner& partitioner, unsigned& ctxSpl, unsigned& ctxQt, unsigned& ctxHv, unsigned& ctxHorBt, unsigned& ctxVerBt, bool* _canSplit /*= nullptr */ )
{
  const Position pos         = partitioner.currArea().blocks[partitioner.chType];
  const unsigned curSliceIdx = cs.slice->getIndependentSliceIdx();
  const unsigned curTileIdx  = cs.pps->getTileIdx( partitioner.currArea().lumaPos() );

  // get left depth
  const CodingUnit* cuLeft = cs.getCURestricted( pos.offset( -1, 0 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  // get above depth
  const CodingUnit* cuAbove = cs.getCURestricted( pos.offset( 0, -1 ), pos, curSliceIdx, curTileIdx, partitioner.chType );

  bool canSplit[6];

  if( _canSplit == nullptr )
  {
    partitioner.canSplit( cs, canSplit[0], canSplit[1], canSplit[2], canSplit[3], canSplit[4], canSplit[5] );
  }
  else
  {
    memcpy( canSplit, _canSplit, 6 * sizeof( bool ) );
  }

  ///////////////////////
  // CTX do split (0-8)
  ///////////////////////
  const unsigned widthCurr  = partitioner.currArea().blocks[partitioner.chType].width;
  const unsigned heightCurr = partitioner.currArea().blocks[partitioner.chType].height;

  ctxSpl = 0;

  if( cuLeft )
  {
    const unsigned heightLeft = cuLeft->blocks[partitioner.chType].height;
    ctxSpl += ( heightLeft < heightCurr ? 1 : 0 );
  }
  if( cuAbove )
  {
    const unsigned widthAbove = cuAbove->blocks[partitioner.chType].width;
    ctxSpl += ( widthAbove < widthCurr ? 1 : 0 );
  }

  unsigned numSplit = 0;
  if (canSplit[1])
  {
    numSplit += 2;
  }
  if (canSplit[2])
  {
    numSplit += 1;
  }
  if (canSplit[3])
  {
    numSplit += 1;
  }
  if (canSplit[4])
  {
    numSplit += 1;
  }
  if (canSplit[5])
  {
    numSplit += 1;
  }

  if (numSplit > 0)
  {
    numSplit--;
  }

  ctxSpl += 3 * ( numSplit >> 1 );

  //////////////////////////
  // CTX is qt split (0-5)
  //////////////////////////
  ctxQt =  ( cuLeft  && cuLeft->qtDepth  > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += ( cuAbove && cuAbove->qtDepth > partitioner.currQtDepth ) ? 1 : 0;
  ctxQt += partitioner.currQtDepth < 2 ? 0 : 3;

  ////////////////////////////
  // CTX is ver split (0-4)
  ////////////////////////////
  ctxHv = 0;

  const unsigned numHor = ( canSplit[2] ? 1 : 0 ) + ( canSplit[4] ? 1 : 0 );
  const unsigned numVer = ( canSplit[3] ? 1 : 0 ) + ( canSplit[5] ? 1 : 0 );

  if( numVer == numHor )
  {
    const Area& area = partitioner.currArea().blocks[partitioner.chType];

    const unsigned wAbove       = cuAbove ? cuAbove->blocks[partitioner.chType].width  : 1;
    const unsigned hLeft        = cuLeft  ? cuLeft ->blocks[partitioner.chType].height : 1;

    const unsigned depAbove     = area.width / wAbove;
    const unsigned depLeft      = area.height / hLeft;

    if (depAbove == depLeft || !cuLeft || !cuAbove)
    {
      ctxHv = 0;
    }
    else if (depAbove < depLeft)
    {
      ctxHv = 1;
    }
    else
    {
      ctxHv = 2;
    }
  }
  else if( numVer < numHor )
  {
    ctxHv = 3;
  }
  else
  {
    ctxHv = 4;
  }

  //////////////////////////
  // CTX is h/v bt (0-3)
  //////////////////////////
  ctxHorBt = ( partitioner.currMtDepth <= 1 ? 1 : 0 );
  ctxVerBt = ( partitioner.currMtDepth <= 1 ? 3 : 2 );
}

unsigned DeriveCtx::CtxQtCbf( const ComponentID compID, const bool prevCbf, const int ispIdx )
{
  if( ispIdx && isLuma( compID ) )
  {
    return 2 + (int)prevCbf;
  }
  if( compID == COMPONENT_Cr )
  {
    return ( prevCbf ? 1 : 0 );
  }
  return 0;
}

unsigned DeriveCtx::CtxInterDir( const PredictionUnit& pu )
{
#if CTU_256
  return ( MAX_CU_DEPTH - ( ( floorLog2( pu.lumaSize().width ) + floorLog2( pu.lumaSize().height ) + 1 ) >> 1 ) );
#else
  return ( 7 - ((floorLog2(pu.lumaSize().width) + floorLog2(pu.lumaSize().height) + 1) >> 1) );
#endif
}

#if JVET_X0049_ADAPT_DMVR
unsigned DeriveCtx::CtxBMMrgFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && (cuLeft->firstPU->bmMergeFlag || (!cuLeft->firstPU->mergeFlag && cuLeft->firstPU->interDir == 3))) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && (cuAbove->firstPU->bmMergeFlag || (!cuAbove->firstPU->mergeFlag && cuAbove->firstPU->interDir == 3))) ? 1 : 0;

  return ctxId;
}
#endif

#if JVET_AA0070_RRIBC
unsigned DeriveCtx::CtxRribcFlipType(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && cuLeft->predMode == MODE_IBC && !cuLeft->firstPU->mergeFlag && cuLeft->rribcFlipType) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->predMode == MODE_IBC && !cuAbove->firstPU->mergeFlag && cuAbove->rribcFlipType) ? 1 : 0;

  return ctxId;
}
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
unsigned DeriveCtx::CtxBvOneNullComp(const CodingUnit &cu)
{
  const CodingStructure *cs    = cu.cs;
  unsigned               ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && cuLeft->predMode == MODE_IBC && !cuLeft->firstPU->mergeFlag && cuLeft->bvOneNullComp) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->predMode == MODE_IBC && !cuAbove->firstPU->mergeFlag && cuAbove->bvOneNullComp) ? 1 : 0;

  return ctxId;
}
#endif

unsigned DeriveCtx::CtxAffineFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->affine ) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->affine ) ? 1 : 0;

  return ctxId;
}
unsigned DeriveCtx::CtxSkipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = ( cuLeft && cuLeft->skip ) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += ( cuAbove && cuAbove->skip ) ? 1 : 0;

  return ctxId;
}

#if ENABLE_DIMD 
unsigned DeriveCtx::CtxDIMDFlag(const CodingUnit& cu)
{
#if 1 // one context
  return 0;
#else
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  // Get BCBP of left PU
  const CodingUnit *cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId = (cuLeft && cuLeft->dimd) ? 1 : 0;

  // Get BCBP of above PU
  const CodingUnit *cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->dimd) ? 1 : 0;

  return ctxId;
#endif
}
#endif

#if JVET_W0123_TIMD_FUSION
unsigned DeriveCtx::CtxTimdFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = (cuLeft && cuLeft->timd) ? 1 : 0;
  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += (cuAbove && cuAbove->timd) ? 1 : 0;
  return ctxId;
}
#endif

#if JVET_AB0155_SGPM
unsigned DeriveCtx::CtxSgpmFlag(const CodingUnit &cu)
{
  const CodingStructure *cs     = cu.cs;
  unsigned               ctxId  = 0;
  const CodingUnit *     cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  ctxId                         = (cuLeft && cuLeft->sgpm) ? 1 : 0;
  const CodingUnit *cuAbove     = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
  ctxId += (cuAbove && cuAbove->sgpm) ? 1 : 0;
  return ctxId;
}
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
int DeriveCtx::CtxSmBvdBin(const int iPreviousBinIsCorrect2, const int iPreviousBinIsCorrect, const int isHor, const int significance)
{
  bool constexpr checkPrev2 = false;
  const int ctxNumInGroup = checkPrev2 ? 5 : 2;
  CHECK(iPreviousBinIsCorrect2 < -1, "Illegal iPreviousBinIsCorrect2");
  int iCtxIdx = (0 == iPreviousBinIsCorrect) ? 0 : 1;
  if (checkPrev2)
  {
    if (iCtxIdx == 0)
    {
      iCtxIdx = iPreviousBinIsCorrect2 < 0 ? 2 : iPreviousBinIsCorrect2; // # 0, 1 and 2 for cases 00, 10 and X0
    }
    else // (iCtxIdx == 1) 
    {
      iCtxIdx = 2 + (iPreviousBinIsCorrect2 < 0 ? 2 : iPreviousBinIsCorrect2); // # 2, 3 and 4 for cases 01, 11 and X1
    }
  }
  if (significance < 5)
  {
    return iCtxIdx;
  }
  return ctxNumInGroup + (0 == isHor ? 0 : ctxNumInGroup) + iCtxIdx;
}
#endif //JVET_AC0104_IBC_BVD_PREDICTION

unsigned DeriveCtx::CtxPredModeFlag( const CodingUnit& cu )
{
  const CodingUnit *cuLeft  = cu.cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
  const CodingUnit *cuAbove = cu.cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);

  unsigned ctxId = ((cuAbove && cuAbove->predMode == MODE_INTRA) || (cuLeft && cuLeft->predMode == MODE_INTRA)) ? 1 : 0;

  return ctxId;
}
unsigned DeriveCtx::CtxIBCFlag(const CodingUnit& cu)
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;
  const Position pos = cu.chType == CHANNEL_TYPE_CHROMA ? cu.chromaPos() : cu.lumaPos();
  const CodingUnit *cuLeft = cs->getCURestricted(pos.offset(-1, 0), cu, cu.chType);
  ctxId += (cuLeft && CU::isIBC(*cuLeft)) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted(pos.offset(0, -1), cu, cu.chType);
  ctxId += (cuAbove && CU::isIBC(*cuAbove)) ? 1 : 0;
  return ctxId;
}

#if TM_MRG
void MergeCtx::copyRegularMergeCand(int dstCandIdx, MergeCtx& srcCtx, int srcCandIdx)
{
  if (this == (&srcCtx) && dstCandIdx == srcCandIdx)
  {
    return;
  }

  mvFieldNeighbours[ dstCandIdx << 1     ] = srcCtx.mvFieldNeighbours[ srcCandIdx << 1     ];
  mvFieldNeighbours[(dstCandIdx << 1) + 1] = srcCtx.mvFieldNeighbours[(srcCandIdx << 1) + 1];
  bcwIdx            [dstCandIdx] = srcCtx.bcwIdx            [srcCandIdx];
#if INTER_LIC
  licFlags          [dstCandIdx] = srcCtx.licFlags          [srcCandIdx];
#endif
  interDirNeighbours[dstCandIdx] = srcCtx.interDirNeighbours[srcCandIdx];
  useAltHpelIf      [dstCandIdx] = srcCtx.useAltHpelIf      [srcCandIdx];
#if MULTI_HYP_PRED
  addHypNeighbours  [dstCandIdx] = srcCtx.addHypNeighbours  [srcCandIdx];
#endif
}

void MergeCtx::convertRegularMergeCandToBi(int candIdx)
{
  if( interDirNeighbours[candIdx] < 3
#if INTER_LIC
      && !licFlags[candIdx]
#endif
    )
  {
    MvField& mvfL0 = mvFieldNeighbours[ candIdx << 1     ];
    MvField& mvfL1 = mvFieldNeighbours[(candIdx << 1) + 1];

    if (mvfL0.refIdx >= 0 && mvfL1.refIdx < 0)
    {
      mvfL1.mv = Mv() - mvfL0.mv;
      mvfL1.refIdx = 0;
    }
    else if (mvfL1.refIdx >= 0 && mvfL0.refIdx < 0)
    {
      mvfL0.mv = Mv() - mvfL1.mv;
      mvfL0.refIdx = 0;
    }
    else
    {
      CHECK(true, "Invalid merge candidate");
    }

    interDirNeighbours[candIdx] = 3;
    bcwIdx            [candIdx] = BCW_DEFAULT;
  } 
}
#endif
#if ENABLE_INTER_TEMPLATE_MATCHING && JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
void MergeCtx::saveMergeInfo(PredictionUnit& puTmp, PredictionUnit pu)
{
  puTmp.mergeIdx = pu.mergeIdx;
#if !JVET_Z0075_IBC_HMVP_ENLARGE
  CHECK(candIdx >= numValidMergeCand, "Merge candidate does not exist");
#endif

  puTmp.regularMergeFlag = pu.regularMergeFlag;
  puTmp.mergeFlag = pu.mergeFlag;
  puTmp.mmvdMergeFlag = pu.mmvdMergeFlag;
  puTmp.interDir = pu.interDir;
  puTmp.cu->imv = pu.cu->imv;
  puTmp.mergeType = pu.mergeType;
  puTmp.mv[REF_PIC_LIST_0] = pu.mv[REF_PIC_LIST_0];
  puTmp.mv[REF_PIC_LIST_1] = pu.mv[REF_PIC_LIST_1];
#if MULTI_PASS_DMVR
  puTmp.bdmvrRefine = pu.bdmvrRefine;
#endif
  puTmp.refIdx[REF_PIC_LIST_0] = pu.refIdx[REF_PIC_LIST_0];
  puTmp.refIdx[REF_PIC_LIST_1] = pu.refIdx[REF_PIC_LIST_1];

  puTmp.bv = pu.bv;

  puTmp.cu->bcwIdx = pu.cu->bcwIdx;
  puTmp.addHypData = pu.addHypData;
  puTmp.numMergedAddHyps = pu.numMergedAddHyps;

  puTmp.mmvdEncOptMode = pu.mmvdEncOptMode;
  puTmp.cu->licFlag = pu.cu->licFlag;
}
#endif
void MergeCtx::setMergeInfo( PredictionUnit& pu, int candIdx )
{
#if JVET_X0049_ADAPT_DMVR
  pu.mergeIdx = candIdx;
  if (pu.bmMergeFlag && pu.bmDir == 2)
  {
    candIdx -= BM_MRG_MAX_NUM_CANDS;
  }
#endif
#if !JVET_Z0075_IBC_HMVP_ENLARGE
  CHECK( candIdx >= numValidMergeCand, "Merge candidate does not exist" );
#endif

  pu.regularMergeFlag        = !(pu.ciipFlag || pu.cu->geoFlag);
  pu.mergeFlag               = true;
  pu.mmvdMergeFlag = false;
  pu.interDir                = interDirNeighbours[candIdx];
  pu.cu->imv = (!pu.cu->geoFlag && useAltHpelIf[candIdx]) ? IMV_HPEL : 0;
#if !JVET_X0049_ADAPT_DMVR
  pu.mergeIdx                = candIdx;
#endif
  pu.mergeType               = CU::isIBC( *pu.cu ) ? MRG_TYPE_IBC : MRG_TYPE_DEFAULT_N;
  pu.mv     [REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].mv;
  pu.mv     [REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].mv;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif
  pu.refIdx[REF_PIC_LIST_0] = mvFieldNeighbours[(candIdx << 1) + 0].refIdx;
  pu.refIdx[REF_PIC_LIST_1] = mvFieldNeighbours[(candIdx << 1) + 1].refIdx;

  if (CU::isIBC(*pu.cu))
  {
    pu.bv = pu.mv[REF_PIC_LIST_0];
    pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // used for only integer resolution
    pu.cu->imv = pu.cu->imv == IMV_HPEL ? 0 : pu.cu->imv;
#if MULTI_HYP_PRED
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
#endif
#if JVET_AC0112_IBC_LIC
    pu.cu->ibcLicFlag = ibcLicFlags[candIdx];
#endif
#if JVET_AA0070_RRIBC
    pu.cu->rribcFlipType = rribcFlipTypes[candIdx];
  }
  else
  {
    pu.cu->rribcFlipType = 0;
#endif
  }
  pu.cu->bcwIdx = ( interDirNeighbours[candIdx] == 3 ) ? bcwIdx[candIdx] : BCW_DEFAULT;
#if MULTI_HYP_PRED
  if (pu.ciipFlag
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    || pu.tmMergeFlag
#endif
#if JVET_X0049_ADAPT_DMVR
    || pu.bmMergeFlag
#endif
    )
  {
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
  }
#if MULTI_HYP_PRED
  else if (pu.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE || std::min(pu.Y().width, pu.Y().height) < MULTI_HYP_PRED_RESTRICT_MIN_WH)
  {
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
  }
#endif
  else
  {
    pu.addHypData = addHypNeighbours[candIdx];
    pu.numMergedAddHyps = int(addHypNeighbours[candIdx].size());
  }
#endif

#if !INTER_RM_SIZE_CONSTRAINTS  
  PU::restrictBiPredMergeCandsOne(pu);
#endif
  pu.mmvdEncOptMode = 0;

#if INTER_LIC
  pu.cu->licFlag = pu.cs->slice->getUseLIC() ? licFlags[candIdx] : false;
  if (pu.interDir == 3)
  {
    CHECK(pu.cu->licFlag, "LIC is not used with bi-prediction in merge");
  }
#endif
}
#if ENABLE_INTER_TEMPLATE_MATCHING && JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION                                
bool MergeCtx::xCheckSimilarMotionSubTMVP(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (interDirNeighbours[mergeCandIndex] == 0)
  {
    return true;
  }
  Mv cVector;

  CHECK(interDirNeighbours[mergeCandIndex] != 1 && interDirNeighbours[mergeCandIndex] != 2, "Wrong interDir.");

  cVector = (interDirNeighbours[mergeCandIndex] == 1) ? mvFieldNeighbours[(mergeCandIndex << 1)].mv : mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;
  cVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    Mv cTempVector = (interDirNeighbours[ui] == 1) ? mvFieldNeighbours[(ui << 1)].mv : mvFieldNeighbours[(ui << 1) + 1].mv;
    cTempVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
    Mv mvDiff = cTempVector - cVector;
    if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
    {
      return true;
    }
  }
  return false;
}
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || MULTI_PASS_DMVR || JVET_W0097_GPM_MMVD_TM || (JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM) || JVET_Y0058_IBC_LIST_MODIFY
#if JVET_Z0075_IBC_HMVP_ENLARGE
bool MergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh, int compareNum) const
#else
bool MergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
#endif
{
  if (mvFieldNeighbours[(mergeCandIndex << 1)].refIdx < 0 && mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx < 0)
  {
    return true;
  }

#if JVET_Z0075_IBC_HMVP_ENLARGE
  if (compareNum == -1)
  {
    compareNum = mergeCandIndex;
  }
#endif
  if (mvdSimilarityThresh > 1)
  {
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)    ].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)    ].refIdx &&
              mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx )
          {
            Mv mvDiffL0 = mvFieldNeighbours[(ui << 1)    ].mv - mvFieldNeighbours[(mergeCandIndex << 1)    ].mv;
            Mv mvDiffL1 = mvFieldNeighbours[(ui << 1) + 1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;

            if (mvDiffL0.getAbsHor() < mvdSimilarityThresh && mvDiffL0.getAbsVer() < mvdSimilarityThresh
             && mvDiffL1.getAbsHor() < mvdSimilarityThresh && mvDiffL1.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)].refIdx )
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1)].mv - mvFieldNeighbours[(mergeCandIndex << 1)].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx )
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1) + 1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
      }
    }

    return false;
  }

#if JVET_Z0075_IBC_HMVP_ENLARGE
  for (uint32_t ui = 0; ui < compareNum; ui++)
#else
  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
#endif
  {
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)    ].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)    ].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
            mvFieldNeighbours[(ui << 1)    ].mv     == mvFieldNeighbours[(mergeCandIndex << 1)    ].mv     &&
            mvFieldNeighbours[(ui << 1) + 1].mv     == mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
            mvFieldNeighbours[(ui << 1)].mv     == mvFieldNeighbours[(mergeCandIndex << 1)].mv)
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1].mv     == mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
          return true;
        }
      }
    }
  }

  return false;
}
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
bool MergeCtx::xCheckSimilarMotion2Lists(int mergeCandIndex, MergeCtx *mrgCtx, uint32_t mvdSimilarityThresh ) const
{
  if (mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx < 0 && mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    for (uint32_t ui = 0; ui < numValidMergeCand; ui++)
    {
      if (interDirNeighbours[ui] == mrgCtx->interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
            mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx)
          {
            Mv mvDiffL0 = mvFieldNeighbours[(ui << 1)].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv;
            Mv mvDiffL1 = mvFieldNeighbours[(ui << 1) + 1].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;

            if (mvDiffL0.getAbsHor() < mvdSimilarityThresh && mvDiffL0.getAbsVer() < mvdSimilarityThresh
              && mvDiffL1.getAbsHor() < mvdSimilarityThresh && mvDiffL1.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx)
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1)].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx)
          {
            Mv mvDiff = mvFieldNeighbours[(ui << 1) + 1].mv - mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv;
            if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }
  for (uint32_t ui = 0; ui < numValidMergeCand; ui++)
  {
    if (interDirNeighbours[ui] == mrgCtx->interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
          mvFieldNeighbours[(ui << 1)].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv &&
          mvFieldNeighbours[(ui << 1) + 1].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].refIdx &&
          mvFieldNeighbours[(ui << 1)].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1)].mv)
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1].refIdx == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1].mv == mrgCtx->mvFieldNeighbours[(mergeCandIndex << 1) + 1].mv)
        {
          return true;
        }
      }
    }
  }
  return false;
}
#endif
#if JVET_Z0084_IBC_TM
#if JVET_Z0075_IBC_HMVP_ENLARGE
bool MergeCtx::xCheckSimilarIBCMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh, int compareNum) const
#else
bool MergeCtx::xCheckSimilarIBCMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
#endif
{
  if (mvFieldNeighbours[mergeCandIndex << 1].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      Mv mvDiff = mvFieldNeighbours[ui << 1].mv - mvFieldNeighbours[mergeCandIndex << 1].mv;
      if (mvDiff.getAbsHor() < mvdSimilarityThresh && mvDiff.getAbsVer() < mvdSimilarityThresh)
      {
        return true;
      }
    }
  }
  else
  {
#if JVET_Z0075_IBC_HMVP_ENLARGE
    if (compareNum == -1)
    {
      compareNum = mergeCandIndex;
    }

    for (uint32_t ui = 0; ui < compareNum; ui++)
#else
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
#endif
    {
      if (mvFieldNeighbours[ui << 1].mv == mvFieldNeighbours[mergeCandIndex << 1].mv)
      {
        return true;
      }
    }
  }

  return false;
}
#endif
#if JVET_W0097_GPM_MMVD_TM
void MergeCtx::setGeoMmvdMergeInfo(PredictionUnit& pu, int mergeIdx, int mmvdIdx)
{
  bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  CHECK(mergeIdx >= numValidMergeCand, "Merge candidate does not exist");
  CHECK(mmvdIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM), "GPM MMVD index is invalid");
  CHECK(!pu.cu->geoFlag || CU::isIBC(*pu.cu), "incorrect GPM setting")

    pu.regularMergeFlag = !(pu.ciipFlag || pu.cu->geoFlag);
  pu.mergeFlag = true;
  pu.mmvdMergeFlag = false;
  pu.interDir = interDirNeighbours[mergeIdx];
  pu.cu->imv = 0;
  pu.mergeIdx = mergeIdx;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif

  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  const int refExtMvdCands[9] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  int fPosStep = (extMMVD ? (mmvdIdx >> 3) : (mmvdIdx >> 2));
  int fPosPosition = (extMMVD ? (mmvdIdx - (fPosStep << 3)) : (mmvdIdx - (fPosStep << 2)));
  int offset = (extMMVD ? refExtMvdCands[fPosStep] : refMvdCands[fPosStep]);
  Mv  mvOffset;

  if (fPosPosition == 0)
  {
    mvOffset = Mv(offset, 0);
  }
  else if (fPosPosition == 1)
  {
    mvOffset = Mv(-offset, 0);
  }
  else if (fPosPosition == 2)
  {
    mvOffset = Mv(0, offset);
  }
  else if (fPosPosition == 3)
  {
    mvOffset = Mv(0, -offset);
  }
  else if (fPosPosition == 4)
  {
    mvOffset = Mv(offset, offset);
  }
  else if (fPosPosition == 5)
  {
    mvOffset = Mv(offset, -offset);
  }
  else if (fPosPosition == 6)
  {
    mvOffset = Mv(-offset, offset);
  }
  else if (fPosPosition == 7)
  {
    mvOffset = Mv(-offset, -offset);
  }

  pu.refIdx[REF_PIC_LIST_0] = mvFieldNeighbours[(mergeIdx << 1) + 0].refIdx;
  pu.refIdx[REF_PIC_LIST_1] = mvFieldNeighbours[(mergeIdx << 1) + 1].refIdx;
  if (pu.refIdx[REF_PIC_LIST_0] >= 0)
  {
    pu.mv[REF_PIC_LIST_0] = mvFieldNeighbours[(mergeIdx << 1) + 0].mv + mvOffset;
  }
  else
  {
    pu.mv[REF_PIC_LIST_0] = Mv();
  }

  if (pu.refIdx[REF_PIC_LIST_1] >= 0)
  {
    pu.mv[REF_PIC_LIST_1] = mvFieldNeighbours[(mergeIdx << 1) + 1].mv + mvOffset;
  }
  else
  {
    pu.mv[REF_PIC_LIST_1] = Mv();
  }
  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->bcwIdx = (interDirNeighbours[mergeIdx] == 3) ? bcwIdx[mergeIdx] : BCW_DEFAULT;

#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif

#if !INTER_RM_SIZE_CONSTRAINTS
  PU::restrictBiPredMergeCandsOne(pu);
#endif
  pu.mmvdEncOptMode = 0;

#if INTER_LIC
  pu.cu->licFlag = pu.cs->slice->getUseLIC() ? licFlags[mergeIdx] : false;
  if (pu.interDir == 3)
  {
    CHECK(pu.cu->licFlag, "LIC is not used with bi-prediction in merge");
  }
#endif
}
void MergeCtx::copyMergeCtx(MergeCtx & orgMergeCtx)
{
  memcpy(interDirNeighbours, orgMergeCtx.interDirNeighbours, MRG_MAX_NUM_CANDS * sizeof(unsigned char));
  memcpy(mvFieldNeighbours, orgMergeCtx.mvFieldNeighbours, (MRG_MAX_NUM_CANDS << 1) * sizeof(MvField));
}
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped)
#else
void MergeCtx::setMmvdMergeCandiInfo(PredictionUnit& pu, int candIdx)
#endif
{
  const Slice &slice = *pu.cs->slice;
  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED && !JVET_AA0132_CONFIGURABLE_TM_TOOLS
  const int refMvdCands[] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift ,  32 << mvShift };
#else
  const int refMvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
#endif
  int fPosGroup = 0;
  int fPosBaseIdx = 0;
  int fPosStep = 0;
  int tempIdx = 0;
  int fPosPosition = 0;
  Mv tempMv[2];

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if(candIdxMaped == -1)
  {
    candIdxMaped = candIdx;
  }
  tempIdx = candIdxMaped;
#else
  tempIdx = candIdx;
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (pu.cs->sps->getUseTMMMVD())
  {
#endif
  fPosGroup = tempIdx / (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (MMVD_BASE_MV_NUM * MMVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / MMVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (MMVD_MAX_REFINE_NUM);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  fPosStep = tempIdx / MMVD_MAX_DIR;
  fPosPosition = tempIdx - fPosStep * MMVD_MAX_DIR;
#else
  fPosStep = tempIdx / 4;
  fPosPosition = tempIdx - fPosStep * (4);
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  }
  else
  {
    fPosGroup = tempIdx / (VVC_MMVD_BASE_MV_NUM * VVC_MMVD_MAX_REFINE_NUM);
    tempIdx = tempIdx - fPosGroup * (VVC_MMVD_BASE_MV_NUM * VVC_MMVD_MAX_REFINE_NUM);
    fPosBaseIdx = tempIdx / VVC_MMVD_MAX_REFINE_NUM;
    tempIdx = tempIdx - fPosBaseIdx * (VVC_MMVD_MAX_REFINE_NUM);
    fPosStep = tempIdx / VVC_MMVD_MAX_DIR;
    fPosPosition = tempIdx - fPosStep * VVC_MMVD_MAX_DIR;
  }
#endif
  int offset = refMvdCands[fPosStep];
  if ( pu.cu->slice->getPicHeader()->getDisFracMMVD() )
  {
    offset <<= 2;
  }
  const int refList0 = mmvdBaseMv[fPosBaseIdx][0].refIdx;
  const int refList1 = mmvdBaseMv[fPosBaseIdx][1].refIdx;

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
                    //0   1   2   3   4   5   6   7  8   9  10  11 12  13  14  15
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2, -2,  2, 1, -1, -1,  1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1, -1,  1, -1, 2, -2,  2, -2};
#else
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2,  2, -2, 1,  1, -1, -1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1,  1, -1, -1, 2, -2,  2, -2};
#endif
#endif
   
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  if ((refList0 != -1) && (refList1 != -1)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    && pu.cs->sps->getUseTMMMVD()
#endif
    )
  {
    tempMv[0] = Mv(0,0);
    tempMv[1] = Mv(0,0);
    int cutOff1 = 2 * MMVD_MAX_DIR_UNI;
    int cutOff2 = MMVD_MAX_DIR_UNI;
    if (fPosPosition >= cutOff1)
    {
      fPosPosition -= cutOff1;
      const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
      const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
      const int currPoc = slice.getPOC();
      tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
      if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
      {
        tempMv[1] = tempMv[0];
      }
      else
      {
        tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
      }
    }
    else if (fPosPosition >= cutOff2)
    {
      fPosPosition -= cutOff2;
      tempMv[1] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
    }
    else
    {
      tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
    }
    pu.interDir = 3;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  else
#endif
#endif
#if !JVET_AA0093_ENHANCED_MMVD_EXTENSION || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_ENHANCED_MMVD_EXTENSION)
  if ((refList0 != -1) && (refList1 != -1))
  {
    const int poc0 = slice.getRefPOC(REF_PIC_LIST_0, refList0);
    const int poc1 = slice.getRefPOC(REF_PIC_LIST_1, refList1);
    const int currPoc = slice.getPOC();
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#else
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
#endif
    if ((poc0 - currPoc) == (poc1 - currPoc))
    {
      tempMv[1] = tempMv[0];
    }
    else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
      tempMv[1] = tempMv[0];
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[0] = tempMv[1];
        }
        else
        {
          tempMv[0].set(-1 * tempMv[1].getHor(), -1 * tempMv[1].getVer());
        }
      }
      else
      tempMv[0] = tempMv[1].scaleMv(scale);
    }
    else
    {
      const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
      const bool isL0RefLongTerm = slice.getRefPic(REF_PIC_LIST_0, refList0)->longTerm;
      const bool isL1RefLongTerm = slice.getRefPic(REF_PIC_LIST_1, refList1)->longTerm;
      if (isL0RefLongTerm || isL1RefLongTerm)
      {
        if ((poc1 - currPoc)*(poc0 - currPoc) > 0)
        {
          tempMv[1] = tempMv[0];
        }
        else
        {
          tempMv[1].set(-1 * tempMv[0].getHor(), -1 * tempMv[0].getVer());
        }
      }
      else
      tempMv[1] = tempMv[0].scaleMv(scale);
    }

    pu.interDir = 3;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }
#endif
  else if (refList0 != -1)
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    tempMv[0] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#else
    if (fPosPosition == 0)
    {
      tempMv[0] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[0] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[0] = Mv(0, offset);
    }
    else
    {
      tempMv[0] = Mv(0, -offset);
    }
#endif
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0] = mmvdBaseMv[fPosBaseIdx][0].mv + tempMv[0];
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
  }
  else if (refList1 != -1)
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    tempMv[1] = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#else
    if (fPosPosition == 0)
    {
      tempMv[1] = Mv(offset, 0);
    }
    else if (fPosPosition == 1)
    {
      tempMv[1] = Mv(-offset, 0);
    }
    else if (fPosPosition == 2)
    {
      tempMv[1] = Mv(0, offset);
    }
    else
    {
      tempMv[1] = Mv(0, -offset);
    }
#endif
    pu.interDir = 2;
    pu.mv[REF_PIC_LIST_0] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_0] = -1;
    pu.mv[REF_PIC_LIST_1] = mmvdBaseMv[fPosBaseIdx][1].mv + tempMv[1];
    pu.refIdx[REF_PIC_LIST_1] = refList1;
  }

  pu.mmvdMergeFlag = true;
  pu.mmvdMergeIdx = candIdx;
  pu.mergeFlag = true;
  pu.regularMergeFlag = true;
  pu.mergeIdx = candIdx;
  pu.mergeType = MRG_TYPE_DEFAULT_N;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine             = false;
#endif
  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
  pu.cu->imv = mmvdUseAltHpelIf[fPosBaseIdx] ? IMV_HPEL : 0;
#if INTER_LIC
  pu.cu->licFlag = licFlags[fPosBaseIdx];
#endif
  pu.cu->bcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? bcwIdx[fPosBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }

#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif

  PU::restrictBiPredMergeCandsOne(pu);
}
#if JVET_AB0112_AFFINE_DMVR
bool AffineMergeCtx::xCheckSimilarMotion(int mergeCandIndex, uint32_t mvdSimilarityThresh) const
{
  if (mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx < 0 && mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx < 0)
  {
    return true;
  }

  if (mvdSimilarityThresh > 1)
  {
    int mvdTh = mvdSimilarityThresh;
    for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
    {
      if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
      {
        if (interDirNeighbours[ui] == 3)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
              mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff0L0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiff0L1 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;

            Mv mvDiff1L0 = mvFieldNeighbours[(ui << 1)][1].mv - mvFieldNeighbours[(mergeCandIndex << 1)][1].mv;
            Mv mvDiff1L1 = mvFieldNeighbours[(ui << 1) + 1][1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv;

            Mv mvDiff2L0 = mvFieldNeighbours[(ui << 1)][2].mv - mvFieldNeighbours[(mergeCandIndex << 1)][2].mv;
            Mv mvDiff2L1 = mvFieldNeighbours[(ui << 1) + 1][2].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv;
            if (mvDiff0L0.getAbsHor() < mvdTh && mvDiff0L0.getAbsVer() < mvdTh
              && mvDiff0L1.getAbsHor() < mvdTh && mvDiff0L1.getAbsVer() < mvdTh
              &&mvDiff1L0.getAbsHor() < mvdTh && mvDiff1L0.getAbsVer() < mvdTh
              && mvDiff1L1.getAbsHor() < mvdTh && mvDiff1L1.getAbsVer() < mvdTh
              &&mvDiff2L0.getAbsHor() < mvdTh && mvDiff2L0.getAbsVer() < mvdTh
              && mvDiff2L1.getAbsHor() < mvdTh && mvDiff2L1.getAbsVer() < mvdTh
              )
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 1)
        {
          if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx)
          {
            Mv mvDiff0 = mvFieldNeighbours[(ui << 1)][0].mv - mvFieldNeighbours[(mergeCandIndex << 1)][0].mv;
            Mv mvDiff1 = mvFieldNeighbours[(ui << 1)][1].mv - mvFieldNeighbours[(mergeCandIndex << 1)][1].mv;
            Mv mvDiff2 = mvFieldNeighbours[(ui << 1)][2].mv - mvFieldNeighbours[(mergeCandIndex << 1)][2].mv;
            if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh
              &&mvDiff1.getAbsHor() < mvdTh && mvDiff1.getAbsVer() < mvdTh
              &&mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh
              )
            {
              return true;
            }
          }
        }
        else if (interDirNeighbours[ui] == 2)
        {
          if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx)
          {
            Mv mvDiff0 = mvFieldNeighbours[(ui << 1) + 1][0].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv;
            Mv mvDiff1 = mvFieldNeighbours[(ui << 1) + 1][1].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv;
            Mv mvDiff2 = mvFieldNeighbours[(ui << 1) + 1][2].mv - mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv;
            if (mvDiff0.getAbsHor() < mvdTh && mvDiff0.getAbsVer() < mvdTh
              &&mvDiff1.getAbsHor() < mvdTh && mvDiff1.getAbsVer() < mvdTh
              && mvDiff2.getAbsHor() < mvdTh && mvDiff2.getAbsVer() < mvdTh
              )
            {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  for (uint32_t ui = 0; ui < mergeCandIndex; ui++)
  {
    if (interDirNeighbours[ui] == interDirNeighbours[mergeCandIndex])
    {
      if (interDirNeighbours[ui] == 3)
      {
        if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx &&
          mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv     &&
          mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv&&
          mvFieldNeighbours[(ui << 1)][1].mv == mvFieldNeighbours[(mergeCandIndex << 1)][1].mv     &&
          mvFieldNeighbours[(ui << 1) + 1][1].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv&&
          mvFieldNeighbours[(ui << 1)][2].mv == mvFieldNeighbours[(mergeCandIndex << 1)][2].mv     &&
          mvFieldNeighbours[(ui << 1) + 1][2].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv
          )
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 1)
      {
        if (mvFieldNeighbours[(ui << 1)][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1)][0].refIdx &&
          mvFieldNeighbours[(ui << 1)][0].mv == mvFieldNeighbours[(mergeCandIndex << 1)][0].mv&&
          mvFieldNeighbours[(ui << 1)][1].mv == mvFieldNeighbours[(mergeCandIndex << 1)][1].mv&&
          mvFieldNeighbours[(ui << 1)][2].mv == mvFieldNeighbours[(mergeCandIndex << 1)][2].mv
          )
        {
          return true;
        }
      }
      else if (interDirNeighbours[ui] == 2)
      {
        if (mvFieldNeighbours[(ui << 1) + 1][0].refIdx == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].refIdx &&
          mvFieldNeighbours[(ui << 1) + 1][0].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][0].mv &&
          mvFieldNeighbours[(ui << 1) + 1][1].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][1].mv &&
          mvFieldNeighbours[(ui << 1) + 1][2].mv == mvFieldNeighbours[(mergeCandIndex << 1) + 1][2].mv
          )
        {
          return true;
        }
      }
    }
  }
  return false;
}
#endif
#if JVET_AA0061_IBC_MBVD
bool MergeCtx::setIbcMbvdMergeCandiInfo(PredictionUnit& pu, int candIdx, int candIdxMaped)
{
  const int mvShift = MV_FRACTIONAL_BITS_DIFF + 2;
  const int refMvdCands[IBC_MBVD_STEP_NUM] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift , 24 << mvShift , 32 << mvShift , 40 << mvShift , 48 << mvShift , 56 << mvShift ,
    64 << mvShift , 72 << mvShift , 80 << mvShift , 88 << mvShift , 96 << mvShift , 104 << mvShift , 112 << mvShift , 120 << mvShift , 128 << mvShift };
  int fPosGroup = 0;
  int fPosBaseIdx = 0;
  int fPosStep = 0;
  int tempIdx = 0;
  int fPosPosition = 0;
  Mv tempMv;

  if(candIdxMaped == -1)
  {
    candIdxMaped = candIdx;
  }
  tempIdx = candIdxMaped;

  fPosGroup = tempIdx / (IBC_MBVD_BASE_NUM * IBC_MBVD_MAX_REFINE_NUM);
  tempIdx = tempIdx - fPosGroup * (IBC_MBVD_BASE_NUM * IBC_MBVD_MAX_REFINE_NUM);
  fPosBaseIdx = tempIdx / IBC_MBVD_MAX_REFINE_NUM;
  tempIdx = tempIdx - fPosBaseIdx * (IBC_MBVD_MAX_REFINE_NUM);
  fPosStep = tempIdx / IBC_MBVD_OFFSET_DIR;
  fPosPosition = tempIdx - fPosStep * (IBC_MBVD_OFFSET_DIR);
  int offset = refMvdCands[fPosStep];

  const int refList0 = ibcMbvdBaseBv[fPosBaseIdx][0].refIdx;
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2,  2, -2, 1,  1, -1, -1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1,  1, -1, -1, 2, -2,  2, -2};

  if (refList0 != -1)
  {
    tempMv = Mv(xDir[fPosPosition] * offset, yDir[fPosPosition] * offset);
#if JVET_AA0070_RRIBC
    //check the BVD direction and base BV flip direction
    if ((rribcFlipTypes[fPosBaseIdx] == 1) && (yDir[fPosPosition] != 0))
    {
      return true;
    }
    if ((rribcFlipTypes[fPosBaseIdx] == 2) && (xDir[fPosPosition] != 0))
    {
      return true;
    }
#endif
    pu.interDir = 1;
    pu.mv[REF_PIC_LIST_0] = ibcMbvdBaseBv[fPosBaseIdx][0].mv + tempMv;
    pu.refIdx[REF_PIC_LIST_0] = refList0;
    pu.mv[REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
  }

  pu.ibcMbvdMergeFlag = true;
  pu.ibcMbvdMergeIdx = candIdx;
  pu.mergeFlag = true;
  pu.mergeType = MRG_TYPE_IBC;

  pu.mvd[REF_PIC_LIST_0] = Mv();
  pu.mvd[REF_PIC_LIST_1] = Mv();
  pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
  pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;

  pu.cu->bcwIdx = (interDirNeighbours[fPosBaseIdx] == 3) ? bcwIdx[fPosBaseIdx] : BCW_DEFAULT;

  for (int refList = 0; refList < 2; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      pu.mv[refList].clipToStorageBitDepth();
    }
  }
  pu.bv = pu.mv[REF_PIC_LIST_0];
  pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // used for only integer resolution
  pu.cu->imv = pu.cu->imv == IMV_HPEL ? 0 : pu.cu->imv;
#if JVET_AC0112_IBC_LIC
  pu.cu->ibcLicFlag = ibcLicFlags[fPosBaseIdx];
#endif
#if JVET_AA0070_RRIBC
  pu.cu->rribcFlipType = rribcFlipTypes[fPosBaseIdx];
#endif
#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif
  return false;
}
#endif

#if JVET_V0130_INTRA_TMP
unsigned DeriveCtx::CtxTmpFlag(const CodingUnit& cu)
{
	const CodingStructure* cs = cu.cs;
	unsigned ctxId = 0;

	const CodingUnit* cuLeft = cs->getCURestricted(cu.lumaPos().offset(-1, 0), cu, CH_L);
	ctxId = (cuLeft && cuLeft->tmpFlag) ? 1 : 0;

	const CodingUnit* cuAbove = cs->getCURestricted(cu.lumaPos().offset(0, -1), cu, CH_L);
	ctxId += (cuAbove && cuAbove->tmpFlag) ? 1 : 0;

	ctxId = (cu.lwidth() > 2 * cu.lheight() || cu.lheight() > 2 * cu.lwidth()) ? 3 : ctxId;

	return ctxId;
}
#endif

unsigned DeriveCtx::CtxMipFlag( const CodingUnit& cu )
{
  const CodingStructure *cs = cu.cs;
  unsigned ctxId = 0;

  const CodingUnit *cuLeft = cs->getCURestricted( cu.lumaPos().offset( -1, 0 ), cu, CH_L );
  ctxId = (cuLeft && cuLeft->mipFlag) ? 1 : 0;

  const CodingUnit *cuAbove = cs->getCURestricted( cu.lumaPos().offset( 0, -1 ), cu, CH_L );
  ctxId += (cuAbove && cuAbove->mipFlag) ? 1 : 0;

  ctxId  = (cu.lwidth() > 2*cu.lheight() || cu.lheight() > 2*cu.lwidth()) ? 3 : ctxId;

  return ctxId;
}

unsigned DeriveCtx::CtxPltCopyFlag( const unsigned prevRunType, const unsigned dist )
{
  uint8_t *ucCtxLut = (prevRunType == PLT_RUN_INDEX) ? g_paletteRunLeftLut : g_paletteRunTopLut;
  if ( dist <= RUN_IDX_THRE )
  {
     return ucCtxLut[dist];
  }
  else
  {
    return ucCtxLut[RUN_IDX_THRE];
  }
}
