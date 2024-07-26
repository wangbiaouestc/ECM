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

/** \file     CABACWriter.cpp
 *  \brief    Writer for low level syntax
 */

#include "CommonLib/Contexts.h"
#include "CABACWriter.h"

#include "EncLib.h"

#include "CommonLib/UnitTools.h"
#include "CommonLib/dtrace_buffer.h"

#include <map>
#include <algorithm>
#include <limits>


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
Partitioner *g_encPartitionerSST=nullptr;
#else
Partitioner* g_encPartitionerSST;
#endif

//! \ingroup EncoderLib
//! \{

#if JVET_AG0196_CABAC_RETRAIN
void CABACWriter::initCtxModels( Slice& slice )
#else
void CABACWriter::initCtxModels( const Slice& slice )
#endif
{
  int       qp                = slice.getSliceQp();
  SliceType sliceType         = slice.getSliceType();
  SliceType encCABACTableIdx  = slice.getEncCABACTableIdx();
  if( !slice.isIntra() && (encCABACTableIdx==B_SLICE || encCABACTableIdx==P_SLICE) && slice.getPPS()->getCabacInitPresentFlag() )
  {
    sliceType = encCABACTableIdx;
  }

#if JVET_AH0176_LOW_DELAY_B_CTX
  if (sliceType == B_SLICE && slice.getCheckLDC())
  {
    sliceType = L_SLICE;
  }
#endif

#if JVET_AG0196_CABAC_RETRAIN
    slice.setCabacInitSliceType( sliceType );
#endif

  m_BinEncoder.reset(qp, (int)sliceType);
#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  if( slice.getSPS()->getTempCabacInitMode() )
  {
    m_CABACDataStore->loadCtxStates( &slice, getCtx() );
  }
#endif
}



template <class BinProbModel>
SliceType xGetCtxInitId( const Slice& slice, const BinEncIf& binEncoder, Ctx& ctxTest )
{
  const CtxStore<BinProbModel>& ctxStoreTest = static_cast<const CtxStore<BinProbModel>&>( ctxTest );
  const CtxStore<BinProbModel>& ctxStoreRef  = static_cast<const CtxStore<BinProbModel>&>( binEncoder.getCtx() );
  int qp = slice.getSliceQp();
  if( !slice.isIntra() )
  {
    SliceType aSliceTypeChoices[] = { B_SLICE, P_SLICE };
    uint64_t  bestCost            = std::numeric_limits<uint64_t>::max();
    SliceType bestSliceType       = aSliceTypeChoices[0];
    for (uint32_t idx=0; idx<2; idx++)
    {
      uint64_t  curCost           = 0;
      SliceType curSliceType      = aSliceTypeChoices[idx];
#if JVET_AH0176_LOW_DELAY_B_CTX
      if (curSliceType == B_SLICE && slice.getCheckLDC())
      {
        ctxTest.init(qp, (int)L_SLICE);
      }
      else
      {
        ctxTest.init(qp, (int)curSliceType);
      }
#else
      ctxTest.init(qp, (int)curSliceType);
#endif
      for( int k = 0; k < Ctx::NumberOfContexts; k++ )
      {
        if( binEncoder.getNumBins(k) > 0 )
        {
          curCost += uint64_t( binEncoder.getNumBins(k) ) * ctxStoreRef[k].estFracExcessBits( ctxStoreTest[k] );
        }
      }
      if (curCost < bestCost)
      {
        bestSliceType = curSliceType;
        bestCost      = curCost;
      }
    }
    return bestSliceType;
  }
  else
  {
    return I_SLICE;
  }
}


SliceType CABACWriter::getCtxInitId( const Slice& slice )
{
  switch( m_TestCtx.getBPMType() )
  {
  case BPM_Std:   return  xGetCtxInitId<BinProbModel_Std>   ( slice, m_BinEncoder, m_TestCtx );
  default:        return  NUMBER_OF_SLICE_TYPES;
  }
}



unsigned estBits( BinEncIf& binEnc, const std::vector<bool>& bins, const Ctx& ctx, const int ctxId, const uint8_t winSize )
{
  binEnc.initCtxAndWinSize( ctxId, ctx, winSize );
  binEnc.start();
  const std::size_t numBins   = bins.size();
  unsigned          startBits = binEnc.getNumWrittenBits();
  for( std::size_t binId = 0; binId < numBins; binId++ )
  {
    unsigned  bin = ( bins[binId] ? 1 : 0 );
    binEnc.encodeBin( bin, ctxId );
  }
  unsigned endBits    = binEnc.getNumWrittenBits();
  unsigned codedBits  = endBits - startBits;
  return   codedBits;
}





//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    void  end_of_slice()
//================================================================================

void CABACWriter::end_of_slice()
{
  m_BinEncoder.encodeBinTrm ( 1 );
  m_BinEncoder.finish       ();
}

#if JVET_V0094_BILATERAL_FILTER
void CABACWriter::bif( const ComponentID compID, const Slice& slice, const BifParams& bifParams )
{
  for (int i = 0; i < bifParams.numBlocks; ++i)
  {
    bif(compID, slice, bifParams, i);
  }
}

void CABACWriter::bif( const ComponentID compID, const Slice& slice, const BifParams& bifParams, unsigned ctuRsAddr)
{
  const PPS& pps = *slice.getPPS();

  if( isLuma( compID ) && !pps.getUseBIF() )
  {
    return;
  }

#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if( isChroma( compID ) && !pps.getUseChromaBIF() )
  {
    return;
  }
#endif

  if( ctuRsAddr == 0 )
  {
    m_BinEncoder.encodeBinEP( bifParams.allCtuOn );
    if( bifParams.allCtuOn == 0 )
    {
      m_BinEncoder.encodeBinEP( bifParams.frmOn );
    }
  }

  if( bifParams.allCtuOn == 0 && bifParams.frmOn )
  {
    m_BinEncoder.encodeBin( bifParams.ctuOn[ctuRsAddr], Ctx::BifCtrlFlags[compID]() );
  }
}
#endif

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    bool  coding_tree_unit( cs, area, qp, ctuRsAddr, skipSao, skipAlf )
//================================================================================

void CABACWriter::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr, bool skipSao /* = false */, bool skipAlf /* = false */ )
{
  DTRACE(g_trace_ctx, D_SYNTAX, "coding_tree_unit() pos=(%d,%d)\n", area.lx(), area.ly());
  CUCtx cuCtx( qps[CH_L] );
  QTBTPartitioner partitioner;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (g_encPartitionerSST == nullptr)
  {
    g_encPartitionerSST = new QTBTPartitioner;
  }
#endif

  partitioner.initCtu(area, CH_L, *cs.slice);

  if( !skipSao )
  {
    sao( *cs.slice, ctuRsAddr );
  }

#if JVET_W0066_CCSAO
  if ( !skipSao )
  {
    for ( int compIdx = 0; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->m_ccSaoComParam.enabled[compIdx])
      {
        const int setNum = cs.slice->m_ccSaoComParam.setNum[compIdx];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        codeCcSaoControlIdc(cs.slice->m_ccSaoControl[compIdx][ctuRsAddr], cs, ComponentID(compIdx),
                            ctuRsAddr, cs.slice->m_ccSaoControl[compIdx], lumaPos, setNum);
      }
    }
  }
#endif

  if (!skipAlf)
  {
    for (int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++)
    {
      if (!cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx))
      {
        continue;
      }
      codeAlfCtuEnableFlag(cs, ctuRsAddr, compIdx, NULL);
      if (isLuma(ComponentID(compIdx)))
      {
        codeAlfCtuFilterIndex(cs, ctuRsAddr, cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y));
#if ALF_IMPROVEMENT
        if (cs.slice->getPic()->getAlfCtuEnableFlag(compIdx)[ctuRsAddr] && cs.slice->getPic()->getAlfCtbFilterIndex()[ctuRsAddr] >= NUM_FIXED_FILTER_SETS)
        {
          int apsIdx = cs.slice->getTileGroupApsIdLuma()[cs.slice->getPic()->getAlfCtbFilterIndex()[ctuRsAddr] - NUM_FIXED_FILTER_SETS];
          codeAlfCtuAlternative(cs, ctuRsAddr, 0, NULL, cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam().numAlternativesLuma);
        }
#endif
      }
      if (isChroma(ComponentID(compIdx)))
      {
        uint8_t* ctbAlfFlag = cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx) ? cs.slice->getPic()->getAlfCtuEnableFlag( compIdx ) : nullptr;
        if( ctbAlfFlag && ctbAlfFlag[ctuRsAddr] )
        {
          codeAlfCtuAlternative( cs, ctuRsAddr, compIdx );
        }
      }
    }
  }

  if ( !skipAlf )
  {
    for ( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount   = cs.slice->m_ccAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        codeCcAlfFilterControlIdc(cs.slice->m_ccAlfFilterControl[compIdx - 1][ctuRsAddr], cs, ComponentID(compIdx),
                                  ctuRsAddr, cs.slice->m_ccAlfFilterControl[compIdx - 1], lumaPos, filterCount);
      }
    }
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  coding_tree( cs, partitioner, cuCtx, qps );
  qps[CH_L] = cuCtx.qp;
#else
#if TU_256
  if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE ) )
#else
  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
#endif
  {
    CUCtx chromaCuCtx(qps[CH_C]);
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, CH_C, *cs.slice);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &chromaCuCtx);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = chromaCuCtx.qp;
  }
  else
  {
    coding_tree(cs, partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner.initCtu(area, CH_C, *cs.slice);
      coding_tree(cs, partitioner, cuCtxChroma);
      qps[CH_C] = cuCtxChroma.qp;
    }
  }
#endif
}





//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao             ( slice, ctuRsAddr )
//    void  sao_block_pars  ( saoPars, bitDepths, sliceEnabled, leftMergeAvail, aboveMergeAvail, onlyEstMergeInfo )
//    void  sao_offset_pars ( ctbPars, compID, sliceEnabled, bitDepth )
//================================================================================

void CABACWriter::sao( const Slice& slice, unsigned ctuRsAddr )
{
  const SPS& sps = *slice.getSPS();
  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  CodingStructure&     cs                     = *slice.getPic()->cs;
  const PreCalcValues& pcv                    = *cs.pcv;
  const SAOBlkParam&  sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
  bool                slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool                slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  bool                sliceEnabled[3]         = { slice_sao_luma_flag, slice_sao_chroma_flag, slice_sao_chroma_flag };
  int                 frame_width_in_ctus     = pcv.widthInCtus;
  int                 ry                      = ctuRsAddr      / frame_width_in_ctus;
  int                 rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  const Position      pos                     ( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned      curSliceIdx             = slice.getIndependentSliceIdx();
  const unsigned      curTileIdx              = cs.pps->getTileIdx( pos );
  bool                leftMergeAvail          = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0  ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool                aboveMergeAvail         = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  sao_block_pars( sao_ctu_pars, sps.getBitDepths(), sliceEnabled, leftMergeAvail, aboveMergeAvail, false );
}


void CABACWriter::sao_block_pars( const SAOBlkParam& saoPars, const BitDepths& bitDepths, bool* sliceEnabled, bool leftMergeAvail, bool aboveMergeAvail, bool onlyEstMergeInfo )
{
  bool isLeftMerge  = false;
  bool isAboveMerge = false;
  if( leftMergeAvail )
  {
    // sao_merge_left_flag
    isLeftMerge   = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_LEFT );
    m_BinEncoder.encodeBin( (isLeftMerge), Ctx::SaoMergeFlag() );
  }
  if( aboveMergeAvail && !isLeftMerge )
  {
    // sao_merge_above_flag
    isAboveMerge  = ( saoPars[COMPONENT_Y].modeIdc == SAO_MODE_MERGE && saoPars[COMPONENT_Y].typeIdc == SAO_MERGE_ABOVE );
    m_BinEncoder.encodeBin( (isAboveMerge), Ctx::SaoMergeFlag() );
  }
  if( onlyEstMergeInfo )
  {
    return; //only for RDO
  }
  if( !isLeftMerge && !isAboveMerge )
  {
    // explicit parameters
    for( int compIdx=0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      sao_offset_pars( saoPars[compIdx], ComponentID(compIdx), sliceEnabled[compIdx], bitDepths.recon[ toChannelType(ComponentID(compIdx)) ] );
    }
  }
}


void CABACWriter::sao_offset_pars( const SAOOffset& ctbPars, ComponentID compID, bool sliceEnabled, int bitDepth )
{
  if( !sliceEnabled )
  {
    CHECK( ctbPars.modeIdc != SAO_MODE_OFF, "Sao must be off, if it is disabled on slice level" );
    return;
  }
  const bool isFirstCompOfChType = ( getFirstComponentOfChannel( toChannelType(compID) ) == compID );

  if( isFirstCompOfChType )
  {
    // sao_type_idx_luma / sao_type_idx_chroma
    if( ctbPars.modeIdc == SAO_MODE_OFF )
    {
      m_BinEncoder.encodeBin  ( 0, Ctx::SaoTypeIdx() );
    }
    else if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 0 );
    }
    else
    {
      CHECK(!( ctbPars.typeIdc < SAO_TYPE_START_BO ), "Unspecified error");
      m_BinEncoder.encodeBin  ( 1, Ctx::SaoTypeIdx() );
      m_BinEncoder.encodeBinEP( 1 );
    }
  }

  if( ctbPars.modeIdc == SAO_MODE_NEW )
  {
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( bitDepth );
    int       numClasses    = ( ctbPars.typeIdc == SAO_TYPE_BO ? 4 : NUM_SAO_EO_CLASSES );
    int       k             = 0;
    int       offset[4];
    for( int i = 0; i < numClasses; i++ )
    {
      if( ctbPars.typeIdc != SAO_TYPE_BO && i == SAO_CLASS_EO_PLAIN )
      {
        continue;
      }
      int classIdx = ( ctbPars.typeIdc == SAO_TYPE_BO ? ( ctbPars.typeAuxInfo + i ) % NUM_SAO_BO_CLASSES : i );
      offset[k++]  = ctbPars.offset[classIdx];
    }

    // sao_offset_abs
    for( int i = 0; i < 4; i++ )
    {
      unsigned absOffset = ( offset[i] < 0 ? -offset[i] : offset[i] );
      unary_max_eqprob( absOffset, maxOffsetQVal );
    }

    // band offset mode
    if( ctbPars.typeIdc == SAO_TYPE_BO )
    {
      // sao_offset_sign
      for( int i = 0; i < 4; i++ )
      {
        if( offset[i] )
        {
          m_BinEncoder.encodeBinEP( (offset[i] < 0) );
        }
      }
      // sao_band_position
      m_BinEncoder.encodeBinsEP( ctbPars.typeAuxInfo, NUM_SAO_BO_CLASSES_LOG2 );
    }
    // edge offset mode
    else
    {
      if( isFirstCompOfChType )
      {
        // sao_eo_class_luma / sao_eo_class_chroma
        CHECK( ctbPars.typeIdc - SAO_TYPE_START_EO < 0, "sao edge offset class is outside valid range" );
        m_BinEncoder.encodeBinsEP( ctbPars.typeIdc - SAO_TYPE_START_EO, NUM_SAO_EO_TYPES_LOG2 );
      }
    }
  }
}

#if JVET_W0066_CCSAO
void CABACWriter::codeCcSaoControlIdc(uint8_t idcVal, CodingStructure &cs, const ComponentID compID,
                                      const int curIdx, const uint8_t *controlIdc, Position lumaPos,
                                      const int setNum)
{
  CHECK(idcVal > setNum, "Set index is too large");

  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( controlIdc[curIdx - 1]) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += (controlIdc[curIdx - cs.pcv->widthInCtus]) ? 1 : 0;
  }
  ctxt += ( compID == COMPONENT_Y  ) ? 0 
        : ( compID == COMPONENT_Cb ) ? 3 : 6;

  m_BinEncoder.encodeBin( ( idcVal == 0 ) ? 0 : 1, Ctx::CcSaoControlIdc( ctxt ) ); // ON/OFF flag is context coded
  if ( idcVal > 0 )
  {
    int val = (idcVal - 1);
    while ( val )
    {
      m_BinEncoder.encodeBinEP( 1 );
      val--;
    }
    if ( idcVal < setNum )
    {
      m_BinEncoder.encodeBinEP( 0 );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cc_sao_control_idc() compID=%d pos=(%d,%d) ctxt=%d, setNum=%d, idcVal=%d\n", compID, lumaPos.x, lumaPos.y, ctxt, setNum, idcVal );
}
#endif

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    void  split_cu_flag     ( split, cs, partitioner )
//    void  split_cu_mode_mt  ( split, cs, partitioner )
//================================================================================

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
//void CABACWriter::coding_tree( const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, int (&qps)[2] )
void CABACWriter::coding_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, int (&qps)[2], Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
#else
void CABACWriter::coding_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
#endif
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();
  const CodingUnit &cu        = *cs.getCU( currArea.blocks[partitioner.chType], partitioner.chType );

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (pps.getUseDQP() && partitioner.currQgEnable())
#else
  //Note: do not reset qg at chroma CU
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma( partitioner.chType ) )
#endif
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded          = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
  }
  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
    }
  }
#endif

  const PartSplit splitMode = CU::getSplitAtDepth( cu, partitioner.currDepth );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  split_cu_mode( splitMode, cs, partitioner, &cu);
#else
  split_cu_mode( splitMode, cs, partitioner);
#endif

  CHECK( !partitioner.canSplit( splitMode, cs ), "The chosen split mode is invalid!" );

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if( splitMode != CU_DONT_SPLIT && ( cs.slice->getProcessingIntraRegion() || !CU::isIntraRegionRoot( cu, partitioner ) ) )
#else
  if( splitMode != CU_DONT_SPLIT )
#endif
  {
#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
#if TU_256
    const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );

    if( CS::isDualITree( cs ) && pPartitionerChroma != nullptr && ( partitioner.currArea().lwidth() >= maxSize || partitioner.currArea().lheight() >= maxSize ) )
#else
      if (CS::isDualITree(cs) && pPartitionerChroma != nullptr && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
#endif
      {
        partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
        pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
        bool beContinue = true;
        bool lumaContinue = true;
        bool chromaContinue = true;

        while (beContinue)
        {
#if TU_256
          if( partitioner.currArea().lwidth() > maxSize || partitioner.currArea().lheight() > maxSize )
#else
          if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
#endif
          {
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
            }
            lumaContinue = partitioner.nextPart(cs);
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
          else
          {
            //dual tree coding under 64x64 block
            if (cs.picture->blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
            {
              coding_tree(cs, partitioner, cuCtx);
            }
            lumaContinue = partitioner.nextPart(cs);
            if (cs.picture->blocks[pPartitionerChroma->chType].contains(pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
            {
              coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
            }
            chromaContinue = pPartitionerChroma->nextPart(cs);
            CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
            beContinue = lumaContinue;
          }
        }
        partitioner.exitCurrSplit();
        pPartitionerChroma->exitCurrSplit();

      }
      else
      {
#endif //!JVET_AI0136_ADAPTIVE_DUAL_TREE
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        const ModeType modeTypeParent = partitioner.modeType;
        const ModeType modeTypeChild = CU::getModeTypeAtDepth( cu, partitioner.currDepth );
        mode_constraint( splitMode, cs, partitioner, modeTypeChild );
        partitioner.modeType = modeTypeChild;

        bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
        CHECK( chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );
        if( partitioner.treeType == TREE_D )
        {
          partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
        }
#endif
      partitioner.splitCurrArea( splitMode, cs );

      do
      {
        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
          coding_tree( cs, partitioner, cuCtx, qps );
#else
          coding_tree( cs, partitioner, cuCtx );
#endif
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( chromaNotSplit )
      {
        if (isChromaEnabled(cs.pcv->chrFormat))
        {
        CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "must be luma status" );
        partitioner.chType = CHANNEL_TYPE_CHROMA;
        partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
        }

        //recover
        partitioner.chType = CHANNEL_TYPE_LUMA;
        partitioner.treeType = TREE_D;
      }
      partitioner.modeType = modeTypeParent;
#endif
#if !JVET_AI0136_ADAPTIVE_DUAL_TREE
      }
#endif
      return;
  }

  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK( cu.treeType != partitioner.treeType, "treeType mismatch" );
#endif

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // If context data collection is active and if on bottom of the CTU, start the counters
  if ( m_BinEncoder.getBinBuffer() )
  {
    m_BinEncoder.setBinBufferActive( CU::isOnCtuBottom( cu ) );
  }
#endif

  // coding unit
#if  !JVET_AI0136_ADAPTIVE_DUAL_TREE
    DTRACE(g_trace_ctx, D_SYNTAX, "coding_unit() pos=(%d,%d) size=%dx%d chType=%d depth=%d\n", cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, cu.blocks[cu.chType].width, cu.blocks[cu.chType].height, cu.chType, cu.depth);
#endif
  coding_unit( cu, partitioner, cuCtx );

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // Done with the data collection for this CU
  if ( m_BinEncoder.getBinBuffer() )
  {
    m_BinEncoder.setBinBufferActive( false );
  }
#endif


#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( cs.slice->getSeparateTreeEnabled() && (CU::isIntra( cu ) || cu.slice->isIntra()) && !cs.slice->getProcessingIntraRegion() && cu.isSST && cu.separateTree)
  {
    Partitioner *partitionerSST = g_encPartitionerSST;

    partitionerSST->copyState( partitioner );
    partitionerSST->chType = CH_L;

    cs.slice->setProcessingIntraRegion   ( true );
    cs.slice->setProcessingSeparateTrees ( cu.separateTree );
    cs.slice->setIntraRegionRoot         ( &partitioner );
    cs.slice->setProcessingChannelType   ( CH_L );

    coding_tree( cs, *partitionerSST, cuCtx, qps );

    qps[CH_L] = cuCtx.qp;
    if ( cu.separateTree )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      cs.slice->setProcessingChannelType ( CH_C );
      partitionerSST->copyState          ( partitioner );
      partitionerSST->chType = CH_C;

      coding_tree( cs, *partitionerSST, cuCtx, qps );
      qps[CH_C] = cuCtxChroma.qp;
    }
    cs.slice->setProcessingIntraRegion( false );
  }
#endif


#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.chType == CHANNEL_TYPE_CHROMA )
  {
    DTRACE_COND( (isEncoding()), g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
#endif
  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height, cu.qp );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  }
#endif
  DTRACE_BLOCK_REC_COND( ( !isEncoding() ), cs.picture->getRecoBuf( cu ), cu, cu.predMode );
  if (CU::isInter(cu))
  {
    DTRACE_MOT_FIELD(g_trace_ctx, *cu.firstPU);
  }
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
void CABACWriter::mode_constraint( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner, const ModeType modeType )
{
  CHECK( split == CU_DONT_SPLIT, "splitMode shall not be no split" );
  int val = cs.signalModeCons( split, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    CHECK( modeType == MODE_TYPE_ALL, "shall not be no constraint case" );
    bool flag = modeType == MODE_TYPE_INTRA;
    int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    m_BinEncoder.encodeBin( flag, Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    assert( modeType == MODE_TYPE_INTRA );
  }
  else
  {
    assert( modeType == partitioner.modeType );
  }
}
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACWriter::split_cu_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner, const CodingUnit* cu )
#else
void CABACWriter::split_cu_mode( const PartSplit split, const CodingStructure& cs, Partitioner& partitioner )
#endif
{
  bool canNo, canQt, canBh, canBv, canTh, canTv;
#if JVET_AH0135_TEMPORAL_PARTITIONING
  unsigned maxMtt;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv, maxMtt );
#else
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );
#endif

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  const bool canSplit = canBh || canBv || canTh || canTv || canQt;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  bool isNo     = split == CU_DONT_SPLIT;
#else
  const bool isNo     = split == CU_DONT_SPLIT;
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( cu != nullptr && cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && (CU::isIntra( *cu ) || cu->cs->slice->isIntra()) && CU::isIntraRegionRoot(*cu, partitioner))
  {
    //split = false;
    isNo=true;

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if ( cs.slice->isIntra() && cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && ( partitioner.currArea().lwidth() <= 256 && partitioner.currArea().lheight() <= 256 ) )
    {
#if ENABLE_TRACING
      const CompArea& block = partitioner.currArea().blocks[partitioner.chType];
#endif
      DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctx=%d split=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxSplit, !isNo);
      return;
    }
#endif
  }
#endif

#if JVET_AH0135_TEMPORAL_PARTITIONING
  bool canBtt = canBh || canBv || canTh || canTv;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  const bool isQt = split == CU_QUAD_SPLIT && !isNo;
#else
  const bool isQt = split == CU_QUAD_SPLIT;
#endif
#endif

  if( canNo && canSplit )
  {
#if JVET_AH0135_TEMPORAL_PARTITIONING
    bool colSplitPredExist = false;
    SplitPred currentSplitPred;

    Picture* pColPic = cs.slice->getRefPic(RefPicList(cs.slice->isInterB() ? 1 - cs.slice->getColFromL0Flag() : 0), cs.slice->getColRefIdx());

    if (!cs.slice->isIntra() && pColPic != NULL && pColPic->cs->slice != NULL
      && (pColPic->cs->area.Y().contains(partitioner.currArea().blocks[partitioner.chType].pos().offset((partitioner.currArea().blocks[partitioner.chType].lumaSize().width) >> 1,
        ((partitioner.currArea().blocks[partitioner.chType].lumaSize().height) >> 1)))))
    {
      currentSplitPred = pColPic->cs->getQtDepthInfo(partitioner.currArea().blocks[partitioner.chType].pos().offset((partitioner.currArea().blocks[partitioner.chType].lumaSize().width) >> 1,
        ((partitioner.currArea().blocks[partitioner.chType].lumaSize().height) >> 1)));
      colSplitPredExist = true;
    }
    if (colSplitPredExist && (partitioner.currQtDepth < currentSplitPred.qtDetphCol))
    {
      if (canQt && canBtt)
      {
        m_BinEncoder.encodeBin(isQt, Ctx::SplitQtFlag(ctxQtSplit));
        canBtt = false;
        if (isQt)
        {
          return;
        }
        else
        {
          m_BinEncoder.encodeBin(!isNo, Ctx::SplitFlag(ctxSplit));
        }
      }
      else
      {
        m_BinEncoder.encodeBin(!isNo, Ctx::SplitFlag(ctxSplit));
      }
    }
    else
    {
#endif
      m_BinEncoder.encodeBin(!isNo, Ctx::SplitFlag(ctxSplit));
#if JVET_AH0135_TEMPORAL_PARTITIONING
    }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, !isNo );
#endif
  }

#if JVET_AI0136_ADAPTIVE_DUAL_TREE // FLL to check later
  if ( cu != nullptr && cs.slice->getSeparateTreeEnabled() && !cs.slice->getProcessingIntraRegion() && CU::isIntra( *cu ) && CU::isIntraRegionRoot( *cu, partitioner ) )
  {
    canBh = canBv = canTh = canTv = false;
  }
#endif

#if ENABLE_TRACING
  const CompArea& block = partitioner.currArea().blocks[partitioner.chType];
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctx=%d split=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxSplit, !isNo);

  if( isNo )
  {
    return;
  }



#if !JVET_AH0135_TEMPORAL_PARTITIONING
  const bool canBtt = canBh || canBv || canTh || canTv;
  const bool isQt   = split == CU_QUAD_SPLIT;
#endif

  if( canQt && canBtt )
  {
    m_BinEncoder.encodeBin( isQt, Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctx=%d qt=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxQtSplit, isQt );

  if( isQt )
  {
    return;
  }

  const bool canHor = canBh || canTh;
  const bool canVer = canBv || canTv;
  const bool  isVer = split == CU_VERT_SPLIT || split == CU_TRIV_SPLIT;

  if( canVer && canHor )
  {
    m_BinEncoder.encodeBin( isVer, Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  const bool can12 = isVer ? canBv : canBh;
  const bool  is12 = isVer ? ( split == CU_VERT_SPLIT ) : ( split == CU_HORZ_SPLIT );

  if( can12 && can14 )
  {
    m_BinEncoder.encodeBin( is12, Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() pos=(%d,%d) size=%dx%d chType=%d ctxHv=%d ctx12=%d mode=%d\n", block.x, block.y, block.width, block.height, partitioner.chType, ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, split);
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================
void CABACWriter::coding_unit( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
#endif
  CodingStructure& cs = *cu.cs;

  // skip flag
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( !cu.slice->getSeparateTreeEnabled() || !cu.slice->getProcessingIntraRegion() || (cu.slice->isIntra() && cu.slice->getUseIBC()) )
  {
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ((!cs.slice->isIntra() || cs.slice->getUseIBC()) && cu.Y().valid())
#else
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
#endif
  {
#if JVET_AG0117_CABAC_SPATIAL_TUNING && JVET_AI0136_ADAPTIVE_DUAL_TREE
    // If context data collection is active and if on bottom of the CTU, start the counters
    if ( m_BinEncoder.getBinBuffer() )
    {
      m_BinEncoder.setBinBufferActive( CU::isPartitionerOnCtuBottom( cu, partitioner ) );
    }
#endif
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    cu_skip_flag(cu, partitioner);
#else
    cu_skip_flag( cu );
#endif
#if JVET_AG0117_CABAC_SPATIAL_TUNING && JVET_AI0136_ADAPTIVE_DUAL_TREE
    // If context data collection is active and if on bottom of the CTU, start the counters
    if ( /*CU::isPartitionerOnCtuBottom( cu, partitioner ) &&*/ !CU::isOnCtuBottom(cu))
    {
      m_BinEncoder.setBinBufferActive( false );
    }
#endif
  }


  // skip data
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( !cu.slice->isIntra() || 
       (cu.slice->isIntra() && cu.cs->slice->getUseIBC() && cu.slice->getSPS()->getUseIbcMerge()
        && partitioner.currArea().lwidth() < 128 && partitioner.currArea().lheight() < 128 && cu.lwidth() < 128 && cu.lheight() < 128)  // disable IBC mode larger than 64x64
    ) 
#endif
  if( cu.skip )
  {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    DTRACE(g_trace_ctx, D_SYNTAX, "coding_unit() pos=(%d,%d) size=%dx%d chType=%d depth=%d\n", cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, cu.blocks[cu.chType].width, cu.blocks[cu.chType].height, cu.chType, cu.depth);
#endif
    CHECK( !cu.firstPU->mergeFlag, "Merge flag has to be on!" );
    CHECK(cu.colorTransform, "ACT should not be enabled for skip mode");
    PredictionUnit&   pu = *cu.firstPU;
    prediction_unit ( pu );
#if INTER_LIC
    cu_lic_flag(cu);
#endif
    end_of_ctu      ( cu, cuCtx );
    return;
  }


  // prediction mode and partitioning data
#if JVET_AI0136_ADAPTIVE_DUAL_TREE

#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // If context data collection is active and if on bottom of the CTU, start the counters
  if ( m_BinEncoder.getBinBuffer() )
  {
    m_BinEncoder.setBinBufferActive( CU::isPartitionerOnCtuBottom( cu, partitioner ) );
  }
#endif
  pred_mode( cu , partitioner);
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  // If context data collection is active and if on bottom of the CTU, start the counters
  if ( /*CU::isPartitionerOnCtuBottom( cu, partitioner ) &&*/ !CU::isOnCtuBottom(cu))
  {
    m_BinEncoder.setBinBufferActive( false );
  }
#endif
  if (!cu.slice->getSeparateTreeEnabled() || !cu.slice->getProcessingIntraRegion())
  {
#if JVET_AG0117_CABAC_SPATIAL_TUNING 
    // If context data collection is active and if on bottom of the CTU, start the counters
    if ( m_BinEncoder.getBinBuffer() )
    {
      m_BinEncoder.setBinBufferActive( CU::isPartitionerOnCtuBottom( cu, partitioner ) );
    }
#endif
    separate_tree_cu_flag(cu, partitioner);
#if JVET_AG0117_CABAC_SPATIAL_TUNING 
    // If context data collection is active and if on bottom of the CTU, start the counters
    if ( /*CU::isPartitionerOnCtuBottom( cu, partitioner ) &&*/ !CU::isOnCtuBottom(cu))
    {
      m_BinEncoder.setBinBufferActive( false );
    }
#endif
    if (cu.slice->getSeparateTreeEnabled() && (CU::isIntra(cu) || cu.slice->isIntra()) && cu.isSST && !cu.slice->getProcessingIntraRegion() && cu.separateTree)
    {
      return;
    }
  }
  }
#else
  pred_mode ( cu );
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  DTRACE(g_trace_ctx, D_SYNTAX, "coding_unit() pos=(%d,%d) size=%dx%d chType=%d depth=%d\n", cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, cu.blocks[cu.chType].width, cu.blocks[cu.chType].height, cu.chType, cu.depth);
#endif

#if ENABLE_DIMD
  cu_dimd_flag( cu );
#endif
  if (CU::isIntra(cu))
  {
    adaptive_color_transform(cu);
  }
  if (CU::isPLT(cu))
  {
    CHECK(cu.colorTransform, "ACT should not be enabled for PLT mode");
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (CS::isDualITree(*cu.cs))
#else
    if (cu.isSepTree())
#endif
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
      if (cu.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
      {
        cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
      }
    }
    else
    {
      if( cu.chromaFormat != CHROMA_400 )
      {
        cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
      }
      else
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
    }
    end_of_ctu(cu, cuCtx);
    return;
  }

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // end of cu
  end_of_ctu( cu, cuCtx );
}

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACWriter::cu_skip_flag( const CodingUnit& cu , Partitioner& partitioner)
#else
void CABACWriter::cu_skip_flag( const CodingUnit& cu )
#endif
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if  ( cu.slice->getSeparateTreeEnabled() && cu.slice->getProcessingIntraRegion() &&  !cu.slice->getSPS()->getUseIbcMerge() )
  {
    return;
  }
#endif

#if MULTI_HYP_PRED
  CHECK(cu.skip && cu.firstPU->numMergedAddHyps != cu.firstPU->addHypData.size(), "Multi Hyp: cu.skip && cu.firstPU->numMergedAddHyps != cu.firstPU->addHypData.size()");
  CHECK(cu.skip && !cu.firstPU->mergeFlag, "merge_flag has to be true for skipped CUs");
#endif
  unsigned ctxId = DeriveCtx::CtxSkipFlag( cu );

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cu.slice->isIntra() && cu.cs->slice->getUseIBC())
#else
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getUseIBC())
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
#else
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getSPS()->getIBCFlag())
#endif
#endif
  {

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if( !cu.slice->getSPS()->getUseIbcMerge() )
    {      
      return;
    }
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if (partitioner.currArea().lwidth() < 128 && partitioner.currArea().lheight() < 128 && cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#endif
    {
      m_BinEncoder.encodeBin((cu.skip), Ctx::SkipFlag(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0);
    }
    return;
  }
#if !INTER_RM_SIZE_CONSTRAINTS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ( !cu.cs->slice->getUseIBC() && cu.lwidth() == 4 && cu.lheight() == 4 )
#else
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
#endif
  {
    return;
  }
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if( !cu.cs->slice->getUseIBC() && cu.isConsIntra() )
#else
  if( !cu.cs->slice->getSPS()->getIBCFlag() && cu.isConsIntra() )
#endif
  {
    return;
  }
#endif
  m_BinEncoder.encodeBin( ( cu.skip ), Ctx::SkipFlag( ctxId ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, cu.skip ? 1 : 0 );

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (cu.skip && cu.cs->slice->getUseIBC())
#else
  if (cu.skip && cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if( !cu.slice->getSPS()->getUseIbcMerge() )
    {
      CHECK( CU::isIBC( cu ), "IBC skip shall not be used with IBC merge disabled" );
    }
    else
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.lwidth() < 128 && cu.lheight() < 128 ) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#endif
    {
#if !INTER_RM_SIZE_CONSTRAINTS
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        return;
      }
#endif
    unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
    m_BinEncoder.encodeBin(CU::isIBC(cu) ? 1 : 0, Ctx::IBCFlag(ctxidx));
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
  }
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
  if (cu.skip && CU::interCcpMergeZeroRootCbfAllowed(cu))
  {
    inter_ccp_merge_root_cbf_zero(cu);
  }
#endif
}

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACWriter::separate_tree_cu_flag( const CodingUnit& cu, Partitioner& partitioner )
{
  unsigned ctxId = 0;
  bool inferredSeparateTreeFlag = false;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( cu.slice->getSeparateTreeEnabled() && !cu.slice->getProcessingIntraRegion() && (CU::isIntra( cu ) || cu.slice->isIntra()) && CU::isIntraRegionRoot(cu, partitioner))
#else
  if ( cu.slice->getSeparateTreeEnabled() && !cu.slice->getProcessingIntraRegion() && CU::isIntra( cu ) && CU::isIntraRegionRoot( cu, partitioner ) )
#endif
  {
    bool canSplit = partitioner.canSplit( CU_QUAD_SPLIT, *cu.cs );
    canSplit = canSplit || partitioner.canSplit( CU_MT_SPLIT, *cu.cs );
    cu.cs->determineIfSeparateTreeFlagInferred( inferredSeparateTreeFlag, partitioner.currArea().lumaSize().width, partitioner.currArea().lumaSize().height, canSplit );

    if ( !inferredSeparateTreeFlag )
    {
      ctxId = DeriveCtx::CtxCUSeparateTree( *cu.cs, partitioner );
      m_BinEncoder.encodeBin( cu.separateTree, Ctx::SeparateTree( ctxId ) );
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "separate_tree_cu_flag() pred_mode=%d, separateTree=%d, ctxId=%d, inferredSeparateTreeFlag=%d, processingIntraRegion=%d, (x=%d, y=%d, w=%d, h=%d)\n", cu.predMode ? 1 : 0, cu.separateTree, ctxId, inferredSeparateTreeFlag ? 1 : 0, cu.cs->slice->getProcessingIntraRegion() ? 1: 0, cu.lumaPos().x, cu.lumaPos().y, partitioner.currArea().lumaSize().width, partitioner.currArea().lumaSize().height );
    //DTRACE( g_trace_ctx, D_SYNTAX, "separate_tree_cu_flag() pred_mode=%d, separateTree=%d, ctxId=%d, inferredSeparateTreeFlag=%d, processingSST=%d, (x=%d, y=%d, w=%d, h=%d)\n", cu.predMode ? 1 : 0, cu.separateTree, ctxId, inferredSeparateTreeFlag ? 1 : 0, cu.cs->slice->getProcessingIntraRegion() ? 1: 0, cu.lumaPos().x, cu.lumaPos().y, partitioner.currArea().lumaSize().width, partitioner.currArea().lumaSize().height );
  }
}
#endif

#if JVET_AI0136_ADAPTIVE_DUAL_TREE
void CABACWriter::pred_mode( const CodingUnit& cu ,  Partitioner&      partitioner )
#else
void CABACWriter::pred_mode( const CodingUnit& cu )
#endif
{
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( cu.slice->getSeparateTreeEnabled() && cu.slice->getProcessingIntraRegion() && !(cu.cs->slice->getUseIBC() && cu.chType != CHANNEL_TYPE_CHROMA) 
    && !(cu.cs->slice->getSPS()->getPLTMode())
    )
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "pred_mode() pred_mode=%d\n", cu.predMode);
    return;
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (cu.cs->slice->getUseIBC() && cu.chType != CHANNEL_TYPE_CHROMA)
#else
  if (cu.cs->slice->getSPS()->getIBCFlag() && cu.chType != CHANNEL_TYPE_CHROMA)
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isConsInter() )
    {
      assert( CU::isInter( cu ) );
      return;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra())
#else
    if ( cu.cs->slice->isIntra() || cu.isConsIntra() )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4))
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#endif
#endif
    {
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      if (partitioner.currArea().lwidth() < 128 && partitioner.currArea().lheight() < 128 && 
        cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#else
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
#endif
      {
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
      }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() 
        && partitioner.currArea().lwidth() < 128 && partitioner.currArea().lheight() < 128
        && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
#else
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
#endif
      {
        m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
    }
    else
    {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isConsInter() )
      {
        return;
      }
#endif
      m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
      if (CU::isIntra(cu) || CU::isPLT(cu))
      {
        if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
          m_BinEncoder.encodeBin(CU::isPLT(cu), Ctx::PLTFlag(0));
      }
      else
      {
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
        {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        m_BinEncoder.encodeBin(CU::isIBC(cu), Ctx::IBCFlag(ctxidx));
        }
      }
    }
  }
  else
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isConsInter() )
    {
      assert( CU::isInter( cu ) );
      return;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra())
#else
    if ( cu.cs->slice->isIntra() || cu.isConsIntra() )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4))
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#endif
#endif
    {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (CS::isDualITree(*cu.cs) || isLuma(cu.chType)))
#else
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  ) )
#endif
        m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
    }
    else
    {
      m_BinEncoder.encodeBin((CU::isIntra(cu) || CU::isPLT(cu)), Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu)));
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if ((CU::isIntra(cu) || CU::isPLT(cu)) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (CS::isDualITree(*cu.cs) || isLuma(cu.chType)))
#else
      if ((CU::isIntra(cu) || CU::isPLT(cu)) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (!cu.isLocalSepTree() || isLuma(cu.chType)))
#endif
      {
        m_BinEncoder.encodeBin((CU::isPLT(cu)), Ctx::PLTFlag(0));
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pred_mode() pred_mode=%d\n", cu.predMode);
}
void CABACWriter::bdpcm_mode( const CodingUnit& cu, const ComponentID compID )
{
  if( !cu.cs->sps->getBDPCMEnabledFlag() ) return;
  if( !CU::bdpcmAllowed( cu, compID ) ) return;

  int bdpcmMode = isLuma(compID) ? cu.bdpcmMode : cu.bdpcmModeChroma;

  unsigned ctxId = isLuma(compID) ? 0 : 2;
  m_BinEncoder.encodeBin(bdpcmMode > 0 ? 1 : 0, Ctx::BDPCMMode(ctxId));

  if (bdpcmMode)
  {
    m_BinEncoder.encodeBin(bdpcmMode > 1 ? 1 : 0, Ctx::BDPCMMode(ctxId+1));
  }
  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_LUMA, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_CHROMA, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmModeChroma);
  }
}


void CABACWriter::cu_pred_data( const CodingUnit& cu )
{
  if( CU::isIntra( cu ) )
  {
    if( cu.Y().valid() )
    {
      bdpcm_mode( cu, COMPONENT_Y );
    }

    intra_luma_pred_modes  ( cu );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if ((!cu.Y().valid() || (!CS::isDualITree(*cu.cs) && cu.Y().valid())) && isChromaEnabled(cu.chromaFormat))
#else
    if( ( !cu.Y().valid() || ( !cu.isSepTree() && cu.Y().valid() ) ) && isChromaEnabled(cu.chromaFormat) )
#endif
    {
      bdpcm_mode( cu, ComponentID(CHANNEL_TYPE_CHROMA) );
    }
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu );
  }

  imv_mode   ( cu );
  affine_amvr_mode( cu );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  for (auto &pu : CU::traversePUs(cu))
  {
    mvsd_data(pu);
  }
#endif
#if INTER_LIC
  cu_lic_flag(cu);
#endif
#if !(JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION) || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && (JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION))
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && (JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION)
  if (!cu.cs->sps->getUseMvdPred())
  {
#endif
  cu_bcw_flag(cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && (JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION)
  }
#endif
#endif
#if ENABLE_OBMC 
  obmc_flag( cu );
#endif
}

void CABACWriter::cu_bcw_flag(const CodingUnit& cu)
{
  if(!CU::isBcwIdxCoded(cu))
  {
    return;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  auto &pu = *cu.firstPU;
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    return;
  }
#endif

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");
  const uint8_t bcwCodingIdx = (uint8_t)g_bcwCodingOrder[CU::getValidBcwIdx(cu)];

  const int32_t numBcw = (cu.slice->getCheckLDC()) ? 5 : 3;
  m_BinEncoder.encodeBin((bcwCodingIdx == 0 ? 0 : 1), Ctx::BcwIdx(0));
  if(numBcw > 2 && bcwCodingIdx != 0)
  {
    const uint32_t prefixNumBits = numBcw - 2;
    const uint32_t step = 1;

    uint8_t idx = 1;
    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      if (bcwCodingIdx == idx)
      {
        m_BinEncoder.encodeBinEP(0);
        break;
      }
      else
      {
        m_BinEncoder.encodeBinEP(1);
        idx += step;
      }
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.bcwIdx ? 1 : 0);
#if MULTI_HYP_PRED
  mh_pred_data(*cu.firstPU);
#endif
}

#if ENABLE_OBMC
void CABACWriter::obmc_flag(const CodingUnit& cu)
{
  //obmc is false
  if (!cu.cs->sps->getUseOBMC() || CU::isIBC(cu) || cu.predMode == MODE_INTRA
#if INTER_LIC && !JVET_AD0213_LIC_IMP
    || cu.licFlag
#endif
    || cu.lwidth() * cu.lheight() < 32
    )
  {
    return;
  }

  //obmc is true
  if (cu.firstPU->mergeFlag)
  {
    return;
  }

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    CHECK(!cu.obmcFlag, "obmc flag should be true for AMVP with SbTMVP mode");
    return;
  }
#endif

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  bool ctxCond = cu.affine == false || cu.firstPU->addHypData.size() > 0;
  m_BinEncoder.encodeBin(cu.obmcFlag ? 1 : 0, ctxCond ? Ctx::ObmcFlag(0) : Ctx::ObmcFlag(1));
  DTRACE(g_trace_ctx, D_SYNTAX, "obmc_flag() pos=(%d,%d) ctx=%d obmc_flag=%d\n", cu.lx(), cu.ly(), ctxCond ? 0 : 1, cu.obmcFlag);
#else
  m_BinEncoder.encodeBin(cu.obmcFlag ? 1 : 0, Ctx::ObmcFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "obmc_flag() pos=(%d,%d) obmc_flag=%d\n", cu.lx(), cu.ly(), cu.obmcFlag);
#endif
}
#endif

void CABACWriter::xWriteTruncBinCode(uint32_t symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }

  int val = 1 << thresh;
  assert(val <= maxSymbol);
  assert((val << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  int b = maxSymbol - val;
  assert(b < val);
  if (symbol < val - b)
  {
    m_BinEncoder.encodeBinsEP(symbol, thresh);
  }
  else
  {
    symbol += val - b;
    assert(symbol < (val << 1));
    assert((symbol >> 1) >= val - b);
    m_BinEncoder.encodeBinsEP(symbol, thresh + 1);
  }
}

void CABACWriter::extend_ref_line(const PredictionUnit& pu)
{

  const CodingUnit& cu = *pu.cu;
  if( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma( cu.chType ) || cu.bdpcmMode
#if ENABLE_DIMD
    || cu.dimd
#endif
#if JVET_AB0155_SGPM
      || cu.sgpm
#endif
    )
  {
    return;
  }
  if( !cu.cs->sps->getUseMRL() )
  {
    return;
  }
#if JVET_AH0065_RELAX_LINE_BUFFER
  bool isFirstLineOfCtu = cu.block(COMPONENT_Y).y == 0;
#else
  bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
#endif
  if (isFirstLineOfCtu)
  {
    return;
  }
  int multiRefIdx = pu.multiRefIdx;
#if JVET_Y0116_EXTENDED_MRL_LIST
  if (MRL_NUM_REF_LINES > 1)
  {
#if JVET_W0123_TIMD_FUSION
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], cu.timd ? Ctx::MultiRefLineIdx(5) : Ctx::MultiRefLineIdx(0));
#else
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
#endif
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
#if JVET_W0123_TIMD_FUSION
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], cu.timd ? Ctx::MultiRefLineIdx(6) : Ctx::MultiRefLineIdx(1));
#else
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
#endif
      if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1]
#if JVET_W0123_TIMD_FUSION
        && !cu.timd
#endif
        )
      {
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[2], Ctx::MultiRefLineIdx(2));
        if (MRL_NUM_REF_LINES > 4 && multiRefIdx != MULTI_REF_LINE_IDX[2])
        {
          m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[3], Ctx::MultiRefLineIdx(3));
          if (MRL_NUM_REF_LINES > 5 && multiRefIdx != MULTI_REF_LINE_IDX[3])
          {
            m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[4], Ctx::MultiRefLineIdx(4));
          }
        }
      }
    }
  }
#else
  if (MRL_NUM_REF_LINES > 1)
  {
#if JVET_W0123_TIMD_FUSION
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], cu.timd ? Ctx::MultiRefLineIdx(2) : Ctx::MultiRefLineIdx(0));
#else
    m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
#endif
    if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
    {
#if JVET_W0123_TIMD_FUSION
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], cu.timd ? Ctx::MultiRefLineIdx(3) : Ctx::MultiRefLineIdx(1));
#else
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
#endif
    }
  }
#endif
}

void CABACWriter::extend_ref_line(const CodingUnit& cu)
{

  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode
#if ENABLE_DIMD 
    || cu.dimd
#endif
#if JVET_AB0155_SGPM
    || cu.sgpm
#endif
    )
  {
    return;
  }
  if( !cu.cs->sps->getUseMRL() )
  {
    return;
  }

  const int numBlocks = CU::getNumPUs(cu);
  const PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
#if JVET_AH0065_RELAX_LINE_BUFFER
    bool isFirstLineOfCtu = cu.block(COMPONENT_Y).y == 0;
#else
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
#endif
    if (isFirstLineOfCtu)
    {
      return;
    }
    int multiRefIdx = pu->multiRefIdx;
#if JVET_Y0116_EXTENDED_MRL_LIST
    if (MRL_NUM_REF_LINES > 1)
    {
#if JVET_W0123_TIMD_FUSION
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], cu.timd ? Ctx::MultiRefLineIdx(5) : Ctx::MultiRefLineIdx(0));
#else
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
#endif
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
#if JVET_W0123_TIMD_FUSION
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], cu.timd ? Ctx::MultiRefLineIdx(6) : Ctx::MultiRefLineIdx(1));
#else
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
#endif
        if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1]
#if JVET_W0123_TIMD_FUSION
          && !cu.timd
#endif
          )
        {
          m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[2], Ctx::MultiRefLineIdx(2));
          if (MRL_NUM_REF_LINES > 4 && multiRefIdx != MULTI_REF_LINE_IDX[2])
          {
            m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[3], Ctx::MultiRefLineIdx(3));
            if (MRL_NUM_REF_LINES > 5 && multiRefIdx != MULTI_REF_LINE_IDX[3])
            {
              m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[4], Ctx::MultiRefLineIdx(4));
            }
          }
        }
      }
    }
#else
    if (MRL_NUM_REF_LINES > 1)
    {
#if JVET_W0123_TIMD_FUSION
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], cu.timd ? Ctx::MultiRefLineIdx(2) : Ctx::MultiRefLineIdx(0));
#else
      m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[0], Ctx::MultiRefLineIdx(0));
#endif
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
#if JVET_W0123_TIMD_FUSION
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], cu.timd ? Ctx::MultiRefLineIdx(3) : Ctx::MultiRefLineIdx(1));
#else
        m_BinEncoder.encodeBin(multiRefIdx != MULTI_REF_LINE_IDX[1], Ctx::MultiRefLineIdx(1));
#endif
      }

    }
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) ref_idx=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->multiRefIdx);
    pu = pu->next;
  }
}

void CABACWriter::intra_luma_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
#if JVET_AH0076_OBIC
  if (cu.obicFlag)
  {
    return;
  }
#endif
  if( cu.bdpcmMode )
  {
    cu.firstPU->intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
    return;
  }
#if JVET_V0130_INTRA_TMP
  int tmpMaxSize = cu.cs->sps->getIntraTMPMaxSize();
  if( cu.lwidth() <= tmpMaxSize && cu.lheight() <= tmpMaxSize )
  {
	  tmp_flag(cu);
    if( cu.tmpFlag )
    {
      return;
    }
  }
#endif
  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
#if JVET_W0123_TIMD_FUSION
  cu_timd_flag(cu);
#endif
#if JVET_AG0058_EIP
  cu_eip_flag(cu);
  if (cu.eipFlag)
  {
    return;
  }
#endif
#if JVET_AB0155_SGPM
  sgpm_flag(cu);
  if (cu.sgpm)
  {
    return;
  }
#endif
#if JVET_AD0082_TMRL_CONFIG
  if (CU::allowTmrl(cu))
  {
    cuTmrlFlag(cu);
    if (cu.tmrlFlag)
    {
      return;
    }
  }
  else
  {
    extend_ref_line(cu);
  }
#else
#if JVET_AB0157_TMRL
  cuTmrlFlag(cu);
  if (cu.tmrlFlag)
  {
    return;
  }
#else
  extend_ref_line( cu );
#endif
#endif
  isp_mode( cu );
#if ENABLE_DIMD
  if (cu.dimd)
  {
    return;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (cu.timd)
  {
    return;
  }
#endif
  const int numBlocks = CU::getNumPUs( cu );
#if !SECONDARY_MPM
  unsigned  mpmPreds   [4][numMPMs];
#endif

  const PredictionUnit* pu = cu.firstPU;

  // prev_intra_luma_pred_flag
  for( int k = 0; k < numBlocks; k++ )
  {
    if ( pu->multiRefIdx )
    {
      CHECK(!pu->mpmFlag, "use of non-MPM");
    }
    else
    {
      m_BinEncoder.encodeBin(pu->mpmFlag, Ctx::IntraLumaMpmFlag());
    }

    pu = pu->next;
  }

  pu = cu.firstPU;

  // mpm_idx / rem_intra_luma_pred_mode
  for( int k = 0; k < numBlocks; k++ )
  {
    if(pu->mpmFlag)
    {
      const unsigned mpm_idx = pu->ipredIdx;
      {
        unsigned ctx = (pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
#if SECONDARY_MPM
        unsigned ctx2 = (ctx ? (pu->multiRefIdx == 0 ? 2 : 1) : 0);
#endif
        if( pu->multiRefIdx == 0 )
        {
          m_BinEncoder.encodeBin( mpm_idx > 0, Ctx::IntraLumaPlanarFlag( ctx ) );
        }

        if( mpm_idx )
        {
#if SECONDARY_MPM
          m_BinEncoder.encodeBin(mpm_idx > 1, Ctx::IntraLumaMPMIdx(0 + ctx2));
#else
          m_BinEncoder.encodeBinEP( mpm_idx > 1 );
#endif
        }
        if (mpm_idx > 1)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 2);
        }
        if (mpm_idx > 2)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 3);
        }
        if (mpm_idx > 3)
        {
          m_BinEncoder.encodeBinEP(mpm_idx > 4);
        }
      }
    }
    else
    {

      {        
#if SECONDARY_MPM
        if( pu->secondMpmFlag)
        {
          uint8_t secondaryMPMIdx = pu->ipredIdx - NUM_PRIMARY_MOST_PROBABLE_MODES;
          m_BinEncoder.encodeBin(1, Ctx::IntraLumaSecondMpmFlag());
#if JVET_AD0085_MPM_SORTING
          if (cu.cs->sps->getUseMpmSorting())
          {
            int ctxId = 0;
            const int maxNumCtxBins = (NUM_SECONDARY_MOST_PROBABLE_MODES / 4) - 1;
            int prefixIdx = secondaryMPMIdx / 4;
            for (int val = 0; val < maxNumCtxBins; val++)
            {
              unsigned int bin = (val == prefixIdx ? 0 : 1);
              m_BinEncoder.encodeBin(bin, Ctx::IntraLumaSecondMpmIdx(ctxId++));
              if (!bin)
              {
                break;
              }
            }

            m_BinEncoder.encodeBinsEP(secondaryMPMIdx - prefixIdx * 4, 2);
          }
          else
          {
#endif
          m_BinEncoder.encodeBinsEP( secondaryMPMIdx, 4);
#if JVET_AD0085_MPM_SORTING
          }
#endif
        }
        else
        {
          m_BinEncoder.encodeBin(0, Ctx::IntraLumaSecondMpmFlag());
                 
          unsigned nonMPMIdx = pu->ipredIdx;

          xWriteTruncBinCode( nonMPMIdx, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
        }
#else
        std::sort(mpmPred, mpmPred + numMPMs);

        for (int idx = numMPMs - 1; idx >= 0; idx--)
        {
          if (ipredMode > mpmPred[idx])
          {
            ipredMode--;
          }
        }
        CHECK(ipredMode >= 64, "Incorrect mode");

        xWriteTruncBinCode(ipredMode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
#endif
      }
    }

#if JVET_AC0105_DIRECTIONAL_PLANAR
    if (CU::isDirectionalPlanarAvailable(cu) && pu->mpmFlag && pu->ipredIdx == 0)
    {
      m_BinEncoder.encodeBin(cu.plIdx > 0, Ctx::IntraLumaPlanarFlag(2));
      if (cu.plIdx)
      {
        m_BinEncoder.encodeBin(cu.plIdx > 1, Ctx::IntraLumaPlanarFlag(3));
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) pl_idx=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, cu.plIdx);
    }
#endif

#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) predIdx=%d mpm=%d secondmpm=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->ipredIdx, pu->mpmFlag, pu->secondMpmFlag);
#else
    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
#endif
    pu = pu->next;
  }
}


void CABACWriter::intra_luma_pred_mode( const PredictionUnit& pu )
{
#if JVET_AH0076_OBIC
  if (pu.cu->obicFlag)
  {
    return;
  }
#endif
  if( pu.cu->bdpcmMode ) return;
#if JVET_V0130_INTRA_TMP
  // check if sufficient search range is available
  //bool bCheck = pu.cu->
  int tmpMaxSize = pu.cu->cs->sps->getIntraTMPMaxSize();
  if( pu.cu->lwidth() <= tmpMaxSize && pu.cu->lheight() <= tmpMaxSize )
  {
	  tmp_flag(*pu.cu);
    if( pu.cu->tmpFlag )
    {
      return;
    }
  }
#endif
  mip_flag(*pu.cu);
  if (pu.cu->mipFlag)
  {
    mip_pred_mode(pu);
    return;
  }
#if JVET_W0123_TIMD_FUSION
  cu_timd_flag(*pu.cu);
#endif
#if JVET_AG0058_EIP
  cu_eip_flag(*pu.cu);
  if (pu.cu->eipFlag)
  {
    return;
  }
#endif
#if JVET_AB0155_SGPM
  sgpm_flag(*pu.cu);
  if (pu.cu->sgpm)
  {
    return;
  }
#endif
#if JVET_AD0082_TMRL_CONFIG
  if (CU::allowTmrl(*pu.cu))
  {
    cuTmrlFlag(*pu.cu);
    if (pu.cu->tmrlFlag)
    {
      return;
    }
  }
  else
  {
    extend_ref_line(pu);
  }
#else
#if JVET_AB0157_TMRL
  cuTmrlFlag(*pu.cu);
  if (pu.cu->tmrlFlag)
  {
    return;
  }
#else
  extend_ref_line( pu );
#endif
#endif
  isp_mode( *pu.cu );
#if ENABLE_DIMD
  if (pu.cu->dimd)
  {
    return;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (pu.cu->timd)
  {
    return;
  }
#endif
  // prev_intra_luma_pred_flag
  if ( pu.multiRefIdx )
  {
    CHECK(!pu.mpmFlag, "use of non-MPM");
  }
  else
  {
    m_BinEncoder.encodeBin(pu.mpmFlag, Ctx::IntraLumaMpmFlag());
  }

  // mpm_idx / rem_intra_luma_pred_mode
  if (pu.mpmFlag)
  {
    unsigned mpm_idx = pu.ipredIdx;
    {
      unsigned ctx = (pu.cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
#if SECONDARY_MPM
      unsigned ctx2 = (ctx ? (pu.multiRefIdx == 0 ? 2 : 1) : 0);
#endif
      if (pu.multiRefIdx == 0)
        m_BinEncoder.encodeBin( mpm_idx > 0, Ctx::IntraLumaPlanarFlag(ctx) );
      if( mpm_idx )
      {
#if SECONDARY_MPM
        m_BinEncoder.encodeBin(mpm_idx > 1, Ctx::IntraLumaMPMIdx(0 + ctx2));
#else
        m_BinEncoder.encodeBinEP( mpm_idx > 1 );
#endif
      }
      if (mpm_idx > 1)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 2);
      }
      if (mpm_idx > 2)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 3);
      }
      if (mpm_idx > 3)
      {
        m_BinEncoder.encodeBinEP(mpm_idx > 4);
      }
    }
  }
  else
  {
#if !SECONDARY_MPM
    std::sort( mpmPred, mpmPred + numMPMs );
#endif
    { 
#if SECONDARY_MPM
      if (pu.secondMpmFlag)
      {
        unsigned secondMpmIdx = pu.ipredIdx - NUM_PRIMARY_MOST_PROBABLE_MODES;
        m_BinEncoder.encodeBin(1, Ctx::IntraLumaSecondMpmFlag());
#if JVET_AD0085_MPM_SORTING
        if (pu.cs->sps->getUseMpmSorting())
        {
          int ctxId = 0;
          const int maxNumCtxBins = (NUM_SECONDARY_MOST_PROBABLE_MODES / 4) - 1;
          int prefixIdx = secondMpmIdx / 4;
          for (int val = 0; val < maxNumCtxBins; val++)
          {
            unsigned int bin = (val == prefixIdx ? 0 : 1);
            m_BinEncoder.encodeBin(bin, Ctx::IntraLumaSecondMpmIdx(ctxId++));
            if (!bin)
            {
              break;
            }
          }

          m_BinEncoder.encodeBinsEP(secondMpmIdx - prefixIdx * 4, 2);
        }
        else
        {
#endif
        m_BinEncoder.encodeBinsEP(secondMpmIdx, 4);
#if JVET_AD0085_MPM_SORTING
        }
#endif
      }
      else
      {
        m_BinEncoder.encodeBin(0, Ctx::IntraLumaSecondMpmFlag());
                
        unsigned non_mpm_idx = pu.ipredIdx;

        xWriteTruncBinCode(non_mpm_idx, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
      }
#else
      unsigned ipredMode = pu.intraDir[0];

      xWriteTruncBinCode(ipredMode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);  // Remaining mode is truncated binary coded
#endif
    }
  }

#if JVET_AC0105_DIRECTIONAL_PLANAR
  if (CU::isDirectionalPlanarAvailable(*pu.cu) && pu.mpmFlag && pu.ipredIdx == 0)
  {
    m_BinEncoder.encodeBin(pu.cu->plIdx > 0, Ctx::IntraLumaPlanarFlag(2));
    if (pu.cu->plIdx)
    {
      m_BinEncoder.encodeBin(pu.cu->plIdx > 1, Ctx::IntraLumaPlanarFlag(3));
    }
  }
#endif
}

#if JVET_AG0058_EIP
void CABACWriter::cu_eip_flag(const CodingUnit& cu)
{
  if (cu.timd || cu.dimd || !cu.Y().valid() || !isLuma(cu.chType))
  {
    return;
  }

  const bool bCanUseEip = getAllowedEip(cu, COMPONENT_Y) || getAllowedEipMerge(cu, COMPONENT_Y);
  if (bCanUseEip)
  {
    m_BinEncoder.encodeBin(cu.eipFlag, Ctx::EipFlag(0));
    if (cu.eipFlag)
    {
      if (getAllowedEip(cu, COMPONENT_Y) && getAllowedEipMerge(cu, COMPONENT_Y))
      {
        m_BinEncoder.encodeBin(cu.eipMerge, Ctx::EipFlag(1));
      }
      if(cu.eipMerge)
      {
        CHECK(cu.firstPU->intraDir[0] >= NUM_EIP_MERGE_SIGNAL, "cu.firstPU->intraDir[0] >= NUM_EIP_MERGE_SIGNAL");
        unary_max_eqprob(cu.firstPU->intraDir[0], NUM_EIP_MERGE_SIGNAL - 1);
      }
      else
      {
        CHECK(cu.firstPU->intraDir[0] >= NUM_DERIVED_EIP, "cu.firstPU->intraDir[0] >= NUM_DERIVED_EIP");
        static_vector<EIPInfo, NUM_DERIVED_EIP> eipInfoList;
        xWriteTruncBinCode(cu.firstPU->intraDir[0], getAllowedCurEip(cu, COMPONENT_Y, eipInfoList));
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "eip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.eipFlag ? 1 : 0);
}
#endif

#if JVET_W0123_TIMD_FUSION
void CABACWriter::cu_timd_flag( const CodingUnit& cu )
{
  if (!cu.cs->sps->getUseTimd())
  {
    return;
  }
  if (cu.lwidth() * cu.lheight() > 1024 && cu.slice->getSliceType() == I_SLICE)
  {
    return;
  }
#if ENABLE_DIMD
  if (cu.dimd)
  {
    return;
  }
#endif
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxTimdFlag(cu);
  m_BinEncoder.encodeBin(cu.timd, Ctx::TimdFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_timd_flag() ctx=%d pos=(%d,%d) timd=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.timd);
}
#endif

#if JVET_AB0155_SGPM
void CABACWriter::sgpm_flag(const CodingUnit &cu)
{
  if (!cu.cs->sps->getUseSgpm())
  {
    return;
  }
  if (!(cu.lwidth() >= GEO_MIN_CU_SIZE_EX && cu.lheight() >= GEO_MIN_CU_SIZE_EX && cu.lwidth() <= GEO_MAX_CU_SIZE_EX
        && cu.lheight() <= GEO_MAX_CU_SIZE_EX && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth()
        && cu.lwidth() * cu.lheight() >= SGPM_MIN_PIX))
  {
    return;
  }

  if( cu.mipFlag
#if ENABLE_DIMD
    || cu.dimd
#endif
#if JVET_W0123_TIMD_FUSION
    || cu.timd
#endif
#if JVET_V0130_INTRA_TMP
    || cu.tmpFlag
#endif
    )
  {
    return;
  }
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    return;
  }
  if (!(cu.lx() && cu.ly()))
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxSgpmFlag(cu);
  m_BinEncoder.encodeBin(cu.sgpm, Ctx::SgpmFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_flag() pos=(%d,%d) ctx=%d sgpm_flag=%d\n", cu.lumaPos().x, cu.lumaPos().y, ctxId, cu.sgpm);

  if (cu.sgpm)
  {
    xWriteTruncBinCode(cu.sgpmIdx, SGPM_NUM);
    DTRACE(g_trace_ctx, D_SYNTAX, "sgpm_flag() pos=(%d,%d) sgpm_idx=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.sgpmIdx);
  }
}
#endif

#if ENABLE_DIMD
void CABACWriter::cu_dimd_flag(const CodingUnit& cu)
{
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || !cu.slice->getSPS()->getUseDimd())
  {
    return;
  }
  unsigned ctxId = DeriveCtx::CtxDIMDFlag(cu);
  m_BinEncoder.encodeBin(cu.dimd, Ctx::DimdFlag(ctxId));
#if JVET_AH0076_OBIC
  cu_obic_flag(cu);
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_dimd_flag() ctx=%d pos=(%d,%d) dimd=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.dimd);
}
#endif

#if JVET_AH0076_OBIC
void CABACWriter::cu_obic_flag(const CodingUnit& cu )
{
  if (!cu.dimd || !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    return;
  }
  if (!PU::isObicAvail(*cu.firstPU))
  {
    return;
  }
  m_BinEncoder.encodeBin(cu.obicFlag ? 1 : 0, Ctx::obicFlag(0));
}
#endif

void CABACWriter::intra_chroma_pred_modes( const CodingUnit& cu )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA))
#else
  if (cu.chromaFormat == CHROMA_400 || (cu.isSepTree() && cu.chType == CHANNEL_TYPE_LUMA))
#endif
  {
    return;
  }

  if( cu.bdpcmModeChroma )
  {
    PredictionUnit* pu = cu.firstPU;
    pu->intraDir[1] = cu.bdpcmModeChroma == 2 ? VER_IDX : HOR_IDX;
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
      pu->blocks[CHANNEL_TYPE_CHROMA].x, pu->blocks[CHANNEL_TYPE_CHROMA].y, pu->intraDir[CHANNEL_TYPE_CHROMA]);
    return;
  }
  const PredictionUnit* pu = cu.firstPU;

  intra_chroma_pred_mode( *pu );
  DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) fusion_mode=%d cccm_mode=%d\n",
    pu->blocks[CHANNEL_TYPE_CHROMA].x, pu->blocks[CHANNEL_TYPE_CHROMA].y, pu->isChromaFusion, pu->cccmFlag);
}

#if JVET_Z0050_CCLM_SLOPE
void CABACWriter::cclmDelta(const PredictionUnit& pu, int8_t delta)
{
  if ( delta )
  {
    int deltaAbs = abs( delta );
    
    if ( deltaAbs == 1 )
    {
      m_BinEncoder.encodeBinsEP( 0, 1 );
    }
    else if ( deltaAbs == 2 )
    {
      m_BinEncoder.encodeBinsEP( 2, 2 );
    }
    else if ( deltaAbs == 3 )
    {
      m_BinEncoder.encodeBinsEP( 6, 3 );
    }
    else
    {
      m_BinEncoder.encodeBinsEP( 7, 3 );
    }

    m_BinEncoder.encodeBin( delta < 0 ? 1 : 0, Ctx::CclmDeltaFlags(4) );
  }
}

void CABACWriter::cclmDeltaSlope(const PredictionUnit& pu)
{
#if JVET_AA0057_CCCM
  if ( pu.cccmFlag )
  {
    return;
  }
#endif

  if ( PU::hasCclmDeltaFlag( pu ) )
  {
    bool deltaActive = pu.cclmOffsets.isActive();

    m_BinEncoder.encodeBin( deltaActive ? 1 : 0, Ctx::CclmDeltaFlags(0) );
    
    if ( deltaActive )
    {
      bool bothActive = pu.cclmOffsets.cb0 && pu.cclmOffsets.cr0;

      m_BinEncoder.encodeBin( bothActive ? 1 : 0, Ctx::CclmDeltaFlags(3) );

      if ( !bothActive )
      {
        m_BinEncoder.encodeBin( pu.cclmOffsets.cb0 ? 1 : 0, Ctx::CclmDeltaFlags(1) );
        
#if MMLM
        if ( PU::isMultiModeLM( pu.intraDir[1] ) && !pu.cclmOffsets.cb0 )
        {
          m_BinEncoder.encodeBin( pu.cclmOffsets.cr0 ? 1 : 0, Ctx::CclmDeltaFlags(2) );
        }
#endif
      }

      cclmDelta( pu, pu.cclmOffsets.cb0 );
      cclmDelta( pu, pu.cclmOffsets.cr0 );
    
#if MMLM
      // Now the same for the second model (if applicable)
      if ( PU::isMultiModeLM( pu.intraDir[1] ) )
      {
        bool bothActive = pu.cclmOffsets.cb1 && pu.cclmOffsets.cr1;
        
        m_BinEncoder.encodeBin( bothActive ? 1 : 0, Ctx::CclmDeltaFlags(3) );
        
        if ( !bothActive )
        {
          m_BinEncoder.encodeBin( pu.cclmOffsets.cb1 ? 1 : 0, Ctx::CclmDeltaFlags(1) );
          
          if ( !pu.cclmOffsets.cb1 )
          {
            if ( pu.cclmOffsets.cb0 || pu.cclmOffsets.cr0 || pu.cclmOffsets.cb1 )
            {
              m_BinEncoder.encodeBin( pu.cclmOffsets.cr1 ? 1 : 0, Ctx::CclmDeltaFlags(2) );
            }
          }
        }

        cclmDelta( pu, pu.cclmOffsets.cb1 );
        cclmDelta( pu, pu.cclmOffsets.cr1 );
      }
#endif
    }
  }
}
#endif

#if JVET_AA0126_GLM
void CABACWriter::glmIdc(const PredictionUnit& pu)
{
  if ( PU::hasGlmFlag( pu ) )
  {
    bool glmActive = pu.glmIdc.isActive();

    m_BinEncoder.encodeBin( glmActive ? 1 : 0, Ctx::GlmFlags(0) );
    
    if ( glmActive )
    {
#if JVET_AB0092_GLM_WITH_LUMA
      int glmIdx = pu.glmIdc.cb0 - 1;
#if NUM_GLM_WEIGHT
      m_BinEncoder.encodeBin(glmIdx >= NUM_GLM_PATTERN ? 1 : 0, Ctx::GlmFlags(1));
      glmIdx -= glmIdx >= NUM_GLM_PATTERN ? NUM_GLM_PATTERN : 0;
#endif
      CHECK(pu.glmIdc.cb0 != pu.glmIdc.cr0, "wrong glm idx");
#if JVET_AA0057_CCCM
      m_BinEncoder.encodeBin(glmIdx > 0, Ctx::GlmFlags(2));
      if (glmIdx > 0)
      {
        m_BinEncoder.encodeBin(glmIdx > 1, Ctx::GlmFlags(3));
        if (glmIdx > 1)
        {
          m_BinEncoder.encodeBin(glmIdx > 2, Ctx::GlmFlags(4));
        }
      }
#else
      m_BinEncoder.encodeBinsEP(glmIdx, NUM_GLM_PATTERN_BITS);
#endif
#else
      bool bothActive = pu.glmIdc.cb0 && pu.glmIdc.cr0;

      m_BinEncoder.encodeBin( bothActive ? 1 : 0, Ctx::GlmFlags(3) );

      if ( !bothActive )
      {
        m_BinEncoder.encodeBin( pu.glmIdc.cb0 ? 1 : 0, Ctx::GlmFlags(1) );
      }

      if ( pu.glmIdc.cb0 ) 
      {
        int glmIdx = pu.glmIdc.cb0 - 1;
#if NUM_GLM_WEIGHT
        m_BinEncoder.encodeBin( glmIdx >= NUM_GLM_PATTERN ? 1 : 0, Ctx::GlmFlags(2) );
        glmIdx -= glmIdx >= NUM_GLM_PATTERN ? NUM_GLM_PATTERN : 0;
#endif
        m_BinEncoder.encodeBinsEP( glmIdx, NUM_GLM_PATTERN_BITS );
      }
      
      if ( pu.glmIdc.cr0 ) 
      {
        int glmIdx = pu.glmIdc.cr0 - 1;
#if NUM_GLM_WEIGHT
        m_BinEncoder.encodeBin( glmIdx >= NUM_GLM_PATTERN ? 1 : 0, Ctx::GlmFlags(4) );
        glmIdx -= glmIdx >= NUM_GLM_PATTERN ? NUM_GLM_PATTERN : 0;
#endif
        m_BinEncoder.encodeBinsEP( glmIdx, NUM_GLM_PATTERN_BITS );
      }
#endif
        
#if MMLM
      if ( PU::isMultiModeLM( pu.intraDir[1] ) )
      {
        CHECK(pu.glmIdc.cb0 != pu.glmIdc.cb1 
           || pu.glmIdc.cr0 != pu.glmIdc.cr1, "GLM cb0 != cb1 || cr0 != cr1")
      }
#endif
    }
  }
}
#endif
void CABACWriter::intra_chroma_lmc_mode(const PredictionUnit& pu)
{
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  decoderDerivedCcpModes(pu);
  if (pu.decoderDerivedCcpMode)
  {
    return;
  }
#endif
#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  nonLocalCCPIndex(pu);
  if (pu.idxNonLocalCCP)
  {
    return;
  }
#endif
  const unsigned intraDir = pu.intraDir[1];
#if MMLM
  int lmModeList[NUM_CHROMA_MODE];
#else
  int lmModeList[10];
#endif
  PU::getLMSymbolList(pu, lmModeList);
  int symbol = -1;
  for (int k = 0; k < LM_SYMBOL_NUM; k++)
  {
    if (lmModeList[k] == intraDir)
    {
      symbol = k;
      break;
    }
  }
  CHECK(symbol < 0, "invalid symbol found");

  m_BinEncoder.encodeBin(symbol == 0 ? 0 : 1, Ctx::CclmModeIdx(0));

  if (symbol > 0)
  {
#if MMLM
    CHECK(symbol > 5, "invalid symbol for MMLM");
    m_BinEncoder.encodeBin(symbol == 1 ? 0 : 1, Ctx::MMLMFlag(0));


    if (symbol > 1)
    {
      m_BinEncoder.encodeBinEP(symbol > 2);
    }

    if (symbol > 2)
    {
      m_BinEncoder.encodeBinEP(symbol > 3);
    }
    if (symbol > 3)
    {
      m_BinEncoder.encodeBinEP(symbol > 4);
    }
#else
    CHECK(symbol > 2, "invalid symbol for MMLM");
    unsigned int symbol_minus_1 = symbol - 1;
    m_BinEncoder.encodeBinEP(symbol_minus_1);
#endif
  }

#if JVET_AA0057_CCCM
  cccmFlag( pu );
#endif
#if JVET_AA0126_GLM
  glmIdc( pu );
#endif
#if JVET_Z0050_CCLM_SLOPE
  cclmDeltaSlope( pu );
#endif
#if JVET_AD0120_LBCCP
  ccInsideFilterFlag(pu);
#endif
}

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
void CABACWriter::decoderDerivedCcpModes(const PredictionUnit &pu)
{
  if (PU::hasDecoderDerivedCCP(pu))
  {
    m_BinEncoder.encodeBin(pu.decoderDerivedCcpMode > 0 ? 1 : 0, Ctx::decoderDerivedCCP(0));
  }
}
#endif

#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
void CABACWriter::nonLocalCCPIndex(const PredictionUnit &pu)
{
  if (PU::hasNonLocalCCP(pu))
  {
    CHECK(pu.idxNonLocalCCP < 0 || pu.idxNonLocalCCP > MAX_CCP_CAND_LIST_SIZE, "Invalid idxNonLocalCCP");
    {
      m_BinEncoder.encodeBin(pu.idxNonLocalCCP ? 1 : 0, Ctx::nonLocalCCP(0));
      if (pu.idxNonLocalCCP)
      {
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
        if (pu.cu->slice->getSPS()->getUseDdCcpFusion())
        {
          m_BinEncoder.encodeBin(pu.ddNonLocalCCPFusion > 0 ? 1 : 0, Ctx::ddNonLocalCCP(0));
        }
        if (pu.ddNonLocalCCPFusion > 0)
        {
          unary_max_eqprob(pu.ddNonLocalCCPFusion - 1, MAX_CCP_FUSION_NUM - 1);
        }
        else
        {
#endif
        unary_max_eqprob(pu.idxNonLocalCCP - 1, MAX_CCP_CAND_LIST_SIZE - 1);
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
        if (PU::hasCCPMergeFusionFlag(pu))
        {
          m_BinEncoder.encodeBin(pu.ccpMergeFusionFlag ? 1 : 0, Ctx::CCPMergeFusionFlag(0));
          if (pu.ccpMergeFusionFlag)
          {
            m_BinEncoder.encodeBin(pu.ccpMergeFusionType ? 1 : 0, Ctx::CCPMergeFusionType(0));
          }
        }
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
        }
#endif
      }
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
      DTRACE(g_trace_ctx, D_SYNTAX, "nonLocalCCPIndex() pos=(%d, %d) idxNonLocalCCP=%d\n", pu.blocks[pu.chType].pos().x, pu.blocks[pu.chType].pos().y, pu.idxNonLocalCCP);
#endif
    }
  }
}
#endif

#if JVET_AA0057_CCCM
void CABACWriter::cccmFlag(const PredictionUnit& pu)
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if ( pu.cs->sps->getUseCccm() == 0 )
  {    
    return;
  }
#endif
  const unsigned intraDir = pu.intraDir[1];
  
#if JVET_AE0100_BVGCCCM
  if ((intraDir == LM_CHROMA_IDX || intraDir == MMLM_CHROMA_IDX) && PU::hasBvgCccmFlag(pu) && PU::cccmSingleModeAvail(pu, LM_CHROMA_IDX) && PU::bvgCccmMultiModeAvail(pu, intraDir))
  {
    m_BinEncoder.encodeBin( pu.bvgCccmFlag ? 1 : 0, Ctx::BvgCccmFlag( 0 ) );
    if (pu.bvgCccmFlag)
    {
      return;
    }
  }
#endif
#if JVET_AB0143_CCCM_TS
  bool isCCCMEnabled = false;
  if (intraDir == LM_CHROMA_IDX)
  {
    isCCCMEnabled = PU::cccmSingleModeAvail(pu, LM_CHROMA_IDX);
  }
  else if (intraDir == MDLM_L_IDX)
  {
    isCCCMEnabled = PU::isLeftCccmMode(pu, MDLM_L_IDX);
  }
  else if (intraDir == MDLM_T_IDX)
  {
    isCCCMEnabled = PU::isTopCccmMode(pu, MDLM_T_IDX);
  }
#if MMLM
  else if (intraDir == MMLM_CHROMA_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX);
  }
  else if (intraDir == MMLM_L_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_L_IDX);
  }
  else if (intraDir == MMLM_T_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_T_IDX);
  }
#endif

  if (isCCCMEnabled)
#else
  if ( PU::cccmSingleModeAvail(pu, intraDir) || PU::cccmMultiModeAvail(pu, intraDir) )
#endif
  {
    m_BinEncoder.encodeBin( pu.cccmFlag ? 1 : 0, Ctx::CccmFlag( 0 ) );
#if JVET_AD0202_CCCM_MDF
    if (pu.cccmFlag)
    {
      bool isCccmWithMdfEnabled = true;
      if (intraDir == MMLM_CHROMA_IDX)
      {
        isCccmWithMdfEnabled = PU::isMultiCccmWithMdf(pu, MMLM_CHROMA_IDX);
      }
      else if (intraDir == MMLM_L_IDX)
      {
        isCccmWithMdfEnabled = PU::isMultiCccmWithMdf(pu, MMLM_L_IDX);
      }
      else if (intraDir == MMLM_T_IDX)
      {
        isCccmWithMdfEnabled = PU::isMultiCccmWithMdf(pu, MMLM_T_IDX);
      }

      if (isCccmWithMdfEnabled)
      {
        m_BinEncoder.encodeBin(pu.cccmMultiFilterIdx > 0 ? 1 : 0, Ctx::CccmMpfFlag(0));
        if (pu.cccmMultiFilterIdx > 0)
        {
          m_BinEncoder.encodeBin(pu.cccmMultiFilterIdx > 1 ? 1 : 0, Ctx::CccmMpfFlag(1));
          if (pu.cccmMultiFilterIdx > 1)
          {
            m_BinEncoder.encodeBin(pu.cccmMultiFilterIdx > 2 ? 1 : 0, Ctx::CccmMpfFlag(2));
          }
        }
      }
    }

    if (!pu.cccmMultiFilterIdx)
    {
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
    if ( pu.cccmFlag && ( pu.cs->sps->getUseCccm() == 2 ) )
    {
      m_BinEncoder.encodeBin( pu.cccmNoSubFlag ? 1 : 0, Ctx::CccmFlag( 1 ) );
    }
#endif
#if JVET_AC0054_GLCCCM
#if !JVET_AC0147_CCCM_NO_SUBSAMPLING
    unsigned ctxId = 1;
    if (pu.cccmFlag)
#else
    unsigned ctxId = 2;
    if (pu.cccmFlag && !pu.cccmNoSubFlag)
#endif
    {
      m_BinEncoder.encodeBin( pu.glCccmFlag ? 1 : 0, Ctx::CccmFlag( ctxId ) );
    }
#endif
#if JVET_AD0202_CCCM_MDF
    }
#endif
  }
}
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
void CABACWriter::intraChromaFusionMode(const PredictionUnit& pu)
{
  int symbol = pu.isChromaFusion;
  m_BinEncoder.encodeBin(symbol > 0 ? 1 : 0, Ctx::ChromaFusionMode());

  if (symbol > 0)
  {
    m_BinEncoder.encodeBin(symbol > 1 ? 1 : 0, Ctx::ChromaFusionType()); // Default=1

#if MMLM
    if (symbol > 1)
    {
      m_BinEncoder.encodeBin(symbol > 2 ? 1 : 0, Ctx::ChromaFusionCclm());  // LM=2
    }
#endif
  }
}
#endif

#if JVET_AD0120_LBCCP
void CABACWriter::ccInsideFilterFlag(const PredictionUnit &pu)
{
#if JVET_AE0100_BVGCCCM
  if (pu.bvgCccmFlag)
  {
    return;
  }
#endif
  const unsigned intraDir = pu.intraDir[1];

  if (PU::hasCcInsideFilterFlag(pu, intraDir))
  {
    m_BinEncoder.encodeBin(pu.ccInsideFilter, Ctx::CcInsideFilterFlag(0));
  }
}
#endif

void CABACWriter::intra_chroma_pred_mode(const PredictionUnit& pu)
{
  const unsigned intraDir = pu.intraDir[1];
  if (pu.cu->colorTransform)
  {
    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM for adaptive color transform");
    return;
  }
#if CCLM_LATENCY_RESTRICTION_RMV
  if (pu.cs->sps->getUseLMChroma() )
#else
  if (pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed())
#endif
  {
    m_BinEncoder.encodeBin(PU::isLMCMode(intraDir) ? 1 : 0, Ctx::CclmModeFlag(0));
    if (PU::isLMCMode(intraDir))
    {
      intra_chroma_lmc_mode(pu);
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
        pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);

      return;
    }
  }

#if JVET_AC0071_DBV
#if JVET_AH0136_CHROMA_REORDERING
  bool hasDBV = false;
#endif
  if (PU::hasChromaBvFlag(pu))
  {
#if JVET_AH0136_CHROMA_REORDERING
    hasDBV = true;
    bool isDbvChromaMode = intraDir == DBV_CHROMA_IDX;
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
    if (( CS::isDualITree(*pu.cs) || (pu.cu->isSST && pu.cu->separateTree) ) && pu.cs->sps->getUseChromaReordering() && pu.cs->slice->isIntra())
#else
    if (CS::isDualITree(*pu.cs) && pu.cs->sps->getUseChromaReordering())
#endif
    {
      int mode = PU::isDbvMode(pu.intraDir[1]) ? pu.intraDir[1] : PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA);
      isDbvChromaMode = mode == pu.cu->chromaList[0];
    }
#else
    const bool isDbvChromaMode = intraDir == DBV_CHROMA_IDX;
#endif
    m_BinEncoder.encodeBin(isDbvChromaMode ? 0 : 1, Ctx::DbvChromaMode());
    if (isDbvChromaMode)
    {
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
#if JVET_AH0136_CHROMA_REORDERING
        intraChromaFusionMode(pu);
#else
        const bool isFusion = pu.isChromaFusion;
        m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
      }
#if JVET_AH0136_CHROMA_REORDERING
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x,
             pu.blocks[CHANNEL_TYPE_CHROMA].y, isDbvChromaMode ? DBV_CHROMA_IDX : pu.intraDir[CHANNEL_TYPE_CHROMA]);
#else
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
        pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
#endif
      return;
    }
  }
#endif

#if JVET_AH0136_CHROMA_REORDERING
#if JVET_AI0136_ADAPTIVE_DUAL_TREE
  if ( (CS::isDualITree(*pu.cs) || (pu.cu->isSST && pu.cu->separateTree) ) && pu.cs->sps->getUseChromaReordering() && pu.cs->slice->isIntra())
#else
  if (CS::isDualITree(*pu.cs) && pu.cs->sps->getUseChromaReordering())
#endif
  {
    int chromaIdx = 0;
    bool hasMode = false;
    int mode = PU::isDbvMode(pu.intraDir[1]) ? pu.intraDir[1] : PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA);

    int start = 0;
    int end = 6;
#if JVET_AC0071_DBV
    if (hasDBV)
    {
      start++;
      end++;
    }
#endif
#if ENABLE_DIMD && JVET_Z0050_DIMD_CHROMA_FUSION
    if (!pu.cu->slice->getSPS()->getUseDimd())
    {
      end--;
    }
#endif

    for (int i = start; i < end; i++)
    {
      if (mode == pu.cu->chromaList[i])
      {
        chromaIdx = i;
        hasMode = true;
        break;
      }
    }

    if (hasDBV)
    {
      chromaIdx--;
    }

    CHECK(!hasMode, "wrong mode");
    const bool     isDerivedMode = chromaIdx == 0;
    m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));
    if (isDerivedMode)
    {
#if JVET_Z0050_DIMD_CHROMA_FUSION
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
#if JVET_AC0119_LM_CHROMA_FUSION
        intraChromaFusionMode(pu);
#else
        const bool     isFusion = pu.isChromaFusion;
        m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
      }
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
        pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
      return;
    }

#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
    if (pu.cu->slice->getSPS()->getUseDimd())
    {
      const bool     isDimdChromaMode = chromaIdx == 1;
      m_BinEncoder.encodeBin(isDimdChromaMode ? 0 : 1, Ctx::DimdChromaMode());
      if (isDimdChromaMode)
      {
        if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
        {
#if JVET_AC0119_LM_CHROMA_FUSION
          intraChromaFusionMode(pu);
#else
          const bool     isFusion = pu.isChromaFusion;
          m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
          pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
        return;
      }
    }
    else
    {
      chromaIdx++;
    }
#endif

    // chroma candidate index
    {
      m_BinEncoder.encodeBinsEP(chromaIdx - 2, 2);
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) cand_idx=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, chromaIdx - 2);
#if JVET_Z0050_DIMD_CHROMA_FUSION
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
#if JVET_AC0119_LM_CHROMA_FUSION
        intraChromaFusionMode(pu);
#else
        const bool     isFusion = pu.isChromaFusion;
        m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
      }
#endif
    }
    return;
  }
#endif
  const bool     isDerivedMode = intraDir == DM_CHROMA_IDX;
  m_BinEncoder.encodeBin(isDerivedMode ? 0 : 1, Ctx::IntraChromaPredMode(0));
  if (isDerivedMode)
  {
#if JVET_Z0050_DIMD_CHROMA_FUSION
    if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
    {
#if JVET_AC0119_LM_CHROMA_FUSION
      intraChromaFusionMode(pu);
#else
      const bool     isFusion = pu.isChromaFusion;
      m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
    }
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
      pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
    return;
  }

#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  if (pu.cu->slice->getSPS()->getUseDimd())
  {
    const bool     isDimdChromaMode = intraDir == DIMD_CHROMA_IDX;
    m_BinEncoder.encodeBin(isDimdChromaMode ? 0 : 1, Ctx::DimdChromaMode());
    if (isDimdChromaMode)
    {
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
#if JVET_AC0119_LM_CHROMA_FUSION
        intraChromaFusionMode(pu);
#else
        const bool     isFusion = pu.isChromaFusion;
        m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) dir=%d\n",
        pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, pu.intraDir[CHANNEL_TYPE_CHROMA]);
      return;
    }
  }
#endif

  // chroma candidate index
  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(pu, chromaCandModes);

  int candId = 0;
  for (; candId < NUM_CHROMA_MODE; candId++)
  {
    if (intraDir == chromaCandModes[candId])
    {
      break;
    }
  }

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
  {
    m_BinEncoder.encodeBinsEP(candId, 2);
    DTRACE(g_trace_ctx, D_SYNTAX, "intra_chroma_pred_modes() pos=(%d,%d) cand_idx=%d\n", pu.blocks[CHANNEL_TYPE_CHROMA].x, pu.blocks[CHANNEL_TYPE_CHROMA].y, candId);
#if JVET_Z0050_DIMD_CHROMA_FUSION
    if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
    {
#if JVET_AC0119_LM_CHROMA_FUSION
      intraChromaFusionMode(pu);
#else
      const bool     isFusion = pu.isChromaFusion;
      m_BinEncoder.encodeBin(isFusion ? 1 : 0, Ctx::ChromaFusionMode());
#endif
    }
#endif
  }
}

void CABACWriter::cu_residual( const CodingUnit& cu, Partitioner& partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
#if MULTI_HYP_PRED
    if (!(pu.mergeFlag && pu.numMergedAddHyps == pu.addHypData.size()))
#else
    if (!pu.mergeFlag)
#endif
    {
      rqt_root_cbf( cu );
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }

    if( !cu.rootCbf )
    {
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
      if (CU::interCcpMergeZeroRootCbfAllowed(cu))
      {
        inter_ccp_merge_root_cbf_zero(cu);
      }
#endif
      CHECK(cu.colorTransform, "ACT should not be enabled for root_cbf = 0");
      return;
    }
  }

  if (CU::isInter(cu) || CU::isIBC(cu))
  {
    adaptive_color_transform(cu);
  }

  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
  cuCtx.lfnstLastScanPos                              = false;
  cuCtx.violatesMtsCoeffConstraint                    = false;
  cuCtx.mtsLastScanPos                                = false;
#if JVET_Y0142_ADAPT_INTRA_MTS
  cuCtx.mtsCoeffAbsSum                                = 0;
#endif
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    transform_tree( *cu.cs, subTuPartitioner, cuCtx,             CU::getISPType( cu, getFirstComponentOfChannel( partitioner.chType)  ), 0 );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx );
  }

  residual_lfnst_mode( cu, cuCtx );
#if JVET_AE0102_LFNST_CTX
  // call coeff coding, check the ordering of tus
  if ( isEncoding() )
  {
    for (auto &currTU : CU::traverseTUs(cu))
    {
      transform_unit(currTU, cuCtx, partitioner, -1, true);
    }
  }
#endif
  mts_idx            ( cu, &cuCtx );

#if SIGN_PREDICTION
  if(typeid(m_BinEncoder) == typeid(BinEncoder_Std))
  {
    // In the final coding stage, predicted signs are signaled here.
    for( auto &currTU : CU::traverseTUs( cu ) )
    {
      for( int compIdx = COMPONENT_Y; compIdx < MAX_NUM_COMPONENT; ++compIdx)
      {
        ComponentID compID = (ComponentID)compIdx;

        if(compIdx >= currTU.blocks.size())
        {
          continue;
        }
        if(currTU.jointCbCr)
        {
          if( !( currTU.jointCbCr >> 1 ) && compID == COMPONENT_Cb )
          {
            continue;
          }
          if( ( currTU.jointCbCr >> 1 ) && compID == COMPONENT_Cr )
          {
            continue;
          }
        }

        if(currTU.blocks[compID].valid() && TU::getCbf( currTU, compID ) && TU::getDelayedSignCoding(currTU, compID))
        {
          codePredictedSigns(const_cast<TransformUnit &>(currTU), compID);
        }
      }
    }
  }
#endif
}

void CABACWriter::rqt_root_cbf( const CodingUnit& cu )
{
   m_BinEncoder.encodeBin( cu.rootCbf, Ctx::QtRootCbf() ) ;
  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
void CABACWriter::inter_ccp_merge_root_cbf_zero(const CodingUnit &cu) 
{
  unary_max_symbol(cu.interCcpMergeZeroRootCbfIdc, Ctx::InterCcpMergeZeroRootCbfIdc(0),
    Ctx::InterCcpMergeZeroRootCbfIdc(1), MAX_CCP_MERGE_WEIGHT_IDX);
  DTRACE(g_trace_ctx, D_SYNTAX, "inter_ccp_merge_root_cbf_zero() pos=(%d,%d) inter_ccp_merge_root_cbf_zero_flag=%d\n", cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, cu.interCcpMergeZeroRootCbfIdc > 0 ? 1 : 0);
}
#endif

void CABACWriter::adaptive_color_transform(const CodingUnit& cu)
{
  if (!cu.slice->getSPS()->getUseColorTrans())
  {
    return;
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*cu.cs))
#else
  if (cu.isSepTree())
#endif
  {
    CHECK(cu.colorTransform, "adaptive color transform should be disabled when dualtree and localtree are enabled");
    return;
  }
  if (CU::isInter(cu) || CU::isIBC(cu) || CU::isIntra(cu))
  {
    m_BinEncoder.encodeBin(cu.colorTransform, Ctx::ACTFlag());
    DTRACE(g_trace_ctx, D_SYNTAX, "adaptive_color_transform() act_flag=(%d,%d)\n", cu.lumaPos().x, cu.lumaPos().y, cu.colorTransform);
  }
}

void CABACWriter::sbt_mode( const CodingUnit& cu )
{
  uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();
  uint8_t sbtIdx = cu.getSbtIdx();
  uint8_t sbtPos = cu.getSbtPos();

  //bin - flag
  bool sbtFlag = cu.sbtInfo != 0;
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  m_BinEncoder.encodeBin( sbtFlag, Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  bool sbtQuadFlag = sbtIdx == SBT_HOR_QUAD || sbtIdx == SBT_VER_QUAD;
  bool sbtHorFlag = sbtIdx == SBT_HOR_HALF || sbtIdx == SBT_HOR_QUAD;
  bool sbtPosFlag = sbtPos == SBT_POS1;

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  //bin - type
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    m_BinEncoder.encodeBin( sbtQuadFlag, Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    assert( sbtQuadFlag == 0 );
  }

  //bin - dir
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    m_BinEncoder.encodeBin( sbtHorFlag, Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    assert( sbtHorFlag == ( ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow ) ) );
  }

  //bin - pos
  m_BinEncoder.encodeBin( sbtPosFlag, Ctx::SbtPosFlag( 0 ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbt_info=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}

void CABACWriter::end_of_ctu( const CodingUnit& cu, CUCtx& cuCtx )
{
  const bool    isLastSubCUOfCtu  = CU::isLastSubCUOfCtu( cu );

  if ( isLastSubCUOfCtu
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    && (!CS::isDualITree(*cu.cs) || cu.chromaFormat == CHROMA_400 || isChroma(cu.chType))
#else
    && (!cu.isSepTree() || cu.chromaFormat == CHROMA_400 || isChroma(cu.chType))
#endif
      )
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );

  }
}

void CABACWriter::cu_palette_info(const CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  const SPS&       sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  uint32_t indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int maxPltSize = CS::isDualITree(*cu.cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#else
  int maxPltSize = cu.isSepTree() ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#endif
  if (cu.lastPLTSize[compBegin])
  {

    xEncodePLTPredIndicator(cu, maxPltSize, compBegin);
  }

  uint32_t reusedPLTnum = 0;
  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if( cu.reuseflag[compBegin][idx] )
    {
      reusedPLTnum++;
    }
  }
  if (reusedPLTnum < maxPltSize)
  {
    exp_golomb_eqprob(cu.curPLTSize[compBegin] - reusedPLTnum, 0);
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() recieved_plt_num=%d\n", cu.curPLTSize[compBegin] - reusedPLTnum);

  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = cu.reusePLTSize[compBegin]; idx < cu.curPLTSize[compBegin]; idx++)
    {
      ComponentID compID = (ComponentID)comp;
      const int  channelBitDepth = sps.getBitDepth(toChannelType(compID));
      m_BinEncoder.encodeBinsEP(cu.curPLT[comp][idx], channelBitDepth);
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() comp=%d idx=%d cur_plt=%d\n", comp, idx, cu.curPLT[compID][idx]);
    }
  }
  uint32_t signalEscape = (cu.useEscape[compBegin]) ? 1 : 0;
  if (cu.curPLTSize[compBegin] > 0)
  {
    m_BinEncoder.encodeBinEP(signalEscape);
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() esc_code=%d\n", signalEscape);
  }
  //encode index map
  uint32_t   height = cu.block(compBegin).height;
  uint32_t   width = cu.block(compBegin).width;

  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t total = height * width;
  if( indexMaxSize > 1 )
  {
    codeScanRotationModeFlag( cu, compBegin );
  }
  else
  {
    assert( !cu.useRotation[compBegin] );
  }

  if (cu.useEscape[compBegin] && cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
#else
    if (!cu.isSepTree() || isLuma(tu.chType))
#endif
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (cu.useEscape[compBegin] && cu.cs->slice->getUseChromaQpAdj() && !cuCtx.isChromaQpAdjCoded)
  {
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  uint32_t prevRunPos = 0;
  unsigned prevRunType = 0;
  for (int subSetId = 0; subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE; subSetId++)
  {
    cuPaletteSubblockInfo(cu, compBegin, numComp, subSetId, prevRunPos, prevRunType);
  }
  CHECK(cu.curPLTSize[compBegin] > maxPltSize, " Current palette size is larger than maximum palette size");
}
void CABACWriter::cuPaletteSubblockInfo(const CodingUnit& cu, ComponentID compBegin, uint32_t numComp, int subSetId, uint32_t& prevRunPos, unsigned& prevRunType)
{
  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&  tu  = *cu.firstTU;
  PLTtypeBuf      runType = tu.getrunType(compBegin);
  PelBuf          curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t        indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  uint32_t        totalPel = cu.block(compBegin).height*cu.block(compBegin).width;

  int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
  int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
  maxSubPos = (maxSubPos > totalPel) ? totalPel : maxSubPos; // if last position is out of the current CU size

  unsigned runCopyFlag[(1 << LOG2_PALETTE_CG_SIZE)];
  for( int i = 0; i < (1 << LOG2_PALETTE_CG_SIZE); i++ )
  {
    runCopyFlag[i] = MAX_INT;
  }

  if( minSubPos == 0 )
  {
    runCopyFlag[0] = 0;
  }

// PLT runCopy flag and runType - context coded
  int curPos = minSubPos;
  for (; curPos < maxSubPos && indexMaxSize > 1; curPos++)
  {
    uint32_t posy = m_scanOrder[curPos].y;
    uint32_t posx = m_scanOrder[curPos].x;
    uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
    uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
    // encode runCopyFlag
    bool identityFlag = !((runType.at(posx, posy) != runType.at(posxprev, posyprev))
      || ((runType.at(posx, posy) == PLT_RUN_INDEX) && (curPLTIdx.at(posx, posy) != curPLTIdx.at(posxprev, posyprev))));

    const CtxSet&   ctxSet = (prevRunType == PLT_RUN_INDEX)? Ctx::IdxRunModel: Ctx::CopyRunModel;
    if ( curPos > 0 )
    {
      int dist = curPos - prevRunPos - 1;
      const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(prevRunType, dist);
      runCopyFlag[curPos - minSubPos] = identityFlag;
      m_BinEncoder.encodeBin( identityFlag, ctxSet( ctxId ) );
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_copy_flag() bin=%d ctx=%d\n", identityFlag, ctxId);
    }
    // encode run_type
    if ( !identityFlag || curPos == 0 )
    {
      prevRunPos  = curPos;
      prevRunType = runType.at(posx, posy);
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else if (curPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        assert(runType.at(posx, posy) == PLT_RUN_INDEX);
      }
      else
      {
        m_BinEncoder.encodeBin(runType.at(posx, posy), Ctx::RunTypeFlag());
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_type_flag() bin=%d sp=%d\n", runType.at(posx, posy), curPos);
    }
  }

// PLT index values - bypass coded
  if (indexMaxSize > 1)
  {
    curPos = minSubPos;
    for (; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if ( runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        writePLTIndex(cu, curPos, curPLTIdx, runType, indexMaxSize, compBegin);
        DTRACE(g_trace_ctx, D_SYNTAX, "plt_idx_idc() value=%d sp=%d\n", curPLTIdx.at(posx, posy), curPos);
      }
    }
  }

// Quantized escape colors - bypass coded
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    ComponentID compID = (ComponentID)comp;
    for (curPos = minSubPos; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
          PLTescapeBuf    escapeValue = tu.getescapeValue((ComponentID)comp);
          if (compID == COMPONENT_Y || compBegin != COMPONENT_Y)
          {
            exp_golomb_eqprob((unsigned)escapeValue.at(posx, posy), 5);
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
          if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
          {
            uint32_t posxC = posx >> scaleX;
            uint32_t posyC = posy >> scaleY;
            exp_golomb_eqprob((unsigned)escapeValue.at(posxC, posyC), 5);
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
      }
    }
  }
}
void CABACWriter::codeScanRotationModeFlag(const CodingUnit& cu, ComponentID compBegin)
{
  m_BinEncoder.encodeBin((cu.useRotation[compBegin]), Ctx::RotationFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_palette_info() use_rotation=%d\n", cu.useRotation[compBegin]);
}
void CABACWriter::xEncodePLTPredIndicator(const CodingUnit& cu, uint32_t maxPLTSize, ComponentID compBegin)
{
  int lastPredIdx = -1;
  uint32_t run = 0;
  uint32_t numPLTPredicted = 0;
  for (uint32_t idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      numPLTPredicted++;
      lastPredIdx = idx;
    }
  }

  int idx = 0;
  while (idx <= lastPredIdx)
  {
    if (cu.reuseflag[compBegin][idx])
    {
      exp_golomb_eqprob(run ? run + 1 : run, 0);
      run = 0;
    }
    else
    {
      run++;
    }
    idx++;
  }
  if ((numPLTPredicted < maxPLTSize && lastPredIdx + 1 < cu.lastPLTSize[compBegin]) || !numPLTPredicted)
  {
    exp_golomb_eqprob(1, 0);
  }
}
Pel CABACWriter::writePLTIndex(const CodingUnit& cu, uint32_t idx, PelBuf& paletteIdx, PLTtypeBuf& paletteRunType, int maxSymbol, ComponentID compBegin)
{
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  Pel curLevel = (paletteIdx.at(posx, posy) == cu.curPLTSize[compBegin]) ? (maxSymbol - 1) : paletteIdx.at(posx, posy);
  if (idx) // R0348: remove index redundancy
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      Pel leftLevel = paletteIdx.at(prevposx, prevposy); // left index
      if (leftLevel == cu.curPLTSize[compBegin]) // escape mode
      {
        leftLevel = maxSymbol - 1;
      }
      assert(leftLevel != curLevel);
      if (curLevel > leftLevel)
      {
        curLevel--;
      }
    }
    else
    {
      Pel aboveLevel;
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        aboveLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin]) // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        aboveLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin]) // escape mode
        {
          aboveLevel = maxSymbol - 1;
        }
      }
      assert(curLevel != aboveLevel);
      if (curLevel > aboveLevel)
      {
        curLevel--;
      }
    }
    maxSymbol--;
  }
  assert(maxSymbol > 0);
  assert(curLevel >= 0);
  assert(maxSymbol > curLevel);
  if (maxSymbol > 1)
  {
    xWriteTruncBinCode(curLevel, maxSymbol);
  }
  return curLevel;
}


//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu );
//    void  merge_flag      ( pu );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACWriter::prediction_unit( const PredictionUnit& pu )
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK( pu.cu->treeType == TREE_C, "cannot be chroma CU" );
#endif
#if ENABLE_SPLIT_PARALLELISM
  CHECK( pu.cacheUsed, "Processing a PU that should be in cache!" );
  CHECK( pu.cu->cacheUsed, "Processing a CU that should be in cache!" );

#endif
  if( pu.cu->skip )
  {
    CHECK( !pu.mergeFlag, "merge_flag must be true for skipped CUs" );
  }
  else
  {
    merge_flag( pu );
  }
#if !JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AA0070_RRIBC
  rribcData(*pu.cu);
#endif
#else
  if (pu.isBvpClusterApplicable())
  {
    bvOneZeroComp(*pu.cu);
  }
#if JVET_AA0070_RRIBC
  else
  {
    rribcData(*pu.cu);
  }
#endif
#endif
#if JVET_AC0112_IBC_LIC
  cuIbcLicFlag(*pu.cu);
#endif
  if( pu.mergeFlag )
  {
    merge_data(pu);
#if MULTI_HYP_PRED
    if( !pu.cu->skip && !CU::isIBC( *pu.cu ) )
    {
      CHECK(pu.numMergedAddHyps > pu.addHypData.size(), "wrong number of additional hypotheseis in mergemode");
      mh_pred_data(pu);
    }
    else
    {
      CHECK(pu.numMergedAddHyps != pu.addHypData.size(), "wrong number of additional hypotheseis in merge mode");
    }
#endif
  }
  else if (CU::isIBC(*pu.cu))
  {
#if JVET_AE0169_BIPREDICTIVE_IBC
    ibcBiPredictionFlag(pu);
#endif
#if JVET_AC0112_IBC_CIIP
    ibcCiipFlag(pu);
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraIdx(pu);
    }
#endif
    ref_idx(pu, REF_PIC_LIST_0);
    Mv mvd = pu.mvd[REF_PIC_LIST_0];
    mvd.changeIbcPrecInternal2Amvr(pu.cu->imv);
#if JVET_AA0070_RRIBC
#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(mvd, pu.isBvdPredApplicable(), pu.isBvpClusterApplicable(), pu.cu->bvOneZeroComp, pu.cu->bvZeroCompDir,
              pu.cu->rribcFlipType);   // already changed to signaling precision
#else
    bvdCoding(mvd, pu.isBvdPredApplicable(), pu.cu->rribcFlipType);   // already changed to signaling precision
#endif
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(mvd, pu.isBvpClusterApplicable(), pu.cu->bvOneZeroComp, pu.cu->bvZeroCompDir,
              pu.cu->rribcFlipType);   // already changed to signaling precision
#else
    bvdCoding(mvd, pu.cu->rribcFlipType);   // already changed to signaling precision
#endif
#endif

#else
    mvd_coding(mvd, 0, true, pu.cu->rribcFlipType); // already changed to signaling precision
#endif
#else
#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(mvd, pu.isBvdPredApplicable(), pu.isBvpClusterApplicable(), pu.cu->bvOneZeroComp,
              pu.cu->bvZeroCompDir);   // already changed to signaling precision
#else
    bvdCoding(mvd, pu.isBvdPredApplicable());   // already changed to signaling precision
#endif  
#else
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
    bvdCoding(mvd, pu.isBvpClusterApplicable(), pu.cu->bvOneZeroComp,
              pu.cu->bvZeroCompDir);   // already changed to signaling precision
#else
    bvdCoding(mvd);   // already changed to signaling precision
#endif
#endif
#else
    mvd_coding(mvd, 0); // already changed to signaling precision
#endif
#endif
    if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
    {
      CHECK( pu.mvpIdx[REF_PIC_LIST_0], "mvpIdx for IBC mode should be 0" );
    }
    else
    mvp_flag(pu, REF_PIC_LIST_0);
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
    {
      merge_idx(pu);
    }
#endif
  }
  else
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMerge_mode( pu );
    if (!(pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1]))
    {
#endif
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
#if JVET_AG0098_AMVP_WITH_SBTMVP
    amvpSbTmvpFlag(pu);
    if (!pu.amvpSbTmvpFlag)
    {
#endif
    smvd_mode( pu );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if(pu.cs->sps->getUseMvdPred())
    {
#endif
    cu_bcw_flag(*pu.cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    }
#endif
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
    refPairIdx(pu);
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
    } 
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
    refIdxLC(pu);
#endif
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (!pu.amvpMergeModeFlag[REF_PIC_LIST_0])
      {
#endif
      ref_idx     ( pu, REF_PIC_LIST_0 );
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_0 );
#endif
      if ( pu.cu->affine )
      {
        Mv mvd = pu.mvdAffi[REF_PIC_LIST_0][0];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
        {
          const auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0];
          mvd_coding(mvd, pu.cu->imv, &si, !pu.isMvdPredApplicable());
        }
#else
        mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
        mvd_coding(mvd, 0); // already changed to signaling precision
#endif
        mvd = pu.mvdAffi[REF_PIC_LIST_0][1];
        mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
        {
          const auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][1];
          mvd_coding(mvd, pu.cu->imv, &si, !pu.isMvdPredApplicable());
        }
#else
        mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
        mvd_coding(mvd, 0); // already changed to signaling precision
#endif
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
          mvd = pu.mvdAffi[REF_PIC_LIST_0][2];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          const auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][2];
          mvd_coding(mvd, pu.cu->imv, &si, !pu.isMvdPredApplicable());
#else
          mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
          mvd_coding(mvd, 0); // already changed to signaling precision
#endif
        }
      }
#if JVET_AG0098_AMVP_WITH_SBTMVP
      else if (pu.amvpSbTmvpFlag)
      {
        amvpSbTmvpMvdCoding(pu);
      }
#endif
      else
      {
        Mv mvd = pu.mvd[REF_PIC_LIST_0];
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        if (pu.amvpMergeModeFlag[REF_PIC_LIST_1] == true && pu.mvpIdx[REF_PIC_LIST_0] < 2)
        {
          CHECK(mvd.hor != 0, "this is not possible");
          CHECK(mvd.ver != 0, "this is not possible");
        }
        else
        {
#endif
        mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
        const auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0];
        mvd_coding(mvd, pu.cu->imv, &si, !pu.isMvdPredApplicable());
#else
        mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
        mvd_coding(mvd, 0); // already changed to signaling precision
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        }
#endif
      }
#if JVET_X0083_BM_AMVP_MERGE_MODE
      }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_0 );
#endif
    }
    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (!pu.amvpMergeModeFlag[REF_PIC_LIST_1])
      {
#endif
      ref_idx     ( pu, REF_PIC_LIST_1 );
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_1 );
#endif
      if( !pu.cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
      {
        if ( pu.cu->affine )
        {
          Mv mvd = pu.mvdAffi[REF_PIC_LIST_1][0];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          const auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][0];
          mvd_coding(mvd, pu.cu->imv, &si, !pu.isMvdPredApplicable());
#else
          mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
          mvd_coding(mvd, 0); // already changed to signaling precision
#endif
          mvd = pu.mvdAffi[REF_PIC_LIST_1][1];
          mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          const auto& si1 = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][1];
          mvd_coding(mvd, pu.cu->imv, &si1, !pu.isMvdPredApplicable());
#else
          mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
          mvd_coding(mvd, 0); // already changed to signaling precision
#endif
          if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
          {
            mvd = pu.mvdAffi[REF_PIC_LIST_1][2];
            mvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
            const auto& si2 = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][2];
            mvd_coding(mvd, pu.cu->imv, &si2, !pu.isMvdPredApplicable());
#else
            mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
            mvd_coding(mvd, 0); // already changed to signaling precision
#endif
          }
        }
#if JVET_AG0098_AMVP_WITH_SBTMVP
        else if (pu.amvpSbTmvpFlag)
        {
          amvpSbTmvpMvdCoding(pu);
        }
#endif
        else
        {
          Mv mvd = pu.mvd[REF_PIC_LIST_1];
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] == true && pu.mvpIdx[REF_PIC_LIST_1] < 2)
          {
            CHECK(mvd.hor != 0, "this is not possible");
            CHECK(mvd.ver != 0, "this is not possible");
          }
          else
          {
#endif
          mvd.changeTransPrecInternal2Amvr(pu.cu->imv);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AD0140_MVD_PREDICTION
          const auto& si = pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_1][0];
          mvd_coding(mvd, pu.cu->imv, &si, !pu.isMvdPredApplicable());
#else
          mvd_coding(mvd, 0, !pu.isMvdPredApplicable());
#endif
#else
          mvd_coding(mvd, 0); // already changed to signaling precision
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          }
#endif
        }
      }
#if JVET_X0083_BM_AMVP_MERGE_MODE
      }
#endif
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      else
      {
        CHECK( pu.refIdx[REF_PIC_LIST_1] != pu.cs->slice->getSymRefIdx(REF_PIC_LIST_1), "Wrong L1 reference index" );
        mvp_flag    ( pu, REF_PIC_LIST_1 );
      }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_1 );
#endif
    }
  }
}
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
void    CABACWriter::mvsd_data(const PredictionUnit&  pu)
{
  CHECK(pu.cu->slice->getSliceType() == I_SLICE && !CU::isIBC(*pu.cu), "cannot be I Slice");
#if !JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu))
  {
    return;
  }
#endif

  if (pu.cu->skip || pu.mergeFlag 
#if !JVET_AC0104_IBC_BVD_PREDICTION
    || CU::isIBC(*pu.cu)
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    || !pu.isMvdPredApplicable()
#endif
     )
  {
    return;
  }
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif

  if (pu.interDir != 2 /* PRED_L1 */)
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (pu.cu->affine)
    {
      mvsdAffineIdxFunc(pu, REF_PIC_LIST_0);
    }
    else
#endif
    {
      mvsdIdxFunc(pu, REF_PIC_LIST_0);
    }
  }

#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu))
  {
    return;
  }
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (pu.interDir != 1 /* PRED_L0 */ && pu.cu->smvdMode != 1)
  {
    if (pu.cu->affine)
    {
      mvsdAffineIdxFunc(pu, REF_PIC_LIST_1);
    }
    else
    {
      mvsdIdxFunc(pu, REF_PIC_LIST_1);
    }
  }
#endif

}
#endif
void CABACWriter::smvd_mode( const PredictionUnit& pu )
{
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }
  
  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }
  
  m_BinEncoder.encodeBin( pu.cu->smvdMode ? 1 : 0, Ctx::SmvdFlag() );
  
  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

#if JVET_AG0098_AMVP_WITH_SBTMVP
void CABACWriter::amvpSbTmvpFlag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getSbTMVPEnabledFlag())
  {
    return;
  }

  if (!pu.cs->slice->getAmvpSbTmvpEnabledFlag())
  {
    return;
  }

  if (pu.cu->affine)
  {
    return;
  }
  if (pu.interDir == 3)
  {
    return;
  }
  
  m_BinEncoder.encodeBin(pu.amvpSbTmvpFlag ? 1 : 0, Ctx::amvpSbTmvpFlag(0));
  
  DTRACE(g_trace_ctx, D_SYNTAX, "amvpSbTmvpFlag() amvpSbTmvpFlag=%d colIdx:%d pos=(%d,%d) size=%dx%d\n", pu.amvpSbTmvpFlag ? 1 : 0, pu.colIdx ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  if (pu.amvpSbTmvpFlag)
  {
    if (pu.cs->slice->getAmvpSbTmvpNumColPic() > 1)
    {
      m_BinEncoder.encodeBin((pu.interDir == 2), Ctx::amvpSbTmvpFlag(1));
    }
    if (isEncoding())
    {
      g_picAmvpSbTmvpEnabledArea += pu.lwidth() * pu.lheight();
    }
  }
}

void CABACWriter::amvpSbTmvpMvdCoding(const PredictionUnit &pu)
{
  if (pu.amvpSbTmvpMvdIdx < 0)
  {
    m_BinEncoder.encodeBin(1, Ctx::amvpSbTmvpMvdIdx(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "amvpSbTmvpMvdCoding() pos=(%d,%d) size=(%d,%d) amvpSbTmvpMvdIdx:%d\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), -1);
  }
  else
  {
    m_BinEncoder.encodeBin(0, Ctx::amvpSbTmvpMvdIdx(0));

    int numStepCandMinus1 = pu.cs->slice->getAmvpSbTmvpNumOffset() - 1;
    unsigned int amvpSbTmvpMvdIdx = (unsigned int)pu.amvpSbTmvpMvdIdx;
    m_BinEncoder.encodeBinsEP(amvpSbTmvpMvdIdx % (1 << 2), 2);
    amvpSbTmvpMvdIdx >>= 2;
    for (unsigned int uiUnaryIdx = 0; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
    {
      unsigned int uiSymbol = amvpSbTmvpMvdIdx == uiUnaryIdx ? 0 : 1;
      m_BinEncoder.encodeBin(uiSymbol, Ctx::amvpSbTmvpMvdIdx(uiUnaryIdx + 1));
      if (uiSymbol == 0)
      {
        break;
      }
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "amvpSbTmvpMvdCoding() pos=(%d,%d) size=(%d,%d) amvpSbTmvpMvdIdx:%d numStepCandMinus1:%d\n", pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), pu.amvpSbTmvpMvdIdx, numStepCandMinus1);
  }
}
#endif

void CABACWriter::subblock_merge_flag( const CodingUnit& cu )
{

  if ( !cu.cs->slice->isIntra() && (cu.slice->getPicHeader()->getMaxNumAffineMergeCand() > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::SubblockMergeFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACWriter::affine_flag( const CodingUnit& cu )
{
#if INTER_RM_SIZE_CONSTRAINTS
  if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8)
#else
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
#endif
  {
    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    m_BinEncoder.encodeBin( cu.affine, Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      unsigned ctxId = 0;
      m_BinEncoder.encodeBin( cu.affineType, Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
  }
}

#if AFFINE_MMVD
void CABACWriter::affine_mmvd_data(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseAffineMmvdMode() || !pu.mergeFlag || !pu.cu->affine)
  {
    return;
  }

  m_BinEncoder.encodeBin(pu.afMmvdFlag, Ctx::AfMmvdFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_flag() af_mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.afMmvdFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);

  if (!pu.afMmvdFlag)
  {
    return;
  }

  // Base affine merge candidate idx
  uint8_t afMmvdBaseIdx = pu.afMmvdBaseIdx;

  int numCandMinus1Base = AF_MMVD_BASE_NUM - 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  unsigned ctxId = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  static_assert(ECM3_AF_MMVD_BASE_NUM == 1, "The value of ECM3_AF_MMVD_BASE_NUM must be 1");
  if (pu.cs->sps->getUseTMMMVD())
#endif
  {
    const CodingStructure *cs = pu.cu->cs;
    const CodingUnit *cuLeft = cs->getCURestricted(pu.cu->lumaPos().offset(-1, 0), *pu.cu, CH_L);
    ctxId = (cuLeft && cuLeft->affine) ? 1 : 0;
    const CodingUnit *cuAbove = cs->getCURestricted(pu.cu->lumaPos().offset(0, -1), *pu.cu, CH_L);
    ctxId += (cuAbove && cuAbove->affine) ? 1 : 0;
  }
  numCandMinus1Base = (ctxId == 0) ? 0 : ((ctxId == 1) ? 1 : AF_MMVD_BASE_NUM-1);
#endif
  if (numCandMinus1Base > 0)
  {
    // to support more base candidates
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    int ctx2 =  (numCandMinus1Base == 1) ? 1 : 0;
    m_BinEncoder.encodeBin((afMmvdBaseIdx == 0 ? 0 : 1), Ctx::AfMmvdIdx(ctx2));
#else
    m_BinEncoder.encodeBin((afMmvdBaseIdx == 0 ? 0 : 1), Ctx::AfMmvdIdx());
#endif
    
    if (afMmvdBaseIdx > 0)
    {
      for (unsigned idx = 1; idx < numCandMinus1Base; idx++)
      {
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
        m_BinEncoder.encodeBin(afMmvdBaseIdx == idx ? 0 : 1, Ctx::AfMmvdIdx(idx + 1));
#else
        m_BinEncoder.encodeBinEP(afMmvdBaseIdx == idx ? 0 : 1);
#endif
        if (afMmvdBaseIdx == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_base_idx() af_mmvd_base_idx=%d\n", afMmvdBaseIdx);

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if(pu.cs->sps->getUseTMMMVD())
  {
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uint16_t sym = pu.afMmvdMergeIdx;
#else
  uint8_t sym = pu.afMmvdMergeIdx;
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  sym -= afMmvdBaseIdx * AF_MMVD_MAX_REFINE_NUM;
#endif
  unsigned int ricePar = 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numStepCandMinus1 =  ((AF_MMVD_MAX_REFINE_NUM >> ricePar) >> AFFINE_MMVD_SIZE_SHIFT) / AFFINE_BI_DIR - 1;
#else
  int numStepCandMinus1 =  ((AF_MMVD_MAX_REFINE_NUM >> ricePar) >> AFFINE_MMVD_SIZE_SHIFT) - 1;
#endif
  if(ricePar > 0)
  {
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    m_BinEncoder.encodeBin( sym % (1 << ricePar), Ctx::AfMmvdOffsetStep(5));
#else
    m_BinEncoder.encodeBinsEP( sym % (1 << ricePar), ricePar);
#endif
  }
  sym >>= ricePar;
  for (unsigned int uiUnaryIdx = 0; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
  {
    unsigned int uiSymbol = sym == uiUnaryIdx ? 0 : 1;
    m_BinEncoder.encodeBin(uiSymbol, Ctx::AfMmvdOffsetStep((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx)));
    if (uiSymbol == 0)
    {
      break;
    }
  }
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    return;
  }
#endif
#endif

#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
  {
    // Code Step Value
    uint8_t step = pu.afMmvdStep;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    int numCandMinus1Base = ECM3_AF_MMVD_STEP_NUM - 1;
#else
    int numCandMinus1Base = AF_MMVD_STEP_NUM - 1;
#endif
    if (numCandMinus1Base > 0)
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      m_BinEncoder.encodeBin((step == 0 ? 0 : 1), Ctx::AfMmvdOffsetStepECM3());
#else
      m_BinEncoder.encodeBin((step == 0 ? 0 : 1), Ctx::AfMmvdOffsetStep());
#endif

      if (step > 0)
      {
        for (unsigned idx = 1; idx < numCandMinus1Base; idx++)
        {
          m_BinEncoder.encodeBinEP(step == idx ? 0 : 1);
          if (step == idx)
          {
            break;
          }
        }
      }
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_offset_step() af_mmvd_offset_step=%d\n", pu.afMmvdStep);
  }

  {
    // Code Dir Value
    uint8_t offsetDir = pu.afMmvdDir;
    uint8_t b0 = offsetDir & 0x1;
    uint8_t b1 = (offsetDir >> 1) & 0x1;
    m_BinEncoder.encodeBinEP(b0);
    m_BinEncoder.encodeBinEP(b1);
    DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_offset_dir() af_mmvd_offset_dir=%d\n", pu.afMmvdDir);
  }
#endif
}
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
void CABACWriter::ibcBiPredictionFlag( const PredictionUnit& pu )
{
  if (!pu.cs->slice->getBiPredictionIBCFlag())
  {
    return;
  }
  if (!pu.mergeFlag && (pu.cu->ibcLicFlag || pu.cu->rribcFlipType))
  {
    return;
  }
  m_BinEncoder.encodeBin(pu.interDir == 3 ? 1 : 0, Ctx::BiPredIbcFlag(pu.mergeFlag ? 0 : 1));

  DTRACE( g_trace_ctx, D_SYNTAX, "ibc_bi_prediction_flag() inter_dir=%d\n", pu.interDir );
}

void CABACWriter::ibcMergeIdx1( const PredictionUnit& pu )
{
  if (pu.interDir != 3)
  {
    return;
  }

  int numCandminus2 = int(pu.cs->sps->getMaxNumIBCMergeCand()) - pu.mergeIdx - 2;
  if( numCandminus2 > 0 )
  {
    CHECK(pu.ibcMergeIdx1 <= pu.mergeIdx, "pu.ibcMergeIdx1 <= pu.mergeIdx");
    int idx1 = pu.ibcMergeIdx1 - pu.mergeIdx - 1;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    const CtxSet mrgIdxCtxSet = pu.tmMergeFlag ? Ctx::TmMergeIdx : Ctx::MergeIdx;
#endif
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus2; ++uiUnaryIdx)
    {
      unsigned int uiSymbol = idx1 == uiUnaryIdx ? 0 : 1;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));
#else
      m_BinEncoder.encodeBin(uiSymbol, Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));
#endif
      if (uiSymbol == 0)
      {
        break;
      }
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "ibc_merge_idx1() ibc_merge_idx1=%d\n", pu.ibcMergeIdx1 );
}
#endif

#if JVET_AA0061_IBC_MBVD
void CABACWriter::ibcMbvdData(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcMbvd() || !pu.mergeFlag || !CU::isIBC(*pu.cu))
  {
    return;
  }
  m_BinEncoder.encodeBin(pu.ibcMbvdMergeFlag, Ctx::IbcMbvdFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() ibc_mbvd_flag=%d pos=(%d,%d) size=%dx%d\n", pu.ibcMbvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);

  if (!pu.ibcMbvdMergeFlag)
  {
    return;
  }

#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  const int mbvdsPerBase = pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch() ? IBC_MBVD_SIZE_ENC : IBC_MBVD_MAX_REFINE_NUM;
#else
  const int mbvdsPerBase = IBC_MBVD_MAX_REFINE_NUM;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.interDir == 3)
  {
    m_BinEncoder.encodeBin(pu.ibcMergeIdx1 < IBC_MRG_MAX_NUM_CANDS ? 0 : 1, Ctx::IbcMbvdFlag(1));
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() bi_mbvd_mode=%d\n", pu.ibcMergeIdx1 < IBC_MRG_MAX_NUM_CANDS ? 1 : 2);
  }
#endif
  int mvpIdx = pu.ibcMbvdMergeIdx;
  uint8_t var0;
  var0 = mvpIdx / mbvdsPerBase;
  mvpIdx -= var0 * mbvdsPerBase;

  // Base affine merge candidate idx

#if JVET_AE0169_BIPREDICTIVE_IBC
  int numBaseCandMinus1 = std::min(int(pu.cs->sps->getMaxNumIBCMergeCand()) - 1, IBC_MBVD_BASE_NUM - 1);
#else
  int numBaseCandMinus1 = IBC_MBVD_BASE_NUM - 1;
#endif
  if (numBaseCandMinus1 > 0)
  {
    // to support more base candidates
    m_BinEncoder.encodeBin((var0 == 0 ? 0 : 1), Ctx::IbcMbvdMergeIdx());

    if (var0 > 0)
    {
      for (unsigned idx = 1; idx < numBaseCandMinus1; idx++)
      {
        m_BinEncoder.encodeBinEP(var0 == idx ? 0 : 1);
        if (var0 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() base_idx=%d\n", var0);

#if JVET_AE0169_BIPREDICTIVE_IBC
  int ibcMbvdSizeEnc = IBC_MBVD_SIZE_ENC;
  uint8_t var1 = 0;
  int mvpIdx1 = 0;
  if (pu.interDir == 3 && pu.ibcMergeIdx1 >= IBC_MRG_MAX_NUM_CANDS)
  {
    mvpIdx1 = pu.ibcMergeIdx1 - IBC_MRG_MAX_NUM_CANDS;
    var1 = mvpIdx1 / mbvdsPerBase;
    mvpIdx1 -= var1 * mbvdsPerBase;
    CHECK(var1 < var0, "var1 < var0");
    if (numBaseCandMinus1 > 0 && var0 < numBaseCandMinus1)
    {
      // to support more base candidates
      m_BinEncoder.encodeBinEP(var0 == var1 ? 0 : 1);

      if (var1 > var0)
      {
        for (unsigned idx = var0+1; idx < numBaseCandMinus1; idx++)
        {
          if (idx == var0+1)
          {
            m_BinEncoder.encodeBin(var1 == idx ? 0 : 1, Ctx::IbcMbvdMergeIdx());
          }
          else
          {
            m_BinEncoder.encodeBinEP(var1 == idx ? 0 : 1);
          }
          if (var1 == idx)
          {
            break;
          }
        }
      }
    }
    if (var1 == var0)
    {
      ibcMbvdSizeEnc--;
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() base_idx1=%d\n", var1);
  }

  unsigned int ricePar = 1;
  unsigned int riceParVal = 1<<ricePar;
  int mvpIdxby2 = mvpIdx >> ricePar;
  int remain = ibcMbvdSizeEnc;
  for (unsigned int uiUnaryIdx = 0; remain > riceParVal; ++uiUnaryIdx, remain-=riceParVal)
  {
    unsigned int uiSymbol = mvpIdxby2 == uiUnaryIdx ? 0 : 1;
    m_BinEncoder.encodeBin(uiSymbol, Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx)));
    if (uiSymbol == 0)
    {
      break;
    }
  }
  int length = remain >= riceParVal ? ricePar : ceilLog2(remain);
  if (length > 0)
  {
    m_BinEncoder.encodeBinsEP( mvpIdx % riceParVal, length);
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() ibc_merge_idx=%d\n", pu.ibcMbvdMergeIdx);

  if (pu.interDir == 3 && pu.ibcMergeIdx1 >= IBC_MRG_MAX_NUM_CANDS)
  {
    if (var0 == var1)
    {
      CHECK(mvpIdx1 <= mvpIdx, "mvpIdx1 <= mvpIdx");
      ibcMbvdSizeEnc = IBC_MBVD_SIZE_ENC - (mvpIdx+1);
      mvpIdx1 -= (mvpIdx+1);
    }
    int mvpIdx1by2 = mvpIdx1 >> ricePar;
    int remain1 = ibcMbvdSizeEnc;
    for (unsigned int uiUnaryIdx = 0; remain1 > riceParVal; ++uiUnaryIdx, remain1-=riceParVal)
    {
      unsigned int uiSymbol = mvpIdx1by2 == uiUnaryIdx ? 0 : 1;
      m_BinEncoder.encodeBin(uiSymbol, Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx)));
      if (uiSymbol == 0)
      {
        break;
      }
    }
    int length1 = remain1 >= riceParVal ? ricePar : ceilLog2(remain1);
    if (length1 > 0)
    {
      m_BinEncoder.encodeBinsEP( mvpIdx1 % riceParVal, length1);
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() ibc_merge_idx1=%d\n", pu.ibcMergeIdx1 - IBC_MRG_MAX_NUM_CANDS);
  }
#else
  unsigned int ricePar = 1;
  int numCandStepMinus1 = (IBC_MBVD_SIZE_ENC >> ricePar) - 1;
  if(ricePar > 0)
  {
    m_BinEncoder.encodeBinsEP( mvpIdx % (1 << ricePar), ricePar);
  }
  mvpIdx >>= ricePar;
  for (unsigned int uiUnaryIdx = 0; uiUnaryIdx < numCandStepMinus1; ++uiUnaryIdx)
  {
    unsigned int uiSymbol = mvpIdx == uiUnaryIdx ? 0 : 1;
    m_BinEncoder.encodeBin(uiSymbol, Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx)));
    if (uiSymbol == 0)
    {
      break;
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_mbvd_data() merge_idx=%d\n", pu.ibcMbvdMergeIdx);
#endif
}
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
void CABACWriter::tm_merge_flag(const PredictionUnit& pu)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (CU::isIBC(*pu.cu) && !pu.cs->sps->getUseTMIbc())
  {
    return;
  }
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if (pu.ciipFlag)
  {
    if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
      m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::CiipTMMergeFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
    }
    return;
  }
#endif

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG
  if (pu.cu->geoFlag)
  {
    if (!pu.cs->slice->getSPS()->getUseGPMTMMode())
    {
      return;
    }
  }
  else
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG
  if (pu.regularMergeFlag
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    && !CU::isIBC(*pu.cu)
#endif
    )
  {
    if (!pu.cs->slice->getSPS()->getUseTMMrgMode())
    {
      return;
    }
  }
  else
#endif
  if (!pu.cs->slice->getSPS()->getUseDMVDMode())
  {
#if (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_X0141_CIIP_TIMD_TM && TM_MRG) && (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG) && (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG)
#if !(JVET_Z0084_IBC_TM && IBC_TM_MRG)
    CHECK(true, "Unknown mode to code tm_merge_flag");
#endif
#endif
    return;
  }

#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_X0049_ADAPT_DMVR
  m_BinEncoder.encodeBin(pu.tmMergeFlag || pu.bmMergeFlag, Ctx::TMMergeFlag(CU::isIBC(*pu.cu) ? 1 : 0));
#else
  m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::TMMergeFlag(CU::isIBC(*pu.cu) ? 1 : 0));
#endif
#else
#if JVET_X0049_ADAPT_DMVR
  m_BinEncoder.encodeBin(pu.tmMergeFlag || pu.bmMergeFlag, Ctx::TMMergeFlag());
#else
  m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::TMMergeFlag());
#endif
#endif

#if JVET_X0049_ADAPT_DMVR
  DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() tm_merge_flag || bm_merge_flag=%d\n", pu.tmMergeFlag || pu.bmMergeFlag);
#else
  DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() tm_merge_flag=%d\n", pu.tmMergeFlag ? 1 : 0);
#endif
}
#endif

#if JVET_AC0112_IBC_CIIP
void CABACWriter::ibcCiipFlag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcCiip() || (pu.lx() == 0 && pu.ly() == 0))
  {
    return;
  }
  if (pu.lwidth() * pu.lheight() < 32 || pu.lwidth() > 32 || pu.lheight() > 32)
  {
    return;
  }
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.interDir == 3)
  {
    return;
  }
#endif
  if (pu.mergeFlag)
  {
    if (pu.cu->skip)
    {
      return;
    }
    m_BinEncoder.encodeBin(pu.ibcCiipFlag, Ctx::IbcCiipFlag(0));
  }
  else
  {
  #if JVET_AA0070_RRIBC
    if (pu.cu->rribcFlipType)
    {
      return;
    }
  #endif
  #if JVET_AC0112_IBC_LIC
    if (pu.cu->ibcLicFlag)
    {
      return;
    }
  #endif
    if (pu.cs->slice->getSliceType() != I_SLICE)
    {
      return;
    }
    m_BinEncoder.encodeBin(pu.ibcCiipFlag, Ctx::IbcCiipFlag(1));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_ciip_flag() ibc_ciip_flag=%d\n", pu.ibcCiipFlag);
}

void CABACWriter::ibcCiipIntraIdx(const PredictionUnit& pu)
{
  m_BinEncoder.encodeBin( pu.ibcCiipIntraIdx > 0, Ctx::IbcCiipIntraIdx() );
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_ciip_intra_idx() ibc_ciip_intra_idx=%d\n", pu.ibcCiipIntraIdx);
}
#endif

#if JVET_AC0112_IBC_GPM
void CABACWriter::ibcGpmFlag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcGpm() || (pu.lx() == 0 && pu.ly() == 0))
  {
    return;
  }
  if (pu.lwidth() < 8 || pu.lheight() < 8 || pu.lwidth() > 32 || pu.lheight() > 32)
  {
    return;
  }
  m_BinEncoder.encodeBin(pu.ibcGpmFlag, Ctx::IbcGpmFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_gpm_flag() ibc_gpm_flag=%d\n", pu.ibcGpmFlag);
}

void CABACWriter::ibcGpmMergeIdx(const PredictionUnit& pu)
{
  uint8_t splitDir = pu.ibcGpmSplitDir;
  uint8_t candIdx0 = pu.ibcGpmMergeIdx0;
  uint8_t candIdx1 = pu.ibcGpmMergeIdx1;

  uint8_t splitDirIdx = 0;
  if (g_geoParams[splitDir][0] % 8 == 0)
  {
    m_BinEncoder.encodeBin( 1, Ctx::IbcGpmSplitDirSetFlag() );
    splitDirIdx = g_ibcGpmFirstSetSplitDirToIdx[splitDir];
    m_BinEncoder.encodeBinsEP(splitDirIdx, 3);
  }
  else
  {
    m_BinEncoder.encodeBin( 0, Ctx::IbcGpmSplitDirSetFlag() );
    uint8_t prefix = 0;
    for (uint8_t i = 0; i < splitDir; i++)
    {
      if (!g_ibcGpmSecondSetSplitDir[i])
      {
        prefix++;
      }
    }
    splitDirIdx = splitDir - prefix;
    xWriteTruncBinCode(splitDirIdx, IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM);
  }

  bool isIntra0 = (pu.ibcGpmMergeIdx0 >= IBC_GPM_MAX_NUM_UNI_CANDS);
  bool isIntra1 = (pu.ibcGpmMergeIdx1 >= IBC_GPM_MAX_NUM_UNI_CANDS);
  m_BinEncoder.encodeBin( isIntra0 ? 1 : 0, Ctx::IbcGpmIntraFlag() );
#if JVET_AE0169_GPM_IBC_IBC
  if (!isIntra0 && pu.cs->sps->getMaxNumIBCMergeCand() > 1)
  {
    m_BinEncoder.encodeBin(isIntra1 ? 1 : 0, Ctx::IbcGpmIntraFlag(1));
  }
#endif

  const int maxNumIbcGpmCand = pu.cs->sps->getMaxNumIBCMergeCand();
  int numCandminus2 = maxNumIbcGpmCand - 2;
  if (isIntra0)
  {
    unary_max_eqprob(candIdx0 - IBC_GPM_MAX_NUM_UNI_CANDS, IBC_GPM_MAX_NUM_INTRA_CANDS-1);
  }
  else if (numCandminus2 >= 0)
  {
    m_BinEncoder.encodeBin(candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx());
    if (candIdx0 > 0)
    {
      unary_max_eqprob(candIdx0 - 1, numCandminus2);
    }
  }
  if (isIntra1)
  {
    unary_max_eqprob(candIdx1 - IBC_GPM_MAX_NUM_UNI_CANDS, IBC_GPM_MAX_NUM_INTRA_CANDS-1);
  }
#if JVET_AE0169_GPM_IBC_IBC
  else
  {
    if (isIntra0)
    {
      if (numCandminus2 >= 0)
      {
        m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
        if (candIdx1 > 0)
        {
          unary_max_eqprob(candIdx1 - 1, numCandminus2);
        }
      }
    }
    else
    {
      candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
      if (numCandminus2 > 0)
      {
        m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
        if (candIdx1 > 0)
        {
          unary_max_eqprob(candIdx1 - 1, numCandminus2 - 1);
        }
      }
    }
  }
#else
  else if (numCandminus2 >= 0)
  {
    m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
    if (candIdx1 > 0)
    {
      unary_max_eqprob(candIdx1 - 1, numCandminus2);
    }
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_gpm_merge_idx() ibc_gpm_splt_dir=%d merge_idx0=%d merge_idx1=%d\n", pu.ibcGpmSplitDir, pu.ibcGpmMergeIdx0, pu.ibcGpmMergeIdx1);
}

void CABACWriter::ibcGpmAdaptBlendIdx(const int idx)
{
  if (IBC_GPM_NUM_BLENDING == 1)
  {
    return;
  }
  if (idx == 0)
  {
    m_BinEncoder.encodeBin(1, Ctx::IbcGpmBldIdx(0));
  }
  else
  {
    m_BinEncoder.encodeBin(0, Ctx::IbcGpmBldIdx(0));
    if (idx == 2 || idx == 1)
    {
      m_BinEncoder.encodeBin(1, Ctx::IbcGpmBldIdx(1));
      m_BinEncoder.encodeBin(idx == 2, Ctx::IbcGpmBldIdx(2));
    }
    else
    {
      m_BinEncoder.encodeBin(0, Ctx::IbcGpmBldIdx(1));
      m_BinEncoder.encodeBin(idx == 3, Ctx::IbcGpmBldIdx(3));
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibc_gpm_adapt_blend_idx() ibc_gpm_bld_idx=%d\n", idx);
}
#endif

#if JVET_AC0112_IBC_LIC
void CABACWriter::cuIbcLicFlag(const CodingUnit& cu)
{
#if JVET_AE0159_FIBC
  if (!(cu.cs->sps->getUseIbcLic() || cu.cs->sps->getUseIbcFilter() ) || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
#else
  if (!cu.cs->sps->getUseIbcLic() || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
#endif
  {
    return;
  }
#if JVET_AA0070_RRIBC
  if (cu.rribcFlipType > 0)
  {
    return;
  }
#endif
#if JVET_AE0159_FIBC || JVET_AE0078_IBC_LIC_EXTENSION
  if (cu.lwidth() * cu.lheight() < 32 )
#else
  if (cu.lwidth() * cu.lheight() < 32 || cu.lwidth() * cu.lheight() > 256)
#endif
  {
    return;
  }
#if JVET_AE0159_FIBC
  if (cu.ibcFilterFlag)
  {
    CHECK(!cu.ibcLicFlag, "LIC flag has to be 1 when FIBC is 1");
  }
  if (cu.lx() < FIBC_TEMPLATE_SIZE && cu.ly() < FIBC_TEMPLATE_SIZE)
  {
    CHECK(cu.ibcFilterFlag, "FIBC has to be 0 when not enough template");
  }
  if ((cu.lx() >= FIBC_TEMPLATE_SIZE || cu.ly() >= FIBC_TEMPLATE_SIZE) && (cu.cs->slice->getSliceType() == I_SLICE) && cu.cs->sps->getUseIbcFilter() )
  {
    unsigned ctxIdx = 1 + DeriveCtx::ctxIbcFilterFlag(cu);
    m_BinEncoder.encodeBin(cu.ibcFilterFlag ? 1 : 0, Ctx::IbcLicFlag(ctxIdx));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() filter_flag=%d\n", cu.ibcFilterFlag);
  }
#if JVET_AE0078_IBC_LIC_EXTENSION
  if (!cu.ibcFilterFlag && cu.cs->sps->getUseIbcLic())
#else
  if (!cu.ibcFilterFlag && cu.cs->sps->getUseIbcLic() && (cu.lwidth() * cu.lheight() <= 256))
#endif
  {
    m_BinEncoder.encodeBin(cu.ibcLicFlag ? 1 : 0, Ctx::IbcLicFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_flag=%d\n", cu.ibcLicFlag);
#if JVET_AE0078_IBC_LIC_EXTENSION
    if (cu.ibcLicFlag)
    {
      const int bin1 = (cu.ibcLicIdx == IBC_LIC_IDX) || (cu.ibcLicIdx == IBC_LIC_IDX_M) ? 0 : 1;
      const int bin2 = (cu.ibcLicIdx == IBC_LIC_IDX) || (cu.ibcLicIdx == IBC_LIC_IDX_T) ? 0 : 1;
      m_BinEncoder.encodeBin(bin1, Ctx::IbcLicIndex(0));
      m_BinEncoder.encodeBin(bin2, Ctx::IbcLicIndex(1));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_idx=%d\n", cu.ibcLicIdx);
    }
#endif
  }
#else
  m_BinEncoder.encodeBin(cu.ibcLicFlag ? 1 : 0, Ctx::IbcLicFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_flag=%d\n", cu.ibcLicFlag);
#if JVET_AE0078_IBC_LIC_EXTENSION
  if (cu.ibcLicFlag)
  {
    const int bin1 = (cu.ibcLicIdx == IBC_LIC_IDX) || (cu.ibcLicIdx == IBC_LIC_IDX_M) ? 0 : 1;
    const int bin2 = (cu.ibcLicIdx == IBC_LIC_IDX) || (cu.ibcLicIdx == IBC_LIC_IDX_T) ? 0 : 1;
    m_BinEncoder.encodeBin(bin1, Ctx::IbcLicIndex(0));
    m_BinEncoder.encodeBin(bin2, Ctx::IbcLicIndex(1));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_ibc_lic_flag() lic_idx=%d\n", cu.ibcLicIdx);
  }
#endif
#endif
}
#endif

#if JVET_X0049_ADAPT_DMVR
void CABACWriter::bm_merge_flag(const PredictionUnit& pu)
{
  if (!PU::isBMMergeFlagCoded(pu))
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxBMMrgFlag(*pu.cu);
  m_BinEncoder.encodeBin(pu.bmMergeFlag, Ctx::BMMergeFlag(ctxId));
  if (pu.bmMergeFlag)
  {
    CHECK(pu.bmDir != 1 && pu.bmDir != 2, "pu.bmDir != 1 && pu.bmDir != 2");
    m_BinEncoder.encodeBin(pu.bmDir >> 1, Ctx::BMMergeFlag(3));
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "bm_merge_flag() bm_merge_flag=%d, bmDir = %d\n", pu.bmMergeFlag ? 1 : 0, pu.bmDir);
}
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
void CABACWriter::affBmFlag(const PredictionUnit& pu)
{

  if (!PU::isAffBMMergeFlagCoded(pu))
  {
    CHECK(pu.affBMMergeFlag != false, "");
    return;
  }
  m_BinEncoder.encodeBin(pu.affBMMergeFlag, Ctx::affBMFlag(0));
  DTRACE(g_trace_ctx, D_SYNTAX, "aff_bm_flag() aff_bm_flag=%d\n", pu.affBMMergeFlag);
  if (pu.affBMMergeFlag)
  {
    CHECK(pu.affBMDir != 1 && pu.affBMDir != 2, "pu.affBMDir != 1 && pu.affBMDir != 2");
    m_BinEncoder.encodeBin(pu.affBMDir >> 1, Ctx::affBMFlag(1));
    DTRACE(g_trace_ctx, D_SYNTAX, "aff_bm_flag() aff_bm_dir=%d\n", pu.affBMDir);
  }
}
#endif
void CABACWriter::merge_flag( const PredictionUnit& pu )
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if( CU::isIBC( *pu.cu ) && !pu.cu->slice->getSPS()->getUseIbcMerge() )
  {
    CHECK( pu.mergeFlag, "IBC merge flag shall be disabled" );
    return;
  }
#endif
  m_BinEncoder.encodeBin( pu.mergeFlag, Ctx::MergeFlag() );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

}

void CABACWriter::merge_data(const PredictionUnit& pu)
{
  if (CU::isIBC(*pu.cu))
  {
#if JVET_AE0169_BIPREDICTIVE_IBC
    ibcBiPredictionFlag(pu);
#endif
#if JVET_AA0061_IBC_MBVD
    ibcMbvdData(pu);
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag)
    {
#endif
      tm_merge_flag(pu);
#if JVET_AA0061_IBC_MBVD
    }
#endif
#endif
#if JVET_AC0112_IBC_CIIP
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir != 3)
#endif
    ibcCiipFlag(pu);
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraIdx(pu);
    }
#endif
#if JVET_AC0112_IBC_GPM
#if JVET_AC0112_IBC_CIIP && JVET_AA0061_IBC_MBVD
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (!pu.ibcMbvdMergeFlag && !pu.ibcCiipFlag && pu.interDir != 3)
#else
    if (!pu.ibcMbvdMergeFlag && !pu.ibcCiipFlag)
#endif
#else
#if JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag)
#else
#if JVET_AC0112_IBC_CIIP
    if (!pu.ibcCiipFlag)
#endif
#endif
#endif
    {
      ibcGpmFlag(pu);
      if (pu.ibcGpmFlag)
      {
        ibcGpmMergeIdx(pu);
#if JVET_AE0169_GPM_IBC_IBC
        if (pu.cs->slice->getSliceType() == I_SLICE)
        {
          ibcGpmAdaptBlendIdx(pu.ibcGpmBldIdx);
        }
#else
        ibcGpmAdaptBlendIdx(pu.ibcGpmBldIdx);
#endif
      }
    }
#endif
    merge_idx(pu);
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (pu.interDir == 3 && !pu.ibcMbvdMergeFlag)
    {
      ibcMergeIdx1(pu);
    }
#endif
    return;
  }
#if JVET_AG0135_AFFINE_CIIP
  if (pu.ciipAffine)
  {
    pu.cu->affine = false;
  }
#endif
  subblock_merge_flag(*pu.cu);
  if (pu.cu->affine)
  {
#if AFFINE_MMVD
    affine_mmvd_data(pu);
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    if (!pu.afMmvdFlag)
    {
      affBmFlag(pu);
    }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (PU::hasOppositeLICFlag(pu) && !pu.afMmvdFlag && !pu.affBMMergeFlag && pu.cs->sps->getUseAffMergeOppositeLic())
    {
      m_BinEncoder.encodeBin(pu.affineOppositeLic, Ctx::AffineFlagOppositeLic(0));
    }
#endif
    merge_idx(pu);
    return;
  }
#if JVET_AG0135_AFFINE_CIIP
  if (pu.ciipAffine)
  {
    pu.cu->affine = true;
  }
#endif
#if CIIP_RM_BLOCK_SIZE_CONSTRAINTS
#if CTU_256
  const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );

  const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() * pu.cu->lheight() >= 32 && pu.cu->lwidth() <= maxSize && pu.cu->lheight() <= maxSize;
#else
  const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() * pu.cu->lheight() >= 32;
#endif
#else
  const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() < MAX_CU_SIZE && pu.cu->lheight() < MAX_CU_SIZE && pu.cu->lwidth() * pu.cu->lheight() >= 64;
#endif

#if JVET_Y0065_GPM_INTRA
  const bool geoAvailable = pu.cu->cs->slice->getSPS()->getUseGeo() && !pu.cu->cs->slice->isIntra() &&
    pu.cs->sps->getMaxNumGeoCand() > 0
#else
  const bool geoAvailable = pu.cu->cs->slice->getSPS()->getUseGeo() && pu.cu->cs->slice->isInterB() &&
    pu.cs->sps->getMaxNumGeoCand() > 1
#endif
                                                                    && pu.cu->lwidth() >= GEO_MIN_CU_SIZE && pu.cu->lheight() >= GEO_MIN_CU_SIZE
                                                                    && pu.cu->lwidth() <= GEO_MAX_CU_SIZE && pu.cu->lheight() <= GEO_MAX_CU_SIZE
                                                                    && pu.cu->lwidth() < 8 * pu.cu->lheight() && pu.cu->lheight() < 8 * pu.cu->lwidth();
  if (geoAvailable || ciipAvailable)
  {
    m_BinEncoder.encodeBin(pu.regularMergeFlag, Ctx::RegularMergeFlag(pu.cu->skip ? 0 : 1));
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() regular_merge=%d pos=(%d,%d) size=%dx%d\n", pu.regularMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  }
  if (pu.regularMergeFlag)
  {
#if TM_MRG
    tm_merge_flag(pu);
#endif
#if JVET_X0049_ADAPT_DMVR 
#if TM_MRG
    if ((pu.tmMergeFlag || pu.bmMergeFlag)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      || !pu.cs->slice->getSPS()->getUseTMMrgMode()
#endif
      )
#endif
    {
       bm_merge_flag(pu);
    }
#endif
    if (pu.cs->sps->getUseMMVD()
#if TM_MRG
      && !pu.tmMergeFlag
#endif
#if JVET_X0049_ADAPT_DMVR
      && !pu.bmMergeFlag
#endif
      )
    {
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
      unsigned  ctxId = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                        !pu.cs->sps->getUseTMMMVD() ||
#endif
                        pu.cu->skip ? 0 : 1;
      m_BinEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::MmvdFlag(ctxId));
#else
      m_BinEncoder.encodeBin(pu.mmvdMergeFlag, Ctx::MmvdFlag(0));
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
    }
    if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
    {
      mmvd_merge_idx(pu);
    }
    else
    {
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (pu.regularMergeFlag && PU::hasOppositeLICFlag(pu) && !pu.bmMergeFlag)
      {
        if (pu.tmMergeFlag && pu.cs->sps->getUseTMMergeOppositeLic())
        {
          m_BinEncoder.encodeBin(pu.tmMergeFlagOppositeLic, Ctx::TmMergeFlagOppositeLic(0));
        }
        else if (pu.cs->sps->getUseMergeOppositeLic())
        {
          m_BinEncoder.encodeBin(pu.mergeOppositeLic, Ctx::MergeFlagOppositeLic(0));
        }
      }
#endif
      merge_idx(pu);
    }
  }
  else
  {
    if (geoAvailable && ciipAvailable)
    {
      Ciip_flag(pu);
    }

#if CIIP_PDPC
    if (pu.ciipFlag && !geoAvailable && ciipAvailable)
    {
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AG0135_AFFINE_CIIP
        ciipAffineFlag(pu);
        if (!pu.ciipAffine)
        {
#endif
          tm_merge_flag(pu);
#if JVET_AG0135_AFFINE_CIIP
        }
#endif
#else
      if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
      {
        m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::CiipTMMergeFlag());
        DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
      }
#endif
#endif
#if JVET_AG0135_AFFINE_CIIP
      if (!pu.ciipAffine)
      {
#endif
        m_BinEncoder.encodeBin(pu.ciipPDPC, Ctx::CiipFlag(1));
#if JVET_AG0135_AFFINE_CIIP
      }
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_data() ciip_pdpc_flag=%d\n", pu.ciipPDPC);
    }
#else
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
    if (pu.ciipFlag && !geoAvailable && ciipAvailable && pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      tm_merge_flag(pu);
#else
      m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::CiipTMMergeFlag());
#endif
    }
#endif
#endif
    merge_idx(pu);
  }
}

void CABACWriter::imv_mode( const CodingUnit& cu )
{
  const SPS *sps = cu.cs->sps;

  if( !sps->getAMVREnabledFlag() )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (CU::isIBC(cu) && !cu.firstPU->mergeFlag)
    {
      CHECK(cu.imv != 1, "Error on default IMV flag of IBC AMVP mode")
    }
#endif
    return;
  }
  if ( cu.affine )
  {
    return;
  }

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag && !cu.cs->slice->getAmvpSbTmvpAmvrEnabledFlag())
  {
    return;
  }
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
  auto &pu = *cu.firstPU;
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (!CU::isIBC(cu) && (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1]))
#else
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
#endif
  {
    return;
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool useIBCFrac = CU::isIBC(cu) && !cu.firstPU->mergeFlag && cu.cs->sps->getIBCFracFlag()
#if JVET_AA0070_RRIBC
                 && (cu.rribcFlipType == 0)
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                 && (cu.bvOneZeroComp == 0)
#endif
    ;
#endif
  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (CU::isIBC(cu) && !cu.firstPU->mergeFlag)
    {
      CHECK(cu.imv != (useIBCFrac ? IBC_SUBPEL_AMVR_MODE_FOR_ZERO_MVD : 1), "Error on default IMV flag of IBC AMVP mode")
    }
#endif
    return;
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const CtxSet& imvCtx = CU::isIBC(cu) && pu.cs->sps->getIBCFracFlag() ? Ctx::ImvFlagIBC : Ctx::ImvFlag;
  if (CU::isIBC(cu) && pu.cs->sps->getIBCFracFlag())
  {
    CHECK(cu.imv == IMV_HPEL, "IBC does not support IMV_HPEL");
  }
#endif

  if (CU::isIBC(cu) == false
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    || useIBCFrac
#endif
    )
    m_BinEncoder.encodeBin( (cu.imv > 0)
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                          , imvCtx( 0 )
#else
                          , Ctx::ImvFlag( 0 )
#endif
    );
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 0), 0 );

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    return;
  }
#endif
  if( sps->getAMVREnabledFlag() && cu.imv > 0 )
  {
    if (!CU::isIBC(cu))
    {
      m_BinEncoder.encodeBin(cu.imv < IMV_HPEL
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                           , imvCtx(4)
#else
                           , Ctx::ImvFlag(4)
#endif
      );
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", cu.imv < 3, 4);
    }
    if (cu.imv < IMV_HPEL)
    {
    m_BinEncoder.encodeBin( (cu.imv > 1)
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                          , imvCtx( 1 )
#else
                          , Ctx::ImvFlag( 1 )
#endif
    );
    DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", (cu.imv > 1), 1 );
    }
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (CU::isIBC(cu))
  {
    CHECK(pu.cs->sps->getIBCFracFlag() && cu.imv == IMV_HPEL, "IBC does not support IMV_HPEL");
    CHECK(!useIBCFrac && (cu.imv == IMV_OFF || cu.imv == IMV_HPEL), "Fractiona IBC is not enabled to support fractional BVD coding");
  }
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::affine_amvr_mode( const CodingUnit& cu )
{
  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  m_BinEncoder.encodeBin( (cu.imv > 0), Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 0), 2 );

  if( cu.imv > 0 )
  {
    m_BinEncoder.encodeBin( (cu.imv > 1), Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", (cu.imv > 1), 3 );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACWriter::merge_idx( const PredictionUnit& pu )
{

  if ( pu.cu->affine )
  {
#if AFFINE_MMVD
    if (pu.afMmvdFlag)
    {
      return;
    }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    if (pu.affBMMergeFlag)
    {
      return;
    }
#endif
    int numCandminus1 = int( pu.cs->picHeader->getMaxNumAffineMergeCand() ) - 1;
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (pu.affineOppositeLic)
    {
      numCandminus1 = int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand()) - 1;
    }
#endif
    if ( numCandminus1 > 0 )
    {
#if JVET_AA0128_AFFINE_MERGE_CTX_INC
      unsigned int unaryIdx = 0;
      for (; unaryIdx < numCandminus1; ++unaryIdx)
      {
        unsigned int symbol = pu.mergeIdx == unaryIdx ? 0 : 1;
        m_BinEncoder.encodeBin(symbol, Ctx::AffMergeIdx((unaryIdx > 2 ? 2 : unaryIdx)));
        if (symbol == 0)
        {
          break;
        }
      }
#else
      if ( pu.mergeIdx == 0 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::AffMergeIdx() );
        DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
        return;
      }
      else
      {
        m_BinEncoder.encodeBin( 1, Ctx::AffMergeIdx() );
        for ( unsigned idx = 1; idx < numCandminus1; idx++ )
        {
            m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
          if ( pu.mergeIdx == idx )
          {
            break;
          }
        }
      }
#endif
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
    if( pu.cu->geoFlag )
    {
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
      bool bUseOnlyOneVector1 = pu.cs->slice->isInterP() || pu.cs->sps->getMaxNumGeoCand() == 1;
      CHECK(bUseOnlyOneVector1, "CABACWriter::merge_idx( bUseOnlyOneVector=1 ) failed.");

      if ( CU::isGeoBlendAvailable(*pu.cu) )
      {
        m_BinEncoder.encodeBin( pu.cu->geoBlendFlag, Ctx::GeoBlendFlag() );
        DTRACE( g_trace_ctx, D_SYNTAX, "geoBlendFlag() geoBlendFlag=%d\n", pu.cu->geoBlendFlag ? 1 : 0 );
      }
      else 
      {
        CHECK(pu.cu->geoBlendFlag, "CABACWriter::merge_idx()\tgeoBlendFlag not available failed.")
      }

      if ( pu.cu->geoBlendFlag )
      {
        int candIdx0 = pu.geoMergeIdx0;
        CHECK( candIdx0 >= pu.cs->sps->getMaxNumGeoBlendCand() , "geoBlend idx should be < sps->getMaxNumGeoCand()");

        const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoBlendCand() - 2;
        m_BinEncoder.encodeBin( candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx() );
        if ( candIdx0 > 0 )
        {
          unary_max_eqprob( candIdx0 - 1, maxNumGeoCand );
        }
        DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.geoMergeIdx0 );
        return;
      }
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
      int blkSizeSmall = pu.lwidth() < pu.lheight() ? pu.lwidth() : pu.lheight();
      if (blkSizeSmall < GPM_BLENDING_SIZE_THRESHOLD)
      {
        geoAdaptiveBlendingIdx(pu, pu.geoBldIdx);
      }
      else
      {
        geoAdaptiveBlendingIdx(pu, pu.geoBldIdx - 1);
      }
#else
      geoAdaptiveBlendingIdx(pu.geoBldIdx);
#endif
#endif

#if JVET_W0097_GPM_MMVD_TM
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
      bool isIntra0 = (pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS);
      bool isIntra1 = (pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS);

      bool isAffGPMValid = PU::isAffineGPMValid(pu);
      int  affGPMFlagCtxOffset = 0;

      affGPMFlagCtxOffset = PU::getAffGPMCtxOffset(pu);
#else
      bool isIntra0 = (pu.geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS);
      bool isIntra1 = (pu.geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS);
#endif
      bool bUseOnlyOneVector = pu.cs->slice->isInterP() || pu.cs->sps->getMaxNumGeoCand() == 1;
#endif
      m_BinEncoder.encodeBin(pu.geoMMVDFlag0, Ctx::GeoMmvdFlag());
      if (pu.geoMMVDFlag0)
      {
        geo_mmvd_idx(pu, REF_PIC_LIST_0);
      }
#if JVET_AG0164_AFFINE_GPM
      else
      if (isAffGPMValid)
      {
        m_BinEncoder.encodeBin(pu.affineGPM[0], Ctx::AffineFlag(affGPMFlagCtxOffset));
      }
      CHECK(pu.geoMMVDFlag0 && pu.affineGPM[0], "Aff GPM does not support MMVD");
      if (pu.affineGPM[0] || pu.geoMMVDFlag0)
      {
        CHECK(isIntra0, "Invalid isIntra0");
      }
#endif
#if JVET_Y0065_GPM_INTRA
      else
      {
        m_BinEncoder.encodeBin( isIntra0 ? 1 : 0, Ctx::GPMIntraFlag() );
      }

      if (!bUseOnlyOneVector || isIntra0)
      {
#endif
      m_BinEncoder.encodeBin(pu.geoMMVDFlag1, Ctx::GeoMmvdFlag());
      if (pu.geoMMVDFlag1)
      {
        geo_mmvd_idx(pu, REF_PIC_LIST_1);
      }
#if JVET_AG0164_AFFINE_GPM
      else
      if (isAffGPMValid)
      {
        m_BinEncoder.encodeBin(pu.affineGPM[1], Ctx::AffineFlag(affGPMFlagCtxOffset));
      }
      CHECK(pu.geoMMVDFlag1&& pu.affineGPM[1], "Aff GPM does not support MMVD");
      if (pu.affineGPM[1] || pu.geoMMVDFlag1)
      {
        CHECK(isIntra1, "Invalid isIntra0");
      }
#endif
#if JVET_Y0065_GPM_INTRA
      else if (!isIntra0)
      {
        m_BinEncoder.encodeBin( isIntra1 ? 1 : 0, Ctx::GPMIntraFlag() );
      }
      }
      else
      {
        CHECK( !isIntra1, "isIntra1 shall be true" );
      }

      CHECK( pu.gpmIntraFlag != (isIntra0 || isIntra1), "gpmIntraFlag shall be equal to (isIntra0 || isIntra1)" );
#endif

#if TM_MRG
      if (!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1)
      {
#if JVET_Y0065_GPM_INTRA
        if (!isIntra0 && !isIntra1)
#endif
#if JVET_AG0164_AFFINE_GPM
          if( !pu.affineGPM[0] && !pu.affineGPM[1])
#endif
        tm_merge_flag(pu);
        if (pu.tmMergeFlag)
        {
#if JVET_AG0164_AFFINE_GPM
          CHECK(pu.affineGPM[0] || pu.affineGPM[1], "Affine GPM cannot be used with TM");
#endif
          CHECK(!pu.geoTmFlag0 || !pu.geoTmFlag1, "both must be true");
          CHECK(pu.geoMergeIdx0 == pu.geoMergeIdx1, "Incorrect geoMergeIdx0 and geoMergeIdx1");
          geo_merge_idx(pu);
        }
        else
        {

          CHECK(pu.geoTmFlag0 || pu.geoTmFlag1, "both must be false");
#if JVET_Y0065_GPM_INTRA
          if (isIntra0 || isIntra1)
          {
            geo_merge_idx1(pu);
          }
          else
#endif
          geo_merge_idx(pu);
        }
      }
#else
      if (!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1)
      {
#if JVET_Y0065_GPM_INTRA
        if( isIntra0 || isIntra1 )
        {
          geo_merge_idx1( pu );
        }
        else
#endif
        geo_merge_idx(pu);
      }
#endif
      else if (pu.geoMMVDFlag0 && pu.geoMMVDFlag1)
      {
        if (pu.geoMMVDIdx0 == pu.geoMMVDIdx1)
        {
          geo_merge_idx(pu);
        }
        else
        {
          geo_merge_idx1(pu);
        }
      }
      else
      {
        geo_merge_idx1(pu);
      }
#else
#if !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      uint8_t splitDir = pu.geoSplitDir;
#endif
      uint8_t candIdx0 = pu.geoMergeIdx0;
      uint8_t candIdx1 = pu.geoMergeIdx1;
#if !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir );
#endif
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n", candIdx0 );
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n", candIdx1 );
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      geoModeIdx(pu);
#else
      xWriteTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
#endif
      candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      CHECK(candIdx0 >= maxNumGeoCand, "Incorrect candIdx0");
      CHECK(candIdx1 >= maxNumGeoCand, "Incorrect candIdx1");
      int numCandminus2 = maxNumGeoCand - 2;
      m_BinEncoder.encodeBin( candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx() );
      if( candIdx0 > 0 )
      {
        unary_max_eqprob(candIdx0 - 1, numCandminus2);
      }
      if (numCandminus2 > 0)
      {
        m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
        if (candIdx1 > 0)
        {
          unary_max_eqprob(candIdx1 - 1, numCandminus2 - 1);
        }
      }
#endif
      return;
    }
    int numCandminus1;
#if JVET_X0049_ADAPT_DMVR
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    uint16_t mergeIdx = pu.mergeIdx;
#else
    uint8_t mergeIdx = pu.mergeIdx;
#endif
#endif
    if (pu.cu->predMode == MODE_IBC)
    {
#if JVET_AA0061_IBC_MBVD
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.ibcMbvdMergeFlag && (pu.interDir == 1 || pu.ibcMergeIdx1 >= IBC_MRG_MAX_NUM_CANDS))
#else
      if (pu.ibcMbvdMergeFlag)
#endif
      {
        return;
      }
#endif
#if JVET_AC0112_IBC_GPM
      if (pu.ibcGpmFlag)
      {
        return;
      }
#endif
      numCandminus1 = int(pu.cs->sps->getMaxNumIBCMergeCand()) - 1;
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.interDir == 3 && pu.mergeFlag)
      {
        if (pu.ibcMbvdMergeFlag)
        {
          numCandminus1 = std::min(numCandminus1, IBC_MBVD_BASE_NUM - 1);
          mergeIdx = pu.ibcMergeIdx1;
        }
        else
        {
          numCandminus1--;
        }
      }
#endif
    }
#if TM_MRG
    else if (pu.tmMergeFlag)
#if JVET_X0141_CIIP_TIMD_TM
    {
      if (pu.ciipFlag)
      {
        numCandminus1 = int(pu.cs->sps->getMaxNumCiipTMMergeCand()) - 1;
      }
      else
      {
        numCandminus1 = int(pu.cs->sps->getMaxNumTMMergeCand()) - 1;
      }
    }
#else
      numCandminus1 = int(pu.cs->sps->getMaxNumTMMergeCand()) - 1;
#endif
#endif
#if JVET_X0049_ADAPT_DMVR
    else if (pu.bmMergeFlag)
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumBMMergeCand()) - 1;
      if (pu.bmDir == 2)
      {
        mergeIdx -= BM_MRG_MAX_NUM_CANDS;
      }
    }
#endif
    else
      numCandminus1 = int(pu.cs->sps->getMaxNumMergeCand()) - 1;

#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (pu.mergeOppositeLic)
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumOppositeLicMergeCand()) - 1;
    }
    else if (pu.tmMergeFlagOppositeLic)
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumTMOppositeLicMergeCand()) - 1;
    }
#endif
  if( numCandminus1 > 0 )
  {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    const CtxSet mrgIdxCtxSet = pu.tmMergeFlag ? Ctx::TmMergeIdx : Ctx::MergeIdx;
#endif
#if NON_ADJACENT_MRG_CAND
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
    {
#if JVET_X0049_ADAPT_DMVR
      unsigned int uiSymbol = mergeIdx == uiUnaryIdx ? 0 : 1;
#else
      unsigned int uiSymbol = pu.mergeIdx == uiUnaryIdx ? 0 : 1;
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));
#else
      m_BinEncoder.encodeBin(uiSymbol, Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));
#endif
      if (uiSymbol == 0)
      {
        break;
      }
    }
#else
#if JVET_X0049_ADAPT_DMVR
    if (mergeIdx == 0)
#else
    if (pu.mergeIdx == 0)
#endif
    {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      m_BinEncoder.encodeBin( 0, mrgIdxCtxSet() );
#else
      m_BinEncoder.encodeBin( 0, Ctx::MergeIdx() );
#endif
      DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
      return;
    }
    else
    {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      m_BinEncoder.encodeBin( 1, mrgIdxCtxSet() );
#else
      m_BinEncoder.encodeBin( 1, Ctx::MergeIdx() );
#endif
      for( unsigned idx = 1; idx < numCandminus1; idx++ )
      {
#if JVET_X0049_ADAPT_DMVR
        m_BinEncoder.encodeBinEP(mergeIdx == idx ? 0 : 1);
        if (mergeIdx == idx)
#else
          m_BinEncoder.encodeBinEP( pu.mergeIdx == idx ? 0 : 1 );
        if( pu.mergeIdx == idx )
#endif
        {
          break;
        }
      }
    }
#endif
  }
#if JVET_X0049_ADAPT_DMVR
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.ibcMbvdMergeFlag)
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() ibc_merge_idx1=%d\n", mergeIdx );
  else 
#endif
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", mergeIdx );
#else
  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
#endif
  }
}

void CABACWriter::mmvd_merge_idx(const PredictionUnit& pu)
{
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if(pu.cs->sps->getUseTMMMVD())
  {
#endif
  int mvpIdx = pu.mmvdMergeIdx;
  int var0;
  var0 = mvpIdx / MMVD_MAX_REFINE_NUM;
  mvpIdx -= var0 * MMVD_MAX_REFINE_NUM;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numCandMinus1Base = std::min<int>(MMVD_BASE_MV_NUM, pu.cs->sps->getMaxNumMergeCand()) - 1;
  if (numCandMinus1Base > 0)
  {
    // to support more base candidates
    m_BinEncoder.encodeBin((var0 == 0 ? 0 : 1), Ctx::MmvdMergeIdx(0));
    if (var0 > 0)
    {
      for (unsigned idx = 1; idx < numCandMinus1Base; idx++)
      {
        m_BinEncoder.encodeBin((var0 == idx ? 0 : 1), Ctx::MmvdMergeIdx(idx));
        if (var0 == idx)
        {
          break;
        }
      }
    }
  }
#else
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    assert(var0 < 2);
    m_BinEncoder.encodeBin(var0, Ctx::MmvdMergeIdx());
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() base_mvp_idx=%d\n", var0);
  unsigned int ricePar = 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numStepCandMinus1 =  ((MMVD_MAX_REFINE_NUM >> ricePar) >> MMVD_SIZE_SHIFT)/MMVD_BI_DIR - 1;
#else
  int numStepCandMinus1 =  ((MMVD_MAX_REFINE_NUM >> ricePar) >> MMVD_SIZE_SHIFT) - 1;
#endif
  if(ricePar > 0)
  {
    m_BinEncoder.encodeBinsEP(mvpIdx % (1 << ricePar), ricePar);
  }
  mvpIdx >>= ricePar;
  for (unsigned int uiUnaryIdx = 0; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
  {
    unsigned int uiSymbol = mvpIdx == uiUnaryIdx ? 0 : 1;
    m_BinEncoder.encodeBin(uiSymbol, Ctx::MmvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx)));
    if (uiSymbol == 0)
    {
      break;
    }
  }

  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    return;
  }
#endif
#endif

#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
  int var0, var1, var2;
  int mvpIdx = pu.mmvdMergeIdx;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  var0 = mvpIdx / VVC_MMVD_MAX_REFINE_NUM;
  var1 = (mvpIdx - (var0 * VVC_MMVD_MAX_REFINE_NUM)) / 4;
  var2 = mvpIdx - (var0 * VVC_MMVD_MAX_REFINE_NUM) - var1 * 4;
#else
  var0 = mvpIdx / MMVD_MAX_REFINE_NUM;
  var1 = (mvpIdx - (var0 * MMVD_MAX_REFINE_NUM)) / 4;
  var2 = mvpIdx - (var0 * MMVD_MAX_REFINE_NUM) - var1 * 4;
#endif
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
#if !JVET_AA0093_ENHANCED_MMVD_EXTENSION
    static_assert(MMVD_BASE_MV_NUM == 2, "");
#endif
    assert(var0 < 2);
    m_BinEncoder.encodeBin(var0, Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);

  int numStepCandMinus1 = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                          VVC_MMVD_REFINE_STEP - 1;
#else
                          MMVD_REFINE_STEP - 1;
#endif
  if (numStepCandMinus1 > 0)
  {
    if (var1 == 0)
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      m_BinEncoder.encodeBin(0, Ctx::MmvdStepMvpIdxECM3());
#else
      m_BinEncoder.encodeBin(0, Ctx::MmvdStepMvpIdx());
#endif
    }
    else
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      m_BinEncoder.encodeBin(1, Ctx::MmvdStepMvpIdxECM3());
#else
      m_BinEncoder.encodeBin(1, Ctx::MmvdStepMvpIdx());
#endif
      for (unsigned idx = 1; idx < numStepCandMinus1; idx++)
      {
        m_BinEncoder.encodeBinEP(var1 == idx ? 0 : 1);
        if (var1 == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_step_mvp_idx() mmvd_step_mvp_idx=%d\n", var1);

  m_BinEncoder.encodeBinsEP(var2, 2);

  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
#endif
}

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void CABACWriter::geoModeIdx(const uint8_t geoMode, const uint8_t altCodeIdx)
{
  if (altCodeIdx == 0)
  {
    xWriteTruncBinCode(geoMode, GEO_NUM_PARTITION_MODE);
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%u\n", geoMode );
    return;
  }

  const int maxNumBins    = (GEO_NUM_SIG_PARTMODE / GEO_SPLIT_MODE_RICE_CODE_DIVISOR) - 1;
  const int maxNumCtxBins = 5;
        int geoModePrefix = ((int)geoMode) / GEO_SPLIT_MODE_RICE_CODE_DIVISOR;

  for (int binIdx = 0; binIdx < maxNumBins; ++binIdx)
  {
    unsigned binVal = (binIdx == geoModePrefix ? 0 : 1);
    if (binIdx < maxNumCtxBins)
    {
      m_BinEncoder.encodeBin(binVal, Ctx::GeoSubModeIdx(binIdx));
      if (binVal == 0)
      {
        break;
      }
    }
    else
    {
      m_BinEncoder.encodeBinEP(binVal);
      if (binVal == 0)
      {
        break;
      }
    }
  }

  if (GEO_SPLIT_MODE_RICE_CODE_DIVISOR > 1)
  {
    uint8_t geoModeSuffix = geoMode & (uint8_t)(GEO_SPLIT_MODE_RICE_CODE_DIVISOR - 1);
    xWriteTruncBinCode(geoModeSuffix, GEO_SPLIT_MODE_RICE_CODE_DIVISOR);
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d(prefix=%d suffix=%u)\n", geoMode, geoModePrefix, geoModeSuffix );
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", geoMode );
  }
}

void CABACWriter::geoModeIdx(const PredictionUnit& pu)
{
  if (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode())
  {
    geoModeIdx(pu.geoSplitDir, 0);
    return;
  }

  geoModeIdx(pu.geoSyntaxMode, 1);
}
#endif


#if JVET_W0097_GPM_MMVD_TM
void CABACWriter::geo_mmvd_idx(const PredictionUnit& pu, RefPicList eRefPicList)
{
  int geoMMVDIdx = (eRefPicList == REF_PIC_LIST_0) ? pu.geoMMVDIdx0 : pu.geoMMVDIdx1;
  bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  CHECK(geoMMVDIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM), "invalid GPM MMVD index exist");

  int step = (extMMVD ? (geoMMVDIdx >> 3) : (geoMMVDIdx >> 2));
  int direction = (extMMVD ? (geoMMVDIdx - (step << 3)) : (geoMMVDIdx - (step << 2)));

  int mmvdStepToIdx[GPM_EXT_MMVD_REFINE_STEP] = { 5, 0, 1, 2, 3, 4, 6, 7, 8 };
  step = mmvdStepToIdx[step];

  int numStepCandMinus1 = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  if (numStepCandMinus1 > 0)
  {
    if (step == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::GeoMmvdStepMvpIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::GeoMmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numStepCandMinus1; idx++)
      {
        m_BinEncoder.encodeBinEP(step == idx ? 0 : 1);
        if (step == idx)
        {
          break;
        }
      }
    }
  }

  int maxMMVDDir = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  m_BinEncoder.encodeBinsEP(direction, maxMMVDDir > 4 ? 3 : 2);
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_mmvd_idx() geo_mmvd_idx%d=%d\n", eRefPicList, geoMMVDIdx);
}

void CABACWriter::geo_merge_idx(const PredictionUnit& pu)
{
#if !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  uint8_t splitDir = pu.geoSplitDir;
#endif
  uint8_t candIdx0 = pu.geoMergeIdx0;
  uint8_t candIdx1 = pu.geoMergeIdx1;

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx(pu);
#else
  xWriteTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
#endif
#if JVET_AG0164_AFFINE_GPM
  candIdx1 -= candIdx1 < candIdx0 || (pu.affineGPM[0] != pu.affineGPM[1] )? 0 : 1;
#else
  candIdx1 -= candIdx1 < candIdx0 ? 0 : 1;
#endif
#if JVET_AG0164_AFFINE_GPM
  int maxNumGeoCand = pu.affineGPM[0] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
#else
  const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
#endif
  int numCandminus2 = maxNumGeoCand - 2;
#if JVET_AG0164_AFFINE_GPM
  CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
  mrgIdxCtxSet = pu.affineGPM[0] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
  unsigned int uiUnaryIdx = 0;
  for (; uiUnaryIdx < numCandminus2 + 1; ++uiUnaryIdx)
  {
    unsigned int uiSymbol = candIdx0 == uiUnaryIdx ? 0 : 1;

    m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));

    if (uiSymbol == 0)
    {
      break;
    }
  }
#else
  m_BinEncoder.encodeBin(candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx());
  if (candIdx0 > 0)
  {
    unary_max_eqprob(candIdx0 - 1, numCandminus2);
  }
#endif

#if JVET_AG0164_AFFINE_GPM
  maxNumGeoCand = pu.affineGPM[1] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
  numCandminus2 = maxNumGeoCand - 2;
#endif
  if (numCandminus2 > 0
#if JVET_AG0164_AFFINE_GPM
    || (pu.affineGPM[0] != pu.affineGPM[1])
#endif
    )
  {
#if JVET_AG0164_AFFINE_GPM
    CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
    mrgIdxCtxSet = pu.affineGPM[1] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus2 + ((pu.affineGPM[0] != pu.affineGPM[1]) ? 1 : 0); ++uiUnaryIdx)
    {
      unsigned int uiSymbol = candIdx1 == uiUnaryIdx ? 0 : 1;

      m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));

      if (uiSymbol == 0)
      {
        break;
      }
    }
#else
    m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
    if (candIdx1 > 0)
    {  
#if JVET_AG0164_AFFINE_GPM
      unary_max_eqprob(candIdx1 - 1, numCandminus2 - ((pu.affineGPM[0] != pu.affineGPM[1]) ? 0 : 1));
#else
      unary_max_eqprob(candIdx1 - 1, numCandminus2 - 1);
#endif
    }
#endif
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx() geo_merge_idx0=%d geo_merge_idx1=%d\n", pu.geoMergeIdx0, pu.geoMergeIdx1);
}

void CABACWriter::geo_merge_idx1(const PredictionUnit& pu)
{
#if !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  uint8_t splitDir = pu.geoSplitDir;
#endif
  uint8_t candIdx0 = pu.geoMergeIdx0;
  uint8_t candIdx1 = pu.geoMergeIdx1;
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
  bool isIntra0 = (pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS);
  bool isIntra1 = (pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS);
#else
  bool isIntra0 = (candIdx0 >= GEO_MAX_NUM_UNI_CANDS);
  bool isIntra1 = (candIdx1 >= GEO_MAX_NUM_UNI_CANDS);
#endif
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx(pu);
#else
  xWriteTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
#endif

#if JVET_AG0164_AFFINE_GPM
  int maxNumGeoCand = pu.affineGPM[0] ? pu.cs->sps->getMaxNumGpmAffCand(): pu.cs->sps->getMaxNumGeoCand();
#else
  const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
#endif
  int numCandminus2 = maxNumGeoCand - 2;
#if JVET_Y0065_GPM_INTRA
  if (isIntra0)
  {
#if JVET_AG0164_AFFINE_GPM
    unary_max_eqprob(candIdx0 - GEO_MAX_ALL_INTER_UNI_CANDS, GEO_MAX_NUM_INTRA_CANDS-1);
#else
    unary_max_eqprob(candIdx0 - GEO_MAX_NUM_UNI_CANDS, GEO_MAX_NUM_INTRA_CANDS-1);
#endif
  }
  else if (numCandminus2 >= 0)
  {
#endif
#if JVET_AG0164_AFFINE_GPM
    CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
    mrgIdxCtxSet = pu.affineGPM[0] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus2 + 1; ++uiUnaryIdx)
    {
      unsigned int uiSymbol = candIdx0 == uiUnaryIdx ? 0 : 1;

      m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));

      if (uiSymbol == 0)
      {
        break;
      }
    }
#else
  m_BinEncoder.encodeBin(candIdx0 == 0 ? 0 : 1, Ctx::MergeIdx());
  if (candIdx0 > 0)
  {
    unary_max_eqprob(candIdx0 - 1, numCandminus2);
  }
#endif
#if JVET_Y0065_GPM_INTRA
  }

#if JVET_AG0164_AFFINE_GPM
  maxNumGeoCand = pu.affineGPM[1] ? pu.cs->sps->getMaxNumGpmAffCand() : pu.cs->sps->getMaxNumGeoCand();
  numCandminus2 = maxNumGeoCand - 2;
#endif

  if (isIntra1)
  {
#if JVET_AG0164_AFFINE_GPM
    unary_max_eqprob(candIdx1 - GEO_MAX_ALL_INTER_UNI_CANDS, GEO_MAX_NUM_INTRA_CANDS - 1);
#else
    unary_max_eqprob(candIdx1 - GEO_MAX_NUM_UNI_CANDS, GEO_MAX_NUM_INTRA_CANDS-1);
#endif
  }
  else if (numCandminus2 >= 0)
  {
#endif
#if JVET_AG0164_AFFINE_GPM
    CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
    mrgIdxCtxSet = pu.affineGPM[1] ? Ctx::GpmAffMergeIdx : Ctx::GpmMergeIdx;
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus2 + 1; ++uiUnaryIdx)
    {
      unsigned int uiSymbol = candIdx1 == uiUnaryIdx ? 0 : 1;

      m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));

      if (uiSymbol == 0)
      {
        break;
      }
    }
#else
  m_BinEncoder.encodeBin(candIdx1 == 0 ? 0 : 1, Ctx::MergeIdx());
  if (candIdx1 > 0)
  {
    unary_max_eqprob(candIdx1 - 1, numCandminus2);
  }
#endif
#if JVET_Y0065_GPM_INTRA
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_merge_idx1() geo_merge_idx0=%d geo_merge_idx1=%d\n", pu.geoMergeIdx0, pu.geoMergeIdx1);
}

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
uint64_t CABACWriter::geo_blend_est( const TempCtx& ctxStart, const int flag )
{
  getCtx() = ctxStart;
  resetBits();

  m_BinEncoder.encodeBin( flag, Ctx::GeoBlendFlag() );

  return getEstFracBits();
}
#endif

uint64_t CABACWriter::geo_mode_est(const TempCtx& ctxStart, const int geoMode
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                                 , const uint8_t altCodeIdx
#endif
)
{
  getCtx() = SubCtx(Ctx::GeoSubModeIdx, ctxStart);
  resetBits();

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx((uint8_t)geoMode, altCodeIdx);
#else
  xWriteTruncBinCode(geoMode, GEO_NUM_PARTITION_MODE);
#endif

  return getEstFracBits();
}

uint64_t CABACWriter::geo_mergeIdx_est(const TempCtx& ctxStart, const int candIdx, const int maxNumGeoCand
#if JVET_AG0164_AFFINE_GPM
  , int isAffine
#endif
)
{
  getCtx() = SubCtx(Ctx::MergeIdx, ctxStart);
  resetBits();

  int numCandminus2 = maxNumGeoCand - 2;
#if JVET_Y0065_GPM_INTRA
  if (numCandminus2 < 0)
  {
    return 0;
  }
#endif
#if JVET_AG0164_AFFINE_GPM
  CtxSet mrgIdxCtxSet = Ctx::MergeIdx;
  mrgIdxCtxSet = isAffine? Ctx::GpmAffMergeIdx: Ctx::GpmMergeIdx;
  unsigned int uiUnaryIdx = 0;
  for (; uiUnaryIdx < numCandminus2 + 1; ++uiUnaryIdx)
  {
    unsigned int uiSymbol = candIdx == uiUnaryIdx ? 0 : 1;

    m_BinEncoder.encodeBin(uiSymbol, mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx)));

    if (uiSymbol == 0)
    {
      break;
    }
  }
#else
  m_BinEncoder.encodeBin(candIdx == 0 ? 0 : 1, Ctx::MergeIdx());
  if (candIdx > 0)
  {
    unary_max_eqprob(candIdx - 1, numCandminus2);
  }
#endif
  return getEstFracBits();
}

#if JVET_Y0065_GPM_INTRA
uint64_t CABACWriter::geo_intraFlag_est( const TempCtx& ctxStart, const int flag)
{
  getCtx() = SubCtx(Ctx::GPMIntraFlag, ctxStart);
  resetBits();

  m_BinEncoder.encodeBin(flag, Ctx::GPMIntraFlag());

  return getEstFracBits();
}

uint64_t CABACWriter::geo_intraIdx_est( const int intraIdx)
{
  resetBits();

  unary_max_eqprob(intraIdx, GEO_MAX_NUM_INTRA_CANDS-1);

  return getEstFracBits();
}
#endif

uint64_t CABACWriter::geo_mmvdFlag_est(const TempCtx& ctxStart, const int flag)
{
  getCtx() = SubCtx(Ctx::GeoMmvdFlag, ctxStart);
  resetBits();

  m_BinEncoder.encodeBin(flag, Ctx::GeoMmvdFlag());

  return getEstFracBits();
}

#if TM_MRG
uint64_t CABACWriter::geo_tmFlag_est(const TempCtx& ctxStart, const int flag)
{
  getCtx() = SubCtx(Ctx::TMMergeFlag, ctxStart);
  resetBits();

  m_BinEncoder.encodeBin(flag, Ctx::TMMergeFlag());

  return getEstFracBits();
}
#endif

uint64_t CABACWriter::geo_mmvdIdx_est(const TempCtx& ctxStart, const int geoMMVDIdx, const bool extMMVD)
{
  getCtx() = SubCtx(Ctx::GeoMmvdStepMvpIdx, ctxStart);
  resetBits();

  CHECK(geoMMVDIdx >= (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM), "invalid GPM MMVD index exist");
  int step = (extMMVD ? (geoMMVDIdx >> 3) : (geoMMVDIdx >> 2));
  int direction = (extMMVD ? (geoMMVDIdx - (step << 3)) : (geoMMVDIdx - (step << 2)));

  int mmvdStepToIdx[GPM_EXT_MMVD_REFINE_STEP] = { 5, 0, 1, 2, 3, 4, 6, 7, 8 };
  step = mmvdStepToIdx[step];

  int numStepCandMinus1 = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  if (numStepCandMinus1 > 0)
  {
    if (step == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::GeoMmvdStepMvpIdx());
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::GeoMmvdStepMvpIdx());
      for (unsigned idx = 1; idx < numStepCandMinus1; idx++)
      {
        m_BinEncoder.encodeBinEP(step == idx ? 0 : 1);
        if (step == idx)
        {
          break;
        }
      }
    }
  }
  int maxMMVDDir = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  m_BinEncoder.encodeBinsEP(direction, maxMMVDDir > 4 ? 3 : 2);
  return getEstFracBits();
}
#endif
#if JVET_AG0164_AFFINE_GPM
uint64_t CABACWriter::geo_affFlag_est(const TempCtx& ctxStart, const int flag, int ctxOffset)
{
  getCtx() = ctxStart;
  resetBits();

  m_BinEncoder.encodeBin(flag, Ctx::AffineFlag(ctxOffset));

  return getEstFracBits();
}
#endif


#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
uint64_t CABACWriter::geoBldFlagEst(const PredictionUnit& pu, const TempCtx& ctxStart, const int flag)
#else
uint64_t CABACWriter::geoBldFlagEst(const TempCtx& ctxStart, const int flag)
#endif
{
  getCtx() = ctxStart;
  resetBits();
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
  int blkSizeSmall = pu.lwidth() < pu.lheight() ? pu.lwidth() : pu.lheight();
  if (blkSizeSmall < GPM_BLENDING_SIZE_THRESHOLD)
  {
    geoAdaptiveBlendingIdx(pu, flag);
  }
  else
    geoAdaptiveBlendingIdx(pu, flag - 1);
#else
  geoAdaptiveBlendingIdx(flag);
#endif

  return getEstFracBits();
}

#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
void CABACWriter::geoAdaptiveBlendingIdx(const PredictionUnit& pu, const int idx)
#else
void CABACWriter::geoAdaptiveBlendingIdx(const int idx)
#endif
{
#if JVET_AH0314_ADAPTIVE_GPM_BLENDING_IMPROV
  int blkSizeSmall = pu.lwidth() < pu.lheight() ? pu.lwidth() : pu.lheight();
  int offset = (blkSizeSmall < GPM_BLENDING_SIZE_THRESHOLD) ? 0 : 4;
  if (idx == 2)
  {
    m_BinEncoder.encodeBin(1, Ctx::GeoBldFlag(0 + offset));
  }
  else
  {
    m_BinEncoder.encodeBin(0, Ctx::GeoBldFlag(0 + offset));
    if (idx == 0 || idx == 1)
    {
      m_BinEncoder.encodeBin(1, Ctx::GeoBldFlag(1 + offset));
      m_BinEncoder.encodeBin(idx == 0, Ctx::GeoBldFlag(2 + offset));
    }
    else
    {
      m_BinEncoder.encodeBin(0, Ctx::GeoBldFlag(1 + offset));
      m_BinEncoder.encodeBin(idx == 3, Ctx::GeoBldFlag(3 + offset));
    }
  }
#else
  if (idx == 2)
  {
    m_BinEncoder.encodeBin(1, Ctx::GeoBldFlag(0));
  }
  else
  {
    m_BinEncoder.encodeBin(0, Ctx::GeoBldFlag(0));
    if (idx == 0 || idx == 1)
    {
      m_BinEncoder.encodeBin(1, Ctx::GeoBldFlag(1));
      m_BinEncoder.encodeBin(idx == 0, Ctx::GeoBldFlag(2));
    }
    else
    {
      m_BinEncoder.encodeBin(0, Ctx::GeoBldFlag(1));
      m_BinEncoder.encodeBin(idx == 3, Ctx::GeoBldFlag(3));
    }
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "geo_adaptive_blending_idx() geo_bld_idx=%d\n", idx);
}
#endif

void CABACWriter::inter_pred_idc( const PredictionUnit& pu )
{
  if( !pu.cs->slice->isInterB() )
  {
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( pu.interDir == 3 )
    {
      m_BinEncoder.encodeBin( 1, Ctx::InterDir(ctxId) );
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
    else
    {
      m_BinEncoder.encodeBin( 0, Ctx::InterDir(ctxId) );
    }
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (pu.cs->sps->getUseARL())
  {
    return;
  }
#endif
#if CTU_256
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 7 ) );
#else
  m_BinEncoder.encodeBin( ( pu.interDir == 2 ), Ctx::InterDir( 6 ) );
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=7 value=%d pos=(%d,%d)\n", pu.interDir, pu.lumaPos().x, pu.lumaPos().y );
}

#if JVET_Z0054_BLK_REF_PIC_REORDER
void CABACWriter::refIdxLC(const PredictionUnit& pu)
{
  if (!PU::useRefCombList(pu))
  {
    return;
  }
  int numRefMinus1 = (int)pu.cs->slice->getRefPicCombinedList().size() - 1;
  int refIdxLC = pu.refIdxLC;
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    numRefMinus1 = (int)pu.cs->slice->getRefPicCombinedListAmvpMerge().size() - 1;
  }
#endif
  if (numRefMinus1 > 0)
  {
    if (refIdxLC == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::RefPicLC(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "refIdxLC() value=%d pos=(%d,%d)\n", refIdxLC, pu.lumaPos().x, pu.lumaPos().y);
      return;
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::RefPicLC(0));
      for (unsigned idx = 1; idx < numRefMinus1; idx++)
      {
        m_BinEncoder.encodeBin(refIdxLC == idx ? 0 : 1, Ctx::RefPicLC(std::min((int)idx, 2)));
        if (refIdxLC == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "refIdxLC() value=%d pos=(%d,%d)\n", refIdxLC, pu.lumaPos().x, pu.lumaPos().y);
}

void CABACWriter::refPairIdx(const PredictionUnit& pu)
{
  if (!PU::useRefPairList(pu))
  {
    return;
  }
  int numRefMinus1 = (int)pu.cs->slice->getRefPicPairList().size() - 1;
  int refPairIdx = pu.refPairIdx;
  if (numRefMinus1 > 0)
  {
    if (refPairIdx == 0)
    {
      m_BinEncoder.encodeBin(0, Ctx::RefPicLC(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "refPairIdx() value=%d pos=(%d,%d)\n", refPairIdx, pu.lumaPos().x, pu.lumaPos().y);
      return;
    }
    else
    {
      m_BinEncoder.encodeBin(1, Ctx::RefPicLC(0));
      for (unsigned idx = 1; idx < numRefMinus1; idx++)
      {
        m_BinEncoder.encodeBin(refPairIdx == idx ? 0 : 1, Ctx::RefPicLC(std::min((int)idx, 2)));
        if (refPairIdx == idx)
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "refPairIdx() value=%d pos=(%d,%d)\n", refPairIdx, pu.lumaPos().x, pu.lumaPos().y);
}
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER && JVET_AD0213_LIC_IMP
void CABACWriter::ref_idx(const PredictionUnit& pu, RefPicList eRefList, bool forceRefIdx)
#else
void CABACWriter::ref_idx( const PredictionUnit& pu, RefPicList eRefList )
#endif
{
  if ( pu.cu->smvdMode )
  {
    CHECK( pu.refIdx[eRefList] != pu.cs->slice->getSymRefIdx( eRefList ), "Invalid reference index!\n" );
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
#if JVET_AD0213_LIC_IMP
  if (!forceRefIdx)
#endif
  if (PU::useRefCombList(pu) || PU::useRefPairList(pu))
  {
    return;
  }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[1 - eRefList])
  {
#if JVET_Y0128_NON_CTC
    if (pu.cu->slice->getAmvpMergeModeOnlyOneValidRefIdx(eRefList) >= 0)
    {
      return;
    }
#else
    const RefPicList refListAmvp = eRefList;
    const RefPicList refListMerge = RefPicList(1 - eRefList);
    const int curPoc = pu.cs->slice->getPOC();
    const int numRefAmvp = pu.cs->slice->getNumRefIdx(refListAmvp);
    const int numRefMerge = pu.cs->slice->getNumRefIdx(refListMerge);
    int candidateRefIdxCount = 0;
    for (int refIdxAmvp = 0; refIdxAmvp < numRefAmvp; refIdxAmvp++)
    {
      const int amvpPoc = pu.cs->slice->getRefPOC(refListAmvp, refIdxAmvp);
      bool validCandidate = false;
      for (int refIdxMerge = 0; refIdxMerge < numRefMerge; refIdxMerge++)
      {
        const int mergePoc = pu.cs->slice->getRefPOC(refListMerge, refIdxMerge);
        if ((amvpPoc - curPoc) * (mergePoc - curPoc) < 0)
        {
          validCandidate = true;
        }
      }
      if (validCandidate)
      {
        candidateRefIdxCount++;
      }
    }
    CHECK(candidateRefIdxCount == 0, "this is not possible");
    if (candidateRefIdxCount == 1)
    {
      return;
    }
#endif
  }
#endif

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (eRefList == REF_PIC_LIST_0 && pu.cs->slice->getUseIBC())
#else
  if (eRefList == REF_PIC_LIST_0 && pu.cs->sps->getIBCFlag())
#endif
  {
    if (CU::isIBC(*pu.cu))
      return;
  }

  if( numRef <= 1 )
  {
    return;
  }
  int refIdx  = pu.refIdx[eRefList];
  m_BinEncoder.encodeBin( (refIdx > 0), Ctx::RefPic() );
  if( numRef <= 2 || refIdx == 0 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  m_BinEncoder.encodeBin( (refIdx > 1), Ctx::RefPic(1) );
  if( numRef <= 3 || refIdx == 1 )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
    return;
  }
  for( int idx = 3; idx < numRef; idx++ )
  {
    if( refIdx > idx - 1 )
    {
      m_BinEncoder.encodeBinEP( 1 );
    }
    else
    {
      m_BinEncoder.encodeBinEP( 0 );
      break;
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", refIdx, pu.lumaPos().x, pu.lumaPos().y );
}

#if MULTI_HYP_PRED
void CABACWriter::mh_pred_data(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseInterMultiHyp() || !pu.cs->slice->isInterB())
  {
    CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }
  if (pu.ciipFlag)
  {
    CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }
  if (pu.cu->geoFlag)
  {
    CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  if (pu.tmMergeFlag)
  {
    CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }
#endif
#if JVET_X0049_ADAPT_DMVR
  if (pu.bmMergeFlag)
  {
    CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }
#endif
#if !JVET_Z0083_PARSINGERROR_FIX
  if (!pu.mergeFlag && pu.cu->affine && pu.cu->imv)
  {
    return;
  }
#endif
  if( !pu.mergeFlag && pu.cu->bcwIdx == BCW_DEFAULT )
  {
    return;
  }

  if( CU::isIBC( *pu.cu ) )
  {
    return;
  }

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return;
  }
#endif

  if (pu.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE || std::min(pu.Y().width, pu.Y().height) < MULTI_HYP_PRED_RESTRICT_MIN_WH)
  {
    CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
    return;
  }

  const int numMHRef = pu.cs->slice->getNumMultiHypRefPics();
  CHECK(numMHRef <= 0, "Multi Hyp: numMHRef <= 0");
  const size_t maxNumAddHyps = pu.cs->sps->getMaxNumAddHyps();
  CHECK(pu.addHypData.size() > maxNumAddHyps, "Multi Hyp: pu.addHypData.size() > maxNumAddHyps");
  int hypIdx = 0;
  for (int i = pu.numMergedAddHyps; i < pu.addHypData.size(); i++)
  {
    m_BinEncoder.encodeBin(1, Ctx::MultiHypothesisFlag(hypIdx));
    if( hypIdx < 1 )
    {
      hypIdx++;
    }
    const MultiHypPredictionData &mhData = pu.addHypData[i];

#if MULTI_HYP_PRED
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
    const int maxNumMHPCand = pu.cs->sps->getMaxNumMHPCand();
    if (maxNumMHPCand > 0)
    {
      m_BinEncoder.encodeBin(mhData.isMrg, Ctx::MultiHypothesisFlag(2));
    }
    else
    {
      CHECK(mhData.isMrg, "mhData.isMrg is true while maxNumMHPCand is 0")
    }
    if (mhData.isMrg)
    {
      CHECK(mhData.mrgIdx >= maxNumMHPCand, "Incorrect mhData.mrgIdx");
      int numCandminus2 = maxNumMHPCand - 2;
#else
    m_BinEncoder.encodeBin(mhData.isMrg, Ctx::MultiHypothesisFlag(2));
    if (mhData.isMrg)
    {
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      CHECK(mhData.mrgIdx >= maxNumGeoCand, "Incorrect mhData.mrgIdx");
      int numCandminus2 = maxNumGeoCand - 2;
#endif
      m_BinEncoder.encodeBin(mhData.mrgIdx == 0 ? 0 : 1, Ctx::MergeIdx());
      if (mhData.mrgIdx > 0)
      {
        unary_max_eqprob(mhData.mrgIdx - 1, numCandminus2);
      }
      unary_max_symbol(mhData.weightIdx, Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getNumAddHypWeights() - 1);
      continue;
    }
#endif
    CHECK(mhData.refIdx < 0, "Multi Hyp: mhData.refIdx < 0");
    CHECK(mhData.refIdx >= numMHRef, "Multi Hyp: mhData.refIdx >= numMHRef");
    ref_idx_mh(numMHRef, mhData.refIdx);
    Mv mhMvd = mhData.mvd;
    if (pu.cu->affine)
    {
      mhMvd.changeAffinePrecInternal2Amvr(pu.cu->imv);
    }
    else
    {
      mhMvd.changeTransPrecInternal2Amvr(pu.cu->imv);
    }
#if JVET_AD0140_MVD_PREDICTION 
    mvd_coding(mhMvd, 0, nullptr, true);
#else
    mvd_coding(mhMvd, 0);
#endif
    m_BinEncoder.encodeBin(mhData.mvpIdx, Ctx::MVPIdx());
    CHECK(mhData.weightIdx < 0, "Multi Hyp: mhData.weightIdx < 0");
    CHECK(mhData.weightIdx >= pu.cs->sps->getNumAddHypWeights(), "Multi Hyp: mhData.weightIdx >= pu.cs->sps->getSpsNext().getNumAddHypWeights()");
    unary_max_symbol(mhData.weightIdx, Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getNumAddHypWeights() - 1);
  }

  if( ( pu.addHypData.size() - pu.numMergedAddHyps ) < maxNumAddHyps )
  {
    m_BinEncoder.encodeBin( 0, Ctx::MultiHypothesisFlag( hypIdx ) );
  }
}

void CABACWriter::ref_idx_mh(const int numRef, const int refIdx)
{
  if (numRef <= 1)
  {
    return;
  }
  m_BinEncoder.encodeBin((refIdx > 0), Ctx::MHRefPic());
  if (numRef <= 2 || refIdx == 0)
  {
    return;
  }
  m_BinEncoder.encodeBin((refIdx > 1), Ctx::MHRefPic(1));
  if (numRef <= 3 || refIdx == 1)
  {
    return;
  }
  for (int idx = 3; idx < numRef; idx++)
  {
    if (refIdx > idx - 1)
    {
      m_BinEncoder.encodeBinEP(1);
    }
    else
    {
      m_BinEncoder.encodeBinEP(0);
      break;
    }
  }
}
#endif

void CABACWriter::mvp_flag( const PredictionUnit& pu, RefPicList eRefList )
{
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[eRefList])
  {
    return;
  }
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (pu.amvpMergeModeFlag[1 - eRefList] == true && !CU::isIBC(*pu.cu))
#else
  if (pu.amvpMergeModeFlag[1 - eRefList] == true)
#endif
  {
    if (pu.mvpIdx[eRefList] < 2)
    {
#if TM_AMVP
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
#if JVET_Z0054_BLK_REF_PIC_REORDER
      if (pu.cs->sps->getUseARL())
      {
        RefListAndRefIdx refListComb = pu.cs->slice->getRefPicCombinedListAmvpMerge()[pu.refIdxLC];
        if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(refListComb.refList, refListComb.refIdx)) == false)
        {
          m_BinEncoder.encodeBin(0, Ctx::MVPIdx());
        }
      }
      else
#endif
        if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) == false)
#else
      if(!pu.cu->cs->sps->getUseDMVDMode() || pu.cu->affine || CU::isIBC(*pu.cu))
#endif
#endif
      {
        m_BinEncoder.encodeBin( 0, Ctx::MVPIdx() );
      }
      m_BinEncoder.encodeBinEP( pu.mvpIdx[eRefList] );
    }
    else
    {
      m_BinEncoder.encodeBin( 1, Ctx::MVPIdx() );
    }
    return;
  }
#endif
#if TM_AMVP
#if JVET_Y0128_NON_CTC || JVET_AA0132_CONFIGURABLE_TM_TOOLS
  bool needToCodeMvpIdx = false;
  if (pu.cu->affine || CU::isIBC(*pu.cu))
  {
    needToCodeMvpIdx = true;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  else if (PU::useRefCombList(pu))
  {
    needToCodeMvpIdx = pu.refIdxLC >= pu.cs->slice->getNumNonScaledRefPic()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                    || !pu.cs->sps->getUseTMAmvpMode()
#else
                    || !pu.cs->sps->getUseDMVDMode()
#endif
      ;
  }
  else if (PU::useRefPairList(pu))
  {
    needToCodeMvpIdx = pu.refPairIdx >= pu.cs->slice->getNumNonScaledRefPicPair()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                    || !pu.cs->sps->getUseTMAmvpMode()
#else
                    || !pu.cs->sps->getUseDMVDMode()
#endif
      ;
  }
#endif
  else if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) == false)
  {
    needToCodeMvpIdx = true;
  }
  if (needToCodeMvpIdx)
#else
  if(!pu.cu->cs->sps->getUseDMVDMode() || pu.cu->affine || CU::isIBC(*pu.cu))
#endif
#endif
  m_BinEncoder.encodeBin( pu.mvpIdx[eRefList], Ctx::MVPIdx() );
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", pu.mvpIdx[eRefList], pu.lumaPos().x, pu.lumaPos().y );
#if !JVET_Z0054_BLK_REF_PIC_REORDER
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, pu.mvpIdx[eRefList] );
#endif
}
#if JVET_AG0135_AFFINE_CIIP
void CABACWriter::ciipAffineFlag(const PredictionUnit& pu)
{
  if (!pu.cu->slice->isIntra() && pu.cs->sps->getUseCiipAffine() && (pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand() > 0) && pu.cu->lumaSize().width >= 8 && pu.cu->lumaSize().height >= 8)
  {
    unsigned ctxId = DeriveCtx::CtxCiipAffineFlag(*pu.cu);
    m_BinEncoder.encodeBin(pu.ciipAffine, Ctx::CiipAffineFlag(ctxId));
  }
}
#endif
void CABACWriter::Ciip_flag(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseCiip())
  {
    CHECK(pu.ciipFlag == true, "invalid Ciip SPS");
    return;
  }
  if (pu.cu->skip)
  {
    CHECK(pu.ciipFlag == true, "invalid Ciip and skip");
    return;
  }
  m_BinEncoder.encodeBin(pu.ciipFlag, Ctx::CiipFlag());
  DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip=%d pos=(%d,%d) size=%dx%d\n", pu.ciipFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
#if CIIP_PDPC
  if( pu.ciipFlag )
  {
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AG0135_AFFINE_CIIP
      ciipAffineFlag(pu);
      if (!pu.ciipAffine)
      {
#endif
        tm_merge_flag(pu);
#if JVET_AG0135_AFFINE_CIIP
      }
#endif
#else
    if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
      m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::CiipTMMergeFlag());
      DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
    }
#endif
#endif
#if JVET_AG0135_AFFINE_CIIP
    if (!pu.ciipAffine)
    {
#endif
      m_BinEncoder.encodeBin(pu.ciipPDPC, Ctx::CiipFlag(1));
#if JVET_AG0135_AFFINE_CIIP
    }
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip_pdpc_flag=%d pos=(%d,%d) size=%dx%d\n", pu.ciipPDPC ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  }
#else
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if (pu.ciipFlag && pu.cs->slice->getSPS()->getUseCiipTmMrg())
  {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    tm_merge_flag(pu);
#else
    m_BinEncoder.encodeBin(pu.tmMergeFlag, Ctx::CiipTMMergeFlag());
    DTRACE(g_trace_ctx, D_SYNTAX, "ciip_flag() ciip_tm_merge_flag=%d\n", pu.tmMergeFlag);
#endif
  }
#endif
#endif
}





//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( split, depth )
//    bool  cbf_comp            ( cbf, area, depth )
//================================================================================
void CABACWriter::transform_tree(const CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, const PartSplit ispType, const int subTuIdx
#if JVET_AE0102_LFNST_CTX
  , const bool codeTuCoeff
#endif
)
{
  const UnitArea&       area = partitioner.currArea();
  int             subTuCounter = subTuIdx;
  const TransformUnit&  tu = *cs.getTU(area.blocks[partitioner.chType].pos(), partitioner.chType, subTuIdx);
  const CodingUnit&     cu = *tu.cu;
  const unsigned        trDepth = partitioner.currTrDepth;
  const bool            split = (tu.depth > trDepth);

  // split_transform_flag
  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
    CHECK( !split, "transform split implied" );
  }
  else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    CHECK( !split, "transform split implied - sbt" );
  }
  else
  CHECK( split && !cu.ispMode, "transform split not allowed with QTBT" );


  if( split )
  {

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE( g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n", partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height );

#endif
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
    {
      partitioner.splitCurrArea( PartSplit( cu.getSbtTuSplit() ), cs );
    }
    else
      THROW( "Implicit TU split not available" );

    do
    {
      transform_tree( cs, partitioner, cuCtx,                ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    transform_unit( tu, cuCtx, partitioner, subTuCounter);
  }
}

void CABACWriter::cbf_comp( const CodingStructure& cs, bool cbf, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  unsigned  ctxId = DeriveCtx::CtxQtCbf(area.compID, prevCbf, useISP && isLuma(area.compID));
  const CtxSet &ctxSet = Ctx::QtCbf[area.compID];
  if ((area.compID == COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmMode)
   || (area.compID != COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID)) != NULL && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmModeChroma))
  {
    if (area.compID == COMPONENT_Y)
      ctxId = 1;
    else if (area.compID == COMPONENT_Cb)
      ctxId = 1;
    else
      ctxId = 2;
    m_BinEncoder.encodeBin(cbf, ctxSet(ctxId));
  }
  else
  {
  m_BinEncoder.encodeBin( cbf, ctxSet( ctxId ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
}





//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================
#if JVET_AD0140_MVD_PREDICTION
void CABACWriter::mvd_coding( const Mv &rMvd, int8_t imv, const MvdSuffixInfo* const pSi
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                            , bool codeSign
#endif
                            )
{
  const bool ctxCoding = nullptr != pSi && !codeSign;

  int horMvd = rMvd.getHor();
  int verMvd = rMvd.getVer();

  unsigned horAbs = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned verAbs = unsigned( verMvd < 0 ? -verMvd : verMvd );

  // abs_mvd_greater0_flag[ 0 | 1 ]
  m_BinEncoder.encodeBin((horAbs > 0), Ctx::Mvd());
  m_BinEncoder.encodeBin((verAbs > 0), Ctx::Mvd());

  const int iEgcOffset = (!codeSign && ctxCoding) ? pSi->getEGCOffset() : 1;

  if ( iEgcOffset!=0 ) // if abs_mvd_greater1_flag is compensated in remainder?
  {
    // abs_mvd_greater1_flag[ 0 | 1 ]
    if( horAbs > 0 )
    {
      m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
    }
    if( verAbs > 0 )
    {
      m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
    }
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (!codeSign && ctxCoding)
  {
#if ENABLE_TRACING
    int horParam = -1;
    int verParam = -1;
#endif
    if (horAbs > iEgcOffset)
    {
#if ENABLE_TRACING
      horParam =
#endif
        xWriteMvdPrefix(horAbs - 1 - iEgcOffset, MVD_CODING_GOLOMB_ORDER);
    }
    if (verAbs > iEgcOffset)
    {
#if ENABLE_TRACING
      verParam =
#endif
        xWriteMvdPrefix(verAbs - 1 - iEgcOffset, MVD_CODING_GOLOMB_ORDER);
    }
#if ENABLE_TRACING
    if (horParam == -1)
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_hor) = %d \n", horAbs);
    }
    else
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() mvd_hor prefix=%d \n", horParam);
    }
    if (verParam == -1)
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_ver) = %d \n", verAbs);
    }
    else
    {
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() mvd_ver prefix=%d \n", verParam);
    }
#endif
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() code_sign=%d \n", codeSign);   
    if( horAbs > 0 )
    {
      if( horAbs > 1 )
      {
        m_BinEncoder.encodeRemAbsEP(horAbs - 2, 1, 0, MV_BITS - 1);
      }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      if (codeSign)
#endif
      m_BinEncoder.encodeBinEP( (horMvd < 0) );
    }
    if( verAbs > 0 )
    {
      if( verAbs > 1 )
      {
        m_BinEncoder.encodeRemAbsEP(verAbs - 2, 1, 0, MV_BITS - 1);
      }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      if (codeSign)
#endif
      m_BinEncoder.encodeBinEP( (verMvd < 0) );
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_hor)=%d \n", horAbs);
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding() abs(mvd_ver)=%d \n", verAbs);
  }
}

unsigned CABACWriter::xWriteMvdPrefix( unsigned uiSymbol, int param )
{ 
  unsigned bins = 0;
  unsigned numBins = 0;
  while (uiSymbol >= (unsigned)(1 << param))
  {
    bins <<= 1;
    bins++;
    numBins++;
    uiSymbol -= 1 << param;
    param++;
  }

  bins <<= 1;
  numBins++;

  unsigned temp = 0;
  for (int i = numBins - 1; i >= 0; i--)
  {
    temp = bins >> i;
    m_BinEncoder.encodeBinEP(temp);
    bins -= (temp << i);
  }
  return numBins - 1; // less by 1 as compared to what xReadBvdContextPrefix() returns
}

void CABACWriter::xWriteMvdContextSuffix(unsigned uiSymbol, int param, int paramUpdated, int numSkipMSB )
{
  unsigned numBins = 0;
  while (uiSymbol >= (unsigned) (1 << param))
  {
    numBins++;
    uiSymbol -= 1 << param;
    param++;
  }
  numBins++;
  paramUpdated++;

  CHECK(paramUpdated != numBins, "Enc side prefix bits check error");

  if (0 != numSkipMSB)
  {
    CHECK(paramUpdated == 0, "param_updated = 0");
    paramUpdated-= numSkipMSB;
    unsigned skipMask = ( 1 << (paramUpdated + numSkipMSB)) -1 - ((1 << (paramUpdated)) - 1);
    uiSymbol &= ~skipMask;
    CHECK(uiSymbol >= (1 << paramUpdated), "uiSymbol >= (1<<paramUpdated)");
  }

  if (paramUpdated > 0)
  {
    m_BinEncoder.encodeBinsEP(uiSymbol, paramUpdated);
  }
}

void CABACWriter::mvdCodingRemainder(const Mv& rMvd, const MvdSuffixInfo& si, int8_t imv)
{
  CHECK(si.horOffsetPrediction > (1<<15), "Attempt to write uninitialized horOffsetPrediction");
  CHECK(si.verOffsetPrediction > (1<<15), "Attempt to write uninitialized verOffsetPrediction");

  int horAbs = rMvd.getAbsHor();
  int verAbs = rMvd.getAbsVer();

  const int horParam = si.horPrefix;
  const int verParam = si.verPrefix;

  const unsigned int horOffsetPrediction = si.horOffsetPrediction;
  const unsigned int verOffsetPrediction = si.verOffsetPrediction;

  const int iEgcOffset = si.getEGCOffset();

  if (horParam >= 0 || verParam >= 0)
  {
    const int iHorMSBins = std::max(0, si.horOffsetPredictionNumBins);
    const int iVerMSBins = std::max(0, si.verOffsetPredictionNumBins);

    if (horParam >= 0)
    {
      for (int i = iHorMSBins - 1; i >= 0; --i)
      {
        int bin = (horOffsetPrediction >> i) & 1;

        const int prev2Bin = (i + 1 > iHorMSBins - 1) ? -1
                           : (i + 1 == iHorMSBins - 1) ? si.horSignHypMatch
                                       : /*otherwise*/ (horOffsetPrediction >> 1) & 1;
        const int prevBin  = (i == iHorMSBins - 1) ? si.horSignHypMatch : (horOffsetPrediction & 1);
        const int imvShift = MotionModelCheck::isAffine(si.m_motionModel) ? Mv::getImvPrecShiftAffineMvd(imv) : Mv::getImvPrecShiftMvd(imv);
        const int iCtxIdx  = DeriveCtx::ctxSmMvdBin(prev2Bin, prevBin, true, i + imvShift, si.m_motionModel);

        const unsigned int ctx = Ctx::MvsdIdxMVDMSB(iCtxIdx);
        m_BinEncoder.encodeBin((0 == bin) ? 1 : 0, ctx);
      }

      DTRACE(g_trace_ctx, D_SYNTAX, "Codeword for MVD suffix prediction bins for horizontal component: %d \n", horOffsetPrediction);
      DTRACE(g_trace_ctx, D_SYNTAX, "Number of MVD suffix prediction bins for horizontal component: %d \n", iHorMSBins);

      CHECK(horParam < 0, "horParam < 0");

      DTRACE(g_trace_ctx, D_SYNTAX, "Number of explicitly coded MVD horizontal suffix bins: %d \n", horParam - iHorMSBins + 1);
      xWriteMvdContextSuffix(horAbs - 1 - iEgcOffset, MVD_CODING_GOLOMB_ORDER, horParam, iHorMSBins);
    }
    if (verParam >= 0)
    {
      for (int i = iVerMSBins - 1; i >= 0; --i)
      {
        int bin = (verOffsetPrediction >> i) & 1;
        
        const int prev2Bin = (i + 1 > iVerMSBins - 1) ? -1
                           : (i + 1 == iVerMSBins - 1) ? si.verSignHypMatch
                                         : /*otherwise*/ (verOffsetPrediction >> 1) & 1;
        const int prevBin  = (i == iVerMSBins - 1) ? si.verSignHypMatch : (verOffsetPrediction & 1);
        const int imvShift = MotionModelCheck::isAffine(si.m_motionModel) ? Mv::getImvPrecShiftAffineMvd(imv) : Mv::getImvPrecShiftMvd(imv);
        const int iCtxIdx  = DeriveCtx::ctxSmMvdBin(prev2Bin, prevBin, false, i + imvShift, si.m_motionModel);

        const unsigned int ctx = Ctx::MvsdIdxMVDMSB(iCtxIdx);
        m_BinEncoder.encodeBin((0 == bin) ? 1 : 0, ctx);
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "Codeword for MVD suffix prediction bins for vertical component: %d \n", verOffsetPrediction);
      DTRACE(g_trace_ctx, D_SYNTAX, "Number of MVD suffix prediction bins for vertical component: %d \n", iVerMSBins);

      CHECK(verParam < 0, "verParam < 0");

      DTRACE(g_trace_ctx, D_SYNTAX, "Number of explicitly coded MVD vertical suffix bins: %d \n", verParam - iVerMSBins + 1);
      xWriteMvdContextSuffix(verAbs - 1 - iEgcOffset, MVD_CODING_GOLOMB_ORDER, verParam, iVerMSBins);
    }
  }
  else
  {
    if (horAbs > 1)
    {
      xWriteMvdContextSuffix(horAbs - 2, MVD_CODING_GOLOMB_ORDER, horParam, 0 );
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding_remainder() hor_suffix=%d \n", horAbs - 2);
    }
    if (verAbs > 1)
    {
      xWriteMvdContextSuffix(verAbs - 2, MVD_CODING_GOLOMB_ORDER, verParam, 0 );
      DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding_remainder() ver_suffix=%d \n", verAbs - 2);
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "mvd_coding_remainder() abs(mvd)=(%d,%d) \n", horAbs, verAbs);
  }

}
#else
void CABACWriter::mvd_coding(const Mv& rMvd, int8_t imv
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  , bool codeSign
#endif
#if JVET_AA0070_RRIBC
  , const int &rribcFlipType
#endif
)
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();
  if ( imv > 0 )
  {
    CHECK((horMvd % 2) != 0 && (verMvd % 2) != 0, "IMV: MVD is not a multiple of 2");
    horMvd >>= 1;
    verMvd >>= 1;
    if (imv < IMV_HPEL)
    {
      CHECK((horMvd % 2) != 0 && (verMvd % 2) != 0, "IMV: MVD is not a multiple of 4");
      horMvd >>= 1;
      verMvd >>= 1;
      if (imv == IMV_4PEL)//IMV_4PEL
      {
        CHECK((horMvd % 4) != 0 && (verMvd % 4) != 0, "IMV: MVD is not a multiple of 16");
        horMvd >>= 2;
        verMvd >>= 2;
      }
    }
  }
  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );


  // abs_mvd_greater0_flag[ 0 | 1 ]
#if JVET_AA0070_RRIBC
  if (rribcFlipType != 2)
  {
    m_BinEncoder.encodeBin((horAbs > 0), Ctx::Mvd());
  }
  if (rribcFlipType != 1)
  {
    m_BinEncoder.encodeBin((verAbs > 0), Ctx::Mvd());
  }
#else
  m_BinEncoder.encodeBin((horAbs > 0), Ctx::Mvd());
  m_BinEncoder.encodeBin((verAbs > 0), Ctx::Mvd());
#endif

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    m_BinEncoder.encodeBin( (horAbs > 1), Ctx::Mvd(1) );
  }
  if( verAbs > 0 )
  {
    m_BinEncoder.encodeBin( (verAbs > 1), Ctx::Mvd(1) );
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if( horAbs > 0 )
  {
    if( horAbs > 1 )
    {
      m_BinEncoder.encodeRemAbsEP(horAbs - 2, 1, 0, MV_BITS - 1);
    }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (codeSign)
#endif
    m_BinEncoder.encodeBinEP( (horMvd < 0) );
  }
  if( verAbs > 0 )
  {
    if( verAbs > 1 )
    {
      m_BinEncoder.encodeRemAbsEP(verAbs - 2, 1, 0, MV_BITS - 1);
    }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (codeSign)
#endif
    m_BinEncoder.encodeBinEP( (verMvd < 0) );
  }
}
#endif

#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AC0104_IBC_BVD_PREDICTION
unsigned CABACWriter::xWriteBvdContextPrefix(unsigned uiSymbol, unsigned ctxT, int offset, int param )
{ 
  unsigned bins = 0;
  unsigned numBins = 0;
  while (uiSymbol >= (unsigned)(1 << param))
  {
    bins <<= 1;
    bins++;
    numBins++;
    uiSymbol -= 1 << param;
    param++;
  }

  bins <<= 1;
  numBins++;

  unsigned temp = 0;
  unsigned bitCount = 0;
  for (int i = numBins - 1; i >= 0; i--)
  {
    temp = bins >> i;
    if (bitCount >= ctxT)
    {
      m_BinEncoder.encodeBinEP(temp);
    }
    else
    {
      m_BinEncoder.encodeBin(temp, Ctx::Bvd(offset + bitCount + 1));
    }
    bins -= (temp << i);
    bitCount++;
  }
  return numBins - 1; // less by 1 as compared to what xReadBvdContextPrefix() returns
}
void CABACWriter::xWriteBvdContextSuffix(unsigned uiSymbol, int param, int paramUpdated, int numSkipMSB )
{
  unsigned numBins = 0;
  while (uiSymbol >= (unsigned) (1 << param))
  {
    numBins++;
    uiSymbol -= 1 << param;
    param++;
  }
  numBins++;
  paramUpdated++;

  CHECK(paramUpdated != numBins, "Enc side prefix bits check error");

  if (0 != numSkipMSB)
  {
    CHECK(paramUpdated == 0, "paramUpdated = 0");
    paramUpdated-= numSkipMSB;
    unsigned skipMask = ( 1 << (paramUpdated + numSkipMSB)) -1 - ((1 << (paramUpdated)) - 1);
    uiSymbol &= ~skipMask;
    CHECK(uiSymbol >= (1 << paramUpdated), "uiSymbol >= (1<<paramUpdated)");
  }

  if (paramUpdated > 0)
  {
    CHECK(uiSymbol >= (1 << (paramUpdated+1)), "uiSymbol >= (1<<paramUpdated)");
    m_BinEncoder.encodeBinsEP(uiSymbol, paramUpdated);
  }
}
#endif

void CABACWriter::xWriteBvdContext(unsigned uiSymbol, unsigned ctxT, int offset, int param)
{
  unsigned bins = 0;
  unsigned numBins = 0;
  while (uiSymbol >= (unsigned)(1 << param))
  {
    bins <<= 1;
    bins++;
    numBins++;
    uiSymbol -= 1 << param;
    param++;
  }

  bins <<= 1;
  numBins++;



  unsigned temp = 0;
  unsigned bitCount = 0;
  for (int i = numBins-1; i >=0 ; i--)
  {
    temp = bins>>i;
    if (bitCount >= ctxT)
    {
      m_BinEncoder.encodeBinEP(temp);
    }
    else
    {
      m_BinEncoder.encodeBin(temp, Ctx::Bvd(offset + bitCount + 1));
    }
    bins -= (temp << i);
    bitCount++;
  }
  m_BinEncoder.encodeBinsEP(uiSymbol, param);
}

#endif


#if JVET_AC0104_IBC_BVD_PREDICTION
void CABACWriter::bvdCodingRemainder(const Mv& rMvd, const MvdSuffixInfo& si, int8_t imv )
{
  int horAbs = rMvd.getAbsHor();
  int verAbs = rMvd.getAbsVer();

  const unsigned int horOffsetPrediction = si.horOffsetPrediction;
  const unsigned int verOffsetPrediction = si.verOffsetPrediction;

  const int horParam = si.horPrefix;
  const int verParam = si.verPrefix;

  if (horParam >= 0 || verParam >= 0)
  {
    const int iHorMSBins = si.horOffsetPredictionNumBins;
    const int iVerMSBins = si.verOffsetPredictionNumBins;

    if (horParam >= 0)
    {
      for (int i = iHorMSBins - 1; i >= 0; --i)
      {
        const int          bin          = (horOffsetPrediction >> i) & 1;
        const int          prev2Bin     = (i + 1 > iHorMSBins - 1) ? -1 :
                                                                     (i + 1 == iHorMSBins - 1) ? si.horSignHypMatch :
                                                                      /*otherwise*/              (horOffsetPrediction>> (i + 2)) & 1;
        const int          prevBin      = (i == iHorMSBins - 1) ? si.horSignHypMatch : ((horOffsetPrediction >> (i+1)) & 1);
        const int          imvShift     = Mv::getImvPrecShift(imv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                                            , si.isFracBvEnabled
#endif
        );
        const int          iCtxIdx      = DeriveCtx::CtxSmBvdBin(prev2Bin, prevBin, true, i + imvShift);
        const unsigned int ctx          = Ctx::MvsdIdxBVDMSB(iCtxIdx);

        m_BinEncoder.encodeBin((0 == bin) ? 1 : 0, ctx);
      }

      xWriteBvdContextSuffix(horAbs - 1, BVD_CODING_GOLOMB_ORDER, horParam, iHorMSBins );
    }
    if (verParam >= 0)
    {
      for (int i = iVerMSBins - 1; i >= 0; --i)
      {
        const int          bin      = (verOffsetPrediction >> i) & 1;
        const int          prev2Bin = (i + 1 > iVerMSBins - 1) ? -1 :
                                                                 (i + 1 == iVerMSBins - 1) ? si.verSignHypMatch :
                                                                 /*otherwise*/               (verOffsetPrediction>> (i + 2)) & 1;
        const int          prevBin  = (i == iVerMSBins - 1) ? si.verSignHypMatch : ((verOffsetPrediction>>(i+1)) & 1);
        const int          imvShift = Mv::getImvPrecShift(imv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                                        , si.isFracBvEnabled
#endif
        );
        const int          iCtxIdx  = DeriveCtx::CtxSmBvdBin(prev2Bin, prevBin, false, i + imvShift);
        const unsigned int ctx      = Ctx::MvsdIdxBVDMSB(iCtxIdx);

        m_BinEncoder.encodeBin((0 == bin) ? 1 : 0, ctx);
      }

      xWriteBvdContextSuffix(verAbs - 1, BVD_CODING_GOLOMB_ORDER, verParam, iVerMSBins);
    }
  }
  else
  {
    if (horAbs != 0)
    {
      xWriteBvdContextSuffix(horAbs - 1, BVD_CODING_GOLOMB_ORDER, horParam, 0 );
    }
    if (verAbs != 0)
    {
      xWriteBvdContextSuffix(verAbs - 1, BVD_CODING_GOLOMB_ORDER, verParam, 0 );
    }
  }
}
#endif

#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AA0070_RRIBC
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACWriter::bvdCoding(const Mv &rMvd, const bool useBvdPred, const bool useBvpCluster, int bvOneZeroComp,
                            int bvZeroCompDir, const int &rribcFlipType)
#else
void CABACWriter::bvdCoding(const Mv& rMvd, const bool useBvdPred, const int& rribcFlipType)
#endif
#else 
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACWriter::bvdCoding(const Mv &rMvd, const bool useBvpCluster, int bvOneZeroComp, int bvZeroCompDir,
                            const int &rribcFlipType)
#else
void CABACWriter::bvdCoding(const Mv& rMvd, const int& rribcFlipType)
#endif
#endif
#else
#if JVET_AC0104_IBC_BVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACWriter::bvdCoding(const Mv &rMvd, const bool useBvdPred, const bool useBvpCluster, int bvOneZeroComp,
                            int bvZeroCompDir)
#else
void CABACWriter::bvdCoding(const Mv& rMvd, const bool useBvdPred)
#endif
#else 
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACWriter::bvdCoding(const Mv &rMvd, const bool useBvpCluster, int bvOneZeroComp, int bvZeroCompDir)
#else
void CABACWriter::bvdCoding(const Mv& rMvd)
#endif
#endif
#endif
{
  int       horMvd = rMvd.getHor();
  int       verMvd = rMvd.getVer();

  unsigned  horAbs  = unsigned( horMvd < 0 ? -horMvd : horMvd );
  unsigned  verAbs  = unsigned( verMvd < 0 ? -verMvd : verMvd );

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  if (useBvpCluster)
  {
    if (bvOneZeroComp)
    {
      if (bvZeroCompDir == 1)
      {
        m_BinEncoder.encodeBin((horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET));
      }
      if (bvZeroCompDir == 2)
      {
        m_BinEncoder.encodeBin((verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET));
      }
    }
    else
    {
      m_BinEncoder.encodeBin((horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET));
      m_BinEncoder.encodeBin((verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET));
    }
  }
  else 
  {
#if JVET_AA0070_RRIBC
    if (rribcFlipType != 2)
    {
      m_BinEncoder.encodeBin((horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET));
    }
    if (rribcFlipType != 1)
    {
      m_BinEncoder.encodeBin((verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET));
    }
#else
    m_BinEncoder.encodeBin((horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET));
    m_BinEncoder.encodeBin((verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET));
#endif
  }
#else
#if JVET_AA0070_RRIBC
  if (rribcFlipType != 2)
  {
    m_BinEncoder.encodeBin((horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  }
  if (rribcFlipType != 1)
  {
    m_BinEncoder.encodeBin((verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET));
  }
#else
  m_BinEncoder.encodeBin( (horAbs > 0), Ctx::Bvd(HOR_BVD_CTX_OFFSET) );
  m_BinEncoder.encodeBin( (verAbs > 0), Ctx::Bvd(VER_BVD_CTX_OFFSET) );
#endif
#endif


#if JVET_AC0104_IBC_BVD_PREDICTION
  if (useBvdPred)
  {
    if (horAbs)
    {
      xWriteBvdContextPrefix(horAbs - 1, NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
    }
    if (verAbs)
    {
      xWriteBvdContextPrefix(verAbs - 1, NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
    }
  }
  else
  {
#endif
    if (horAbs > 0)
    {
      xWriteBvdContext(horAbs - 1, NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      if (useBvpCluster)
      {
        if (!bvOneZeroComp)   // not send the sign
        {
          m_BinEncoder.encodeBinEP((horMvd < 0));
        }
      }
      else
      {
        m_BinEncoder.encodeBinEP((horMvd < 0));
      }
#else
      m_BinEncoder.encodeBinEP((horMvd < 0));
#endif
    }
    if (verAbs > 0)
    {
      xWriteBvdContext(verAbs - 1, NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
      if (useBvpCluster)
      {
        if (!bvOneZeroComp)   // not send the sign
        {
          m_BinEncoder.encodeBinEP((verMvd < 0));
        }
      }
      else
      {
        m_BinEncoder.encodeBinEP((verMvd < 0));
      }
#else
      m_BinEncoder.encodeBinEP((verMvd < 0));
#endif
    }
#if JVET_AC0104_IBC_BVD_PREDICTION
  }
#endif
}
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
void CABACWriter::mvsdIdxFunc(const PredictionUnit &pu, RefPicList eRefList)
{
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (!pu.isMvdPredApplicable())
  {
    return;
  }
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu) && !pu.isBvdPredApplicable())
  {
    return;
  }
#endif

  if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pu.interDir == 3)
  {
    return;
  }
  if (pu.cu->smvdMode && eRefList == REF_PIC_LIST_1)
  {
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER && !JVET_AD0140_MVD_PREDICTION
  if (PU::useRefPairList(pu))
  {
    if (pu.interDir == 3 && eRefList == REF_PIC_LIST_1 && (pu.mvd[0].getHor() || pu.mvd[0].getVer()))
    {
      if (pu.mvd[eRefList].getHor())
      {
        m_BinEncoder.encodeBinEP(pu.mvd[eRefList].getHor() < 0);
      }
      if (pu.mvd[eRefList].getVer())
      {
        m_BinEncoder.encodeBinEP(pu.mvd[eRefList].getVer() < 0);
      }
      return;
    }
  }
#endif

  Mv  trMv  = Mv(pu.mvd[eRefList].getAbsHor(), pu.mvd[eRefList].getAbsVer());
  int Thres = THRES_TRANS;

  int mvsdIdx = pu.mvsdIdx[eRefList];

#if !JVET_AD0140_MVD_PREDICTION
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC 
  if (pu.isBvpClusterApplicable())
  {
    if (trMv != Mv(0, 0) && pu.cu->rribcFlipType == 0)
    {
      CHECK(mvsdIdx == -1, "mvsdIdx == -1 for transMv");
    }
  }
  else
  {
    if (trMv != Mv(0, 0))
    {
      CHECK(mvsdIdx == -1, "mvsdIdx == -1 for transMv");
    }
  }
#else
  if (trMv != Mv(0, 0))
  {
    CHECK(mvsdIdx == -1, "mvsdIdx == -1 for transMv");
  }
#endif 
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  if (CU::isIBC(*pu.cu))
  {
    Mv mvd = pu.mvd[eRefList];
    mvd.changeIbcPrecInternal2Amvr(pu.cu->imv);

    MvdSuffixInfo si = pu.bvdSuffixInfo;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    si.isFracBvEnabled = pu.cs->sps->getIBCFracFlag();
#endif
    si.initPrefixes(mvd, pu.cu->imv, false);
    if (si.horPrefix >= 0)
    {
      CHECK(si.horOffsetPredictionNumBins != pu.bvdSuffixInfo.horOffsetPredictionNumBins, "mismatch in RDO and writing");
      CHECK(si.horPrefix != pu.bvdSuffixInfo.horPrefix, "mismatch in RDO and writing");
    }
    if (si.verPrefix >= 0)
    {
      CHECK(si.verPrefix != pu.bvdSuffixInfo.verPrefix, "mismatch in RDO and writing");
      CHECK(si.verOffsetPredictionNumBins != pu.bvdSuffixInfo.verOffsetPredictionNumBins, "mismatch in RDO and writing");
    }

    int horPrefix = si.horPrefix;
    int verPrefix = si.verPrefix;

    si.horSignHypMatch = -1;
    si.verSignHypMatch = -1;

    if (horPrefix < 0 && verPrefix < 0)
    {
      return;
    }
    Mv trMv = Mv(horPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix),
                 verPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix));
    trMv.changeTransPrecAmvr2Internal(pu.cu->imv);
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV && JVET_AA0070_RRIBC
    if (0 != pu.cu->rribcFlipType && pu.isBvpClusterApplicable())
    {
      bvdCodingRemainder(mvd, si, pu.cu->imv);
      return;
    }
#endif
    if (pu.mvd[eRefList].getHor())
    {
      if (pu.bvdSuffixInfo.horEncodeSignInEP)
      {
        unsigned bin = pu.mvd[eRefList].getHor() < 0 ? 1 : 0;
        m_BinEncoder.encodeBinEP(bin);
      }
      else
      {
        uint8_t ctxId = (trMv.getHor() <= Thres) ? 0 : 1;
        int     bin   = mvsdIdx & 1;

#if JVET_AD0140_MVD_PREDICTION
        m_BinEncoder.encodeBin(bin, Ctx::MvsdIdxIBC(ctxId));
#else
        m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
#endif


        si.horSignHypMatch = 0 == bin;
        mvsdIdx >>= 1;
      }
    } // if (pu.mvd[eRefList].getHor())

    if (pu.mvd[eRefList].getVer())
    {
      if (pu.bvdSuffixInfo.verEncodeSignInEP)
      {
        unsigned bin = pu.mvd[eRefList].getVer() < 0 ? 1 : 0;
        m_BinEncoder.encodeBinEP(bin);
      }
      else
      {
        uint8_t ctxId = (trMv.getVer() <= Thres) ? 0 : 1;
        int     bin   = mvsdIdx & 1;

#if JVET_AD0140_MVD_PREDICTION
        m_BinEncoder.encodeBin(bin, Ctx::MvsdIdxIBC(ctxId));
#else
        m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
#endif


        si.verSignHypMatch = 0 == bin;
        mvsdIdx >>= 1;
      }
    } // if (pu.mvd[eRefList].getVer())
    bvdCodingRemainder(mvd, si, pu.cu->imv );
    return;
  }
#endif
#if JVET_AD0140_MVD_PREDICTION
  MvdSuffixInfo si = pu.mvdSuffixInfo.mvBins[eRefList][0];

  const int horPrefix = si.horPrefix;
  const int verPrefix = si.verPrefix;

  trMv = Mv(horPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix),
            verPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix));

  trMv.changeTransPrecAmvr2Internal(pu.cu->imv);

  if (pu.mvd[eRefList].getHor())
  {
    if( si.horEncodeSignInEP )
    {
      unsigned bin = pu.mvd[eRefList].getHor() < 0 ? 1 : 0;
      m_BinEncoder.encodeBinEP( bin );
    }
    else
    {
      uint8_t ctxId = ( trMv.getHor() <= Thres ) ? 0 : 1;
      int     bin = mvsdIdx & 1;
      m_BinEncoder.encodeBin( bin, Ctx::MvsdIdx( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "mvsd hor: bin=%d, ctx=%d \n", bin, ctxId );
      mvsdIdx >>= 1;
    }
  }
  if (pu.mvd[eRefList].getVer())
  {
    if( si.verEncodeSignInEP )
    {
      unsigned bin = pu.mvd[eRefList].getVer() < 0 ? 1 : 0;
      m_BinEncoder.encodeBinEP( bin );
    }
    else
    {
      uint8_t ctxId = ( trMv.getVer() <= Thres ) ? 0 : 1;
      int     bin = mvsdIdx & 1;
      m_BinEncoder.encodeBin( bin, Ctx::MvsdIdx( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "mvsd ver: bin=%d, ctx=%d \n", bin, ctxId );
      mvsdIdx >>= 1;
    }
  }

  if (eRefList == REF_PIC_LIST_1 || pu.interDir == 1
      || (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_0 && pu.interDir == 3)
      || pu.cu->smvdMode
      )
  {
    bool writeL0Suffixes = pu.interDir != 2;
    bool writeL1Suffixes = pu.interDir != 1 && !( pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3 );

    if( pu.cu->smvdMode )
    {
      CHECK( pu.interDir != 3, "SMVD mode should be B-prediction" );
      writeL0Suffixes &= ( eRefList == 0 );
      writeL1Suffixes = false; // &= (eRefList == 1);
      const int maxNumBins = MvdSuffixInfoMv::getBinBudgetForPrediction( pu.Y().width, pu.Y().height, pu.cu->imv );
      const auto& si = pu.mvdSuffixInfo.mvBins[0][0];
      CHECK( si.getNumBinsOfSignsAndSuffixes() > maxNumBins, "Bin budget exceeded" );
    }

    for( int i = REF_PIC_LIST_0; i < NUM_REF_PIC_LIST_01; ++i )   // read MVD suffixes
    {
      Mv mvd = pu.mvd[i];
      mvd.changeTransPrecInternal2Amvr( pu.cu->imv );

      if( ( i == 0 && !writeL0Suffixes ) || ( i == 1 && !writeL1Suffixes ) || ( !mvd.getHor() && !mvd.getVer() ) )
      {
        continue;
      }

      MvdSuffixInfoMv currSI = pu.mvdSuffixInfo;

      int verPrefix = currSI.mvBins[i][0].verPrefix;
      int horPrefix = currSI.mvBins[i][0].horPrefix;

      CHECK( 0 > currSI.actualMvCompNum[i], "Uninitialized suffix info" );

      if( horPrefix >= 0 || verPrefix >= 0 )
      {
        mvdCodingRemainder( mvd, currSI.mvBins[i][0], pu.cu->imv );
      }
    }
  }
  else
  {
    Mv mvd = pu.mvd[eRefList];
    mvd.changeTransPrecInternal2Amvr( pu.cu->imv );
  }
#elif JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (pu.mvd[eRefList].getHor())
  {
    uint8_t ctxId = (trMv.getHor() <= Thres) ? 0 : 1;
    int bin = mvsdIdx & 1;
    m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
    mvsdIdx >>= 1;
  }
  if (pu.mvd[eRefList].getVer())
  {
    uint8_t ctxId = (trMv.getVer() <= Thres) ? 0 : 1;

    int bin = mvsdIdx & 1;
    m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
    mvsdIdx >>= 1;
  }
#endif


}
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
void CABACWriter::mvsdAffineIdxFunc(const PredictionUnit &pu, RefPicList eRefList)
{
  if (!pu.cu->affine)
  {
    return;
  }
  if (!pu.isMvdPredApplicable())
  {
    return;
  }
  if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pu.interDir == 3)
  {
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER && !JVET_AD0140_MVD_PREDICTION
  if (PU::useRefPairList(pu))
  {
    if (pu.interDir == 3 && eRefList == REF_PIC_LIST_1 &&
      (pu.mvdAffi[0][0].getHor() || pu.mvdAffi[0][0].getVer() ||
        pu.mvdAffi[0][1].getHor() || pu.mvdAffi[0][1].getVer() ||
        (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvdAffi[0][2].getHor() || pu.mvdAffi[0][2].getVer()))
        )
      )
    {
      if (pu.mvdAffi[eRefList][0].getHor())
      {
        m_BinEncoder.encodeBinEP(pu.mvdAffi[eRefList][0].getHor() < 0);
      }
      if (pu.mvdAffi[eRefList][0].getVer())
      {
        m_BinEncoder.encodeBinEP(pu.mvdAffi[eRefList][0].getVer() < 0);
      }
      if (pu.mvdAffi[eRefList][1].getHor())
      {
        m_BinEncoder.encodeBinEP(pu.mvdAffi[eRefList][1].getHor() < 0);
      }
      if (pu.mvdAffi[eRefList][1].getVer())
      {
        m_BinEncoder.encodeBinEP(pu.mvdAffi[eRefList][1].getVer() < 0);
      }
      if (pu.cu->affineType == AFFINEMODEL_6PARAM)
      {
        if (pu.mvdAffi[eRefList][2].getHor())
        {
          m_BinEncoder.encodeBinEP(pu.mvdAffi[eRefList][2].getHor() < 0);
        }
        if (pu.mvdAffi[eRefList][2].getVer())
        {
          m_BinEncoder.encodeBinEP(pu.mvdAffi[eRefList][2].getVer() < 0);
        }
      }
      return;
    }
  }
#endif

  Mv AffMv[3];
  AffMv[0] = Mv(pu.mvdAffi[eRefList][0].getAbsHor(), pu.mvdAffi[eRefList][0].getAbsVer());
  AffMv[1] = Mv(pu.mvdAffi[eRefList][1].getAbsHor(), pu.mvdAffi[eRefList][1].getAbsVer());
  AffMv[2] = Mv(pu.mvdAffi[eRefList][2].getAbsHor(), pu.mvdAffi[eRefList][2].getAbsVer());
  int Thres = THRES_AFFINE;
  
  int mvsdIdx = pu.mvsdIdx[eRefList];
#if JVET_AD0140_MVD_PREDICTION
  const int numAffinesInRpl = 2 + (pu.cu->affineType == AFFINEMODEL_6PARAM);
  for (int i = 0; i < numAffinesInRpl; ++i)
  {
    MvdSuffixInfo si = pu.mvdSuffixInfo.mvBins[eRefList][i]; //create modifiable copy

    // get signs
    const int horPrefix = si.horPrefix;
    const int verPrefix = si.verPrefix;

    Mv TrMv = Mv(horPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(horPrefix),
                 verPrefix < 0 ? 0 : MvdSuffixInfo::xGetGolombGroupMinValue(verPrefix));

    TrMv.changeAffinePrecAmvr2Internal(pu.cu->imv);

    if (pu.mvdAffi[eRefList][i].getHor())
    {
      if (si.horEncodeSignInEP)
      {
        unsigned bin = pu.mvdAffi[eRefList][i].getHor() < 0 ? 1 : 0;
        m_BinEncoder.encodeBinEP(bin);
      }
      else
      {
        uint8_t ctxId = (TrMv.getHor() <= Thres) ? 2 : 3;
        int     bin   = mvsdIdx & 1;
        m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
        mvsdIdx >>= 1;
      }
    }
    if (pu.mvdAffi[eRefList][i].getVer())
    {
      if (si.verEncodeSignInEP)
      {
        unsigned bin = pu.mvdAffi[eRefList][i].getVer() < 0 ? 1 : 0;
        m_BinEncoder.encodeBinEP(bin);
      }
      else
      {
        uint8_t ctxId = (TrMv.getVer() <= Thres) ? 2 : 3;
        int     bin   = mvsdIdx & 1;
        m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
        mvsdIdx >>= 1;
      }
    }
  }

  // calc and write suffix bins
  if (eRefList == REF_PIC_LIST_1 || pu.interDir == 1
      || (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_0 && pu.interDir == 3))   // current RPL = L1 or there is only L0: now we can specify num of predicted suffix bins
  {
    bool writeL0Suffixes = pu.interDir != 2;
    bool writeL1Suffixes = pu.interDir != 1 && !(pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3);

    // write suffixes
    for (int i = REF_PIC_LIST_0; i < NUM_REF_PIC_LIST_01; ++i)   // read MVD suffixes
    {
      for (int j = 0; j < numAffinesInRpl; ++j)
      {
        Mv mvdAffine = pu.mvdAffi[i][j];
        mvdAffine.changeAffinePrecInternal2Amvr(pu.cu->imv);

        if ((i == 0 && !writeL0Suffixes) || (i == 1 && !writeL1Suffixes) || (!mvdAffine.getHor() && !mvdAffine.getVer()))
        {
          continue;
        }

        auto si = pu.mvdSuffixInfo.mvBins[i][j];
        int verPrefix = si.verPrefix;
        int horPrefix = si.horPrefix;

        if (horPrefix >= 0 || verPrefix >= 0)
        {
          mvdCodingRemainder(mvdAffine, si, pu.cu->imv);
        }
      }
    }
  }
#else
  if (AffMv[0] != Mv(0, 0) || AffMv[1] != Mv(0, 0) || (pu.cu->affineType == AFFINEMODEL_6PARAM && AffMv[2] != Mv(0, 0)))
  {
    CHECK(mvsdIdx == -1, "mvsdIdx == -1 for AffineMv");
  }
  if (pu.mvdAffi[eRefList][0].getHor())
  {
    uint8_t ctxId = (AffMv[0].getHor() <= Thres) ? 2 : 3;
    int bin = mvsdIdx & 1;
    m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
    mvsdIdx >>= 1;
  }
  if (pu.mvdAffi[eRefList][0].getVer())
  {
    uint8_t ctxId = (AffMv[0].getVer() <= Thres) ? 2 : 3;
    int bin = mvsdIdx & 1;
    m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
    mvsdIdx >>= 1;
  }
  if (pu.mvdAffi[eRefList][1].getHor())
  {
    uint8_t ctxId = (AffMv[1].getHor() <= Thres) ? 2 : 3;
    int bin = mvsdIdx & 1;
    m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
    mvsdIdx >>= 1;
  }
  if (pu.mvdAffi[eRefList][1].getVer())
  {
    uint8_t ctxId = (AffMv[1].getVer() <= Thres) ? 2 : 3;
    int bin = mvsdIdx & 1;
    m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
    mvsdIdx >>= 1;
  }
  if (pu.cu->affineType == AFFINEMODEL_6PARAM)
  {
    if (pu.mvdAffi[eRefList][2].getHor())
    {
      uint8_t ctxId = (AffMv[2].getHor() <= Thres) ? 2 : 3;
      int bin = mvsdIdx & 1;
      m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
      mvsdIdx >>= 1;
    }
    if (pu.mvdAffi[eRefList][2].getVer())
    {
      uint8_t ctxId = (AffMv[2].getVer() <= Thres) ? 2 : 3;
      int bin = mvsdIdx & 1;
      m_BinEncoder.encodeBin(bin, Ctx::MvsdIdx(ctxId));
      mvsdIdx >>= 1;
    }
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "mvsd_affine_idx_func() mvsdIdx=%d\n", pu.mvsdIdx[eRefList]); // eRefList can have enc-dec mismatch
}
#endif



//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACWriter::transform_unit(const TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter
#if JVET_AE0102_LFNST_CTX
  , const bool codeTuCoeff
#endif
)
{
  const CodingStructure&  cs = *tu.cs;
  const CodingUnit&       cu = *tu.cu;
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;
  ChromaCbfs              chromaCbfs;

#if JVET_AE0102_LFNST_CTX
  if ( codeTuCoeff == false )
  {
#endif
  CHECK(tu.depth != trDepth, " transform unit should be not be futher partitioned");

  // cbf_cb & cbf_cr
  if (area.chromaFormat != CHROMA_400)
  {
    const bool              chromaCbfISP = area.blocks[COMPONENT_Cb].valid() && cu.ispMode;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (area.blocks[COMPONENT_Cb].valid() && (!CS::isDualITree(cs) || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#else
    if (area.blocks[COMPONENT_Cb].valid() && (!cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#endif
    {
      unsigned cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
      {
        chromaCbfs.Cb = TU::getCbfAtDepth( tu, COMPONENT_Cb, trDepth );

        if( !(cu.sbtInfo && tu.noResidual) )
        {
          cbf_comp( cs, chromaCbfs.Cb, area.blocks[COMPONENT_Cb], cbfDepth );
        }
      }

      {
        chromaCbfs.Cr = TU::getCbfAtDepth( tu, COMPONENT_Cr, trDepth );

        if( !(cu.sbtInfo && tu.noResidual) )
        {
          cbf_comp( cs, chromaCbfs.Cr, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb );
        }
      }
    }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    else if (CS::isDualITree(cs))
#else
    else if (cu.isSepTree())
#endif
    {
      chromaCbfs = ChromaCbfs(false);
    }
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  else if (CS::isDualITree(cs))
#else
  else if (cu.isSepTree())
#endif
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      CHECK(!TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be true for inter units with no chroma coeffs");
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      CHECK(TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be false for inter sbt no-residual tu");
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      CHECK(!TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "Luma cbf must be true for inter sbt residual tu");
    }
    else
    {
      bool lumaCbfIsInferredACT = (cu.colorTransform && cu.predMode == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat));
      CHECK(lumaCbfIsInferredACT && !TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), "adaptive color transform cannot have all zero coefficients");
      bool lastCbfIsInferred = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
      bool previousCbf = false;
      bool rootCbfSoFar = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < subTuCounter; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMPONENT_Y, partitioner.currTrDepth);
        }
      }
      if (!lastCbfIsInferred)
      {
        cbf_comp(cs, TU::getCbfAtDepth(tu, COMPONENT_Y, trDepth), tu.Y(), trDepth, previousCbf, cu.ispMode);
      }
    }
  }
  bool lumaOnly  = (cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid());
  bool cbf[3]    = { TU::getCbf(tu, COMPONENT_Y), chromaCbfs.Cb, chromaCbfs.Cr };
  bool cbfLuma   = (cbf[COMPONENT_Y] != 0);
  bool cbfChroma = false;

  if (!lumaOnly)
  {
    if (tu.blocks[COMPONENT_Cb].valid())
    {
      cbf[COMPONENT_Cb] = TU::getCbf(tu, COMPONENT_Cb);
      cbf[COMPONENT_Cr] = TU::getCbf(tu, COMPONENT_Cr);
    }
    cbfChroma = (cbf[COMPONENT_Cb] || cbf[COMPONENT_Cr]);
  }

#if TU_256
    if ((cu.lwidth() > MAX_TB_SIZEY || cu.lheight() > MAX_TB_SIZEY || cbfLuma || cbfChroma) &&
#else
    if ((cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma) &&
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    (!CS::isDualITree(*tu.cs) || isLuma(tu.chType)))
#else
      (!tu.cu->isSepTree() || isLuma(tu.chType)))
#endif
    {
      if (cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
      {
        cu_qp_delta(cu, cuCtx.qp, cu.qp);
        cuCtx.qp = cu.qp;
        cuCtx.isDQPCoded = true;
      }
    }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))   // !DUAL_TREE_LUMA
#else
    if (!cu.isSepTree() || isChroma(tu.chType))   // !DUAL_TREE_LUMA
#endif
    {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      SizeType channelWidth = !CS::isDualITree(*tu.cs) ? cu.lwidth() : cu.chromaSize().width;
      SizeType channelHeight = !CS::isDualITree(*tu.cs) ? cu.lheight() : cu.chromaSize().height;
#else
      SizeType channelWidth = !cu.isSepTree() ? cu.lwidth() : cu.chromaSize().width;
      SizeType channelHeight = !cu.isSepTree() ? cu.lheight() : cu.chromaSize().height;
#endif
#if TU_256
      if (cu.cs->slice->getUseChromaQpAdj() && (channelWidth > MAX_TB_SIZEY || channelHeight > MAX_TB_SIZEY || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
#else
      if (cu.cs->slice->getUseChromaQpAdj() && (channelWidth > 64 || channelHeight > 64 || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
#endif
      {
        cu_chroma_qp_offset(cu);
        cuCtx.isChromaQpAdjCoded = true;
      }
    }

#if JVET_AF0073_INTER_CCP_MERGE
  if ( !lumaOnly )
  {
    interCcpMerge( tu );
  }
#endif
#if JVET_AE0059_INTER_CCCM
  if ( !lumaOnly )
  {
    interCccm( tu );
  }
#endif
  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( cbf[COMPONENT_Cb] ? 2 : 0 ) + ( cbf[COMPONENT_Cr] ? 1 : 0 ) );
  }

#if JVET_AE0102_LFNST_CTX
  }
#endif

#if JVET_AE0102_LFNST_CTX
     if ( TU::getCbf(tu, COMPONENT_Y) != 0 )
#else
    if( cbfLuma )
#endif
    {
      residual_coding( tu, COMPONENT_Y, &cuCtx
#if JVET_AE0102_LFNST_CTX
        , codeTuCoeff
#endif      
      );
    }
#if JVET_AE0102_LFNST_CTX
    if ( !(cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid()))
#else
    if( !lumaOnly )
#endif
    {
      for( ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID( compID + 1 ) )
      {
#if JVET_AE0102_LFNST_CTX
        if ( TU::getCbf(tu, compID) )
#else
        if( cbf[ compID ] )
#endif
        {
          residual_coding( tu, compID, &cuCtx
#if JVET_AE0102_LFNST_CTX
            , codeTuCoeff
#endif          
          );
      }
    }
  }
}

void CABACWriter::cu_qp_delta( const CodingUnit& cu, int predQP, const int8_t qp )
{
  CHECK(!( predQP != std::numeric_limits<int>::max()), "Unspecified error");
  int       DQp         = qp - predQP;
  int       qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
  DQp                   = ( DQp + (MAX_QP + 1) + (MAX_QP + 1) / 2 + qpBdOffsetY + (qpBdOffsetY / 2)) % ((MAX_QP + 1) + qpBdOffsetY) - (MAX_QP + 1) / 2 - (qpBdOffsetY / 2);
  unsigned  absDQP      = unsigned( DQp < 0 ? -DQp : DQp );
  unsigned  unaryDQP    = std::min<unsigned>( absDQP, CU_DQP_TU_CMAX );

  unary_max_symbol( unaryDQP, Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( absDQP >= CU_DQP_TU_CMAX )
  {
    exp_golomb_eqprob( absDQP - CU_DQP_TU_CMAX, CU_DQP_EG_k );
  }
  if( absDQP > 0 )
  {
    m_BinEncoder.encodeBinEP( DQp < 0 );
  }

  DTRACE_COND( ( isEncoding() ), g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACWriter::cu_chroma_qp_offset( const CodingUnit& cu )
{
  // cu_chroma_qp_offset_flag
  unsigned qpAdj = cu.chromaQpAdj;
  if( qpAdj == 0 )
  {
    m_BinEncoder.encodeBin( 0, Ctx::ChromaQpAdjFlag() );
  }
  else
  {
    m_BinEncoder.encodeBin( 1, Ctx::ChromaQpAdjFlag() );
    int length = cu.cs->pps->getChromaQpOffsetListLen();
    if( length > 1 )
    {
      unary_max_symbol( qpAdj-1, Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_chroma_qp_offset() chroma_qp_adj=%d\n", cu.chromaQpAdj);
}





//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    void        transform_skip_flag     ( tu, compID )
//    void        last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACWriter::joint_cb_cr( const TransformUnit& tu, const int cbfMask )
{
  if ( !tu.cu->slice->getSPS()->getJointCbCrEnabledFlag() )
  {
    return;
  }

  CHECK( tu.jointCbCr && tu.jointCbCr != cbfMask, "wrong value of jointCbCr (" << (int)tu.jointCbCr << " vs " << (int)cbfMask << ")" );
  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    m_BinEncoder.encodeBin( tu.jointCbCr ? 1 : 0, Ctx::JointCbCrFlag( cbfMask - 1 ) );
  }
}

void CABACWriter::residual_coding( const TransformUnit& tu, ComponentID compID, CUCtx* cuCtx
#if JVET_AE0102_LFNST_CTX  
  , const bool codeTuCoeff
#endif    
)
{
  const CodingUnit& cu = *tu.cu;
#if JVET_AE0102_LFNST_CTX
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() codeTuCoeff=%d etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", codeTuCoeff, tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );
#else
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );
#endif

  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
    return;
#if SIGN_PREDICTION
  const bool  signPredQualified = TU::getDelayedSignCoding(tu, compID);
#endif
#if JVET_AE0102_LFNST_CTX
  if (codeTuCoeff == false)
  {
#endif
  ts_flag            ( tu, compID );
#if JVET_AE0102_LFNST_CTX
  }
#endif
  if( tu.mtsIdx[compID] == MTS_SKIP && !tu.cs->slice->getTSResidualCodingDisabledFlag() )
  {
#if JVET_AE0102_LFNST_CTX
    if (!isEncoding())
    {
#endif
    residual_codingTS( tu, compID );
#if JVET_AE0102_LFNST_CTX
    }
    else
    {
      if (codeTuCoeff == true)
      {
        residual_codingTS(tu, compID);
      }
    }
#endif
    return;
  }

  // determine sign hiding
  bool signHiding = cu.cs->slice->getSignDataHidingEnabledFlag();

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;

  // determine and set last coeff position and sig group flags
  int                      scanPosLast = -1;
  std::bitset<MLS_GRP_NUM> sigGroupFlags;

  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      scanPosLast = scanPos;
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }
  CHECK( scanPosLast < 0, "Coefficient coding called for empty TU" );

    cctx.setScanPosLast(scanPosLast);
#if JVET_AE0102_LFNST_CTX
    if (codeTuCoeff == false)
    {
#endif
#if !EXTENDED_LFNST
    if (cuCtx && tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
    {
#if JVET_AC0130_NSPT
      uint32_t  width = tu.blocks[compID].width;
      uint32_t height = tu.blocks[compID].height;
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
      bool spsIntraLfnstEnabled = ( ( tu.cu->slice->getSliceType() == I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTISlice() ) ||
                                    ( tu.cu->slice->getSliceType() != I_SLICE && tu.cu->cs->sps->getUseIntraLFNSTPBSlice() ) );
      bool allowNSPT = CU::isNSPTAllowed( tu, compID, width, height, spsIntraLfnstEnabled && CU::isIntra( *( tu.cu ) ) );
#else
      bool  allowNSPT = CU::isNSPTAllowed(tu, compID, width, height, CU::isIntra(*(tu.cu)));
#endif

#if JVET_W0119_LFNST_EXTENSION
      const int maxLfnstPos = (allowNSPT ? PU::getNSPTMatrixDim(width, height) : PU::getLFNSTMatrixDim(width, height)) - 1;
#else
      const int maxLfnstPos = allowNSPT ? PU::getNSPTMatrixDim(width, height) - 1 : (((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15);
#endif
#else
#if JVET_W0119_LFNST_EXTENSION
      const int maxLfnstPos = PU::getLFNSTMatrixDim(tu.blocks[compID].width, tu.blocks[compID].height) - 1;
#else
      const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
#endif
#endif
      cuCtx->violatesLfnstConstrained[toChannelType(compID)] |= cctx.scanPosLast() > maxLfnstPos;
    }
#endif

    if (cuCtx && tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4)
    {
#if JVET_AG0061_INTER_LFNST_NSPT
      const int lfnstLastScanPosTh = isLuma( compID ) ? ( CU::isIntra( *( tu.cu ) ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_LUMA_INTER ) : LFNST_LAST_SIG_CHROMA;
      cuCtx->lfnstLastScanPos |= ( CU::isIntra( *( tu.cu ) ) || isLuma( compID ) ) ? ( cctx.scanPosLast() >= lfnstLastScanPosTh ) : false;
#else
      const int lfnstLastScanPosTh = isLuma(compID) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
      cuCtx->lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
#endif
    }
    if (cuCtx && isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
    {
      cuCtx->mtsLastScanPos |= cctx.scanPosLast() >= 1;
#if JVET_Y0142_ADAPT_INTRA_MTS
      const int  coeffStride = tu.getCoeffs(compID).stride;
      const int  uiWidth = tu.getCoeffs(compID).width;
      const int  uiHeight = tu.getCoeffs(compID).height;
      uint64_t coeffAbsSum = 0;

      for (int y = 0; y < uiHeight; y++)
      {
        for (int x = 0; x < uiWidth; x++)
        {
          coeffAbsSum += abs(coeff[(y * coeffStride) + x]);
        }
      }
      cuCtx->mtsCoeffAbsSum = (int64_t)coeffAbsSum;
#endif
    }


  // code last coeff position
  last_sig_coeff( cctx, tu, compID );

#if JVET_AE0102_LFNST_CTX
  if ( isEncoding() && codeTuCoeff == false)
  {
#if EXTENDED_LFNST
    int subSetId = cctx.scanPosLast() >> cctx.log2CGSize();
    cctx.initSubblock(subSetId);
    if (cuCtx && tu.blocks[compID].width >= 4 && tu.blocks[compID].height >= 4)
    {
      const bool whge3 = tu.blocks[compID].width >= 8 && tu.blocks[compID].height >= 8;
      const bool isLfnstViolated = whge3 ? ((cctx.cgPosY() > 1 || cctx.cgPosX() > 1)) : ( (cctx.cgPosY() > 0 || cctx.cgPosX() > 0));
      cuCtx->violatesLfnstConstrained[toChannelType(compID)] |= isLfnstViolated;
    }
#endif
    return;
  }
  }
#endif
  // code subblocks
#if TCQ_8STATES
	const uint64_t stateTab = g_stateTransTab[ tu.cs->slice->getDepQuantEnabledIdc() ];
#else
  const int stateTab = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
#endif
  int       state     = 0;

  int ctxBinSampleRatio = (compID == COMPONENT_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

  for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
  {
    cctx.initSubblock       ( subSetId, sigGroupFlags[subSetId] );
#if !TU_256
    if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 && compID == COMPONENT_Y )
    {
      if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) )
       || ( tu.blocks[ compID ].width  == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth()  ) ) )
      {
        continue;
      }
    }
#endif

#if JVET_AE0102_LFNST_CTX 
    int lfnstidx = (cuCtx != nullptr && !cuCtx->lfnstLastScanPos && !cu.ispMode) ? 0 : cu.lfnstIdx;
    if (lfnstidx == 0 && cu.lfnstIdx != 0)
    {
      cctx.updateCtxSets();
    }
    residual_coding_subblock(cctx, coeff, stateTab, state, lfnstidx);

#else
    residual_coding_subblock(cctx, coeff, stateTab, state);
#endif
    
#if !TU_256
    if ( cuCtx && isLuma(compID) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
    {
      cuCtx->violatesMtsCoeffConstraint = true;
    }
#endif
#if JVET_AE0102_LFNST_CTX
    if ( !isEncoding() )
    {
#endif
#if EXTENDED_LFNST
      if (cuCtx && tu.blocks[compID].width >= 4 && tu.blocks[compID].height >= 4)
      {
        const bool whge3 = tu.blocks[compID].width >= 8 && tu.blocks[compID].height >= 8;
        const bool isLfnstViolated = whge3 ? (cctx.isSigGroup() && (cctx.cgPosY() > 1 || cctx.cgPosX() > 1)) : (cctx.isSigGroup() && (cctx.cgPosY() > 0 || cctx.cgPosX() > 0));
        cuCtx->violatesLfnstConstrained[toChannelType(compID)] |= isLfnstViolated;
      }
#endif
#if JVET_AE0102_LFNST_CTX
    }
#endif

  }
#if SIGN_PREDICTION
  if(signPredQualified && typeid(m_BinEncoder) == typeid(BitEstimator_Std))
  {
    bool codeSigns = true;
    if(tu.jointCbCr)
    {
      if( (tu.jointCbCr>>1) && compID == COMPONENT_Cb )
      {
        codeSigns = true;
      }
      if( !(tu.jointCbCr>>1) && compID == COMPONENT_Cr)
      {
        codeSigns = true;
      }
    }

    if(codeSigns)
    {
      codePredictedSigns(const_cast<TransformUnit &>(tu), compID);
    }
  }
#endif
}

void CABACWriter::ts_flag( const TransformUnit& tu, ComponentID compID )
{
  int tsFlag = tu.mtsIdx[compID] == MTS_SKIP ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 0 : 1;

  if( TU::isTSAllowed ( tu, compID ) )
  {
    m_BinEncoder.encodeBin( tsFlag, Ctx::TransformSkipFlag(ctxIdx));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.blocks[compID].x, tu.blocks[compID].y, tsFlag);
}

void CABACWriter::mts_idx( const CodingUnit& cu, CUCtx* cuCtx )
{
  TransformUnit &tu = *cu.firstTU;
  int        mtsIdx = tu.mtsIdx[COMPONENT_Y];

  if( CU::isMTSAllowed( cu, COMPONENT_Y ) && cuCtx && !cuCtx->violatesMtsCoeffConstraint &&
      cuCtx->mtsLastScanPos && cu.lfnstIdx == 0 && mtsIdx != MTS_SKIP)
  {
    int symbol = mtsIdx != MTS_DCT2_DCT2 ? 1 : 0;
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_Y0159_INTER_MTS
    int ctxIdx = 0;
    if (CU::isIntra(cu))
    {
      ctxIdx = (cuCtx->mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx->mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
    }
#else
    int ctxIdx = (cuCtx->mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx->mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
#endif
#else
    int ctxIdx = (cu.mipFlag) ? 3 : 0;
#endif
#else
    int ctxIdx = 0;
#endif

    m_BinEncoder.encodeBin( symbol, Ctx::MTSIdx(ctxIdx));

    if( symbol )
    {
#if JVET_W0103_INTRA_MTS
      int trIdx = (tu.mtsIdx[COMPONENT_Y] - MTS_DST7_DST7);
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_Y0159_INTER_MTS
      int nCands = CU::isIntra(cu) ? MTS_NCANDS[ctxIdx] : 4;
#else
      int nCands = MTS_NCANDS[ctxIdx];
#endif
      if (trIdx < 0 || trIdx >= nCands)
      {
        //Don't do anything.
      }
      else
      {
        CHECK(trIdx < 0 || trIdx >= nCands, "trIdx outside range");
        xWriteTruncBinCode(trIdx, nCands);
      }
#else
      CHECK(trIdx < 0 || trIdx >= 4, "trIdx outside range");
      m_BinEncoder.encodeBin(trIdx >> 1, Ctx::MTSIdx(1));
      m_BinEncoder.encodeBin(trIdx & 1, Ctx::MTSIdx(2));
#endif
#else
      ctxIdx = 1;
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol = mtsIdx > i + MTS_DST7_DST7 ? 1 : 0;
        m_BinEncoder.encodeBin( symbol, Ctx::MTSIdx(ctxIdx));

        if( !symbol )
        {
          break;
        }
      }
#endif
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
  }
}

void CABACWriter::isp_mode( const CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) || cu.colorTransform 
#if  ENABLE_DIMD && JVET_V0087_DIMD_NO_ISP
    || cu.dimd
#endif
#if JVET_AB0155_SGPM
      || cu.sgpm
#endif
    )
  {
    CHECK( cu.ispMode != NOT_INTRA_SUBPARTITIONS, "cu.ispMode != 0" );
    return;
  }
  if ( cu.ispMode == NOT_INTRA_SUBPARTITIONS )
  {
#if JVET_W0123_TIMD_FUSION
    m_BinEncoder.encodeBin( 0, cu.timd ? Ctx::ISPMode( 2 ) : Ctx::ISPMode( 0 ) );
#else
    m_BinEncoder.encodeBin( 0, Ctx::ISPMode( 0 ) );
#endif
  }
  else
  {
#if JVET_W0123_TIMD_FUSION
    m_BinEncoder.encodeBin( 1, cu.timd ? Ctx::ISPMode( 2 ) : Ctx::ISPMode( 0 ) );
#else
    m_BinEncoder.encodeBin( 1, Ctx::ISPMode( 0 ) );
#endif
    m_BinEncoder.encodeBin( cu.ispMode - 1, Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subpartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACWriter::residual_lfnst_mode( const CodingUnit& cu, CUCtx& cuCtx )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int chIdx = CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#else
  int chIdx = cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  bool spsIntraLfnstEnabled = ( ( cu.slice->getSliceType() == I_SLICE && cu.cs->sps->getUseIntraLFNSTISlice() ) ||
                                ( cu.slice->getSliceType() != I_SLICE && cu.cs->sps->getUseIntraLFNSTPBSlice() ) );
#endif
  if( ( cu.ispMode && !CU::canUseLfnstWithISP( cu, cu.chType ) ) ||
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
#if JVET_V0130_INTRA_TMP
    ( spsIntraLfnstEnabled && CU::isIntra( cu ) && ( ( cu.mipFlag && !allowLfnstWithMip( cu.firstPU->lumaSize() ) ) || ( cu.tmpFlag && !allowLfnstWithTmp() ) ) ) ||
#else
    ( spsIntraLfnstEnabled && CU::isIntra( cu ) && cu.mipFlag && !allowLfnstWithMip( cu.firstPU->lumaSize() ) ) ||
#endif
#else
#if JVET_V0130_INTRA_TMP
    (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && ((cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) || (cu.tmpFlag && !allowLfnstWithTmp()))) ||
#else
    (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) ||
#endif
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_CHROMA && std::min(cu.blocks[1].width, cu.blocks[1].height) < 4)
#else
    ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA && std::min( cu.blocks[ 1 ].width, cu.blocks[ 1 ].height ) < 4 )
#endif
    || ( cu.blocks[ chIdx ].lumaSize().width > cu.cs->sps->getMaxTbSize() || cu.blocks[ chIdx ].lumaSize().height > cu.cs->sps->getMaxTbSize() )
#if JVET_AG0061_INTER_LFNST_NSPT
    || ( cu.sbtInfo && CU::isInter( cu ) ) //JVET-AG0208 (EE2-related: On LFNST/NSPT index signalling)
#endif
    )
  {
    return;
  }
#if JVET_W0123_TIMD_FUSION
  if (cu.timd && (cu.ispMode || cu.firstPU->multiRefIdx))
  {
    return;
  }
#endif

#if JVET_AG0061_INTER_LFNST_NSPT
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  if( ( spsIntraLfnstEnabled && CU::isIntra( cu ) ) || ( cu.cs->sps->getUseInterLFNST() && CU::isInter( cu ) ) )
#else
  if (cu.cs->sps->getUseLFNST() && (CU::isIntra(cu) || CU::isInter(cu)))
#endif
#else
#if JVET_AH0103_LOW_DELAY_LFNST_NSPT
  if( spsIntraLfnstEnabled && CU::isIntra( cu ) )
#else
  if (cu.cs->sps->getUseLFNST() && CU::isIntra(cu))
#endif
#endif
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    const bool lumaFlag                   = CS::isDualITree(*cu.cs) ? (isLuma(cu.chType) ? true : false) : true;
    const bool chromaFlag                 = CS::isDualITree(*cu.cs) ? (isChroma(cu.chType) ? true : false) : true;
#else
    const bool lumaFlag                   = cu.isSepTree() ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag                 = cu.isSepTree() ? ( isChroma( cu.chType ) ? true : false ) : true;
#endif
    bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
    bool isTrSkip = false;
    for (auto &currTU : CU::traverseTUs(cu))
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);
      for (uint32_t compID = COMPONENT_Y; compID < numValidComp; compID++)
      {
        if (currTU.blocks[compID].valid() && TU::getCbf(currTU, (ComponentID)compID) && currTU.mtsIdx[compID] == MTS_SKIP)
        {
          isTrSkip = true;
          break;
        }
      }
    }
    if( (!cuCtx.lfnstLastScanPos && !cu.ispMode) || nonZeroCoeffNonTsCorner8x8 || isTrSkip )
    {
      return;
    }
  }
  else
  {
    return;
  }
#if JVET_AG0061_INTER_LFNST_NSPT
  if (CU::isInter(cu))
  {
    uint32_t idxLFNST = cu.lfnstIdx;
    assert(idxLFNST < 4);
    m_BinEncoder.encodeBin(idxLFNST > 0, Ctx::InterLFNSTIdx(0));
    if (idxLFNST > 0)
    {
      m_BinEncoder.encodeBin(idxLFNST > 1, Ctx::InterLFNSTIdx(1));
      if (idxLFNST > 1)
      {
        m_BinEncoder.encodeBin(idxLFNST > 2, Ctx::InterLFNSTIdx(2));
      }
    }
  }
  else
  {
#endif
    unsigned cctx = 0;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (CS::isDualITree(*cu.cs)) cctx++;
#else
    if ( cu.isSepTree() ) cctx++;
#endif
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
    uint32_t idxLFNST = cu.lfnstIdx;
    assert(idxLFNST < 4);

    uint32_t firstBit  = idxLFNST & 1;
    uint32_t secondBit = (idxLFNST >> 1) & 1;
    m_BinEncoder.encodeBin(firstBit, Ctx::LFNSTIdx(cctx));
    cctx = 2 + firstBit;
    m_BinEncoder.encodeBin(secondBit, Ctx::LFNSTIdx(cctx));
#else
    const uint32_t idxLFNST = cu.lfnstIdx;
    assert( idxLFNST < 3 );
    m_BinEncoder.encodeBin( idxLFNST ? 1 : 0, Ctx::LFNSTIdx( cctx ) );

    if( idxLFNST )
    {
      m_BinEncoder.encodeBin( (idxLFNST - 1) ? 1 : 0, Ctx::LFNSTIdx(2));
    }
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  }
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

void CABACWriter::last_sig_coeff( CoeffCodingContext& cctx, const TransformUnit& tu, ComponentID compID )
{
  unsigned blkPos = cctx.blockPos( cctx.scanPosLast() );
  unsigned posX, posY;
  {
    posY  = blkPos / cctx.width();
    posX  = blkPos - ( posY * cctx.width() );
  }

  unsigned CtxLast;
  unsigned GroupIdxX = g_uiGroupIdx[ posX ];
  unsigned GroupIdxY = g_uiGroupIdx[ posY ];

  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

#if !TU_256
  if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 && compID == COMPONENT_Y )
  {
    maxLastPosX = ( tu.blocks[compID].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[compID].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }
#endif
  for( CtxLast = 0; CtxLast < GroupIdxX; CtxLast++ )
  {
    m_BinEncoder.encodeBin( 1, cctx.lastXCtxId( CtxLast ) );
  }
  if( GroupIdxX < maxLastPosX )
  {
    m_BinEncoder.encodeBin(0, cctx.lastXCtxId(CtxLast));
  }
  for( CtxLast = 0; CtxLast < GroupIdxY; CtxLast++ )
  {
    m_BinEncoder.encodeBin(1, cctx.lastYCtxId(CtxLast));
  }
  if( GroupIdxY < maxLastPosY )
  {
    m_BinEncoder.encodeBin(0, cctx.lastYCtxId(CtxLast));
  }
  if( GroupIdxX > 3 )
  {
    posX -= g_uiMinInGroup[ GroupIdxX ];
    for (int i = ( ( GroupIdxX - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posX >> i ) & 1 );
    }
  }
  if( GroupIdxY > 3 )
  {
    posY -= g_uiMinInGroup[ GroupIdxY ];
    for ( int i = ( ( GroupIdxY - 2 ) >> 1 ) - 1 ; i >= 0; i-- )
    {
      m_BinEncoder.encodeBinEP( ( posY >> i ) & 1 );
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX_RESI, "last_sig_coeff() scan_pos_last=%d\n", cctx.scanPosLast());
}
#if TCQ_8STATES
#if JVET_AE0102_LFNST_CTX 
void CABACWriter::residual_coding_subblock(CoeffCodingContext& cctx, const TCoeff* coeff, const uint64_t stateTransTable, int& state, int lfnstIdx)
#else
void CABACWriter::residual_coding_subblock(CoeffCodingContext& cctx, const TCoeff* coeff, const uint64_t stateTransTable, int& state)
#endif
#else
void CABACWriter::residual_coding_subblock(CoeffCodingContext &cctx, const TCoeff *coeff, const int stateTransTable,
                                           int &state)
#endif
{
  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  const int   cgStartPosX = cctx.cgPosX() << cctx.log2CGWidth();
  const int   cgStartPosY = cctx.cgPosY() << cctx.log2CGHeight();
  const bool  isSPArea = (cgStartPosX < SIGN_PRED_FREQ_RANGE) && (cgStartPosY < SIGN_PRED_FREQ_RANGE);
  const bool  signPredQualified = cctx.getPredSignsQualified() > 0 && isSPArea && cctx.width() >= 4 && cctx.height() >= 4;
#else
  const bool  isFirst      = !cctx.isNotFirst();
  const bool  signPredQualified = cctx.getPredSignsQualified() > 0 && isFirst && cctx.width() >= 4 && cctx.height() >= 4 ;
#endif
#endif
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !isLast && cctx.isNotFirst() )
  {
    if( cctx.isSigGroup() )
    {
      m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId() );
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "sig_group() bin=%d\n", 1);
    }
    else
    {
      m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId() );
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "sig_group() bin=%d\n", 0);
      return;
    }
  }

  uint8_t   ctxOffset[16];

  //===== encode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
  TCoeff    remAbsLevel   = -1;
#else
  int       remAbsLevel   = -1;
#endif
  int       numNonZero    =  0;
  unsigned  signPattern   =  0;
  int       remRegBins    = cctx.regBinLimit;
  int       firstPosMode2 = minSubPos - 1;

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
#if JVET_AE0102_LFNST_CTX 
      const unsigned sigCtxId = cctx.sigCtxIdAbs(nextSigPos, coeff, state, lfnstIdx);
#else
      const unsigned sigCtxId = cctx.sigCtxIdAbs(nextSigPos, coeff, state);
#endif
      m_BinEncoder.encodeBin( sigFlag, sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
#if JVET_AE0102_LFNST_CTX 
      cctx.sigCtxIdAbs(nextSigPos, coeff, state, lfnstIdx); // required for setting variables that are needed for gtx/par context selection
#else
      cctx.sigCtxIdAbs(nextSigPos, coeff, state); // required for setting variables that are needed for gtx/par context selection
#endif
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff  = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff            = cctx.ctxOffsetAbs();
      numNonZero++;
      firstNZPos  = nextSigPos;
      lastNZPos   = std::max<int>( lastNZPos, nextSigPos );
      remAbsLevel = abs( Coeff ) - 1;

      if( nextSigPos != cctx.scanPosLast() ) signPattern <<= 1;
      if( Coeff < 0 )                        signPattern++;

      unsigned gt1 = !!remAbsLevel;

      m_BinEncoder.encodeBin( gt1, cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      if( gt1 )
      {
        remAbsLevel  -= 1;
#if !JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
        m_BinEncoder.encodeBin(remAbsLevel & 1, cctx.parityCtxIdAbs(ctxOff));
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbs( ctxOff ) );
        remAbsLevel >>= 1;
        remRegBins--;
#endif
        unsigned gt2 = !!remAbsLevel;

        m_BinEncoder.encodeBin(gt2, cctx.greater2CtxIdAbs(ctxOff));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2, cctx.greater2CtxIdAbs(ctxOff));
        remRegBins--;

#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
        if (gt2)
        {
          int gtX = 1;
          for (int i = 1; i < GTN - 1; i++)
          {
            if (gtX)
            {
              remAbsLevel -= 1;
              gtX = !!remAbsLevel;
              unsigned int ctxId = (i == 1) ? cctx.greater3CtxIdAbs(ctxOff) : cctx.greater4CtxIdAbs(ctxOff);
              m_BinEncoder.encodeBin(gtX, ctxId);
              DTRACE(g_trace_ctx, D_SYNTAX_RESI, "gt%d_flag() bin=%d ctx=%d\n", i + 2, gtX, ctxId);
              remRegBins--;
            }
            else
            {
              gtX = 0;
              break;
            }
          }
          if (gtX)
          {
            remAbsLevel -= 1;
            m_BinEncoder.encodeBinEP(remAbsLevel & 1);
            DTRACE(g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d \n", remAbsLevel & 1);
            remAbsLevel >>= 1;
          }
        }
#endif
      }
    }
#if TCQ_8STATES
    state = int( ( stateTransTable >> ((state<<3)+((Coeff&1)<<2)) ) & 15 );
#else
    state = ( stateTransTable >> ((state<<2)+((Coeff&1)<<1)) ) & 3;
#endif
  }
  firstPosMode2 = nextSigPos;
  cctx.regBinLimit = remRegBins;


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
#if JVET_AE0102_LFNST_CTX
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    int       sumAll = cctx.templateAbsSum2(scanPos, coeff, GTN_LEVEL);
#else
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4, lfnstIdx);
#endif
#else
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
#endif
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    ricePar = g_auiGoRiceParsCoeffGTN[sumAll];
#else
    ricePar = g_auiGoRiceParsCoeff[sumAll];
#endif
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    unsigned absLevel = (unsigned) abs( coeff[ cctx.blockPos( scanPos ) ] );
#else
    unsigned absLevel = abs( coeff[ cctx.blockPos( scanPos ) ] );
#endif
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
    if (absLevel >= GTN_LEVEL)
#else
    if( absLevel >= 4 )
#endif
    {
#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
      unsigned rem = (absLevel - GTN_LEVEL) >> 1;
#else
      unsigned rem      = ( absLevel - 4 ) >> 1;
#endif
      m_BinEncoder.encodeRemAbsEP( rem, ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    TCoeff    Coeff     = coeff[ cctx.blockPos( scanPos ) ];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    unsigned    absLevel  = (unsigned) abs( Coeff );
#else
    unsigned  absLevel  = abs( Coeff );
#endif
#if JVET_AE0102_LFNST_CTX
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0, lfnstIdx);
#else
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
#endif
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    unsigned  rem       = ( absLevel == 0 ? pos0 : absLevel <= pos0 ? absLevel-1 : absLevel );
    m_BinEncoder.encodeRemAbsEP( rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
#if TCQ_8STATES
    state = int( ( stateTransTable >> ((state<<3)+((absLevel&1)<<2)) ) & 15 );
#else
    state = ( stateTransTable >> ((state<<2)+((absLevel&1)<<1)) ) & 3;
#endif
    if( absLevel )
    {
      numNonZero++;
      firstNZPos = scanPos;
      lastNZPos   = std::max<int>( lastNZPos, scanPos );
      signPattern <<= 1;
      if( Coeff < 0 ) signPattern++;
    }
  }

  //===== encode sign's =====
  unsigned numSigns = numNonZero;
  if( cctx.hideSign( firstNZPos, lastNZPos ) )
  {
    numSigns    --;
    signPattern >>= 1;
  }
#if SIGN_PREDICTION
  if( !signPredQualified)
  {
#endif
  m_BinEncoder.encodeBinsEP( signPattern, numSigns );
#if SIGN_PREDICTION
  }
#endif
}

void CABACWriter::residual_codingTS( const TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding_ts() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, isLuma(compID) ? tu.cu->bdpcmMode : tu.cu->bdpcmModeChroma);
  const TCoeff*       coeff   = tu.getCoeffs( compID ).buf;
  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  // determine and set last coeff position and sig group flags
  std::bitset<MLS_GRP_NUM> sigGroupFlags;
  for( int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
  {
    unsigned blkPos = cctx.blockPos( scanPos );
    if( coeff[blkPos] )
    {
      sigGroupFlags.set( scanPos >> cctx.log2CGSize() );
    }
  }

  // code subblocks
  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId, sigGroupFlags[subSetId] );
    residual_coding_subblockTS(cctx, coeff);
  }
}

void CABACWriter::residual_coding_subblockTS(CoeffCodingContext &cctx, const TCoeff *coeff)
{
  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;

  //===== encode significant_coeffgroup_flag =====
  if( !cctx.isLastSubSet() || !cctx.only1stSigGroup() )
  {
    if( cctx.isSigGroup() )
    {
        m_BinEncoder.encodeBin( 1, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_group() bin=%d ctx=%d\n", 1, cctx.sigGroupCtxId() );
    }
    else
    {
        m_BinEncoder.encodeBin( 0, cctx.sigGroupCtxId( true ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_group() bin=%d ctx=%d\n", 0, cctx.sigGroupCtxId() );
      return;
    }
  }

  //===== encode absolute values =====
  const int inferSigPos   = minSubPos;
  int       remAbsLevel   = -1;
  int       numNonZero    =  0;

  int rightPixel, belowPixel, modAbsCoeff;

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    TCoeff    Coeff      = coeff[ cctx.blockPos( nextSigPos ) ];
    unsigned  sigFlag    = ( Coeff != 0 );
    if( numNonZero || nextSigPos != inferSigPos )
    {
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS(nextSigPos, coeff);
        m_BinEncoder.encodeBin( sigFlag, sigCtxId );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
        cctx.decimateNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== encode sign's =====
      int sign = Coeff < 0;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      m_BinEncoder.encodeBin(sign, signCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sign() bin=%d ctx=%d\n", sign, signCtxId);
      cctx.decimateNumCtxBins(1);
      numNonZero++;
      cctx.neighTS(rightPixel, belowPixel, nextSigPos, coeff);
      modAbsCoeff = cctx.deriveModCoeff(rightPixel, belowPixel, abs(Coeff), cctx.bdpcm());
      remAbsLevel = modAbsCoeff - 1;
      unsigned gt1 = !!remAbsLevel;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      m_BinEncoder.encodeBin(gt1, gt1CtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1, gt1CtxId);
      cctx.decimateNumCtxBins(1);
      if( gt1 )
      {
        remAbsLevel  -= 1;
        m_BinEncoder.encodeBin(remAbsLevel & 1, cctx.parityCtxIdAbsTS());
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", remAbsLevel&1, cctx.parityCtxIdAbsTS() );
        cctx.decimateNumCtxBins(1);
      }
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  int numGtBins = 4;
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm());
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if (absLevel >= cutoffVal)
      {
        unsigned gt2 = (absLevel >= (cutoffVal + 2));
        m_BinEncoder.encodeBin(gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2, cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, min<int>(absLevel, cutoffVal + 2 + (absLevel&1)));
        cctx.decimateNumCtxBins(1);
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }

  //===== coeff bypass ====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    unsigned absLevel;
    cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    absLevel = cctx.deriveModCoeff(rightPixel, belowPixel, abs(coeff[cctx.blockPos(scanPos)]), cctx.bdpcm()||!cutoffVal);

    if( absLevel >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      unsigned  rem = scanPos <= lastScanPosPass1 ? (absLevel - cutoffVal) >> 1 : absLevel;
      m_BinEncoder.encodeRemAbsEP( rem, rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );

      if (absLevel && scanPos > lastScanPosPass1)
      {
        int sign = coeff[cctx.blockPos(scanPos)] < 0;
        m_BinEncoder.encodeBinEP(sign);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_rice_sign() bin=%d\n", sign);
      }
    }
  }
}







//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    void  unary_max_symbol  ( symbol, ctxId0, ctxIdN, maxSymbol )
//    void  unary_max_eqprob  ( symbol,                 maxSymbol )
//    void  exp_golomb_eqprob ( symbol, count )
//================================================================================

void CABACWriter::unary_max_symbol( unsigned symbol, unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol )
{
  CHECK( symbol > maxSymbol, "symbol > maxSymbol" );
  const unsigned totalBinsToWrite = std::min( symbol + 1, maxSymbol );
  for( unsigned binsWritten = 0; binsWritten < totalBinsToWrite; ++binsWritten )
  {
    const unsigned nextBin = symbol > binsWritten;
    m_BinEncoder.encodeBin( nextBin, binsWritten == 0 ? ctxId0 : ctxIdN );
  }
}


void CABACWriter::unary_max_eqprob( unsigned symbol, unsigned maxSymbol )
{
  if( maxSymbol == 0 )
  {
    return;
  }
  bool     codeLast = ( maxSymbol > symbol );
  unsigned bins     = 0;
  unsigned numBins  = 0;
  while( symbol-- )
  {
    bins   <<= 1;
    bins   ++;
    numBins++;
  }
  if( codeLast )
  {
    bins  <<= 1;
    numBins++;
  }
  CHECK(!( numBins <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP( bins, numBins );
}


void CABACWriter::exp_golomb_eqprob( unsigned symbol, unsigned count )
{
  unsigned bins    = 0;
  unsigned numBins = 0;
  while( symbol >= (unsigned)(1<<count) )
  {
    bins <<= 1;
    bins++;
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  bins <<= 1;
  numBins++;
  //CHECK(!( numBins + count <= 32 ), "Unspecified error");
  m_BinEncoder.encodeBinsEP(bins, numBins);
  m_BinEncoder.encodeBinsEP(symbol, count);
}

void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isLuma( channel ) )
  {
    if( alfParam->enabledFlag[COMPONENT_Y] )
    {
      codeAlfCtuEnableFlags( cs, COMPONENT_Y, alfParam );
    }
  }
  else
  {
    if( alfParam->enabledFlag[COMPONENT_Cb] )
    {
      codeAlfCtuEnableFlags( cs, COMPONENT_Cb, alfParam );
    }

    if( alfParam->enabledFlag[COMPONENT_Cr] )
    {
      codeAlfCtuEnableFlags( cs, COMPONENT_Cr, alfParam );
    }
  }
}
void CABACWriter::codeAlfCtuEnableFlags( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  uint32_t numCTUs = cs.pcv->sizeInCtus;

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    codeAlfCtuEnableFlag( cs, ctuIdx, compID, alfParam );
  }
}

void CABACWriter::codeAlfCtuEnableFlag( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, AlfParam* alfParam)
{
  const bool alfComponentEnabled = (alfParam != NULL) ? alfParam->enabledFlag[compIdx] : cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx);

  if( cs.sps->getALFEnabledFlag() && alfComponentEnabled )
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
    const uint32_t      curTileIdx = cs.pps->getTileIdx( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
    int ctx = 0;
    ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
    ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;
    m_BinEncoder.encodeBin( ctbAlfFlag[ctuRsAddr], Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );
  }
}


#if JVET_X0071_LONGER_CCALF
void CABACWriter::codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const ComponentID compID,
  const int curIdx, const uint8_t *filterControlIdc, Position lumaPos,
  const int filterCount)
{
  CHECK(idcVal > filterCount, "Filter index is too large");

  const uint32_t curSliceIdx = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx = cs.pps->getTileIdx(lumaPos);
  Position       leftLumaPos = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  bool           leftAvail = cs.getCURestricted(leftLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L) ? true : false;
  bool           aboveAvail = cs.getCURestricted(aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L) ? true : false;
  int            ctxt = 0;

  if (leftAvail)
  {
    ctxt += (filterControlIdc[curIdx - 1] != 1) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus] != 1) ? 1 : 0;
  }
  ctxt += (compID == COMPONENT_Cr) ? 3 : 0;

  int       pos0 = 1;
  unsigned  mappedIdc = (idcVal == 0 ? pos0 : idcVal <= pos0 ? idcVal - 1 : idcVal);

  m_BinEncoder.encodeBin((mappedIdc == 0) ? 0 : 1, Ctx::CcAlfFilterControlFlag(ctxt)); // ON/OFF flag is context coded
  if (mappedIdc > 0)
  {
    int val = (mappedIdc - 1);
    while (val)
    {
      m_BinEncoder.encodeBinEP(1);
      val--;
    }
    if (mappedIdc < filterCount)
    {
      m_BinEncoder.encodeBinEP(0);
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "cc_alf_filter_control_idc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n", compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal);
}

#else

void CABACWriter::codeCcAlfFilterControlIdc(uint8_t idcVal, CodingStructure &cs, const ComponentID compID,
                                            const int curIdx, const uint8_t *filterControlIdc, Position lumaPos,
                                            const int filterCount)
{
  CHECK(idcVal > filterCount, "Filter index is too large");

  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( filterControlIdc[curIdx - 1]) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus]) ? 1 : 0;
  }
  ctxt += ( compID == COMPONENT_Cr ) ? 3 : 0;

  m_BinEncoder.encodeBin( ( idcVal == 0 ) ? 0 : 1, Ctx::CcAlfFilterControlFlag( ctxt ) ); // ON/OFF flag is context coded
  if ( idcVal > 0 )
  {
    int val = (idcVal - 1);
    while ( val )
    {
      m_BinEncoder.encodeBinEP( 1 );
      val--;
    }
    if ( idcVal < filterCount )
    {
      m_BinEncoder.encodeBinEP( 0 );
    }
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "cc_alf_filter_control_idc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n", compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal );
}

#endif



void CABACWriter::code_unary_fixed( unsigned symbol, unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  bool unary = (symbol <= unary_max);
  m_BinEncoder.encodeBin( unary, ctxId );
  if( unary )
  {
    unary_max_eqprob( symbol, unary_max );
  }
  else
  {
    m_BinEncoder.encodeBinsEP( symbol - unary_max - 1, fixed );
  }
}

#if JVET_V0130_INTRA_TMP
void CABACWriter::tmp_flag(const CodingUnit& cu)
{
  if (!cu.Y().valid())
  {
    return;
  }

#if JVET_X0124_TMP_SIGNAL
  if (cu.dimd)
  {
    return;
  }
#endif

  if (!cu.cs->sps->getUseIntraTMP())
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxTmpFlag(cu);
  m_BinEncoder.encodeBin(cu.tmpFlag, Ctx::TmpFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "tmp_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpFlag ? 1 : 0);
#if JVET_AD0086_ENHANCED_INTRA_TMP
  if (cu.tmpFlag)
  {
    unsigned ctxId_fusion = DeriveCtx::CtxTmpFusionFlag(cu);
    m_BinEncoder.encodeBin(cu.tmpFusionFlag, Ctx::TmpFusion(ctxId_fusion));
    DTRACE(g_trace_ctx, D_SYNTAX, "tmp_fusion_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
           cu.tmpFusionFlag ? 1 : 0);
    if (cu.tmpFusionFlag)
    {
      m_BinEncoder.encodeBin(cu.tmpIdx >= TMP_GROUP_IDX ? 1: 0, Ctx::TmpFusion(4));

      int tmpFusionIdx = cu.tmpIdx;
      tmpFusionIdx = tmpFusionIdx >=  TMP_GROUP_IDX ? tmpFusionIdx - TMP_GROUP_IDX: tmpFusionIdx;
      m_BinEncoder.encodeBin(tmpFusionIdx ? 1: 0, Ctx::TmpFusion(5));
      if (tmpFusionIdx)
      {
        m_BinEncoder.encodeBinEP(tmpFusionIdx > 1 ? 1 : 0);
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "tmp_fusion_idx() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpIdx);
#if JVET_AG0136_INTRA_TMP_LIC
      m_BinEncoder.encodeBin(cu.tmpLicFlag, Ctx::TmpLic(0));
      DTRACE(g_trace_ctx, D_SYNTAX, "tmpLicFlag pos=(%d,%d) value=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpLicFlag);
#endif
    }
    else
    {
      if (cu.tmpIdx < 3)
      {
        m_BinEncoder.encodeBin(1, Ctx::TmpIdx(0));
        if (cu.tmpIdx == 0)
        {
          m_BinEncoder.encodeBin(1, Ctx::TmpIdx(1));
        }
        else
        {
          m_BinEncoder.encodeBin(0, Ctx::TmpIdx(1));
          if (cu.tmpIdx == 1)
          {
            m_BinEncoder.encodeBin(1, Ctx::TmpIdx(2));
          }
          else
          {
            m_BinEncoder.encodeBin(0, Ctx::TmpIdx(2));
          }
        }
      }
      else
      {
        m_BinEncoder.encodeBin(0, Ctx::TmpIdx(0));
        xWriteTruncBinCode(cu.tmpIdx - 3, MTMP_NUM - 3);
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "tmp_idx() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpIdx);

      m_BinEncoder.encodeBin(cu.tmpFlmFlag, Ctx::TmpFusion(3));
      DTRACE(g_trace_ctx, D_SYNTAX, "tmp_flm_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y,
             cu.tmpFlmFlag);
#if JVET_AG0136_INTRA_TMP_LIC
      if (!cu.tmpFlmFlag)
      {
        m_BinEncoder.encodeBin(cu.tmpLicFlag, Ctx::TmpLic(0));
        DTRACE(g_trace_ctx, D_SYNTAX, "tmpLicFlag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpLicFlag);
        if (cu.slice->getSPS()->getItmpLicExtension() && cu.tmpLicFlag)
        {
          const int bin1 = (cu.ibcLicIdx == IBC_LIC_IDX) || (cu.ibcLicIdx == IBC_LIC_IDX_M) ? 0 : 1;
          const int bin2 = (cu.ibcLicIdx == IBC_LIC_IDX) || (cu.ibcLicIdx == IBC_LIC_IDX_T) ? 0 : 1;
          m_BinEncoder.encodeBin(bin1, Ctx::ItmpLicIndex(0));
          m_BinEncoder.encodeBin(bin2, Ctx::ItmpLicIndex(1));
          DTRACE(g_trace_ctx, D_SYNTAX, "tmp_lic_idx=%d\n", cu.ibcLicIdx);
        }
      }
#if !JVET_AH0200_INTRA_TMP_BV_REORDER
      if (!cu.tmpFlmFlag && !cu.tmpLicFlag)
#else
      if (!cu.tmpFlmFlag)
#endif
#else
      if (!cu.tmpFlmFlag)
#endif
      {
#if JVET_AH0200_INTRA_TMP_BV_REORDER
        if(cu.lwidth() * cu.lheight() <= TMP_SKIP_REFINE_THRESHOLD)
        {
          if(cu.tmpFracIdx > 0)
          {
            m_BinEncoder.encodeBin(1, Ctx::TmpFlag(7));
          }
          else
          {
            m_BinEncoder.encodeBin(0, Ctx::TmpFlag(7));
          }
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "tmpFracIdx pos=(%d,%d) value=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpFracIdx);
#else
        m_BinEncoder.encodeBin(cu.tmpIsSubPel != 0 ? 1 : 0, Ctx::TmpFlag(4));
        if (cu.tmpIsSubPel)
        {
          m_BinEncoder.encodeBin(cu.tmpIsSubPel >= 2 ? 1 : 0, Ctx::TmpFlag(5));
          if (cu.tmpIsSubPel >= 2)
          {
            m_BinEncoder.encodeBin(cu.tmpIsSubPel == 3 ? 1 : 0, Ctx::TmpFlag(6));
          }
          m_BinEncoder.encodeBinsEP(cu.tmpSubPelIdx, 3);
        }
        DTRACE(g_trace_ctx, D_SYNTAX, "tmp_is_subpel() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpIsSubPel);
#endif
      }
    }
  }
#endif
}
#endif

void CABACWriter::mip_flag( const CodingUnit& cu )
{
#if ENABLE_DIMD
  if (cu.dimd)
  {
    return;
  }
#endif 
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  m_BinEncoder.encodeBin( cu.mipFlag, Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACWriter::mip_pred_modes( const CodingUnit& cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }
  for( const auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACWriter::mip_pred_mode( const PredictionUnit& pu )
{
  m_BinEncoder.encodeBinEP( (pu.mipTransposedFlag ? 1 : 0) );

  const int numModes = getNumModesMip( pu.Y() );
  CHECKD( pu.intraDir[CHANNEL_TYPE_LUMA] < 0 || pu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode" );
  xWriteTruncBinCode( pu.intraDir[CHANNEL_TYPE_LUMA], numModes );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA], pu.mipTransposedFlag ? 1 : 0 );
}

void CABACWriter::codeAlfCtuFilterIndex(CodingStructure& cs, uint32_t ctuRsAddr, bool alfEnableLuma)
{
  if ( (!cs.sps->getALFEnabledFlag()) || (!alfEnableLuma))
  {
    return;
  }

  uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag(COMPONENT_Y);
  if (!ctbAlfFlag[ctuRsAddr])
  {
    return;
  }

  short* alfCtbFilterIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  const unsigned filterSetIdx = alfCtbFilterIndex[ctuRsAddr];
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    int useTemporalFilt = (filterSetIdx >= NUM_FIXED_FILTER_SETS) ? 1 : 0;
    m_BinEncoder.encodeBin(useTemporalFilt, Ctx::AlfUseTemporalFilt());
    if (useTemporalFilt)
    {
      CHECK((filterSetIdx - NUM_FIXED_FILTER_SETS) >= (numAvailableFiltSets - NUM_FIXED_FILTER_SETS), "temporal non-latest set");
      if (numAps > 1)
      {
        xWriteTruncBinCode(filterSetIdx - NUM_FIXED_FILTER_SETS, numAvailableFiltSets - NUM_FIXED_FILTER_SETS);
      }
    }
    else
    {
      CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set larger than temporal");
      xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
    }
  }
  else
  {
    CHECK(filterSetIdx >= NUM_FIXED_FILTER_SETS, "fixed set numavail < num_fixed");
    xWriteTruncBinCode(filterSetIdx, NUM_FIXED_FILTER_SETS);
  }
}
void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ChannelType channel, AlfParam* alfParam)
{
  if( isChroma( channel ) )
  {
    if( alfParam->enabledFlag[COMPONENT_Cb] )
    {
      codeAlfCtuAlternatives( cs, COMPONENT_Cb, alfParam );
    }

    if( alfParam->enabledFlag[COMPONENT_Cr] )
    {
      codeAlfCtuAlternatives( cs, COMPONENT_Cr, alfParam );
    }
  }
}
void CABACWriter::codeAlfCtuAlternatives( CodingStructure& cs, ComponentID compID, AlfParam* alfParam)
{
  if( compID == COMPONENT_Y )
  {
    return;
  }

  uint32_t numCTUs = cs.pcv->sizeInCtus;
  uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compID );

  for( int ctuIdx = 0; ctuIdx < numCTUs; ctuIdx++ )
  {
    if( ctbAlfFlag[ctuIdx] )
    {
      codeAlfCtuAlternative( cs, ctuIdx, compID, alfParam );
    }
  }
}

void CABACWriter::codeAlfCtuAlternative( CodingStructure& cs, uint32_t ctuRsAddr, const int compIdx, const AlfParam* alfParam
#if ALF_IMPROVEMENT
  , int numAltLuma
#endif
)
{
#if ALF_IMPROVEMENT
  if (compIdx == COMPONENT_Y)
  {
    if (alfParam || (cs.sps->getALFEnabledFlag() && cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx)))
    {
      uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag(compIdx);
      short*   ctbAlfFilterSetIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
      if (ctbAlfFlag[ctuRsAddr] && ctbAlfFilterSetIndex[ctuRsAddr] >= NUM_FIXED_FILTER_SETS)
      {
        uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData(compIdx);
        unsigned numOnes = ctbAlfAlternative[ctuRsAddr];
        assert(ctbAlfAlternative[ctuRsAddr] < numAltLuma);

        for( int i = 0; i < numOnes; ++i )
        {
          m_BinEncoder.encodeBin( 1, Ctx::ctbAlfAlternative( compIdx ) );
        }

        if( numOnes < numAltLuma - 1 )
        {
          m_BinEncoder.encodeBin( 0, Ctx::ctbAlfAlternative( compIdx ) );
        }
      }
    }
  }
  else
  {
#else
  if( compIdx == COMPONENT_Y )
    return;
#endif
  int apsIdx = alfParam ? 0 : cs.slice->getTileGroupApsIdChroma();
  const AlfParam& alfParamRef = alfParam ? (*alfParam) : cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();

  if( alfParam || (cs.sps->getALFEnabledFlag() && cs.slice->getTileGroupAlfEnabledFlag( (ComponentID)compIdx )) )
  {
    uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );

    if( ctbAlfFlag[ctuRsAddr] )
    {
      const int numAlts = alfParamRef.numAlternativesChroma;
      uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData( compIdx );
      unsigned numOnes = ctbAlfAlternative[ctuRsAddr];
      assert( ctbAlfAlternative[ctuRsAddr] < numAlts );
#if ALF_IMPROVEMENT
      for( int i = 0; i < numOnes; ++i )
      {
        m_BinEncoder.encodeBin( 1, Ctx::ctbAlfAlternative( compIdx ) );
      }
      if( numOnes < numAlts-1 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::ctbAlfAlternative( compIdx ) );
      }
#else
      for( int i = 0; i < numOnes; ++i )
      {
        m_BinEncoder.encodeBin( 1, Ctx::ctbAlfAlternative( compIdx - 1 ) );
      }

      if( numOnes < numAlts - 1 )
      {
        m_BinEncoder.encodeBin( 0, Ctx::ctbAlfAlternative( compIdx - 1 ) );
      }
#endif
    }
  }
#if ALF_IMPROVEMENT
  }
#endif
}

#if INTER_LIC
void CABACWriter::cu_lic_flag(const CodingUnit& cu)
{
  if (CU::isLICFlagPresent(cu))
  {
#if JVET_AG0276_LIC_SLOPE_ADJUST
    unsigned ctxId = DeriveCtx::CtxLicFlag( cu );
    m_BinEncoder.encodeBin(cu.licFlag ? 1 : 0, Ctx::LICFlag(ctxId));
#else
    m_BinEncoder.encodeBin(cu.licFlag ? 1 : 0, Ctx::LICFlag(0));
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.licFlag ? 1 : 0);
#if JVET_AG0276_LIC_SLOPE_ADJUST
    if (cu.licFlag && CU::isLicSlopeAllowed(cu) && cu.firstPU->interDir != 3)
    {
      int delta = cu.licDelta;
      m_BinEncoder.encodeBin( delta != 0 ? 1 : 0, Ctx::LicDelta(0) );
      if ( delta )
      {
        m_BinEncoder.encodeBin( delta < 0 ? 1 : 0, Ctx::LicDelta(1) );
      }
    }
#endif
  }
}
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
void CABACWriter::bvOneZeroComp(const CodingUnit &cu)
{
  if (!CU::isIBC(cu) || cu.firstPU->mergeFlag)
  {
    return;
  }
  unsigned ctxId = DeriveCtx::CtxbvOneZeroComp(cu);
  m_BinEncoder.encodeBin(cu.bvOneZeroComp > 0, Ctx::bvOneZeroComp(ctxId));
  if (cu.bvOneZeroComp)
  {
    // Write the BV direction
    m_BinEncoder.encodeBin(cu.bvZeroCompDir >> 1, Ctx::bvOneZeroComp(3));
#if JVET_AA0070_RRIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (cu.cs->sps->getUseRRIbc())
    {
#endif
      ctxId = DeriveCtx::CtxRribcFlipType(cu);
      m_BinEncoder.encodeBin(cu.rribcFlipType > 0, Ctx::rribcFlipType(ctxId));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    }
#endif
#endif
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "rribc_data() rribc_flip_type = %d\n", cu.rribcFlipType);
}
#endif

#if JVET_AA0070_RRIBC
void CABACWriter::rribcData(const CodingUnit& cu)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (!cu.cs->sps->getUseRRIbc() || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
#else
  if (!CU::isIBC(cu) || cu.firstPU->mergeFlag)
#endif
  {
    return;
  }

  unsigned ctxId = DeriveCtx::CtxRribcFlipType(cu);
  m_BinEncoder.encodeBin(cu.rribcFlipType > 0, Ctx::rribcFlipType(ctxId));
  if (cu.rribcFlipType)
  {
    CHECK(cu.rribcFlipType != 1 && cu.rribcFlipType != 2, "cu.rribcFlipType != 1 && cu.rribcFlipType != 2");
    m_BinEncoder.encodeBin(cu.rribcFlipType >> 1, Ctx::rribcFlipType(3));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "rribc_data() rribc_flip_type = %d\n", cu.rribcFlipType);
}
#endif

#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
struct SignCombInfo
{
  unsigned idx;
  bool     sign;
  bool     isSignPred;

  SignCombInfo(const unsigned _sign, const unsigned _idx, const bool _isPred)
    : idx(_idx), sign(_sign), isSignPred(_isPred)
  {
  }
};

bool compareOrderIdx(SignCombInfo cand0, SignCombInfo cand1)
{
  return (cand0.idx < cand1.idx);
}
#endif
void CABACWriter::codePredictedSigns( TransformUnit &tu, ComponentID compID )
{
  const CtxSet* ctx = &Ctx::signPred[toChannelType( compID )];
  int ctxOffset = CU::isIntra( *tu.cu ) ? 0 : 2;
  const bool useSignPred = TU::getUseSignPred( tu, compID );

  CoeffBuf buff = tu.getCoeffs( compID );
  AreaBuf<SIGN_PRED_TYPE> signBuff = tu.getCoeffSigns(compID);
  TCoeff *coeff = buff.buf;
  SIGN_PRED_TYPE         *signs    = signBuff.buf;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  IdxBuf signScanIdxBuff = tu.getCoeffSignsScanIdx(compID);
  unsigned *signScanIdx = signScanIdxBuff.buf;
  uint32_t extAreaWidth = std::min(tu.blocks[compID].width, (uint32_t)SIGN_PRED_FREQ_RANGE);
  uint32_t extAreaHeight = std::min(tu.blocks[compID].height, (uint32_t)SIGN_PRED_FREQ_RANGE);
  if (!useSignPred)
  {
    for (uint32_t y = 0; y < extAreaHeight; y++)
    {
      for (uint32_t x = 0; x < extAreaWidth; x++)
      {
        TCoeff coef = coeff[x];
        if (coef)
        {
          if (signs[x] != SIGN_PRED_HIDDEN)
          {
            m_BinEncoder.encodeBinEP(coef < 0 ? 1 : 0);
          }
        }
      }
      coeff += buff.stride;
      signs += signBuff.stride;
    }
  }
  else
  {
    std::vector<SignCombInfo>                  signCombList;
    bool lfnstEnabled = tu.checkLFNSTApplied(compID);
    static_vector<uint32_t, SIGN_PRED_MAX_NUM> levelList;
    const int32_t                              maxNumPredSigns =
      lfnstEnabled ? std::min<int>(4, tu.cs->sps->getNumPredSigns()) : tu.cs->sps->getNumPredSigns();
    CHECK(maxNumPredSigns > levelList.max_size(), "levelList is too small");
    uint32_t extAreaSize = (lfnstEnabled ? 4 : tu.cs->sps->getSignPredArea());
    uint32_t spAreaWidth = std::min(tu.blocks[compID].width, extAreaSize);
    uint32_t spAreaHeight = std::min(tu.blocks[compID].height, extAreaSize);
    for (uint32_t y = 0; y < spAreaHeight; y++)
    {
      for (uint32_t x = 0; x < spAreaWidth; x++)
      {
        TCoeff coef = coeff[x];
        if (coef)
        {
          SIGN_PRED_TYPE sign = signs[x];
          if (sign != SIGN_PRED_HIDDEN)
          {
            if (sign == SIGN_PRED_BYPASS)
            {
              const bool   curSign = coef < 0;
              unsigned scanIdx = signScanIdx[x];
              SignCombInfo signCand(curSign, scanIdx, false);
              signCombList.push_back(signCand);
            }
            else
            {
              const bool errSignPred =
                !(coef > 0 && sign == SIGN_PRED_POSITIVE) && !(coef < 0 && sign == SIGN_PRED_NEGATIVE);
              unsigned scanIdx = signScanIdx[x];
              SignCombInfo signCand(errSignPred, scanIdx, true);
              signCombList.push_back(signCand);
            }
            if (levelList.size() < maxNumPredSigns)
            {
              levelList.push_back(abs(coef));
            }
          }
        }
      }
      coeff += buff.stride;
      signs += signBuff.stride;
      signScanIdx += signScanIdxBuff.stride;
    }
    std::stable_sort(signCombList.begin(), signCombList.end(), compareOrderIdx);
    int numScanPos = 0;
    for (uint32_t idx = 0; idx < signCombList.size(); idx++)
    {
      if (signCombList[idx].isSignPred)
      {
        const bool errSignPred = signCombList[idx].sign;
        uint32_t level = levelList[numScanPos++];
        int levOffset = (level < 2) ? 0 : 1;
        m_BinEncoder.encodeBin(errSignPred ? 1 : 0, (*ctx)(ctxOffset + levOffset));
      }
      else
      {
        const bool curSign = signCombList[idx].sign;
        m_BinEncoder.encodeBinEP(curSign ? 1 : 0);
      }
    }
    if (spAreaWidth != extAreaWidth || spAreaHeight != extAreaHeight)
    {
      coeff = buff.buf;
      for (uint32_t y = 0; y < extAreaHeight; y++)
      {
        uint32_t startX = (y < spAreaHeight) ? spAreaWidth : 0;
        uint32_t endX = extAreaWidth - 1;
        for (uint32_t x = startX; x <= endX; x++)
        {
          TCoeff coef = coeff[x];
          if (coef)
          {
            uint32_t curSign = (coef < 0) ? 1 : 0;
            m_BinEncoder.encodeBinEP(curSign);
          }
        }
        coeff += buff.stride;
      }
    }
  }
#else
  for( uint32_t y = 0; y < SIGN_PRED_FREQ_RANGE; y++ )
  {
    for( uint32_t x = 0; x < SIGN_PRED_FREQ_RANGE; x++ )
    {
      TCoeff coef = coeff[x];

      if( coef )
      {
        SIGN_PRED_TYPE sign = signs[x];
        if (sign != SIGN_PRED_HIDDEN)
        {

          if( sign == SIGN_PRED_BYPASS || !useSignPred )
          {
            m_BinEncoder.encodeBinEP( coef < 0 ? 1 : 0 );
          }
          else
          {
            uint32_t errSignPred =
              ((coef > 0 && sign == SIGN_PRED_POSITIVE) || (coef < 0 && sign == SIGN_PRED_NEGATIVE)) ? 0 : 1;
            uint32_t ctxId = ( x || y ) ? 1 : 0;
            m_BinEncoder.encodeBin( errSignPred, ( *ctx )( ctxId + ctxOffset ) );
          }
        }
      }
    }

    coeff += buff.stride;
    signs += signBuff.stride;
  }
#endif
}
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
void CABACWriter::amvpMerge_mode( const PredictionUnit& pu )
{
  if (PU::isBipredRestriction(pu))
  {
    CHECK(1, "this is not possible");
    return;
  }
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    m_BinEncoder.encodeBin(1, Ctx::amFlagState());
    DTRACE(g_trace_ctx, D_SYNTAX, "amvp_merge_mode() am_flag=%d\n", 1);
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (pu.cs->sps->getUseARL())
    {
      // signaled by refIdxLC
    }
    else
#endif
    if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() == false)
    {
      if (pu.amvpMergeModeFlag[REF_PIC_LIST_0])
      {
        m_BinEncoder.encodeBinEP(0);
        DTRACE(g_trace_ctx, D_SYNTAX, "amvp_merge_mode() dir=%d\n", 0);
      }
      else
      {
        m_BinEncoder.encodeBinEP(1);
        DTRACE(g_trace_ctx, D_SYNTAX, "amvp_merge_mode() dir=%d\n", 1);
      }
    }
  }
  else
  {
#if JVET_Y0128_NON_CTC
    if (pu.cu->slice->getUseAmvpMergeMode())
#else
    if (!pu.cu->slice->getCheckLDC())
#endif
    {
      m_BinEncoder.encodeBin(0, Ctx::amFlagState());
      DTRACE(g_trace_ctx, D_SYNTAX, "amvp_merge_mode() am_flag=%d\n", 0);
    }
  }
}
#endif

#if JVET_AB0157_TMRL
void CABACWriter::cuTmrlFlag(const CodingUnit& cu)
{
#if !JVET_AD0082_TMRL_CONFIG
  if (!CU::allowTmrl(cu))
  {
    return;
  }
#endif
  const PredictionUnit* pu = cu.firstPU;
#if !JVET_AD0082_TMRL_CONFIG
#if JVET_W0123_TIMD_FUSION
  if (cu.timd)
  {
    CHECK(cu.tmrlFlag, "TMRL cannot combine with TIMD.");
    unsigned int bin = pu->multiRefIdx != MULTI_REF_LINE_IDX[0];
    m_BinEncoder.encodeBin(bin, Ctx::MultiRefLineIdx(5)); // TIMD MRL
    if (bin)
    {
      bin = pu->multiRefIdx != MULTI_REF_LINE_IDX[1];
      m_BinEncoder.encodeBin(bin, Ctx::MultiRefLineIdx(6)); // which line
    }
    DTRACE(g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) ref_idx=%d\n", bin, pu->lumaPos().x, pu->lumaPos().y, pu->multiRefIdx);
  }
  else
  {
#endif
#endif
    int ctxId = 0;
    m_BinEncoder.encodeBin(cu.tmrlFlag, Ctx::TmrlDerive(ctxId++));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_tmrl_flag() ctx=%d pos=(%d,%d) tmrl=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.tmrlFlag);
    if (cu.tmrlFlag)
    {
      const int maxNumCtxBins = (MRL_LIST_SIZE / MRL_IDX_RICE_CODE_DIVISOR) - 1;
      int mrlIdxPrefix = cu.tmrlListIdx / MRL_IDX_RICE_CODE_DIVISOR;
      for (int val = 0; val < maxNumCtxBins; val++)
      {
        unsigned int bin = (val == mrlIdxPrefix ? 0 : 1);
        m_BinEncoder.encodeBin(bin, Ctx::TmrlDerive(ctxId++));
        if (!bin)
        {
          break;
        }
      }

      uint32_t mrlIdxSuffix = uint32_t(cu.tmrlListIdx & (MRL_IDX_RICE_CODE_DIVISOR - 1));
      m_BinEncoder.encodeBin((mrlIdxSuffix & 1), Ctx::TmrlDerive(maxNumCtxBins + 1));
      m_BinEncoder.encodeBin(((mrlIdxSuffix >> 1) & 1), Ctx::TmrlDerive(maxNumCtxBins + 2));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_tmrl_idx() ctx=%d pos=(%d,%d) tmrlidx=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.tmrlListIdx);
    }
    else
    {
      CHECK(pu->multiRefIdx, "?");
    }
#if !JVET_AD0082_TMRL_CONFIG
#if JVET_W0123_TIMD_FUSION
  }
#endif
#endif
}
#endif
#if JVET_AE0059_INTER_CCCM
void CABACWriter::interCccm(const TransformUnit& tu)
{
  if (TU::interCccmAllowed(tu))
  {
    m_BinEncoder.encodeBin(tu.interCccm > 0 ? 1 : 0, Ctx::InterCccmFlag(0));
    DTRACE(g_trace_ctx, D_SYNTAX, "inter_cccm() pos=(%d,%d) inter_cccm_flag=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.interCccm > 0 ? 1 : 0);
  }
}
#endif
#if JVET_AF0073_INTER_CCP_MERGE
void CABACWriter::interCcpMerge(const TransformUnit& tu)
{
  if (TU::interCcpMergeAllowed(tu))
  {
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
    m_BinEncoder.encodeBin(tu.interCcpMerge > 0 ? 1 : 0, Ctx::InterCcpMergeFlag(tu.cbf[COMPONENT_Y] ? 0 : 1));
#else
    m_BinEncoder.encodeBin(tu.interCcpMerge > 0 ? 1 : 0, Ctx::InterCcpMergeFlag(0));
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "inter_ccp_merge() pos=(%d,%d) inter_ccp_merge_flag=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.interCcpMerge > 0 ? 1 : 0);
  }
}
#endif
//! \}
