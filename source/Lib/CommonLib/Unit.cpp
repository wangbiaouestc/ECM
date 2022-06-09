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

/** \file     Unit.cpp
 *  \brief    defines unit as a set of blocks and basic unit types (coding, prediction, transform)
 */

#include "Unit.h"

#include "Buffer.h"
#include "Picture.h"
#include "ChromaFormat.h"

#include "UnitTools.h"
#include "UnitPartitioner.h"

#include "ChromaFormat.h"

 // ---------------------------------------------------------------------------
 // block method definitions
 // ---------------------------------------------------------------------------

void CompArea::xRecalcLumaToChroma()
{
  const uint32_t csx = getComponentScaleX(compID, chromaFormat);
  const uint32_t csy = getComponentScaleY(compID, chromaFormat);

  x      >>= csx;
  y      >>= csy;
  width  >>= csx;
  height >>= csy;
}

Position CompArea::chromaPos() const
{
  if (isLuma(compID))
  {
    uint32_t scaleX = getComponentScaleX(compID, chromaFormat);
    uint32_t scaleY = getComponentScaleY(compID, chromaFormat);

    return Position(x >> scaleX, y >> scaleY);
  }
  else
  {
    return *this;
  }
}

Size CompArea::lumaSize() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width << scaleX, height << scaleY );
  }
  else
  {
    return *this;
  }
}

Size CompArea::chromaSize() const
{
  if( isLuma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Size( width >> scaleX, height >> scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::lumaPos() const
{
  if( isChroma( compID ) )
  {
    uint32_t scaleX = getComponentScaleX( compID, chromaFormat );
    uint32_t scaleY = getComponentScaleY( compID, chromaFormat );

    return Position( x << scaleX, y << scaleY );
  }
  else
  {
    return *this;
  }
}

Position CompArea::compPos( const ComponentID compID ) const
{
  return isLuma( compID ) ? lumaPos() : chromaPos();
}

Position CompArea::chanPos( const ChannelType chType ) const
{
  return isLuma( chType ) ? lumaPos() : chromaPos();
}

// ---------------------------------------------------------------------------
// unit method definitions
// ---------------------------------------------------------------------------

UnitArea::UnitArea(const ChromaFormat _chromaFormat) : chromaFormat(_chromaFormat) { }

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const Area &_area) : chromaFormat(_chromaFormat), blocks(getNumberValidComponents(_chromaFormat))
{
  const uint32_t numCh = getNumberValidComponents(chromaFormat);

  for (uint32_t i = 0; i < numCh; i++)
  {
    blocks[i] = CompArea(ComponentID(i), chromaFormat, _area, true);
  }
}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY) : chromaFormat(_chromaFormat), blocks { blkY } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY) } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat, const CompArea &blkY, const CompArea &blkCb, const CompArea &blkCr)  : chromaFormat(_chromaFormat), blocks { blkY, blkCb, blkCr } {}

UnitArea::UnitArea(const ChromaFormat _chromaFormat,       CompArea &&blkY,      CompArea &&blkCb,      CompArea &&blkCr) : chromaFormat(_chromaFormat), blocks { std::forward<CompArea>(blkY), std::forward<CompArea>(blkCb), std::forward<CompArea>(blkCr) } {}

bool UnitArea::contains(const UnitArea& other) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

bool UnitArea::contains( const UnitArea& other, const ChannelType chType ) const
{
  bool ret = true;
  bool any = false;

  for( const auto &blk : other.blocks )
  {
    if( toChannelType( blk.compID ) == chType && blk.valid() && blocks[blk.compID].valid() )
    {
      ret &= blocks[blk.compID].contains( blk );
      any = true;
    }
  }

  return any && ret;
}

#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
void UnitArea::resizeTo( const UnitArea& unitArea )
{
  for( uint32_t i = 0; i < blocks.size(); i++ )
  {
    blocks[i].resizeTo( unitArea.blocks[i] );
  }
}
#endif

void UnitArea::repositionTo(const UnitArea& unitArea)
{
  for(uint32_t i = 0; i < blocks.size(); i++)
  {
    blocks[i].repositionTo(unitArea.blocks[i]);
  }
}

const UnitArea UnitArea::singleComp(const ComponentID compID) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (blk.compID == compID)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

const UnitArea UnitArea::singleChan(const ChannelType chType) const
{
  UnitArea ret(chromaFormat);

  for (const auto &blk : blocks)
  {
    if (toChannelType(blk.compID) == chType)
    {
      ret.blocks.push_back(blk);
    }
    else
    {
      ret.blocks.push_back(CompArea());
    }
  }

  return ret;
}

// ---------------------------------------------------------------------------
// coding unit method definitions
// ---------------------------------------------------------------------------

CodingUnit::CodingUnit(const UnitArea &unit)                                : UnitArea(unit),                 cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }
CodingUnit::CodingUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cs(nullptr), slice(nullptr), chType( CH_L ), next(nullptr), firstPU(nullptr), lastPU(nullptr), firstTU(nullptr), lastTU(nullptr) { initData(); }

CodingUnit& CodingUnit::operator=( const CodingUnit& other )
{
  slice             = other.slice;
  predMode          = other.predMode;
  qtDepth           = other.qtDepth;
  depth             = other.depth;
  btDepth           = other.btDepth;
  mtDepth           = other.mtDepth;
  splitSeries       = other.splitSeries;
  skip              = other.skip;
  mmvdSkip = other.mmvdSkip;
  affine            = other.affine;
  affineType        = other.affineType;
  colorTransform = other.colorTransform;
  geoFlag           = other.geoFlag;
  bdpcmMode         = other.bdpcmMode;
  bdpcmModeChroma   = other.bdpcmModeChroma;
  qp                = other.qp;
  chromaQpAdj       = other.chromaQpAdj;
  rootCbf           = other.rootCbf;
  sbtInfo           = other.sbtInfo;
  mtsFlag           = other.mtsFlag;
  lfnstIdx          = other.lfnstIdx;
  tileIdx           = other.tileIdx;
#if ENABLE_DIMD
  dimd = other.dimd;
  dimdBlending = other.dimdBlending;
  dimdMode = other.dimdMode;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  dimdChromaMode = other.dimdChromaMode;
#endif
  for( int i = 0; i < 2; i++ )
  {
    dimdBlendMode[i] = other.dimdBlendMode[i];
  }

  for( int i = 0; i < 3; i++ )
  {
    dimdRelWeight[i] = other.dimdRelWeight[i];
  }
#endif
#if JVET_W0123_TIMD_FUSION
  timd              = other.timd;
  timdMode          = other.timdMode;
  timdModeSecondary = other.timdModeSecondary;
  timdIsBlended     = other.timdIsBlended;
  timdFusionWeight[0] = other.timdFusionWeight[0];
  timdFusionWeight[1] = other.timdFusionWeight[1];
#endif
#if ENABLE_OBMC
  obmcFlag          = other.obmcFlag;
  isobmcMC          = other.isobmcMC;
#endif
  imv               = other.imv;
  imvNumCand        = other.imvNumCand;
  BcwIdx            = other.BcwIdx;
  for (int i = 0; i<2; i++)
    refIdxBi[i] = other.refIdxBi[i];

  smvdMode        = other.smvdMode;
  ispMode           = other.ispMode;
  mipFlag           = other.mipFlag;
#if JVET_V0130_INTRA_TMP
  tmpFlag           = other.tmpFlag;
#endif
#if INTER_LIC
  LICFlag           = other.LICFlag;
#endif

  for (int idx = 0; idx < MAX_NUM_CHANNEL_TYPE; idx++)
  {
    curPLTSize[idx]   = other.curPLTSize[idx];
    useEscape[idx]    = other.useEscape[idx];
    useRotation[idx]  = other.useRotation[idx];
    reusePLTSize[idx] = other.reusePLTSize[idx];
    lastPLTSize[idx]  = other.lastPLTSize[idx];
    if (slice->getSPS()->getPLTMode())
    {
      memcpy(reuseflag[idx], other.reuseflag[idx], MAXPLTPREDSIZE * sizeof(bool));
    }
  }

  if (slice->getSPS()->getPLTMode())
  {
    for (int idx = 0; idx < MAX_NUM_COMPONENT; idx++)
    {
      memcpy(curPLT[idx], other.curPLT[idx], MAXPLTSIZE * sizeof(Pel));
    }
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  treeType          = other.treeType;
  modeType          = other.modeType;
  modeTypeSeries    = other.modeTypeSeries;
#endif
  return *this;
}

void CodingUnit::initData()
{
  predMode          = NUMBER_OF_PREDICTION_MODES;
  qtDepth           = 0;
  depth             = 0;
  btDepth           = 0;
  mtDepth           = 0;
  splitSeries       = 0;
  skip              = false;
  mmvdSkip = false;
  affine            = false;
  affineType        = 0;
  colorTransform = false;
  geoFlag           = false;
  bdpcmMode         = 0;
  bdpcmModeChroma   = 0;
  qp                = 0;
  chromaQpAdj       = 0;
  rootCbf           = true;
  sbtInfo           = 0;
  mtsFlag           = 0;
  lfnstIdx          = 0;
  tileIdx           = 0;
#if ENABLE_DIMD
  dimd = false;
  dimdBlending = false;
  dimdMode = -1;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  dimdChromaMode   = -1;
#endif
  for( int i = 0; i < 2; i++ )
  {
    dimdBlendMode[i] = -1;
  }

  for( int i = 0; i < 3; i++ )
  {
    dimdRelWeight[i] = -1;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  timd              = false;
  timdMode          = -1;
  timdModeSecondary = -1;
  timdIsBlended     = false;
  timdFusionWeight[0] = -1;
  timdFusionWeight[1] = -1;
#endif
#if ENABLE_OBMC
  obmcFlag          = true;
  isobmcMC          = false;
#endif
  imv               = 0;
  imvNumCand        = 0;
  BcwIdx            = BCW_DEFAULT;
  for (int i = 0; i < 2; i++)
    refIdxBi[i] = -1;
  smvdMode        = 0;
  ispMode           = 0;
  mipFlag           = false;
#if JVET_V0130_INTRA_TMP
  tmpFlag = false;
#endif
#if INTER_LIC
  LICFlag = false;
#endif

  for (int idx = 0; idx < MAX_NUM_CHANNEL_TYPE; idx++)
  {
    curPLTSize[idx]   = 0;
    reusePLTSize[idx] = 0;
    lastPLTSize[idx]  = 0;
    useEscape[idx]    = false;
    useRotation[idx]  = false;
    memset(reuseflag[idx], false, MAXPLTPREDSIZE * sizeof(bool));
  }

  for (int idx = 0; idx < MAX_NUM_COMPONENT; idx++)
  {
    memset(curPLT[idx], 0, MAXPLTSIZE * sizeof(Pel));
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  treeType          = TREE_D;
  modeType          = MODE_TYPE_ALL;
  modeTypeSeries    = 0;
#endif
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
const bool CodingUnit::isSepTree() const
{
  return treeType != TREE_D || CS::isDualITree( *cs );
}

const bool CodingUnit::isLocalSepTree() const
{
  return treeType != TREE_D && !CS::isDualITree(*cs);
}
#endif
#if !CCLM_LATENCY_RESTRICTION_RMV
const bool CodingUnit::checkCCLMAllowed() const
{
  bool allowCCLM = false;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( chType != CHANNEL_TYPE_CHROMA ) //single tree I slice or non-I slice
#else
  if( !CS::isDualITree( *cs ) ) //single tree I slice or non-I slice (Note: judging chType is no longer equivalent to checking dual-tree I slice since the local dual-tree is introduced)
#endif
  {
    allowCCLM = true;
  }
  else if( slice->getSPS()->getCTUSize() <= 32 ) //dual tree, CTUsize < 64
  {
    allowCCLM = true;
  }
  else //dual tree, CTU size 64 or 128
  {
#if TU_256
    int depthFor64x64Node = 0;
#else
    int depthFor64x64Node = slice->getSPS()->getCTUSize() == 128 ? 1 : 0;
#endif
    const PartSplit cuSplitTypeDepth1 = CU::getSplitAtDepth( *this, depthFor64x64Node );
    const PartSplit cuSplitTypeDepth2 = CU::getSplitAtDepth( *this, depthFor64x64Node + 1 );

    //allow CCLM if 64x64 chroma tree node uses QT split or HBT+VBT split combination
    if( cuSplitTypeDepth1 == CU_QUAD_SPLIT || (cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_VERT_SPLIT) )
    {
      if( chromaFormat == CHROMA_420 )
      {
#if TU_256
        CHECK( !( blocks[COMPONENT_Cb].width <= 32 && blocks[COMPONENT_Cb].height <= 32 ), "chroma cu size shall be <= 16x16 for YUV420 format" );
#else
        CHECK( !(blocks[COMPONENT_Cb].width <= 16 && blocks[COMPONENT_Cb].height <= 16), "chroma cu size shall be <= 16x16 for YUV420 format" );
#endif
      }
      allowCCLM = true;
    }
    //allow CCLM if 64x64 chroma tree node uses NS (No Split) and becomes a chroma CU containing 32x32 chroma blocks
    else if( cuSplitTypeDepth1 == CU_DONT_SPLIT )
    {
      if( chromaFormat == CHROMA_420 )
      {
#if TU_256
        CHECK( !( blocks[COMPONENT_Cb].width >= 32 && blocks[COMPONENT_Cb].height >= 32 ), "chroma cu size shall be 64x64 for YUV420 format" );
#else
        CHECK( !(blocks[COMPONENT_Cb].width == 32 && blocks[COMPONENT_Cb].height == 32), "chroma cu size shall be 32x32 for YUV420 format" );
#endif
      }
      allowCCLM = true;
    }
    //allow CCLM if 64x32 chroma tree node uses NS and becomes a chroma CU containing 32x16 chroma blocks
    else if( cuSplitTypeDepth1 == CU_HORZ_SPLIT && cuSplitTypeDepth2 == CU_DONT_SPLIT )
    {
      if( chromaFormat == CHROMA_420 )
      {
#if TU_256
        CHECK( !( blocks[COMPONENT_Cb].width >= 32 && blocks[COMPONENT_Cb].height >= 16 ), "chroma cu size shall be 64x32 for YUV420 format" );
#else
        CHECK( !(blocks[COMPONENT_Cb].width == 32 && blocks[COMPONENT_Cb].height == 16), "chroma cu size shall be 32x16 for YUV420 format" );
#endif
      }
      allowCCLM = true;
    }

    //further check luma conditions
    if( allowCCLM )
    {
      //disallow CCLM if luma 64x64 block uses BT or TT or NS with ISP
      const Position lumaRefPos( chromaPos().x << getComponentScaleX( COMPONENT_Cb, chromaFormat ), chromaPos().y << getComponentScaleY( COMPONENT_Cb, chromaFormat ) );
      const CodingUnit* colLumaCu = cs->picture->cs->getCU( lumaRefPos, CHANNEL_TYPE_LUMA );

#if TU_256
      if( colLumaCu->lwidth() < MAX_TB_SIZEY || colLumaCu->lheight() < MAX_TB_SIZEY ) //further split at CTU luma node
#else
      if( colLumaCu->lwidth() < 64 || colLumaCu->lheight() < 64 ) //further split at 64x64 luma node
#endif
      {
        const PartSplit cuSplitTypeDepth1Luma = CU::getSplitAtDepth( *colLumaCu, depthFor64x64Node );
        CHECK( !(cuSplitTypeDepth1Luma >= CU_QUAD_SPLIT && cuSplitTypeDepth1Luma <= CU_TRIV_SPLIT), "split mode shall be BT, TT or QT" );
        if( cuSplitTypeDepth1Luma != CU_QUAD_SPLIT )
        {
          allowCCLM = false;
        }
      }
#if TU_256
      else if( colLumaCu->lwidth() == MAX_TB_SIZEY && colLumaCu->lheight() == MAX_TB_SIZEY && colLumaCu->ispMode ) //not split at CTU luma node and use ISP mode
#else
      else if( colLumaCu->lwidth() == 64 && colLumaCu->lheight() == 64 && colLumaCu->ispMode ) //not split at 64x64 luma node and use ISP mode
#endif
      {
        allowCCLM = false;
      }
    }
  }

  return allowCCLM;
}
#endif
const uint8_t CodingUnit::checkAllowedSbt() const
{
  if( !slice->getSPS()->getUseSBT() )
  {
    return 0;
  }

  //check on prediction mode
  if (predMode == MODE_INTRA || predMode == MODE_IBC || predMode == MODE_PLT ) //intra, palette or IBC
  {
    return 0;
  }
  if( firstPU->ciipFlag )
  {
    return 0;
  }
#if JVET_Y0065_GPM_INTRA
  if (firstPU->gpmIntraFlag)
  {
    return 0;
  }
#endif

  uint8_t sbtAllowed = 0;
  int cuWidth  = lwidth();
  int cuHeight = lheight();
  bool allow_type[NUMBER_SBT_IDX];
  memset( allow_type, false, NUMBER_SBT_IDX * sizeof( bool ) );

  //parameter
  int maxSbtCUSize = cs->sps->getMaxTbSize();
  int minSbtCUSize = 1 << ( MIN_CU_LOG2 + 1 );

  //check on size
  if( cuWidth > maxSbtCUSize || cuHeight > maxSbtCUSize )
  {
    return 0;
  }

  allow_type[SBT_VER_HALF] = cuWidth  >= minSbtCUSize;
  allow_type[SBT_HOR_HALF] = cuHeight >= minSbtCUSize;
  allow_type[SBT_VER_QUAD] = cuWidth  >= ( minSbtCUSize << 1 );
  allow_type[SBT_HOR_QUAD] = cuHeight >= ( minSbtCUSize << 1 );

  for( int i = 0; i < NUMBER_SBT_IDX; i++ )
  {
    sbtAllowed += (uint8_t)allow_type[i] << i;
  }

  return sbtAllowed;
}

uint8_t CodingUnit::getSbtTuSplit() const
{
  uint8_t sbtTuSplitType = 0;

  switch( getSbtIdx() )
  {
  case SBT_VER_HALF: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_VER_HALF_POS0_SPLIT; break;
  case SBT_HOR_HALF: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_HOR_HALF_POS0_SPLIT; break;
  case SBT_VER_QUAD: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_VER_QUAD_POS0_SPLIT; break;
  case SBT_HOR_QUAD: sbtTuSplitType = ( getSbtPos() == SBT_POS0 ? 0 : 1 ) + SBT_HOR_QUAD_POS0_SPLIT; break;
  default: assert( 0 );  break;
  }

  assert( sbtTuSplitType <= SBT_HOR_QUAD_POS1_SPLIT && sbtTuSplitType >= SBT_VER_HALF_POS0_SPLIT );
  return sbtTuSplitType;
}

// ---------------------------------------------------------------------------
// prediction unit method definitions
// ---------------------------------------------------------------------------

PredictionUnit::PredictionUnit(const UnitArea &unit)                                : UnitArea(unit)                , cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }
PredictionUnit::PredictionUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next(nullptr) { initData(); }

void PredictionUnit::initData()
{
  // intra data - need this default initialization for PCM
#if SECONDARY_MPM
  ::memset(intraMPM, 0, sizeof(intraMPM));
  ::memset(intraNonMPM, 0, sizeof(intraNonMPM));
#endif

  intraDir[0] = DC_IDX;
  intraDir[1] = PLANAR_IDX;
#if JVET_Z0050_DIMD_CHROMA_FUSION
  isChromaFusion = false;
#endif
  mipTransposedFlag = false;
  multiRefIdx = 0;
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
  parseLumaMode = false;
  candId = -1;
  parseChromaMode = false;
  mpmFlag = false;
  ipred_idx = -1;
#endif
#if JVET_Z0050_CCLM_SLOPE
  cclmOffsets = {};
#endif
  // inter data
  mergeFlag   = false;
  regularMergeFlag = false;
  mergeIdx    = MAX_UCHAR;
  geoSplitDir  = MAX_UCHAR;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoSyntaxMode = MAX_UCHAR;
#endif
  geoMergeIdx0 = MAX_UCHAR;
  geoMergeIdx1 = MAX_UCHAR;
#if JVET_Y0065_GPM_INTRA
  gpmIntraFlag = false;
#endif
#if JVET_W0097_GPM_MMVD_TM
  geoMMVDFlag0 = false;
  geoMMVDIdx0 = MAX_UCHAR;
  geoMMVDFlag1 = false;
  geoMMVDIdx1 = MAX_UCHAR;
#if TM_MRG
  geoTmFlag0 = false;
  geoTmFlag1 = false;
#endif
#endif
  mmvdMergeFlag = false;
  mmvdMergeIdx = MAX_UCHAR;
#if AFFINE_MMVD
  afMmvdFlag    = false;
  afMmvdBaseIdx = UINT8_MAX;
  afMmvdStep    = UINT8_MAX;
  afMmvdDir     = UINT8_MAX;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  afMmvdMergeIdx = UINT8_MAX;
#endif
#endif
#if TM_MRG
  tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
  bmMergeFlag = false;
  bmDir = 0;
#endif
  interDir    = MAX_UCHAR;
  mergeType   = MRG_TYPE_DEFAULT_N;
  bv.setZero();
  bvd.setZero();
  mvRefine = false;
#if MULTI_PASS_DMVR
  bdmvrRefine = false;
#else
  ::memset(mvdL0SubPu, 0, sizeof(mvdL0SubPu));
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
  refIdxLC = -1;
  refPairIdx = -1;
#endif
  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMergeModeFlag[i] = false;
#endif
    mvpIdx[i] = MAX_UCHAR;
    mvpNum[i] = MAX_UCHAR;
    refIdx[i] = -1;
    mv[i]     .setZero();
    mvd[i]    .setZero();
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    mvsdIdx[i] = -1;
#endif
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j].setZero();
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j].setZero();
    }
  }
  ciipFlag = false;
#if CIIP_PDPC
  ciipPDPC = false;
#endif
  mmvdEncOptMode = 0;
#if MULTI_HYP_PRED
  addHypData.clear();
  numMergedAddHyps = 0;
#endif
}

PredictionUnit& PredictionUnit::operator=(const IntraPredictionData& predData)
{
#if SECONDARY_MPM
  ::memcpy(intraMPM, predData.intraMPM, sizeof(intraMPM));
  ::memcpy(intraNonMPM, predData.intraNonMPM, sizeof(intraNonMPM));
#endif

  for (uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    intraDir[i] = predData.intraDir[i];
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION
  isChromaFusion = predData.isChromaFusion;
#endif
  mipTransposedFlag = predData.mipTransposedFlag;
  multiRefIdx = predData.multiRefIdx;
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
  parseLumaMode = predData.parseLumaMode;
  candId = predData.candId;
  parseChromaMode = predData.parseChromaMode;
  mpmFlag = predData.mpmFlag;
  ipred_idx = predData.ipred_idx;
#endif
#if JVET_Z0050_CCLM_SLOPE
  cclmOffsets = predData.cclmOffsets;
#endif
  return *this;
}

PredictionUnit& PredictionUnit::operator=(const InterPredictionData& predData)
{
  mergeFlag   = predData.mergeFlag;
  regularMergeFlag = predData.regularMergeFlag;
  mergeIdx    = predData.mergeIdx;
  geoSplitDir  = predData.geoSplitDir;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoSyntaxMode = predData.geoSyntaxMode;
#endif
  geoMergeIdx0 = predData.geoMergeIdx0;
  geoMergeIdx1 = predData.geoMergeIdx1;
#if JVET_Y0065_GPM_INTRA
  gpmIntraFlag = predData.gpmIntraFlag;
#endif
#if JVET_W0097_GPM_MMVD_TM
  geoMMVDFlag0 = predData.geoMMVDFlag0;
  geoMMVDIdx0 = predData.geoMMVDIdx0;
  geoMMVDFlag1 = predData.geoMMVDFlag1;
  geoMMVDIdx1 = predData.geoMMVDIdx1;
#if TM_MRG
  geoTmFlag0 = predData.geoTmFlag0;
  geoTmFlag1 = predData.geoTmFlag1;
#endif
#endif
  mmvdMergeFlag = predData.mmvdMergeFlag;
  mmvdMergeIdx = predData.mmvdMergeIdx;
#if AFFINE_MMVD
  afMmvdFlag    = predData.afMmvdFlag;
  afMmvdBaseIdx = predData.afMmvdBaseIdx;
  afMmvdStep    = predData.afMmvdStep;
  afMmvdDir     = predData.afMmvdDir;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  afMmvdMergeIdx = predData.afMmvdMergeIdx;
#endif
#endif
#if TM_MRG
  tmMergeFlag = predData.tmMergeFlag;
#endif
#if JVET_X0049_ADAPT_DMVR
  bmMergeFlag = predData.bmMergeFlag;
  bmDir = predData.bmDir;
#endif
  interDir    = predData.interDir;
  mergeType   = predData.mergeType;
  bv          = predData.bv;
  bvd         = predData.bvd;
  mvRefine = predData.mvRefine;
#if MULTI_PASS_DMVR
  bdmvrRefine = predData.bdmvrRefine;
#else
  ::memcpy(mvdL0SubPu, predData.mvdL0SubPu, sizeof(mvdL0SubPu));
#endif

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMergeModeFlag[i] = predData.amvpMergeModeFlag[i];
#endif
    mvpIdx[i]   = predData.mvpIdx[i];
    mvpNum[i]   = predData.mvpNum[i];
    mv[i]       = predData.mv[i];
    mvd[i]      = predData.mvd[i];
    refIdx[i]   = predData.refIdx[i];
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    mvsdIdx[i] = predData.mvsdIdx[i];
#endif
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = predData.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = predData.mvAffi[i][j];
    }
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  refIdxLC = predData.refIdxLC;
  refPairIdx = predData.refPairIdx;
#endif
  ciipFlag = predData.ciipFlag;
#if CIIP_PDPC
  ciipPDPC = predData.ciipPDPC;
#endif
#if MULTI_HYP_PRED
  addHypData = predData.addHypData;
  numMergedAddHyps = predData.numMergedAddHyps;
#endif
  return *this;
}

PredictionUnit& PredictionUnit::operator=( const PredictionUnit& other )
{
#if SECONDARY_MPM
  ::memcpy(intraMPM, other.intraMPM, sizeof(intraMPM));
  ::memcpy(intraNonMPM, other.intraNonMPM, sizeof(intraNonMPM));
#endif

  for( uint32_t i = 0; i < MAX_NUM_CHANNEL_TYPE; i++ )
  {
    intraDir[ i ] = other.intraDir[ i ];
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION
  isChromaFusion = other.isChromaFusion;
#endif
  mipTransposedFlag = other.mipTransposedFlag;
  multiRefIdx = other.multiRefIdx;
#if JVET_Z0050_CCLM_SLOPE
  cclmOffsets = other.cclmOffsets;
#endif

  mergeFlag   = other.mergeFlag;
  regularMergeFlag = other.regularMergeFlag;
  mergeIdx    = other.mergeIdx;
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
  parseLumaMode = other.parseLumaMode;
  candId = other.candId;
  parseChromaMode = other.parseChromaMode;
  mpmFlag = other.mpmFlag;
  ipred_idx = other.ipred_idx;
#endif
  geoSplitDir  = other.geoSplitDir;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoSyntaxMode = other.geoSyntaxMode;
#endif
  geoMergeIdx0 = other.geoMergeIdx0;
  geoMergeIdx1 = other.geoMergeIdx1;
#if JVET_Y0065_GPM_INTRA
  gpmIntraFlag = other.gpmIntraFlag;
#endif
#if JVET_W0097_GPM_MMVD_TM
  geoMMVDFlag0 = other.geoMMVDFlag0;
  geoMMVDIdx0 = other.geoMMVDIdx0;
  geoMMVDFlag1 = other.geoMMVDFlag1;
  geoMMVDIdx1 = other.geoMMVDIdx1;
#if TM_MRG
  geoTmFlag0 = other.geoTmFlag0;
  geoTmFlag1 = other.geoTmFlag1;
#endif
#endif
  mmvdMergeFlag = other.mmvdMergeFlag;
  mmvdMergeIdx = other.mmvdMergeIdx;
#if AFFINE_MMVD
  afMmvdFlag    = other.afMmvdFlag;
  afMmvdBaseIdx = other.afMmvdBaseIdx;
  afMmvdStep    = other.afMmvdStep;
  afMmvdDir     = other.afMmvdDir;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  afMmvdMergeIdx = other.afMmvdMergeIdx;
#endif
#endif
#if TM_MRG
  tmMergeFlag = other.tmMergeFlag;
#endif
#if JVET_X0049_ADAPT_DMVR
  bmMergeFlag = other.bmMergeFlag;
  bmDir = other.bmDir;
#endif
  interDir    = other.interDir;
  mergeType   = other.mergeType;
  bv          = other.bv;
  bvd         = other.bvd;
  mvRefine = other.mvRefine;
#if MULTI_PASS_DMVR
  bdmvrRefine = other.bdmvrRefine;
#else
  ::memcpy(mvdL0SubPu, other.mvdL0SubPu, sizeof(mvdL0SubPu));
#endif

  for (uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMergeModeFlag[i] = other.amvpMergeModeFlag[i];
#endif
    mvpIdx[i]   = other.mvpIdx[i];
    mvpNum[i]   = other.mvpNum[i];
    mv[i]       = other.mv[i];
    mvd[i]      = other.mvd[i];
    refIdx[i]   = other.refIdx[i];
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    mvsdIdx[i] = other.mvsdIdx[i];
#endif
    for( uint32_t j = 0; j < 3; j++ )
    {
      mvdAffi[i][j] = other.mvdAffi[i][j];
    }
    for ( uint32_t j = 0; j < 3; j++ )
    {
      mvAffi[i][j] = other.mvAffi[i][j];
    }
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  refIdxLC = other.refIdxLC;
  refPairIdx = other.refPairIdx;
#endif
  ciipFlag = other.ciipFlag;
#if CIIP_PDPC
  ciipPDPC = other.ciipPDPC;
#endif
#if MULTI_HYP_PRED
  addHypData = other.addHypData;
  numMergedAddHyps = other.numMergedAddHyps;
#endif
  return *this;
}

PredictionUnit& PredictionUnit::operator=( const MotionInfo& mi )
{
  interDir = mi.interDir;

  for( uint32_t i = 0; i < NUM_REF_PIC_LIST_01; i++ )
  {
    refIdx[i] = mi.refIdx[i];
    mv    [i] = mi.mv[i];
  }
#if MULTI_HYP_PRED
  addHypData.clear();
  numMergedAddHyps = 0;
#endif

  return *this;
}

#if JVET_Z0139_HIST_AFF
void PredictionUnit::getAffineMotionInfo(AffineMotionInfo affineMiOut[2], int refIdxOut[2]) const
{

  for (int list = 0; list < 2; list++)
  {
    RefPicList eRefList = (list == 0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    refIdxOut[list] = NOT_VALID;
    affineMiOut[list].oneSetAffineParametersPattern = 0;

    if ((interDir == 1 && list == 1) || (interDir == 2 && list == 0))
    {
      continue;
    }
    refIdxOut[list] = refIdx[list];

    int affpara[4];
    PU::deriveAffineParametersFromMVs(*this, mvAffi[eRefList], affpara, (EAffineModel)this->cu->affineType);
    PU::storeAffParas(affpara);

    affineMiOut[list].oneSetAffineParameters[0] = (short)(affpara[0]);
    affineMiOut[list].oneSetAffineParameters[1] = (short)(affpara[1]);
    affineMiOut[list].oneSetAffineParameters[2] = (short)(affpara[2]);
    affineMiOut[list].oneSetAffineParameters[3] = (short)(affpara[3]);
  }
}
#endif

const MotionInfo& PredictionUnit::getMotionInfo() const
{
  return cs->getMotionInfo( lumaPos() );
}

const MotionInfo& PredictionUnit::getMotionInfo( const Position& pos ) const
{
#if JVET_Z0118_GDR
  if( cs->sps->getGDREnabledFlag() )
  {
    bool isSrcClean = cs->isClean(Y().bottomRight(), CHANNEL_TYPE_LUMA);
    bool isTarClean = cs->isClean(pos, CHANNEL_TYPE_LUMA);

    if (isSrcClean && !isTarClean)
    {
      return constMotionIntra;
    }
  }
#endif
  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getMotionInfo( pos );
}

MotionBuf PredictionUnit::getMotionBuf()
{
  return cs->getMotionBuf( *this );
}

CMotionBuf PredictionUnit::getMotionBuf() const
{
  return cs->getMotionBuf( *this );
}
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
bool PredictionUnit::isMvsdApplicable() const
{
  if (!cs->sps->getUseMVSD())
  {
    return false;
  }
  if (cu->firstPU->addHypData.size() - cu->firstPU->numMergedAddHyps > 0)
  {
    return false;
  }
  return true;
}
#endif
#if JVET_W0123_TIMD_FUSION
const uint8_t& PredictionUnit::getIpmInfo() const
{
  return cs->getIpmInfo( lumaPos() );
}

const uint8_t& PredictionUnit::getIpmInfo( const Position& pos ) const
{
#if JVET_Z0118_GDR
  if( cs->sps->getGDREnabledFlag() )
  {
    bool isSrcClean = cs->isClean(Y().bottomRight(), CHANNEL_TYPE_LUMA);
    bool isTarClean = cs->isClean(pos, CHANNEL_TYPE_LUMA);

    if (isSrcClean && !isTarClean)
    {
      return constIpm;    
    }
  }
#endif

  CHECKD( !Y().contains( pos ), "Trying to access motion info outsied of PU" );
  return cs->getIpmInfo( pos );
}

IpmBuf PredictionUnit::getIpmBuf()
{
  return cs->getIpmBuf( *this );
}

CIpmBuf PredictionUnit::getIpmBuf() const
{
  return cs->getIpmBuf( *this );
}
#endif


// ---------------------------------------------------------------------------
// transform unit method definitions
// ---------------------------------------------------------------------------

TransformUnit::TransformUnit(const UnitArea& unit) : UnitArea(unit), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
#if SIGN_PREDICTION
    m_coeffSigns[i] = nullptr;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = nullptr;
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = nullptr;
#endif
  }

  for (unsigned i = 0; i < MAX_NUM_TBLOCKS - 1; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = nullptr;
#endif
    m_runType[i] = nullptr;
  }

  initData();
}

TransformUnit::TransformUnit(const ChromaFormat _chromaFormat, const Area &_area) : UnitArea(_chromaFormat, _area), cu(nullptr), cs(nullptr), chType( CH_L ), next( nullptr )
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    m_coeffs[i] = nullptr;
#if SIGN_PREDICTION
    m_coeffSigns[i] = nullptr;
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = nullptr;
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = nullptr;
#endif
  }

  for (unsigned i = 0; i < MAX_NUM_TBLOCKS - 1; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = nullptr;
#endif
    m_runType[i] = nullptr;
  }

  initData();
}

void TransformUnit::initData()
{
  for( unsigned i = 0; i < MAX_NUM_TBLOCKS; i++ )
  {
    cbf[i]           = 0;
    mtsIdx[i]        = MTS_DCT2_DCT2;
  }
  depth              = 0;
  noResidual         = false;
  jointCbCr          = 0;
  m_chromaResScaleInv = 0;
}
#if REMOVE_PCM
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
void TransformUnit::init(TCoeff **coeffs, TCoeff **signs, unsigned **signsScanIdx, Pel **pltIdx, bool **runType)
#else
void TransformUnit::init(TCoeff **coeffs, TCoeff **signs, Pel **pltIdx, bool **runType)
#endif
#else
void TransformUnit::init(TCoeff **coeffs, Pel **pltIdx, bool **runType)
#endif
#else
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
void TransformUnit::init(TCoeff **coeffs, TCoeff **signs, unsigned **signsScanIdx, Pel **pcmbuf, bool **runType)
#else
void TransformUnit::init(TCoeff **coeffs, TCoeff **signs, Pel **pcmbuf, bool **runType)
#endif
#else
void TransformUnit::init(TCoeff **coeffs, Pel **pcmbuf, bool **runType)
#endif
#endif
{
  uint32_t numBlocks = getNumberValidTBlocks(*cs->pcv);

  for (uint32_t i = 0; i < numBlocks; i++)
  {
    m_coeffs[i] = coeffs[i];
#if SIGN_PREDICTION
    m_coeffSigns[i] = signs[i];
#if JVET_Y0141_SIGN_PRED_IMPROVE
    m_coeffSignsIdx[i] = signsScanIdx[i];
#endif
#endif
#if !REMOVE_PCM
    m_pcmbuf[i] = pcmbuf[i];
#endif
  }

  // numBlocks is either 1 for 4:0:0, or 3 otherwise. It would perhaps be better to loop over getNumberValidChannels(*cs->pcv.chrFormat) for m_runType.
  for (uint32_t i = 0; i < std::max<uint32_t>(2, numBlocks)-1; i++)
  {
#if REMOVE_PCM
    m_pltIdx[i] = pltIdx[i];
#endif
    m_runType[i] = runType[i];
  }
}

TransformUnit& TransformUnit::operator=(const TransformUnit& other)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  unsigned numBlocks = ::getNumberValidTBlocks(*cs->pcv);
  for( unsigned i = 0; i < numBlocks; i++ )
  {
    CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

    uint32_t area = blocks[i].area();

    if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
#if SIGN_PREDICTION
    if (m_coeffSigns[i] && other.m_coeffSigns[i] && m_coeffSigns[i] != other.m_coeffSigns[i]) memcpy(m_coeffSigns[i], other.m_coeffSigns[i], sizeof(TCoeff) * area);
#if JVET_Y0141_SIGN_PRED_IMPROVE
    if (m_coeffSignsIdx[i] && other.m_coeffSignsIdx[i] && m_coeffSignsIdx[i] != other.m_coeffSignsIdx[i]) memcpy(m_coeffSignsIdx[i], other.m_coeffSignsIdx[i], sizeof(unsigned) * area);
#endif
#endif
#if !REMOVE_PCM
    if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);
#endif
    if (cu->slice->getSPS()->getPLTMode() && i < 2)
    {
#if REMOVE_PCM
      if (m_pltIdx[i] && other.m_pltIdx[i] && m_pltIdx[i] != other.m_pltIdx[i]) memcpy(m_pltIdx[i], other.m_pltIdx[i], sizeof(Pel) * area);
#endif
      if (m_runType[i]   && other.m_runType[i]   && m_runType[i]   != other.m_runType[i]  ) memcpy(m_runType[i],   other.m_runType[i],   sizeof(bool) * area);
    }
    cbf[i]           = other.cbf[i];
    mtsIdx[i] = other.mtsIdx[i];
  }
  depth              = other.depth;
  noResidual         = other.noResidual;
  jointCbCr          = other.jointCbCr;
  return *this;
}

void TransformUnit::copyComponentFrom(const TransformUnit& other, const ComponentID i)
{
  CHECK( chromaFormat != other.chromaFormat, "Incompatible formats" );

  CHECKD( blocks[i].area() != other.blocks[i].area(), "Transformation units cover different areas" );

  uint32_t area = blocks[i].area();

  if (m_coeffs[i] && other.m_coeffs[i] && m_coeffs[i] != other.m_coeffs[i]) memcpy(m_coeffs[i], other.m_coeffs[i], sizeof(TCoeff) * area);
#if SIGN_PREDICTION
  if (m_coeffSigns[i] && other.m_coeffSigns[i] && m_coeffSigns[i] != other.m_coeffSigns[i]) memcpy(m_coeffSigns[i], other.m_coeffSigns[i], sizeof(TCoeff) * area);
#if JVET_Y0141_SIGN_PRED_IMPROVE
  if (m_coeffSignsIdx[i] && other.m_coeffSignsIdx[i] && m_coeffSignsIdx[i] != other.m_coeffSignsIdx[i]) memcpy(m_coeffSignsIdx[i], other.m_coeffSignsIdx[i], sizeof(unsigned) * area);
#endif
#endif
#if !REMOVE_PCM
  if (m_pcmbuf[i] && other.m_pcmbuf[i] && m_pcmbuf[i] != other.m_pcmbuf[i]) memcpy(m_pcmbuf[i], other.m_pcmbuf[i], sizeof(Pel   ) * area);
#endif
  if ((i == COMPONENT_Y || i == COMPONENT_Cb))
  {
#if REMOVE_PCM
    if (m_pltIdx[i] && other.m_pltIdx[i] && m_pltIdx[i] != other.m_pltIdx[i]) memcpy(m_pltIdx[i], other.m_pltIdx[i], sizeof(Pel) * area);
#endif
    if (m_runType[i] && other.m_runType[i] && m_runType[i] != other.m_runType[i])   memcpy(m_runType[i], other.m_runType[i], sizeof(bool) * area);
  }

  cbf[i]           = other.cbf[i];
  depth            = other.depth;
  mtsIdx[i]        = other.mtsIdx[i];
  noResidual       = other.noResidual;
  jointCbCr        = isChroma( i ) ? other.jointCbCr : jointCbCr;
}

       CoeffBuf TransformUnit::getCoeffs(const ComponentID id)       { return  CoeffBuf(m_coeffs[id], blocks[id]); }
const CCoeffBuf TransformUnit::getCoeffs(const ComponentID id) const { return CCoeffBuf(m_coeffs[id], blocks[id]); }

#if SIGN_PREDICTION
       CoeffBuf TransformUnit::getCoeffSigns(const ComponentID id)       { return  CoeffBuf(m_coeffSigns[id], blocks[id]); }
const CCoeffBuf TransformUnit::getCoeffSigns(const ComponentID id) const { return CCoeffBuf(m_coeffSigns[id], blocks[id]); }
#if JVET_Y0141_SIGN_PRED_IMPROVE
      IdxBuf    TransformUnit::getCoeffSignsScanIdx(const ComponentID id) { return  IdxBuf(m_coeffSignsIdx[id], blocks[id]); }
const CIdxBuf   TransformUnit::getCoeffSignsScanIdx(const ComponentID id) const { return CIdxBuf(m_coeffSignsIdx[id], blocks[id]); }
#endif
#endif

#if REMOVE_PCM
       PelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)         { return        PelBuf(m_pltIdx[id], blocks[id]); }
const CPelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)   const { return       CPelBuf(m_pltIdx[id], blocks[id]); }
#else
       PelBuf   TransformUnit::getPcmbuf(const ComponentID id)       { return  PelBuf  (m_pcmbuf[id], blocks[id]); }
const CPelBuf   TransformUnit::getPcmbuf(const ComponentID id) const { return CPelBuf  (m_pcmbuf[id], blocks[id]); }

       PelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)         { return        PelBuf(m_pcmbuf[id], blocks[id]); }
const CPelBuf       TransformUnit::getcurPLTIdx(const ComponentID id)   const { return       CPelBuf(m_pcmbuf[id], blocks[id]); }
#endif

       PLTtypeBuf   TransformUnit::getrunType  (const ComponentID id)         { return   PLTtypeBuf(m_runType[id], blocks[id]); }
const CPLTtypeBuf   TransformUnit::getrunType  (const ComponentID id)   const { return  CPLTtypeBuf(m_runType[id], blocks[id]); }

       PLTescapeBuf TransformUnit::getescapeValue(const ComponentID id)       { return  PLTescapeBuf(m_coeffs[id], blocks[id]); }
const CPLTescapeBuf TransformUnit::getescapeValue(const ComponentID id) const { return CPLTescapeBuf(m_coeffs[id], blocks[id]); }

#if REMOVE_PCM
      Pel*          TransformUnit::getPLTIndex   (const ComponentID id)       { return  m_pltIdx[id];    }
#else
      Pel*          TransformUnit::getPLTIndex   (const ComponentID id)       { return  m_pcmbuf[id];    }
#endif
      bool*         TransformUnit::getRunTypes   (const ComponentID id)       { return  m_runType[id];   }

void TransformUnit::checkTuNoResidual( unsigned idx )
{
  if( CU::getSbtIdx( cu->sbtInfo ) == SBT_OFF_DCT )
  {
    return;
  }

  if( ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS0 && idx == 1 ) || ( CU::getSbtPos( cu->sbtInfo ) == SBT_POS1 && idx == 0 ) )
  {
    noResidual = true;
  }
}

int TransformUnit::getTbAreaAfterCoefZeroOut(ComponentID compID) const
{
  int tbArea = blocks[compID].width * blocks[compID].height;
  int tbZeroOutWidth = blocks[compID].width;
  int tbZeroOutHeight = blocks[compID].height;

#if !TU_256
  if ( cs->sps->getUseMTS() && cu->sbtInfo != 0 && blocks[compID].width <= 32 && blocks[compID].height <= 32 && compID == COMPONENT_Y )
  {
    tbZeroOutWidth = (blocks[compID].width == 32) ? 16 : tbZeroOutWidth;
    tbZeroOutHeight = (blocks[compID].height == 32) ? 16 : tbZeroOutHeight;
  }
#endif
  tbZeroOutWidth = std::min<int>(JVET_C0024_ZERO_OUT_TH, tbZeroOutWidth);
  tbZeroOutHeight = std::min<int>(JVET_C0024_ZERO_OUT_TH, tbZeroOutHeight);
  tbArea = tbZeroOutWidth * tbZeroOutHeight;
  return tbArea;
}
#if JVET_Y0141_SIGN_PRED_IMPROVE
bool TransformUnit::checkLFNSTApplied(ComponentID compID)
{
  const uint32_t  lfnstIdx = this->cu->lfnstIdx;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool  lfnstApplied = lfnstIdx && this->mtsIdx[compID] != MTS_SKIP && (this->cu->isSepTree() ? true : isLuma(compID));
#else
  bool  lfnstApplied = lfnstIdx && this->mtsIdx[compID] != MTS_SKIP && (CS::isDualITree(*this->cs) ? true : isLuma(compID));
#endif
  return lfnstApplied;
}
#endif
int          TransformUnit::getChromaAdj()                     const { return m_chromaResScaleInv; }
void         TransformUnit::setChromaAdj(int i)                      { m_chromaResScaleInv = i;    }
