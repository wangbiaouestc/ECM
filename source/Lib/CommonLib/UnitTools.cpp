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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


uint64_t CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}

bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIntra() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  return isDualITree(cs) ? area.singleChan(chType) : area;
#else
  return isDualITree( cs ) || cs.treeType != TREE_D ? area.singleChan( chType ) : area;
#endif
}
#if !MULTI_PASS_DMVR
void CS::setRefinedMotionField(CodingStructure &cs)
{
  for (CodingUnit *cu : cs.cus)
  {
    for (auto &pu : CU::traversePUs(*cu))
    {
      PredictionUnit subPu = pu;
      int dx, dy, x, y, num = 0;
      dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
      dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
      Position puPos = pu.lumaPos();
      if (PU::checkDMVRCondition(pu))
      {
        for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy)
        {
          for (x = puPos.x; x < (puPos.x + pu.lumaSize().width); x = x + dx)
          {
            subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
            subPu.mv[0] = pu.mv[0];
            subPu.mv[1] = pu.mv[1];
            subPu.mv[REF_PIC_LIST_0] += pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_1] -= pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_0].clipToStorageBitDepth();
            subPu.mv[REF_PIC_LIST_1].clipToStorageBitDepth();
            pu.mvdL0SubPu[num].setZero();
            num++;
#if JVET_W0123_TIMD_FUSION
            PU::spanMotionInfo2(subPu);
#else
            PU::spanMotionInfo(subPu);
#endif
          }
        }
      }
    }
  }
}
#endif
// CU tools

bool CU::getRprScaling( const SPS* sps, const PPS* curPPS, Picture* refPic, int& xScale, int& yScale )
{
  const Window& curScalingWindow = curPPS->getScalingWindow();
  int curPicWidth = curPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps->getChromaFormatIdc() ) * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset());
  int curPicHeight = curPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( sps->getChromaFormatIdc() ) * (curScalingWindow.getWindowTopOffset()  + curScalingWindow.getWindowBottomOffset());

  const Window& refScalingWindow = refPic->getScalingWindow();
  int refPicWidth = refPic->getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps->getChromaFormatIdc() ) * (refScalingWindow.getWindowLeftOffset() + refScalingWindow.getWindowRightOffset());
  int refPicHeight = refPic->getPicHeightInLumaSamples() - SPS::getWinUnitY( sps->getChromaFormatIdc() ) * (refScalingWindow.getWindowTopOffset()  + refScalingWindow.getWindowBottomOffset());

  xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
  yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;

  int curSeqMaxPicWidthY = sps->getMaxPicWidthInLumaSamples();                  // pic_width_max_in_luma_samples
  int curSeqMaxPicHeightY = sps->getMaxPicHeightInLumaSamples();                // pic_height_max_in_luma_samples
  int curPicWidthY = curPPS->getPicWidthInLumaSamples();                        // pic_width_in_luma_samples
  int curPicHeightY = curPPS->getPicHeightInLumaSamples();                      // pic_height_in_luma_samples
  int max8MinCbSizeY = std::max((int)8, (1<<sps->getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)

  CHECK((curPicWidth * curSeqMaxPicWidthY) < refPicWidth * (curPicWidthY - max8MinCbSizeY), "(curPicWidth * curSeqMaxPicWidthY) should be greater than or equal to refPicWidth * (curPicWidthY - max8MinCbSizeY))");
  CHECK((curPicHeight * curSeqMaxPicHeightY) < refPicHeight * (curPicHeightY - max8MinCbSizeY), "(curPicHeight * curSeqMaxPicHeightY) should be greater than or equal to refPicHeight * (curPicHeightY - max8MinCbSizeY))");

  CHECK(curPicWidth * 2 < refPicWidth, "curPicWidth * 2 shall be greater than or equal to refPicWidth");
  CHECK(curPicHeight * 2 < refPicHeight, "curPicHeight * 2 shall be greater than or equal to refPicHeight");
  CHECK(curPicWidth > refPicWidth * 8, "curPicWidth shall be less than or equal to refPicWidth * 8");
  CHECK(curPicHeight > refPicHeight * 8, "curPicHeight shall be less than or equal to refPicHeight * 8");

#if JVET_S0048_SCALING_OFFSET
  int subWidthC = SPS::getWinUnitX(sps->getChromaFormatIdc());
  int subHeightC = SPS::getWinUnitY(sps->getChromaFormatIdc());

  CHECK(subWidthC * curScalingWindow.getWindowLeftOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_left_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * curScalingWindow.getWindowLeftOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_left_offset shall be less than pic_width_in_luma_samples");
  CHECK(subWidthC * curScalingWindow.getWindowRightOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_right_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * curScalingWindow.getWindowRightOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_right_offset shall be less than pic_width_in_luma_samples");

  CHECK(subHeightC * curScalingWindow.getWindowTopOffset() < (-curPicHeightY) * 15, "The value of SubHeightC * pps_scaling_win_top_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * curScalingWindow.getWindowTopOffset() >= curPicHeightY, "The value of SubHeightC * pps_scaling_win_top_offset shall be less than pps_pic_height_in_luma_samples");
  CHECK(subHeightC * curScalingWindow.getWindowBottomOffset() < (-curPicHeightY) * 15, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * curScalingWindow.getWindowBottomOffset() >= curPicHeightY, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be less than pps_pic_height_in_luma_samples");

  CHECK(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) < (-curPicWidthY) * 15, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) >= curPicWidthY, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be less than pic_width_in_luma_samples");
  CHECK(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) < (-curPicHeightY) * 15, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) >= curPicHeightY, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be less than pic_height_in_luma_samples");
#else
  CHECK(SPS::getWinUnitX(sps->getChromaFormatIdc()) * (abs(curScalingWindow.getWindowLeftOffset()) + abs(curScalingWindow.getWindowRightOffset())) > curPPS->getPicWidthInLumaSamples(), "The value of SubWidthC * ( Abs(pps_scaling_win_left_offset) + Abs(pps_scaling_win_right_offset) ) shall be less than pic_width_in_luma_samples");
  CHECK(SPS::getWinUnitY(sps->getChromaFormatIdc()) * (abs(curScalingWindow.getWindowTopOffset()) + abs(curScalingWindow.getWindowBottomOffset())) > curPPS->getPicHeightInLumaSamples(), "The value of SubHeightC * ( Abs(pps_scaling_win_top_offset) + Abs(pps_scaling_win_bottom_offset) ) shall be less than pic_height_in_luma_samples");
#endif

  return refPic->isRefScaled( curPPS );
}

void CU::checkConformanceILRP(Slice *slice)
{
  const int numRefList = slice->isInterB() ? 2 : 1;

#if JVET_S0258_SUBPIC_CONSTRAINTS
  int currentSubPicIdx = NOT_VALID;

  // derive sub-picture index for the current slice
  for( int subPicIdx = 0; subPicIdx < slice->getPic()->cs->sps->getNumSubPics(); subPicIdx++ )
  {
    if( slice->getPic()->cs->pps->getSubPic( subPicIdx ).getSubPicID() == slice->getSliceSubPicId() )
    {
      currentSubPicIdx = subPicIdx;
      break;
    }
  }

  CHECK( currentSubPicIdx == NOT_VALID, "Sub-picture was not found" );

  if( !slice->getPic()->cs->sps->getSubPicTreatedAsPicFlag( currentSubPicIdx ) )
  {
    return;
  }
#endif

  //constraint 1: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] has the same subpicture layout as the current picture
  bool isAllRefSameSubpicLayout = true;
  for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
  {
    RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
    {
#if JVET_S0258_SUBPIC_CONSTRAINTS
      const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );

      if( refPic->subPictures.size() != slice->getPic()->cs->pps->getNumSubPics() )
#else
      const Picture* refPic = slice->getRefPic(eRefPicList, refIdx)->unscaledPic;

      if (refPic->numSubpics != slice->getPic()->cs->pps->getNumSubPics())
#endif
      {
        isAllRefSameSubpicLayout = false;
        refList = numRefList;
        break;
      }
      else
      {
#if JVET_S0258_SUBPIC_CONSTRAINTS
        for( int i = 0; i < refPic->subPictures.size(); i++ )
        {
          const SubPic& refSubPic = refPic->subPictures[i];
          const SubPic& curSubPic = slice->getPic()->cs->pps->getSubPic( i );

          if( refSubPic.getSubPicWidthInCTUs() != curSubPic.getSubPicWidthInCTUs()
            || refSubPic.getSubPicHeightInCTUs() != curSubPic.getSubPicHeightInCTUs()
            || refSubPic.getSubPicCtuTopLeftX() != curSubPic.getSubPicCtuTopLeftX()
            || refSubPic.getSubPicCtuTopLeftY() != curSubPic.getSubPicCtuTopLeftY()
            || ( refPic->layerId != slice->getPic()->layerId && refSubPic.getSubPicID() != curSubPic.getSubPicID() )
            || refSubPic.getTreatedAsPicFlag() != curSubPic.getTreatedAsPicFlag())
#else
        for (int i = 0; i < refPic->numSubpics; i++)
        {
          if (refPic->subpicWidthInCTUs[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicWidthInCTUs()
            || refPic->subpicHeightInCTUs[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicHeightInCTUs()
            || refPic->subpicCtuTopLeftX[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicCtuTopLeftX()
            || refPic->subpicCtuTopLeftY[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicCtuTopLeftY())
#endif
          {
            isAllRefSameSubpicLayout = false;
            refIdx = slice->getNumRefIdx(eRefPicList);
            refList = numRefList;
            break;
          }
        }

#if JVET_S0258_SUBPIC_CONSTRAINTS
        // A picture with different sub-picture ID of the collocated sub-picture cannot be used as an active reference picture in the same layer
        if( refPic->layerId == slice->getPic()->layerId )
        {
          isAllRefSameSubpicLayout = isAllRefSameSubpicLayout && refPic->subPictures[currentSubPicIdx].getSubPicID() == slice->getSliceSubPicId();
        }
#endif
      }
    }
  }

  //constraint 2: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] is an ILRP for which the value of sps_num_subpics_minus1 is equal to 0
  if (!isAllRefSameSubpicLayout)
  {
    for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
    {
      RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
      for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
      {
#if JVET_S0258_SUBPIC_CONSTRAINTS
        const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );
        CHECK( refPic->layerId == slice->getPic()->layerId || refPic->subPictures.size() > 1, "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the current picture" );
#else
        const Picture* refPic = slice->getRefPic(eRefPicList, refIdx)->unscaledPic;
        CHECK(!(refPic->layerId != slice->getPic()->layerId && refPic->numSubpics == 1), "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the current picture");
#endif
      }
    }
  }

  return;
}

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isIBC(const CodingUnit &cu)
{
  return cu.predMode == MODE_IBC;
}

bool CU::isPLT(const CodingUnit &cu)
{
  return cu.predMode == MODE_PLT;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}

bool CU::isSameSubPic(const CodingUnit& cu, const CodingUnit& cu2)
{
  return (cu.slice->getPPS()->getSubPicFromCU(cu).getSubPicIdx() == cu2.slice->getPPS()->getSubPicFromCU(cu2).getSubPicIdx()) ;
}

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = floorLog2(cu.cs->sps->getMaxCUWidth());

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const Area cuAreaY = CS::isDualITree(*cu.cs) ? Area(recalcPosition(cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos()), recalcSize(cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size())) : (const Area&)cu.Y();
#else
  const Area cuAreaY = cu.isSepTree() ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : (const Area&)cu.Y();
#endif

  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == cu.cs->pps->getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == cu.cs->pps->getPicHeightInLumaSamples() ) );
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}
#if JVET_V0130_INTRA_TMP
Position CU::getCtuXYAddr(const CodingUnit& cu)
{
	return Position((cu.blocks[cu.chType].lumaPos().x >> cu.cs->pcv->maxCUWidthLog2) << cu.cs->pcv->maxCUWidthLog2, (cu.blocks[cu.chType].lumaPos().y >> cu.cs->pcv->maxCUHeightLog2) << cu.cs->pcv->maxCUHeightLog2);
}
#endif
int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

  uint32_t  ctuRsAddr       = getCtuAddr( cu );
  uint32_t  ctuXPosInCtus   = ctuRsAddr % cs.pcv->widthInCtus;
  uint32_t  tileColIdx      = cu.slice->getPPS()->ctuToTileCol( ctuXPosInCtus );
  uint32_t  tileXPosInCtus  = cu.slice->getPPS()->getTileColumnBd( tileColIdx );
  if( ctuXPosInCtus == tileXPosInCtus &&
      !( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) &&
      !( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) &&
      ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType) != NULL ) &&
      CU::isSameSliceAndTile( *cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType), cu ) )
  {
    return ( ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp );
  }
  else
  {
    const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType))->qp : prevQP;
    const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType))->qp : prevQP;

    return ( a + b + 1 ) >> 1;
  }
}


uint32_t CU::getNumPUs( const CodingUnit& cu )
{
  uint32_t cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
}

void CU::saveMotionInHMVP( const CodingUnit& cu, const bool isToBeDone )
{
  const PredictionUnit& pu = *cu.firstPU;

  if (!cu.geoFlag && !cu.affine && !isToBeDone)
  {
    MotionInfo mi = pu.getMotionInfo();
#if MULTI_HYP_PRED
    mi.addHypData = pu.addHypData;
#endif

    mi.BcwIdx = (mi.interDir == 3) ? cu.BcwIdx : BCW_DEFAULT;

    const unsigned log2ParallelMergeLevel = (pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2);
    const unsigned xBr = pu.cu->Y().width + pu.cu->Y().x;
    const unsigned yBr = pu.cu->Y().height + pu.cu->Y().y;
    bool enableHmvp = ((xBr >> log2ParallelMergeLevel) > (pu.cu->Y().x >> log2ParallelMergeLevel)) && ((yBr >> log2ParallelMergeLevel) > (pu.cu->Y().y >> log2ParallelMergeLevel));
    bool enableInsertion = CU::isIBC(cu) || enableHmvp;
    if (enableInsertion)
    cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut, mi);
  }
}

PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if (depth >= cu.depth)
  {
    return CU_DONT_SPLIT;
  }

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if (cuSplitType == CU_QUAD_SPLIT)
  {
    return CU_QUAD_SPLIT;
  }

  else if (cuSplitType == CU_HORZ_SPLIT)
  {
    return CU_HORZ_SPLIT;
  }

  else if (cuSplitType == CU_VERT_SPLIT)
  {
    return CU_VERT_SPLIT;
  }

  else if (cuSplitType == CU_TRIH_SPLIT)
  {
    return CU_TRIH_SPLIT;
  }
  else if (cuSplitType == CU_TRIV_SPLIT)
  {
    return CU_TRIV_SPLIT;
  }
  else
  {
    THROW("Unknown split mode");
    return CU_QUAD_SPLIT;
  }
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
ModeType CU::getModeTypeAtDepth( const CodingUnit& cu, const unsigned depth )
{
  ModeType modeType = ModeType( (cu.modeTypeSeries >> (depth * 3)) & 0x07 );
  CHECK( depth > cu.depth, " depth is wrong" );
  return modeType;
}
#endif


bool CU::divideTuInRows( const CodingUnit &cu )
{
  CHECK( cu.ispMode != HOR_INTRA_SUBPARTITIONS && cu.ispMode != VER_INTRA_SUBPARTITIONS, "Intra Subpartitions type not recognized!" );
  return cu.ispMode == HOR_INTRA_SUBPARTITIONS ? true : false;
}


PartSplit CU::getISPType( const CodingUnit &cu, const ComponentID compID )
{
  if( cu.ispMode && isLuma( compID ) )
  {
    const bool tuIsDividedInRows = CU::divideTuInRows( cu );

    return tuIsDividedInRows ? TU_1D_HORZ_SPLIT : TU_1D_VERT_SPLIT;
  }
  return TU_NO_ISP;
}

bool CU::isISPLast( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  PartSplit partitionType = CU::getISPType( cu, compID );

  Area originalArea = cu.blocks[compID];
  switch( partitionType )
  {
    case TU_1D_HORZ_SPLIT:
      return tuArea.y + tuArea.height == originalArea.y + originalArea.height;
    case TU_1D_VERT_SPLIT:
      return tuArea.x + tuArea.width == originalArea.x + originalArea.width;
    default:
      THROW( "Unknown ISP processing order type!" );
      return false;
  }
}

bool CU::isISPFirst( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  return tuArea == cu.firstTU->blocks[compID];
}

bool CU::canUseISP( const CodingUnit &cu, const ComponentID compID )
{
  const int width     = cu.blocks[compID].width;
  const int height    = cu.blocks[compID].height;
  const int maxTrSize = cu.cs->sps->getMaxTbSize();
  return CU::canUseISP( width, height, maxTrSize );
}

bool CU::canUseISP( const int width, const int height, const int maxTrSize )
{
  bool  notEnoughSamplesToSplit = ( floorLog2(width) + floorLog2(height) <= ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  bool  cuSizeLargerThanMaxTrSize = width > maxTrSize || height > maxTrSize;
  if ( notEnoughSamplesToSplit || cuSizeLargerThanMaxTrSize )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CompArea& cuArea, const ISPType ispSplitType )
{
  if( ispSplitType == NOT_INTRA_SUBPARTITIONS )
  {
    return false;
  }
  Size tuSize = ( ispSplitType == HOR_INTRA_SUBPARTITIONS ) ? Size( cuArea.width, CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_HORZ_SPLIT ) ) :
    Size( CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_VERT_SPLIT ), cuArea.height );

  if( !( tuSize.width >= MIN_TB_SIZEY && tuSize.height >= MIN_TB_SIZEY ) )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CodingUnit& cu, const ChannelType chType )
{
  CHECK( !isLuma( chType ), "Wrong ISP mode!" );
  return CU::canUseLfnstWithISP( cu.blocks[chType == CHANNEL_TYPE_LUMA ? 0 : 1], (ISPType)cu.ispMode );
}

#if JVET_W0119_LFNST_EXTENSION
Size CU::getLfnstSize( const CodingUnit& cu, const ChannelType chType )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int chIdx = CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#else
  int chIdx = cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
  Size lfnstSize;
  if( cu.ispMode )
  {
    CHECK( !isLuma( chType ), "Wrong ISP mode!" );
    const CompArea& cuArea = cu.blocks[ 0 ];
    lfnstSize = ( cu.ispMode == HOR_INTRA_SUBPARTITIONS ) ? Size( cuArea.width, CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_HORZ_SPLIT ) ) :
                Size( CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_VERT_SPLIT ), cuArea.height );
  }
  else
  {
    lfnstSize.width  = cu.blocks[ chIdx ].lumaSize().width;
    lfnstSize.height = cu.blocks[ chIdx ].lumaSize().height;
  }
  return lfnstSize;
}
#endif

uint32_t CU::getISPSplitDim( const int width, const int height, const PartSplit ispType )
{
  bool divideTuInRows = ispType == TU_1D_HORZ_SPLIT;
  uint32_t splitDimensionSize, nonSplitDimensionSize, partitionSize, divShift = 2;

  if( divideTuInRows )
  {
    splitDimensionSize    = height;
    nonSplitDimensionSize = width;
  }
  else
  {
    splitDimensionSize    = width;
    nonSplitDimensionSize = height;
  }

  const int minNumberOfSamplesPerCu = 1 << ( ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  const int factorToMinSamples = nonSplitDimensionSize < minNumberOfSamplesPerCu ? minNumberOfSamplesPerCu >> floorLog2(nonSplitDimensionSize) : 1;
  partitionSize = ( splitDimensionSize >> divShift ) < factorToMinSamples ? factorToMinSamples : ( splitDimensionSize >> divShift );

  CHECK( floorLog2(partitionSize) + floorLog2(nonSplitDimensionSize) < floorLog2(minNumberOfSamplesPerCu), "A partition has less than the minimum amount of samples!" );
  return partitionSize;
}

bool CU::allLumaCBFsAreZero(const CodingUnit& cu)
{
  if (!cu.ispMode)
  {
    return TU::getCbf(*cu.firstTU, COMPONENT_Y) == false;
  }
  else
  {
    int numTotalTUs = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(cu.firstTU->lheight()) : cu.lwidth() >> floorLog2(cu.firstTU->lwidth());
    TransformUnit* tuPtr = cu.firstTU;
    for (int tuIdx = 0; tuIdx < numTotalTUs; tuIdx++)
    {
      if (TU::getCbf(*tuPtr, COMPONENT_Y) == true)
      {
        return false;
      }
      tuPtr = tuPtr->next;
    }
    return true;
  }
}

#if JVET_W0123_TIMD_FUSION
TEMPLATE_TYPE CU::deriveTimdRefType( int iCurX, int iCurY, uint32_t uiCurWidth, uint32_t uiCurHeight, int iTemplateWidth, int iTemplateHeight, int& iRefX, int& iRefY, uint32_t& uiRefWidth, uint32_t& uiRefHeight )
{
  if(iCurX == 0 && iCurY == 0)
  {
    return NO_NEIGHBOR;
  }

  TEMPLATE_TYPE eTempType = NO_NEIGHBOR;
  iRefX = iRefY = -1;
  if(iCurX > 0 && iCurY > 0)
  {
    iRefX       = iCurX - iTemplateWidth;
    iRefY       = iCurY - iTemplateHeight;
    uiRefWidth  = uiCurWidth + iTemplateWidth;
    uiRefHeight = uiCurHeight + iTemplateHeight;
    eTempType   = LEFT_ABOVE_NEIGHBOR;
  }
  else if(iCurX == 0 && iCurY > 0)
  {
    iRefX       = iCurX;
    iRefY       = iCurY - iTemplateHeight;
    uiRefWidth  = uiCurWidth;
    uiRefHeight = uiCurHeight;
    eTempType   = ABOVE_NEIGHBOR;
  }
  else if(iCurX > 0 && iCurY == 0)
  {
    iRefX       = iCurX - iTemplateWidth;
    iRefY       = iCurY;
    uiRefWidth  = uiCurWidth;
    uiRefHeight = uiCurHeight;
    eTempType   = LEFT_NEIGHBOR;
  }
  else
  {
    assert(0);
  }
  return eTempType;
}
#endif


PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

// PU tools

#if SECONDARY_MPM
int PU::getIntraMPMs( const PredictionUnit &pu, uint8_t* mpm, uint8_t* non_mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/ )
#else
int PU::getIntraMPMs(const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/)
#endif
{
#if SECONDARY_MPM
  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));

  int numValidMPM = 0;
  mpm[numValidMPM++] = PLANAR_IDX;
  includedMode[PLANAR_IDX] = true;
#endif

  const int numMPMs = NUM_MOST_PROBABLE_MODES;
  {
    CHECK(channelType != CHANNEL_TYPE_LUMA, "Not harmonized yet");
    int numCand      = -1;
#if !SECONDARY_MPM
    int leftIntraDir = PLANAR_IDX, aboveIntraDir = PLANAR_IDX;
#endif

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position posRT = area.topRight();
    const Position posLB = area.bottomLeft();

    // Get intra direction of left PU
#if SECONDARY_MPM
    const PredictionUnit *puLeft = (pu.lheight() >= pu.lwidth())
      ? pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType)
      : pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
#else
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
#endif
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
#if SECONDARY_MPM
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puLeft)) : PU::getIntraDirLuma(*puLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puLeft);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
#else
      leftIntraDir = PU::getIntraDirLuma( *puLeft );
#if JVET_W0123_TIMD_FUSION
      if (puLeft->cu->timd)
      {
        leftIntraDir = MAP131TO67(leftIntraDir);
      }
#endif
#endif
    }
#if JVET_W0123_TIMD_FUSION && !SECONDARY_MPM
    if (puLeft && CU::isInter(*puLeft->cu))
    {
      leftIntraDir = puLeft->getIpmInfo(posLB.offset(-1, 0));
    }
#endif

    // Get intra direction of above PU
#if SECONDARY_MPM
    const PredictionUnit *puAbove = (pu.lheight() >= pu.lwidth())
      ? pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType)
      : pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
#else
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
#endif
    if (puAbove && CU::isIntra(*puAbove->cu) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
#if SECONDARY_MPM
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAbove->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAbove)) : PU::getIntraDirLuma(*puAbove);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAbove);
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
#else
      aboveIntraDir = PU::getIntraDirLuma(*puAbove);
#if JVET_W0123_TIMD_FUSION
      if (puAbove->cu->timd)
      {
        aboveIntraDir = MAP131TO67(aboveIntraDir);
      }
#endif
#endif
    }
#if JVET_W0123_TIMD_FUSION && !SECONDARY_MPM
    if (puAbove && CU::isInter(*puAbove->cu))
    {
      aboveIntraDir = puAbove->getIpmInfo(posRT.offset(0, -1));
    }
#endif

#if SECONDARY_MPM
#if JVET_W0123_TIMD_FUSION
    if (puLeft && CU::isInter(*puLeft->cu))
    {
      mpm[numValidMPM] = puLeft->getIpmInfo(pu.lheight() >= pu.lwidth() ? posRT.offset(0, -1) : posLB.offset(-1, 0));
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
    if (puAbove && CU::isInter(*puAbove->cu))
    {
      mpm[numValidMPM] = puAbove->getIpmInfo(pu.lheight() >= pu.lwidth() ? posLB.offset(-1, 0) : posRT.offset(0, -1));
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
#endif
    // Get intra direction of below-left PU
    const PredictionUnit *puBelowLeft = pu.cs->getPURestricted(posLB.offset(-1, 1), pu, channelType);
    if (puBelowLeft && CU::isIntra(*puBelowLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puBelowLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puBelowLeft)) : PU::getIntraDirLuma(*puBelowLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puBelowLeft);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }

    // Get intra direction of above-right PU
    const PredictionUnit *puAboveRight = pu.cs->getPURestricted(posRT.offset(1, -1), pu, channelType);
    if (puAboveRight && CU::isIntra(*puAboveRight->cu) && CU::isSameCtu(*pu.cu, *puAboveRight->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAboveRight->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveRight)) : PU::getIntraDirLuma(*puAboveRight);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveRight);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }

    // Get intra direction of above-left PU
    const Position posTL = area.topLeft();
    const PredictionUnit *puAboveLeft = pu.cs->getPURestricted(posTL.offset(-1, -1), pu, channelType);
    if (puAboveLeft && CU::isIntra(*puAboveLeft->cu) && CU::isSameCtu(*pu.cu, *puAboveLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAboveLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveLeft)) : PU::getIntraDirLuma(*puAboveLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveLeft);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
#if JVET_W0123_TIMD_FUSION
    if (puBelowLeft && CU::isInter(*puBelowLeft->cu))
    {
      mpm[numValidMPM] = puBelowLeft->getIpmInfo(posLB.offset(-1, 1));
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
    if (puAboveRight && CU::isInter(*puAboveRight->cu))
    {
      mpm[numValidMPM] = puAboveRight->getIpmInfo(posRT.offset(1, -1));
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
    if (puAboveLeft && CU::isInter(*puAboveLeft->cu))
    {
      mpm[numValidMPM] = puAboveLeft->getIpmInfo(posTL.offset(-1, -1));
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
#endif
#endif

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = ( int ) NUM_LUMA_MODE - 6;
    const int mod = offset + 3;

    {
#if SECONDARY_MPM
      numCand = numValidMPM;
#else
      mpm[0] = PLANAR_IDX;
      mpm[1] = DC_IDX;
      mpm[2] = VER_IDX;
      mpm[3] = HOR_IDX;
      mpm[4] = VER_IDX - 4;
      mpm[5] = VER_IDX + 4;
#endif
#if ENABLE_DIMD && SECONDARY_MPM
      //adding dimd modes
      if (pu.cu->slice->getSPS()->getUseDimd())
      {
        if (pu.cu->dimdMode != -1)
        {
          mpm[numValidMPM] = pu.cu->dimdMode;
          if( !includedMode[mpm[numValidMPM]] )
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }

        if (pu.cu->dimdBlendMode[0] != -1)
        {
          mpm[numValidMPM] = pu.cu->dimdBlendMode[0];
          if( !includedMode[mpm[numValidMPM]] )
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }
      }
#endif

#if SECONDARY_MPM
      bool checkDCEnabled = false;

      // Derived modes of mpm[1]
      if (numCand >= 2)
      {
        if (mpm[1] > DC_IDX)
        {
          for (int i = 0; i < 4 && numValidMPM < numMPMs; i++)
          {
            mpm[numValidMPM] = ((mpm[1] + offset - i) % mod) + 2;
            if( !includedMode[mpm[numValidMPM]] )
            {
              includedMode[mpm[numValidMPM++]] = true;
            }

            if( numValidMPM >= numMPMs )
            {
              break;
            }

            mpm[numValidMPM] = ((mpm[1] - 1 + i) % mod) + 2;
            if( !includedMode[mpm[numValidMPM]] )
            {
              includedMode[mpm[numValidMPM++]] = true;
            }
          }
        }
        else if( mpm[1] == DC_IDX )
        {
          checkDCEnabled = true;
        }
      }


      // Derived modes of mpm[2]
      if (numCand >= 3)
      {
        if (mpm[2] > DC_IDX)
        {
          for (int i = 0; i < 4 && numValidMPM < numMPMs; i++)
          {
            mpm[numValidMPM] = ((mpm[2] + offset - i) % mod) + 2;
            if( !includedMode[mpm[numValidMPM]] )
            {
              includedMode[mpm[numValidMPM++]] = true;
            }

            if (numValidMPM >= numMPMs)
              break;

            mpm[numValidMPM] = ((mpm[2] - 1 + i) % mod) + 2;
            if( !includedMode[mpm[numValidMPM]] )
            {
              includedMode[mpm[numValidMPM++]] = true;
            }
          }
        }
        else if( mpm[2] == DC_IDX )
        {
          checkDCEnabled = true;
        }
      }


      // Derived modes of mpm[3]
      if (checkDCEnabled && numCand >= 4 && mpm[3] > DC_IDX)
      {
        for (int i = 0; i < 3 && numValidMPM < numMPMs; i++)
        {
          mpm[numValidMPM] = ((mpm[3] + offset - i) % mod) + 2;
          if( !includedMode[mpm[numValidMPM]] )
          {
            includedMode[mpm[numValidMPM++]] = true;
          }

          if( numValidMPM >= numMPMs )
          {
            break;
          }

          mpm[numValidMPM] = ((mpm[3] - 1 + i) % mod) + 2;
          if( !includedMode[mpm[numValidMPM]] )
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }
      }
#else
      if (leftIntraDir == aboveIntraDir)
      {
        numCand = 1;
        if (leftIntraDir > DC_IDX)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = ((leftIntraDir + offset) % mod) + 2;
          mpm[3] = ((leftIntraDir - 1) % mod) + 2;
          mpm[4] = ((leftIntraDir + offset - 1) % mod) + 2;
          mpm[5] = ( leftIntraDir               % mod) + 2;
        }
      }
      else //L!=A
      {
        numCand = 2;
        int  maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;

        if ((leftIntraDir > DC_IDX) && (aboveIntraDir > DC_IDX))
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = aboveIntraDir;
          maxCandModeIdx = mpm[1] > mpm[2] ? 1 : 2;
          int minCandModeIdx = mpm[1] > mpm[2] ? 2 : 1;
          if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1)
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset)     % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
            mpm[5] = ((mpm[minCandModeIdx] + offset - 1) % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ( mpm[minCandModeIdx]           % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] - 1)      % mod) + 2;
          }
          else
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          }
        }
        else if (leftIntraDir + aboveIntraDir >= 2)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = (leftIntraDir < aboveIntraDir) ? aboveIntraDir : leftIntraDir;
          maxCandModeIdx = 1;
          mpm[2] = ((mpm[maxCandModeIdx] + offset)     % mod) + 2;
          mpm[3] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
          mpm[5] = ( mpm[maxCandModeIdx]               % mod) + 2;
        }
      }
#endif

#if SECONDARY_MPM
      unsigned mpm_default[numMPMs - 1] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, 14, 22, 42, 58, 10, 26,
                                           38, 62, 6, 30, 34, 66, 2, 48, 52, 16 };
      for (int idx = 0; (idx < numMPMs - 1) && numValidMPM < numMPMs; idx++)
      {
        mpm[numValidMPM] = mpm_default[idx];
        if( !includedMode[mpm[numValidMPM]] )
        {
          includedMode[mpm[numValidMPM++]] = true;
        }
      }

      int numNonMPM = 0;
      for (int idx = 0; idx < NUM_LUMA_MODE; idx++)
      {
        if( !includedMode[idx] )
        {
          non_mpm[numNonMPM++] = idx;
        }
      }
#endif
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK(numCand == 0, "No candidates found");
    return numCand;
  }
}

bool PU::isMIP(const PredictionUnit &pu, const ChannelType &chType)
{
  if (chType == CHANNEL_TYPE_LUMA)
  {
    // Default case if chType is omitted.
    return pu.cu->mipFlag;
  }
  else
  {
    return isDMChromaMIP(pu) && (pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX);
  }
}
#if JVET_V0130_INTRA_TMP
bool PU::isTmp(const PredictionUnit& pu, const ChannelType& chType)
{
	return (chType == CHANNEL_TYPE_LUMA && pu.cu->tmpFlag);
}
#endif
bool PU::isDMChromaMIP(const PredictionUnit &pu)
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  return !pu.cu->isSepTree() && (pu.chromaFormat == CHROMA_444) && getCoLocatedLumaPU(pu).cu->mipFlag;
#else
  return !(CS::isDualITree(*pu.cs)) && (pu.chromaFormat == CHROMA_444) && getCoLocatedLumaPU(pu).cu->mipFlag;
#endif
}

uint32_t PU::getIntraDirLuma( const PredictionUnit &pu )
{
#if JVET_V0130_INTRA_TMP
	if (isMIP(pu) || isTmp(pu))
#else
  if (isMIP(pu))
#endif
  {
    return PLANAR_IDX;
  }
  else
  {
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}

void PU::getIntraChromaCandModes(const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE])
{
  modeList[0] = PLANAR_IDX;
  modeList[1] = VER_IDX;
  modeList[2] = HOR_IDX;
  modeList[3] = DC_IDX;
  modeList[4] = LM_CHROMA_IDX;
#if MMLM
  modeList[5] = MDLM_L_IDX;
  modeList[6] = MDLM_T_IDX;
  modeList[7] = MMLM_CHROMA_IDX;
  modeList[8] = MMLM_L_IDX;
  modeList[9] = MMLM_T_IDX;
  modeList[10] = DM_CHROMA_IDX;
#else
  modeList[5] = MDLM_L_IDX;
  modeList[6] = MDLM_T_IDX;
  modeList[7] = DM_CHROMA_IDX;
#endif

  // If Direct Mode is MIP, mode cannot be already in the list.
  if (isDMChromaMIP(pu))
  {
    return;
  }

  const uint32_t lumaMode = getCoLocatedIntraLumaMode(pu);
  for (int i = 0; i < 4; i++)
  {
    if (lumaMode == modeList[i])
    {
      modeList[i] = VDIA_IDX;
      break;
    }
  }
}

bool PU::isLMCMode(unsigned mode)
{
#if MMLM
  return (mode >= LM_CHROMA_IDX && mode <= MMLM_T_IDX);
#else
  return (mode >= LM_CHROMA_IDX && mode <= MDLM_T_IDX);
#endif
}
#if MMLM
bool PU::isMultiModeLM(unsigned mode)
{
  return (mode == MMLM_CHROMA_IDX || mode == MMLM_L_IDX || mode == MMLM_T_IDX);
}
#endif
bool PU::isLMCModeEnabled(const PredictionUnit &pu, unsigned mode)
{
#if CCLM_LATENCY_RESTRICTION_RMV
  if (pu.cs->sps->getUseLMChroma() )
#else
  if ( pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed() )
#endif
  {
    return true;
  }
  return false;
}

int PU::getLMSymbolList(const PredictionUnit &pu, int *modeList)
{
  int idx = 0;

  modeList[idx++] = LM_CHROMA_IDX;
#if MMLM
  modeList[idx++] = MMLM_CHROMA_IDX;
#endif
  modeList[idx++] = MDLM_L_IDX;
  modeList[idx++] = MDLM_T_IDX;
#if MMLM
  modeList[idx++] = MMLM_L_IDX;
  modeList[idx++] = MMLM_T_IDX;
#endif
  return idx;
}

bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return !pu.cu->bdpcmModeChroma && pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    uiIntraMode = getCoLocatedIntraLumaMode(pu);
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) && uiIntraMode < NUM_LUMA_MODE ) // map directional, planar and dc
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
  return uiIntraMode;
}

const PredictionUnit &PU::getCoLocatedLumaPU(const PredictionUnit &pu)
{
  Position              topLeftPos = pu.blocks[pu.chType].lumaPos();
  Position              refPos     = topLeftPos.offset(pu.blocks[pu.chType].lumaSize().width  >> 1,
                                                       pu.blocks[pu.chType].lumaSize().height >> 1);
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const PredictionUnit &lumaPU     = CS::isDualITree(*pu.cs) ? *pu.cs->picture->cs->getPU(refPos, CHANNEL_TYPE_LUMA)
                                                        : *pu.cs->getPU(topLeftPos, CHANNEL_TYPE_LUMA);
#else
  const PredictionUnit &lumaPU     = pu.cu->isSepTree() ? *pu.cs->picture->cs->getPU(refPos, CHANNEL_TYPE_LUMA)
                                                        : *pu.cs->getPU(topLeftPos, CHANNEL_TYPE_LUMA);
#endif
  return lumaPU;
}

uint32_t PU::getCoLocatedIntraLumaMode(const PredictionUnit &pu)
{
#if JVET_W0123_TIMD_FUSION
  if (PU::getCoLocatedLumaPU(pu).cu->timd)
  {
    return MAP131TO67(PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu)));
  }
#endif
  return PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu));
}

int PU::getWideAngle( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID )
{
  //This function returns a wide angle index taking into account that the values 0 and 1 are reserved
  //for Planar and DC respectively, as defined in the Spec. Text.
  if( dirMode < 2 )
  {
    return ( int ) dirMode;
  }

  const CompArea&  area         = tu.cu->ispMode && isLuma(compID) ? tu.cu->blocks[compID] : tu.blocks[ compID ];
  int              width        = area.width;
  int              height       = area.height;
  int              modeShift[ ] = { 0, 6, 10, 12, 14, 15 };
  int              deltaSize    = abs( floorLog2( width ) - floorLog2( height ) );
  int              predMode     = dirMode;

  if( width > height && dirMode < 2 + modeShift[ deltaSize ] )
  {
    predMode += ( VDIA_IDX - 1 );
  }
  else if( height > width && predMode > VDIA_IDX - modeShift[ deltaSize ] )
  {
    predMode -= ( VDIA_IDX + 1 );
  }

  return predMode;
}

#if JVET_W0119_LFNST_EXTENSION
int PU::getLFNSTMatrixDim( int width, int height )
{
  int dimension = ( width == 8 && height == 8 ) ? 16 : ( ( ( width >= 8 ) && ( height >= 8 ) ) ? 32 : 16 );

  if( ( width >= 16 ) && ( height >= 16 ) )
  {
    dimension = L16H;
  }

  return dimension;
}

bool PU::getUseLFNST8( int width, int height )
{
  return ( width >= 8 ) && ( height >= 8 );
}

bool PU::getUseLFNST16( int width, int height )
{
  return ( width >= 16 ) && ( height >= 16 );
}

uint8_t PU::getLFNSTIdx( int intraMode, int mtsMode )
{
  return g_lfnstLut[ intraMode ];
}
#endif

bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
                          const bool ibcFlag, const bool isGt4x4
#if JVET_X0083_BM_AMVP_MERGE_MODE
                        , const PredictionUnit &pu
                        , const int curPoc
                        , const int amvpPoc
#endif
#if TM_MRG
                        , const uint32_t mvdSimilarityThresh
#endif
  )
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
  int num_avai_candInLUT = (int)lut.size();

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];

#if JVET_X0083_BM_AMVP_MERGE_MODE
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miNeighbor.refIdx);
    if (isValidAmMode &&
        ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
          || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
          )
#else
    if ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
#endif
    {
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = miNeighbor.usesLIC;

      if (ibcFlag)
      {
        CHECK(mrgCtx.LICFlags[cnt], "addMergeHMVPCand: LIC is not used with IBC mode")
      }
#endif
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
      mrgCtx.useAltHpelIf      [cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
      mrgCtx.BcwIdx            [cnt] = (miNeighbor.interDir == 3) ? miNeighbor.BcwIdx : BCW_DEFAULT;

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
      }
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt] = miNeighbor.addHypData;
#endif

#if NON_ADJACENT_MRG_CAND
      if (mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                            , mvdSimilarityThresh
#endif
      ))
      {
        continue;
      }
#endif
      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt ++;

      if (cnt  == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
  return false;
}

void PU::getIBCMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumIBCMergeCand();
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[ui * 2].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui * 2 + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if INTER_LIC
    mrgCtx.LICFlags[ui] = false;
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU

  int cnt = 0;

  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  bool isGt4x4 = pu.lwidth() * pu.lheight() > 16;
  const bool isAvailableA1 = puLeft && pu.cu != puLeft->cu && CU::isIBC(*puLeft->cu);
  if (isGt4x4 && isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    if (mrgCandIdx == cnt)
    {
      return;
    }
    cnt++;
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool isAvailableB1 = puAbove && pu.cu != puAbove->cu && CU::isIBC(*puAbove->cu);
  if (isGt4x4 && isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (!isAvailableA1 || (miAbove != miLeft))
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      if (mrgCandIdx == cnt)
      {
        return;
      }

      cnt++;
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

  if (cnt != maxNumMergeCand)
  {
    bool bFound = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCand, cnt, isAvailableA1, miLeft, isAvailableB1,
#if JVET_X0083_BM_AMVP_MERGE_MODE
                                   miAbove, true, isGt4x4, pu);
#else
                                   miAbove, true, isGt4x4);
#endif

    if (bFound)
    {
      return;
    }
  }

    while (cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), MAX_NUM_REF);
      mrgCtx.interDirNeighbours[cnt] = 1;
      if (mrgCandIdx == cnt)
      {
        return;
      }
      cnt++;
    }

  mrgCtx.numValidMergeCand = cnt;
}

#if MULTI_PASS_DMVR || JVET_W0097_GPM_MMVD_TM
uint32_t PU::getBDMVRMvdThreshold(const PredictionUnit &pu)
{
  uint32_t numPixels = pu.lwidth() * pu.lheight();
  if (numPixels < 64)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 2;
  }
  else if (numPixels < 256)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 0;
  }
}
#endif

#if TM_MRG
uint32_t PU::getTMMvdThreshold(const PredictionUnit &pu)
{
  uint32_t numPixels = pu.lwidth() * pu.lheight();
  if (numPixels < 64)
  {
    return 1 << MV_FRACTIONAL_BITS_INTERNAL;
  }
  else if (numPixels < 256)
  {
    return 2 << MV_FRACTIONAL_BITS_INTERNAL;
  }
  else
  {
    return 4 << MV_FRACTIONAL_BITS_INTERNAL;
  }
}

int PU::reorderInterMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, int numCand, uint32_t mvdSimilarityThresh)
{
  Slice& slice = *pu.cu->slice;

  if (numCand > 0 && slice.isInterB() && slice.getRefPOC(REF_PIC_LIST_0, 0) != slice.getRefPOC(REF_PIC_LIST_1, 0))
  {
    int numCandUni = 0;
    int numCandBi  = 0;

    // Promote bi candidates and collect uni candidates
    MergeCtx mrgCtxUni;

    for (int mergeCand = 0; mergeCand < numCand; ++mergeCand)
    {
      if (mrgCtx.interDirNeighbours[mergeCand] == 3)
      {
        mrgCtx.copyRegularMergeCand(numCandBi, mrgCtx, mergeCand);
        ++numCandBi;
      }
      else
      {
        mrgCtxUni.copyRegularMergeCand(numCandUni, mrgCtx, mergeCand);
        mrgCtxUni.convertRegularMergeCandToBi(numCandUni);
        ++numCandUni;
      }
    }

    // Insert uni-to-bi candidates
    for (int uniCandIdx = 0; uniCandIdx < numCandUni; ++uniCandIdx)
    {
      int mergeCand = numCandBi;
      mrgCtx.copyRegularMergeCand(mergeCand, mrgCtxUni, uniCandIdx);

      if (!mrgCtx.xCheckSimilarMotion(mergeCand, mvdSimilarityThresh))
      {
        ++numCandBi;
      }
    }

    return numCandBi;
  }
  else
  {
    return numCand;
  }
}
#endif

void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx,
                                 int mmvdList,
                                 const int& mrgCandIdx )
{
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
#if TM_MRG
  const uint32_t mvdSimilarityThresh = pu.tmMergeFlag ? PU::getTMMvdThreshold(pu) : 1;
  const uint32_t maxNumMergeCand     = pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : pu.cs->sps->getMaxNumMergeCand();
#else
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
#endif
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.LICFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  // compute the location of the current PU
#if JVET_X0083_BM_AMVP_MERGE_MODE
  const int curPoc = slice.getPOC();
  RefPicList amvpRefList = REF_PIC_LIST_X;
  RefPicList mergeRefList = REF_PIC_LIST_X;
  int amvpPoc = -1;
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    mergeRefList = pu.amvpMergeModeFlag[0] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    amvpRefList = RefPicList(1 - mergeRefList);
    amvpPoc = slice.getRefPOC(amvpRefList, pu.refIdx[amvpRefList]);
    mrgCtx.numValidMergeCand = (mrgCandIdx + 1) > maxNumMergeCand ? maxNumMergeCand : (mrgCandIdx + 1);
  }
#endif

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);

  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

#if JVET_X0083_BM_AMVP_MERGE_MODE
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miAbove.refIdx);
    if (isValidAmMode)
    {
#endif
    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
    mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
    // get Mv from Above
    mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->BcwIdx : BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.LICFlags[cnt] = miAbove.usesLIC;
#endif
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt] = puAbove->addHypData;
#endif
    }
#if NON_ADJACENT_MRG_CAND || TM_MRG
    if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                           , mvdSimilarityThresh
#endif
    ) )
    {
#endif
    if (mrgCandIdx == cnt)
    {
#if TM_MRG
      if( !pu.tmMergeFlag )
#endif
      return;
    }

    cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
    }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    } // if (isValidAmMode)
#endif
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);

  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

#if JVET_X0083_BM_AMVP_MERGE_MODE
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miLeft.refIdx);
    if (isValidAmMode && (!isAvailableB1 || (miAbove != miLeft)))
#else
    if (!isAvailableB1 || (miAbove != miLeft))
#endif
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->BcwIdx : BCW_DEFAULT;
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = miLeft.usesLIC;
#endif
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = puLeft->addHypData;
#endif
      }
#if NON_ADJACENT_MRG_CAND || TM_MRG
      if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                             , mvdSimilarityThresh
#endif
      ) )
      {
#endif
      if (mrgCandIdx == cnt)
      {
#if TM_MRG
        if (!pu.tmMergeFlag)
#endif
        return;
      }

      cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#endif
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

  bool isAvailableB0 = puAboveRight && isDiffMER( pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter( *puAboveRight->cu );

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

#if JVET_X0083_BM_AMVP_MERGE_MODE
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miAboveRight.refIdx);
    if (isValidAmMode && (!isAvailableB1 || ( miAbove != miAboveRight )))
#else
    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
#endif
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
      // get Mv from Above-right
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->BcwIdx : BCW_DEFAULT;
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = miAboveRight.usesLIC;
#endif
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = puAboveRight->addHypData;
#endif
      }

#if NON_ADJACENT_MRG_CAND || TM_MRG
      if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                             , mvdSimilarityThresh
#endif
      ) )
      {
#endif
      if (mrgCandIdx == cnt)
      {
#if TM_MRG
        if (!pu.tmMergeFlag)
#endif
        return;
      }

      cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#endif
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

  bool isAvailableA0 = puLeftBottom && isDiffMER( pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter( *puLeftBottom->cu );

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

#if JVET_X0083_BM_AMVP_MERGE_MODE
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miBelowLeft.refIdx);
    if (isValidAmMode && (!isAvailableA1 || ( miBelowLeft != miLeft )))
#else
    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
#endif
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
      mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->BcwIdx : BCW_DEFAULT;
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = miBelowLeft.usesLIC;
#endif
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = puLeftBottom->addHypData;
#endif
      }

#if NON_ADJACENT_MRG_CAND || TM_MRG
      if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                             , mvdSimilarityThresh
#endif
      ) )
      {
#endif
      if (mrgCandIdx == cnt)
      {
#if TM_MRG
        if (!pu.tmMergeFlag)
#endif
        return;
      }

      cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#endif
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

#if TM_MRG
  if (pu.tmMergeFlag)
  {
    cnt = PU::reorderInterMergeCandidates(pu, mrgCtx, cnt, mvdSimilarityThresh);

    if ((mrgCandIdx >= 0 && mrgCandIdx < cnt) || cnt >= maxNumMergeCand)
    {
      return;
    }
  }
#endif

  // above left
  if ( cnt < 4 )
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

    bool isAvailableB2 = puAboveLeft && isDiffMER( pu.lumaPos(), posLT.offset(-1, -1), plevel ) && CU::isInter( *puAboveLeft->cu );

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

#if JVET_X0083_BM_AMVP_MERGE_MODE
      bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miAboveLeft.refIdx);
      if (isValidAmMode && ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ))
#else
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
#endif
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
        mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->BcwIdx : BCW_DEFAULT;
#if INTER_LIC
        mrgCtx.LICFlags[cnt] = miAboveLeft.usesLIC;
#endif
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = puAboveLeft->addHypData;
#endif
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                               , mvdSimilarityThresh
#endif
        ) )
        {
#endif
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

#if INTER_RM_SIZE_CONSTRAINTS
  if (slice.getPicHeader()->getEnableTMVPFlag())
#else
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV    = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false ) )
                              || getColocatedMVP( pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false );
    if (bExistMV)
    {
      dir     |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = ( C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false ) )
                   || getColocatedMVP( pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false );
      if (bExistMV)
      {
        dir     |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

#if JVET_X0083_BM_AMVP_MERGE_MODE
    int8_t tempRefIdx[2] = { mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].refIdx, mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].refIdx };
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, tempRefIdx);
    if (isValidAmMode && ( dir != 0 ))
#else
    if( dir != 0 )
#endif
    {
      bool addTMvp = true;
      if( addTMvp )
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        mrgCtx.BcwIdx[uiArrayAddr] = BCW_DEFAULT;
#if INTER_LIC
        mrgCtx.LICFlags[uiArrayAddr] = false;
#endif
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[uiArrayAddr].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                               , mvdSimilarityThresh
#endif
        ) )
        {
#endif
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

#if NON_ADJACENT_MRG_CAND
  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  const int iNACANDIDATE_NUM[4] = { 3, 5, 5, 5 };
  const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };

  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCand - 1; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int NASPIdx = 0; NASPIdx < iNACANDIDATE_NUM[iDistanceIndex] && cnt < maxNumMergeCand - 1; NASPIdx++)
    {
      switch (idxMap[iDistanceIndex][NASPIdx])
      {
      case 0:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1; break;
      case 2:offsetX = pu.Y().width >> 1;   offsetY = -iNADistanceVer - 1;    break;
      case 3:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height >> 1; break;
      case 4:offsetX = -iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;    break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER( pu.lumaPos(), posLT.offset( offsetX, offsetY ) , plevel) && CU::isInter(*puNonAdjacent->cu);

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));


#if JVET_X0083_BM_AMVP_MERGE_MODE
        bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miNeighbor.refIdx);
        if (isValidAmMode)
        {
#endif
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        // get Mv from Above-right
        mrgCtx.BcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->BcwIdx : BCW_DEFAULT;
#if INTER_LIC
        mrgCtx.LICFlags[cnt] = miNeighbor.usesLIC;
#endif
        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = puNonAdjacent->addHypData;
#endif
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                               , mvdSimilarityThresh
#endif
        ))
#endif
        {
          if (mrgCandIdx == cnt)
          {
            return;
          }
          cnt++;
        }
#if JVET_X0083_BM_AMVP_MERGE_MODE
        } // if (isValidAmMode)
#endif
      }

    }
  }
#endif

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
  if (cnt != maxNumMergeCandMin1)
  {
    bool isGt4x4 = true;
    bool bFound = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt
      , isAvailableA1, miLeft, isAvailableB1, miAbove
      , CU::isIBC(*pu.cu)
      , isGt4x4
#if JVET_X0083_BM_AMVP_MERGE_MODE
      , pu
      , curPoc
      , amvpPoc
#endif
#if TM_MRG
      , mvdSimilarityThresh
#endif
      );

    if (bFound)
    {
      return;
    }
  }

  // pairwise-average candidates
  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), NOT_VALID );
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt].clear();
#endif
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;
#if INTER_LIC
      bool averageUsed = false;
#endif

      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);

          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
#if INTER_LIC
          mrgCtx.LICFlags[cnt] = false;
          averageUsed = true;
#endif
        }
        // only one MV is valid, take the only one MV
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
#if INTER_LIC
          if (!averageUsed)
          {
            mrgCtx.LICFlags[cnt] |= mrgCtx.LICFlags[0];
          }
#endif
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
#if INTER_LIC
          if (!averageUsed)
          {
            mrgCtx.LICFlags[cnt] |= mrgCtx.LICFlags[1];
          }
#endif
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
#if INTER_LIC
        if (interDir == 3)
        {
          mrgCtx.LICFlags[cnt] = false;
        }
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                               , mvdSimilarityThresh
#endif
        ) )
#endif
        cnt++;
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }

  uint32_t uiArrayAddr = cnt;

  int iNumRefIdx = slice.isInterB() ? std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1)) : slice.getNumRefIdx(REF_PIC_LIST_0);
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    int mergeRefIdx = 0;
    const int targetMaxNumMergeCand = mrgCtx.numValidMergeCand;
    mrgCtx.numValidMergeCand = uiArrayAddr;
    while ((mrgCtx.numValidMergeCand < targetMaxNumMergeCand) && (mergeRefIdx < iNumRefIdx))
    {
      const int mergePoc = slice.getRefPOC(mergeRefList, mergeRefIdx);
      if ((amvpPoc - curPoc) * (mergePoc - curPoc) > 0)
      {
        mergeRefIdx++;
        continue;
      }
      mrgCtx.interDirNeighbours [ mrgCtx.numValidMergeCand ] = 3;
      mrgCtx.BcwIdx             [ mrgCtx.numValidMergeCand ] = BCW_DEFAULT;
      mrgCtx.LICFlags           [ mrgCtx.numValidMergeCand ] = false;
      mrgCtx.useAltHpelIf       [ mrgCtx.numValidMergeCand ] = false;
      mrgCtx.addHypNeighbours   [ mrgCtx.numValidMergeCand ].clear();
      mrgCtx.mvFieldNeighbours  [ mrgCtx.numValidMergeCand << 1].setMvField(Mv(0, 0), mergeRefIdx);
      mrgCtx.mvFieldNeighbours  [(mrgCtx.numValidMergeCand << 1) + 1].setMvField(Mv(0, 0), mergeRefIdx);
      mrgCtx.numValidMergeCand++;
      mergeRefIdx++;
    }
    return;
  }
#endif

  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.BcwIdx             [uiArrayAddr     ] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.LICFlags           [uiArrayAddr     ] = false;
#endif
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[uiArrayAddr].clear();
#endif
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);
    mrgCtx.useAltHpelIf[uiArrayAddr] = false;

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }
  }
#if NON_ADJACENT_MRG_CAND
  CHECK(mrgCtx.numValidMergeCand != uiArrayAddr, "not enough number of merge candidates!");
#else
  mrgCtx.numValidMergeCand = uiArrayAddr;
#endif
}

#if JVET_X0049_ADAPT_DMVR
bool PU::isBMMergeFlagCoded(const PredictionUnit& pu)
{

  if (pu.cs->slice->getSPS()->getUseDMVDMode()
    && !pu.cs->slice->getCheckLDC()
    )
  {
    return (pu.cs->sps->getMaxNumBMMergeCand() > 0);
  }
  return false;
}

bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu, int refIdx0, int refIdx1)
{
  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    if (pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0)->longTerm
      || pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1)->longTerm)
    {
      return false;
    }
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, refIdx0);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, refIdx1);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}

bool PU::addBMMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
  const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
  const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
  const bool ibcFlag, const bool isGt4x4
#if TM_MRG
  , const uint32_t mvdSimilarityThresh
#endif
)
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
  int num_avai_candInLUT = (int)lut.size();

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];

    if (mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))))
    {
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = miNeighbor.usesLIC;

      if (ibcFlag)
      {
        CHECK(mrgCtx.LICFlags[cnt], "addMergeHMVPCand: LIC is not used with IBC mode")
      }
#endif

      bool isBmCand = false;
      int refIdx0 = miNeighbor.refIdx[0];
      int refIdx1 = miNeighbor.refIdx[1];
      if (refIdx0 >= 0 && refIdx1 >= 0 
        && miNeighbor.BcwIdx == BCW_DEFAULT
        && miNeighbor.addHypData.empty())
      {
        if (cs.slice->getRefPic(REF_PIC_LIST_0, refIdx0)->longTerm
          || cs.slice->getRefPic(REF_PIC_LIST_1, refIdx1)->longTerm)
        {
          isBmCand = false;
        }
        const int poc0 = cs.slice->getRefPOC(REF_PIC_LIST_0, refIdx0);
        const int poc1 = cs.slice->getRefPOC(REF_PIC_LIST_1, refIdx1);
        const int poc = cs.slice->getPOC();
        if ((poc - poc0)*(poc - poc1) < 0)
        {
          if (abs(poc - poc0) == abs(poc - poc1))
          {
            isBmCand = true;
          }
        }
      }
      if (!isBmCand)
      {
        continue;
      }
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
      mrgCtx.useAltHpelIf[cnt] = !ibcFlag && miNeighbor.useAltHpelIf;

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
      }
#if NON_ADJACENT_MRG_CAND
      if (mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
        , mvdSimilarityThresh
#endif
      ))
      {
        continue;
      }
#endif
      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt++;

      if (cnt == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
  return false;
}

void PU::getInterBMCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx,
  const int& mrgCandIdx)
{
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumBMMergeCand();
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.BcwIdx[ui] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.LICFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  mrgCtx.numCandToTestEnc = maxNumMergeCand;
  // compute the location of the current PU

  int mvThreshod = 1;

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);

  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (isBiPredFromDifferentDirEqDistPoc(pu, miAbove.refIdx[0], miAbove.refIdx[1])
      )
    {
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
      {
#endif
        if (mrgCandIdx == cnt)
        {
          mrgCtx.numValidMergeCand = cnt + 1;
          return;
        }
        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#endif
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);

  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    if (!isAvailableB1 || (miAbove != miLeft))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miLeft.refIdx[0], miLeft.refIdx[1])
        )
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
        // get Mv from Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);

  bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);

  if (isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));

    if (!isAvailableB1 || (miAbove != miAboveRight))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miAboveRight.refIdx[0], miAboveRight.refIdx[1])
        )
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveRight.mv[1], miAboveRight.refIdx[1]);

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }
          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);

  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);

  if (isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));

    if (!isAvailableA1 || (miBelowLeft != miLeft))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miBelowLeft.refIdx[0], miBelowLeft.refIdx[1])
        )
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
        // get Mv from Bottom-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miBelowLeft.mv[1], miBelowLeft.refIdx[1]);

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
#if TM_MRG
            if (!pu.tmMergeFlag)
#endif
              return;
          }
          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }


  const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);

  bool isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);

  if (isAvailableB2)
  {
    miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));

    if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miAboveLeft.refIdx[0], miAboveLeft.refIdx[1])
        )
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveLeft.mv[1], miAboveLeft.refIdx[1]);

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

#if INTER_RM_SIZE_CONSTRAINTS
  if (slice.getPicHeader()->getEnableTMVPFlag())
#else
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset(-3, -3);
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool C0Avail = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
        (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }

    Mv        cColMv;
    int       iRefIdx = 0;
    int       dir = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV = (C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false))
      || getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false);
    if (bExistMV)
    {
      dir |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = (C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false))
        || getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false);
      if (bExistMV)
      {
        dir |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if (dir != 0)
    {
      bool addTMvp = isBiPredFromDifferentDirEqDistPoc(pu, mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 0].refIdx, mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].refIdx);
      if (addTMvp)
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[uiArrayAddr].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#endif
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
#if NON_ADJACENT_MRG_CAND
  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  const int iNACANDIDATE_NUM[4] = { 3, 5, 5, 5 };
  const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };

  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCandMin1; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int NASPIdx = 0; NASPIdx < iNACANDIDATE_NUM[iDistanceIndex] && cnt < maxNumMergeCandMin1; NASPIdx++)
    {
      switch (idxMap[iDistanceIndex][NASPIdx])
      {
      case 0:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1; break;
      case 2:offsetX = pu.Y().width >> 1;   offsetY = -iNADistanceVer - 1;    break;
      case 3:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height >> 1; break;
      case 4:offsetX = -iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;    break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));


        if (isBiPredFromDifferentDirEqDistPoc(pu, miNeighbor.refIdx[0], miNeighbor.refIdx[1])
          )
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
          // get Mv from Above-Left
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
          mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);

#if NON_ADJACENT_MRG_CAND || TM_MRG
          if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
          {
            if (mrgCandIdx == cnt)
            {
              mrgCtx.numValidMergeCand = cnt + 1;
              return;
            }
            cnt++;
          }
        }
      }
    }
  }
#endif

  if (cnt != maxNumMergeCandMin1)
  {
    bool isGt4x4 = true;
    bool bFound = addBMMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt
      , isAvailableA1, miLeft, isAvailableB1, miAbove
      , CU::isIBC(*pu.cu)
      , isGt4x4
#if TM_MRG
      , mvThreshod
#endif
    );

    if (bFound)
    {
      return;
    }
  }

  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), NOT_VALID);
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField(Mv(0, 0), NOT_VALID);
#if INTER_LIC
      mrgCtx.LICFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt].clear();
#endif
      mrgCtx.BcwIdx[cnt] = BCW_DEFAULT;
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;
      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for (int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++)
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if (refIdxI != refIdxJ)
        {
          continue;
        }

        interDir += 1 << refListId;
        const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
        const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

        // average two MVs
        Mv avgMv = MvI;
        avgMv += MvJ;
        roundAffineMv(avgMv.hor, avgMv.ver, 1);

        mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField(avgMv, refIdxI);
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if (interDir == 3 && isBiPredFromDifferentDirEqDistPoc(pu, mrgCtx.mvFieldNeighbours[cnt * 2 + 0].refIdx, mrgCtx.mvFieldNeighbours[cnt * 2 + 1].refIdx))
      {
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
          cnt++;
      }
    }

    // early termination
    if (cnt == maxNumMergeCand)
    {
      mrgCtx.numValidMergeCand = cnt;
      return;
    }
  }

  mrgCtx.numCandToTestEnc = cnt;

  if (cnt < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours[cnt] = 3;
    mrgCtx.BcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.LICFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[cnt].clear();
#endif
    mrgCtx.useAltHpelIf[cnt] = false;
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(0, 0), 0);
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(0, 0), 0);
#if NON_ADJACENT_MRG_CAND || TM_MRG
    if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
    {
      if (mrgCandIdx == cnt)
      {
        mrgCtx.numValidMergeCand = cnt + 1;
        return;
      }
      cnt++;
    }
  }
  mrgCtx.numValidMergeCand = cnt;
}
#endif

bool PU::checkDMVRCondition(const PredictionUnit& pu)
{
#if !MULTI_PASS_DMVR
  if( pu.cs->sps->getUseDMVR() && !pu.cs->picHeader->getDisDmvrFlag() )
  {
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling( REF_PIC_LIST_0, refIdx0 );
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling( REF_PIC_LIST_1, refIdx1 );

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps );
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps );

    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
      && !pu.cu->mmvdSkip && PU::isBiPredFromDifferentDirEqDistPoc( pu )
#if TM_MRG
      && !pu.tmMergeFlag
#endif
      && ( pu.lheight() >= 8 ) && ( pu.lwidth() >= 8 )
      && ( ( pu.lheight() * pu.lwidth() ) >= 128 )
      && ( pu.cu->BcwIdx == BCW_DEFAULT )
      && !WPScalingParam::isWeighted( wp0 ) && !WPScalingParam::isWeighted( wp1 ) && !ref0IsScaled && !ref1IsScaled;
  }
  else
#endif
  {
    return false;
  }
}

#if MULTI_PASS_DMVR
bool PU::checkBDMVRCondition(const PredictionUnit& pu)
{
  if( pu.cs->sps->getUseDMVDMode() )
  {
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling( REF_PIC_LIST_0, refIdx0 );
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling( REF_PIC_LIST_1, refIdx1 );

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps );
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps );

#if JVET_X0083_BM_AMVP_MERGE_MODE
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
#else
    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
#endif
      && !pu.cu->mmvdSkip
      && PU::isBiPredFromDifferentDirEqDistPoc( pu )
      && ( pu.cu->BcwIdx == BCW_DEFAULT )
      && !WPScalingParam::isWeighted( wp0 ) && !WPScalingParam::isWeighted( wp1 ) && !ref0IsScaled && !ref1IsScaled;
  }
  else
  {
    return false;
  }
}
#endif

#if INTER_LIC && RPR_ENABLE
bool PU::checkRprLicCondition( const PredictionUnit & pu )
{
  if ( pu.cs->sps->getLicEnabledFlag() )
  {
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
    {
      return false;
    }

    bool  bLicEnabled = true;
    int iRefIdx             = (pu.refIdx[0] >= 0) ? pu.refIdx[0] : pu.refIdx[1];
    RefPicList  eRefPicList = (pu.refIdx[0] >= 0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();
    if( isResamplingPossible && pu.cu->slice->getRefPic( eRefPicList, iRefIdx )->isRefScaled( pu.cs->pps ) )
    {
      bLicEnabled = false;
    }

    return bLicEnabled;
  }
  else
  {
    return false;
  }
}
#endif

static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  int sign  = val >> 31;
  int scale = floorLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign << (MV_MANTISSA_BITCOUNT - 1));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa << MV_EXPONENT_BITCOUNT);
}

int convertMvFloatToFixed(int val)
{
  int exponent = val & MV_EXPONENT_MASK;
  int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) << (exponent - 1);
}

int roundMvComp(int x)
{
  return convertMvFloatToFixed(convertMvFixedToFloat(x));
}

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;

  for( k = 0; k < maxNumMergeCand; k++ )
  {
    refIdxList0 = mrgCtx.mvFieldNeighbours[( k << 1 )].refIdx;
    refIdxList1 = mrgCtx.mvFieldNeighbours[( k << 1 ) + 1].refIdx;

    if( ( refIdxList0 >= 0 ) && ( refIdxList1 >= 0 ) )
    {
      mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[( k << 1 )];
      mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[( k << 1 ) + 1];
    }
    else if( refIdxList0 >= 0 )
    {
      mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[( k << 1 )];
      mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField( Mv( 0, 0 ), -1 );
    }
    else if( refIdxList1 >= 0 )
    {
      mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField( Mv( 0, 0 ), -1 );
      mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[( k << 1 ) + 1];
    }
    mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];
    currBaseNum++;

    if( currBaseNum == MMVD_BASE_MV_NUM )
    {
      break;
    }
  }
}
bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx, bool sbFlag)
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

  if( !pColPic )
  {
    return false;
  }

  // Check the position of colocated block is within a subpicture
  const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
  if (curSubPic.getTreatedAsPicFlag())
  {
    if (!curSubPic.isContainingPos(pos))
    {
      return false;
    }
  }
  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }
  if (mi.isIBCmot)
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (sbFlag && !slice.getCheckLDC())
  {
    eColRefPicList = eRefPicList;
    iColRefIdx = mi.refIdx[eColRefPicList];
    if (iColRefIdx < 0)
    {
      return false;
    }
  }
  else
  {
    if (iColRefIdx < 0)
    {
      eColRefPicList = RefPicList(1 - eColRefPicList);
      iColRefIdx = mi.refIdx[eColRefPicList];

      if (iColRefIdx < 0)
      {
        return false;
      }
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;

  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
    rcMv.clipToStorageBitDepth();
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
      rcMv.clipToStorageBitDepth();
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
}

bool PU::isDiffMER(const Position &pos1, const Position &pos2, const unsigned plevel)
{
  const unsigned xN = pos1.x;
  const unsigned yN = pos1.y;
  const unsigned xP = pos2.x;
  const unsigned yP = pos2.y;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

bool PU::isAddNeighborMv(const Mv& currMv, Mv* neighborMvs, int numNeighborMv)
{
  bool existed = false;
  for (uint32_t cand = 0; cand < numNeighborMv && !existed; cand++)
  {
    if (currMv == neighborMvs[cand])
    {
      existed = true;
    }
  }

  if (!existed)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PU::getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred)
{
  const PreCalcValues   &pcv = *pu.cs->pcv;
  const int  cuWidth = pu.blocks[COMPONENT_Y].width;
  const int  cuHeight = pu.blocks[COMPONENT_Y].height;
  const int  log2UnitWidth = floorLog2(pcv.minCUWidth);
  const int  log2UnitHeight = floorLog2(pcv.minCUHeight);
  const int  totalAboveUnits = (cuWidth >> log2UnitWidth) + 1;
  const int  totalLeftUnits = (cuHeight >> log2UnitHeight) + 1;

  nbPred = 0;
  Position posLT = pu.Y().topLeft();

  // above-left
  const PredictionUnit *aboveLeftPU = pu.cs->getPURestricted(posLT.offset(-1, -1), pu, CHANNEL_TYPE_LUMA);
  if (aboveLeftPU && CU::isIBC(*aboveLeftPU->cu))
  {
    if (isAddNeighborMv(aboveLeftPU->bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = aboveLeftPU->bv;
    }
  }

  // above neighbors
  for (uint32_t dx = 0; dx < totalAboveUnits && nbPred < IBC_NUM_CANDIDATES; dx++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset((dx << log2UnitWidth), -1), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  // left neighbors
  for (uint32_t dy = 0; dy < totalLeftUnits && nbPred < IBC_NUM_CANDIDATES; dy++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset(-1, (dy << log2UnitHeight)), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
    }
  }

  size_t numAvaiCandInLUT = pu.cs->motionLut.lutIbc.size();
  for (uint32_t cand = 0; cand < numAvaiCandInLUT && nbPred < IBC_NUM_CANDIDATES; cand++)
  {
    MotionInfo neibMi = pu.cs->motionLut.lutIbc[cand];
    if (isAddNeighborMv(neibMi.bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = neibMi.bv;
    }
  }

  bool isBvCandDerived[IBC_NUM_CANDIDATES];
  ::memset(isBvCandDerived, false, IBC_NUM_CANDIDATES);

  int curNbPred = nbPred;
  if (curNbPred < IBC_NUM_CANDIDATES)
  {
    do
    {
      curNbPred = nbPred;
      for (uint32_t idx = 0; idx < curNbPred && nbPred < IBC_NUM_CANDIDATES; idx++)
      {
        if (!isBvCandDerived[idx])
        {
          Mv derivedBv;
          if (getDerivedBV(pu, mvPred[idx], derivedBv))
          {
            if (isAddNeighborMv(derivedBv, mvPred, nbPred))
            {
              mvPred[nbPred++] = derivedBv;
            }
          }
          isBvCandDerived[idx] = true;
        }
      }
    } while (nbPred > curNbPred && nbPred < IBC_NUM_CANDIDATES);
  }
}

bool PU::getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv)
{
  int   cuPelX = pu.lumaPos().x;
  int   cuPelY = pu.lumaPos().y;
  int rX = cuPelX + currentMv.getHor();
  int rY = cuPelY + currentMv.getVer();
  int offsetX = currentMv.getHor();
  int offsetY = currentMv.getVer();

  if( rX < 0 || rY < 0 || rX >= pu.cs->slice->getPPS()->getPicWidthInLumaSamples() || rY >= pu.cs->slice->getPPS()->getPicHeightInLumaSamples() )
  {
    return false;
  }

  const PredictionUnit *neibRefPU = NULL;
  neibRefPU = pu.cs->getPURestricted(pu.lumaPos().offset(offsetX, offsetY), pu, CHANNEL_TYPE_LUMA);

  bool isIBC = (neibRefPU) ? CU::isIBC(*neibRefPU->cu) : 0;
  if (isIBC)
  {
    derivedMv = neibRefPU->bv;
    derivedMv += currentMv;
  }
  return isIBC;
}

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector predictor candidates")
 */
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo)
{
  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;

  MergeCtx mergeCtx;
  PU::getIBCMergeCandidates(pu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;;
    pInfo->numCand++;
    candIdx++;
  }

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundIbcPrecInternal2Amvr(pu.cu->imv);
  }
}


/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo
#if TM_AMVP
                   , InterPrediction* interPred
#endif
)
{
  CodingStructure &cs = *pu.cs;
#if TM_AMVP
  interPred = pu.cu->cs->sps->getUseDMVDMode() ? interPred : nullptr;
  if (cs.pcv->isEncoder && interPred != nullptr && interPred->readTplAmvpBuffer(amvpInfo, *pu.cu, eRefPicList, refIdx))
  {
    return;
  }
#endif
  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;
#if TM_AMVP
  pInfo->maxStorageSize         = interPred != nullptr ? REGULAR_AMVP_MAX_NUM_CANDS : AMVP_MAX_NUM_CANDS;
  pInfo->maxSimilarityThreshold = interPred != nullptr ? 1 : 0;
#endif

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

    }
  }

  // Above predictor search
  {
    bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

    if( !bAdded )
    {
      bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

      if( !bAdded )
      {
        addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
      }
    }
  }

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

#if TM_AMVP
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < pInfo->maxStorageSize && (pu.lumaSize().width + pu.lumaSize().height > 12))
#else
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
    // Get Temporal Motion Predictor
    const int refIdx_Col = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool C0Avail = false;
    Position posC1 = pu.Y().center();
    Mv cColMv;

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        C0Avail = true;
      }
    }
    if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdx_Col, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdx_Col, false ) )
    {
      cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
#if TM_AMVP
      pInfo->mvCand[pInfo->numCand] = cColMv;
      if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
      {
        pInfo->numCand++;
      }
#else
      pInfo->mvCand[pInfo->numCand++] = cColMv;
#endif
    }
  }

#if TM_AMVP
  // Non-adjacent candidates
  if (pInfo->numCand < pInfo->maxStorageSize && interPred != nullptr)
  {
    const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
    MotionInfo miNeighbor;
    int offsetX = 0;
    int offsetY = 0;
    const int iNACANDIDATE_NUM[4] = { 3, 5, 5, 5 };
    const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };

    for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && pInfo->numCand < pInfo->maxStorageSize; iDistanceIndex++)
    {
      const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
      const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

      for (int NASPIdx = 0; NASPIdx < iNACANDIDATE_NUM[iDistanceIndex] && pInfo->numCand < pInfo->maxStorageSize; NASPIdx++)
      {
        switch (idxMap[iDistanceIndex][NASPIdx])
        {
        case 0: offsetX = -iNADistanceHor - 1;               offsetY = pu.Y().height + iNADistanceVer - 1; break;
        case 1: offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;                break;
        case 2: offsetX = pu.Y().width >> 1;                 offsetY = -iNADistanceVer - 1;                break;
        case 3: offsetX = -iNADistanceHor - 1;               offsetY = pu.Y().height >> 1;                 break;
        case 4: offsetX = -iNADistanceHor - 1;               offsetY = -iNADistanceVer - 1;                break;
        default: CHECK(true, "Unknown index for non-adjacent AMVP candidate"); break;
        }

        const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

        bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);

        if (isAvailableNonAdjacent)
        {
          miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

          const int        currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
          const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

          for (int predictorSource = 0; predictorSource < 2; predictorSource++) // examine the indicated reference picture list, then if not available, examine the other list.
          {
            const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
            const int        neibRefIdx = miNeighbor.refIdx[eRefPicListIndex];

            if (neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC(eRefPicListIndex, neibRefIdx))
            {
              miNeighbor.mv[eRefPicListIndex].roundTransPrecInternal2Amvr(pu.cu->imv);
              pInfo->mvCand[(pInfo->numCand)] = miNeighbor.mv[eRefPicListIndex];
              if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
              {
                pInfo->numCand++;
              }
            }
            if (pInfo->numCand > pInfo->maxStorageSize)
            {
              pInfo->numCand = pInfo->maxStorageSize;
            }
          }
        }
      }
    }
  }

  for (int i = 0; i < pInfo->numCand; i++)
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }
#endif

#if TM_AMVP
  if (pInfo->numCand < pInfo->maxStorageSize)
#else
  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
#endif
  {
    const int currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
    addAMVPHMVPCand(pu, eRefPicList, currRefPOC, *pInfo);
  }
#if TM_AMVP
  if (pInfo->numCand > pInfo->maxStorageSize)
  {
    pInfo->numCand = pInfo->maxStorageSize;
  }
#else
  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }
#endif

#if TM_AMVP
  if (pu.cu->cs->sps->getUseDMVDMode() && interPred != nullptr && pInfo->numCand > 0)
  {
    struct AMVPSort
    {
      Mv AMVPCand;
      Distortion cost;
    };
    AMVPSort temp;
    std::vector<AMVPSort> input;
    const auto CostIncSort = [](const AMVPSort &x, const AMVPSort &y) { return x.cost < y.cost; };
    Distortion tmCost[REGULAR_AMVP_MAX_NUM_CANDS];
    for (int i = 0; i < REGULAR_AMVP_MAX_NUM_CANDS; i++)
    {
      tmCost[i] = std::numeric_limits<Distortion>::max();
    }

    for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
    {
      tmCost[candIdx] = interPred->deriveTMMv(pu, true, std::numeric_limits<Distortion>::max(), eRefPicList, refIdx, 0, pInfo->mvCand[candIdx]);
      temp.AMVPCand = pInfo->mvCand[candIdx];
      temp.cost = tmCost[candIdx];
      input.push_back(temp);
    }

    stable_sort(input.begin(), input.end(), CostIncSort);
    for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
    {
      pInfo->mvCand[candIdx] = input.at(candIdx).AMVPCand;
      tmCost[candIdx] = input.at(candIdx).cost;
    }

    pInfo->numCand = 1;
    interPred->deriveTMMv(pu, true, tmCost[0], eRefPicList, refIdx, TM_MAX_NUM_OF_ITERATIONS, pInfo->mvCand[0]);
  }

  while (pInfo->numCand < pInfo->maxStorageSize)
#else
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
#endif
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }
#if TM_AMVP
  pInfo->numCand = (pu.cu->cs->sps->getUseDMVDMode() && interPred != nullptr ? 1 : pInfo->numCand);
#endif

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(pu.cu->imv);
  }

#if TM_AMVP
  if (cs.pcv->isEncoder && interPred != nullptr)
  {
    interPred->writeTplAmvpBuffer(*pInfo, *pu.cu, eRefPicList, refIdx);
  }
#endif
}

bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo )
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch ( dir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if (neibPU == NULL || !CU::isInter(*neibPU->cu) || !neibPU->cu->affine || neibPU->mergeType != MRG_TYPE_DEFAULT_N)
  {
    return false;
  }

  Mv outputAffineMv[3];
  const MotionInfo& neibMi = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC = cs.slice->getRefPic( refPicList, refIdx )->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

    if ( ((neibPU->interDir & (eRefPicListIndex + 1)) == 0) || pu.cu->slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;
    }

    xInheritedAffineMv( pu, neibPU, eRefPicListIndex, outputAffineMv );
    outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
    affiAMVPInfo.numCand++;
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  bool isTopCtuBoundary = false;
  if ( (posNeiY + neiH) % pu.cs->sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }
#endif

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(neiW));
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(neiW));
#if AFFINE_RM_CONSTRAINTS_AND_OPT
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM )
#else
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
#endif
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - floorLog2(neiH));
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - floorLog2(neiH));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}


void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

  // insert inherited affine candidates
  Mv outputAffineMv[3];
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  // check left neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo ) )
  {
    addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo );
  }

  // check above neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo ) )
  {
    if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo ) )
    {
      addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo );
    }
  }

  if ( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    for (int i = 0; i < affiAMVPInfo.numCand; i++)
    {
      affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0 );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0 );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0 );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1 );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2 );
  if ( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);

  if ( cornerMVPattern == 7 || (cornerMVPattern == 3 && pu.cu->affineType == AFFINEMODEL_4PARAM) )
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    affiAMVPInfo.numCand++;
  }

  if ( affiAMVPInfo.numCand < 2 )
  {
    // check corner MVs
    for ( int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i-- )
    {
      if ( cornerMVPattern & (1 << i) ) // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.numCand++;
      }
    }

    // Get Temporal Motion Predictor
    if ( affiAMVPInfo.numCand < 2 && pu.cs->picHeader->getEnableTMVPFlag() )
    {
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset( -3, -3 );

      const PreCalcValues& pcv = *pu.cs->pcv;

      Position posC0;
      bool C0Avail = false;
      Position posC1 = pu.Y().center();
      Mv cColMv;
      bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
      const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      if (curSubPic.getTreatedAsPicFlag())
      {
        boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
          (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
      }
      if (boundaryCond)
      {
        int posYInCtu = posRB.y & pcv.maxCUHeightMask;
        if (posYInCtu + 4 < pcv.maxCUHeight)
        {
          posC0 = posRB.offset(4, 4);
          C0Avail = true;
        }
      }
      if ( ( C0Avail && getColocatedMVP( pu, eRefPicList, posC0, cColMv, refIdxCol, false ) ) || getColocatedMVP( pu, eRefPicList, posC1, cColMv, refIdxCol, false ) )
      {
        cColMv.roundAffinePrecInternal2Amvr(pu.cu->imv);
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
        affiAMVPInfo.numCand++;
      }
    }

    if ( affiAMVPInfo.numCand < 2 )
    {
      // add zero MV
      for ( int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++ )
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
  }

  for (int i = 0; i < affiAMVPInfo.numCand; i++)
  {
    affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
#if TM_AMVP
      info.mvCand[info.numCand] = neibMi.mv[eRefPicListIndex];
      if (!info.xCheckSimilarMotion(info.numCand))
      {
        info.numCand++;
        return true;
      }
#else
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
#endif
    }
  }

  return false;
}


void PU::addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const int currRefPOC, AMVPInfo &info)
{
  const Slice &slice = *(*pu.cs).slice;

  MotionInfo neibMi;
  auto &lut = CU::isIBC(*pu.cu) ? pu.cs->motionLut.lutIbc : pu.cs->motionLut.lut;
  int num_avai_candInLUT = (int) lut.size();
  int num_allowedCand = std::min(MAX_NUM_HMVP_AVMPCANDS, num_avai_candInLUT);
  const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for (int mrgIdx = 1; mrgIdx <= num_allowedCand; mrgIdx++)
  {
#if TM_AMVP
    if (info.numCand >= info.maxStorageSize)
#else
    if (info.numCand >= AMVP_MAX_NUM_CANDS)
#endif
    {
      return;
    }
    neibMi = lut[mrgIdx - 1];

    for (int predictorSource = 0; predictorSource < 2; predictorSource++)
    {
      const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

      if (neibRefIdx >= 0 && (CU::isIBC(*pu.cu) || (currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundTransPrecInternal2Amvr(pu.cu->imv);

#if TM_AMVP
        info.mvCand[info.numCand] = pmv;
        if (!info.xCheckSimilarMotion(info.numCand))
        {
          info.numCand++;
        }

        if( info.numCand >= info.maxStorageSize )
        {
          return;
        }
#else
        info.mvCand[info.numCand++] = pmv;
        if (info.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return;
        }
#endif
      }
    }
  }
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
#if !INTER_RM_SIZE_CONSTRAINTS
  if(pu.cu->lumaSize().width == 4 && pu.cu->lumaSize().height ==4 )
  {
    return true;
  }
  /* disable bi-prediction for 4x8/8x4 */
  if ( pu.cu->lumaSize().width + pu.cu->lumaSize().height == 12 )
  {
    return true;
  }
#endif
  return false;
}

void PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgType)
{
  int cuW = pu.Y().width;
  int cuH = pu.Y().height;
  int vx, vy;
  int shift = MAX_CU_DEPTH;
  int shiftHtoW = shift + floorLog2(cuW) - floorLog2(cuH);

  // motion info
  Mv cMv[2][4];
  int refIdx[2] = { -1, -1 };
  int dir = 0;
  EAffineModel curType = (verNum == 2) ? AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;
#if INTER_LIC
  bool LICFlag = false;
#endif
  if ( verNum == 2 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if ( !isAvailable[idx0] || !isAvailable[idx1] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }
#if INTER_LIC
    LICFlag = mi[idx0].usesLIC || mi[idx1].usesLIC;
#endif
  }
  else if ( verNum == 3 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if ( !isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2] )
    {
      return;
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 && mi[idx2].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l] && mi[idx0].refIdx[l] == mi[idx2].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }

#if INTER_LIC
    LICFlag = mi[idx0].usesLIC || mi[idx1].usesLIC || mi[idx2].usesLIC;
#endif
  }

  if ( dir == 0 )
  {
    return;
  }

  for ( int l = 0; l < 2; l++ )
  {
    if ( dir & (l + 1) )
    {
      for ( int i = 0; i < verNum; i++ )
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch ( modelIdx )
      {
      case 0: // 0 : LT, RT, LB
        break;

      case 1: // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
        cMv[l][2].clipToStorageBitDepth();
        break;

      case 2: // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
        cMv[l][1].clipToStorageBitDepth();
        break;

      case 3: // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
        cMv[l][0].clipToStorageBitDepth();
        break;

      case 4: // 4 : LT, RT
        break;

      case 5: // 5 : LT, LB
        vx = (cMv[l][0].hor << shift) + ((cMv[l][2].ver - cMv[l][0].ver) << shiftHtoW);
        vy = (cMv[l][0].ver << shift) - ((cMv[l][2].hor - cMv[l][0].hor) << shiftHtoW);
        roundAffineMv( vx, vy, shift );
        cMv[l][1].set( vx, vy );
        cMv[l][1].clipToStorageBitDepth();
        break;

      default:
        CHECK( 1, "Invalid model index!\n" );
        break;
      }
    }
    else
    {
      for ( int i = 0; i < 4; i++ )
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for ( int i = 0; i < 3; i++ )
  {
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mv = cMv[0][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mv = cMv[1][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
  }
  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType[affMrgType.numValidMergeCand] = curType;
  affMrgType.BcwIdx[affMrgType.numValidMergeCand] = (dir == 3) ? bcwIdx : BCW_DEFAULT;
#if INTER_LIC
  affMrgType.LICFlags[affMrgType.numValidMergeCand] = LICFlag;
#endif
  affMrgType.numValidMergeCand++;

  return;
}

const int getAvailableAffineNeighboursForLeftPredictor( const PredictionUnit &pu, const PredictionUnit* npu[] )
{
  const Position posLB = pu.Y().bottomLeft();
  int num = 0;
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if (puLeftBottom && puLeftBottom->cu->affine && puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel))
  {
    npu[num++] = puLeftBottom;
    return num;
  }

  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if (puLeft && puLeft->cu->affine && puLeft->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel))
  {
    npu[num++] = puLeft;
    return num;
  }

  return num;
}

const int getAvailableAffineNeighboursForAbovePredictor( const PredictionUnit &pu, const PredictionUnit* npu[], int numAffNeighLeft )
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  int num = numAffNeighLeft;

  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if (puAboveRight && puAboveRight->cu->affine && puAboveRight->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel))
  {
    npu[num++] = puAboveRight;
    return num;
  }

  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if (puAbove && puAbove->cu->affine && puAbove->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel))
  {
    npu[num++] = puAbove;
    return num;
  }

  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if (puAboveLeft && puAboveLeft->cu->affine && puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel))
  {
    npu[num++] = puAboveLeft;
    return num;
  }

  return num;
}

void PU::getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx,
#if !AFFINE_MMVD
                             const int mrgCandIdx
#else
                                   int mrgCandIdx, bool isAfMmvd
#endif
)
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

  for ( int i = 0; i < maxNumAffineMergeCand; i++ )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField( Mv(), -1 );
      affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField( Mv(), -1 );
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.BcwIdx[i] = BCW_DEFAULT;
#if INTER_LIC
    affMrgCtx.LICFlags[i] = false;
#endif
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand = maxNumAffineMergeCand;

  bool sbTmvpEnableFlag = slice.getSPS()->getSbTMVPEnabledFlag()
                          && !(slice.getPOC() == slice.getRefPic(REF_PIC_LIST_0, 0)->getPOC() && slice.isIRAP());
  bool isAvailableSubPu = false;
  if (sbTmvpEnableFlag && slice.getPicHeader()->getEnableTMVPFlag())
  {
    MergeCtx mrgCtx = *affMrgCtx.mrgCtx;
    bool tmpLICFlag = false;

    CHECK( mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized" );
    mrgCtx.subPuMvpMiBuf.fill( MotionInfo() );

    int pos = 0;
    // Get spatial MV
    const Position posCurLB = pu.Y().bottomLeft();
    MotionInfo miLeft;

    //left
    const PredictionUnit* puLeft = cs.getPURestricted( posCurLB.offset( -1, 0 ), pu, pu.chType );
    const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posCurLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter( *puLeft->cu );
    if ( isAvailableA1 )
    {
#if INTER_LIC
      affMrgCtx.mrgCtx->LICFlags[pos] = false;
#endif

      miLeft = puLeft->getMotionInfo( posCurLB.offset( -1, 0 ) );
      // get Inter Dir
      mrgCtx.interDirNeighbours[pos] = miLeft.interDir;

      // get Mv from Left
      mrgCtx.mvFieldNeighbours[pos << 1].setMvField( miLeft.mv[0], miLeft.refIdx[0] );

      if ( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[(pos << 1) + 1].setMvField( miLeft.mv[1], miLeft.refIdx[1] );
      }
      pos++;
    }

    mrgCtx.numValidMergeCand = pos;

    isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, pos, 0);
    if ( isAvailableSubPu )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 0].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 0].refIdx );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( mrgCtx.mvFieldNeighbours[(pos << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 1].refIdx );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = mrgCtx.interDirNeighbours[pos];

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINE_MODEL_NUM;
      affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;
#if AFFINE_MMVD
      mrgCandIdx += (isAfMmvd && mrgCandIdx >= 0 ? 1 : 0);
#endif
      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
  }

  if ( slice.getSPS()->getUseAffine() )
  {
    ///> Start: inherited affine candidates
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor( pu, npu );
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor( pu, npu, numAffNeighLeft );
    for ( int idx = 0; idx < numAffNeigh; idx++ )
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];
      pu.cu->affineType = puNeigh->cu->affineType;
      if ( puNeigh->interDir != 2 )
      {
        xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_0, cMv[0] );
      }
      if ( slice.isInterB() )
      {
        if ( puNeigh->interDir != 1 )
        {
          xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_1, cMv[1] );
        }
      }

      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( cMv[0][mvNum], puNeigh->refIdx[0] );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( cMv[1][mvNum], puNeigh->refIdx[1] );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = (EAffineModel)(puNeigh->cu->affineType);
      affMrgCtx.BcwIdx[affMrgCtx.numValidMergeCand] = puNeigh->cu->BcwIdx;
#if INTER_LIC
      CHECK( puNeigh->interDir == 3 && puNeigh->cu->LICFlag, "LIC should not be enabled for affine bi-pred" );
      affMrgCtx.LICFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->LICFlag;
#endif
      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };

      int8_t neighBcw[2] = { BCW_DEFAULT, BCW_DEFAULT };
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
      for ( int i = 0; i < 3; i++ )
      {
        const Position pos = posLT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
        {
          isAvailable[0] = true;
          mi[0] = puNeigh->getMotionInfo( pos );
          neighBcw[0] = puNeigh->cu->BcwIdx;
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset( 0, -1 ), pu.Y().topRight().offset( 1, -1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posRT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
        {
          isAvailable[1] = true;
          mi[1] = puNeigh->getMotionInfo( pos );
          neighBcw[1] = puNeigh->cu->BcwIdx;
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset( -1, 0 ), pu.Y().bottomLeft().offset( -1, 1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posLB[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
        {
          isAvailable[2] = true;
          mi[2] = puNeigh->getMotionInfo( pos );
          break;
        }
      }

      // control point: RB
      if ( slice.getPicHeader()->getEnableTMVPFlag() )
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset( -3, -3 );

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool C0Avail = false;

        bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
            (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(4, 4);
            C0Avail = true;
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_0, posC0, cColMv, refIdx, false );
        if ( bExistMV )
        {
          mi[3].mv[0] = cColMv;
          mi[3].refIdx[0] = refIdx;
          mi[3].interDir = 1;
          isAvailable[3] = true;
        }

        if ( slice.isInterB() )
        {
          bExistMV = C0Avail && getColocatedMVP( pu, REF_PIC_LIST_1, posC0, cColMv, refIdx, false );
          if ( bExistMV )
          {
            mi[3].mv[1] = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3] = true;
          }
        }
#if INTER_LIC
        mi[3].usesLIC = false;
#endif
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = pu.cs->sps->getUseAffineType() ? 0 : 4;
      for ( int idx = startIdx; idx < modelNum; idx++ )
      {
        int modelIdx = order[idx];
        getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx], affMrgCtx);
        if ( affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx )
        {
          return;
        }

        // early termination
        if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
        {
          return;
        }
      }
    }
    ///> End: Constructed affine candidates
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while ( cnt < maxNumAffineMergeCand )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField( Mv( 0, 0 ), 0 );
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;

    if ( slice.isInterB() )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField( Mv( 0, 0 ), 0 );
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AFFINEMODEL_4PARAM;
    if ( cnt == mrgCandIdx )
    {
      return;
    }
    cnt++;
    affMrgCtx.numValidMergeCand++;
  }
}

#if AFFINE_MMVD
uint8_t PU::getMergeIdxFromAfMmvdBaseIdx(AffineMergeCtx& affMrgCtx, uint16_t afMmvdBaseIdx)
{
  uint8_t mergeIdx = (uint8_t)afMmvdBaseIdx;
  for (int i = 0; i < affMrgCtx.numValidMergeCand; i++)
  {
    if (affMrgCtx.mergeType[i] != MRG_TYPE_DEFAULT_N)
    {
      ++mergeIdx;
    }
    else
    {
      break;
    }
  }

  return mergeIdx;
}

void PU::getAfMmvdMvf(const PredictionUnit& pu, const AffineMergeCtx& affineMergeCtx, MvField mvfMmvd[2][3], const uint16_t afMmvdBase, const uint16_t offsetStep, const uint16_t offsetDir)
{
  CHECK(!pu.cu->affine, "Affine flag is not on for Affine MMVD mode!");
  CHECK(affineMergeCtx.mergeType[afMmvdBase] != MRG_TYPE_DEFAULT_N, "AFF_MMVD base candidate type is not regular Affine!");

  static const int32_t refMvdCands[AF_MMVD_STEP_NUM] = { 1, 2, 4, 8, 16 };
  static const int32_t iPicSize = pu.cu->slice->getPic()->lumaSize().area();
  static const int32_t mvShift  = iPicSize < 921600 ? 0 : ( iPicSize < 4096000 ? 2 : MV_FRACTIONAL_BITS_INTERNAL - 1); // 921600 = 1280x720, 4096000 = 2560x1600
  int step = refMvdCands[offsetStep] << mvShift;
  int affineType = affineMergeCtx.affineType[afMmvdBase];

  uint8_t interDir = affineMergeCtx.interDirNeighbours[afMmvdBase];
  int8_t refIdxL0 = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1)    ][0].refIdx;
  int8_t refIdxL1 = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1) + 1][0].refIdx;

  Mv baseMv[2][3];
  for (int i = 0; i < 3; i++)
  {
    baseMv[0][i] = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1)    ][i].mv;
    baseMv[1][i] = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1) + 1][i].mv;
    mvfMmvd[0][i].refIdx = refIdxL0;
    mvfMmvd[1][i].refIdx = refIdxL1;
    mvfMmvd[0][i].mv = Mv();
    mvfMmvd[1][i].mv = Mv();
  }

  int magY = (offsetDir >> 1) & 0x1;
  int sign = (offsetDir & 0x1) ? -1 : 1;
  int offsetX = (1 - magY) * sign * step;
  int offsetY = (    magY) * sign * step;
  Mv offsetMv(offsetX, offsetY);

  int numCp = (affineType == AFFINEMODEL_4PARAM) ? 2 : 3;
  for (int cpIdx = 0; cpIdx < numCp; cpIdx++)
  {
    if (interDir == 1)
    {
      mvfMmvd[0][cpIdx].mv = baseMv[0][cpIdx] + offsetMv;
    }
    else if (interDir == 2)
    {
      mvfMmvd[1][cpIdx].mv = baseMv[1][cpIdx] + offsetMv;
    }
    else if (interDir == 3)
    {
      int poc_cur = pu.cu->slice->getPOC();
      int poc_l0  = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, refIdxL0);
      int poc_l1  = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, refIdxL1);

      int distL0 = poc_l0 - poc_cur;
      int distL1 = poc_l1 - poc_cur;
      mvfMmvd[0][cpIdx].mv =                                                     baseMv[0][cpIdx] + offsetMv;
      mvfMmvd[1][cpIdx].mv = distL0 * distL1 < 0 ? baseMv[1][cpIdx] - offsetMv : baseMv[1][cpIdx] + offsetMv;
    }
  }
}

int32_t PU::getAfMmvdEstBits(const PredictionUnit &pu)
{
  int baseBits = (AF_MMVD_BASE_NUM == 1 ? 0 : pu.afMmvdBaseIdx + (pu.afMmvdBaseIdx == AF_MMVD_BASE_NUM - 1 ? 0: 1));
  int stepBits = pu.afMmvdStep + (pu.afMmvdStep == AF_MMVD_STEP_NUM - 1 ? 0 : 1);
  int dirBits  = gp_sizeIdxInfo->idxFrom(AF_MMVD_OFFSET_DIR);

  return stepBits + dirBits + baseBits;
}
#endif

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set Mv
  Mv mv[3];
  for ( int i = 0; i < 3; i++ )
  {
    mv[i] = mvField[i].mv;
  }
  setAllAffineMv( pu, mv[0], mv[1], mv[2], eRefList );

  // Set RefIdx
  CHECK( mvField[0].refIdx != mvField[1].refIdx || mvField[0].refIdx != mvField[2].refIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].refIdx;
}

void PU::setAllAffineMv(PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs)
{
  int width  = pu.Y().width;
  int shift = MAX_CU_DEPTH;
  const bool isTranslational = (affLT == affRT && affLT == affLB);

  if (clipCPMVs)
  {
    affLT.mvCliptoStorageBitDepth();
    affRT.mvCliptoStorageBitDepth();
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      affLB.mvCliptoStorageBitDepth();
    }
  }
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() << (shift - floorLog2(width));
  deltaMvHorY = (affRT - affLT).getVer() << (shift - floorLog2(width));
  int height = pu.Y().height;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = (affLB - affLT).getHor() << (shift - floorLog2(height));
    deltaMvVerY = (affLB - affLT).getVer() << (shift - floorLog2(height));
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }

  int mvScaleHor = affLT.getHor() << shift;
  int mvScaleVer = affLT.getVer() << shift;

  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
  const int halfBW = blockWidth >> 1;
  const int halfBH = blockHeight >> 1;

  MotionBuf mb = pu.getMotionBuf();
  int mvScaleTmpHor, mvScaleTmpVer;
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir );
#endif

  if (isTranslational)
  {
    mvScaleTmpHor = mvScaleHor;
    mvScaleTmpVer = mvScaleVer;
    roundAffineMv(mvScaleTmpHor, mvScaleTmpVer, shift);
    Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
    curMv.clipToStorageBitDepth();
    MotionInfo *mi = mb.buf;

    for (int y = 0; y < (pu.Y().height >> MIN_CU_LOG2); y++)
    {
      for (int x = 0; x < (pu.Y().width >> MIN_CU_LOG2); x++)
      {
        mi[x].mv[eRefList] = curMv;
      }
      mi += mb.stride;
    }
  }
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  else if (subblkMVSpreadOverLimit)
  {
      mvScaleTmpHor = mvScaleHor + deltaMvHorX * ( pu.Y().width >> 1 ) + deltaMvVerX * ( pu.Y().height >> 1 );
      mvScaleTmpVer = mvScaleVer + deltaMvHorY * ( pu.Y().width >> 1 ) + deltaMvVerY * ( pu.Y().height >> 1 );
      roundAffineMv(mvScaleTmpHor, mvScaleTmpVer, shift);
      Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
      curMv.clipToStorageBitDepth();
      MotionInfo *mi = mb.buf;
      for (int y = 0; y < (pu.Y().height >> MIN_CU_LOG2); y++)
      {
        for (int x = 0; x < (pu.Y().width >> MIN_CU_LOG2); x++)
        {
          mi[x].mv[eRefList] = curMv;
        }
        mi += mb.stride;
      }
  }
#endif
  else
  {
    int mvScaleHorLine = mvScaleHor + deltaMvHorX * halfBW + deltaMvVerX * halfBH;
    int mvScaleVerLine = mvScaleVer + deltaMvHorY * halfBW + deltaMvVerY * halfBH;
    int deltaMvVerXLine = deltaMvVerX * blockHeight;
    int deltaMvVerYLine = deltaMvVerY * blockHeight;
    int deltaMvHorXBlk = deltaMvHorX * blockWidth;
    int deltaMvHorYBlk = deltaMvHorY * blockWidth;

    MotionInfo *mi = mb.buf;

    for( int h = 0; h < pu.Y().height; h += blockHeight )
    {
      int mvScaleHorBlk = mvScaleHorLine;
      int mvScaleVerBlk = mvScaleVerLine;

      for( int w = 0; w < pu.Y().width; w += blockWidth )
      {
        mvScaleTmpHor = mvScaleHorBlk;
        mvScaleTmpVer = mvScaleVerBlk;

        mvScaleHorBlk += deltaMvHorXBlk;
        mvScaleVerBlk += deltaMvHorYBlk;

        roundAffineMv( mvScaleTmpHor, mvScaleTmpVer, shift );
        Mv curMv( mvScaleTmpHor, mvScaleTmpVer );
        curMv.clipToStorageBitDepth();

        mi[w >> MIN_CU_LOG2].mv[eRefList] = curMv;
      }

      mi += ( blockHeight >> MIN_CU_LOG2 ) * mb.stride;
      mvScaleHorLine += deltaMvVerXLine;
      mvScaleVerLine += deltaMvVerYLine;
    }
  }

  pu.mvAffi[eRefList][0] = affLT;
  pu.mvAffi[eRefList][1] = affRT;
  pu.mvAffi[eRefList][2] = affLB;
}

void clipColPos(int& posX, int& posY, const PredictionUnit& pu)
{
  Position puPos = pu.lumaPos();
  int log2CtuSize = floorLog2(pu.cs->sps->getCTUSize());
  int ctuX = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int horMax;
  const SubPic &curSubPic = pu.cu->slice->getPPS()->getSubPicFromPos(puPos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = std::min((int)curSubPic.getSubPicRight(), ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  }
  else
  {
    horMax = std::min((int)pu.cs->pps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  }
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min( (int)pu.cs->pps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1 );
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
}

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
  , int mmvdList
)
{
  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
#if INTER_LIC
  LICFlag = false;
#endif

  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
  Mv cTMv;

  if ( count )
  {
    if ( (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_0)) && slice.getRefPic( REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].refIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].mv;
    }
    else if ( slice.isInterB() && (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_1)) && slice.getRefPic( REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].refIdx ) == pColPic )
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].mv;
    }
  }

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////

  Mv cTempVector = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
  int numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv cColMv;
  int refIdx = 0;
  // use coldir.
  bool     bBSlice = slice.isInterB();

  Position centerPos;

  bool found = false;
  cTempVector = cTMv;

  cTempVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
  int tempX = cTempVector.getHor();
  int tempY = cTempVector.getVer();

  centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos(centerPos.x, centerPos.y, pu);

  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

  if (mi.isInter && mi.isIBCmot == false)
  {
    mrgCtx.interDirNeighbours[count] = 0;

    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (getColocatedMVP(pu, currRefPicList, centerPos, cColMv, refIdx, true))
      {
        // set as default, for further motion vector field spanning
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        mrgCtx.BcwIdx[count] = BCW_DEFAULT;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }

  if (!found)
  {
    return false;
  }
  if (mmvdList != 1)
  {
    int xOff = (puWidth >> 1) + tempX;
    int yOff = (puHeight >> 1) + tempY;

    MotionBuf &mb = mrgCtx.subPuMvpMiBuf;

    const bool isBiPred = isBipredRestriction(pu);

    for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
    {
      for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
      {
        Position colPos{ x + xOff, y + yOff };

        clipColPos(colPos.x, colPos.y, pu);

        colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };

        const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

        MotionInfo mi;

        found       = false;
        mi.isInter  = true;
        mi.sliceIdx = slice.getIndependentSliceIdx();
        mi.isIBCmot = false;
        if (colMi.isInter && colMi.isIBCmot == false)
        {
          for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
          {
            RefPicList currRefPicList = RefPicList(currRefListId);
            if (getColocatedMVP(pu, currRefPicList, colPos, cColMv, refIdx, true))
            {
              mi.refIdx[currRefListId] = 0;
              mi.mv[currRefListId]     = cColMv;
              found                    = true;
            }
          }
        }
        if (!found)
        {
          mi.mv[0]     = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
          mi.mv[1]     = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
          mi.refIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].refIdx;
          mi.refIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].refIdx;
        }

        mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

        if (isBiPred && mi.interDir == 3)
        {
          mi.interDir  = 1;
          mi.mv[1]     = Mv();
          mi.refIdx[1] = NOT_VALID;
        }

        mb.subBuf(g_miScaling.scale(Position{ x, y } - pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight)))
          .fill(mi);
      }
    }
  }
  return true;
}

#if MULTI_PASS_DMVR
void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx, Mv* bdmvrSubPuMv0, Mv* bdmvrSubPuMv1, Mv* bdofSubPuMvOffset)
#else
void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
#endif
{
#if !MULTI_PASS_DMVR
  MotionBuf mb = pu.getMotionBuf();
#endif
#if JVET_W0123_TIMD_FUSION
  IpmBuf ib = pu.getIpmBuf();
#endif

  if (!pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N || pu.mergeType == MRG_TYPE_IBC)
  {
    MotionInfo mi;

    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();
#if INTER_LIC
    mi.usesLIC = pu.cu->LICFlag;
#endif

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;
      mi.useAltHpelIf = pu.cu->imv == IMV_HPEL;
      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

#if MULTI_PASS_DMVR
    if (pu.bdmvrRefine)
    {
      CHECK(bdmvrSubPuMv0 == nullptr, "this is not possible");
      const int dx = std::min<int>(pu.lwidth (), BDOF_SUBPU_DIM);
      const int dy = std::min<int>(pu.lheight(), BDOF_SUBPU_DIM);
      int subPuIdx = 0;
      const int bioSubPuIdxStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(pu.lwidth() >> BDOF_SUBPU_DIM_LOG2));

      for (int yStart = 0; yStart < pu.lheight(); yStart += dy)
      {
        for (int xStart = 0; xStart < pu.lwidth(); xStart += dx)
        {
          const int bdmvrSubPuIdx = (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
          mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx] + bdofSubPuMvOffset[subPuIdx];
          mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx] - bdofSubPuMvOffset[subPuIdx];

          subPuIdx++;
          MotionBuf mb = pu.cs->getMotionBuf(Area(pu.lx() + xStart, pu.ly() + yStart, dx, dy));
          mb.fill(mi);
        }
        subPuIdx += bioSubPuIdxStrideIncr;
      }
#if JVET_W0123_TIMD_FUSION
      MotionBuf mb = pu.getMotionBuf();
      spanIpmInfoInter(pu, mb, ib);
#endif
      return;
    }
    MotionBuf mb = pu.getMotionBuf();
#endif
    if (pu.cu->affine)
    {
      mi.mv[0].setZero(); // to make sure filling of MV in unused reference list
      mi.mv[1].setZero();
      mb.fill(mi);
      if (pu.refIdx[0] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
      }
      if (pu.refIdx[1] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
      }
#if JVET_W0123_TIMD_FUSION
      spanIpmInfoInter(pu, mb, ib);
#endif
    }
    else
    {
      mb.fill(mi);
#if JVET_W0123_TIMD_FUSION
      if (mi.isIBCmot
#if JVET_X0141_CIIP_TIMD_TM
        || pu.ciipFlag
#endif
        )
      {
        ib.fill(PLANAR_IDX);
      }
      else
      {
        spanIpmInfoInter(pu, mb, ib);
      }
#endif
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
#if MULTI_PASS_DMVR
    MotionBuf mb = pu.getMotionBuf();
#endif
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
#if JVET_W0123_TIMD_FUSION
    spanIpmInfoInter(pu, mb, ib);
#endif
  }
}

#if JVET_W0123_TIMD_FUSION
#if MULTI_PASS_DMVR
void PU::spanMotionInfo2( PredictionUnit &pu, const MergeCtx &mrgCtx, Mv* bdmvrSubPuMv0, Mv* bdmvrSubPuMv1, Mv* bdofSubPuMvOffset)
#else
void PU::spanMotionInfo2( PredictionUnit &pu, const MergeCtx &mrgCtx )
#endif
{
#if !MULTI_PASS_DMVR
  MotionBuf mb = pu.getMotionBuf();
#endif

  if (!pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N || pu.mergeType == MRG_TYPE_IBC)
  {
    MotionInfo mi;

    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();
#if INTER_LIC
    mi.usesLIC = pu.cu->LICFlag;
#endif

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;
      mi.useAltHpelIf = pu.cu->imv == IMV_HPEL;
      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

#if MULTI_PASS_DMVR
    if (pu.bdmvrRefine)
    {
      CHECK(bdmvrSubPuMv0 == nullptr, "this is not possible");
      const int dx = std::min<int>(pu.lwidth (), BDOF_SUBPU_DIM);
      const int dy = std::min<int>(pu.lheight(), BDOF_SUBPU_DIM);
      int subPuIdx = 0;
      const int bioSubPuIdxStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(pu.lwidth() >> BDOF_SUBPU_DIM_LOG2));

      for (int yStart = 0; yStart < pu.lheight(); yStart += dy)
      {
        for (int xStart = 0; xStart < pu.lwidth(); xStart += dx)
        {
          const int bdmvrSubPuIdx = (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
          mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx] + bdofSubPuMvOffset[subPuIdx];
          mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx] - bdofSubPuMvOffset[subPuIdx];

          subPuIdx++;
          MotionBuf mb = pu.cs->getMotionBuf(Area(pu.lx() + xStart, pu.ly() + yStart, dx, dy));
          mb.fill(mi);
        }
        subPuIdx += bioSubPuIdxStrideIncr;
      }
      return;
    }
    MotionBuf mb = pu.getMotionBuf();
#endif
    if (pu.cu->affine)
    {
      mi.mv[0].setZero(); // to make sure filling of MV in unused reference list
      mi.mv[1].setZero();
      mb.fill(mi);
      if (pu.refIdx[0] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
      }
      if (pu.refIdx[1] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
      }
    }
    else
    {
      mb.fill(mi);
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
#if MULTI_PASS_DMVR
    MotionBuf mb = pu.getMotionBuf();
#endif
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
  }
}

void PU::spanIpmInfoIntra( PredictionUnit &pu)
{
  int ipm = PU::getIntraDirLuma(pu);
  if (pu.cu->timd)
  {
    ipm = MAP131TO67(ipm);
  }
  IpmBuf ib = pu.getIpmBuf();
  ib.fill(ipm);
}

#if RPR_ENABLE
void  scalePositionInRef( PredictionUnit& pu, const PPS& pps, RefPicList refList, int refIdx, Position& PosY )
{
  const Picture* refPic = pu.cu->slice->getRefPic( refList, refIdx )->unscaledPic;
  const bool scaled = refPic->isRefScaled( &pps );
  if (scaled)
  {
    const SPS* sps = pu.cu->cs->sps;
    ChromaFormat chFmt = sps->getChromaFormatIdc();

    const std::pair<int, int> scalingRatio = pu.cu->slice->getScalingRatio( refList, refIdx );

    int64_t x0Int;
    int64_t y0Int;

    int posX = PosY.x;
    int posY = PosY.y;

    const int posShift = SCALE_RATIO_BITS - 4;

    int shiftHor = MV_FRACTIONAL_BITS_INTERNAL ;
    int shiftVer = MV_FRACTIONAL_BITS_INTERNAL ;
    int offX = 1 << (posShift - shiftHor - 1);
    int offY = 1 << (posShift - shiftVer - 1);

    x0Int = (posX << (4)) * (int64_t)scalingRatio.first ;
    x0Int = SIGN(x0Int) * ((llabs(x0Int) + ((long long)1 << 7)) >> 8) + ((refPic->getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX(chFmt)) << (posShift));

    y0Int = (posY << (4)) * (int64_t)scalingRatio.second ;
    y0Int = SIGN(y0Int) * ((llabs(y0Int) + ((long long)1 << 7)) >> 8) + ((refPic->getScalingWindow().getWindowTopOffset() * SPS::getWinUnitY(chFmt)) << (posShift));

    int xInt0 = ((int32_t)x0Int + offX) >> posShift;
    int yInt0 = ((int32_t)y0Int + offY) >> posShift;

    PosY.x = std::min( std::max(xInt0, 0), (int)refPic->unscaledPic->getPicWidthInLumaSamples()  - 1 );
    PosY.y = std::min( std::max(yInt0, 0), (int)refPic->unscaledPic->getPicHeightInLumaSamples() - 1 );
  }
  //else
  //  clipColPos( PosY.x, PosY.y, pu );
}
#endif

void PU::spanIpmInfoInter( PredictionUnit &pu, MotionBuf &mb, IpmBuf &ib)
{
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
  Mv cMv;
  RefPicList refList;
  int refIdx;
  Position PosY;
  MotionInfo tempMi;
  MotionInfo mi0;
  MotionInfo mi1;
  Position PosY0;
  Position PosY1;
  Mv cMv0;
  Mv cMv1;
#if RPR_ENABLE
  const Picture* pRefPic0;
  const Picture* pRefPic1;
#else
  Picture* pRefPic0;
  Picture* pRefPic1;
#endif
  uint8_t* ii = ib.buf;
  int ibH = mb.height;
  int ibW = mb.width;
  for (int y = 0; y < ibH; y++)
  {
    for (int x = 0; x < ibW; x++)
    {
      uint8_t ipm = PLANAR_IDX;
      tempMi = mb.at(x, y);
      if (tempMi.interDir != 3)
      {
        if (tempMi.interDir != 2)
        {
          cMv = tempMi.mv[0];
          refList = REF_PIC_LIST_0;
          refIdx = tempMi.refIdx[0];
        }
        else
        {
          cMv = tempMi.mv[1];
          refList = REF_PIC_LIST_1;
          refIdx = tempMi.refIdx[1];
        }
        if (refList < 0 || refIdx < 0)
        {
          ipm = PLANAR_IDX;
        }
        else
        {
          cMv.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
          PosY.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv.getHor();
          PosY.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv.getVer();
          clipColPos(PosY.x, PosY.y, pu);
#if RPR_ENABLE
          scalePositionInRef( pu, *pu.cs->pps, refList, refIdx, PosY );
          PosY.x = (PosY.x & mask);
          PosY.y = (PosY.y & mask);
          ipm = pu.cu->slice->getRefPic(refList, refIdx)->unscaledPic->cs->getIpmInfo(PosY);
#else
          PosY.x = (PosY.x & mask);
          PosY.y = (PosY.y & mask);
          ipm = pu.cu->slice->getRefPic(refList, refIdx)->cs->getIpmInfo(PosY);
#endif
        }
      }
      else
      {
#if RPR_ENABLE
        pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0])->unscaledPic;
#else
        pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0]);
#endif
        cMv0 = tempMi.mv[0];
        cMv0.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        PosY0.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv0.getHor();
        PosY0.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv0.getVer();
        clipColPos(PosY0.x, PosY0.y, pu);
#if RPR_ENABLE
        scalePositionInRef( pu, *pu.cs->pps, REF_PIC_LIST_0, tempMi.refIdx[0], PosY0 );
#endif
        PosY0.x = (PosY0.x & mask);
        PosY0.y = (PosY0.y & mask);
        mi0 = pRefPic0->cs->getMotionInfo(PosY0);
        int ipm0 = pRefPic0->cs->getIpmInfo(PosY0);
        int pocDiff0 = abs(pRefPic0->getPOC() - pu.cu->slice->getPOC());

#if RPR_ENABLE
        pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1])->unscaledPic;
#else
        pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1]);
#endif
        cMv1 = tempMi.mv[1];
        cMv1.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        PosY1.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv1.getHor();
        PosY1.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv1.getVer();
        clipColPos(PosY1.x, PosY1.y, pu);
#if RPR_ENABLE
        scalePositionInRef( pu, *pu.cs->pps, REF_PIC_LIST_1, tempMi.refIdx[1], PosY1 );
#endif
        PosY1.x = (PosY1.x & mask);
        PosY1.y = (PosY1.y & mask);
        mi1 = pRefPic1->cs->getMotionInfo(PosY1);
        int ipm1 = pRefPic1->cs->getIpmInfo(PosY1);
        int pocDiff1 = abs(pRefPic1->getPOC() - pu.cu->slice->getPOC());

        if (!mi0.isInter && mi1.isInter)
        {
          ipm = ipm0;
        }
        else if (!mi1.isInter && mi0.isInter)
        {
          ipm = ipm1;
        }
        else if (ipm0 > DC_IDX && ipm1 <= DC_IDX)
        {
          ipm = ipm0;
        }
        else if (ipm0 <= DC_IDX && ipm1 > DC_IDX)
        {
          ipm = ipm1;
        }
        else if (pocDiff0 < pocDiff1)
        {
          ipm = ipm0;
        }
        else if (pocDiff1 < pocDiff0)
        {
          ipm = ipm1;
        }
        else if (pRefPic0->m_prevQP[0] > pRefPic1->m_prevQP[0])
        {
          ipm = ipm1;
        }
        else
        {
          ipm = ipm0;
        }
      }
      ii[x] = ipm;
    }
    ii += ib.stride;
  }
}
#endif

void PU::applyImv( PredictionUnit& pu, MergeCtx &mrgCtx, InterPrediction *interPred )
{
  if( !pu.mergeFlag )
  {
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
      pu.mvd[0].changeTransPrecAmvr2Internal(pu.cu->imv);
      unsigned mvp_idx = pu.mvpIdx[0];
      AMVPInfo amvpInfo;
      if (CU::isIBC(*pu.cu))
      {
        PU::fillIBCMvpCand(pu, amvpInfo);
      }
      else
      PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo
#if TM_AMVP
                    , interPred
#endif
      );
      pu.mvpNum[0] = amvpInfo.numCand;
      pu.mvpIdx[0] = mvp_idx;
      pu.mv    [0] = amvpInfo.mvCand[mvp_idx] + pu.mvd[0];
      pu.mv[0].mvCliptoStorageBitDepth();
    }

    if (pu.interDir != 1 /* PRED_L0 */)
    {
      if( !( pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3 ) && pu.cu->imv )/* PRED_BI */
      {
        pu.mvd[1].changeTransPrecAmvr2Internal(pu.cu->imv);
      }
      unsigned mvp_idx = pu.mvpIdx[1];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo
#if TM_AMVP
                    , interPred
#endif
      );
      pu.mvpNum[1] = amvpInfo.numCand;
      pu.mvpIdx[1] = mvp_idx;
      pu.mv    [1] = amvpInfo.mvCand[mvp_idx] + pu.mvd[1];
      pu.mv[1].mvCliptoStorageBitDepth();
    }
  }
  else
  {
    // this function is never called for merge
    THROW("unexpected");
    PU::getInterMergeCandidates(pu, mrgCtx, 0);

    mrgCtx.setMergeInfo( pu, pu.mergeIdx );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}

bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu)
{
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    if (pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[0])->longTerm
      || pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[1])->longTerm)
    {
      return false;
    }
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, pu.refIdx[0]);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, pu.refIdx[1]);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}

void PU::restrictBiPredMergeCandsOne(PredictionUnit &pu)
{
  if (PU::isBipredRestriction(pu))
  {
    if (pu.interDir == 3)
    {
      pu.interDir = 1;
      pu.refIdx[1] = -1;
      pu.mv[1] = Mv(0, 0);
      pu.cu->BcwIdx = BCW_DEFAULT;
#if MULTI_HYP_PRED
      pu.addHypData.clear();
      pu.numMergedAddHyps = 0;
#endif
    }
  }
}

#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
void PU::getGeoMergeCandidates(PredictionUnit &pu, MergeCtx& geoMrgCtx, MergeCtx* mergeCtx)
#else
void PU::getGeoMergeCandidates(const PredictionUnit &pu, MergeCtx& geoMrgCtx, MergeCtx* mergeCtx)
#endif
#else
void PU::getGeoMergeCandidates( const PredictionUnit &pu, MergeCtx& geoMrgCtx )
#endif
{
  MergeCtx tmpMergeCtx;

  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
  geoMrgCtx.numValidMergeCand = 0;

  for (int32_t i = 0; i < GEO_MAX_NUM_UNI_CANDS; i++)
  {
    geoMrgCtx.BcwIdx[i] = BCW_DEFAULT;
    geoMrgCtx.interDirNeighbours[i] = 0;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].mv = Mv();
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
    geoMrgCtx.useAltHpelIf[i] = false;
#if INTER_LIC
    geoMrgCtx.LICFlags[i] = false;
#endif
  }
#if JVET_W0097_GPM_MMVD_TM
  if (mergeCtx == NULL)
  {
#if TM_MRG
    const bool tmMergeFlag = pu.tmMergeFlag;
    pu.tmMergeFlag = false;
#endif
#endif
  PU::getInterMergeCandidates(pu, tmpMergeCtx, 0);
#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
  pu.tmMergeFlag = tmMergeFlag;
#endif
  }
  else
  {
    memcpy(tmpMergeCtx.interDirNeighbours, mergeCtx->interDirNeighbours, maxNumMergeCand * sizeof(unsigned char));
    memcpy(tmpMergeCtx.mvFieldNeighbours, mergeCtx->mvFieldNeighbours, (maxNumMergeCand << 1) * sizeof(MvField));
  }
#endif
  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if( tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity) )
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1 + parity;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].refIdx;
#if JVET_W0097_GPM_MMVD_TM
      if (geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        continue;
      }
#endif
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
#if JVET_W0097_GPM_MMVD_TM
      if (geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        continue;
      }
#endif
      geoMrgCtx.numValidMergeCand++;
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
      {
        return;
      }
    }
  }
#if JVET_W0097_GPM_MMVD_TM
  // add more parity based geo candidates, in an opposite parity rule
  if (geoMrgCtx.numValidMergeCand < pu.cs->sps->getMaxNumGeoCand())
  {
    for (int32_t i = 0; i < maxNumMergeCand; i++)
    {
      int parity = i & 1;
      if (tmpMergeCtx.interDirNeighbours[i] == 3)
      {
        geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
        if (geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
        {
          continue;
        }
        geoMrgCtx.numValidMergeCand++;
        if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
        {
          return;
        }
      }
    }
  }

  // add at most two average based geo candidates
  if (geoMrgCtx.numValidMergeCand < pu.cs->sps->getMaxNumGeoCand())
  {
    // add one L0 cand by averaging the first two available L0 candidates
    int cnt = 0;
    int firstAvailRefIdx = -1;
    Mv  avgMv;
    avgMv.setZero();
    for (int i = 0; i < geoMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (geoMrgCtx.interDirNeighbours[i] == 1)
      {
        avgMv += geoMrgCtx.mvFieldNeighbours[i * 2].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = geoMrgCtx.mvFieldNeighbours[i * 2].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv.hor, avgMv.ver, 1);
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2].setMvField(avgMv, firstAvailRefIdx);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2 + 1].setMvField(Mv(0, 0), NOT_VALID);

      if (!geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        geoMrgCtx.numValidMergeCand++;
      }
      if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
      {
        return;
      }
    }

    // add one L1 cand by averaging the first two available L1 candidates
    cnt = 0;
    firstAvailRefIdx = -1;
    avgMv.setZero();
    for (int i = 0; i < geoMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (geoMrgCtx.interDirNeighbours[i] == 2)
      {
        avgMv += geoMrgCtx.mvFieldNeighbours[i * 2 + 1].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = geoMrgCtx.mvFieldNeighbours[i * 2 + 1].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv.hor, avgMv.ver, 1);
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2 + 1].setMvField(avgMv, firstAvailRefIdx);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2].setMvField(Mv(0, 0), NOT_VALID);
      if (!geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        geoMrgCtx.numValidMergeCand++;
      }
      if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
      {
        return;
      }
    }
  }
  if (geoMrgCtx.numValidMergeCand < pu.cs->sps->getMaxNumGeoCand())
  {
    const Slice &slice = *pu.cs->slice;
    int         iNumRefIdx = std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1));

    int r = 0;
    int refcnt = 0;

    for (int32_t i = geoMrgCtx.numValidMergeCand; i < pu.cs->sps->getMaxNumGeoCand(); i++)
    {
      int parity = i & 1;
      if (0x01 + parity)
      {
        geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1 + parity;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = r;

        if (refcnt == iNumRefIdx - 1)
        {
          r = 0;
        }
        else
        {
          ++r;
          ++refcnt;
        }

        geoMrgCtx.numValidMergeCand++;
        if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
        {
          return;
        }
      }
    }
  }
#endif
}

void PU::spanGeoMotionInfo( PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1)
{
  pu.geoSplitDir  = splitDir;
  pu.geoMergeIdx0 = candIdx0;
  pu.geoMergeIdx1 = candIdx1;
  MotionBuf mb = pu.getMotionBuf();
#if JVET_W0123_TIMD_FUSION
  IpmBuf ib = pu.getIpmBuf();
#endif

  MotionInfo biMv;
  biMv.isInter  = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir = 1;
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }

  int16_t angle = g_GeoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_GeoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;

  if (distanceIdx > 0)
  {
    if( angle % 16 == 8 || ( angle % 16 != 0 && pu.lheight() >= pu.lwidth() ) )
    {
      offsetY += angle < 16 ? ( ( distanceIdx * pu.lheight() ) >> 3 ) : -( int ) ( ( distanceIdx * pu.lheight() ) >> 3 );
    }
    else
    {
      offsetX += angle < 16 ? ( ( distanceIdx * pu.lwidth() ) >> 3 ) : -( int ) ( ( distanceIdx * pu.lwidth() ) >> 3 );
    }
  }

  MotionInfo *motionInfo = mb.buf;

  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_Dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_Dis[distanceX] + lookUpY;
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        motionInfo[x].isInter = true;
        motionInfo[x].interDir = biMv.interDir;
        motionInfo[x].refIdx[0] = biMv.refIdx[0];
        motionInfo[x].refIdx[1] = biMv.refIdx[1];
        motionInfo[x].mv[0] = biMv.mv[0];
        motionInfo[x].mv[1] = biMv.mv[1];
        motionInfo[x].sliceIdx = biMv.sliceIdx;
      }
      else if (tpmMask == 0)
      {
        motionInfo[x].isInter = true;
        motionInfo[x].interDir = geoMrgCtx.interDirNeighbours[candIdx0];
        motionInfo[x].refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
        motionInfo[x].refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
        motionInfo[x].mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
        motionInfo[x].mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
        motionInfo[x].sliceIdx = biMv.sliceIdx;
      }
      else
      {
        motionInfo[x].isInter = true;
        motionInfo[x].interDir = geoMrgCtx.interDirNeighbours[candIdx1];
        motionInfo[x].refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
        motionInfo[x].refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
        motionInfo[x].mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
        motionInfo[x].mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
        motionInfo[x].sliceIdx = biMv.sliceIdx;
      }
    }
    motionInfo += mb.stride;
  }
#if JVET_W0123_TIMD_FUSION
  spanIpmInfoInter(pu, mb, ib);
#endif
}

#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
void PU::spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1)
{
  pu.geoSplitDir = splitDir;
  pu.geoMergeIdx0 = mergeIdx0;
  pu.geoMergeIdx1 = mergeIdx1;
  pu.geoTmFlag0 = tmFlag0;
  pu.geoMMVDFlag0 = mmvdFlag0;
  pu.geoMMVDIdx0 = mmvdIdx0;
  pu.geoTmFlag1 = tmFlag1;
  pu.geoMMVDFlag1 = mmvdFlag1;
  pu.geoMMVDIdx1 = mmvdIdx1;

  MergeCtx *mergeCtx0 = (tmFlag0 ? &geoTmMrgCtx0 : &geoMrgCtx);
  MergeCtx *mergeCtx1 = (tmFlag1 ? &geoTmMrgCtx1 : &geoMrgCtx);

  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  const int MmvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  const int ExtMmvdCands[9] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  const int* refMvdCands = (extMMVD ? ExtMmvdCands : MmvdCands);
  Mv mvOffset0[2], mvOffset1[2], deltaMv;

  if (mmvdFlag0)
  {
    int fPosStep = (extMMVD ? (mmvdIdx0 >> 3) : (mmvdIdx0 >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx0 - (fPosStep << 3)) : (mmvdIdx0 - (fPosStep << 2)));

    if (fPosPosition == 0)
    {
      deltaMv = Mv(refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 1)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 2)
    {
      deltaMv = Mv(0, refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 3)
    {
      deltaMv = Mv(0, -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 4)
    {
      deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 5)
    {
      deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 6)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 7)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }

    if (mergeCtx0->interDirNeighbours[mergeIdx0] == 1)
    {
      mvOffset0[0] = deltaMv;
    }
    else
    {
      mvOffset0[1] = deltaMv;
    }
  }

  if (mmvdFlag1)
  {
    int fPosStep = (extMMVD ? (mmvdIdx1 >> 3) : (mmvdIdx1 >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx1 - (fPosStep << 3)) : (mmvdIdx1 - (fPosStep << 2)));

    if (fPosPosition == 0)
    {
      deltaMv = Mv(refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 1)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 2)
    {
      deltaMv = Mv(0, refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 3)
    {
      deltaMv = Mv(0, -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 4)
    {
      deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 5)
    {
      deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 6)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 7)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }

    if (mergeCtx1->interDirNeighbours[mergeIdx1] == 1)
    {
      mvOffset1[0] = deltaMv;
    }
    else
    {
      mvOffset1[1] = deltaMv;
    }
  }

  MotionBuf mb = pu.getMotionBuf();

  MotionInfo biMv;
  biMv.isInter = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if (mergeCtx0->interDirNeighbours[mergeIdx0] == 1 && mergeCtx1->interDirNeighbours[mergeIdx1] == 2)
  {
    biMv.interDir = 3;
    biMv.mv[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
    biMv.mv[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
    biMv.refIdx[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx;
    biMv.refIdx[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
  }
  else if (mergeCtx0->interDirNeighbours[mergeIdx0] == 2 && mergeCtx1->interDirNeighbours[mergeIdx1] == 1)
  {
    biMv.interDir = 3;
    biMv.mv[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
    biMv.mv[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
    biMv.refIdx[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
    biMv.refIdx[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
  }
  else if (mergeCtx0->interDirNeighbours[mergeIdx0] == 1 && mergeCtx1->interDirNeighbours[mergeIdx1] == 1)
  {
    biMv.interDir = 1;
    biMv.mv[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
  }
  else if (mergeCtx0->interDirNeighbours[mergeIdx0] == 2 && mergeCtx1->interDirNeighbours[mergeIdx1] == 2)
  {
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
  }

  int16_t angle = g_GeoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_GeoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && pu.lheight() >= pu.lwidth()))
    {
      offsetY += angle < 16 ? ((distanceIdx * pu.lheight()) >> 3) : -(int)((distanceIdx * pu.lheight()) >> 3);
    }
    else
    {
      offsetX += angle < 16 ? ((distanceIdx * pu.lwidth()) >> 3) : -(int)((distanceIdx * pu.lwidth()) >> 3);
    }
  }
  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_Dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_Dis[distanceX] + lookUpY;
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = biMv.interDir;
        mb.at(x, y).refIdx[0] = biMv.refIdx[0];
        mb.at(x, y).refIdx[1] = biMv.refIdx[1];
        mb.at(x, y).mv[0] = biMv.mv[0];
        mb.at(x, y).mv[1] = biMv.mv[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
      else if (tpmMask == 0)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = mergeCtx0->interDirNeighbours[mergeIdx0];
        mb.at(x, y).refIdx[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx;
        mb.at(x, y).refIdx[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
        mb.at(x, y).mv[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
      else
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = mergeCtx1->interDirNeighbours[mergeIdx1];
        mb.at(x, y).refIdx[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
        mb.at(x, y).refIdx[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
        mb.at(x, y).mv[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
    }
  }
}
#else
void PU::spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1)
{
  pu.geoSplitDir = splitDir;
  pu.geoMergeIdx0 = mergeIdx0;
  pu.geoMergeIdx1 = mergeIdx1;
  pu.geoMMVDFlag0 = mmvdFlag0;
  pu.geoMMVDIdx0 = mmvdIdx0;
  pu.geoMMVDFlag1 = mmvdFlag1;
  pu.geoMMVDIdx1 = mmvdIdx1;

  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  const int MmvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  const int ExtMmvdCands[9] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  const int* refMvdCands = (extMMVD ? ExtMmvdCands : MmvdCands);
  Mv mvOffset0[2], mvOffset1[2], deltaMv;

  if (mmvdFlag0)
  {
    int fPosStep = (extMMVD ? (mmvdIdx0 >> 3) : (mmvdIdx0 >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx0 - (fPosStep << 3)) : (mmvdIdx0 - (fPosStep << 2)));

    if (fPosPosition == 0)
    {
      deltaMv = Mv(refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 1)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 2)
    {
      deltaMv = Mv(0, refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 3)
    {
      deltaMv = Mv(0, -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 4)
    {
      deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 5)
    {
      deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 6)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 7)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }

    if (geoMrgCtx.interDirNeighbours[mergeIdx0] == 1)
    {
      mvOffset0[0] = deltaMv;
    }
    else
    {
      mvOffset0[1] = deltaMv;
    }
  }

  if (mmvdFlag1)
  {
    int fPosStep = (extMMVD ? (mmvdIdx1 >> 3) : (mmvdIdx1 >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx1 - (fPosStep << 3)) : (mmvdIdx1 - (fPosStep << 2)));

    if (fPosPosition == 0)
    {
      deltaMv = Mv(refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 1)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 2)
    {
      deltaMv = Mv(0, refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 3)
    {
      deltaMv = Mv(0, -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 4)
    {
      deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 5)
    {
      deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 6)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 7)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }

    if (geoMrgCtx.interDirNeighbours[mergeIdx1] == 1)
    {
      mvOffset1[0] = deltaMv;
    }
    else
    {
      mvOffset1[1] = deltaMv;
    }
  }

  MotionBuf mb = pu.getMotionBuf();

  MotionInfo biMv;
  biMv.isInter = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if (geoMrgCtx.interDirNeighbours[mergeIdx0] == 1 && geoMrgCtx.interDirNeighbours[mergeIdx1] == 2)
  {
    biMv.interDir = 3;
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx0 << 1].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
  }
  else if (geoMrgCtx.interDirNeighbours[mergeIdx0] == 2 && geoMrgCtx.interDirNeighbours[mergeIdx1] == 1)
  {
    biMv.interDir = 3;
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx1 << 1].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
  }
  else if (geoMrgCtx.interDirNeighbours[mergeIdx0] == 1 && geoMrgCtx.interDirNeighbours[mergeIdx1] == 1)
  {
    biMv.interDir = 1;
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
  }
  else if (geoMrgCtx.interDirNeighbours[mergeIdx0] == 2 && geoMrgCtx.interDirNeighbours[mergeIdx1] == 2)
  {
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
  }

  int16_t angle = g_GeoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_GeoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && pu.lheight() >= pu.lwidth()))
    {
      offsetY += angle < 16 ? ((distanceIdx * pu.lheight()) >> 3) : -(int)((distanceIdx * pu.lheight()) >> 3);
    }
    else
    {
      offsetX += angle < 16 ? ((distanceIdx * pu.lwidth()) >> 3) : -(int)((distanceIdx * pu.lwidth()) >> 3);
    }
  }
  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_Dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_Dis[distanceX] + lookUpY;
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = biMv.interDir;
        mb.at(x, y).refIdx[0] = biMv.refIdx[0];
        mb.at(x, y).refIdx[1] = biMv.refIdx[1];
        mb.at(x, y).mv[0] = biMv.mv[0];
        mb.at(x, y).mv[1] = biMv.mv[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
      else if (tpmMask == 0)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = geoMrgCtx.interDirNeighbours[mergeIdx0];
        mb.at(x, y).refIdx[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx0 << 1].refIdx;
        mb.at(x, y).refIdx[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
        mb.at(x, y).mv[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
      else
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = geoMrgCtx.interDirNeighbours[mergeIdx1];
        mb.at(x, y).refIdx[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx1 << 1].refIdx;
        mb.at(x, y).refIdx[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = geoMrgCtx.mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
        mb.at(x, y).mv[1] = geoMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
    }
  }
}
#endif
#endif

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
  bool bNonZeroMvd = false;

  for( const auto &pu : CU::traversePUs( cu ) )
  {
    if( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getHor() != 0;
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getVer() != 0;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        if( !pu.cu->cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getHor() != 0;
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getVer() != 0;
        }
      }
    }
  }

  return bNonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd( const CodingUnit& cu )
{
  bool nonZeroAffineMvd = false;

  if ( !cu.affine || cu.firstPU->mergeFlag )
  {
    return false;
  }

  for ( const auto &pu : CU::traversePUs( cu ) )
  {
    if ( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if ( pu.interDir != 2 /* PRED_L1 */ )
      {
        for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
        {
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getHor() != 0;
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getVer() != 0;
        }
      }

      if ( pu.interDir != 1 /* PRED_L0 */ )
      {
        if ( !pu.cu->cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
          {
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getHor() != 0;
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getVer() != 0;
          }
        }
      }
    }
  }

  return nonZeroAffineMvd;
}

uint8_t CU::getSbtInfo( uint8_t idx, uint8_t pos )
{
  return ( pos << 4 ) + ( idx << 0 );
}

uint8_t CU::getSbtIdx( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 0 ) & 0xf;
}

uint8_t CU::getSbtPos( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 4 ) & 0x3;
}

uint8_t CU::getSbtMode( uint8_t sbtIdx, uint8_t sbtPos )
{
  uint8_t sbtMode = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: sbtMode = sbtPos + SBT_VER_H0;  break;
  case SBT_HOR_HALF: sbtMode = sbtPos + SBT_HOR_H0;  break;
  case SBT_VER_QUAD: sbtMode = sbtPos + SBT_VER_Q0;  break;
  case SBT_HOR_QUAD: sbtMode = sbtPos + SBT_HOR_Q0;  break;
  default:           assert( 0 );
  }

  assert( sbtMode < NUMBER_SBT_MODE );
  return sbtMode;
}

uint8_t CU::getSbtIdxFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
  {
    return SBT_VER_HALF;
  }
  else if( sbtMode <= SBT_HOR_H1 )
  {
    return SBT_HOR_HALF;
  }
  else if( sbtMode <= SBT_VER_Q1 )
  {
    return SBT_VER_QUAD;
  }
  else if( sbtMode <= SBT_HOR_Q1 )
  {
    return SBT_HOR_QUAD;
  }
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::getSbtPosFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
  {
    return sbtMode - SBT_VER_H0;
  }
  else if( sbtMode <= SBT_HOR_H1 )
  {
    return sbtMode - SBT_HOR_H0;
  }
  else if( sbtMode <= SBT_VER_Q1 )
  {
    return sbtMode - SBT_VER_Q0;
  }
  else if( sbtMode <= SBT_HOR_Q1 )
  {
    return sbtMode - SBT_HOR_Q0;
  }
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::targetSbtAllowed( uint8_t sbtIdx, uint8_t sbtAllowed )
{
  uint8_t val = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: val = ( ( sbtAllowed >> SBT_VER_HALF ) & 0x1 ); break;
  case SBT_HOR_HALF: val = ( ( sbtAllowed >> SBT_HOR_HALF ) & 0x1 ); break;
  case SBT_VER_QUAD: val = ( ( sbtAllowed >> SBT_VER_QUAD ) & 0x1 ); break;
  case SBT_HOR_QUAD: val = ( ( sbtAllowed >> SBT_HOR_QUAD ) & 0x1 ); break;
  default:           CHECK( 1, "unknown SBT type" );
  }
  return val;
}

uint8_t CU::numSbtModeRdo( uint8_t sbtAllowed )
{
  uint8_t num = 0;
  uint8_t sum = 0;
  num = targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  num = targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  return sum;
}

bool CU::isSbtMode( const uint8_t sbtInfo )
{
  uint8_t sbtIdx = getSbtIdx( sbtInfo );
  return sbtIdx >= SBT_VER_HALF && sbtIdx <= SBT_HOR_QUAD;
}

bool CU::isSameSbtSize( const uint8_t sbtInfo1, const uint8_t sbtInfo2 )
{
  uint8_t sbtIdx1 = getSbtIdxFromSbtMode( sbtInfo1 );
  uint8_t sbtIdx2 = getSbtIdxFromSbtMode( sbtInfo2 );
  if( sbtIdx1 == SBT_HOR_HALF || sbtIdx1 == SBT_VER_HALF )
  {
    return sbtIdx2 == SBT_HOR_HALF || sbtIdx2 == SBT_VER_HALF;
  }
  else if( sbtIdx1 == SBT_HOR_QUAD || sbtIdx1 == SBT_VER_QUAD )
  {
    return sbtIdx2 == SBT_HOR_QUAD || sbtIdx2 == SBT_VER_QUAD;
  }
  else
  {
    return false;
  }
}

bool CU::isPredRegDiffFromTB(const CodingUnit &cu, const ComponentID compID)
{
  return (compID == COMPONENT_Y)
         && (cu.ispMode == VER_INTRA_SUBPARTITIONS
             && CU::isMinWidthPredEnabledForBlkSize(cu.blocks[compID].width, cu.blocks[compID].height));
}

bool CU::isMinWidthPredEnabledForBlkSize(const int w, const int h)
{
  return ((w == 8 && h > 4) || w == 4);
}

bool CU::isFirstTBInPredReg(const CodingUnit& cu, const ComponentID compID, const CompArea &area)
{
  return (compID == COMPONENT_Y) && cu.ispMode && ((area.topLeft().x - cu.Y().topLeft().x) % PRED_REG_MIN_WIDTH == 0);
}

void CU::adjustPredArea(CompArea &area)
{
  area.width = std::max<int>(PRED_REG_MIN_WIDTH, area.width);
}

#if MULTI_HYP_PRED
AMVPInfo PU::getMultiHypMVPCandsMerge(PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx)
{
  CHECK(!pu.mergeFlag, "this function is only for merge");

  AMVPInfo amvpInfo;
  PU::fillMvpCand(pu, eRefPicList, refIdx, amvpInfo);

  return amvpInfo;
}

AMVPInfo PU::getMultiHypMVPCandsAMVP(PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx)
{
  CHECK(pu.mergeFlag, "this function is only for AMVP");
  CHECK(pu.interDir != 3, "Multi-Hyp is only allowed for bi-predictive AMVP mode")

  const int refListForAMVP = eRefPicList;
  const int refIdxForAMVP = refIdx;

  AMVPInfo amvpInfo;
  if (pu.cu->affine) // TODO: check performance impact
  {
    AffineAMVPInfo affineAMVPInfo;
    PU::fillAffineMvpCand(pu, RefPicList(refListForAMVP), refIdxForAMVP, affineAMVPInfo);
    amvpInfo.numCand = affineAMVPInfo.numCand;
    memcpy(amvpInfo.mvCand, affineAMVPInfo.mvCandLT, sizeof(amvpInfo.mvCand));
  }
  else
  {
    PU::fillMvpCand(pu, RefPicList(refListForAMVP), refIdxForAMVP, amvpInfo);
  }

  return amvpInfo;
}

AMVPInfo PU::getMultiHypMVPCands(PredictionUnit &pu, const MultiHypPredictionData &mhData)
{
  const auto &mhDataForAMVPList = !pu.mergeFlag || pu.addHypData.size() == pu.numMergedAddHyps ? mhData : pu.addHypData[pu.numMergedAddHyps];
  const auto mhRefIdxForAMVPList = mhDataForAMVPList.refIdx;
  const auto &MHRefPics = pu.cs->slice->getMultiHypRefPicList();
  CHECK(MHRefPics.empty(), "Multi Hyp: MHRefPics.empty()");
  CHECK(mhRefIdxForAMVPList >= MHRefPics.size(), "Multi Hyp: mhRefIdxForAMVPList >= MHRefPics.size()");
  const auto eRefPicList = RefPicList(MHRefPics[mhRefIdxForAMVPList].refList);
  const int iRefIdx = MHRefPics[mhRefIdxForAMVPList].refIdx;

  return (pu.mergeFlag ? PU::getMultiHypMVPCandsMerge : PU::getMultiHypMVPCandsAMVP)(pu, eRefPicList, iRefIdx);
}

Mv PU::getMultiHypMVP(PredictionUnit &pu, const MultiHypPredictionData &mhData)
{
  const auto amvpInfo = getMultiHypMVPCands(pu, mhData);
  CHECK(mhData.mvpIdx < 0, "Multi Hyp: mhData.mvpIdx < 0");
  CHECK(mhData.mvpIdx >= amvpInfo.numCand, "Multi Hyp: mhData.mvpIdx >= amvpInfo.numCand");
  const Mv mvp = amvpInfo.mvCand[mhData.mvpIdx];
  return mvp;
}
#endif

bool CU::isBcwIdxCoded( const CodingUnit &cu )
{
  if( cu.cs->sps->getUseBcw() == false )
  {
    CHECK(cu.BcwIdx != BCW_DEFAULT, "Error: cu.BcwIdx != BCW_DEFAULT");
    return false;
  }

  if (cu.predMode == MODE_IBC)
  {
    return false;
  }

  if( cu.predMode == MODE_INTRA || cu.cs->slice->isInterP() )
  {
    return false;
  }

  if( cu.lwidth() * cu.lheight() < BCW_SIZE_CONSTRAINT )
  {
    return false;
  }

  if( !cu.firstPU->mergeFlag )
  {
    if( cu.firstPU->interDir == 3 )
    {
      const int refIdx0 = cu.firstPU->refIdx[REF_PIC_LIST_0];
      const int refIdx1 = cu.firstPU->refIdx[REF_PIC_LIST_1];

      const WPScalingParam *wp0 = cu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
      const WPScalingParam *wp1 = cu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

      return !(WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1));
    }
  }

  return false;
}

uint8_t CU::getValidBcwIdx( const CodingUnit &cu )
{
  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    return cu.BcwIdx;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    CHECK(cu.BcwIdx != BCW_DEFAULT, " cu.BcwIdx != BCW_DEFAULT ");
  }
  else
  {
    CHECK(cu.BcwIdx != BCW_DEFAULT, " cu.BcwIdx != BCW_DEFAULT ");
  }

  return BCW_DEFAULT;
}

void CU::setBcwIdx( CodingUnit &cu, uint8_t uh )
{
  int8_t uhCnt = 0;

  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    cu.BcwIdx = uh;
    ++uhCnt;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    cu.BcwIdx = BCW_DEFAULT;
  }
  else
  {
    cu.BcwIdx = BCW_DEFAULT;
  }

  CHECK(uhCnt <= 0, " uhCnt <= 0 ");
}

uint8_t CU::deriveBcwIdx( uint8_t bcwLO, uint8_t bcwL1 )
{
  if( bcwLO == bcwL1 )
  {
    return bcwLO;
  }
  const int8_t w0 = getBcwWeight(bcwLO, REF_PIC_LIST_0);
  const int8_t w1 = getBcwWeight(bcwL1, REF_PIC_LIST_1);
  const int8_t th = g_BcwWeightBase >> 1;
  const int8_t off = 1;

  if( w0 == w1 || (w0 < (th - off) && w1 < (th - off)) || (w0 >(th + off) && w1 >(th + off)) )
  {
    return BCW_DEFAULT;
  }
  else
  {
    if( w0 > w1 )
    {
      return ( w0 >= th ? bcwLO : bcwL1 );
    }
    else
    {
      return ( w1 >= th ? bcwL1 : bcwLO );
    }
  }
}

bool CU::bdpcmAllowed( const CodingUnit& cu, const ComponentID compID )
{
  SizeType transformSkipMaxSize = 1 << cu.cs->sps->getLog2MaxTransformSkipBlockSize();

  bool bdpcmAllowed = cu.cs->sps->getBDPCMEnabledFlag();
       bdpcmAllowed &= CU::isIntra( cu );
       if (isLuma(compID))
       {
         bdpcmAllowed &= (cu.lwidth() <= transformSkipMaxSize && cu.lheight() <= transformSkipMaxSize);
#if ENABLE_DIMD
         bdpcmAllowed &= !cu.dimd;
#endif
       }
       else
       {
         bdpcmAllowed &=
           (cu.chromaSize().width <= transformSkipMaxSize && cu.chromaSize().height <= transformSkipMaxSize)
           && !cu.colorTransform;
       }
  return bdpcmAllowed;
}

bool CU::isMTSAllowed(const CodingUnit &cu, const ComponentID compID)
{
  SizeType tsMaxSize = 1 << cu.cs->sps->getLog2MaxTransformSkipBlockSize();
  const int maxSize  = CU::isIntra( cu ) ? MTS_INTRA_MAX_CU_SIZE : MTS_INTER_MAX_CU_SIZE;
  const int cuWidth  = cu.blocks[0].lumaSize().width;
  const int cuHeight = cu.blocks[0].lumaSize().height;
  bool mtsAllowed    = cu.chType == CHANNEL_TYPE_LUMA && compID == COMPONENT_Y;

  mtsAllowed &= CU::isIntra( cu ) ? cu.cs->sps->getUseIntraMTS() : cu.cs->sps->getUseInterMTS() && CU::isInter( cu );
  mtsAllowed &= cuWidth <= maxSize && cuHeight <= maxSize;
  mtsAllowed &= !cu.ispMode;
  mtsAllowed &= !cu.sbtInfo;
#if JVET_V0130_INTRA_TMP
  mtsAllowed &= !cu.tmpFlag;
#endif
#if JVET_W0123_TIMD_FUSION
  mtsAllowed &= !(cu.timd && cu.firstPU->multiRefIdx);
#endif
  mtsAllowed &= !(cu.bdpcmMode && cuWidth <= tsMaxSize && cuHeight <= tsMaxSize);
  return mtsAllowed;
}

#if ENABLE_OBMC
unsigned int PU::getSameNeigMotion(PredictionUnit &pu, MotionInfo& mi, Position off, int iDir, int& iLength, int iMaxLength)
{

  PredictionUnit* tmpPu = nullptr;
  Position posNeighborMotion = Position(0, 0);
  const Position  posSubBlock(pu.lumaPos().offset(off));

  const unsigned int  uiMinCUW = pu.cs->pcv->minCUWidth;
  if (iDir == 0)
  {
    posNeighborMotion = posSubBlock.offset(0, -1);
  }
  else if (iDir == 1)
  {
    posNeighborMotion = posSubBlock.offset(-1, 0);
  }
  else
  {
    THROW("iDir not supported");
  }
  tmpPu = pu.cs->getPU(posNeighborMotion, pu.chType);

  //if neighbor pu is not available, return.
  if (!tmpPu)
  {
    return 0;
  }

  mi = tmpPu->getMotionInfo(posNeighborMotion);

  iLength = 1;
  if (mi.isInter && !mi.isIBCmot) // inter
  {
    MotionInfo currMi = pu.getMotionInfo(posSubBlock);
    // check similar MV

    bool bIsSimilarMV = PU::identicalMvOBMC(currMi, mi, pu.cs->slice->getCheckLDC());
    for (unsigned int idx = 1; idx < iMaxLength; idx++)
    {

      if (iDir == 0)//top
      {
        posNeighborMotion = posSubBlock.offset(idx*uiMinCUW, -1);
      }
      else//left
      {
        posNeighborMotion = posSubBlock.offset(-1, idx*uiMinCUW);
      }
      tmpPu = pu.cs->getPU(posNeighborMotion, pu.chType);

      MotionInfo miNeigh = tmpPu->getMotionInfo(posNeighborMotion);

      bool       bSameMv = true;

      if (mi.interDir != miNeigh.interDir || miNeigh.isIBCmot)
      {
        bSameMv = false;
      }
      else
      {
        for (unsigned int iRefList = 0; iRefList < 2; iRefList++)
        {
          if (miNeigh.interDir & (1 << iRefList))
          {
            if (!(miNeigh.mv[iRefList] == mi.mv[iRefList] && miNeigh.refIdx[iRefList] == mi.refIdx[iRefList]))
            {
              bSameMv = false; break;
            }
          }
        }
      }
      if (bSameMv)
      {
        iLength++;
      }
      else
      {
        break;
      }
    }
    return 2 + bIsSimilarMV; // 2: not skip; 3: skip
  }
  else             // intra
  {
    for (unsigned int idx = 1; idx < iMaxLength; idx++)
    {
      if (iDir == 0)
      {
        posNeighborMotion = posSubBlock.offset(idx*uiMinCUW, -1);
      }
      else
      {
        posNeighborMotion = posSubBlock.offset(-1, idx*uiMinCUW);
      }
      tmpPu = pu.cs->getPU(posNeighborMotion, pu.chType);

      if (!tmpPu->getMotionInfo(posNeighborMotion).isInter || tmpPu->getMotionInfo(posNeighborMotion).isIBCmot)
      {
        iLength++;
      }
      else
      {
        break;
      }
    }
    return 1;
  }
}

bool PU::identicalMvOBMC(MotionInfo curMI, MotionInfo neighMI, bool bLD)
{
  if (curMI.interDir != neighMI.interDir)
  {
    return false;
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      if (curMI.refIdx[i] != -1)
      {
        if (!(curMI.mv[i] == neighMI.mv[i] && curMI.refIdx[i] == neighMI.refIdx[i]))
        {
          return false;
        }
      }
    }
    return true;
  }
}

bool PU::getNeighborMotion(PredictionUnit &pu, MotionInfo& mi, Position off, Size unitSize, int iDir)
{
  PredictionUnit* tmpPu = &pu;
  Position posNeighborMotion = Position(0, 0);
  const Position posSubBlock(pu.lumaPos().offset(off));


  if (iDir == 0) //above
  {
    posNeighborMotion = posSubBlock.offset(0, -1);
  }
  else if (iDir == 1) //left
  {
    posNeighborMotion = posSubBlock.offset(-1, 0);
  }
  else if (iDir == 2) //below
  {
    posNeighborMotion = posSubBlock.offset(0, unitSize.height);
    CHECK(pu.cu != tmpPu->cu, "Got a PU from a different CU when fetching a below PU");
  }
  else if (iDir == 3) //right
  {
    posNeighborMotion = posSubBlock.offset(unitSize.width, 0);
    CHECK(pu.cu != tmpPu->cu, "Got a PU from a different CU when fetching a right PU");
  }

  const bool bNoAdjacentMotion = !tmpPu || CU::isIntra(*tmpPu->cu);

  if (bNoAdjacentMotion)
  {
    return false;
  }

  mi = tmpPu->getMotionInfo(posNeighborMotion);
  const MotionInfo currMotion = pu.getMotionInfo(posSubBlock);

  if (mi.interDir)
  {
    if (mi.interDir != currMotion.interDir)
    {
      return true;
    }
    else
    {
      for (uint32_t iRefList = 0; iRefList < 2; iRefList++)
      {
        if (currMotion.interDir & (1 << iRefList))
        {
          Mv mvd = currMotion.mv[iRefList] - mi.mv[iRefList];

          if (!(mvd.getAbsHor() < 1 && mvd.getAbsVer() < 1 && currMotion.refIdx[iRefList] == mi.refIdx[iRefList]))
          {
            return true;
          }
        }
      }
      return false;
    }
  }
  else
  {
    return false;
  }
}
#endif

// TU tools

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
  return getCbfAtDepth( tu, compID, tu.depth );
}

bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( !tu.blocks[compID].valid() )
  {
    CHECK(tu.cbf[compID] != 0, "cbf must be 0 if the component is not available");
  }
#endif
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}

bool TU::isTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  const int maxSize = tu.cs->sps->getLog2MaxTransformSkipBlockSize();

  bool tsAllowed = tu.cs->sps->getTransformSkipEnabledFlag();
  tsAllowed &= ( !tu.cu->ispMode || !isLuma(compID) );
  SizeType transformSkipMaxSize = 1 << maxSize;
  tsAllowed &= !(tu.cu->bdpcmMode && isLuma(compID));
  tsAllowed &= !(tu.cu->bdpcmModeChroma && isChroma(compID));
  tsAllowed &= tu.blocks[compID].width <= transformSkipMaxSize && tu.blocks[compID].height <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo;

  return tsAllowed;
}


int TU::getICTMode( const TransformUnit& tu, int jointCbCr )
{
  if( jointCbCr < 0 )
  {
    jointCbCr = tu.jointCbCr;
  }
  return g_ictModes[ tu.cs->picHeader->getJointCbCrSignFlag() ][ jointCbCr ];
}


bool TU::needsSqrt2Scale( const TransformUnit &tu, const ComponentID &compID )
{
  const Size &size=tu.blocks[compID];
  const bool isTransformSkip = (tu.mtsIdx[compID] == MTS_SKIP);
  return (!isTransformSkip) && (((floorLog2(size.width) + floorLog2(size.height)) & 1) == 1);
}

bool TU::needsBlockSizeTrafoScale( const TransformUnit &tu, const ComponentID &compID )
{
  return needsSqrt2Scale( tu, compID ) || isNonLog2BlockSize( tu.blocks[compID] );
}

TransformUnit* TU::getPrevTU( const TransformUnit &tu, const ComponentID compID )
{
  TransformUnit* prevTU = tu.prev;

  if( prevTU != nullptr && ( prevTU->cu != tu.cu || !prevTU->blocks[compID].valid() ) )
  {
    prevTU = nullptr;
  }

  return prevTU;
}

bool TU::getPrevTuCbfAtDepth( const TransformUnit &currentTu, const ComponentID compID, const int trDepth )
{
  const TransformUnit* prevTU = getPrevTU( currentTu, compID );
  return ( prevTU != nullptr ) ? TU::getCbfAtDepth( *prevTU, compID, trDepth ) : false;
}


// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}

int getNumModesMip(const Size& block)
{
  switch( getMipSizeId(block) )
  {
  case 0: return 16;
  case 1: return  8;
  case 2: return  6;
  default: THROW( "Invalid mipSizeId" );
  }
}


int getMipSizeId(const Size& block)
{
  if( block.width == 4 && block.height == 4 )
  {
    return 0;
  }
  else if( block.width == 4 || block.height == 4 || (block.width == 8 && block.height == 8) )
  {
    return 1;
  }
  else
  {
    return 2;
  }

}

bool allowLfnstWithMip(const Size& block)
{
  if (block.width >= 16 && block.height >= 16)
  {
    return true;
  }
  return false;
}
#if JVET_V0130_INTRA_TMP
bool allowLfnstWithTmp()
{
	return true;
}
#endif
#if INTER_LIC
bool CU::isLICFlagPresent(const CodingUnit& cu)
{
  if (CU::isIntra(cu) || !cu.slice->getUseLIC())
  {
    return false;
  }

  if (cu.geoFlag || cu.firstPU->mergeFlag || cu.firstPU->interDir == 3 || CU::isIBC(cu) || cu.Y().area() < LIC_MIN_CU_PIXELS)
  {
    return false;
  }

  return true;
}

void PU::spanLICFlags(PredictionUnit &pu, const bool LICFlag)
{
  MotionBuf mb = pu.getMotionBuf();
  MotionInfo *motionInfo = mb.buf;

  for (int y = 0; y < mb.height; y++)
  {
    for (int x = 0; x < mb.width; x++)
    {
      motionInfo[x].usesLIC = LICFlag;
    }
    motionInfo += mb.stride;
  }
}
#endif

#if SIGN_PREDICTION
bool TU::getDelayedSignCoding( const TransformUnit &tu, const ComponentID compID )
{
  const uint32_t maxSize = CU::isIntra( *tu.cu ) ? SIGN_PRED_MAX_BS_INTRA : SIGN_PRED_MAX_BS_INTER;

  const uint32_t width = tu.blocks[compID].width;
  const uint32_t height = tu.blocks[compID].height;
  if( tu.cs->sps->getNumPredSigns() <= 0 || tu.mtsIdx[compID] == MTS_SKIP )
  {
    return false;
  }

  if( width < 4 || height < 4 || width > maxSize || height > maxSize )
  {
    return false;
  }
  return true;
}

bool TU::getUseSignPred( const TransformUnit &tu, const ComponentID compID )
{
  return TU::getDelayedSignCoding( tu, compID ) && !tu.cu->lfnstIdx;
}

void TU::predBorderResi( const Position blkPos, const CPelBuf &recoBuf, const CPelBuf &predBuf, const ComponentID compID,
  const uint32_t uiWidth, const uint32_t uiHeight, Pel *predResiBorder, const Pel defaultPel )
{
  Pel r1, r2, p0;

  const Pel *pReco1, *pReco2, *pPred0;

  const Pel *pReco = recoBuf.buf;
  const Pel *pPred = predBuf.buf;
  uint32_t strideReco = recoBuf.stride;
  uint32_t stridePred = predBuf.stride;


  if( blkPos.y )
  {
    pReco1 = pReco - strideReco;
    pReco2 = pReco - 2 * strideReco;
    pPred0 = pPred;
    for( int32_t x = 0; x < uiWidth; x++ )
    {
      p0 = pPred0[x];
      r1 = pReco1[x];
      r2 = pReco2[x];
      predResiBorder[uiHeight + x] = ( ( r1 << 1 ) - r2 - p0 );
    }
  }
  else
  {
    for( int32_t x = 0; x < uiWidth; x++ )
    {
      predResiBorder[uiHeight + x] = defaultPel - pPred[x];
    }
  }

  if( blkPos.x )
  {
    pReco1 = pReco - 1;
    pReco2 = pReco - 2;
    pPred0 = pPred;
    for( int32_t y = 0; y < uiHeight; y++ )
    {
      p0 = pPred0[0];
      r1 = pReco1[0];
      r2 = pReco2[0];
      predResiBorder[uiHeight - 1 - y] = ( ( r1 << 1 ) - r2 - p0 );
      pReco1 += strideReco;
      pReco2 += strideReco;
      pPred0 += stridePred;
    }
  }
  else
  {
    for( int32_t y = 0; y < uiHeight; y++ )
    {
      predResiBorder[uiHeight - 1 - y] = defaultPel - pPred[0];
      pPred += stridePred;
    }
  }

}

Position TU::posSignHidingFirstCG( const TransformUnit &tu, ComponentID compID )
{
  Position pos( -1, -1 );
  int width = tu.blocks[compID].width;
  int height = tu.blocks[compID].height;
  if( tu.cu->cs->slice->getSignDataHidingEnabledFlag() && width >= 4 && height >= 4 )
  {
    // Mayte use 4x3 size is enough
    const ScanElement * scan = g_scanOrder[SCAN_GROUPED_4x4][SCAN_DIAG][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )];
    const int cgSize = 16;
    int   lastNZPosInCG = -1, firstNZPosInCG = cgSize;
    Position posHiding( -1, -1 );
    const CCoeffBuf &buf = tu.getCoeffs( compID );

    for( auto n = cgSize - 1; n >= 0; --n )
    {
      ScanElement currSE = scan[n];
      Position    currPos = Position( currSE.x, currSE.y );

      if( buf.at( currPos ) )
      {
        lastNZPosInCG = n;
        posHiding = currPos;
        break;
      }
    }

    if( lastNZPosInCG != -1 )
    {
      for( auto n = 0; n < cgSize; n++ )
      {
        ScanElement currSE = scan[n];
        Position    currPos = Position( currSE.x, currSE.y );
        if( buf.at( currPos ) )
        {
          firstNZPosInCG = n;
          break;
        }
      }

    }
    if( firstNZPosInCG - lastNZPosInCG >= SBH_THRESHOLD )
    {
      return posHiding;
    }
  }
  return Position( -1, -1 );
}
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
bool PU::checkIsValidMergeMvCand(const CodingStructure &cs, const PredictionUnit &pu, const int curPoc, const int amvpPoc, int8_t mergeRefIdx[ NUM_REF_PIC_LIST_01 ])
{
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0])
  {
    if (mergeRefIdx[REF_PIC_LIST_0] < 0)
    {
      return false;
    }
    const int mergePoc = cs.slice->getRefPOC(REF_PIC_LIST_0, mergeRefIdx[REF_PIC_LIST_0]);
    if ((amvpPoc - curPoc) * (mergePoc -curPoc) > 0)
    {
      return false;
    }
  }
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    if (mergeRefIdx[REF_PIC_LIST_1] < 0)
    {
      return false;
    }
    const int mergePoc = cs.slice->getRefPOC(REF_PIC_LIST_1, mergeRefIdx[REF_PIC_LIST_1]);
    if ((amvpPoc - curPoc) * (mergePoc -curPoc) > 0)
    {
      return false;
    }
  }
  return true;
}
#endif
