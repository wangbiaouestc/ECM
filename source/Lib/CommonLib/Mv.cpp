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

/** \file     Mv.cpp
    \brief    motion vector class
*/

#include "Mv.h"

#include "Common.h"
#include "Slice.h"

const MvPrecision Mv::m_amvrPrecision[4] = { MV_PRECISION_QUARTER, MV_PRECISION_INT, MV_PRECISION_4PEL, MV_PRECISION_HALF }; // for cu.imv=0, 1, 2 and 3
const MvPrecision Mv::m_amvrPrecAffine[3] = { MV_PRECISION_QUARTER, MV_PRECISION_SIXTEENTH, MV_PRECISION_INT }; // for cu.imv=0, 1 and 2
const MvPrecision Mv::m_amvrPrecIbc[3] = { MV_PRECISION_INT, MV_PRECISION_INT, MV_PRECISION_4PEL }; // for cu.imv=0, 1 and 2

void roundAffineMv( int& mvx, int& mvy, int nShift )
{
  const int nOffset = 1 << (nShift - 1);
  mvx = (mvx + nOffset - (mvx >= 0)) >> nShift;
  mvy = (mvy + nOffset - (mvy >= 0)) >> nShift;
}

void (*clipMv) ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps );

void clipMvInPic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps )
{
  if (sps.getWrapAroundEnabledFlag())
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps);
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
#if JVET_AA0096_MC_BOUNDARY_PADDING
  offset += MC_PAD_SIZE;
#endif
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;

  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

void clipMvInSubpic ( Mv& rcMv, const struct Position& pos, const struct Size& size, const class SPS& sps, const class PPS& pps )
{
  if (sps.getWrapAroundEnabledFlag())
  {
    wrapClipMv(rcMv, pos, size, &sps, &pps);
    return;
  }

  int mvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int offset = 8;
  int horMax = (pps.getPicWidthInLumaSamples() + offset - (int)pos.x - 1) << mvShift;
  int horMin = (-(int)sps.getMaxCUWidth() - offset - (int)pos.x + 1) << mvShift;

  int verMax = (pps.getPicHeightInLumaSamples() + offset - (int)pos.y - 1) << mvShift;
  int verMin = (-(int)sps.getMaxCUHeight() - offset - (int)pos.y + 1) << mvShift;
  const SubPic& curSubPic = pps.getSubPicFromPos(pos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = ((curSubPic.getSubPicRight() + 1) + offset - (int)pos.x - 1) << mvShift;
    horMin = (-(int)sps.getMaxCUWidth() - offset - ((int)pos.x - curSubPic.getSubPicLeft()) + 1) << mvShift;

    verMax = ((curSubPic.getSubPicBottom() + 1) + offset - (int)pos.y - 1) << mvShift;
    verMin = (-(int)sps.getMaxCUHeight() - offset - ((int)pos.y - curSubPic.getSubPicTop()) + 1) << mvShift;
  }
  rcMv.setHor(std::min(horMax, std::max(horMin, rcMv.getHor())));
  rcMv.setVer(std::min(verMax, std::max(verMin, rcMv.getVer())));
}

bool wrapClipMv( Mv& rcMv, const Position& pos, const struct Size& size, const SPS *sps, const PPS *pps )
{
  bool wrapRef = true;
  int iMvShift = MV_FRACTIONAL_BITS_INTERNAL;
  int iOffset = 8;
  int iHorMax = ( pps->getPicWidthInLumaSamples() + sps->getMaxCUWidth() - size.width + iOffset - (int)pos.x - 1 ) << iMvShift;
  int iHorMin = ( -( int ) sps->getMaxCUWidth()                                      - iOffset - ( int ) pos.x + 1 ) << iMvShift;

  int iVerMax = ( pps->getPicHeightInLumaSamples() + iOffset - ( int ) pos.y - 1 ) << iMvShift;
  int iVerMin = ( -( int ) sps->getMaxCUHeight() - iOffset - ( int ) pos.y + 1 ) << iMvShift;

  const SubPic& curSubPic = pps->getSubPicFromPos( pos );
  if( curSubPic.getTreatedAsPicFlag() )
  {
    iVerMax = ( ( curSubPic.getSubPicBottom() + 1 ) + iOffset - ( int ) pos.y - 1 ) << iMvShift;
    iVerMin = ( -( int ) sps->getMaxCUHeight() - iOffset - ( ( int ) pos.y - curSubPic.getSubPicTop() ) + 1 ) << iMvShift;
  }
  int mvX = rcMv.getHor();

  if(mvX > iHorMax)
  {
    mvX -= ( pps->getWrapAroundOffset() << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }
  if(mvX < iHorMin)
  {
    mvX += ( pps->getWrapAroundOffset() << iMvShift );
    mvX = std::min( iHorMax, std::max( iHorMin, mvX ) );
    wrapRef = false;
  }

  rcMv.setHor( mvX );
  rcMv.setVer( std::min( iVerMax, std::max( iVerMin, rcMv.getVer() ) ) );
  return wrapRef;
}

#if JVET_AC0104_IBC_BVD_PREDICTION
void MvdSuffixInfo::initPrefixes(const Mv& mv, const int imv, const bool isInternalPrecision)
{
  Mv tmp = mv;
  if (isInternalPrecision)
  {
    tmp.changeTransPrecInternal2Amvr(imv);
  }

  int       horMvd = tmp.getHor();
  int       verMvd = tmp.getVer();

  unsigned  absHor = unsigned(horMvd < 0 ? -horMvd : horMvd);
  unsigned  absVer = unsigned(verMvd < 0 ? -verMvd : verMvd);
  const auto horRemainderNonZero = (absHor > 0);
  const auto verRemainderNonZero = (absVer > 0);

  horPrefix = -1;
  if (horRemainderNonZero)
  {
    horPrefix = getPrefixLength(absHor - 1 );
  }

  verPrefix = -1;
  if (verRemainderNonZero)
  {
    verPrefix = getPrefixLength(absVer - 1 );
  }

  initSuffixesAndSigns(mv, imv);
}


void MvdSuffixInfo::initSuffixesAndSigns(const Mv& mv, const int imv)
{
  if (horPrefix >= 0)
  {
    horPrefixGroupStartValue = xGetGolombGroupMinValue(horPrefix);
    iBinsInHorSuffix = horPrefix+1;
  }
  if (verPrefix >= 0)
  {
    verPrefixGroupStartValue = xGetGolombGroupMinValue(verPrefix);
    iBinsInVerSuffix = verPrefix + 1;
  }

  defineNumberOfPredictedBinsInSuffix(horPrefix, verPrefix, imv);
}

void MvdSuffixInfo::defineNumberOfPredictedBinsInSuffix(const int iHorPrefix, const int iVerPrefix, const uint8_t imv)
{
  const int iNumberOfMSBins = IBC_BVD_PREDICTION_MAX_BIN_NUM;
  CHECK(iNumberOfMSBins < 0, "Incorrect/negative value of bins to be predicted in suffixes of exp-Golomb code");



  const unsigned int uiExpGolombParam = BVD_CODING_GOLOMB_ORDER;

  const int iBinsInHorSuffix = iHorPrefix < 0 ? 0 : iHorPrefix + uiExpGolombParam;
  const int iBinsInVerSuffix = iVerPrefix < 0 ? 0 : iVerPrefix + uiExpGolombParam;

  const int iPrecShift = Mv::getImvPrecShift(imv);
  constexpr int iMaxNumberOfInsignLSBins = 0;

  const int iNumberOfInsignLSBins = std::max(0, iMaxNumberOfInsignLSBins - iPrecShift);
  const int iAvailBinsInHorSuffix = std::max(0, iBinsInHorSuffix - iNumberOfInsignLSBins);
  const int iAvailBinsInVerSuffix = std::max(0, iBinsInVerSuffix - iNumberOfInsignLSBins);
  const int iTotalNumberOfPredBins = std::min(iNumberOfMSBins, iAvailBinsInHorSuffix + iAvailBinsInVerSuffix);

  horEncodeSignInEP = false;
  verEncodeSignInEP = false;

  if (0 >= iTotalNumberOfPredBins)
  {
    horOffsetPredictionNumBins = 0;
    verOffsetPredictionNumBins = 0;
    return;
  }

  int iNumberOfHorMSBins = 0;
  int iNumberOfVerMSBins = 0;
  if (iHorPrefix > iVerPrefix)
  {
    if (iVerPrefix < 0)
    {
      iNumberOfHorMSBins = iTotalNumberOfPredBins;
    }
    else
    {
      const int iDiffHorVer = iAvailBinsInHorSuffix - iAvailBinsInVerSuffix;
      const int iNumberOfEqSignBins = iTotalNumberOfPredBins - iDiffHorVer;
      if (iNumberOfEqSignBins <= 0)
      {

        const bool magnitudeCheck = 2 * (verPrefixGroupStartValue + (1 << (iBinsInVerSuffix - 1))) < (1 << std::max(0, iBinsInHorSuffix - iTotalNumberOfPredBins - 1));
        if (iAvailBinsInHorSuffix > iTotalNumberOfPredBins && magnitudeCheck)
        {
          iNumberOfHorMSBins = iTotalNumberOfPredBins + 1; //1 bin instead of VER sign
          verEncodeSignInEP = true;
        }
        else
        {
          iNumberOfHorMSBins = iTotalNumberOfPredBins;
        }
      }
      else
      {
        iNumberOfVerMSBins = iNumberOfEqSignBins >> 1;
        CHECK(iAvailBinsInHorSuffix + iAvailBinsInVerSuffix == iTotalNumberOfPredBins && iNumberOfEqSignBins - (iNumberOfVerMSBins << 1) != 0, "iNumberOfEqSignBins is odd for iHorPrefix > iVerPrefix");
        iNumberOfHorMSBins = iNumberOfEqSignBins - iNumberOfVerMSBins + iDiffHorVer;
      }
    }
  }
  else if (iVerPrefix > iHorPrefix)
  {
    if (iHorPrefix < 0)
    {
      iNumberOfVerMSBins = iTotalNumberOfPredBins;
    }
    else
    {
      const int iDiffVerHor = iAvailBinsInVerSuffix - iAvailBinsInHorSuffix;
      const int iNumberOfEqSignBins = iTotalNumberOfPredBins - iDiffVerHor;
      if (iNumberOfEqSignBins <= 0)
      {
        const bool magnitudeCheck = 2 * (horPrefixGroupStartValue + (1 << (iBinsInHorSuffix - 1))) < (1 << std::max(0, iBinsInVerSuffix - iTotalNumberOfPredBins - 1));
        if (iAvailBinsInVerSuffix > iTotalNumberOfPredBins && magnitudeCheck)
        {
          iNumberOfVerMSBins = iTotalNumberOfPredBins + 1; //1 bin instead of HOR sign
          horEncodeSignInEP = true;
        }
        else
        {
          iNumberOfVerMSBins = iTotalNumberOfPredBins;
        }
      }
      else
      {
        iNumberOfHorMSBins = iNumberOfEqSignBins >> 1;
        CHECK(iAvailBinsInHorSuffix + iAvailBinsInVerSuffix == iTotalNumberOfPredBins && iNumberOfEqSignBins - (iNumberOfHorMSBins << 1) != 0, "iNumberOfEqSignBins is odd for iVerPrefix > iHorPrefix");
        iNumberOfVerMSBins = iNumberOfEqSignBins - iNumberOfHorMSBins + iDiffVerHor;
      }
    }
  }
  else
  {
    iNumberOfVerMSBins = std::min(iAvailBinsInVerSuffix, (iTotalNumberOfPredBins >> 1));
    iNumberOfHorMSBins = iTotalNumberOfPredBins - iNumberOfVerMSBins;
  };

  CHECK(iNumberOfHorMSBins < 0, "iNumberOfHorMSBins < 0");
  CHECK(iNumberOfVerMSBins < 0, "iNumberOfVerMSBins < 0");

  horOffsetPredictionNumBins = std::min(iAvailBinsInHorSuffix, iNumberOfHorMSBins);
  verOffsetPredictionNumBins = std::min(iAvailBinsInVerSuffix, iNumberOfVerMSBins);
}
#endif // JVET_AC0104_IBC_BVD_PREDICTION

//! \}
