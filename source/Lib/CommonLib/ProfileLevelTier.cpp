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

/** \file     ProfileLevelTier.cpp
    \brief    Handle profile, level and tier information.
*/


#include "ProfileLevelTier.h"
#include "CommonLib/Slice.h"
#include <math.h>

uint32_t
LevelTierFeatures::getMaxPicWidthInLumaSamples()  const
{
  return uint32_t(sqrt(maxLumaPs*8.0));
}

uint32_t
LevelTierFeatures::getMaxPicHeightInLumaSamples() const
{
  return uint32_t(sqrt(maxLumaPs*8.0));
}

static const uint64_t MAX_CNFUINT64 = std::numeric_limits<uint64_t>::max();

static const LevelTierFeatures mainLevelTierInfo[] =
{
#if JVET_S0156_LEVEL_DEFINITION
      //  level,       maxlumaps,      maxcpb[tier],,  maxSlicesPerAu,maxTilesPerAu,cols, maxLumaSr,       maxBr[tier],,    minCr[tier],,
    { Level::LEVEL1  ,    36864, {      350,        0 },       16,        1,        1,     552960ULL, {     128,        0 }, { 2, 2} },
    { Level::LEVEL2  ,   122880, {     1500,        0 },       16,        1,        1,    3686400ULL, {    1500,        0 }, { 2, 2} },
    { Level::LEVEL2_1,   245760, {     3000,        0 },       20,        1,        1,    7372800ULL, {    3000,        0 }, { 2, 2} },
    { Level::LEVEL3  ,   552960, {     6000,        0 },       30,        4,        2,   16588800ULL, {    6000,        0 }, { 2, 2} },
    { Level::LEVEL3_1,   983040, {    10000,        0 },       40,        9,        3,   33177600ULL, {   10000,        0 }, { 2, 2} },
    { Level::LEVEL4  ,  2228224, {    12000,    30000 },       75,       25,        5,   66846720ULL, {   12000,    30000 }, { 4, 4} },
    { Level::LEVEL4_1,  2228224, {    20000,    50000 },       75,       25,        5,  133693440ULL, {   20000,    50000 }, { 4, 4} },
    { Level::LEVEL5  ,  8912896, {    25000,   100000 },      200,      110,       10,  267386880ULL, {   25000,   100000 }, { 6, 4} },
    { Level::LEVEL5_1,  8912896, {    40000,   160000 },      200,      110,       10,  534773760ULL, {   40000,   160000 }, { 8, 4} },
    { Level::LEVEL5_2,  8912896, {    60000,   240000 },      200,      110,       10, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { Level::LEVEL6  , 35651584, {    80000,   240000 },      600,      440,       20, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { Level::LEVEL6_1, 35651584, {   120000,   480000 },      600,      440,       20, 2139095040ULL, {  120000,   480000 }, { 8, 4} },
    { Level::LEVEL6_2, 35651584, {   180000,   800000 },      600,      440,       20, 4278190080ULL, {  240000,   800000 }, { 8, 4} },
    { Level::LEVEL15_5, MAX_UINT,{ MAX_UINT, MAX_UINT }, MAX_UINT, MAX_UINT, MAX_UINT, MAX_CNFUINT64, {MAX_UINT, MAX_UINT }, { 0, 0} },
    { Level::NONE    }
#else
    //  level        , maxlumaps,      maxcpb[tier],,    maxSlice,    tile rows, cols,     maxLumaSr,       maxBr[tier],,  , minCr[tier],,
    { Level::LEVEL1  ,    36864, {      350,        0 },       16,        1,        1,     552960ULL, {     128,        0 }, { 2, 2} },
    { Level::LEVEL2  ,   122880, {     1500,        0 },       16,        1,        1,    3686400ULL, {    1500,        0 }, { 2, 2} },
    { Level::LEVEL2_1,   245760, {     3000,        0 },       20,        1,        1,    7372800ULL, {    3000,        0 }, { 2, 2} },
    { Level::LEVEL3  ,   552960, {     6000,        0 },       30,        2,        2,   16588800ULL, {    6000,        0 }, { 2, 2} },
    { Level::LEVEL3_1,   983040, {    10000,        0 },       40,        3,        3,   33177600ULL, {   10000,        0 }, { 2, 2} },
    { Level::LEVEL4  ,  2228224, {    12000,    30000 },       75,        5,        5,   66846720ULL, {   12000,    30000 }, { 4, 4} },
    { Level::LEVEL4_1,  2228224, {    20000,    50000 },       75,        5,        5,  133693440ULL, {   20000,    50000 }, { 4, 4} },
    { Level::LEVEL5  ,  8912896, {    25000,   100000 },      200,       11,       10,  267386880ULL, {   25000,   100000 }, { 6, 4} },
    { Level::LEVEL5_1,  8912896, {    40000,   160000 },      200,       11,       10,  534773760ULL, {   40000,   160000 }, { 8, 4} },
    { Level::LEVEL5_2,  8912896, {    60000,   240000 },      200,       11,       10, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { Level::LEVEL6  , 35651584, {    80000,   240000 },      600,       22,       20, 1069547520ULL, {   60000,   240000 }, { 8, 4} },
    { Level::LEVEL6_1, 35651584, {   120000,   480000 },      600,       22,       20, 2139095040ULL, {  120000,   480000 }, { 8, 4} },
    { Level::LEVEL6_2, 35651584, {   180000,   800000 },      600,       22,       20, 4278190080ULL, {  240000,   800000 }, { 8, 4} },
    { Level::LEVEL15_5, MAX_UINT, { MAX_UINT, MAX_UINT }, MAX_UINT, MAX_UINT, MAX_UINT, MAX_CNFUINT64, {MAX_UINT, MAX_UINT }, { 0, 0} },
    { Level::NONE    }
#endif
};

static const ProfileFeatures validProfiles[] = {
// profile, pNameString, maxBitDepth, maxChrFmt, lvl15.5, cpbvcl, cpbnal, fcf*1000, mincr*100, levelInfo
// most constrained profiles must appear first.
#if JVET_S_PROFILES
  { Profile::MAIN_10_STILL_PICTURE, "Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100, 1875, 100,
    mainLevelTierInfo, true },
  { Profile::MULTILAYER_MAIN_10_STILL_PICTURE, "Multilayer_Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100,
    1875, 100, mainLevelTierInfo, true },
  { Profile::MAIN_10_444_STILL_PICTURE, "Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500, 2750, 3750, 75,
    mainLevelTierInfo, true },
  { Profile::MULTILAYER_MAIN_10_444_STILL_PICTURE, "Multilayer_Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500,
    2750, 3750, 75, mainLevelTierInfo, true },
  { Profile::MAIN_10, "Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo, false },
  { Profile::MULTILAYER_MAIN_10, "Multilayer_Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo,
    false },
  { Profile::MAIN_10_444, "Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75, mainLevelTierInfo, false },
  { Profile::MULTILAYER_MAIN_10_444, "Multilayer_Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75,
    mainLevelTierInfo, false },
#else
  { Profile::MAIN_10, "Main_10_Still_Picture", 10, CHROMA_420, true, 1000, 1100, 1875, 100, mainLevelTierInfo, true },
  { Profile::MAIN_444_10, "Main_444_10_Still_Picture", 10, CHROMA_444, true, 2500, 2750, 3750, 75, mainLevelTierInfo,
    true },
  { Profile::MAIN_10, "Main_10", 10, CHROMA_420, false, 1000, 1100, 1875, 100, mainLevelTierInfo, false },
  { Profile::MAIN_444_10, "Main_444_10", 10, CHROMA_444, false, 2500, 2750, 3750, 75, mainLevelTierInfo, false },
#endif
  { Profile::NONE, 0 },
};

#if JVET_S_PROFILES
const ProfileFeatures *ProfileFeatures::getProfileFeatures(const Profile::Name p)
{
  int i;
  for (i = 0; validProfiles[i].profile != Profile::NONE; i++)
  {
    if (validProfiles[i].profile == p)
    {
      return &validProfiles[i];
    }
  }

  return &validProfiles[i];
}
#endif

void
ProfileLevelTierFeatures::extractPTLInformation(const SPS &sps)
{
  const ProfileTierLevel &spsPtl =*(sps.getProfileTierLevel());

  m_pProfile = nullptr;
  m_pLevelTier = nullptr;
  m_tier = spsPtl.getTierFlag();

  // Identify the profile from the profile Idc, and possibly other constraints.
#if !JVET_S_PROFILES
  bool onePictureOnlyConstraintFlag=spsPtl.getConstraintInfo()->getOnePictureOnlyConstraintFlag();
#endif
  for(int32_t i=0; validProfiles[i].profile != Profile::NONE; i++)
  {
#if JVET_S_PROFILES
    if (spsPtl.getProfileIdc() == validProfiles[i].profile)
#else
    if (spsPtl.getProfileIdc() == validProfiles[i].profile && !(validProfiles[i].onePictureOnlyFlagMustBe1 && !onePictureOnlyConstraintFlag))
#endif
    {
      m_pProfile = &(validProfiles[i]);
      break;
    }
  }

  if (m_pProfile != nullptr)
  {
    // Now identify the level:
    const LevelTierFeatures *pLTF = m_pProfile->pLevelTiersListInfo;
    const Level::Name spsLevelName = spsPtl.getLevelIdc();
    if (spsLevelName!=Level::LEVEL15_5 || m_pProfile->canUseLevel15p5)
    {
      for(int i=0; pLTF[i].level!=Level::NONE; i++)
      {
        if (pLTF[i].level == spsLevelName)
        {
          m_pLevelTier = &(pLTF[i]);
        }
      }
    }
  }
}

double ProfileLevelTierFeatures::getMinCr() const
{
  return (m_pLevelTier!=0 && m_pProfile!=0) ? (m_pProfile->minCrScaleFactorx100 * m_pLevelTier->minCrBase[m_tier?1:0])/100.0 : 0.0 ;
}

uint64_t ProfileLevelTierFeatures::getCpbSizeInBits() const
{
  return (m_pLevelTier!=0 && m_pProfile!=0) ? uint64_t(m_pProfile->cpbVclFactor) * m_pLevelTier->maxCpb[m_tier?1:0] : uint64_t(0);
}

uint32_t ProfileLevelTierFeatures::getMaxDpbSize( uint32_t picSizeMaxInSamplesY ) const
{
  const uint32_t maxDpbPicBuf = 8;
  uint32_t       maxDpbSize;

  if (m_pLevelTier->level == Level::LEVEL15_5)
  {
    // maxDpbSize is unconstrained in this case
    maxDpbSize = std::numeric_limits<uint32_t>::max();
  }
  else if (2 * picSizeMaxInSamplesY <= m_pLevelTier->maxLumaPs)
  {
    maxDpbSize = 2 * maxDpbPicBuf;
  }
  else if (3 * picSizeMaxInSamplesY <= 2 * m_pLevelTier->maxLumaPs)
  {
    maxDpbSize = 3 * maxDpbPicBuf / 2;
  }
  else
  {
    maxDpbSize = maxDpbPicBuf;
  }

  return maxDpbSize;
}
