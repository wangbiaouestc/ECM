/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2019, ITU/ISO/IEC
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

#pragma once

#include "../CommonLib/CommonDef.h"
#include <cstdint>
#include <cassert>
#include <algorithm>

#if EXTENSION_CABAC_TRAINING
//static constexpr int     SCALE_BITS =  15; ///< Precision for fractional bit estimates
static constexpr int     PROB_BITS   = 15;   // Nominal number of bits to represent probabilities#if EC_HIGH_PRECISION
#if EC_HIGH_PRECISION
static constexpr int     PROB_BITS_0 = 15;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 15;   // Number of bits to represent 2nd estimate
#else
static constexpr int     PROB_BITS_0 = 10;   // Number of bits to represent 1st estimate
static constexpr int     PROB_BITS_1 = 14;   // Number of bits to represent 2nd estimate
#endif
static constexpr int     MASK_0      = ~(~0u << PROB_BITS_0) << (PROB_BITS - PROB_BITS_0);
static constexpr int     MASK_1      = ~(~0u << PROB_BITS_1) << (PROB_BITS - PROB_BITS_1);
static constexpr uint8_t DWS         = 8;   // 0x47 Default window sizes
static constexpr uint8_t DWE = 18;  // default weights
static constexpr uint8_t DWO = 119; // default window offsets
const uint8_t weightedAdaptRate[5] = { 10, 12, 16, 20, 22 };

struct BinFracBits
{
  uint32_t intBits[2];
};


class ProbModelTables
{
protected:
#if EC_HIGH_PRECISION
  static const BinFracBits m_binFracBits[512];
#else
  static const BinFracBits m_binFracBits[256];
#endif
};


class BinProbModel : public ProbModelTables
{
public:
  BinProbModel()
  {
    uint16_t half = 1 << (PROB_BITS - 1);
    m_state[0]    = half;
    m_state[1]    = half;
    m_rate        = DWS;
    m_weight        = DWE;
    m_stateUsed[0]  = half;
    m_stateUsed[1]  = half;
    m_rateOffset[0] = DWO;
    m_rateOffset[1] = DWO;
  }
  ~BinProbModel ()                {}

  int getNumInitVals() const { return 64; }

  void init(int qp, int initId)
  {
    int slope = (initId >> 3) - 4;
    int offset = ((initId & 7) * 18) + 1;
    int inistate = ((slope   * (qp - 16)) >> 1) + offset;
    int state_clip = inistate < 1 ? 1 : inistate > 127 ? 127 : inistate;
    const int p1 = (state_clip << 8);

    m_state[0] = p1 & MASK_0;
    m_state[1] = p1 & MASK_1;
    m_stateUsed[0] = m_state[0];
    m_stateUsed[1] = m_state[1];
  }

  void update(unsigned bin)
  {
    int rate0 = m_rate >> 4;
    int rate1 = m_rate & 15;

    auto ws = m_rateOffset[bin];
    int rateUsed0 = std::max(2, rate0 + (ws >> 4) - ADJUSTMENT_RANGE);
    int rateUsed1 = std::max(2, rate1 + (ws & 15) - ADJUSTMENT_RANGE);

    m_stateUsed[0] = m_state[0] - ((m_state[0] >> rateUsed0) & MASK_0);
    m_stateUsed[1] = m_state[1] - ((m_state[1] >> rateUsed1) & MASK_1);

    m_state[0] -= (m_state[0] >> rate0) & MASK_0;
    m_state[1] -= (m_state[1] >> rate1) & MASK_1;

    if (bin)
    {
      m_stateUsed[0] += (0x7FFFU >> rateUsed0) & MASK_0;
      m_stateUsed[1] += (0x7FFFU >> rateUsed1) & MASK_1;

      m_state[0] += (0x7fffu >> rate0) & MASK_0;
      m_state[1] += (0x7fffu >> rate1) & MASK_1;
    }
  }
  void setLog2WindowSize(uint8_t log2WindowSize)
  {
    int rate0 = 2 + ((log2WindowSize >> 2) & 3);
    int rate1 = 3 + rate0 + (log2WindowSize & 3);
    m_rate = 16 * rate0 + rate1;
  }
  void estFracBitsUpdate(unsigned bin, uint64_t &b)
  {
    b += estFracBits(bin);
    update(bin);
  }
  uint32_t        estFracBits(unsigned bin) const { return getFracBitsArray().intBits[bin]; }
#if EC_HIGH_PRECISION
  BinFracBits     getFracBitsArray() const { return m_binFracBits[state_est()]; }
  void     setAdaptRateWeight(uint8_t weight) { m_weight = weight; }
  uint8_t  getAdaptRateWeight() const { return m_weight; }
  void     setAdaptRateOffset(uint8_t rateOffset, bool bin) { m_rateOffset[bin] = rateOffset; }
  uint16_t        state_est() const
  {
    uint8_t  wIdx0 = (m_weight >> 3);
    uint8_t  wIdx1 = (m_weight & 0x07);
    uint32_t w0 = weightedAdaptRate[wIdx0];
    uint32_t w1 = weightedAdaptRate[wIdx1];
    uint32_t pd;
    if ((w0 + w1) <= 32)
    {
      pd = (uint32_t(m_stateUsed[0]) * w0 + uint32_t(m_stateUsed[1]) * w1) >> 11;
    }
    else
    {
      pd = (uint32_t(m_stateUsed[0]) * w0 + uint32_t(m_stateUsed[1]) * w1) >> 12;
    }
    return uint16_t(pd);
  }

#else
  BinFracBits     getFracBitsArray() const { return m_binFracBits[state()]; }
  uint8_t state() const { return (m_state[0] + m_state[1]) >> 8; }
#endif

protected:
  uint16_t m_state[2];
  uint8_t  m_rate;
  uint16_t m_stateUsed[2];
  uint8_t  m_rateOffset[2];
  uint8_t  m_weight;
};


class CtxSet
{
public:
  CtxSet(uint16_t offset, uint16_t size) : Offset(offset), Size(size) {}
  CtxSet(const CtxSet& ctxSet) : Offset(ctxSet.Offset), Size(ctxSet.Size) {}
  CtxSet(std::initializer_list<CtxSet> ctxSets);
public:
  uint16_t  operator()  ()  const
  {
    return Offset;
  }
  uint16_t  operator()  (uint16_t inc)  const
  {
    CHECKD(inc >= Size, "Specified context increment (" << inc << ") exceed range of context set [0;" << Size - 1 << "].");
    return Offset + inc;
  }
  bool operator== (const CtxSet& ctxSet) const
  {
    return (Offset == ctxSet.Offset && Size == ctxSet.Size);
  }
  bool operator!= (const CtxSet& ctxSet) const
  {
    return (Offset != ctxSet.Offset || Size != ctxSet.Size);
  }
public:
  uint16_t  Offset;
  uint16_t  Size;
};

class ContextSetCfg
{
public:
  // context sets: specify offset and size
  static const CtxSet   SplitFlag;
  static const CtxSet   SplitQtFlag;
  static const CtxSet   SplitHvFlag;
  static const CtxSet   Split12Flag;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  static const CtxSet   ModeConsFlag;
#endif
  static const CtxSet   SkipFlag;
  static const CtxSet   MergeFlag;
  static const CtxSet   RegularMergeFlag;
  static const CtxSet   MergeIdx;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  static const CtxSet   TmMergeIdx;
#endif
#if JVET_X0049_ADAPT_DMVR
  static const CtxSet   BMMergeFlag;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  static const CtxSet   affBMFlag;
#endif
#if JVET_AA0070_RRIBC
  static const CtxSet   rribcFlipType;
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  static const CtxSet bvOneZeroComp;
#endif
#if JVET_Y0065_GPM_INTRA
  static const CtxSet   GPMIntraFlag;
#endif
  static const CtxSet   PredMode;
  static const CtxSet   MultiRefLineIdx;
  static const CtxSet   IntraLumaMpmFlag;
#if SECONDARY_MPM
  static const CtxSet   IntraLumaSecondMpmFlag;
#if JVET_AD0085_MPM_SORTING
  static const CtxSet   IntraLumaSecondMpmIdx;
#endif
#endif
  static const CtxSet   IntraLumaPlanarFlag;
#if SECONDARY_MPM
  static const CtxSet   IntraLumaMPMIdx;
#endif
  static const CtxSet   CclmModeFlag;
  static const CtxSet   CclmModeIdx;
  static const CtxSet   IntraChromaPredMode;
#if JVET_AD0188_CCP_MERGE
  static const CtxSet   nonLocalCCP;
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
#if ENABLE_DIMD
  static const CtxSet   DimdChromaMode;
#endif
  static const CtxSet   ChromaFusionMode;
#if JVET_AC0119_LM_CHROMA_FUSION
  static const CtxSet   ChromaFusionType;
  static const CtxSet   ChromaFusionCclm;
#endif
#endif
#if JVET_AC0071_DBV
  static const CtxSet DbvChromaMode;
#endif
  static const CtxSet   MipFlag;
#if JVET_V0130_INTRA_TMP
  static const CtxSet   TmpFlag;
#if JVET_AD0086_ENHANCED_INTRA_TMP
  static const CtxSet   TmpIdx;
  static const CtxSet   TmpFusion;
#endif  
#endif
#if MMLM
  static const CtxSet   MMLMFlag;
#endif
  static const CtxSet   DeltaQP;
  static const CtxSet   InterDir;
  static const CtxSet   RefPic;
#if JVET_Z0054_BLK_REF_PIC_REORDER
  static const CtxSet   RefPicLC;
#endif
  static const CtxSet   MmvdFlag;
  static const CtxSet   MmvdMergeIdx;
  static const CtxSet   MmvdStepMvpIdx;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  static const CtxSet   MmvdStepMvpIdxECM3;
#endif
#if JVET_W0097_GPM_MMVD_TM
  static const CtxSet   GeoMmvdFlag;
  static const CtxSet   GeoMmvdStepMvpIdx;
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  static const CtxSet   GeoBldFlag;
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  static const CtxSet   GeoSubModeIdx;
#endif
  static const CtxSet   SubblockMergeFlag;
  static const CtxSet   AffineFlag;
  static const CtxSet   AffineType;
  static const CtxSet   AffMergeIdx;
#if AFFINE_MMVD
  static const CtxSet   AfMmvdFlag;
  static const CtxSet   AfMmvdIdx;
  static const CtxSet   AfMmvdOffsetStep;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  static const CtxSet   AfMmvdOffsetStepECM3;
#endif
#endif
#if JVET_AA0061_IBC_MBVD
  static const CtxSet   IbcMbvdFlag;
  static const CtxSet   IbcMbvdMergeIdx;
  static const CtxSet   IbcMbvdStepMvpIdx;
#endif
#if JVET_AC0112_IBC_CIIP
  static const CtxSet   IbcCiipFlag;
  static const CtxSet   IbcCiipIntraIdx;
#endif
#if JVET_AC0112_IBC_GPM
  static const CtxSet   IbcGpmFlag;
  static const CtxSet   IbcGpmIntraFlag;
  static const CtxSet   IbcGpmSplitDirSetFlag;
  static const CtxSet   IbcGpmBldIdx;
#endif
#if JVET_AC0112_IBC_LIC
  static const CtxSet   IbcLicFlag;
#if JVET_AE0078_IBC_LIC_EXTENSION
  static const CtxSet   IbcLicIndex;
#endif
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  static const CtxSet   TMMergeFlag;
#endif
#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
  static const CtxSet   CiipTMMergeFlag;
#endif
#endif
  static const CtxSet   Mvd;
#if JVET_Z0131_IBC_BVD_BINARIZATION
  static const CtxSet   Bvd;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AC0104_IBC_BVD_PREDICTION
  static const CtxSet   MvsdIdx;
#endif
#if JVET_AD0140_MVD_PREDICTION && JVET_AC0104_IBC_BVD_PREDICTION
  static const CtxSet   MvsdIdxIBC;
#endif

#if JVET_AD0140_MVD_PREDICTION
  static const CtxSet   MvsdIdxMVDMSB;
#endif

#if JVET_AC0104_IBC_BVD_PREDICTION
  static const CtxSet   MvsdIdxBVDMSB;
#endif

#if MULTI_HYP_PRED
  static const CtxSet   MultiHypothesisFlag;
  static const CtxSet   MHRefPic;
  static const CtxSet   MHWeight;
#endif
  static const CtxSet   BDPCMMode;
  static const CtxSet   QtRootCbf;
  static const CtxSet   ACTFlag;
  static const CtxSet   QtCbf[3];    // [ channel ]
  static const CtxSet   SigCoeffGroup[2];    // [ ChannelType ]
  static const CtxSet   LastX[2];    // [ ChannelType ]
  static const CtxSet   LastY[2];    // [ ChannelType ]
#if JVET_AE0102_LFNST_CTX
  static const CtxSet   SigFlagL[6];    // [ ChannelType + State ]
  static const CtxSet   ParFlagL[2];    // [ ChannelType ]
  static const CtxSet   GtxFlagL[4];    // [ ChannelType + x ]
#endif
  static const CtxSet   SigFlag[6];    // [ ChannelType + State ]
  static const CtxSet   ParFlag[2];    // [ ChannelType ]
  static const CtxSet   GtxFlag[4];    // [ ChannelType + x ]
  static const CtxSet   TsSigCoeffGroup;
  static const CtxSet   TsSigFlag;
  static const CtxSet   TsParFlag;
  static const CtxSet   TsGtxFlag;
  static const CtxSet   TsLrg1Flag;
  static const CtxSet   TsResidualSign;
  static const CtxSet   MVPIdx;
#if JVET_X0083_BM_AMVP_MERGE_MODE
  static const CtxSet   amFlagState;
#endif
  static const CtxSet   SaoMergeFlag;
  static const CtxSet   SaoTypeIdx;
#if JVET_V0094_BILATERAL_FILTER
  static const CtxSet   BifCtrlFlags[];
#endif
#if JVET_W0066_CCSAO
  static const CtxSet   CcSaoControlIdc;
#endif
  static const CtxSet   TransformSkipFlag;
  static const CtxSet   MTSIdx;
  static const CtxSet   LFNSTIdx;
  static const CtxSet   PLTFlag;
  static const CtxSet   RotationFlag;
  static const CtxSet   RunTypeFlag;
  static const CtxSet   IdxRunModel;
  static const CtxSet   CopyRunModel;
  static const CtxSet   SbtFlag;
  static const CtxSet   SbtQuadFlag;
  static const CtxSet   SbtHorFlag;
  static const CtxSet   SbtPosFlag;
  static const CtxSet   ChromaQpAdjFlag;
  static const CtxSet   ChromaQpAdjIdc;
  static const CtxSet   ImvFlag;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  static const CtxSet   ImvFlagIBC;
#endif
#if ENABLE_DIMD
  static const CtxSet   DimdFlag;
#endif
#if JVET_W0123_TIMD_FUSION
  static const CtxSet   TimdFlag;
#endif
#if JVET_AB0155_SGPM
  static const CtxSet   SgpmFlag;
#endif
#if ENABLE_OBMC
  static const CtxSet   ObmcFlag;
#endif 
  static const CtxSet   BcwIdx;
  static const CtxSet   ctbAlfFlag;
  static const CtxSet   ctbAlfAlternative;
  static const CtxSet   AlfUseTemporalFilt;
  static const CtxSet   CcAlfFilterControlFlag;
  static const CtxSet   CiipFlag;
  static const CtxSet   SmvdFlag;
  static const CtxSet   IBCFlag;
#if JVET_AE0169_BIPREDICTIVE_IBC
  static const CtxSet   BiPredIbcFlag;
#endif
  static const CtxSet   ISPMode;
  static const CtxSet   JointCbCrFlag;
#if INTER_LIC
  static const CtxSet   LICFlag;
#endif
#if SIGN_PREDICTION
  static const CtxSet   signPred[2];
#endif
#if JVET_Z0050_CCLM_SLOPE
  static const CtxSet   CclmDeltaFlags;
#endif
#if JVET_AA0126_GLM
  static const CtxSet   GlmFlags;
#endif
#if JVET_AA0057_CCCM
  static const CtxSet   CccmFlag;
#endif
#if JVET_AE0100_BVGCCCM
  static const CtxSet   BvgCccmFlag;
#endif
#if JVET_AD0202_CCCM_MDF
  static const CtxSet   CccmMpfFlag;
#endif
#if JVET_AD0120_LBCCP
  static const CtxSet   CcInsideFilterFlag;
#endif
#if JVET_AB0157_TMRL
  static const CtxSet   TmrlDerive;
#endif
#if JVET_AE0059_INTER_CCCM
  static const CtxSet   InterCccmFlag;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  static const CtxSet   InterCcpMergeFlag;
#endif
#if JVET_AG0058_EIP
  static const CtxSet   EipFlag;
#endif
  static const unsigned NumberOfContexts;

  // combined sets for less complex copying
  // NOTE: The contained CtxSet's should directly follow each other in the initalization list;
  //       otherwise, you will copy more elements than you want !!!
  static const CtxSet   Sao;
  static const CtxSet   Alf;
  static const CtxSet   Palette;

public:
  static const std::vector<uint8_t>&  getInitTable(unsigned initId);
  static std::vector<uint16_t> sm_InitTableNameIndexes;
private:
  static std::vector<std::vector<uint8_t> > sm_InitTables;
  static CtxSet addCtxSet(std::initializer_list<std::initializer_list<uint8_t> > initSet2d);
};

#endif