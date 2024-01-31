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

/** \file     IntraPrediction.h
    \brief    prediction class (header)
*/

#ifndef __INTRAPREDICTION__
#define __INTRAPREDICTION__


// Include files
#include "Unit.h"
#include "Buffer.h"
#include "Picture.h"
#if JVET_W0123_TIMD_FUSION
#include "RdCost.h"
#endif

#include "MatrixIntraPrediction.h"
#if JVET_AB0155_SGPM
#include "CommonLib/InterpolationFilter.h"
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#include "CommonLib/InterPrediction.h"
class InterPrediction;
#endif
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// prediction class
enum PredBuf
{
  PRED_BUF_UNFILTERED = 0,
  PRED_BUF_FILTERED   = 1,
  NUM_PRED_BUF        = 2
};

static const uint32_t MAX_INTRA_FILTER_DEPTHS=8;

#if JVET_V0130_INTRA_TMP
class TempLibFast
{
public:
  int   m_pX;           //offset X
  int   m_pY;           //offset Y
#if JVET_AD0086_ENHANCED_INTRA_TMP
  int   m_rId;
#else
  int   m_pDiff;        //mse
  int   m_diffMax;
#endif

  TempLibFast();
  ~TempLibFast();
#if JVET_AD0086_ENHANCED_INTRA_TMP
  TempLibFast(const int pX, const int pY, const int rId)
  {
    m_pX = pX;
    m_pY = pY;
    m_rId = rId;
  };
#endif

  void  initTemplateDiff              ( unsigned int uiPatchWidth, unsigned int uiPatchHeight, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int bitDepth );

  int   getX                          ()       { return m_pX;      }
  int   getY                          ()       { return m_pY;      }
#if !JVET_AD0086_ENHANCED_INTRA_TMP
  int   getDiff                       ()       { return m_pDiff;   }
  int   getDiffMax                    ()       { return m_diffMax; }
#endif
};

typedef short TrainDataType;
#endif

#if JVET_AA0057_CCCM || JVET_AB0092_GLM_WITH_LUMA || JVET_AC0119_LM_CHROMA_FUSION || JVET_AG0058_EIP

typedef int64_t TCccmCoeff;
#define FIXED_MULT(x, y) TCccmCoeff((int64_t(x)*(y) + CCCM_DECIM_ROUND) >> CCCM_DECIM_BITS )
#if !JVET_AB0174_CCCM_DIV_FREE
#define FIXED_DIV(x, y)  TCccmCoeff((int64_t(x)    << CCCM_DECIM_BITS ) / (y) )
#endif

struct CccmModel
{
  CccmModel( int num, int bitdepth )
  {
    bd = bitdepth;
    midVal = ( 1 << ( bitdepth - 1 ) );
    params.resize( num );
  }

  ~CccmModel() {}

  std::vector<TCccmCoeff> params;
  int        bd;
  int        midVal;
  
  const int getNumParams() const
  {
    return (int)params.size();
  }

  void clearModel()
  {
    const int numParams = (int)params.size();

    std::fill( params.begin(), params.end(), 0 );

    params[numParams - 1] = 1 << CCCM_DECIM_BITS; // Default bias to 1
  }

  Pel convolve(Pel* vector)
  {
    TCccmCoeff sum = 0;
    
    for( int i = 0; i < params.size(); i++)
    {
      sum += params[i] * vector[i];
    }

    return Pel( (sum + CCCM_DECIM_ROUND ) >> CCCM_DECIM_BITS );
  }
  
  Pel nonlinear(const Pel val) { return (val * val + midVal) >> bd; }
  Pel bias     ()              { return midVal; }
#if JVET_AE0059_INTER_CCCM
  void setBd(const int bitdepth)
  {
    bd = bitdepth;
    midVal = (1 << (bitdepth - 1));
  }
#endif
};

struct CccmCovariance
{
  CccmCovariance() {}
  ~CccmCovariance() {}

#if JVET_AB0174_CCCM_DIV_FREE
  void solve1                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* C, const int sampleNum, const int chromaOffset, CccmModel& model );
  void solve2                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* Cb, const Pel* Cr, const int sampleNum, const int chromaOffsetCb, const int chromaOffsetCr, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode = false
#endif
  );
#else
  void solve1                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* C, const int sampleNum, CccmModel& model );
  void solve2                      ( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* Cb, const Pel* Cr, const int sampleNum, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode = false
#endif
  );
#endif
#if JVET_AG0058_EIP
#if JVET_AB0174_CCCM_DIV_FREE
  void solveEip                    ( const TCccmCoeff* A, const TCccmCoeff* Y, const int sampleNum, const int lumaOffset, CccmModel& model );
#else
  void solveEip                    ( const TCccmCoeff* A, const TCccmCoeff* Y, const int sampleNum, CccmModel& model );
#endif
#endif
private:
  TCccmCoeff ATA[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX];
  TCccmCoeff ATCb[CCCM_NUM_PARAMS_MAX];
  TCccmCoeff ATCr[CCCM_NUM_PARAMS_MAX];
  TCccmCoeff C[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX + 2];

#if JVET_AC0053_GAUSSIAN_SOLVER
  void gaussBacksubstitution       ( TCccmCoeff* x, int numEq, int col );
#if JVET_AE0059_INTER_CCCM
  void gaussElimination            ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y0, TCccmCoeff* x0, TCccmCoeff* y1, TCccmCoeff* x1, int numEq, int numFilters, int bd, const bool interCccmMode = false);
#else
  void gaussElimination            ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y0, TCccmCoeff* x0, TCccmCoeff* y1, TCccmCoeff* x1, int numEq, int numFilters, int bd);
#endif
#else
  bool ldlDecompose                ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, int numEq) const;
  void ldlSolve                    ( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, TCccmCoeff* y, TCccmCoeff* x, int numEq, bool decompOk) const;
  void ldlBacksubstitution         ( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* z, TCccmCoeff* x, int numEq) const;
  void ldlTransposeBacksubstitution( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y, TCccmCoeff* z, int numEq) const;
  bool ldlDecomp                   ( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* outDiag, int numEq) const;
#endif
};
#endif

class IntraPrediction
{
public:
#if MMLM
  bool m_encPreRDRun;
#endif

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  Pel* m_cccmLumaBuf[CCCM_NUM_PRED_FILTER + 1];
#else
  Pel* m_cccmLumaBuf[2];
#endif
#else
  Pel* m_cccmLumaBuf;
#endif
#if JVET_AA0057_CCCM
  CccmCovariance m_cccmSolver;

  Pel m_samples[CCCM_NUM_PARAMS_MAX];
  Pel m_a[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX];
  Pel m_cb[CCCM_REF_SAMPLES_MAX];
  Pel m_cr[CCCM_REF_SAMPLES_MAX];
#endif

#if SECONDARY_MPM
  uint8_t m_intraMPM[NUM_MOST_PROBABLE_MODES];
  uint8_t m_intraNonMPM[NUM_NON_MPM_MODES];
#endif
protected:
#if JVET_AC0094_REF_SAMPLES_OPT
  Pel m_refBuffer[MAX_NUM_COMPONENT][NUM_PRED_BUF][((MAX_CU_SIZE << 3) + 1 + MAX_REF_LINE_IDX) * 2];
#else
  Pel      m_refBuffer[MAX_NUM_COMPONENT][NUM_PRED_BUF][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
#endif
  uint32_t m_refBufferStride[MAX_NUM_COMPONENT];
#if JVET_AB0155_SGPM
  InterpolationFilter m_if;
#endif
#if JVET_AB0157_INTRA_FUSION
  Pel      m_refBuffer2nd[MAX_NUM_COMPONENT][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
#endif
#if JVET_AG0058_EIP
  Pel        m_eipBuffer[(MAX_EIP_SIZE * 2 + MAX_EIP_REF_SIZE) * (MAX_EIP_SIZE * 2 + MAX_EIP_REF_SIZE)];
  Pel        m_eipYBuffer[NUM_EIP_BASE_RECOTYPE][MAX_EIP_SIZE * MAX_EIP_SIZE * 2];
  Pel        m_eipPredTpl[2][MAX_EIP_SIZE * EIP_TPL_SIZE];
  TCccmCoeff ATABuf[NUM_EIP_COMB][((EIP_FILTER_TAP + 1) * EIP_FILTER_TAP) >> 1];
  TCccmCoeff ATYBuf[NUM_EIP_COMB][EIP_FILTER_TAP];
  bool       bSrcBufFilled[NUM_EIP_SHAPE * NUM_EIP_BASE_RECOTYPE];
  bool       bDstBufFilled[NUM_EIP_BASE_RECOTYPE];
  int        numSamplesBuf[NUM_EIP_BASE_RECOTYPE];
#endif
private:
#if !MERGE_ENC_OPT
  Pel* m_yuvExt2[MAX_NUM_COMPONENT][4];
  int  m_yuvExtSize2;
#endif

#if JVET_AA0057_CCCM || JVET_AC0119_LM_CHROMA_FUSION || JVET_AG0058_EIP
  Area m_cccmBlkArea;
#if JVET_AB0174_CCCM_DIV_FREE
  int  m_cccmLumaOffset;
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  Area m_tmpRefArea[MTMP_NUM];
  Pel* m_tmpRefBuf[MTMP_NUM];
#endif

#if JVET_AA0057_CCCM
  Area m_cccmRefArea;
#if JVET_AE0100_BVGCCCM
  Pel* m_bvgCccmLumaBuf[NUM_BVG_CCCM_CANDS];
  Pel* m_bvgCccmChromaBuf[NUM_BVG_CCCM_CANDS][2];
  Area m_bvgCccmBlkArea;
  Area m_bvgCccmRefArea;
#endif
#endif
  
  static const uint8_t m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS];
#if JVET_W0123_TIMD_FUSION
  static const uint8_t m_aucIntraFilterExt[MAX_INTRA_FILTER_DEPTHS];
  RdCost* m_timdSatdCost;
#endif
#if JVET_AC0071_DBV
  RdCost *m_dbvSadCost;
#endif
#if LMS_LINEAR_MODEL
  unsigned m_auShiftLM[32]; // Table for substituting division operation by multiplication
#endif
  struct IntraPredParam //parameters of Intra Prediction
  {
    bool refFilterFlag;
    bool applyPDPC;
#if GRAD_PDPC
    bool useGradPDPC;
#endif
    bool isModeVer;
    int  multiRefIndex;
    int  intraPredAngle;
    int  absInvAngle;
    bool interpolationFlag;
    int  angularScale;
#if JVET_AB0157_INTRA_FUSION
    bool fetchRef2nd;
    bool applyFusion;
#endif

    // clang-format off
    IntraPredParam()
      : refFilterFlag(false)
      , applyPDPC(false)
#if GRAD_PDPC
      , useGradPDPC(false)
#endif
      , isModeVer(false)
      , multiRefIndex(-1)
      , intraPredAngle(std::numeric_limits<int>::max())
      , absInvAngle(std::numeric_limits<int>::max())
      , interpolationFlag(false)
      , angularScale(-1)
#if JVET_AB0157_INTRA_FUSION
      , fetchRef2nd(false)
      , applyFusion(false)
#endif
    // clang-format on
    {
    }
  };

  IntraPredParam m_ipaParam;

#if JVET_AD0120_LBCCP
  Pel* m_pCCFilterTemp;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  Pel* m_pCcpMerge[2];
#endif
#if JVET_AB0067_MIP_DIMD_LFNST
  Pel* m_pMipTemp;
#endif
  Pel* m_piTemp;
  Pel* m_pMdlmTemp; // for MDLM mode
#if JVET_AA0126_GLM
  Pel* m_glmTempCb[NUM_GLM_IDC];
  Pel* m_glmTempCr[NUM_GLM_IDC];
#if JVET_AB0092_GLM_WITH_LUMA
  Area m_glmRefArea;
  Pel* m_glmGradBuf[NUM_GLM_IDC];
#if JVET_AB0174_CCCM_DIV_FREE
  int  m_glmLumaOffset;
#endif
#endif
#endif
#if JVET_AC0119_LM_CHROMA_FUSION
  Area m_cflmRefArea;
  Pel* m_cflmBuf[3];
#endif
  MatrixIntraPrediction m_matrixIntraPred;



protected:
  ChromaFormat  m_currChromaFormat;

  int m_topRefLength;
  int m_leftRefLength;
  ScanElement* m_scanOrder;
  bool         m_bestScanRotationMode;
  std::vector<PelStorage>   m_tempBuffer;
#if JVET_AB0155_SGPM
  std::vector<PelStorage>   m_sgpmBuffer;
#endif
  // used in timd tmrl sortedMPM
  std::vector<PelStorage>   m_intraPredBuffer;
  Pel tempRefAbove[(MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX];
  Pel tempRefLeft[(MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX];

#if JVET_V0130_INTRA_TMP
  int          m_uiPartLibSize;
#if JVET_AD0086_ENHANCED_INTRA_TMP
  static_vector<TempLibFast, MTMP_NUM> m_mtmpCandList;
  static_vector<uint64_t, MTMP_NUM>    m_mtmpCostList;
#else
  TempLibFast  m_tempLibFast;
#endif
  Pel*         m_refPicUsed;
  Picture*     m_refPicBuf;
  unsigned int m_uiPicStride;
  unsigned int m_uiVaildCandiNum;
  Pel***       m_pppTarPatch;
#endif

#if TMP_FAST_ENC
#if JVET_AD0086_ENHANCED_INTRA_TMP
  int                m_tmpXdisp[MTMP_NUM];
  int                m_tmpYdisp[MTMP_NUM];
  IntraTMPFusionInfo m_tmpFusionInfo[TMP_GROUP_IDX << 1];
#else
  int            m_tmpXdisp;
  int            m_tmpYdisp;
#endif
  int            m_tmpNumCand;
#endif

  // prediction
  void xPredIntraPlanar           ( const CPelBuf &pSrc, PelBuf &pDst
#if JVET_AC0105_DIRECTIONAL_PLANAR
    , uint8_t plidx
#endif
  );
  void xPredIntraDc               ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter = true );
#if JVET_W0123_TIMD_FUSION
#if JVET_AB0157_INTRA_FUSION
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir, const CPelBuf &pSrc2nd, bool isISP = true, int weightMode = 4);
#else
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir);
#endif
#else
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng);
#endif
#if !(JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
#if JVET_AB0155_SGPM
  void initPredIntraParams(const PredictionUnit &pu, const CompArea compArea, const SPS &sps, const int partIdx = 0);
#else
  void initPredIntraParams        ( const PredictionUnit & pu,  const CompArea compArea, const SPS& sps );
#endif
#endif

  static bool isIntegerSlope(const int absAng) { return (0 == (absAng & 0x1F)); }
#if JVET_W0123_TIMD_FUSION
  static bool isIntegerSlopeExt(const int absAng) { return (0 == (absAng & 0x3F)); }
#endif

  void xPredIntraBDPCM            ( const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng& clpRng );
  Pel  xGetPredValDc              ( const CPelBuf &pSrc, const Size &dstSize );
#if JVET_V0130_INTRA_TMP && !JVET_W0069_TMP_BOUNDARY
  bool isRefTemplateAvailable(CodingUnit& cu, CompArea& area);
#endif

  void xFillReferenceSamples      ( const CPelBuf &recoBuf,      Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu );
  void xFilterReferenceSamples(const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area, const SPS &sps,
                               int multiRefIdx
  );

  static int getModifiedWideAngle         ( int width, int height, int predMode );
#if JVET_W0123_TIMD_FUSION
#if JVET_AB0155_SGPM
  static int getWideAngleExt(int width, int height, int predMode, bool bSgpm = false);
#else
  static int getWideAngleExt      ( int width, int height, int predMode );
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  static int getTimdWideAngleExt(int width, int height, int predMode);
  static int getTimdRegularAngleExt(int width, int height, int predMode);
#endif
#endif

#if JVET_AC0094_REF_SAMPLES_OPT
  static int getTimdWideAngle(int width, int height, int predMode);
#endif
  void setReferenceArrayLengths   ( const CompArea &area );

  void destroy                    ();
#if LMS_LINEAR_MODEL
  void xPadMdlmTemplateSample     (Pel*pSrc, Pel*pCur, int cWidth, int cHeight, int existSampNum, int targetSampNum);
  void xGetLMParametersLMS        (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel);
#else
  void xGetLMParameters           (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel);
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
public:
#endif
#if LMS_LINEAR_MODEL && MMLM
  struct MMLMParameters
  {
    int a;
    int b;
    int shift;
  };
  int xCalcLMParametersGeneralized(int x, int y, int xx, int xy, int count, int bitDepth, int &a, int &b, int &iShift);
  int xLMSampleClassifiedTraining (int count, int mean, int meanC, int lumaSamples[], int chrmSamples[], int bitDepth, MMLMParameters parameters[]);
#endif
#if JVET_AE0078_IBC_LIC_EXTENSION
protected:
#endif
#if JVET_Z0050_CCLM_SLOPE
  void xUpdateCclmModel           (int &a, int &b, int &iShift, int midLuma, int delta);
#endif

public:
  IntraPrediction();
  virtual ~IntraPrediction();

  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

#if JVET_AA0057_CCCM || JVET_AC0119_LM_CHROMA_FUSION
  Pel    xCccmGetLumaVal(const PredictionUnit& pu, const CPelBuf pi, const int x, const int y
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
  ) const;
#if JVET_AB0174_CCCM_DIV_FREE
  void   xCccmSetLumaRefValue(const PredictionUnit& pu);
#endif
#endif
#if JVET_AA0057_CCCM
#if JVET_AD0188_CCP_MERGE
  void   predIntraCCCM            ( PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir );
#else
  void   predIntraCCCM            (const PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir);
#endif

  void   xCccmCalcModels          (const PredictionUnit& pu, CccmModel& cccmModelCb, CccmModel& cccmModelCr, int modelId, int modelThr
#if JVET_AD0120_LBCCP
    , int trainingRange = -1
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool useRefSampOnly = false
#endif
  );

  void   xCccmApplyModel          (const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred);

  void   xCccmCreateLumaRef       (const PredictionUnit& pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
  PelBuf xCccmGetLumaRefBuf       (const PredictionUnit& pu, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY
#if JVET_AD0202_CCCM_MDF
    , int cccmDownsamplesFilterIdx = 0, int numBuffer = 0, PelBuf* refLuma1 = NULL, PelBuf* refLuma3 = NULL, PelBuf* refLuma2 = NULL
#endif
  ) const;
  PelBuf xCccmGetLumaPuBuf        (const PredictionUnit& pu
#if JVET_AD0202_CCCM_MDF
    , int cccmDownsamplesFilterIdx = 0, int numBuffer = 0, CPelBuf* refLuma1 = NULL, CPelBuf* refLuma3 = NULL, CPelBuf* refLuma2 = NULL
#endif
  ) const;
  int    xCccmCalcRefAver         (const PredictionUnit& pu
#if JVET_AD0120_LBCCP
                                    , int trainingRange = -1
#endif
  ) const;
  void   xCccmCalcRefArea         (const PredictionUnit& pu, CompArea chromaArea);
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  void   xCccmCreateLumaNoSubRef  ( const PredictionUnit& pu, CompArea chromaArea 
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
#endif
#if JVET_AE0100_BVGCCCM
  void   xBvgCccmCalcRefArea      (const PredictionUnit& pu, CompArea chromaArea);
  PelBuf xBvgCccmGetLumaPuBuf     (const PredictionUnit& pu, int candIdx = 0) const;
  PelBuf xBvgCccmGetLumaPuBufFul  (const PredictionUnit& pu, int candIdx = 0) const;
  PelBuf xBvgCccmGetChromaPuBuf   (const PredictionUnit& pu, const ComponentID compId, int candIdx = 0) const;
  void   xBvgCccmCreateLumaRef    (const PredictionUnit& pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
                                   );
  int    xBvgCccmCalcBlkAver   (const PredictionUnit& pu) const;
  void   xBvgCccmCalcBlkRange  (const PredictionUnit& pu, int& minVal, int&maxVal) const;
  void   xBvgCccmCalcModels  ( const PredictionUnit& pu, CccmModel& cccmModelCb, CccmModel& cccmModelCr, int modelId, int modelThr, int minVal, int maxVal );
  void   xBvgCccmApplyModel  ( const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred );
#endif
#endif
#if JVET_AB0092_GLM_WITH_LUMA
  void   xGlmCalcModel            (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel);
  void   xGlmApplyModel           (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel, PelBuf &piPred);
  void   xGlmCreateGradRef        (const PredictionUnit& pu, CompArea chromaArea
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
  PelBuf xGlmGetGradRefBuf        (const PredictionUnit& pu, CompArea chromaArea, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY, int glmIdx) const;
  PelBuf xGlmGetGradPuBuf         (const PredictionUnit& pu, CompArea chromaArea, int glmIdx) const;
  Pel    xGlmGetGradVal           (const PredictionUnit& pu, const int glmIdx, const CPelBuf pi, const int x, const int y) const;
  void   xGlmCalcRefArea          (const PredictionUnit& pu, CompArea chromaArea);
#if JVET_AB0174_CCCM_DIV_FREE
  void   xGlmSetLumaRefValue      (const PredictionUnit& pu, CompArea chromaArea);
#endif
#endif
#if JVET_AC0119_LM_CHROMA_FUSION
  void   xCflmCalcModels(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& cflmModel, int modelId, int modelThr);
  void   xCflmApplyModel(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& cflmModel, int modelId, int modelThr, PelBuf& piPred);
  void   xCflmCreateLumaRef       (const PredictionUnit& pu, const CompArea& chromaArea);
  bool   xCflmCreateChromaPred    (const PredictionUnit& pu, const ComponentID compId, PelBuf& piPred);
  PelBuf xCflmGetRefBuf           (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, int& areaWidth, int& areaHeight, int& refSizeX, int& refSizeY, int& refPosPicX, int& refPosPicY) const;
  PelBuf xCflmGetPuBuf            (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea) const;
  int    xCflmCalcRefAver         (const PredictionUnit& pu, const CompArea& chromaArea);
  void   xCflmCalcRefArea         (const PredictionUnit& pu, const CompArea& chromaArea);
#endif

#if JVET_AD0188_CCP_MERGE
  void reorderCCPCandidates       ( PredictionUnit &pu, CCPModelCandidate candList[], int reorderlistSize );
  int  xGetOneCCPCandCost         ( PredictionUnit &pu, CCPModelCandidate &ccpCand );
  void predCCPCandidate           ( PredictionUnit &pu, PelBuf &predCb, PelBuf &predCr);

  void xCclmApplyModel            (const PredictionUnit &pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred);
  void xCccmApplyModelOffset      (const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf& piPred, int lumaOffset, int chromaOffset[2], int type, int refSizeX = 0, int refSizeY = 0 );
  void xGlmApplyModelOffset       (const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel, int glmIdc, PelBuf& piPred, int lumaOffset, int chromaOffset);  

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostCCLM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CclmModel &cclmModel, int modelNum, int glmIdc);

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostCCCM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr, int lumaOffset, int chromaOffset[2], int type, int refSizeX = 0, int refSizeY = 0, const int cccmMultiFilterIdx = -1 );

  template <const bool updateOffsets>
  int xUpdateOffsetsAndGetCostGLM(const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, CccmModel& glmModel, int glmIdc, int lumaOffset, int& chromaOffset);
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  void xInterCccmApplyModelOffset(const PredictionUnit &pu, const ComponentID compId, CccmModel &cccmModel,
                                  PelBuf &piPred, int lumaOffset, int chromaOffset);
  int  xGetCostInterCccm(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea,
                         CccmModel &cccmModel, int lumaOffset, int chromaOffset);
  void xAddOnTheFlyCalcCCPCands4InterBlk(const PredictionUnit &pu, CompArea chromaArea, CCPModelCandidate candList[],
                                         int &validNum);
  void selectCcpMergeCand(PredictionUnit &pu, CCPModelCandidate candList[], int reorderlistSize);
  void combineCcpAndInter(PredictionUnit &pu, PelBuf &inPredCb, PelBuf &inPredCr, PelBuf &outPredCb, PelBuf &outPredCr);
#endif

#if ENABLE_DIMD
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  static int deriveDimdIntraTmpModePred(const CodingUnit cu, CPelBuf predBuf); // using prediction samples
#endif
  static void deriveDimdMode      (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu);
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  static void deriveDimdChromaMode(const CPelBuf &recoBufY, const CPelBuf &recoBufCb, const CPelBuf &recoBufCr, const CompArea &areaY, const CompArea &areaCb, const CompArea &areaCr, CodingUnit &cu);
#endif
#if JVET_AB0067_MIP_DIMD_LFNST && ENABLE_DIMD
  static int deriveDimdMipMode(PelBuf& reducedPred, int width, int height, CodingUnit& cu);
#endif
  static int  buildHistogram      ( const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int* piHistogram, int direction, int bw, int bh );
#endif
#if JVET_W0123_TIMD_FUSION || JVET_AC0119_LM_CHROMA_FUSION
  void xIntraPredTimdHorVerPdpc   (Pel* pDsty,const int dstStride, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, const Pel* refMain, const ClpRng& clpRng);
  void xPredTimdIntraPlanar       (const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight);
  void xPredTimdIntraDc           ( const PredictionUnit &pu, const CPelBuf &pSrc, Pel* pDst, int iDstStride, int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight);
  void xPredTimdIntraAng          ( const CPelBuf &pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride, int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight, uint32_t dirMode
#if JVET_AC0119_LM_CHROMA_FUSION
    , const ChannelType channelType
#endif
  );
  void xIntraPredTimdAngLuma(Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset);
#if JVET_AC0119_LM_CHROMA_FUSION
  void xIntraPredTimdAngChroma(Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset);
#endif
  void xIntraPredTimdPlanarDcPdpc (const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight);
  void xIntraPredTimdAngPdpc(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height, int xOffset, int yOffset, int scale, int invAngle);
  void xFillTimdReferenceSamples  ( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu, int iTemplateWidth, int iTemplateHeight );
  Pel  xGetPredTimdValDc          ( const CPelBuf &pSrc, const Size &dstSize, TemplateType eTempType, int iTempHeight, int iTempWidth );
#if JVET_AB0155_SGPM
  void initPredTimdIntraParams(const PredictionUnit &pu, const CompArea area, int dirMode, bool bSgpm = false
#if JVET_AC0094_REF_SAMPLES_OPT
                               , bool checkWideAngle = true
#endif
  );
#else
  void initPredTimdIntraParams    (const PredictionUnit & pu, const CompArea area, int dirMode);
#endif
  void predTimdIntraAng           ( const ComponentID compId, const PredictionUnit &pu, uint32_t uiDirMode, Pel* pPred, uint32_t uiStride, uint32_t iWidth, uint32_t iHeight, TemplateType eTempType, int32_t iTemplateWidth, int32_t iTemplateHeight);
#if JVET_AB0155_SGPM
  int deriveTimdMode              ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu, bool bFull = true, bool bHorVer = false );
#else
  int deriveTimdMode              ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu );
#endif
  void initTimdIntraPatternLuma   (const CodingUnit &cu, const CompArea &area, int iTemplateWidth, int iTemplateHeight, uint32_t uiRefWidth, uint32_t uiRefHeight);
#if GRAD_PDPC
  void xIntraPredTimdAngGradPdpc  (Pel* pDsty, const int dstStride, Pel* refMain, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, int deltaPos, int intraPredAngle, const ClpRng& clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
    , const bool bExtIntraDir
#endif
  );
#endif
#if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
  void xIntraPredPlanarDcPdpc     (const CPelBuf &pSrc, Pel *pDst, int iDstStride, int width, int height, bool ciipPDPC);
#else
  void xIntraPredPlanarDcPdpc( const CPelBuf &pSrc, Pel *pDst, int iDstStride, int width, int height );
#endif
#endif
#endif
#if JVET_AB0155_SGPM
  void deriveSgpmModeOrdered(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                             static_vector<SgpmInfo, SGPM_NUM> &candModeList,
                             static_vector<double, SGPM_NUM> &  candCostList);
#endif
#if JVET_AD0085_MPM_SORTING
  void deriveMPMSorted(const PredictionUnit& pu, uint8_t* mpm, int& sortedSize, int iStartIdx);
#endif
#if TMP_FAST_ENC && JVET_AD0086_ENHANCED_INTRA_TMP
  int64_t            m_tmpFlmParams[TMP_FLM_PARAMS][MTMP_NUM];
#endif
#if JVET_AB0157_TMRL
  struct TmrlInfo
  {
    uint32_t uiTemplateAbove;
    uint32_t uiTemplateLeft;
    uint32_t uiWidth;
    uint32_t uiHeight;
    uint32_t uiRefWidth;
    uint32_t uiRefHeight;

  };
  struct TmrlMode
  {
    int8_t  multiRefIdx;
    uint8_t intraDir;
    TmrlMode() : multiRefIdx(0), intraDir(0) {}
    TmrlMode(int8_t _multiRefIdx, uint8_t _intraDir) :
      multiRefIdx(_multiRefIdx), intraDir(_intraDir) {}
  };

  TmrlInfo tmrlInfo;
  void xPredTmrlIntraDc(const CPelBuf& pSrc, Pel* pDst, int iDstStride);
  void xPredTmrlIntraAng(const CPelBuf& pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride);
  void predTmrlIntraAng(const PredictionUnit& pu, Pel* pPred, uint32_t uiStride);
  void initTmrlIntraParams(const PredictionUnit& pu, const CompArea area, const SPS& sps);
  void getTmrlSearchRange(const PredictionUnit& pu, int8_t* tmrlRefList, uint8_t* tmrlIntraList, uint8_t& sizeRef, uint8_t& sizeMode);
  TmrlMode m_tmrlList[MRL_LIST_SIZE];
  void getTmrlList(CodingUnit& cu);
#endif
#if JVET_AG0058_EIP
  void initEipParams(const PredictionUnit& pu, const ComponentID compId);
  void eipPred(const PredictionUnit& pu, PelBuf& piPred, const ComponentID compId = COMPONENT_Y);
  void getCurEipCands(const PredictionUnit& pu, static_vector<EipModelCandidate, NUM_DERIVED_EIP>& candList, const ComponentID compId = COMPONENT_Y, const bool fastTest = true); 
  int64_t (*m_calcAeipGroupSum)(const Pel* src1, const Pel* src2, const int numSamples);
  static int64_t calcAeipGroupSum(const Pel* src1, const Pel* src2, const int numSamples);

  void getNeiEipCands(const PredictionUnit &pu, static_vector<EipModelCandidate, MAX_MERGE_EIP> &candList, const ComponentID compId = COMPONENT_Y);
  void reorderEipCands(const PredictionUnit &pu, static_vector<EipModelCandidate, MAX_MERGE_EIP> &candList, const ComponentID compId = COMPONENT_Y);
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING && JVET_Y0065_GPM_INTRA
protected:
  bool    m_abFilledIntraGPMRefTpl[NUM_INTRA_MODE];
  uint8_t m_aiGpmIntraMPMLists[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS];   //[][0][]: part0, [][1][]: part1
  Pel     m_acYuvRefGPMIntraTemplate[NUM_INTRA_MODE][2][GEO_MAX_CU_SIZE * GEO_MODE_SEL_TM_SIZE];   //[][0][]: top, [][1][]: left

  template <uint8_t partIdx>
  bool xFillIntraGPMRefTemplateAll(PredictionUnit& pu, TemplateType eTempType, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut = nullptr, uint8_t candIdx = std::numeric_limits<uint8_t>::max());
  bool xFillIntraGPMRefTemplate   (PredictionUnit& pu, TemplateType eTempType, uint8_t intraMode, bool loadIntraRef, Pel* bufTop, Pel* bufLeft, std::vector<Pel>* lut = nullptr);

public:
  uint8_t    prefillIntraGPMReferenceSamples   (PredictionUnit& pu, int iTempWidth, int iTempHeight);
  bool       fillIntraGPMRefTemplateAll        (PredictionUnit& pu, bool hasAboveTemplate, bool hasLeftTemplate, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut = nullptr, uint8_t candIdx0 = std::numeric_limits<uint8_t>::max(), uint8_t candIdx1 = std::numeric_limits<uint8_t>::max());
  void       setPrefilledIntraGPMMPMModeAll    (const uint8_t (&mpm)[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS]) {memcpy(&m_aiGpmIntraMPMLists[0][0][0], &mpm[0][0][0], sizeof(m_aiGpmIntraMPMLists));}
  uint8_t    getPrefilledIntraGPMMPMMode       (int partIdx, int splitDir, int realCandIdx) { return m_aiGpmIntraMPMLists[splitDir][partIdx][realCandIdx]; }
  Pel*       getPrefilledIntraGPMRefTemplate   (uint8_t intraMode, uint8_t templateIdx) { CHECK(intraMode >= NUM_INTRA_MODE || !m_abFilledIntraGPMRefTpl[intraMode], "Intra GPM reference template not available!"); return m_acYuvRefGPMIntraTemplate[intraMode][templateIdx]; }
  Pel*       getPrefilledIntraGPMRefTemplate   (int partIdx, int splitDir, int realCandIdx, uint8_t templateIdx) { return getPrefilledIntraGPMRefTemplate(getPrefilledIntraGPMMPMMode(partIdx, splitDir, realCandIdx), templateIdx); }
  void       clearPrefilledIntraGPMRefTemplate () {memset(&m_abFilledIntraGPMRefTpl[0], 0, sizeof(m_abFilledIntraGPMRefTpl));}
#endif
  // Angular Intra
#if JVET_AB0157_INTRA_FUSION
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool applyFusion = true);
#else
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
#endif
  Pel *getPredictorPtr(const ComponentID compId)
  {
    return m_refBuffer[compId][m_ipaParam.refFilterFlag ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED];
  }

#if JVET_AC0071_DBV
  // Direct Block Vector
  void predIntraDbv(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                  , InterPrediction *pcInterPred
#endif
  );
  Mv refineChromaBv(const ComponentID compId, const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                  , InterPrediction *pcInterPred
#endif
  );
#endif

  // Cross-component chroma
#if JVET_AD0188_CCP_MERGE
  void predIntraChromaLM( const ComponentID compID, PelBuf &piPred, PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel = true, CclmModel *cclmModelStored = nullptr );
#else
  void predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel = true, CclmModel *cclmModelStored = nullptr);
#endif
  void xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
    , int downsFilterIdx = 0
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate = false
#endif
  );
#if JVET_AA0126_GLM
  void xGetLumaRecPixelsGlmAll(const PredictionUnit &pu, CompArea chromaArea);
  Pel xGlmGetLumaVal    (const int s[6], const int c[6], const int glmIdx, const Pel val) const;
#endif
  /// set parameters from CU data for accessing intra data
  
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
#if JVET_AB0155_SGPM
  void initPredIntraParams(const PredictionUnit &pu, const CompArea compArea, const SPS &sps, const int partIdx = 0);
#else
  void initPredIntraParams        ( const PredictionUnit & pu,  const CompArea compArea, const SPS& sps );
#endif
#endif
#if JVET_AB0155_SGPM
  void initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false,
    const int partIdx = 0
#if JVET_AB0157_INTRA_FUSION
    , bool applyFusion = true
#endif
    );   // use forceRefFilterFlag to get both filtered and unfiltered buffers
#else   // SGPM
#if JVET_AB0157_INTRA_FUSION
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false, bool applyFusion = true ); // use forceRefFilterFlag to get both filtered and unfiltered buffers
#else
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers
#endif
#endif
  void initIntraPatternChTypeISP  (const CodingUnit& cu, const CompArea& area, PelBuf& piReco, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers

  // Matrix-based intra prediction
  void initIntraMip               (const PredictionUnit &pu, const CompArea &area);
#if JVET_AB0067_MIP_DIMD_LFNST
  void predIntraMip(const ComponentID compId, PelBuf& piPred, const PredictionUnit& pu, bool useDimd = false);
#else
  void predIntraMip               (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
#endif

#if JVET_AD0120_LBCCP
#if JVET_AA0057_CCCM
  uint32_t xCalculateCCCMcost     (const PredictionUnit &pu, const ComponentID compID, int intraDir, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr);
  uint32_t xCalculateCCLMcost     (const PredictionUnit &pu, const ComponentID compID, int intraDir, const CompArea  &chromaArea, const CclmModel &cclmModel);
  void     applyChromaLM          (const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea &chromaArea, int intraDir, const CclmModel &cclmModel);
#endif
  void     filterPredInside       (const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu);
#endif
#if JVET_AG0135_AFFINE_CIIP
  template<bool lmcs>
  void geneWeightedCIIPAffinePred(const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT = nullptr);
#endif
  template<bool lmcs>
  void geneWeightedPred           ( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT = nullptr );
  void geneIntrainterPred         (const CodingUnit &cu, PelStorage& pred);
#if JVET_Z0050_DIMD_CHROMA_FUSION
#if JVET_AD0188_CCP_MERGE
  void geneChromaFusionPred       (const ComponentID compId, PelBuf &piPred, PredictionUnit &pu);
#else
  void geneChromaFusionPred       (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
#endif
#endif
#if JVET_AC0112_IBC_CIIP
  void geneWeightedPred           ( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred);
  void(*m_ibcCiipBlending)        ( Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height );
  static void ibcCiipBlending     ( Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height );
#endif
  void reorderPLT                 (CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp);
#if !MERGE_ENC_OPT
  void geneWeightedPred           (const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf);
  Pel* getPredictorPtr2           (const ComponentID compID, uint32_t idx) { return m_yuvExt2[compID][idx]; }
  void switchBuffer               (const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst);
#endif
#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
  void(*m_dimdBlending)           ( Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height );
  static void dimdBlending        ( Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height );
#endif
#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
  void(*m_timdBlending)           ( Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height );
  static void timdBlending        ( Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height );
#endif
#if JVET_V0130_INTRA_TMP
#if JVET_W0069_TMP_BOUNDARY
#if JVET_AD0086_ENHANCED_INTRA_TMP
  void(*m_calcTemplateDiff)      (Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType TempType, int requiredTemplate);
  static void calcTemplateDiff(Pel *ref, unsigned int uiStride, Pel **tarPatch, unsigned int uiPatchWidth,
                               unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType TempType,
                               int requiredTemplate);
#else
  int( *m_calcTemplateDiff )      ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType TempType );
  static int calcTemplateDiff     ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType TempType );
#endif
#else
  int( *m_calcTemplateDiff )      (Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax);
  static int calcTemplateDiff     ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax );
#endif
  Pel** getTargetPatch            ( unsigned int uiDepth )      { return m_pppTarPatch[uiDepth]; }
  Pel* getRefPicUsed              ()                            { return m_refPicUsed;           }
  void setRefPicUsed              ( Pel* ref )                  { m_refPicUsed = ref;            }
  unsigned int getStride          ()                            { return m_uiPicStride;          }
  void         setStride          ( unsigned int uiPicStride )  { m_uiPicStride = uiPicStride;   }

#if JVET_W0069_TMP_BOUNDARY
  RefTemplateType getRefTemplateType ( CodingUnit& cu, CompArea& area );
  void searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, RefTemplateType tempType );
  void candidateSearchIntra          ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType );
#else
  void searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, );
  void candidateSearchIntra          ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  void convertDiff2Weight            (int *pDiff, int *weights, const int start, const int foundCandiNum);
  int  xCalTMPFusionNumber           (const int maxNum, const int numIdx);
  void xTMPBuildFusionCandidate      (CodingUnit &cu, RefTemplateType tempType);

  void xCalcTmpFlmRefArea            (CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType, bool& leftPadding, bool& rightPadding, bool& abovePadding, bool& belowPadding);
  void xGetTmpFlmRefBuf              (CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType);
  void xCalTmpFlmParam               (CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType);
  void xGenerateTmpFlmPred           (PelBuf& piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType, CodingUnit* pcCU, bool bDeriveDimdMode = true);

  void xTMPFusionCalcParams          (CodingUnit* cu, CompArea area, CccmModel& tmpFusionModel, int foundCandiNum, RefTemplateType tempType, Pel* curPointTemplate, Pel* refPointTemplate[]);
  void xTMPFusionCalcModels          (CodingUnit* cu, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType);
  void xTMPFusionApplyModel          (PelBuf& piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType, CodingUnit* pcCU, bool bDeriveDimdMode = true);

  void xPadForInterpolation          (CodingUnit* pcCU);
#endif

#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool generateTMPrediction          (Pel* piPred, unsigned int uiStride, int& foundCandiNum, PredictionUnit& pu, bool bDeriveDimdMode = true);
#endif
#elif TMP_FAST_ENC
  bool generateTMPrediction          (Pel* piPred, unsigned int uiStride, CompArea area, int& foundCandiNum, CodingUnit* cu);
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool generateTMPrediction          (Pel* piPred, unsigned int uiStride, int& foundCandiNum, PredictionUnit& pu);
#endif
#else
  bool generateTMPrediction          ( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int& foundCandiNum );
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool generateTMPrediction          (Pel *piPred, unsigned int uiStride, int &foundCandiNum, PredictionUnit &pu);
#endif
#endif
#if JVET_W0069_TMP_BOUNDARY
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
  bool generateTmDcPrediction        (Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val, CodingUnit* cu);
#else
  bool generateTmDcPrediction        ( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val );
#endif
  void getTargetTemplate             ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType );
#else
  void getTargetTemplate             ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  void initTmpDisp()
  {
    for (int i = 0; i < MTMP_NUM; i++)
    {
      m_tmpXdisp[i] = 0;
      m_tmpYdisp[i] = 0;
    }
  }
  void initTmpFlmParams()
  {
    for (int j = 0; j < MTMP_NUM; j++)
    {
      for (int i = 0; i < TMP_FLM_PARAMS; i++)
      {
        m_tmpFlmParams[i][j] = -1;
      }
    }
  }
  void initTmpFusionInfo()
  {
    for (int i = 0; i < TMP_GROUP_IDX << 1; i++)
    {
      m_tmpFusionInfo[i] = IntraTMPFusionInfo{ false, false, 0, 1 };
    }
  }
#elif TMP_FAST_ENC
    m_tmpXdisp = 0;
    m_tmpYdisp = 0;
#endif
#if TMP_FAST_ENC
    int getTmpNumCand() { return m_tmpNumCand; }
#endif

#ifdef TARGET_SIMD_X86
  void    initIntraX86();
  template <X86_VEXT vext>
  void    _initIntraX86();
#endif
};
//! \}

#if JVET_W0123_TIMD_FUSION && JVET_AG0092_ENHANCED_TIMD_FUSION
void xLocationdepBlending(Pel *pDst, int strideDst, Pel *pVer, int strideVer, Pel *pHor, int strideHor,Pel *pNonLocDep, int strideNonLocDep, int width, int height, int mode, int wVer, int wHor, int wNonLocDep, int range = 10);
#else
#if ENABLE_DIMD
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
void xDimdLocationdepBlending(Pel *pDst, int strideDst, Pel *pVer, int strideVer, Pel *pHor, int strideHor,Pel *pNonLocDep, int strideNonLocDep, int width, int height, int mode, int wVer, int wHor, int wNonLocDep);
#else
void xDimdLocationdepBlending(Pel *pDst, int strideDst, Pel *pMainAng, int strideMainAng, Pel *pSecondAng, int strideSecondAng,Pel *pPlanar, int stridePlanar, int width, int height, int sideMain, int sideSecond, int wMain, int wSecond, int wPlanar);
#endif
#endif
#endif
#endif

#endif // __INTRAPREDICTION__
