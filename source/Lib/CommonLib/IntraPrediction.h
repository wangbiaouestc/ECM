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
  int   m_pDiff;        //mse
  short m_pId;          //frame id
  int   m_diffMax;

  TempLibFast();
  ~TempLibFast();

  void  initTemplateDiff              ( unsigned int uiPatchWidth, unsigned int uiPatchHeight, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int bitDepth );

  int   getX                          ()       { return m_pX;      }
  int   getY                          ()       { return m_pY;      }
  int   getDiff                       ()       { return m_pDiff;   }
  short getId                         ()       { return m_pId;     }
  int   getDiffMax                    ()       { return m_diffMax; }
};

typedef short TrainDataType;
#endif

#if JVET_AA0057_CCCM
typedef int64_t TCccmCoeff;

#define FIXED_MULT(x, y) TCccmCoeff((int64_t(x)*(y) + CCCM_DECIM_ROUND) >> CCCM_DECIM_BITS )
#define FIXED_DIV(x, y)  TCccmCoeff((int64_t(x)    << CCCM_DECIM_BITS ) / (y) )

struct CccmModel
{
  TCccmCoeff params[CCCM_NUM_PARAMS];
  int        bd;
  int        midVal;
  
  CccmModel(int bitdepth)
  {
    bd     = bitdepth;
    midVal = (1 << ( bitdepth - 1));
  }
  
  void clearModel(int numParams)
  {
    for( int i = 0; i < numParams - 1; i++)
    {
      params[i] = 0;
    }

    params[numParams - 1] = 1 << CCCM_DECIM_BITS; // Default bias to 1
  }

  Pel convolve(Pel* vector, int numParams)
  {
    TCccmCoeff sum = 0;
    
    for( int i = 0; i < numParams; i++)
    {
      sum += params[i] * vector[i];
    }

    return Pel( (sum + CCCM_DECIM_ROUND ) >> CCCM_DECIM_BITS );
  }
  
  Pel nonlinear(const Pel val) { return (val * val + midVal) >> bd; }
  Pel bias     ()              { return midVal; }
};

struct CccmCovarianceInt
{
  using TE = TCccmCoeff[CCCM_NUM_PARAMS][CCCM_NUM_PARAMS];
  using Ty = TCccmCoeff[CCCM_NUM_PARAMS];

  CccmCovarianceInt() {}
  ~CccmCovarianceInt() {}

  bool ldlDecompose                (TE A, TE U,          Ty diag,                      int numEq) const;
  void ldlSolve                    (TE U, Ty diag,       TCccmCoeff* y, TCccmCoeff* x, int numEq, bool decompOk) const;

private:
  void ldlBacksubstitution         (TE U, TCccmCoeff* z, TCccmCoeff* x, int numEq) const;
  void ldlTransposeBacksubstitution(TE U, TCccmCoeff* y, TCccmCoeff* z, int numEq) const;
  bool ldlDecomp                   (TE A, TE U,          Ty outDiag,    int numEq) const;
};
#endif

class IntraPrediction
{
#if MMLM
public:
  bool m_encPreRDRun;
#endif
protected:
  Pel      m_refBuffer[MAX_NUM_COMPONENT][NUM_PRED_BUF][(MAX_CU_SIZE * 2 + 1 + MAX_REF_LINE_IDX) * 2];
  uint32_t m_refBufferStride[MAX_NUM_COMPONENT];

private:

#if !MERGE_ENC_OPT
  Pel* m_yuvExt2[MAX_NUM_COMPONENT][4];
  int  m_yuvExtSize2;
#endif

#if JVET_AA0057_CCCM
  Area m_cccmRefArea;
  Pel* m_cccmLumaBuf;
#endif
  
  static const uint8_t m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS];
#if JVET_W0123_TIMD_FUSION
  static const uint8_t m_aucIntraFilterExt[MAX_INTRA_FILTER_DEPTHS];
  RdCost* m_timdSatdCost;
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
    // clang-format on
    {
    }
  };

  IntraPredParam m_ipaParam;

  Pel* m_piTemp;
  Pel* m_pMdlmTemp; // for MDLM mode
#if JVET_AA0126_GLM
  Pel* m_glmTempCb[NUM_GLM_IDC];
  Pel* m_glmTempCr[NUM_GLM_IDC];
#endif
  MatrixIntraPrediction m_matrixIntraPred;



protected:
  ChromaFormat  m_currChromaFormat;

  int m_topRefLength;
  int m_leftRefLength;
  ScanElement* m_scanOrder;
  bool         m_bestScanRotationMode;
  std::vector<PelStorage>   m_tempBuffer;

#if JVET_V0130_INTRA_TMP
  int          m_uiPartLibSize;
  TempLibFast  m_tempLibFast;
  Pel*         m_refPicUsed;
  Picture*     m_refPicBuf;
  unsigned int m_uiPicStride;
  unsigned int m_uiVaildCandiNum;
  Pel***       m_pppTarPatch;
#endif

  // prediction
  void xPredIntraPlanar           ( const CPelBuf &pSrc, PelBuf &pDst );
  void xPredIntraDc               ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter = true );
#if JVET_W0123_TIMD_FUSION
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir);
#else
  void xPredIntraAng              ( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng);
#endif

  void initPredIntraParams        ( const PredictionUnit & pu,  const CompArea compArea, const SPS& sps );

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
  static int getWideAngleExt      ( int width, int height, int predMode );
#endif
  void setReferenceArrayLengths   ( const CompArea &area );

  void destroy                    ();
#if LMS_LINEAR_MODEL
  void xPadMdlmTemplateSample     (Pel*pSrc, Pel*pCur, int cWidth, int cHeight, int existSampNum, int targetSampNum);
  void xGetLMParametersLMS        (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel);
#else
  void xGetLMParameters           (const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel);
#endif
#if LMS_LINEAR_MODEL && MMLM
  struct MMLM_parameter
  {
    int a;
    int b;
    int shift;
  };
  int xCalcLMParametersGeneralized(int x, int y, int xx, int xy, int count, int bitDepth, int &a, int &b, int &iShift);
  int xLMSampleClassifiedTraining (int count, int mean, int meanC, int LumaSamples[], int ChrmSamples[], int bitDepth, MMLM_parameter parameters[]);
#endif
#if JVET_Z0050_CCLM_SLOPE
  void xUpdateCclmModel           (int &a, int &b, int &iShift, int midLuma, int delta);
#endif

public:
  IntraPrediction();
  virtual ~IntraPrediction();

  void init                       (ChromaFormat chromaFormatIDC, const unsigned bitDepthY);

#if JVET_AA0057_CCCM
  void   predIntraCCCM            (const PredictionUnit& pu, PelBuf &predCb, PelBuf &predCr, int intraDir);
  void   xCccmCalcModels          (const PredictionUnit& pu, CccmModel &cccmModelCb, CccmModel &cccmModelCr, int modelId, int modelThr) const;
  void   xCccmApplyModel          (const PredictionUnit& pu, const ComponentID compId, CccmModel &cccmModel, int modelId, int modelThr, PelBuf &piPred) const;
  void   xCccmCreateLumaRef       (const PredictionUnit& pu);
  PelBuf xCccmGetLumaRefBuf       (const PredictionUnit& pu, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY) const;
  PelBuf xCccmGetLumaPuBuf        (const PredictionUnit& pu) const;
  Pel    xCccmGetLumaVal          (const PredictionUnit& pu, const CPelBuf pi, const int x, const int y) const;
  int    xCccmCalcRefAver         (const PredictionUnit& pu) const;
  void   xCccmCalcRefArea         (const PredictionUnit& pu);
#endif
#if ENABLE_DIMD
  static void deriveDimdMode      (const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu);
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  static void deriveDimdChromaMode(const CPelBuf &recoBufY, const CPelBuf &recoBufCb, const CPelBuf &recoBufCr, const CompArea &areaY, const CompArea &areaCb, const CompArea &areaCr, CodingUnit &cu);
#endif
  static int  buildHistogram      ( const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int* piHistogram, int direction, int bw, int bh );
#endif
#if JVET_W0123_TIMD_FUSION
  void xIntraPredTimdHorVerPdpc   (Pel* pDsty,const int dstStride, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, const Pel* refMain, const ClpRng& clpRng);
  void xPredTimdIntraPlanar       (const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TEMPLATE_TYPE eTempType, int iTemplateWidth , int iTemplateHeight);
  void xPredTimdIntraDc           ( const PredictionUnit &pu, const CPelBuf &pSrc, Pel* pDst, int iDstStride, int iWidth, int iHeight, TEMPLATE_TYPE eTempType, int iTemplateWidth , int iTemplateHeight);
  void xPredTimdIntraAng          ( const CPelBuf &pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride, int iWidth, int iHeight, TEMPLATE_TYPE eTempType, int iTemplateWidth , int iTemplateHeight, uint32_t dirMode);
  void xIntraPredTimdAngLuma(Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset);
  void xIntraPredTimdPlanarDcPdpc (const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TEMPLATE_TYPE eTempType, int iTemplateWidth , int iTemplateHeight);
  void xIntraPredTimdAngPdpc(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height, int xOffset, int yOffset, int scale, int invAngle);
  void xFillTimdReferenceSamples  ( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu, int iTemplateWidth, int iTemplateHeight );
  Pel  xGetPredTimdValDc          ( const CPelBuf &pSrc, const Size &dstSize, TEMPLATE_TYPE eTempType, int iTempHeight, int iTempWidth );
  void initPredTimdIntraParams    (const PredictionUnit & pu, const CompArea area, int dirMode);
  void predTimdIntraAng           ( const ComponentID compId, const PredictionUnit &pu, uint32_t uiDirMode, Pel* pPred, uint32_t uiStride, uint32_t iWidth, uint32_t iHeight, TEMPLATE_TYPE eTempType, int32_t iTemplateWidth, int32_t iTemplateHeight);
  int deriveTimdMode              ( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu );
  void initTimdIntraPatternLuma   (const CodingUnit &cu, const CompArea &area, int iTemplateWidth, int iTemplateHeight, uint32_t uiRefWidth, uint32_t uiRefHeight);
#if GRAD_PDPC
  void xIntraPredTimdAngGradPdpc  (Pel* pDsty, const int dstStride, Pel* refMain, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, int deltaPos, int intraPredAngle, const ClpRng& clpRng);
#endif
#if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
  void xIntraPredPlanarDcPdpc     (const CPelBuf &pSrc, Pel *pDst, int iDstStride, int width, int height, bool ciipPDPC);
#else
  void xIntraPredPlanarDcPdpc( const CPelBuf &pSrc, Pel *pDst, int iDstStride, int width, int height );
#endif
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING && JVET_Y0065_GPM_INTRA
protected:
  bool    m_abFilledIntraGPMRefTpl[NUM_INTRA_MODE];
  uint8_t m_aiGpmIntraMPMLists[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS];   //[][0][]: part0, [][1][]: part1
  Pel     m_acYuvRefGPMIntraTemplate[NUM_INTRA_MODE][2][GEO_MAX_CU_SIZE * GEO_MODE_SEL_TM_SIZE];   //[][0][]: top, [][1][]: left

  template <uint8_t partIdx>
  bool xFillIntraGPMRefTemplateAll(PredictionUnit& pu, TEMPLATE_TYPE eTempType, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut = nullptr, uint8_t candIdx = std::numeric_limits<uint8_t>::max());
  bool xFillIntraGPMRefTemplate   (PredictionUnit& pu, TEMPLATE_TYPE eTempType, uint8_t intraMode, bool loadIntraRef, Pel* bufTop, Pel* bufLeft, std::vector<Pel>* lut = nullptr);

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
  void predIntraAng               ( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
  Pel *getPredictorPtr(const ComponentID compId)
  {
    return m_refBuffer[compId][m_ipaParam.refFilterFlag ? PRED_BUF_FILTERED : PRED_BUF_UNFILTERED];
  }

  // Cross-component Chroma
  void predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel = true, CclmModel *cclmModelStored = nullptr);
  void xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea);
#if JVET_AA0126_GLM
  void xGetLumaRecPixelsGlmAll(const PredictionUnit &pu, CompArea chromaArea);
  Pel xGlmGetLumaVal    (const int s[6], const int c[6], const int glmIdx, const Pel val) const;
#endif
  /// set parameters from CU data for accessing intra data
  void initIntraPatternChType     (const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers
  void initIntraPatternChTypeISP  (const CodingUnit& cu, const CompArea& area, PelBuf& piReco, const bool forceRefFilterFlag = false); // use forceRefFilterFlag to get both filtered and unfiltered buffers

  // Matrix-based intra prediction
  void initIntraMip               (const PredictionUnit &pu, const CompArea &area);
  void predIntraMip               (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);

  template<bool lmcs>
  void geneWeightedPred           ( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT = nullptr );
  void geneIntrainterPred         (const CodingUnit &cu, PelStorage& pred);
#if JVET_Z0050_DIMD_CHROMA_FUSION
  void geneChromaFusionPred       (const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu);
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
  int( *m_calcTemplateDiff )      ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType TempType );
  static int calcTemplateDiff     ( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType TempType );
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
  void searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, unsigned int setId, RefTemplateType tempType );
  void candidateSearchIntra          ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType );
#else
  void searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, unsigned int setId );
  void candidateSearchIntra          ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
#endif
  bool generateTMPrediction          ( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int& foundCandiNum );
#if JVET_W0069_TMP_BOUNDARY
  bool generateTmDcPrediction        ( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val );
  void getTargetTemplate             ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType );
#else
  void getTargetTemplate             ( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight );
#endif
#endif

#ifdef TARGET_SIMD_X86
  void    initIntraX86();
  template <X86_VEXT vext>
  void    _initIntraX86();
#endif
};
//! \}

#endif // __INTRAPREDICTION__
