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

/** \file     Prediction.cpp
    \brief    prediction class
*/

#include "IntraPrediction.h"

#include "Unit.h"
#include "UnitTools.h"
#include "Buffer.h"

#include "dtrace_next.h"
#include "Rom.h"

#include <memory.h>
#if INTRA_TRANS_ENC_OPT && defined(TARGET_SIMD_X86)
#include <smmintrin.h>
#endif
#include "CommonLib/InterpolationFilter.h"

//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================

const uint8_t IntraPrediction::m_aucIntraFilter[MAX_INTRA_FILTER_DEPTHS] =
{
  24, //   1xn
  24, //   2xn
  24, //   4xn
  14, //   8xn
  2,  //  16xn
  0,  //  32xn
  0,  //  64xn
  0   // 128xn
};

#if JVET_W0123_TIMD_FUSION
const uint8_t IntraPrediction::m_aucIntraFilterExt[MAX_INTRA_FILTER_DEPTHS] =
{
  48, //   1xn
  48, //   2xn
  48, //   4xn
  28, //   8xn
  4,  //  16xn
  0,  //  32xn
  0,  //  64xn
  0   // 128xn
};
#endif

// ====================================================================================================================
// Constructor / destructor / initialize
// ====================================================================================================================

IntraPrediction::IntraPrediction()
:
  m_currChromaFormat( NUM_CHROMA_FORMAT )
{
#if !MERGE_ENC_OPT
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t buf = 0; buf < 4; buf++ )
    {
      m_yuvExt2[ch][buf] = nullptr;
    }
  }
#endif

#if JVET_AD0120_LBCCP
  m_pCCFilterTemp = nullptr;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  m_pCcpMerge[0] = nullptr;
  m_pCcpMerge[1] = nullptr;
#endif
#if JVET_W0123_TIMD_FUSION
  m_timdSatdCost = nullptr;
#endif
#if JVET_AC0071_DBV
  m_dbvSadCost = nullptr;
#endif
  #if JVET_AB0067_MIP_DIMD_LFNST
  m_pMipTemp = nullptr;
#endif
  m_piTemp = nullptr;
  m_pMdlmTemp = nullptr;
#if JVET_AA0126_GLM
  for (int i = 0; i < NUM_GLM_IDC; i++)
  {
    m_glmTempCb[i] = nullptr;
    m_glmTempCr[i] = nullptr;
#if JVET_AB0092_GLM_WITH_LUMA
    m_glmGradBuf[i] = nullptr;
#endif
  }
#endif
#if MMLM
  m_encPreRDRun = false;
#endif
#if JVET_V0130_INTRA_TMP
  m_pppTarPatch = NULL;
#endif
#if JVET_AA0057_CCCM
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  for (int i = 0; i < (CCCM_NUM_PRED_FILTER + 1); i++)
  {
    m_cccmLumaBuf[i] = nullptr;
  }
#else
  m_cccmLumaBuf[0] = nullptr;
  m_cccmLumaBuf[1] = nullptr;
#endif
#else
  m_cccmLumaBuf = nullptr;
#endif
#if JVET_AE0100_BVGCCCM
  for (int i = 0; i < NUM_BVG_CCCM_CANDS; i++)
  {
    m_bvgCccmLumaBuf[i] = nullptr;
    m_bvgCccmChromaBuf[i][0] = nullptr;
    m_bvgCccmChromaBuf[i][1] = nullptr;
  }
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  for (int j = 0; j < MTMP_NUM; j++)
  {
    m_tmpRefBuf[j] = nullptr;
  }
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
  for (int i = 0; i < 3; i++)
  {
    m_cflmBuf[i] = nullptr;
  }
#endif
}

IntraPrediction::~IntraPrediction()
{
  destroy();
}

void IntraPrediction::destroy()
{
#if !MERGE_ENC_OPT
  for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
  {
    for( uint32_t buf = 0; buf < 4; buf++ )
    {
      delete[] m_yuvExt2[ch][buf];
      m_yuvExt2[ch][buf] = nullptr;
    }
  }
#endif

#if JVET_W0123_TIMD_FUSION
  delete m_timdSatdCost;
#endif
#if JVET_AC0071_DBV
  delete m_dbvSadCost;
#endif
#if JVET_AB0155_SGPM
  for (auto &buffer: m_sgpmBuffer)
  {
    buffer.destroy();
  }
  m_sgpmBuffer.clear();
#endif
  for (auto &buffer: m_intraPredBuffer)
  {
    buffer.destroy();
  }
  m_intraPredBuffer.clear();

#if JVET_AD0120_LBCCP
  delete[] m_pCCFilterTemp;
  m_pCCFilterTemp = nullptr;
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  delete[] m_pCcpMerge[0];
  m_pCcpMerge[0] = nullptr;
  delete[] m_pCcpMerge[1];
  m_pCcpMerge[1] = nullptr;
#endif
#if JVET_AB0067_MIP_DIMD_LFNST
  delete[] m_pMipTemp;
  m_pMipTemp = nullptr;
#endif
  delete[] m_piTemp;
  m_piTemp = nullptr;
  delete[] m_pMdlmTemp;
  m_pMdlmTemp = nullptr;
#if JVET_AA0126_GLM
  for (int i = 0; i < NUM_GLM_IDC; i++)
  {
    delete[] m_glmTempCb[i]; m_glmTempCb[i] = nullptr;
    delete[] m_glmTempCr[i]; m_glmTempCr[i] = nullptr;
#if JVET_AB0092_GLM_WITH_LUMA
    delete[] m_glmGradBuf[i]; m_glmGradBuf[i] = nullptr;
#endif
  }
#endif

  for( auto &buffer : m_tempBuffer )
  {
    buffer.destroy();
  }
  m_tempBuffer.clear();

#if JVET_V0130_INTRA_TMP
  if( m_pppTarPatch != NULL )
  {
    for( unsigned int uiDepth = 0; uiDepth < USE_MORE_BLOCKSIZE_DEPTH_MAX; uiDepth++ )
    {
      unsigned int blkSize = g_uiDepth2Width[uiDepth];

      unsigned int patchSize = blkSize + TMP_TEMPLATE_SIZE;
      for( unsigned int uiRow = 0; uiRow < patchSize; uiRow++ )
      {
        if( m_pppTarPatch[uiDepth][uiRow] != NULL )
        {
          delete[]m_pppTarPatch[uiDepth][uiRow]; m_pppTarPatch[uiDepth][uiRow] = NULL;
        }
      }
      if( m_pppTarPatch[uiDepth] != NULL )
      {
        delete[]m_pppTarPatch[uiDepth]; m_pppTarPatch[uiDepth] = NULL;
      }
    }
    delete[] m_pppTarPatch;
    m_pppTarPatch = NULL;
  }
#endif

#if JVET_AA0057_CCCM
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  for (int i = 0; i < (CCCM_NUM_PRED_FILTER + 1); i++)
  {
    delete[] m_cccmLumaBuf[i];
    m_cccmLumaBuf[i] = nullptr;
  }
#else
  delete[] m_cccmLumaBuf[0];
  m_cccmLumaBuf[0] = nullptr;

  delete[] m_cccmLumaBuf[1];
  m_cccmLumaBuf[1] = nullptr;
#endif
#else
  delete[] m_cccmLumaBuf;
  m_cccmLumaBuf = nullptr;
#endif
#if JVET_AE0100_BVGCCCM
  for (int i = 0; i < NUM_BVG_CCCM_CANDS; i++)
  {
    delete[] m_bvgCccmLumaBuf[i];
    m_bvgCccmLumaBuf[i] = nullptr;
    delete[] m_bvgCccmChromaBuf[i][0];
    m_bvgCccmChromaBuf[i][0] = nullptr;
    delete[] m_bvgCccmChromaBuf[i][1];
    m_bvgCccmChromaBuf[i][1] = nullptr;
  }
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  for (int j = 0; j < MTMP_NUM; j++)
  {
    delete[] m_tmpRefBuf[j];
    m_tmpRefBuf[j] = nullptr;
  }
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
  for (int i = 0; i < 3; i++)
  {
    delete[] m_cflmBuf[i];
    m_cflmBuf[i] = nullptr;
  }
#endif
}

void IntraPrediction::init(ChromaFormat chromaFormatIDC, const unsigned bitDepthY)
{
#if JVET_AB0155_SGPM
  m_if.initInterpolationFilter(true);
#endif

#if MERGE_ENC_OPT
  if (m_currChromaFormat != chromaFormatIDC)
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;
#else
  if( m_yuvExt2[COMPONENT_Y][0] != nullptr && m_currChromaFormat != chromaFormatIDC )
  {
    destroy();
  }

  m_currChromaFormat = chromaFormatIDC;


  if( m_yuvExt2[COMPONENT_Y][0] == nullptr ) // check if first is null (in which case, nothing initialised yet)
  {
    m_yuvExtSize2 = ( MAX_CU_SIZE ) * ( MAX_CU_SIZE );

    for( uint32_t ch = 0; ch < MAX_NUM_COMPONENT; ch++ )
    {
      for( uint32_t buf = 0; buf < 4; buf++ )
      {
        m_yuvExt2[ch][buf] = new Pel[m_yuvExtSize2];
      }
    }
  }
#endif

#if LMS_LINEAR_MODEL
  int shift = bitDepthY + 4;
  for (int i = 32; i < 64; i++)
  {
    m_auShiftLM[i - 32] = ((1 << shift) + i / 2) / i;
  }
#endif

#if JVET_W0123_TIMD_FUSION
  if (m_timdSatdCost == nullptr)
  {
    m_timdSatdCost = new RdCost;
  }
#endif
#if JVET_AC0071_DBV
  if (m_dbvSadCost == nullptr)
  {
    m_dbvSadCost = new RdCost;
  }
#endif
#if JVET_AB0155_SGPM
  for (auto &buffer: m_sgpmBuffer)
  {
    buffer.destroy();
  }

  // the number of total temporal buffers can be adjusted by changing the number here
  m_sgpmBuffer.resize(1);

  for (auto &buffer: m_sgpmBuffer)
  {
    buffer.create(CHROMA_400, Area(0, 0, MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE, MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE));
  }
#endif
  for (auto &buffer: m_intraPredBuffer)
  {
    buffer.destroy();
  }

  // the number of total temporal buffers can be adjusted by changing the number here
  m_intraPredBuffer.resize(1);

  for (auto &buffer: m_intraPredBuffer)
  {
    buffer.create(CHROMA_400, Area(0, 0, MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE, MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE));
  }

  memset(tempRefAbove, 0, ((MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX) * sizeof(Pel));
  memset(tempRefLeft, 0, ((MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX) * sizeof(Pel));

#if JVET_AD0120_LBCCP
  if (m_pCCFilterTemp == nullptr)
  {
    m_pCCFilterTemp = new Pel[(MAX_CU_SIZE + 2) * (MAX_CU_SIZE + 2)];
  }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  if (m_pCcpMerge[0] == nullptr)
  {
    m_pCcpMerge[0] = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
  if (m_pCcpMerge[1] == nullptr)
  {
    m_pCcpMerge[1] = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }  
#endif
#if JVET_AB0067_MIP_DIMD_LFNST
  if (m_pMipTemp == nullptr)
  {
    m_pMipTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
#endif
  if (m_piTemp == nullptr)
  {
    m_piTemp = new Pel[(MAX_CU_SIZE + 1) * (MAX_CU_SIZE + 1)];
  }
  if (m_pMdlmTemp == nullptr)
  {
    m_pMdlmTemp = new Pel[(2 * MAX_CU_SIZE + 1)*(2 * MAX_CU_SIZE + 1)];//MDLM will use top-above and left-below samples.
  }
#if JVET_AA0126_GLM
  for (int i = 0; i < NUM_GLM_IDC; i++)
  {
    if (m_glmTempCb[i] == nullptr)
    {
      m_glmTempCb[i] = new Pel[(2 * MAX_CU_SIZE + 1)*(2 * MAX_CU_SIZE + 1)];
    }
    if (m_glmTempCr[i] == nullptr)
    {
      m_glmTempCr[i] = new Pel[(2 * MAX_CU_SIZE + 1)*(2 * MAX_CU_SIZE + 1)];
    }
#if JVET_AB0092_GLM_WITH_LUMA
    if (m_glmGradBuf[i] == nullptr)
    {
      m_glmGradBuf[i] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE)];
    }
#endif
  }
#endif

  for( auto &buffer : m_tempBuffer )
  {
    buffer.destroy();
  }

  // the number of total temporal buffers can be adjusted by chaning the number here
#if JVET_AB0157_INTRA_FUSION
#if JVET_AC0098_LOC_DEP_DIMD
  m_tempBuffer.resize( DIMD_FUSION_NUM+2 );
#else
  m_tempBuffer.resize( DIMD_FUSION_NUM-1 );
#endif
#else
  m_tempBuffer.resize( 2 );
#endif

  for( auto &buffer : m_tempBuffer )
  {
    buffer.create( chromaFormatIDC, Area( 0, 0, MAX_CU_SIZE, MAX_CU_SIZE ) );
  }
#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
  m_dimdBlending = dimdBlending;
#endif
#if JVET_W0123_TIMD_FUSION && INTRA_TRANS_ENC_OPT
  m_timdBlending = timdBlending;
#endif
#if JVET_AC0112_IBC_CIIP && INTRA_TRANS_ENC_OPT
  m_ibcCiipBlending = ibcCiipBlending;
#endif
#if JVET_V0130_INTRA_TMP
  unsigned int blkSize;
  if( m_pppTarPatch == NULL )
  {
    m_pppTarPatch = new Pel * *[USE_MORE_BLOCKSIZE_DEPTH_MAX];
    for( unsigned int uiDepth = 0; uiDepth < USE_MORE_BLOCKSIZE_DEPTH_MAX; uiDepth++ )
    {
      blkSize = g_uiDepth2Width[uiDepth];

      unsigned int patchSize = blkSize + TMP_TEMPLATE_SIZE;
      m_pppTarPatch[uiDepth] = new Pel *[patchSize];
      for( unsigned int uiRow = 0; uiRow < patchSize; uiRow++ )
      {
        m_pppTarPatch[uiDepth][uiRow] = new Pel[patchSize];
      }
    }
  }

  m_calcTemplateDiff = calcTemplateDiff;
#endif

#if JVET_AA0057_CCCM
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  if (m_cccmLumaBuf[0] == nullptr)
  {
    const int chromaScaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, chromaFormatIDC);
    const int chromaScaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, chromaFormatIDC);

    m_cccmLumaBuf[0] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + (CCCM_FILTER_PADDING << chromaScaleX) + (CCCM_FILTER_PADDING << chromaScaleY)) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + (CCCM_FILTER_PADDING << chromaScaleX) + (CCCM_FILTER_PADDING << chromaScaleY))];
  }

  for (int i = 1; i < (CCCM_NUM_PRED_FILTER + 1); i++)
  {
    if (m_cccmLumaBuf[i] == nullptr)
    {
      m_cccmLumaBuf[i] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
  }
#else
  if( m_cccmLumaBuf[0] == nullptr )
  {
    m_cccmLumaBuf[0] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
  }

  if( m_cccmLumaBuf[1] == nullptr )
  {
    const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, chromaFormatIDC );
    const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, chromaFormatIDC );

    m_cccmLumaBuf[1] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + (CCCM_FILTER_PADDING << chromaScaleX) + (CCCM_FILTER_PADDING << chromaScaleY)) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + (CCCM_FILTER_PADDING << chromaScaleX) + (CCCM_FILTER_PADDING << chromaScaleY))];
  }
#endif
#else
  if (m_cccmLumaBuf == nullptr)
  {
    m_cccmLumaBuf = new Pel[(2*MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2*CCCM_FILTER_PADDING) * (2*MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2*CCCM_FILTER_PADDING)];
  }
#endif
#if JVET_AE0100_BVGCCCM
  for (int i = 0; i < NUM_BVG_CCCM_CANDS; i++)
  {
    if( m_bvgCccmLumaBuf[i] == nullptr )
    {
      m_bvgCccmLumaBuf[i] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
    if (m_bvgCccmChromaBuf[i][0] == nullptr)
    {
      m_bvgCccmChromaBuf[i][0] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
    if (m_bvgCccmChromaBuf[i][1] == nullptr)
    {
      m_bvgCccmChromaBuf[i][1] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE + 2 * CCCM_FILTER_PADDING)];
    }
  }
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  for (int j = 0; j < MTMP_NUM; j++)
  {
    if (m_tmpRefBuf[j] == nullptr)
    {
      m_tmpRefBuf[j] = new Pel[(2 * MAX_CU_SIZE + TMP_TEMPLATE_SIZE + 2 * TMP_FILTER_PADDING)
                               * (2 * MAX_CU_SIZE + TMP_TEMPLATE_SIZE + 2 * TMP_FILTER_PADDING)];
    }
  }
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
  for (int i = 0; i < 3; i++)
  {
    if (m_cflmBuf[i] == nullptr)
    {
      m_cflmBuf[i] = new Pel[(2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE) * (2 * MAX_CU_SIZE + CCCM_WINDOW_SIZE)];
    }
  }
#endif

#if ENABLE_SIMD_TMP
#ifdef TARGET_SIMD_X86
  initIntraX86();
#endif
#endif
}

#if JVET_W0123_TIMD_FUSION || JVET_AC0119_LM_CHROMA_FUSION
void IntraPrediction::xIntraPredTimdAngPdpc(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height, int xOffset, int yOffset, int scale,int invAngle)
{
  int xlim = std::min(3 << scale, width);
  for (int y = yOffset; y<height; y++)
  {
    int invAngleSum = 256;
    if (width < 4)
    {
      for (int x = xOffset; x < 2; x++)
      {
        invAngleSum += invAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
    else
    {
      for (int x = xOffset; x < xlim; x++)
      {
        invAngleSum += invAngle;
        int wL   = 32 >> (2 * x >> scale);
        Pel left = refSide[y + (invAngleSum >> 9) + 1];
        pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
      }
    }
    pDsty += dstStride;
  }
}

#if GRAD_PDPC
void IntraPrediction::xIntraPredTimdAngGradPdpc(Pel* pDsty, const int dstStride, Pel* refMain, Pel* refSide, const int width, const int height, int xOffset, int yOffset, int scale, int deltaPos, int intraPredAngle, const ClpRng& clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
  , const bool bExtIntraDir
#endif
)
{
#if JVET_AC0119_LM_CHROMA_FUSION
  if (!bExtIntraDir)
  {
    for (int y = yOffset; y < height; y++)
    {
      const int deltaInt = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;
      const Pel left = refSide[1 + y];
      const Pel topLeft = refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + 16) >> 5);
      for (int x = xOffset; x < std::min(3 << scale, width); x++)
      {
        int wL = 32 >> (2 * (x - xOffset) >> scale);
        pDsty[x] = ClipPel(pDsty[x] + ((wL * (left - topLeft) + 32) >> 6), clpRng);
      }
      pDsty += dstStride;
      deltaPos += intraPredAngle;
    }
  }
  else
  {
#endif
  for (int y = yOffset; y<height; y++)
  {
    const int deltaInt   = deltaPos >> 6;
    const int deltaFract = deltaPos & 63;
    const Pel left = refSide[1 + y];
    const Pel topLeft = refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + 32) >> 6);
    for (int x = xOffset; x < std::min(3 << scale, width); x++)
    {
      int wL = 32 >> (2 * (x - xOffset) >> scale);
      pDsty[x] = ClipPel(pDsty[x] + ((wL * (left - topLeft) + 32) >> 6), clpRng);
    }
    pDsty += dstStride;
    deltaPos += intraPredAngle;
  }
#if JVET_AC0119_LM_CHROMA_FUSION
  }
#endif
}
#endif

void IntraPrediction::xIntraPredTimdHorVerPdpc(Pel* pDsty,const int dstStride,Pel* refSide,const int width,const int height, int xOffset, int yOffset, int scale,const Pel* refMain, const ClpRng& clpRng)
{
  const Pel topLeft = refMain[0];

  for( int y = yOffset; y < height; y++ )
  {
    memcpy(pDsty,&refMain[1],width*sizeof(Pel));
    const Pel left    = refSide[1 + y];
    for (int x = xOffset; x < std::min(3 << scale, width); x++)
    {
      const int wL  = 32 >> (2 * x >> scale);
      const Pel val = pDsty[x];
      pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
    }
    pDsty += dstStride;
  }
}

void IntraPrediction::xIntraPredTimdPlanarDcPdpc(const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, TemplateType eTempType, int iTemplateWidth, int iTemplateHeight)
{
  if (eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    int xOffset = 0;
    int yOffset = 0;
    // PDPC for above template
    {
      const int iWidth  = width;
      const int iHeight = iTemplateHeight;
      xOffset = iTemplateWidth;
      const int scale = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
      for (int y = 0; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = pSrc.at(y + 1, 1);
        for (int x = xOffset; x < iWidth; x++)
        {
          const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top   = pSrc.at(x + 1, 0);
          const Pel val   = pDst[y * iDstStride + x];
          pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        }
      }
    }

    // PDPC for left template
    {
      const int iWidth  = iTemplateWidth;
      const int iHeight = height;
      yOffset = iTemplateHeight;
      const int scale = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
      for (int y = yOffset; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = pSrc.at(y + 1, 1);
        for (int x = 0; x < iWidth; x++)
        {
          const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top   = pSrc.at(x + 1, 0);
          const Pel val   = pDst[y * iDstStride + x];
          pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        }
      }
    }
  }
  else if (eTempType == LEFT_NEIGHBOR)
  {
    const int iHeight = height;
    const int scale = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
    for (int y = 0; y < iHeight; y++)
    {
      const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = pSrc.at(y + 1, 1);
      for (int x = 0; x < iTemplateWidth; x++)
      {
        const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top   = pSrc.at(x + 1, 0);
        const Pel val   = pDst[y * iDstStride + x];
        pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
      }
    }
  }
  else // eTempType == ABOVE_NEIGHBOR
  {
    const int iWidth  = width;
    const int scale = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
    for (int y = 0; y < iTemplateHeight; y++)
    {
      const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel left = pSrc.at(y + 1, 1);
      for (int x = 0; x < iWidth; x++)
      {
        const int wL    = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top   = pSrc.at(x + 1, 0);
        const Pel val   = pDst[y * iDstStride + x];
        pDst[y * iDstStride + x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
      }
    }
  }
}

void IntraPrediction::xIntraPredTimdAngLuma(Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset)
{
  for (int y = yOffset; y<height; y++ )
  {
    const int deltaInt   = deltaPos >> 6;
    const int deltaFract = deltaPos & 63;
    const TFilterCoeff* const f = InterpolationFilter::getExtIntraCubicFilter(deltaFract);
    int refMainIndex = deltaInt + 1 + xOffset;
    for( int x = xOffset; x < width; x++, refMainIndex++ )
    {
      pDstBuf[y*dstStride + x] = (f[0] * refMain[refMainIndex - 1] + f[1] * refMain[refMainIndex] + f[2] * refMain[refMainIndex + 1] + f[3] * refMain[refMainIndex + 2] + 128) >> 8;
      pDstBuf[y*dstStride + x] = ClipPel( pDstBuf[y*dstStride + x], clpRng ); // always clip even though not always needed
    }
    deltaPos += intraPredAngle;
  }
}

#if JVET_AC0119_LM_CHROMA_FUSION
void IntraPrediction::xIntraPredTimdAngChroma(Pel* pDstBuf, const ptrdiff_t dstStride, Pel* refMain, int width, int height, int deltaPos, int intraPredAngle, const ClpRng& clpRng, int xOffset, int yOffset)
{
  for (int y = yOffset; y < height; y++)
  {
    const int deltaInt = deltaPos >> 5;
    const int deltaFract = deltaPos & 31;
    int refMainIndex = deltaInt + 1 + xOffset;
    for (int x = xOffset; x < width; x++, refMainIndex++)
    {
      pDstBuf[y * dstStride + x] = refMain[refMainIndex] + ((deltaFract * (refMain[refMainIndex + 1] - refMain[refMainIndex]) + 16) >> 5);
    }
    deltaPos += intraPredAngle;
}
}
#endif

#if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
void IntraPrediction::xIntraPredPlanarDcPdpc( const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height, bool ciipPDPC )
#else
void IntraPrediction::xIntraPredPlanarDcPdpc(const CPelBuf &pSrc, Pel* pDst, int iDstStride, int width, int height)
#endif
{
  const int iWidth  = width;
  const int iHeight = height;
  const int scale  = ((floorLog2(iWidth) - 2 + floorLog2(iHeight) - 2 + 2) >> 2);
  CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");
  const Pel *srcLeft = pSrc.bufAt(1, 1);
#if CIIP_PDPC
  if (ciipPDPC)
  {
    for (int y = 0; y < iHeight; y++)
    {
      const int  wT     = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel  left    = *srcLeft;
      const Pel *srcTop = pSrc.buf + 1;
      for (int x = 0; x < iWidth; x++)
      {
        const int wL  = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top = *srcTop;
        pDst[x]        = ((wL * left + wT * top + 32) >> 6);

        srcTop++;
      }
      srcLeft++;
      pDst += iDstStride;
    }
  }
  else
  {
#endif
    for (int y = 0; y < iHeight; y++)
    {
      const int  wT     = 32 >> std::min(31, ((y << 1) >> scale));
      const Pel  left   = *srcLeft;
      const Pel *srcTop = pSrc.buf + 1;

      for (int x = 0; x < iWidth; x++)
      {
        const int wL  = 32 >> std::min(31, ((x << 1) >> scale));
        const Pel top = *srcTop;
        const Pel val = pDst[x];

        pDst[x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
        srcTop++;
      }

      srcLeft++;
      pDst += iDstStride;
    }
#if CIIP_PDPC
  }
#endif
}
#endif
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING && JVET_Y0065_GPM_INTRA
template <uint8_t partIdx>
bool IntraPrediction::xFillIntraGPMRefTemplateAll(PredictionUnit& pu, TemplateType eTempType, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut, uint8_t candIdx)
{
#if JVET_AG0164_AFFINE_GPM
  if (eTempType == NO_NEIGHBOR || candIdx < GEO_MAX_ALL_INTER_UNI_CANDS)
  {
    return false;
  }
#else
  if (eTempType == NO_NEIGHBOR || candIdx < GEO_MAX_NUM_UNI_CANDS)
  {
    return false;
  }
#endif
  doInitMPMList &= !readBufferedMPMList; // No MPM list derivation needed, since MPM list is read from buffer assuming it is already buffered
  const bool doInitAL = true;
  const bool doInitA  = partIdx == 0;
  const bool doInitL  = partIdx == 1;
#if JVET_AG0164_AFFINE_GPM
  uint8_t startIdx = candIdx == std::numeric_limits<uint8_t>::max() ? 0                       : (candIdx - GEO_MAX_ALL_INTER_UNI_CANDS);
#else
  uint8_t startIdx = candIdx == std::numeric_limits<uint8_t>::max() ? 0                       : (candIdx - GEO_MAX_NUM_UNI_CANDS);
#endif
  uint8_t endIdx   = candIdx == std::numeric_limits<uint8_t>::max() ? GEO_MAX_NUM_INTRA_CANDS : (startIdx + 1                   );
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    uint8_t* geoIntraMPMList = m_aiGpmIntraMPMLists[splitDir][partIdx];
    if (!readBufferedMPMList)
    {
      PU::getGeoIntraMPMs(pu, geoIntraMPMList, splitDir, g_geoTmShape[partIdx][g_geoParams[splitDir][0]], doInitMPMList, doInitAL, doInitA, doInitL);
      doInitMPMList = false; // to prevent duplicating initialization
    }

    for (uint8_t tmpCandIdx = startIdx; tmpCandIdx < endIdx; ++tmpCandIdx)
    {
      uint8_t intraMode = geoIntraMPMList[tmpCandIdx];
      if (!m_abFilledIntraGPMRefTpl[intraMode])
      {
        xFillIntraGPMRefTemplate(pu, eTempType, intraMode, loadIntraRef, &m_acYuvRefGPMIntraTemplate[intraMode][0][0], &m_acYuvRefGPMIntraTemplate[intraMode][1][0], lut);
        loadIntraRef = false; // to prevent duplicating initialization
      }      
    }
  }

  return true;
}

bool IntraPrediction::xFillIntraGPMRefTemplate(PredictionUnit& pu, TemplateType eTempType, uint8_t intraMode, bool loadIntraRef, Pel* bufTop, Pel* bufLeft, std::vector<Pel>* lut)
{
  if (eTempType == NO_NEIGHBOR)
  {
    return false;
  }
  CHECK(intraMode >= NUM_INTRA_MODE, "Invalid intra mode for intra GPM template");
  m_abFilledIntraGPMRefTpl[intraMode] = true;

  const uint32_t uiPredStride = MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE;
  static Pel predLuma[uiPredStride * uiPredStride];
  
  int iTempWidth  = GEO_MODE_SEL_TM_SIZE;
  int iTempHeight = GEO_MODE_SEL_TM_SIZE;

  // Load reference samples
  if (loadIntraRef)
  {
    TemplateType tplType = (TemplateType)prefillIntraGPMReferenceSamples(pu, iTempWidth, iTempHeight);
    CHECK(eTempType != tplType, "Inconsistent template block availability");
  }
  else // Just need setting intra ref parameters, when ref samples have not been swapped out
  {
    m_ipaParam.multiRefIndex = iTempWidth;
    m_topRefLength = (pu.lwidth() + (eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0)) << 1;
    m_leftRefLength = (pu.lheight() + (eTempType != LEFT_NEIGHBOR ? iTempHeight : 0)) << 1;
  }

  // Generate intra pred samples
  int dirMode = intraMode > DC_IDX ? MAP67TO131(intraMode) : intraMode;
  uint32_t uiRealW = pu.lwidth()  + (eTempType != ABOVE_NEIGHBOR ? iTempWidth  : 0);
  uint32_t uiRealH = pu.lheight() + (eTempType != LEFT_NEIGHBOR  ? iTempHeight : 0);
  initPredTimdIntraParams(pu, pu.Y(), dirMode);
  predTimdIntraAng(COMPONENT_Y, pu, dirMode, predLuma, uiPredStride, uiRealW, uiRealH, eTempType, (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth, (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);

  // Store intra pred
  Pel* predSrcAbove = nullptr;
  Pel* predSrcLeft  = nullptr;
  if (eTempType == ABOVE_NEIGHBOR)
  {
    predSrcAbove = predLuma;
  }
  else if (eTempType == LEFT_NEIGHBOR)
  {
    predSrcLeft = predLuma;
  }
  else // Above-left
  {
    predSrcAbove = predLuma + iTempWidth;
    predSrcLeft  = predLuma + iTempHeight * uiPredStride;
  }

  if (predSrcAbove != nullptr)
  {
    Pel*   tmpSrc = predSrcAbove;
    Pel*   tmpDst = bufTop;
    if (lut == nullptr)
    {
      size_t szLine = sizeof(Pel) * pu.lwidth();
      for (int h = 0; h < iTempHeight; ++h)
      {
        memcpy(tmpDst, tmpSrc, szLine);
        tmpSrc += uiPredStride;
        tmpDst += pu.lwidth();
      }
    }
    else
    {
      for (int h = 0; h < iTempHeight; ++h)
      {
        for (int w = 0; w < pu.lwidth(); ++w)
        {
          tmpDst[w] = (*lut)[tmpSrc[w]];
        }
        tmpSrc += uiPredStride;
        tmpDst += pu.lwidth();
      }
    }
  }

  if (predSrcLeft != nullptr)
  {
    Pel*   tmpSrc = predSrcLeft;
    Pel*   tmpDst = bufLeft;
    if (lut == nullptr)
    {
      for (int h = 0; h < pu.lheight(); ++h)
      {
        for (int w = 0; w < iTempWidth; ++w)
        {
          tmpDst[w] = tmpSrc[w];
        }
        tmpSrc += uiPredStride;
        tmpDst += iTempWidth;
      }
    }
    else
    {
      for (int h = 0; h < pu.lheight(); ++h)
      {
        for (int w = 0; w < iTempWidth; ++w)
        {
          tmpDst[w] = (*lut)[tmpSrc[w]];
        }
        tmpSrc += uiPredStride;
        tmpDst += iTempWidth;
      }
    }
  }

  return true;
}

uint8_t IntraPrediction::prefillIntraGPMReferenceSamples(PredictionUnit& pu, int iTempWidth, int iTempHeight)
{
  int  iRefX = -1, iRefY = -1;
  uint32_t uiRefWidth = 0, uiRefHeight = 0;
  TemplateType eTempType = CU::deriveTimdRefType(pu.lx(), pu.ly(), pu.lwidth(), pu.lheight(), iTempWidth, iTempHeight, iRefX, iRefY, uiRefWidth, uiRefHeight);
    
  m_ipaParam.multiRefIndex = iTempWidth;
  initTimdIntraPatternLuma(*pu.cu, pu.Y(), (eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0), (eTempType != LEFT_NEIGHBOR ? iTempHeight : 0), uiRefWidth, uiRefHeight);

  return eTempType;
}

bool IntraPrediction::fillIntraGPMRefTemplateAll(PredictionUnit& pu, bool hasAboveTemplate, bool hasLeftTemplate, bool readBufferedMPMList, bool doInitMPMList, bool loadIntraRef, std::vector<Pel>* lut, uint8_t candIdx0, uint8_t candIdx1)
{
#if JVET_AG0164_AFFINE_GPM
  if (candIdx0 < GEO_MAX_ALL_INTER_UNI_CANDS && candIdx1 < GEO_MAX_ALL_INTER_UNI_CANDS)
#else
  if (candIdx0 < GEO_MAX_NUM_UNI_CANDS && candIdx1 < GEO_MAX_NUM_UNI_CANDS)
#endif
  {
    return false;
  }

  TemplateType templateType = (TemplateType)((hasAboveTemplate ? ABOVE_NEIGHBOR : 0) + (hasLeftTemplate ? LEFT_NEIGHBOR : 0));
  if (templateType == NO_NEIGHBOR)
  {
    return false;
  }
#if JVET_AG0164_AFFINE_GPM
  if (candIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS)
#else
  if (candIdx0 >= GEO_MAX_NUM_UNI_CANDS)
#endif
  {
    xFillIntraGPMRefTemplateAll<0>(pu, templateType, readBufferedMPMList, doInitMPMList, loadIntraRef, lut, candIdx0);
    doInitMPMList = false; // to prevent duplicating initialization
    loadIntraRef  = false; // to prevent duplicating initialization
  }
#if JVET_AG0164_AFFINE_GPM
  if (candIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS)
#else
  if (candIdx1 >= GEO_MAX_NUM_UNI_CANDS)
#endif
  {
    xFillIntraGPMRefTemplateAll<1>(pu, templateType, readBufferedMPMList, doInitMPMList, loadIntraRef, lut, candIdx1);
  }

  return true;
}
#endif

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

// Function for calculating DC value of the reference samples used in Intra prediction
//NOTE: Bit-Limit - 25-bit source
Pel IntraPrediction::xGetPredValDc( const CPelBuf &pSrc, const Size &dstSize )
{
  CHECK( dstSize.width == 0 || dstSize.height == 0, "Empty area provided" );

  int idx, sum = 0;
  Pel dcVal;
  const int width  = dstSize.width;
  const int height = dstSize.height;
  const auto denom     = (width == height) ? (width << 1) : std::max(width,height);
  const auto divShift  = floorLog2(denom);
  const auto divOffset = (denom >> 1);
  const Pel* src;

  if ( width >= height )
  {
    src = pSrc.bufAt(m_ipaParam.multiRefIndex + 1, 0);
    for (idx = 0; idx < width; idx++)
    {
      sum += src[idx];
    }
  }
  if ( width <= height )
  {
    src = pSrc.bufAt(m_ipaParam.multiRefIndex + 1, 1);
    for (idx = 0; idx < height; idx++)
    {
      sum += src[idx];
    }
  }

  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

int IntraPrediction::getModifiedWideAngle( int width, int height, int predMode )
{
  //The function returns a 'modified' wide angle index, given that it is not necessary 
  //in this software implementation to reserve the values 0 and 1 for Planar and DC to generate the prediction signal.
  //It should only be used to obtain the intraPredAngle parameter.
  //To simply obtain the wide angle index, the function PU::getWideAngle should be used instead.
  if ( predMode > DC_IDX && predMode <= VDIA_IDX )
  {
    int modeShift[] = { 0, 6, 10, 12, 14, 15 };
    int deltaSize = abs(floorLog2(width) - floorLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
      predMode += (VDIA_IDX - 1);
    }
    else if (height > width && predMode > VDIA_IDX - modeShift[deltaSize])
    {
      predMode -= (VDIA_IDX - 1);
    }
  }
  return predMode;
}
#if JVET_AC0094_REF_SAMPLES_OPT
int IntraPrediction::getTimdWideAngle(int width, int height, int predMode)
{
  int modeShift[] = { 0, 6, 10, 12, 14, 15 };
  int deltaSize   = abs(floorLog2(width) - floorLog2(height));
  int maxIndex    = modeShift[std::min(deltaSize + 2, 5)];
  // Case 1: using a regular mode removed due to WAIP
  if (width > height && predMode < DC_IDX + 1 + maxIndex)
  {
    predMode = (predMode == VDIA_IDX ? DC_IDX + 1 : predMode + 1);
  }
  else if (height > width && predMode > VDIA_IDX - maxIndex)
  {
    predMode = (predMode == DC_IDX + 1 ? VDIA_IDX : predMode - 1);
  }
  // Case 2: using a mode not usually allowed on this block size
  else if (predMode > VDIA_IDX)
  {
    predMode -= (VDIA_IDX - 2);
  }
  else if (predMode < VDIA_IDX)
  {
    predMode += (VDIA_IDX - 2);
  }
  return predMode;
}
#endif

#if JVET_W0123_TIMD_FUSION
#if JVET_AB0155_SGPM
int IntraPrediction::getWideAngleExt(int width, int height, int predMode, bool bSgpm)
#else
int IntraPrediction::getWideAngleExt( int width, int height, int predMode )
#endif
{
  if ( predMode > DC_IDX && predMode <= EXT_VDIA_IDX )
  {
    int modeShift[] = { 0, 11, 19, 23, 27, 29 };
    int deltaSize = abs(floorLog2(width) - floorLog2(height));
    if (width > height && predMode < 2 + modeShift[deltaSize])
    {
#if JVET_AB0155_SGPM
      if (bSgpm)
      {
        predMode += EXT_VDIA_IDX;
      }
      else
      {
        predMode += (EXT_VDIA_IDX - 1);
      }
#else
      predMode += (EXT_VDIA_IDX - 1);
#endif
    }
    else if (height > width && predMode > EXT_VDIA_IDX - modeShift[deltaSize])
    {
#if JVET_AB0155_SGPM
      if (bSgpm)
      {
        predMode -= EXT_VDIA_IDX;
      }
      else
      {
        predMode -= (EXT_VDIA_IDX - 1);
      }
#else
      predMode -= (EXT_VDIA_IDX - 1);
#endif
    }
  }
  return predMode;
}
#if JVET_AC0094_REF_SAMPLES_OPT
int IntraPrediction::getTimdWideAngleExt(int width, int height, int predMode)
{
  int modeShift[] = { 0, 11, 19, 23, 27, 29 };
  int deltaSize   = abs(floorLog2(width) - floorLog2(height));
  int maxIndex    = modeShift[std::min(deltaSize, 5)];
  // Case 1: using a regular mode removed due to WAIP
  if (width > height && predMode < DC_IDX + 1 + maxIndex)
  {
    predMode = (predMode == EXT_VDIA_IDX ? DC_IDX + 1 : predMode + 1);
  }
  else if (height > width && predMode > EXT_VDIA_IDX - maxIndex)
  {
    predMode = (predMode == DC_IDX + 1 ? EXT_VDIA_IDX : predMode - 1);
  }
  // Case 2: using a mode not usually allowed on this block size
  else if (predMode > EXT_DIA_IDX)
  {
    predMode -= (EXT_VDIA_IDX - 2);
  }
  else if (predMode < EXT_DIA_IDX)
  {
    predMode += (EXT_VDIA_IDX - 2);
  }
  return predMode;
}

int IntraPrediction::getTimdRegularAngleExt(int width, int height, int predMode)
{
  int modeShift[] = { 0, 11, 19, 23, 27, 29 };
  int deltaSize   = abs(floorLog2(width) - floorLog2(height));
  int maxIndex    = modeShift[std::min(deltaSize + 2, 5)];
  // Treat edge cases for first and last angular IPMs, 2 and EXT_VDIA_IDX
  if (predMode == EXT_VDIA_IDX)
  {
    return DC_IDX + 1;
  }
  if (predMode == DC_IDX + 1)
  {
    return EXT_VDIA_IDX;
  }
  // Case 1: using a regular mode removed due to WAIP
  if (width > height && predMode <= DC_IDX + 1 + maxIndex && predMode > DC_IDX)
  {
    predMode = predMode - 1;
  }
  else if (height > width && predMode >= EXT_VDIA_IDX - maxIndex && predMode <= EXT_VDIA_IDX)
  {
    predMode = predMode + 1;
  }
  // Case 2: using a mode not usually allowed on this block size
  else if (predMode > EXT_VER_IDX)
  {
    predMode -= (EXT_VDIA_IDX - 2);
  }
  else if (predMode < EXT_HOR_IDX)
  {
    predMode += (EXT_VDIA_IDX - 2);
  }
  CHECK(predMode < DC_IDX + 1 || predMode > EXT_VDIA_IDX, "luma mode out of range");
  return predMode;
}
#endif
#endif

void IntraPrediction::setReferenceArrayLengths( const CompArea &area )
{
  // set Top and Left reference samples length
  const int  width    = area.width;
  const int  height   = area.height;

#if JVET_AC0094_REF_SAMPLES_OPT
  m_leftRefLength = height << 3;
  m_topRefLength  = width << 3;
#else
  m_leftRefLength     = (height << 1);
  m_topRefLength      = (width << 1);
#endif
}

#if JVET_AB0157_INTRA_FUSION
void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu, const bool applyFusion)
#else
void IntraPrediction::predIntraAng( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu)
#endif
{
#if CIIP_PDPC
  CHECK((pu.ciipFlag == false && pu.ciipPDPC == true),"ciip_PDPC can not be true for an non CIIP PU");
#endif
  const ComponentID    compID       = MAP_CHROMA( compId );
  const ChannelType    channelType  = toChannelType( compID );
  const int            iWidth       = piPred.width;
  const int            iHeight      = piPred.height;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK(iWidth == 2, "Width of 2 is not supported");
#endif

  CHECK(PU::isMIP(pu, toChannelType(compId)), "We should not get here for MIP.");
  const uint32_t       uiDirMode    = isLuma( compId ) && pu.cu->bdpcmMode ? BDPCM_IDX : !isLuma(compId) && pu.cu->bdpcmModeChroma ? BDPCM_IDX : PU::getFinalIntraMode(pu, channelType);
#if JVET_AD0085_TMRL_EXTENSION
  bool bExtIntraDir = false;
#if JVET_W0123_TIMD_FUSION
  bExtIntraDir |= (pu.cu->timd && isLuma(compId));
#endif
#if JVET_AD0085_TMRL_EXTENSION
  bExtIntraDir |= (pu.cu->tmrlFlag && isLuma(compId));
#endif
#else
#if JVET_W0123_TIMD_FUSION
  bool bExtIntraDir = pu.cu->timd && isLuma( compId );
#endif
#endif

  CHECK( floorLog2(iWidth) < 2 && pu.cs->pcv->noChroma2x2, "Size not allowed" );
  CHECK( floorLog2(iWidth) > 7, "Size not allowed" );

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  const CPelBuf & srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compID));

#if JVET_AB0157_INTRA_FUSION
  const CPelBuf & srcBuf2nd = CPelBuf(m_refBuffer2nd[compID], srcStride+1, srcHStride);
#endif

#if CIIP_PDPC
  if (!pu.ciipPDPC)
  {
#endif
    switch (uiDirMode)
    {
#if JVET_AC0105_DIRECTIONAL_PLANAR
    case (PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred, isLuma(compID) ? pu.cu->plIdx : 0); break;
#else
    case (PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred); break;
#endif
    case(DC_IDX):     xPredIntraDc(srcBuf, piPred, channelType, false); break;
    case(BDPCM_IDX):  xPredIntraBDPCM(srcBuf, piPred, isLuma(compID) ? pu.cu->bdpcmMode : pu.cu->bdpcmModeChroma, clpRng); break;
#if JVET_W0123_TIMD_FUSION
#if JVET_AB0157_INTRA_FUSION
    default:
#if !JVET_AB0155_SGPM
      int weightMode = ((pu.cu->timd && pu.cu->timdIsBlended) || !applyFusion) ? 4 : 3;
#else
      int weightMode = ((pu.cu->timd && pu.cu->timdIsBlended) || !applyFusion || (PU::isSgpm(pu, CHANNEL_TYPE_LUMA))) ? 4 : 3;
#endif
#if JVET_AC0112_IBC_CIIP
      if (pu.ibcCiipFlag)
      {
        weightMode = 4;
      }
#endif
#if JVET_AC0112_IBC_GPM
      if (pu.ibcGpmFlag)
      {
        weightMode = 4;
      }
#endif
      xPredIntraAng(srcBuf, piPred, channelType, clpRng, bExtIntraDir, srcBuf2nd, pu.cu->ispMode != NOT_INTRA_SUBPARTITIONS, weightMode);
        break;
#else
    default:          xPredIntraAng(srcBuf, piPred, channelType, clpRng, bExtIntraDir); break;
#endif
#else
    default:          xPredIntraAng(srcBuf, piPred, channelType, clpRng); break;
#endif
    }
#if CIIP_PDPC
  }
#endif

#if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
  if( (m_ipaParam.applyPDPC || pu.ciipPDPC) && (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX) )
  {
    xIntraPredPlanarDcPdpc( srcBuf, piPred.buf, piPred.stride, iWidth, iHeight, pu.ciipPDPC );
  }
#else
  if( m_ipaParam.applyPDPC && (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX) )
  {
    xIntraPredPlanarDcPdpc( srcBuf, piPred.buf, piPred.stride, iWidth, iHeight );
  }
#endif
#endif

#if ENABLE_DIMD
#if JVET_AB0157_INTRA_FUSION
  if (pu.cu->dimd && pu.cu->dimdBlending && isLuma(compID))
  {
    int width = piPred.width;
    int height = piPred.height;
    const UnitArea localUnitArea( pu.chromaFormat, Area( 0, 0, width, height ) );

    PelBuf planarBuffer = m_tempBuffer[0].getBuf( localUnitArea.Y() );

#if JVET_AC0105_DIRECTIONAL_PLANAR
    xPredIntraPlanar(srcBuf, planarBuffer, 0);
#else
    xPredIntraPlanar(srcBuf, planarBuffer);
#endif

    const bool applyPdpc = m_ipaParam.applyPDPC;

    bool blendModes[DIMD_FUSION_NUM-2] = {false};
    PelBuf predAngExtra[DIMD_FUSION_NUM-2];
    for( int i = 0; i < DIMD_FUSION_NUM-2; ++i)
    {
#if JVET_AC0098_LOC_DEP_DIMD
      blendModes[i] = (pu.cu->dimdBlendMode[i] != PLANAR_IDX);
#else
      blendModes[i] = (i==0 || pu.cu->dimdBlendMode[i] != PLANAR_IDX);
#endif
      if(blendModes[i])
      {
        predAngExtra[i] = m_tempBuffer[i + 1].getBuf( localUnitArea.Y() );
        PredictionUnit puTmp = pu;
        puTmp.intraDir[0] = pu.cu->dimdBlendMode[i];
        initPredIntraParams(puTmp, pu.Y(), *(pu.cs->sps));
#if JVET_W0123_TIMD_FUSION
        xPredIntraAng(srcBuf, predAngExtra[i], channelType, clpRng, false, srcBuf2nd, pu.cu->ispMode!=NOT_INTRA_SUBPARTITIONS);
#else
        xPredIntraAng(srcBuf, predAngExtra[i], channelType, clpRng);
#endif
      }
    }

    m_ipaParam.applyPDPC = applyPdpc;

    // do blending
#if JVET_AC0098_LOC_DEP_DIMD
    PelBuf predAngNonLocDep = m_tempBuffer[7].getBuf( localUnitArea.Y() );
    PelBuf predAngVer       = m_tempBuffer[5].getBuf( localUnitArea.Y() );
    PelBuf predAngHor       = m_tempBuffer[6].getBuf( localUnitArea.Y() );

    Pel* pelVer = predAngVer.buf;
    int strideVer = predAngVer.stride;
    Pel* pelHor = predAngHor.buf;
    int strideHor = predAngHor.stride;
    Pel *pelNonLocDep = predAngNonLocDep.buf;
    int strideNonLocDep = predAngNonLocDep.stride;

    bool useLocDepBlending = false;
    int weightVer = 0, weightHor = 0, weightNonLocDep = 0;
    weightNonLocDep += pu.cu->dimdRelWeight[1];
    for (int i = 0; i < DIMD_FUSION_NUM-1; i++)
    {
      if (i == 0  || blendModes[i-1])
      {
        if (pu.cu->dimdLocDep[i] == 1)
        {
          weightVer += (i == 0 ? pu.cu->dimdRelWeight[0] : pu.cu->dimdRelWeight[i+ 1]);
        }
        else if (pu.cu->dimdLocDep[i] == 2)
        {
          weightHor += (i == 0 ? pu.cu->dimdRelWeight[0] : pu.cu->dimdRelWeight[i+ 1]);
        }
        else
        {
          weightNonLocDep += (i == 0 ? pu.cu->dimdRelWeight[0] : pu.cu->dimdRelWeight[i+1]);
        }
      }
    }

    if(weightHor || weightVer)
    {
      useLocDepBlending = true;
    }

    if(!useLocDepBlending)
    {
      pelNonLocDep = piPred.buf;
      strideNonLocDep = piPred.stride;
    }

    for (int locDep = 0; locDep < 3; locDep++)
    {
      int totWeight = (locDep == 0 ? weightNonLocDep : (locDep == 1 ? weightVer : weightHor));
      if (totWeight == 0)
      {
        continue;
      }

      int weights[6] = {0};
      weights[0] =  (pu.cu->dimdLocDep[0] == locDep) ? pu.cu->dimdRelWeight[0] : 0;
      weights[1] =  (blendModes[0] && pu.cu->dimdLocDep[1] == locDep) ? pu.cu->dimdRelWeight[2] : 0;
      weights[2] =  (blendModes[1] && pu.cu->dimdLocDep[2] == locDep) ? pu.cu->dimdRelWeight[3] : 0;
      weights[3] =  (blendModes[2] && pu.cu->dimdLocDep[3] == locDep) ? pu.cu->dimdRelWeight[4] : 0;
      weights[4] =  (blendModes[3] && pu.cu->dimdLocDep[4] == locDep) ? pu.cu->dimdRelWeight[5] : 0;
      weights[5] =  (locDep == 0) ? pu.cu->dimdRelWeight[1] : 0;

      int num2blend = 0;
      int blendIndexes[DIMD_FUSION_NUM] = {0};
      for (int i = 0; i < DIMD_FUSION_NUM; i++)
      {
        if (weights[i] != 0)
        {
          blendIndexes[num2blend] = i;
          num2blend++;
        }
      }
#if JVET_W0123_TIMD_FUSION
      if( (num2blend == 1 ) || (num2blend <=3 && (totWeight == (1 << (floorLog2(totWeight))) ) ))
      {
        int index = blendIndexes[0];    
        if(locDep == 0)
        {
          pelNonLocDep = (index == 0 ? piPred.buf : (index == 5 ? planarBuffer.buf : predAngExtra[index-1].buf));
          strideNonLocDep = (index == 0 ? piPred.stride : (index == 5 ? planarBuffer.stride : predAngExtra[index-1].stride));
        }
        else if(locDep == 1)
        {
          pelVer = (index == 0 ? piPred.buf : predAngExtra[index-1].buf);
          strideVer = (index == 0 ? piPred.stride : predAngExtra[index-1].stride);
        }
        else
        {
          pelHor = (index == 0 ? piPred.buf : predAngExtra[index-1].buf);
          strideHor = (index == 0 ? piPred.stride : predAngExtra[index-1].stride);
        }
        Pel* pCur = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
        int strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

        int factor = 64 / totWeight;
        if (num2blend == 2)
        {
          int index1 = blendIndexes[1];
          Pel* p1 = (index1 == 0 ? piPred.buf : (index1 == 5 ?  planarBuffer.buf : predAngExtra[index1-1].buf));
          int stride1 = (index1 == 0 ? piPred.stride : (index1 == 5 ? planarBuffer.stride : predAngExtra[index1-1].stride));

          int w0 = (weights[index]*factor);
          int w1 = 64 - w0;
          m_timdBlending(pCur, strideCur, p1, stride1, w0, w1,width, height);
        }
        else if(num2blend == 3)
        {
          int index1 = blendIndexes[1];
          Pel* p1 = (index1 == 0 ? piPred.buf : (index1 == 5 ?  planarBuffer.buf : predAngExtra[index1-1].buf));
          int stride1 = (index1 == 0 ? piPred.stride : (index1 == 5 ? planarBuffer.stride : predAngExtra[index1-1].stride));

          int index2 = blendIndexes[2];
          Pel* p2 = (index2 == 0 ? piPred.buf : (index2 == 5 ?  planarBuffer.buf : predAngExtra[index2-1].buf));
          int stride2 = (index2 == 0 ? piPred.stride : (index2 == 5 ? planarBuffer.stride : predAngExtra[index2-1].stride));

          int w0 = (weights[index]*factor);
          int w1 = (weights[index1]*factor);
          int w2 = 64  - w0 - w1;
          m_dimdBlending(pCur, strideCur, p1, stride1, p2, stride2, w0, w1, w2, width, height);
        }
      }
      else
#endif
      {
        Pel* pCur = (locDep == 0 ? pelNonLocDep : (locDep == 1 ? pelVer : pelHor));
        int strideCur = (locDep == 0 ? strideNonLocDep : (locDep == 1 ? strideVer : strideHor));

        Pel *pelPredAng0 = piPred.buf;
        Pel *pelPredAng1 = predAngExtra[0].buf;
        Pel *pelPredAng2 = predAngExtra[1].buf;
        Pel *pelPredAng3 = predAngExtra[2].buf;
        Pel *pelPredAng4 = predAngExtra[3].buf;
        Pel *pelPlanar = planarBuffer.buf;
        int stride0 = piPred.stride;
        int stride1 = predAngExtra[0].stride;
        int stride2 = predAngExtra[1].stride;
        int stride3 = predAngExtra[2].stride;
        int stride4 = predAngExtra[3].stride;
        int stridePlanar = planarBuffer.stride;
        for( int y = 0; y < height; y++ )
        {
          for( int x = 0; x < width; x++ )
          {
            int blend = pelPredAng0[x] * weights[0];
            blend += blendModes[0] ? pelPredAng1[x] * weights[1] : 0;
            blend += blendModes[1] ? pelPredAng2[x] * weights[2] : 0;
            blend += blendModes[2] ? pelPredAng3[x] * weights[3] : 0;
            blend += blendModes[3] ? pelPredAng4[x] * weights[4] : 0;
            blend += pelPlanar[x] * weights[5];
            pCur[x] = (Pel)(blend / totWeight);
          }
          pCur += strideCur;
          pelPredAng0 += stride0;
          pelPredAng1 += stride1;
          pelPredAng2 += stride2;
          pelPredAng3 += stride3;
          pelPredAng4 += stride4;
          pelPlanar += stridePlanar;
        }
      }
    }

    if (useLocDepBlending)
    {
      int mode = ((weightHor > 0 && weightVer > 0) ? 0 : (weightVer > 0 ? 1 : 2));

      Pel *pelDst = piPred.buf;
      int strideDst = piPred.stride;
      xDimdLocationdepBlending(pelDst, strideDst, pelVer, strideVer, pelHor, strideHor, pelNonLocDep,strideNonLocDep, width, height,mode, weightVer, weightHor, weightNonLocDep);
    }
#else
    const int log2WeightSum = 6;
    Pel *pelPred = piPred.buf;
    Pel *pelPlanar = planarBuffer.buf;
    Pel *pelPredAng = predAngExtra[0].buf;
    int  w0 = pu.cu->dimdRelWeight[0], w1 = pu.cu->dimdRelWeight[1], w2 = pu.cu->dimdRelWeight[2];

    Pel *pelPredAng2 = predAngExtra[1].buf;
    Pel *pelPredAng3 = predAngExtra[2].buf;
    Pel *pelPredAng4 = predAngExtra[3].buf;
    int  w3         = pu.cu->dimdRelWeight[3];
    int  w4         = pu.cu->dimdRelWeight[4];
    int  w5         = pu.cu->dimdRelWeight[5];

    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        int blend = pelPred[x] * w0;
        blend += pelPlanar[x] * w1;
#if JVET_AC0098_LOC_DEP_DIMD
        blend += blendModes[0] ? pelPredAng[x] * w2 : 0;
#else
        blend += pelPredAng[x] * w2;
#endif
        blend += blendModes[1] ? pelPredAng2[x] * w3 : 0;
        blend += blendModes[2] ? pelPredAng3[x] * w4 : 0;
        blend += blendModes[3] ? pelPredAng4[x] * w5 : 0;
        pelPred[x] = (Pel)(blend >> log2WeightSum);
      }

      pelPred += piPred.stride;
      pelPlanar += planarBuffer.stride;
      pelPredAng += predAngExtra[0].stride;
      pelPredAng2 += predAngExtra[1].stride;
      pelPredAng3 += predAngExtra[2].stride;
      pelPredAng4 += predAngExtra[3].stride;
    }
#endif
  }
#else
  if (pu.cu->dimd && pu.cu->dimdBlending && isLuma(compID))
  {
    int width = piPred.width;
    int height = piPred.height;
    const UnitArea localUnitArea( pu.chromaFormat, Area( 0, 0, width, height ) );

    PelBuf planarBuffer = m_tempBuffer[0].getBuf( localUnitArea.Y() );
    PelBuf predAng = m_tempBuffer[1].getBuf( localUnitArea.Y() );

#if JVET_AC0105_DIRECTIONAL_PLANAR
    xPredIntraPlanar(srcBuf, planarBuffer, 0);
#else
    xPredIntraPlanar(srcBuf, planarBuffer);
#endif

    const bool applyPdpc = m_ipaParam.applyPDPC;
#if JVET_AC0098_LOC_DEP_DIMD
    if (pu.cu->dimdBlendMode[0] != 0)
    {
#endif
#if JVET_V0087_DIMD_NO_ISP   // this is pure cleanup to make code easier to read. It generates identical resut to the else part
    PredictionUnit pu2 = pu;
    pu2.intraDir[0] = pu.cu->dimdBlendMode[0];
    initPredIntraParams(pu2, pu.Y(), *(pu.cs->sps));

#if JVET_W0123_TIMD_FUSION
    xPredIntraAng(srcBuf, predAng, channelType, clpRng, false);
#else
    xPredIntraAng(srcBuf, predAng, channelType, clpRng);
#endif
#if JVET_AC0098_LOC_DEP_DIMD
    }
    else
    {
      predAng = piPred;
    }
#endif
#else
    const bool   useISP = NOT_INTRA_SUBPARTITIONS != pu.cu->ispMode && isLuma( CHANNEL_TYPE_LUMA );//ok
    const Size   cuSize = Size( pu.cu->blocks[compId].width, pu.cu->blocks[compId].height ); //ok
    const Size   puSize = Size( piPred.width, piPred.height );
    const Size&  blockSize = useISP ? cuSize : puSize;
    int blendDir = pu.cu->dimdBlendMode[0];
    const int      dirMode = blendDir;
    const int     predMode = getModifiedWideAngle( blockSize.width, blockSize.height, dirMode ); // to check later
    m_ipaParam.isModeVer = predMode >= DIA_IDX;
    m_ipaParam.multiRefIndex = 0;
    m_ipaParam.refFilterFlag = false;
    m_ipaParam.interpolationFlag = false;
    m_ipaParam.applyPDPC = ( ( puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY ) || !isLuma( compId ) ) && m_ipaParam.multiRefIndex == 0;

    const int    intraPredAngleMode = ( m_ipaParam.isModeVer ) ? predMode - VER_IDX : -( predMode - HOR_IDX );//ok
    int absAng = 0;
    if( dirMode > DC_IDX && dirMode < NUM_LUMA_MODE ) // intraPredAngle for directional modes
    {
      static const int angTable[32] = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };//ok
      static const int invAngTable[32] = { 0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565, 512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16 };   // (512 * 32) / Angle

      const int     absAngMode = abs( intraPredAngleMode );
      const int     signAng = intraPredAngleMode < 0 ? -1 : 1;
      absAng = angTable[absAngMode];

      m_ipaParam.absInvAngle = invAngTable[absAngMode];
      m_ipaParam.intraPredAngle = signAng * absAng;
      if( intraPredAngleMode < 0 )
      {
        m_ipaParam.applyPDPC = false;
      }
      else if( intraPredAngleMode > 0 )
      {
        const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
        const int maxScale = 2;
#if GRAD_PDPC
        m_ipaParam.useGradPDPC = false;
#endif

        m_ipaParam.angularScale = std::min( maxScale, floorLog2( sideSize ) - ( floorLog2( 3 * m_ipaParam.absInvAngle - 2 ) - 8 ) );
#if GRAD_PDPC
        if( ( m_ipaParam.angularScale < 0 ) && ( isLuma( compId ) ) )
        {
          m_ipaParam.angularScale = ( floorLog2( puSize.width ) + floorLog2( puSize.height ) - 2 ) >> 2;
          m_ipaParam.useGradPDPC = true;
        }
#endif
        m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
      }
    }

    if( pu.cs->sps->getSpsRangeExtension().getIntraSmoothingDisabledFlag()
        || ( !isLuma( CHANNEL_TYPE_LUMA ) && pu.chromaFormat != CHROMA_444 )
        || useISP
        || m_ipaParam.multiRefIndex
        || DC_IDX == dirMode
        )
    {
      //do nothing
    }
    else if( !useISP )// HOR, VER and angular modes (MDIS)
    {
      bool filterFlag = false;
      const int diff = std::min<int>( abs( predMode - HOR_IDX ), abs( predMode - VER_IDX ) );
      const int log2Size = ( ( floorLog2( puSize.width ) + floorLog2( puSize.height ) ) >> 1 );
      CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
      filterFlag = ( diff > m_aucIntraFilter[log2Size] );


      if( filterFlag )
      {
        const bool isRefFilter = isIntegerSlope( absAng );
        CHECK( puSize.width * puSize.height <= 32, "DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB" );
        m_ipaParam.refFilterFlag = isRefFilter;
        m_ipaParam.interpolationFlag = !isRefFilter;
      }
    }

#if JVET_W0123_TIMD_FUSION
    xPredIntraAng( srcBuf, predAng, channelType, clpRng, false );
#else
    xPredIntraAng( srcBuf, predAng, channelType, clpRng );
#endif
#endif
    m_ipaParam.applyPDPC = applyPdpc;
#if JVET_AC0098_LOC_DEP_DIMD
    if (pu.cu->dimdLocDep[0] != 0 || pu.cu->dimdLocDep[1] != 0)
    {
      Pel *pelPred = piPred.buf;
      Pel *pelPlanar = planarBuffer.buf;
      Pel *pelPredAng = predAng.buf;
      int  wMain = pu.cu->dimdRelWeight[0], wPlanar = pu.cu->dimdRelWeight[1], wSecond = pu.cu->dimdRelWeight[2];

      xDimdLocationdepBlending(pelPred, piPred.stride, pelPred, piPred.stride, pelPredAng, predAng.stride, pelPlanar, planarBuffer.stride, width, height, pu.cu->dimdLocDep[0], pu.cu->dimdLocDep[1], wMain, wSecond, wPlanar);
    }
    else
    {
#endif
    // do blending
#if INTRA_TRANS_ENC_OPT
    Pel *pelPred = piPred.buf;
    Pel *pelPlanar = planarBuffer.buf;
    Pel *pelPredAng = predAng.buf;
    int  w0 = pu.cu->dimdRelWeight[0], w1 = pu.cu->dimdRelWeight[1], w2 = pu.cu->dimdRelWeight[2];
    m_dimdBlending(pelPred, piPred.stride, pelPlanar, planarBuffer.stride, pelPredAng, predAng.stride, w0, w1, w2, width, height);
#else
    const int log2WeightSum = 6;
    Pel *pelPred = piPred.buf;
    Pel *pelPlanar = planarBuffer.buf;
    Pel *pelPredAng = predAng.buf;
    int  w0 = pu.cu->dimdRelWeight[0], w1 = pu.cu->dimdRelWeight[1], w2 = pu.cu->dimdRelWeight[2];

    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        int blend = pelPred[x] * w0;
        blend += pelPlanar[x] * w1;
        blend += pelPredAng[x] * w2;
        pelPred[x] = (Pel)(blend >> log2WeightSum);
      }

      pelPred += piPred.stride;
      pelPlanar += planarBuffer.stride;
      pelPredAng += predAng.stride;
    }
#endif
#if JVET_AC0098_LOC_DEP_DIMD
    }
#endif
  }
#endif
#endif

#if JVET_W0123_TIMD_FUSION
  if (pu.cu->timd && pu.cu->timdIsBlended && isLuma(compID))
  {
    int width = piPred.width;
    int height = piPred.height;
    const UnitArea localUnitArea( pu.chromaFormat, Area( 0, 0, width, height ) );

    PelBuf predFusion = m_tempBuffer[1].getBuf( localUnitArea.Y() );

    const bool applyPdpc = m_ipaParam.applyPDPC;
    PredictionUnit pu2 = pu;
#if JVET_AC0094_REF_SAMPLES_OPT
    pu2.intraDir[0]  = pu.cu->timdModeSecondary;
    int tmpTimdMode  = pu2.cu->timdMode;
    pu2.cu->timdMode = INVALID_TIMD_IDX;
#else
    pu2.intraDir[0] = pu.cu->timdModeSecondary;
#endif
    initPredIntraParams(pu2, pu.Y(), *(pu.cs->sps));
#if JVET_AC0094_REF_SAMPLES_OPT
    pu2.cu->timdMode = tmpTimdMode;
#endif

    switch (pu.cu->timdModeSecondary)
    {
#if JVET_AC0105_DIRECTIONAL_PLANAR
    case (PLANAR_IDX): xPredIntraPlanar(srcBuf, predFusion, 0); break;
#else
    case (PLANAR_IDX): xPredIntraPlanar(srcBuf, predFusion); break;
#endif
    case(DC_IDX):     xPredIntraDc(srcBuf, predFusion, channelType, false); break;
#if JVET_AB0157_INTRA_FUSION
    default:          xPredIntraAng(srcBuf, predFusion, channelType, clpRng, bExtIntraDir, srcBuf2nd, pu.cu->ispMode!=NOT_INTRA_SUBPARTITIONS, 0); break;
#else
    default:          xPredIntraAng(srcBuf, predFusion, channelType, clpRng, bExtIntraDir); break;
#endif
    }

#if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
    if( (m_ipaParam.applyPDPC || pu.ciipPDPC) && (pu.cu->timdModeSecondary == PLANAR_IDX || pu.cu->timdModeSecondary == DC_IDX) )
    {
      xIntraPredPlanarDcPdpc( srcBuf, m_tempBuffer[1].getBuf( localUnitArea.Y() ).buf, m_tempBuffer[1].getBuf( localUnitArea.Y() ).stride, iWidth, iHeight, pu.ciipPDPC );
    }
#else
    if (m_ipaParam.applyPDPC && (pu.cu->timdModeSecondary == PLANAR_IDX || pu.cu->timdModeSecondary == DC_IDX))
    {
      xIntraPredPlanarDcPdpc(srcBuf, m_tempBuffer[1].getBuf(localUnitArea.Y()).buf, m_tempBuffer[1].getBuf(localUnitArea.Y()).stride, iWidth, iHeight);
    }
#endif
#endif
    m_ipaParam.applyPDPC = applyPdpc;

    // do blending
#if INTRA_TRANS_ENC_OPT
    Pel *pelPred = piPred.buf;
    Pel *pelPredFusion = predFusion.buf;
    int  w0 = pu.cu->timdFusionWeight[0], w1 = pu.cu->timdFusionWeight[1];
    m_timdBlending(pelPred, piPred.stride, pelPredFusion, predFusion.stride, w0, w1, width, height);
#else
    const int log2WeightSum = 6;
    Pel *pelPred = piPred.buf;
    Pel *pelPredFusion = predFusion.buf;
    int  w0 = pu.cu->timdFusionWeight[0], w1 = pu.cu->timdFusionWeight[1];

    for( int y = 0; y < height; y++ )
    {
      for( int x = 0; x < width; x++ )
      {
        int blend = pelPred[x] * w0;
        blend += pelPredFusion[x] * w1;
        pelPred[x] = (Pel)(blend >> log2WeightSum);
      }

      pelPred += piPred.stride;
      pelPredFusion += predFusion.stride;
    }
#endif
  }
#endif

#if JVET_AB0155_SGPM
  
  if(PU::isSgpm(pu, channelType))
  {
    int            width  = piPred.width;
    int            height = piPred.height;
    const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, width, height));
    PelBuf predFusion = m_tempBuffer[1].getBuf(localUnitArea.Y());
    IntraPredParam m_ipaParam2 = m_ipaParam;
    CompArea compArea = (compID == COMPONENT_Y) ? pu.Y()
                                               : (compID == COMPONENT_Cb) ? pu.Cb() : pu.Cr();
    initIntraPatternChType(*pu.cu, compArea, false, 1); 
    const uint32_t uiDirMode2 = PU::getFinalIntraMode(pu, channelType, 1);
    const CPelBuf &srcBuf2 = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
    switch (uiDirMode2)
    {
#if JVET_AC0105_DIRECTIONAL_PLANAR
    case (PLANAR_IDX): xPredIntraPlanar(srcBuf2, predFusion, 0); break;
#else
    case (PLANAR_IDX): xPredIntraPlanar(srcBuf2, predFusion); break;
#endif
    case (DC_IDX): xPredIntraDc(srcBuf2, predFusion, channelType, false); break;
#if JVET_AB0157_INTRA_FUSION
    default:
       int weightMode = 4;
      xPredIntraAng(srcBuf2, predFusion, channelType, clpRng, bExtIntraDir, srcBuf2nd, pu.cu->ispMode!=NOT_INTRA_SUBPARTITIONS, weightMode); break;
#else
    default: xPredIntraAng(srcBuf2, predFusion, channelType, clpRng, bExtIntraDir); break;
#endif
    }

    #if JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
    if ((m_ipaParam.applyPDPC || pu.ciipPDPC) && (uiDirMode2 == PLANAR_IDX || uiDirMode2 == DC_IDX))
#else
    if (m_ipaParam.applyPDPC && (uiDirMode2 == PLANAR_IDX || uiDirMode2 == DC_IDX))
#endif
    {
      xIntraPredPlanarDcPdpc(srcBuf2, m_tempBuffer[1].getBuf(localUnitArea.Y()).buf,
                             m_tempBuffer[1].getBuf(localUnitArea.Y()).stride, iWidth, iHeight
#if CIIP_PDPC       
           ,pu.ciipPDPC
#endif   
      );
    }
#endif
    
    m_ipaParam           = m_ipaParam2;
    
    int     splitDir   = pu.cu->sgpmSplitDir;
    m_if.m_weightedSgpm(pu, width, height, compID, splitDir, piPred, piPred, predFusion);
  }
#endif

#if !JVET_X0148_TIMD_PDPC
#if CIIP_PDPC
  if (m_ipaParam.applyPDPC || pu.ciipPDPC)
#else
  if (m_ipaParam.applyPDPC)
#endif
  {
    PelBuf dstBuf = piPred;
    const int scale = ((floorLog2(iWidth) - 2 + floorLog2(iHeight) - 2 + 2) >> 2);
    CHECK(scale < 0 || scale > 31, "PDPC: scale < 0 || scale > 31");

    if (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX)
    { 
      const Pel* srcLeft = srcBuf.bufAt(1, 1);
      Pel* dst = dstBuf.buf;
#if CIIP_PDPC
      if (pu.ciipPDPC)
      {
        for (int y = 0; y < iHeight; y++)
        {
          const int wT = 32 >> std::min(31, ((y << 1) >> scale));
          const Pel left = *srcLeft;
          const Pel* srcTop = srcBuf.buf + 1;
          for (int x = 0; x < iWidth; x++)
          {
            const int wL = 32 >> std::min(31, ((x << 1) >> scale));
            const Pel top = *srcTop;
            dst[x] = ((wL * left  + wT * top + 32) >> 6);

            srcTop++;
          }
          srcLeft++;
          dst += dstBuf.stride;
        }
      }
      else
      {
#endif

      for (int y = 0; y < iHeight; y++)
      {
        const int wT   = 32 >> std::min(31, ((y << 1) >> scale));
        const Pel left = *srcLeft;
        const Pel* srcTop = srcBuf.buf + 1;

        for (int x = 0; x < iWidth; x++)
        {
          const int wL = 32 >> std::min(31, ((x << 1) >> scale));
          const Pel top = *srcTop;
          const Pel val = dst[x];

          dst[x] = val + ((wL * (left - val) + wT * (top - val) + 32) >> 6);
          srcTop++;
        }

        srcLeft++;
        dst += dstBuf.stride;
      }
#if CIIP_PDPC
      }
#endif
    }
  }
#endif
}

#if JVET_AC0071_DBV
void IntraPrediction::predIntraDbv(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                 , InterPrediction *pcInterPred
#endif
)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  Mv mv = refineChromaBv(compId, pu, pcInterPred);
#if JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE
  if (!CS::isDualITree(*pu.cs))
  {
    const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compId, pu.chromaFormat);
    const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compId, pu.chromaFormat);
    int xFrac = mv.hor & ((1 << bvShiftHor) - 1);
    int yFrac = mv.ver & ((1 << bvShiftVer) - 1);

    PelUnitBuf pcBuf(pu.chromaFormat, PelBuf(), piPred, piPred);
    if (xFrac != 0 || yFrac != 0)
    {
      pcInterPred->getPredIBCBlk(pu, compId, pu.cu->slice->getPic(), mv, pcBuf, false, false);
    }
    else
    {
      int refx = pu.blocks[compId].x + (mv.hor >> bvShiftHor);
      int refy = pu.blocks[compId].y + (mv.ver >> bvShiftVer);
      int refStride = pu.cs->picture->getRecoBuf(compId).stride;
      Pel *ref = pu.cs->picture->getRecoBuf(compId).buf;
      Pel *refTarget = ref + refy * refStride + refx;
      int iStride = piPred.stride;
      Pel *pred = piPred.buf;
      int iHeight = piPred.height;
      int lineBufSize = piPred.width * sizeof(Pel);
      for (int uiY = 0; uiY < iHeight; uiY++)
      {
        ::memcpy(pred, refTarget, lineBufSize);
        refTarget += refStride;
        pred += iStride;
      }
    }
    return;
  }
#endif
  bool isFracMv = pu.cs->sps->getIBCFracFlag() && mv.isFracMv<false>(pu.chromaFormat);
  if (isFracMv)
  {
    PelUnitBuf pcBuf(pu.chromaFormat, PelBuf(), piPred, piPred);
    pcInterPred->getPredIBCBlk(pu, compId, pu.cs->picture, mv, pcBuf);
  }
  else
  {
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const int bvShftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compId, pu.chromaFormat);
  const int bvShftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compId, pu.chromaFormat);
  int refx = pu.blocks[compId].x + (mv.hor >> bvShftHor);
  int refy = pu.blocks[compId].y + (mv.ver >> bvShftVer);
#else
  const int shiftSampleHor = ::getComponentScaleX(compId, pu.chromaFormat);
  const int shiftSampleVer = ::getComponentScaleY(compId, pu.chromaFormat);
  Mv chromaBv = Mv(pu.bv.hor >> shiftSampleHor, pu.bv.ver >> shiftSampleVer);
  chromaBv = refineChromaBv(compId, pu);
  int refx = pu.blocks[compId].x + chromaBv.hor;
  int refy = pu.blocks[compId].y + chromaBv.ver;
#endif
  int refStride = pu.cs->picture->getRecoBuf(compId).stride;
  Pel *ref = pu.cs->picture->getRecoBuf(compId).buf;
  Pel *refTarget = ref + refy * refStride + refx;
  int iStride = piPred.stride;
  Pel *pred = piPred.buf;
  int iHeight = piPred.height;
  int iWidth = piPred.width;
  for (int uiY = 0; uiY < iHeight; uiY++)
  {
    for (int uiX = 0; uiX < iWidth; uiX++)
    {
      pred[uiX] = refTarget[uiX];
    }
    refTarget += refStride;
    pred += iStride;
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  }
#endif
}

Mv IntraPrediction::refineChromaBv(const ComponentID compId, const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                 , InterPrediction *pcInterPred
#endif
)
{
  const CodingStructure &cs = *pu.cs;
  Position posRT = pu.blocks[compId].topRight();
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool topCanUse = puAbove && pu.cu != puAbove->cu;
  Position posLB = pu.blocks[compId].bottomLeft();
  const PredictionUnit *puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  bool leftCanUse = puLeft && pu.cu != puLeft->cu;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleX(compId, pu.chromaFormat);
  const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + ::getComponentScaleY(compId, pu.chromaFormat);
#else
  const int shiftSampleHor = ::getComponentScaleX(compId, pu.chromaFormat);
  const int shiftSampleVer = ::getComponentScaleY(compId, pu.chromaFormat);
#endif
#if JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE
  if (!CS::isDualITree(cs))
  {
#if JVET_AF0079_STORING_INTRATMP
    if (PU::checkIsChromaBvCandidateValid(pu, pu.mv[0], 0))
    {
      return pu.mv[0];
    }
    else
    {
      Mv mv = pu.bv;
      mv <<= MV_FRACTIONAL_BITS_INTERNAL;
      return mv;
    }
#else
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (pu.cs->sps->getIBCFracFlag() && pu.cu->tmpFlag && !pu.cu->tmpFlmFlag && !pu.cu->tmpFusionFlag && pu.cu->tmpIsSubPel)
    {
      int pXFrac = pu.mv[0].hor;
      int pYFrac = pu.mv[0].ver;
      int fracOffset = (pu.cu->tmpIsSubPel == 1 ? 8 : (pu.cu->tmpIsSubPel == 2 ? 4 : 12));
      if (pu.cu->tmpSubPelIdx == LEFT_POS || pu.cu->tmpSubPelIdx == ABOVE_LEFT_POS || pu.cu->tmpSubPelIdx == LEFT_BOTTOM_POS)
      {
        pXFrac -= fracOffset;
      }
      if (pu.cu->tmpSubPelIdx == RIGHT_POS || pu.cu->tmpSubPelIdx == ABOVE_RIGHT_POS || pu.cu->tmpSubPelIdx == RIGHT_BOTTOM_POS)
      {
        pXFrac += fracOffset;
      }
      if (pu.cu->tmpSubPelIdx == ABOVE_POS || pu.cu->tmpSubPelIdx == ABOVE_LEFT_POS || pu.cu->tmpSubPelIdx == ABOVE_RIGHT_POS)
      {
        pYFrac -= fracOffset;
      }
      if (pu.cu->tmpSubPelIdx == BOTTOM_POS || pu.cu->tmpSubPelIdx == LEFT_BOTTOM_POS || pu.cu->tmpSubPelIdx == RIGHT_BOTTOM_POS)
      {
        pYFrac += fracOffset;
      }
      Mv lumaBv = Mv(pXFrac, pYFrac);
      if (PU::checkIsChromaBvCandidateValid(pu, lumaBv, 0))
      {
        return Mv(pXFrac, pYFrac);
      }
    }
    return pu.cs->sps->getIBCFracFlag() ? pu.mv[0] : Mv((pu.mv[0].hor >> bvShiftHor) << bvShiftHor, (pu.mv[0].ver >> bvShiftVer) << bvShiftVer);
#else
    return Mv(pu.bv.hor >> shiftSampleHor, pu.bv.ver >> shiftSampleVer);
#endif
#endif
  }
#endif
  if (topCanUse == false && leftCanUse == false)
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    return pu.cs->sps->getIBCFracFlag() ? pu.mv[0] : Mv((pu.mv[0].hor >> bvShiftHor) << bvShiftHor, (pu.mv[0].ver >> bvShiftVer) << bvShiftVer);
#else
    return Mv(pu.bv.hor >> shiftSampleHor, pu.bv.ver >> shiftSampleVer);
#endif
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  int filterIdx = 0;
  std::vector<Mv> chromaBvList;
  chromaBvList.push_back(pu.cs->sps->getIBCFracFlag() ? pu.mv[0] : Mv((pu.mv[0].hor >> bvShiftHor) << bvShiftHor, (pu.mv[0].ver >> bvShiftVer) << bvShiftVer));
#else
  Mv lumaBv = pu.bv;
  Mv chromaBv(lumaBv.hor >> shiftSampleHor, lumaBv.ver >> shiftSampleVer);
  std::vector<Mv> chromaBvList;
  chromaBvList.push_back(chromaBv);
#endif
  for (int stephor = 0; stephor < 2; stephor++)
  {
    for (int stepver = 0; stepver < 2; stepver++)
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      Mv lumaBv(pu.mv[0].hor + (stephor << MV_FRACTIONAL_BITS_INTERNAL), pu.mv[0].ver + (stepver << MV_FRACTIONAL_BITS_INTERNAL));
      if (!pu.cs->sps->getIBCFracFlag())
      {
        lumaBv.set((lumaBv.hor >> bvShiftHor) << bvShiftHor, (lumaBv.ver >> bvShiftVer) << bvShiftVer);
      }
      if (!PU::xCheckSimilarChromaBv(chromaBvList, lumaBv) && PU::checkIsChromaBvCandidateValid(pu, lumaBv, filterIdx))
      {
        chromaBvList.push_back(lumaBv);
      }
#else
      lumaBv.set(pu.bv.hor + stephor, pu.bv.ver + stepver);
      chromaBv.set(lumaBv.hor >> shiftSampleHor, lumaBv.ver >> shiftSampleVer);
      if (!PU::xCheckSimilarChromaBv(chromaBvList, chromaBv) && PU::checkIsChromaBvCandidateValid(pu, chromaBv))
      {
        chromaBvList.push_back(chromaBv);
      }
#endif
    }
  }
  if (chromaBvList.size() == 1)
  {
    return chromaBvList[0];
  }

  CompArea area = pu.blocks[compId];
  int uiHeight = area.height;
  int uiWidth = area.width;
  Pel *cur = m_refBuffer[compId][PRED_BUF_UNFILTERED];
  PelBuf tempCurTop = PelBuf(cur + 1, uiWidth, Size(uiWidth, DBV_TEMPLATE_SIZE));
  PelBuf tempCurLeft = PelBuf(cur + 1 + m_refBufferStride[compId], uiHeight, Size(uiHeight, DBV_TEMPLATE_SIZE));

  Pel temp[(MAX_CU_SIZE + 1) * 2];
  memset(temp, 0, (MAX_CU_SIZE + 1) * 2 * sizeof(Pel));
  int stride = MAX_CU_SIZE + 1;
  Pel *refPix = temp;
  Pel *refPixTemp;
  const CPelBuf recBuf = pu.cs->picture->getRecoBuf(pu.cs->picture->blocks[compId]);

  DistParam cDistParam;
  cDistParam.applyWeight = false;
  Distortion uiCost;
  std::vector<std::pair<Mv, Distortion>> aBvCostVec;
  for (std::vector<Mv>::iterator it = chromaBvList.begin(); it != chromaBvList.end(); ++it)
  {
    Mv mvCurr = *it;
    if (topCanUse)
    {
      Mv mvTop(0, -DBV_TEMPLATE_SIZE);
#if JVET_AA0070_RRIBC
      if (pu.cu->rribcFlipType == 2)
      {
        mvTop.setVer(uiHeight);
      }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      mvTop <<= bvShiftVer;
#endif
      mvTop += mvCurr;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (!PU::checkIsChromaBvCandidateValid(pu, mvTop, filterIdx, true, true))
      {
#if JVET_AA0070_RRIBC
        if (pu.cu->rribcFlipType == 2)
        {
          mvTop.setVer(mvCurr.getVer() + ((uiHeight - DBV_TEMPLATE_SIZE) << bvShiftVer));
        }
        else
#endif
        {
          mvTop = mvCurr;
        }
      }
#else
#if JVET_AA0070_RRIBC
      if (pu.cu->rribcFlipType == 2)
      {
        if (!PU::checkIsChromaBvCandidateValid(pu, mvTop, true, true))
        {
          mvTop.setVer(mvCurr.getVer() + uiHeight - DBV_TEMPLATE_SIZE);
        }
      }
      else
#endif
      if (!PU::checkIsChromaBvCandidateValid(pu, mvTop, true, true))
      {
        mvTop = mvCurr;
      }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      bool isFracMv = pu.cs->sps->getIBCFracFlag() && mvTop.isFracMv<false>(pu.chromaFormat);
      if (isFracMv)
      {
        PelUnitBuf pcBuf(pu.chromaFormat, PelBuf(), PelBuf(refPix + 1, uiWidth, DBV_TEMPLATE_SIZE), PelBuf(refPix + 1, uiWidth, DBV_TEMPLATE_SIZE));
        pcInterPred->getPredIBCBlk(pu, compId, pu.cs->picture, mvTop, pcBuf, filterIdx == 1);
#if JVET_AA0070_RRIBC
        pcBuf.bufs[compId].flip(pu.cu->rribcFlipType);
#endif
      }
      else
      {
#endif
      refPixTemp = refPix + 1;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      const Pel *rec = recBuf.bufAt(pu.blocks[compId].pos().offset(mvTop.hor >> bvShiftHor, mvTop.ver >> bvShiftVer));
#else
      const Pel *rec = recBuf.bufAt(pu.blocks[compId].pos().offset(mvTop.hor, mvTop.ver));
#endif
      for (int k = 0; k < uiWidth; k++)
      {
        for (int l = 0; l < DBV_TEMPLATE_SIZE; l++)
        {
#if JVET_AA0070_RRIBC
          int recVal;
          if (pu.cu->rribcFlipType == 0)
          {
            recVal = rec[k + l * recBuf.stride];
          }
          else if (pu.cu->rribcFlipType == 1)
          {
            recVal = rec[uiWidth - 1 - k + l * recBuf.stride];
          }
          else
          {
            recVal = rec[k + (DBV_TEMPLATE_SIZE - 1 - l) * recBuf.stride];
          }
#else
          int recVal = rec[k + l * recBuf.stride];
#endif
          refPixTemp[k] = recVal;
        }
      }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      }
#endif
    }

    if (leftCanUse)
    {
      Mv mvLeft(-DBV_TEMPLATE_SIZE, 0);
#if JVET_AA0070_RRIBC
      if (pu.cu->rribcFlipType == 1)
      {
        mvLeft.setHor(uiWidth);
      }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      mvLeft <<= bvShiftHor;
#endif
      mvLeft += mvCurr;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (!PU::checkIsChromaBvCandidateValid(pu, mvLeft, filterIdx, true, false))
      {
#if JVET_AA0070_RRIBC
        if (pu.cu->rribcFlipType == 1)
        {
          mvLeft.setHor(mvCurr.getHor() + ((uiWidth - DBV_TEMPLATE_SIZE) << bvShiftHor));
        }
        else
#endif
        {
          mvLeft = mvCurr;
        }
      }
#else
#if JVET_AA0070_RRIBC
      if (pu.cu->rribcFlipType == 1)
      {
        if (!PU::checkIsChromaBvCandidateValid(pu, mvLeft, true, false))
        {
          mvLeft.setHor(mvCurr.getHor() + uiWidth - DBV_TEMPLATE_SIZE);
        }
      }
      else
#endif
      if (!PU::checkIsChromaBvCandidateValid(pu, mvLeft, true, false))
      {
        mvLeft = mvCurr;
      }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      bool isFracMv = pu.cs->sps->getIBCFracFlag() && mvLeft.isFracMv<false>(pu.chromaFormat);
      if (isFracMv)
      {
        PelUnitBuf pcBuf(pu.chromaFormat, PelBuf(), PelBuf(refPix + 1 + stride, DBV_TEMPLATE_SIZE, uiHeight), PelBuf(refPix + 1 + stride, DBV_TEMPLATE_SIZE, uiHeight));
        pcInterPred->getPredIBCBlk(pu, compId, pu.cs->picture, mvLeft, pcBuf, filterIdx == 1);
#if JVET_AA0070_RRIBC
        pcBuf.bufs[compId].flip(pu.cu->rribcFlipType);
#endif
      }
      else
      {
#endif
      refPixTemp = refPix + 1 + stride;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      const Pel *rec = recBuf.bufAt(pu.blocks[compId].pos().offset(mvLeft.hor >> bvShiftHor, mvLeft.ver >> bvShiftVer));
#else
      const Pel *rec = recBuf.bufAt(pu.blocks[compId].pos().offset(mvLeft.hor, mvLeft.ver));
#endif
      for (int k = 0; k < uiHeight; k++)
      {
        for (int l = 0; l < DBV_TEMPLATE_SIZE; l++)
        {
#if JVET_AA0070_RRIBC
          int recVal;
          if (pu.cu->rribcFlipType == 0)
          {
            recVal = rec[recBuf.stride * k + l];
          }
          else if (pu.cu->rribcFlipType == 1)
          {
            recVal = rec[recBuf.stride * k + DBV_TEMPLATE_SIZE - 1 - l];
          }
          else
          {
            recVal = rec[recBuf.stride * (uiHeight - 1 - k) + l];
          }
#else
          int recVal = rec[recBuf.stride * k + l];
#endif
          refPixTemp[k] = recVal;
        }
      }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      }
#endif
    }

    uiCost = 0;
    if (topCanUse)
    {
      PelBuf tempRef = PelBuf(refPix + 1, uiWidth, Size(uiWidth, DBV_TEMPLATE_SIZE));
      m_dbvSadCost->setDistParam(cDistParam, tempCurTop, tempRef, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), compId, false);
      uiCost += cDistParam.distFunc(cDistParam);
    }
    if (leftCanUse)
    {
      PelBuf tempRef = PelBuf(refPix + 1 + stride, uiHeight, Size(uiHeight, DBV_TEMPLATE_SIZE));
      m_dbvSadCost->setDistParam(cDistParam, tempCurLeft, tempRef, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), compId, false);
      uiCost += cDistParam.distFunc(cDistParam);
    }
    aBvCostVec.push_back(std::pair<Mv, Distortion>(*it, uiCost));
  }
  std::stable_sort(aBvCostVec.begin(), aBvCostVec.end(), [](const std::pair<Mv, Distortion> &l, const std::pair<Mv, Distortion> &r) { return l.second < r.second; });
  return aBvCostVec[0].first;
}
#endif

#if JVET_Z0050_CCLM_SLOPE
void IntraPrediction::xUpdateCclmModel(int &a, int &b, int &iShift, int midLuma, int delta)
{
  if ( delta )
  {
    const int dShift = 3; // For 1/8 sample value adjustment
    
    delta = a > 0 ? -delta : delta;
    
    // Make final shift at least the size of the precision of the update
    if ( iShift < dShift )
    {
      a    <<= ( dShift - iShift );
      iShift = dShift;
    }
    else if ( iShift > dShift )
    {
      // Final shift is larger than the precision of the update: scale the update up to the final precision
      delta <<= ( iShift - dShift );
    }
    
    a += delta;
    b -= ( delta * midLuma ) >> iShift;
  }
}
#endif

#if JVET_AD0188_CCP_MERGE
void IntraPrediction::predIntraChromaLM( const ComponentID compID, PelBuf &piPred, PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel, CclmModel *cclmModelStored )
#else
void IntraPrediction::predIntraChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea& chromaArea, int intraDir, bool createModel, CclmModel *cclmModelStored)
#endif
{
#if JVET_AC0119_LM_CHROMA_FUSION
  if (pu.isChromaFusion > 1)
  {
    CccmModel cflmModel( CFLM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
#if MMLM
    if (PU::isMultiModeLM(pu.intraDir[1]))
    {
      int modelThr = xCflmCalcRefAver(pu, chromaArea);

      xCflmCalcModels(pu, compID, chromaArea, cflmModel, 1, modelThr);
      xCflmApplyModel(pu, compID, chromaArea, cflmModel, 1, modelThr, piPred);

      xCflmCalcModels(pu, compID, chromaArea, cflmModel, 2, modelThr);
      xCflmApplyModel(pu, compID, chromaArea, cflmModel, 2, modelThr, piPred);
    }
    else
    {
#endif
      xCflmCalcModels(pu, compID, chromaArea, cflmModel, 0, 0);
      xCflmApplyModel(pu, compID, chromaArea, cflmModel, 0, 0, piPred);
#if MMLM
    }
#endif
    return;
  }
#endif

#if JVET_AB0092_GLM_WITH_LUMA
  if (pu.glmIdc.getIdc(compID, 0) > NUM_GLM_PATTERN)
  {
    CccmModel glmModel( GLM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
    xGlmCalcModel(pu, compID, chromaArea, glmModel);
    xGlmApplyModel(pu, compID, chromaArea, glmModel, piPred);

 #if JVET_AD0188_CCP_MERGE
    pu.curCand.type   = CCP_TYPE_GLM4567;
    pu.curCand.glmIdc = pu.glmIdc.getIdc(compID, 0);
    PU::glmModelToCcpParams(compID, pu.curCand, glmModel 
#if JVET_AB0174_CCCM_DIV_FREE
                            , m_glmLumaOffset
#endif
                            );
#endif
    return;
  }
#endif

  int  iLumaStride = 0;
  PelBuf temp;
#if JVET_AA0126_GLM
  if (pu.glmIdc.isActive())
  {
    int glmIdc = pu.glmIdc.getIdc(compID, 0);
    Pel* glmTemp = compID == COMPONENT_Cb ? m_glmTempCb[glmIdc] : m_glmTempCr[glmIdc];
    iLumaStride = 2 * MAX_CU_SIZE + 1;
    temp = PelBuf(glmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  else
  {
#endif
#if MMLM
    if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX) || (intraDir == MMLM_L_IDX) || (intraDir == MMLM_T_IDX) || (m_encPreRDRun && intraDir == MMLM_CHROMA_IDX))
#else
    if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
#endif
    {
      iLumaStride = 2 * MAX_CU_SIZE + 1;
      temp = PelBuf(m_pMdlmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
    }
    else
    {
      iLumaStride = MAX_CU_SIZE + 1;
      temp = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
    }
#if JVET_AA0126_GLM
  }
#endif

  CclmModel cclmModel;

  if (createModel)
  {
#if LMS_LINEAR_MODEL
    xGetLMParametersLMS(pu, compID, chromaArea, cclmModel);
#else
    xGetLMParameters(pu, compID, chromaArea, cclmModel);
#endif

    // Store the created model if storage struct was provided
    if (cclmModelStored != nullptr)
    {
      *cclmModelStored = cclmModel;
    }
  }
  else
  {
    // Use the pre-calculated model
    cclmModel = *cclmModelStored;
  }

#if JVET_Z0050_CCLM_SLOPE
  xUpdateCclmModel(cclmModel.a, cclmModel.b, cclmModel.shift, cclmModel.midLuma, compID == COMPONENT_Cb ? pu.cclmOffsets.cb0 : pu.cclmOffsets.cr0);
#if MMLM
  xUpdateCclmModel(cclmModel.a2, cclmModel.b2, cclmModel.shift2, cclmModel.midLuma2, compID == COMPONENT_Cb ? pu.cclmOffsets.cb1 : pu.cclmOffsets.cr1);
#endif
#endif

#if JVET_AD0188_CCP_MERGE
  int glmIdc = pu.glmIdc.getIdc(compID, 0);
  pu.curCand.type = (glmIdc > 0) ? CCP_TYPE_GLM0123 : CCP_TYPE_CCLM;

  if (PU::isMultiModeLM(pu.intraDir[1]))
  {
    pu.curCand.type |= CCP_TYPE_MMLM;
  }

  pu.curCand.glmIdc = glmIdc;
  PU::cclmModelToCcpParams(compID, pu.curCand, cclmModel);
#endif

  ////// final prediction
  piPred.copyFrom(temp);
#if MMLM
  if (PU::isMultiModeLM(pu.intraDir[1]))
  {
    Pel*  pPred = piPred.bufAt(0, 0);
    Pel  *pLuma = temp.bufAt(0, 0);
    int uiPredStride = piPred.stride;
    int uiCWidth = chromaArea.width;
    int uiCHeight = chromaArea.height;

    for (int i = 0; i < uiCHeight; i++)
    {
      for (int j = 0; j < uiCWidth; j++)
      {
        if (pLuma[j] <= cclmModel.yThres)
        {
          pPred[j] = (Pel)ClipPel(((cclmModel.a * pLuma[j]) >> cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(compID));
        }
        else
        {
          pPred[j] = (Pel)ClipPel(((cclmModel.a2 * pLuma[j]) >> cclmModel.shift2) + cclmModel.b2, pu.cs->slice->clpRng(compID));
        }
      }
      pPred += uiPredStride;
      pLuma += iLumaStride;
    }
  }
  else
  {
#endif
    piPred.linearTransform(cclmModel.a, cclmModel.shift, cclmModel.b, true, pu.cs->slice->clpRng(compID));
#if MMLM
  }
#endif
#if JVET_AD0120_LBCCP
  if (pu.ccInsideFilter)
  {
    filterPredInside(compID, piPred, pu);
  }
#endif
}

/** Function for deriving planar intra prediction. This function derives the prediction samples for planar mode (intra coding).
 */

//NOTE: Bit-Limit - 24-bit source
void IntraPrediction::xPredIntraPlanar( const CPelBuf &pSrc, PelBuf &pDst
#if JVET_AC0105_DIRECTIONAL_PLANAR
  , uint8_t plidx
#endif
)
{
  const uint32_t width  = pDst.width;
  const uint32_t height = pDst.height;

  const uint32_t log2W = floorLog2( width );
  const uint32_t log2H = floorLog2( height );

  int leftColumn[MAX_CU_SIZE + 1], topRow[MAX_CU_SIZE + 1], bottomRow[MAX_CU_SIZE], rightColumn[MAX_CU_SIZE];
  const uint32_t offset = 1 << (log2W + log2H);

  // Get left and above reference column and row
  const Pel* src = pSrc.bufAt(1, 0);

  for (int k = 0; k < width + 1; k++)
  {
    topRow[k] = src[k];
  }

  src = pSrc.bufAt(1, 1);

  for (int k = 0; k < height + 1; k++)
  {
    leftColumn[k] = src[k];
  }
#if JVET_AC0105_DIRECTIONAL_PLANAR
  if (plidx == 0)   // original planar
  {
    // Prepare intermediate variables used in interpolation
    int bottomLeft = leftColumn[height];
    int topRight   = topRow[width];

    for (int k = 0; k < width; k++)
    {
      bottomRow[k] = bottomLeft - topRow[k];
      topRow[k]    = topRow[k] << log2H;
    }

    for (int k = 0; k < height; k++)
    {
      rightColumn[k] = topRight - leftColumn[k];
      leftColumn[k]  = leftColumn[k] << log2W;
    }

    const uint32_t finalShift = 1 + log2W + log2H;
    const uint32_t stride     = pDst.stride;
    Pel *          pred       = pDst.buf;
    for (int y = 0; y < height; y++, pred += stride)
    {
      int horPred = leftColumn[y];

      for (int x = 0; x < width; x++)
      {
        horPred += rightColumn[y];
        topRow[x] += bottomRow[x];

        int vertPred = topRow[x];
        pred[x]      = ((horPred << log2H) + (vertPred << log2W) + offset) >> finalShift;
      }
    }
  }
  else if (plidx == 1)   // planar hor
  {
    // Prepare intermediate variables used in interpolation
    int topRight = topRow[width];
    for (int k = 0; k < height; k++)
    {
      rightColumn[k] = topRight - leftColumn[k];
      leftColumn[k]  = leftColumn[k] << log2W;
    }

    const uint32_t stride = pDst.stride;
    Pel *          pred   = pDst.buf;
    int            horPred;
    for (int y = 0; y < height; y++, pred += stride)
    {
      horPred = leftColumn[y];
      for (int x = 0; x < width; x++)
      {
        horPred += rightColumn[y];
        pred[x] = (horPred + (1 << (log2W - 1))) >> log2W;
      }
    }
  }
  else   // planar ver
  {
    // Prepare intermediate variables used in interpolation
    int bottomLeft = leftColumn[height];
    for (int k = 0; k < width; k++)
    {
      bottomRow[k] = bottomLeft - topRow[k];
      topRow[k]    = topRow[k] << log2H;
    }

    const uint32_t stride = pDst.stride;
    Pel *          pred   = pDst.buf;
    int            vertPred;
    for (int y = 0; y < height; y++, pred += stride)
    {
      for (int x = 0; x < width; x++)
      {
        topRow[x] += bottomRow[x];
        vertPred = topRow[x];
        pred[x]  = (vertPred + (1 << (log2H - 1))) >> log2H;
      }
    }
  }
#else
  // Prepare intermediate variables used in interpolation
  int bottomLeft = leftColumn[height];
  int topRight = topRow[width];

  for( int k = 0; k < width; k++ )
  {
    bottomRow[k] = bottomLeft - topRow[k];
    topRow[k]    = topRow[k] << log2H;
  }

  for( int k = 0; k < height; k++ )
  {
    rightColumn[k] = topRight - leftColumn[k];
    leftColumn[k]  = leftColumn[k] << log2W;
  }

  const uint32_t finalShift = 1 + log2W + log2H;
  const uint32_t stride     = pDst.stride;
  Pel*       pred       = pDst.buf;
  for( int y = 0; y < height; y++, pred += stride )
  {
    int horPred = leftColumn[y];

    for( int x = 0; x < width; x++ )
    {
      horPred += rightColumn[y];
      topRow[x] += bottomRow[x];

      int vertPred = topRow[x];
      pred[x]      = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
    }
  }
#endif
}

void IntraPrediction::xPredIntraDc( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const bool enableBoundaryFilter )
{
  const Pel dcval = xGetPredValDc( pSrc, pDst );
  pDst.fill( dcval );
}

// Function for initialization of intra prediction parameters
#if JVET_AB0155_SGPM
void IntraPrediction::initPredIntraParams(const PredictionUnit &pu, const CompArea area, const SPS &sps, const int partIdx)
#else
void IntraPrediction::initPredIntraParams(const PredictionUnit & pu, const CompArea area, const SPS& sps)
#endif
{
  const ComponentID compId = area.compID;
  const ChannelType chType = toChannelType(compId);
#if JVET_AD0085_TMRL_EXTENSION
  bool bExtIntraDir = false;
#if JVET_W0123_TIMD_FUSION
  bExtIntraDir |= (pu.cu->timd && isLuma(chType));
#endif
#if JVET_AD0085_TMRL_EXTENSION
  bExtIntraDir |= (pu.cu->tmrlFlag && isLuma(chType));
#endif
#else
#if JVET_W0123_TIMD_FUSION
  bool bExtIntraDir = pu.cu->timd && isLuma( chType );
#endif
#endif

  const bool        useISP = NOT_INTRA_SUBPARTITIONS != pu.cu->ispMode && isLuma( chType );

  const Size   cuSize    = Size( pu.cu->blocks[compId].width, pu.cu->blocks[compId].height );
  const Size   puSize    = Size( area.width, area.height );
  const Size&  blockSize = useISP ? cuSize : puSize;
#if JVET_AB0155_SGPM
  const int dirMode = PU::getFinalIntraMode(pu, chType, partIdx);
#else
  const int      dirMode = PU::getFinalIntraMode(pu, chType);
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_AC0094_REF_SAMPLES_OPT
#if JVET_AD0085_TMRL_EXTENSION
  bool checkWideAngle = !bExtIntraDir ? true : (pu.cu->timdMode != INVALID_TIMD_IDX ? pu.cu->timdModeCheckWA : pu.cu->timdModeSecondaryCheckWA);

  int predMode = checkWideAngle ? (bExtIntraDir ? getWideAngleExt(blockSize.width, blockSize.height, dirMode) : getModifiedWideAngle(blockSize.width, blockSize.height, dirMode))
    : (bExtIntraDir ? getTimdWideAngleExt(blockSize.width, blockSize.height, dirMode) : getTimdWideAngle(blockSize.width, blockSize.height, dirMode));
  if (pu.cu->tmrlFlag && isLuma(chType))
  {
    predMode = getWideAngleExt(blockSize.width, blockSize.height, dirMode);
  }
#else
  if (bExtIntraDir)
  {
    CHECK(pu.cu->timdMode != dirMode && pu.cu->timdModeSecondary != dirMode, "Unexpected dirMode");
  }
  bool checkWideAngle = !bExtIntraDir ? true : (pu.cu->timdMode != INVALID_TIMD_IDX ? pu.cu->timdModeCheckWA : pu.cu->timdModeSecondaryCheckWA);
  const int predMode = checkWideAngle ? (bExtIntraDir ? getWideAngleExt(blockSize.width, blockSize.height, dirMode) : getModifiedWideAngle(blockSize.width, blockSize.height, dirMode))
                         : (bExtIntraDir ? getTimdWideAngleExt(blockSize.width, blockSize.height, dirMode) : getTimdWideAngle(blockSize.width, blockSize.height, dirMode));
  CHECK(!checkWideAngle && dirMode <= DC_IDX, "Unexpected mode");
#endif
#else
  const int     predMode = bExtIntraDir ? getWideAngleExt( blockSize.width, blockSize.height, dirMode ) : getModifiedWideAngle( blockSize.width, blockSize.height, dirMode );
#endif
#else
  const int     predMode = getModifiedWideAngle( blockSize.width, blockSize.height, dirMode );
#endif

#if JVET_W0123_TIMD_FUSION
  m_ipaParam.isModeVer            = bExtIntraDir ? (predMode >= EXT_DIA_IDX) : (predMode >= DIA_IDX);
#else
  m_ipaParam.isModeVer            = predMode >= DIA_IDX;
#endif
  m_ipaParam.multiRefIndex        = isLuma (chType) ? pu.multiRefIdx : 0 ;
  m_ipaParam.refFilterFlag        = false;
  m_ipaParam.interpolationFlag    = false;
  m_ipaParam.applyPDPC            = (puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY) && m_ipaParam.multiRefIndex == 0;

#if JVET_W0123_TIMD_FUSION
  const int    intraPredAngleMode = (m_ipaParam.isModeVer) ? (predMode - (bExtIntraDir? EXT_VER_IDX : VER_IDX)) : (-(predMode - (bExtIntraDir ? EXT_HOR_IDX : HOR_IDX)));
#else
  const int    intraPredAngleMode = (m_ipaParam.isModeVer) ? predMode - VER_IDX : -(predMode - HOR_IDX);
#endif


  int absAng = 0;
#if JVET_W0123_TIMD_FUSION
  if (dirMode > DC_IDX && dirMode < (bExtIntraDir ? EXT_VDIA_IDX + 1 : NUM_LUMA_MODE)) // intraPredAngle for directional modes
#else
  if (dirMode > DC_IDX && dirMode < NUM_LUMA_MODE) // intraPredAngle for directional modes
#endif
  {
    static const int angTable[32]    = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };
    static const int invAngTable[32] = {
      0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
      512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
    };   // (512 * 32) / Angle
#if JVET_W0123_TIMD_FUSION
    static const int extAngTable[64]    = { 0, 1, 2, 3, 4, 5, 6,7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 74, 78, 84, 90, 96, 102, 108, 114, 121, 128, 137, 146, 159, 172, 188, 204, 230, 256, 299, 342, 427, 512, 597, 682, 853, 1024, 1536, 2048, 3072 };
    static const int extInvAngTable[64] = {
        0, 32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3277, 2731, 2341, 2048, 1820, 1638, 1489, 1365, 1260, 1170, 1092, 1024, 964, 910, 862, 819, 762, 712, 669, 630, 596, 565, 537, 512, 489, 468, 443, 420, 390, 364, 341, 321, 303, 287, 271, 256, 239, 224, 206, 191, 174, 161, 142, 128, 110, 96, 77, 64, 55, 48, 38, 32, 21, 16, 11
    };   // (512 * 64) / Angle
#endif

    const int     absAngMode         = abs(intraPredAngleMode);
    const int     signAng            = intraPredAngleMode < 0 ? -1 : 1;
#if JVET_W0123_TIMD_FUSION
                  absAng             = bExtIntraDir ? extAngTable[absAngMode] : angTable[absAngMode];
    m_ipaParam.absInvAngle              = bExtIntraDir ? extInvAngTable[absAngMode] : invAngTable[absAngMode];
#else
                  absAng             = angTable  [absAngMode];

    m_ipaParam.absInvAngle           = invAngTable[absAngMode];
#endif
    m_ipaParam.intraPredAngle        = signAng * absAng;
    if (intraPredAngleMode < 0)
    {
      m_ipaParam.applyPDPC = false;
    }
    else if (intraPredAngleMode > 0)
    {
      const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
      const int maxScale = 2;
#if GRAD_PDPC
      m_ipaParam.useGradPDPC = false;
#endif

      m_ipaParam.angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.absInvAngle - 2) - 8));
#if GRAD_PDPC
      if ((m_ipaParam.angularScale < 0) && (isLuma(compId)))
      {
        m_ipaParam.angularScale = (floorLog2(puSize.width) + floorLog2(puSize.height) - 2) >> 2;
        m_ipaParam.useGradPDPC = true;
      }
#endif
      m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
    }
  }

  // high level conditions and DC intra prediction
  if(   sps.getSpsRangeExtension().getIntraSmoothingDisabledFlag()
    || !isLuma( chType )
    || useISP
#if JVET_V0130_INTRA_TMP
    || PU::isTmp(pu, chType)
#endif
    || PU::isMIP( pu, chType )
    || m_ipaParam.multiRefIndex
    || DC_IDX == dirMode
    )
  {
  }
  else if ((isLuma(chType) && pu.cu->bdpcmMode) || (!isLuma(chType) && pu.cu->bdpcmModeChroma)) // BDPCM
  {
    m_ipaParam.refFilterFlag = false;
  }
  else if (dirMode == PLANAR_IDX) // Planar intra prediction
  {
    m_ipaParam.refFilterFlag = puSize.width * puSize.height > 32 ? true : false;
  }
  else if (!useISP)// HOR, VER and angular modes (MDIS)
  {
    bool filterFlag = false;
    {
#if JVET_W0123_TIMD_FUSION
      const int diff = std::min<int>( abs( predMode - (bExtIntraDir ? EXT_HOR_IDX : HOR_IDX) ), abs( predMode - (bExtIntraDir ? EXT_VER_IDX : VER_IDX) ) );
#else
      const int diff = std::min<int>( abs( predMode - HOR_IDX ), abs( predMode - VER_IDX ) );
#endif
      const int log2Size = ((floorLog2(puSize.width) + floorLog2(puSize.height)) >> 1);
      CHECK( log2Size >= MAX_INTRA_FILTER_DEPTHS, "Size not supported" );
#if JVET_W0123_TIMD_FUSION
      filterFlag = (diff > (bExtIntraDir ? m_aucIntraFilterExt[log2Size] : m_aucIntraFilter[log2Size]));
#if JVET_AC0094_REF_SAMPLES_OPT
      filterFlag = filterFlag && (puSize.width * puSize.height > 32);
#endif
#else
      filterFlag = (diff > m_aucIntraFilter[log2Size]);
#endif
    }

    // Selelection of either ([1 2 1] / 4 ) refrence filter OR Gaussian 4-tap interpolation filter
    if (filterFlag)
    {
#if JVET_W0123_TIMD_FUSION
      const bool isRefFilter       =  bExtIntraDir ? isIntegerSlopeExt(absAng) : isIntegerSlope(absAng);
#else
      const bool isRefFilter       =  isIntegerSlope(absAng);
#endif
      CHECK( puSize.width * puSize.height <= 32, "DCT-IF interpolation filter is always used for 4x4, 4x8, and 8x4 luma CB" );
      m_ipaParam.refFilterFlag     =  isRefFilter;
      m_ipaParam.interpolationFlag = !isRefFilter;
    }
  }

#if JVET_AB0157_INTRA_FUSION
  m_ipaParam.applyFusion = false;
  if (m_ipaParam.fetchRef2nd)
  {
    bool isAngularMode = !(dirMode == PLANAR_IDX || dirMode == DC_IDX);

#if JVET_W0123_TIMD_FUSION
    const bool isIntSlope       =  bExtIntraDir ? isIntegerSlopeExt(absAng) : isIntegerSlope(absAng);
#else
    const bool isIntSlope       =  isIntegerSlope(absAng);
#endif

    m_ipaParam.applyFusion = isAngularMode && !isIntSlope && !useISP;

  }
#endif
}


/** Function for deriving the simplified angular intra predictions.
*
* This function derives the prediction samples for the angular mode based on the prediction direction indicated by
* the prediction mode index. The prediction direction is given by the displacement of the bottom row of the block and
* the reference row above the block in the case of vertical prediction or displacement of the rightmost column
* of the block and reference column left from the block in the case of the horizontal prediction. The displacement
* is signalled at 1/32 pixel accuracy. When projection of the predicted pixel falls inbetween reference samples,
* the predicted value for the pixel is linearly interpolated from the reference samples. All reference samples are taken
* from the extended main reference.
*/
//NOTE: Bit-Limit - 25-bit source

#if JVET_W0123_TIMD_FUSION
#if JVET_AB0157_INTRA_FUSION
void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir, const CPelBuf &pSrc2nd, bool isISP, int weightMode)
#else
void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng, const bool bExtIntraDir)
#endif
#else
void IntraPrediction::xPredIntraAng( const CPelBuf &pSrc, PelBuf &pDst, const ChannelType channelType, const ClpRng& clpRng)
#endif
{
  int width =int(pDst.width);
  int height=int(pDst.height);

  const bool bIsModeVer     = m_ipaParam.isModeVer;
  const int  multiRefIdx    = m_ipaParam.multiRefIndex;
#if JVET_AB0157_INTRA_FUSION
  const int  multiRefIdx2nd = multiRefIdx + 1;
#endif
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  absInvAngle    = m_ipaParam.absInvAngle;

#if JVET_AB0157_INTRA_FUSION
  weightMode = m_ipaParam.applyFusion ? weightMode : 4;

  bool bRefL1Only =  weightMode ==  0;
  bool bRefL0Only =  weightMode ==  4;

  Pel* refMain = 0;
  Pel* refSide = 0;
#else
  Pel* refMain;
  Pel* refSide;
#endif

#if !INTRA_6TAP
  Pel  refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel  refLeft [2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
#else
  // 2 pixels more for 6 tap filter.
#if JVET_AC0094_REF_SAMPLES_OPT
  Pel refAbove[(MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX];
  Pel refLeft[(MAX_CU_SIZE << 3) + 5 + 33 * MAX_REF_LINE_IDX];
#else
  Pel  refAbove[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];
  Pel  refLeft[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];
#endif
#if JVET_AB0157_INTRA_FUSION
  if( !bRefL1Only)
  {
#endif
  // initializing for safeguard.
  ::memset(refAbove, 0, sizeof(refAbove));
  ::memset(refLeft, 0, sizeof(refLeft));
#if JVET_AB0157_INTRA_FUSION
  }

  Pel *refMain2nd = NULL;
  Pel *refSide2nd = NULL;
#if JVET_AC0094_REF_SAMPLES_OPT
  Pel refAbove2nd[(MAX_CU_SIZE << 3) + 10 + 33 * (MAX_REF_LINE_IDX + 1)];
  Pel refLeft2nd[(MAX_CU_SIZE << 3) + 10 + 33 * (MAX_REF_LINE_IDX + 1)];
#else
  Pel refAbove2nd[2 * MAX_CU_SIZE + 10 + 33 * (MAX_REF_LINE_IDX+1)];
  Pel refLeft2nd[2 * MAX_CU_SIZE + 10 + 33 * (MAX_REF_LINE_IDX+1)];
#endif
#endif
#endif

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
#if INTRA_6TAP
    // x, y range increase by 1 (right extend)
    const Pel *src = pSrc.buf;
    Pel *dst = refAbove + height + 1;
    ::memcpy(dst, src, sizeof(Pel) * (width + 2 + multiRefIdx + 1));

    src = pSrc.buf + pSrc.stride;
    dst = refLeft + width + 1;
    ::memcpy(dst, src, sizeof(Pel) * (height + 2 + multiRefIdx + 1));

    refMain = bIsModeVer ? refAbove + height + 1 : refLeft + width + 1;
    refSide = bIsModeVer ? refLeft + width + 1: refAbove + height + 1;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    // left extend by 1
    for (int k = -(sizeSide + 1); k <= -1; k++)
    {
      int frac32precision = (-k * absInvAngle + 8) >> 4;
      int intpel = frac32precision >> 5;
      int fracpel = frac32precision & 31;
      //std::cout << " fracPel: " << fracpel << std::endl;
      int left_minus1 = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel - 1)];
      int left        = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel)];
      int right       = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel + 1)];
      int right_plus1 = refSide[Clip3(0, sizeSide + 2 + multiRefIdx, intpel + 2)];

      const TFilterCoeff* f = InterpolationFilter::getWeak4TapFilterTable(fracpel);
      int val = ((int)f[0] * left_minus1 + (int)f[1] * left + (int)f[2] * right + f[3] * (int)right_plus1 + 32) >> 6;
      refMain[k] = (Pel)ClipPel(val, clpRng);
    }
#else
    for (int x = 0; x <= width + 1 + multiRefIdx; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= height + 1 + multiRefIdx; y++)
    {
      refLeft[y + width] = pSrc.at(y, 1);
    }
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;

    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
    }
#endif
  }
  else
  {
#if INTRA_6TAP
    const Pel *src = pSrc.buf;
    Pel *dst = refAbove + 1;
    ::memcpy(dst, src, sizeof(Pel) * (m_topRefLength + multiRefIdx + 1));

    src = pSrc.buf + pSrc.stride;
    dst = refLeft + 1;
    ::memcpy(dst, src, sizeof(Pel) * (m_leftRefLength + multiRefIdx + 1));

    // left extended by 1
    refAbove[0] = refAbove[1];
    refLeft[0] = refLeft[1];
    refMain = bIsModeVer ? refAbove + 1 : refLeft + 1;
    refSide = bIsModeVer ? refLeft + 1 : refAbove + 1;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
#if JVET_W0123_TIMD_FUSION
    const int maxIndex  = (multiRefIdx << s) + 6;
#else
    const int maxIndex = (multiRefIdx << s) + 2;
#endif
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val = refMain[refLength + multiRefIdx];
    // right extended by 1 (z range)
    for (int z = 1; z <= (maxIndex + 1); z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
#else
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(y, 1);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width) - floorLog2(height);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
#if JVET_W0123_TIMD_FUSION
    const int maxIndex  = (multiRefIdx << s) + 6;
#else
    const int maxIndex  = (multiRefIdx << s) + 2;
#endif
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
#endif
  }


#if JVET_AB0157_INTRA_FUSION
  if (!bRefL0Only)
  {
    // initializing for safeguard.
    ::memset(refAbove2nd, 0, sizeof(refAbove2nd));
    ::memset(refLeft2nd, 0, sizeof(refAbove2nd));
    // Initialize the Main and Left reference array.
    if (intraPredAngle < 0)
    {
      // x, y range increase by 1 (right extend)
      const Pel *src = pSrc2nd.buf;
      Pel *      dst = refAbove2nd + height + 2;
      ::memcpy(dst, src, sizeof(Pel) * (m_topRefLength + 1 + multiRefIdx2nd));

      src = pSrc2nd.buf + pSrc2nd.stride;
      dst = refLeft2nd + width + 2;
      ::memcpy(dst, src, sizeof(Pel) * (m_leftRefLength + 1 + multiRefIdx2nd));

      refMain2nd = bIsModeVer ? refAbove2nd + height + 2 : refLeft2nd + width + 2;
      refSide2nd = bIsModeVer ? refLeft2nd + width + 2 : refAbove2nd + height + 2;

      // Extend the Main reference to the left.
      int sizeSide = bIsModeVer ? height : width;
      int sizeSideRange = bIsModeVer ? m_leftRefLength + multiRefIdx2nd : m_topRefLength + multiRefIdx2nd;
      // left extend by 1
      for (int k = -(sizeSide + 2); k <= -1; k++)
      {
        int frac32precision = (-k * absInvAngle + 8) >> 4;
        int intpel          = frac32precision >> 5;
        int fracpel         = frac32precision & 31;
        // std::cout << " fracPel: " << fracpel << std::endl;
        int leftMinus1  = refSide2nd[Clip3(0, sizeSideRange, intpel - 1)];
        int left        = refSide2nd[Clip3(0, sizeSideRange, intpel)];
        int right       = refSide2nd[Clip3(0, sizeSideRange, intpel + 1)];
        int rightPlus1  = refSide2nd[Clip3(0, sizeSideRange, intpel + 2)];

        const TFilterCoeff *f = InterpolationFilter::getWeak4TapFilterTable(fracpel);
        int                 val =
          ((int) f[0] * leftMinus1 + (int) f[1] * left + (int) f[2] * right + f[3] * (int) rightPlus1 + 32) >> 6;
        refMain2nd[k] = (Pel) ClipPel(val, clpRng);
      }
    }
    else
    {
      const Pel *src = pSrc2nd.buf;
      Pel *      dst = refAbove2nd + 1;
      ::memcpy(dst, src, sizeof(Pel) * (m_topRefLength + multiRefIdx2nd + 1));

      src = pSrc2nd.buf + pSrc2nd.stride;
      dst = refLeft2nd + 1;
      ::memcpy(dst, src, sizeof(Pel) * (m_leftRefLength + multiRefIdx2nd + 1));

      // left extended by 1
      refAbove2nd[0] = refAbove2nd[1];
      refLeft2nd[0]  = refLeft2nd[1];
      refMain2nd     = bIsModeVer ? refAbove2nd + 1 : refLeft2nd + 1;
      refSide2nd     = bIsModeVer ? refLeft2nd + 1 : refAbove2nd + 1;

      const int log2Ratio = floorLog2(width) - floorLog2(height);
      const int s = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);

      // Extend main reference to right using replication
#if JVET_W0123_TIMD_FUSION
      const int maxIndex = (multiRefIdx2nd << s) + 6 + 4;
#else
      const int maxIndex = (multiRefIdx2nd << s) + 2;
#endif
      const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
      const Pel val       = refMain2nd[refLength + multiRefIdx2nd];
      // right extended by 1 (z range)
      for (int z = 1; z <= (maxIndex + 1); z++)
      {
        refMain2nd[refLength + multiRefIdx2nd + z] = val;
      }
    }


    refMain2nd += multiRefIdx2nd;
    refSide2nd += multiRefIdx2nd;
  }

#endif

  // swap width/height if we are doing a horizontal mode:
  if (!bIsModeVer)
  {
    std::swap(width, height);
  }
  Pel       tempArray[MAX_CU_SIZE * MAX_CU_SIZE];
  const int dstStride = bIsModeVer ? pDst.stride : width;
  Pel *     pDstBuf   = bIsModeVer ? pDst.buf : tempArray;

  // compensate for line offset in reference line buffers
  refMain += multiRefIdx;
  refSide += multiRefIdx;

  Pel *pDsty = pDstBuf;

  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    for( int y = 0; y < height; y++ )
    {
      ::memcpy(pDsty, refMain + 1, width * sizeof(Pel));

      if (m_ipaParam.applyPDPC)
      {
        const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        const Pel topLeft = refMain[0];
        const Pel left    = refSide[1 + y];
        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          const int wL  = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x]      = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }

      pDsty += dstStride;
    }
  }
  else
  {
    for (int y = 0, deltaPos = intraPredAngle * (1 + multiRefIdx); y<height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
#if JVET_W0123_TIMD_FUSION
      const int deltaInt   = bExtIntraDir ? deltaPos >> 6 : deltaPos >> 5;
      const int deltaFract = bExtIntraDir ? deltaPos & 63 : deltaPos & 31;
#if JVET_AB0157_INTRA_FUSION
      const int deltaInt2nd   = bExtIntraDir ? (deltaPos + intraPredAngle) >> 6 : (deltaPos + intraPredAngle) >> 5;
      const int deltaFract2nd = bExtIntraDir ? (deltaPos + intraPredAngle) & 63 : (deltaPos + intraPredAngle) & 31;
#endif
#else
      const int deltaInt   = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;
#endif

#if JVET_W0123_TIMD_FUSION
      bool bIntSlope = bExtIntraDir ? isIntegerSlopeExt( abs(intraPredAngle) ) : isIntegerSlope( abs(intraPredAngle) );
      if ( !bIntSlope )
#else
      if ( !isIntegerSlope( abs(intraPredAngle) ) )
#endif
      {
        if( isLuma(channelType) )
        {
          const bool useCubicFilter = !m_ipaParam.interpolationFlag;

#if INTRA_6TAP
          const TFilterCoeff        intraSmoothingFilter[6] = { TFilterCoeff(0), TFilterCoeff(64 - (deltaFract << 1)), TFilterCoeff(128 - (deltaFract << 1)), TFilterCoeff(64 + (deltaFract << 1)), TFilterCoeff(deltaFract << 1), TFilterCoeff(0) };
          const TFilterCoeff        intraSmoothingFilter2[6] = { TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(64 - 3*(deltaFract >> 1)), TFilterCoeff(96 - (deltaFract)), TFilterCoeff(64 + (deltaFract)),
            TFilterCoeff(16 + 3*(deltaFract >> 1)), TFilterCoeff((deltaFract >> 1)) };
#if JVET_W0123_TIMD_FUSION
          const TFilterCoeff        intraSmoothingFilterExt[6] = { TFilterCoeff(0), TFilterCoeff(64 - (deltaFract)), TFilterCoeff(128 - (deltaFract)), TFilterCoeff(64 + (deltaFract)), TFilterCoeff(deltaFract), TFilterCoeff(0) };
          const TFilterCoeff        intraSmoothingFilter2Ext[6] = { TFilterCoeff(16 - (deltaFract >> 2)), TFilterCoeff(64 - 3*(deltaFract >> 2)), TFilterCoeff(96 - (deltaFract >> 1)), TFilterCoeff(64 + (deltaFract >> 1)),
            TFilterCoeff(16 + 3*(deltaFract >> 2)), TFilterCoeff((deltaFract >> 2)) };
          const TFilterCoeff* const f = (useCubicFilter) ? ( bExtIntraDir ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract) : InterpolationFilter::getIntraLumaFilterTable(deltaFract)) : (width >=32 && height >=32)? (bExtIntraDir ? intraSmoothingFilter2Ext : intraSmoothingFilter2) : (bExtIntraDir ? intraSmoothingFilterExt : intraSmoothingFilter);
#else
          const TFilterCoeff* const f = (useCubicFilter) ? InterpolationFilter::getIntraLumaFilterTable(deltaFract) : (width >=32 && height >=32)? intraSmoothingFilter2 : intraSmoothingFilter;
#endif
#else
#if IF_12TAP
          const TFilterCoeff        intraSmoothingFilter[4] = { TFilterCoeff(64 - (deltaFract << 1)), TFilterCoeff(128 - (deltaFract << 1)), TFilterCoeff(64 + (deltaFract << 1)), TFilterCoeff(deltaFract << 1) };   
#if JVET_W0123_TIMD_FUSION
          const TFilterCoeff        intraSmoothingFilterExt[4] = { TFilterCoeff(64 - (deltaFract)), TFilterCoeff(128 - (deltaFract)), TFilterCoeff(64 + (deltaFract)), TFilterCoeff(deltaFract) };
#endif
#else
          const TFilterCoeff        intraSmoothingFilter[4] = {TFilterCoeff(16 - (deltaFract >> 1)), TFilterCoeff(32 - (deltaFract >> 1)), TFilterCoeff(16 + (deltaFract >> 1)), TFilterCoeff(deltaFract >> 1)};
#endif  

#if JVET_W0123_TIMD_FUSION
          const TFilterCoeff* const f                       = (useCubicFilter) ? (bExtIntraDir ? InterpolationFilter::getExtIntraCubicFilter(deltaFract) : InterpolationFilter::getChromaFilterTable(deltaFract)) : (bExtIntraDir ? InterpolationFilter::getExtIntraGaussFilter(deltaFract) : intraSmoothingFilter);
#else
          const TFilterCoeff* const f                       = (useCubicFilter) ? InterpolationFilter::getChromaFilterTable(deltaFract) : intraSmoothingFilter;
#endif
#endif

#if JVET_AB0157_INTRA_FUSION
          const TFilterCoeff        intraSmoothingFilterRL[6] = { TFilterCoeff(0), TFilterCoeff(64 - (deltaFract2nd << 1)), TFilterCoeff(128 - (deltaFract2nd << 1)), TFilterCoeff(64 + (deltaFract2nd << 1)), TFilterCoeff(deltaFract2nd << 1), TFilterCoeff(0) };
          const TFilterCoeff        intraSmoothingFilter2RL[6] = { TFilterCoeff(16 - (deltaFract2nd >> 1)), TFilterCoeff(64 - 3*(deltaFract2nd >> 1)), TFilterCoeff(96 - (deltaFract2nd)), TFilterCoeff(64 + (deltaFract2nd)),
            TFilterCoeff(16 + 3*(deltaFract2nd >> 1)), TFilterCoeff((deltaFract2nd >> 1)) };
#if JVET_W0123_TIMD_FUSION
          const TFilterCoeff        intraSmoothingFilterExtRL[6] = { TFilterCoeff(0), TFilterCoeff(64 - (deltaFract2nd)), TFilterCoeff(128 - (deltaFract2nd)), TFilterCoeff(64 + (deltaFract2nd)), TFilterCoeff(deltaFract2nd), TFilterCoeff(0) };
          const TFilterCoeff        intraSmoothingFilter2ExtRL[6] = { TFilterCoeff(16 - (deltaFract2nd >> 2)), TFilterCoeff(64 - 3*(deltaFract2nd >> 2)), TFilterCoeff(96 - (deltaFract2nd >> 1)), TFilterCoeff(64 + (deltaFract2nd >> 1)),
            TFilterCoeff(16 + 3*(deltaFract2nd >> 2)), TFilterCoeff((deltaFract2nd >> 2)) };
          const TFilterCoeff* const fRL = (useCubicFilter) ? ( bExtIntraDir ? InterpolationFilter::getIntraLumaFilterTableExt(deltaFract2nd) : InterpolationFilter::getIntraLumaFilterTable(deltaFract2nd)) : (width >=32 && height >=32)? (bExtIntraDir ? intraSmoothingFilter2ExtRL : intraSmoothingFilter2RL) : (bExtIntraDir ? intraSmoothingFilterExtRL : intraSmoothingFilterRL);
#else
          const TFilterCoeff* const fRL = (useCubicFilter) ? InterpolationFilter::getIntraLumaFilterTable(deltaFract2nd) : (width >=32 && height >=32)? intraSmoothingFilter2RL : intraSmoothingFilterRL;
#endif
          if (weightMode == 0)
          {
            for (int x = 0; x < width; x++)
            {
              int val32 = 0;
              Pel *q     = refMain2nd + deltaInt2nd + x - 1;
              val32 = ( ((int) fRL[0] * q[0] + (int) fRL[1] * q[1] + (int) fRL[2] * q[2] + (int) fRL[3] * q[3] + (int) fRL[4] * q[4] + (int) fRL[5] * q[5]) + 128) >> 8;
              Pel val = (Pel)ClipPel(val32, clpRng);
              pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
            }
          }
          else if(weightMode == 4)
          {
            for (int x = 0; x < width; x++)
            {
              int val32 = 0;
              Pel *p     = refMain + deltaInt + x - 1;
              val32 = ((int) f[0] * p[0] + (int) f[1] * p[1] + (int) f[2] * p[2] + (int) f[3] * p[3] + (int) f[4] * p[4] + (int) f[5] * p[5] + 128) >> 8;
              Pel val = (Pel)ClipPel(val32, clpRng);
              pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
            }
          }
          else
          {
            for (int x = 0; x < width; x++)
            {
              int val32 = 0;
              int w0 = weightMode, w1;
              w1 = 4 - weightMode;

              Pel *q     = refMain2nd + deltaInt2nd + x - 1;
              Pel *p     = refMain + deltaInt + x - 1;
              val32 = (w0*((int) f[0] * p[0] + (int) f[1] * p[1] + (int) f[2] * p[2] + (int) f[3] * p[3] + (int) f[4] * p[4] + (int) f[5] * p[5])
                  + w1*((int) fRL[0] * q[0] + (int) fRL[1] * q[1] + (int) fRL[2] * q[2] + (int) fRL[3] * q[3] + (int) fRL[4] * q[4] + (int) fRL[5] * q[5]) + 512) >> 10;
              Pel val = (Pel)ClipPel(val32, clpRng);
              pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
            }
          }
#else
          for (int x = 0; x < width; x++)
          {
#if INTRA_6TAP
            Pel* p = refMain + deltaInt + x - 1;
            int val32 = ((int)f[0] * p[0] + (int)f[1] * p[1] + (int)f[2] * p[2] + (int)f[3] * p[3] + (int)f[4] * p[4] + (int)f[5] * p[5] + 128) >> 8;
            Pel val = (Pel)ClipPel(val32, clpRng);
#else
            Pel p[4];

            p[0] = refMain[deltaInt + x];
            p[1] = refMain[deltaInt + x + 1];
            p[2] = refMain[deltaInt + x + 2];
            p[3] = refMain[deltaInt + x + 3];

#if IF_12TAP
            Pel val = ( f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 128 ) >> 8;
#else
#if JVET_W0123_TIMD_FUSION
            int tOffset = 32;
            int tShift = 6;
            if (bExtIntraDir)
            {
              tOffset = 128;
              tShift = 8;
            }
            Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + tOffset) >> tShift;
#else
            Pel val = (f[0] * p[0] + f[1] * p[1] + f[2] * p[2] + f[3] * p[3] + 32) >> 6;
#endif
#endif
#endif

            pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
          }
#endif
        }
        else
        {
          // Do linear filtering
          for (int x = 0; x < width; x++)
          {
            Pel* p = refMain + deltaInt + x + 1;
            pDsty[x] = p[0] + ((deltaFract * (p[1] - p[0]) + 16) >> 5);
          }
        }
      }
      else
      {
        // Just copy the integer samples
        ::memcpy(pDsty, refMain + deltaInt + 1, width * sizeof(Pel));
      }
#if GRAD_PDPC
#if JVET_AB0157_INTRA_FUSION
      if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC && !bRefL1Only)
#else
      if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
#endif
      {
        const int scale = m_ipaParam.angularScale;
        const Pel left = refSide[1 + y];
#if JVET_W0123_TIMD_FUSION
        int gradOffset = 16;
        int gradShift = 5;
        if (bExtIntraDir)
        {
          gradOffset = 32;
          gradShift = 6;
        }
        const Pel topLeft = refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + gradOffset) >> gradShift);
#else
        const Pel topLeft = refMain[deltaInt] + ((deltaFract * (refMain[deltaInt + 1] - refMain[deltaInt]) + 16) >> 5);
#endif

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          int wL = 32 >> (2 * x >> scale);
          const Pel val = pDsty[x];
          pDsty[x] = ClipPel(val + ((wL * (left - topLeft) + 32) >> 6), clpRng);
        }
      }
      else
#endif
#if JVET_AB0157_INTRA_FUSION
      if (m_ipaParam.applyPDPC && !bRefL1Only)
#else
      if (m_ipaParam.applyPDPC)
#endif
      {
        const int scale       = m_ipaParam.angularScale;
        int       invAngleSum = 256;

        for (int x = 0; x < std::min(3 << scale, width); x++)
        {
          invAngleSum += absInvAngle;

          int wL   = 32 >> (2 * x >> scale);
          Pel left = refSide[y + (invAngleSum >> 9) + 1];
          pDsty[x] = pDsty[x] + ((wL * (left - pDsty[x]) + 32) >> 6);
        }
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if( !bIsModeVer )
  {
    for( int y = 0; y < height; y++ )
    {
      Pel *dst = pDst.buf + y;

      for( int x = 0; x < width; x++ )
      {
        *dst = pDstBuf[x];
        dst += pDst.stride;
      }
      pDstBuf += dstStride;
    }
  }
}

#if JVET_Z0050_DIMD_CHROMA_FUSION
#if JVET_AD0188_CCP_MERGE
void IntraPrediction::geneChromaFusionPred( const ComponentID compId, PelBuf &piPred, PredictionUnit &pu )
#else
void IntraPrediction::geneChromaFusionPred(const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu)
#endif
{
  int width = piPred.width;
  int height = piPred.height;
  const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, width, height));

  PelBuf predLmBuffer = m_tempBuffer[0].getBuf(localUnitArea.Y());
  PredictionUnit pu2 = pu;

#if MMLM
  pu2.intraDir[1] = MMLM_CHROMA_IDX;
#else
  pu2.intraDir[1] = LM_CHROMA_IDX;
#endif

  const CompArea &area = pu2.blocks[compId];

#if JVET_AC0119_LM_CHROMA_FUSION
  if (!pu.cs->pcv->isEncoder || !pu2.cs->slice->isIntra() || !CS::isDualITree(*pu.cs))
  {
#endif
  xGetLumaRecPixels(pu2, area);
#if JVET_AC0119_LM_CHROMA_FUSION
  }
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
  if (pu.isChromaFusion > 1)
  {
    pu2.intraDir[1] = LM_CHROMA_IDX + pu.isChromaFusion - 2;
    if (!pu.cs->pcv->isEncoder || !pu2.cs->slice->isIntra() || !CS::isDualITree(*pu.cs))
    {
      xCflmCreateChromaPred(pu, compId, piPred);
    }
    predIntraChromaLM(compId, piPred, pu2, area, pu2.intraDir[1]);
    return;
  }
#endif

#if JVET_AD0120_LBCCP
  static int cclmSAD  = MAX_INT;
  static int cccmSAD  = MAX_INT;
#if JVET_AA0057_CCCM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (pu.cs->slice->isIntra() && PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX) && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
  bool isMultiCccmFullEnabled = false;
  isMultiCccmFullEnabled = PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX);
  if (pu.cs->slice->isIntra() && isMultiCccmFullEnabled)
#endif
  {
    const int  bitDepth = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
    const CompArea                   &areaCb = pu2.blocks[COMPONENT_Cb];
    const CompArea                   &areaCr = pu2.blocks[COMPONENT_Cr];
    static CclmModel                  cclmModelCb;
    static CclmModel                  cclmModelCr;
    static int                        modelThr       = 0;
    static CccmModel cccmModelCb[2] = { CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth) };
    static CccmModel cccmModelCr[2] = { CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth) };

    if (compId == COMPONENT_Cb)
    {
      pu2.cccmFlag = 1;
#if JVET_AC0054_GLCCCM
      pu2.glCccmFlag = 0;
#endif
#if JVET_AD0202_CCCM_MDF
      pu2.cccmMultiFilterIdx = 0;
#endif
      xGetLumaRecPixels(pu2, area);
      modelThr = xCccmCalcRefAver(pu2);
      xCccmCalcModels(pu2, cccmModelCb[0], cccmModelCr[0], 1, modelThr);
      xCccmCalcModels(pu2, cccmModelCb[1], cccmModelCr[1], 2, modelThr);
      cccmSAD = xCalculateCCCMcost(pu2, COMPONENT_Cb, pu2.intraDir[1], areaCb, cccmModelCb, modelThr);
      cccmSAD += xCalculateCCCMcost(pu2, COMPONENT_Cr, pu2.intraDir[1], areaCr, cccmModelCr, modelThr);

      pu2.cccmFlag = 0;
      xGetLumaRecPixels(pu2, area);
      initIntraPatternChType(*pu.cu, areaCb);
      xGetLMParametersLMS(pu2, COMPONENT_Cb, areaCb, cclmModelCb);
      initIntraPatternChType(*pu.cu, areaCr);
      xGetLMParametersLMS(pu2, COMPONENT_Cr, areaCr, cclmModelCr);
      cclmSAD = xCalculateCCLMcost(pu2, COMPONENT_Cb, pu2.intraDir[1], areaCb, cclmModelCb);
      cclmSAD += xCalculateCCLMcost(pu2, COMPONENT_Cr, pu2.intraDir[1], areaCr, cclmModelCr);
    }

    if (cccmSAD < cclmSAD)
    {
      xCccmApplyModel(pu2, compId, compId == COMPONENT_Cb ? cccmModelCb[0] : cccmModelCr[0], 1, modelThr, predLmBuffer);
      xCccmApplyModel(pu2, compId, compId == COMPONENT_Cb ? cccmModelCb[1] : cccmModelCr[1], 2, modelThr, predLmBuffer);
    }
    else
    {
      applyChromaLM(compId, predLmBuffer, pu2, area, pu2.intraDir[1], compId == COMPONENT_Cb ? cclmModelCb : cclmModelCr);
    }
#if JVET_AD0188_CCP_MERGE
    if (compId == COMPONENT_Cr)
    {
      if (cccmSAD < cclmSAD)
      {
#if JVET_AC0054_GLCCCM
        pu.curCand.type = ((pu.glCccmFlag ? CCP_TYPE_GLCCCM : CCP_TYPE_CCCM) | CCP_TYPE_MMLM);
        pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
        pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;
#else
        pu.curCand.type = (CCP_TYPE_CCCM | CCP_TYPE_MMLM);
#endif
        PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, modelThr
#if JVET_AB0174_CCCM_DIV_FREE
                                 , m_cccmLumaOffset
#endif
        );
      }
      else
      {
        pu.curCand.type = (CCP_TYPE_CCLM | CCP_TYPE_MMLM);
        PU::cclmModelToCcpParams(COMPONENT_Cb, pu.curCand, cclmModelCb);
        PU::cclmModelToCcpParams(COMPONENT_Cr, pu.curCand, cclmModelCr);
      }
    }
#endif
  }
  else
#endif
  {
#endif
    predIntraChromaLM(compId, predLmBuffer, pu2, area, pu2.intraDir[1]);

#if JVET_AD0188_CCP_MERGE
    pu.curCand      = pu2.curCand;
    pu.curCand.type = CCP_TYPE_CCLM;

    if (PU::isMultiModeLM(pu2.intraDir[1]))
    {
      pu.curCand.type |= CCP_TYPE_MMLM;
    }
#endif
#if JVET_AD0120_LBCCP
  }
#endif

#if JVET_AC0071_DBV && JVET_AA0070_RRIBC
  if (pu.intraDir[1] == DBV_CHROMA_IDX && pu.cu->rribcFlipType != 0)
  {
    predLmBuffer.flipSignal(pu.cu->rribcFlipType == 1);
  }
#endif

  Pel *pelPred = piPred.buf;
  Pel *pelLm = predLmBuffer.buf;
  int  w0 = 2;
  int  w1 = 2;
  int  shift = 2;

#if JVET_AD0120_LBCCP
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (pu.cs->slice->isIntra() && PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX) && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
  if (pu.cs->slice->isIntra() && isMultiCccmFullEnabled)
#endif
  {
    const bool aboveAvailable = pu.cu->cs->getCU(pu.blocks[compId].pos().offset(0, -1), toChannelType(compId)) ? true : false;
    const bool leftAvailable = pu.cu->cs->getCU(pu.blocks[compId].pos().offset(-1, 0), toChannelType(compId)) ? true : false;
    int numSample = (aboveAvailable ? width : 0) + (leftAvailable ? height : 0);

    if (numSample > 0)
    {
      int bestSAD = cccmSAD < cclmSAD ? cccmSAD : cclmSAD;
      if (bestSAD > 64 * numSample)
      {
        w0 = 3;
        w1 = 1;
      }
      else if (bestSAD < 4 * numSample)
      {
        w0 = 1;
        w1 = 3;
      }
    }
  }
  else
#endif
  if (pu.cs->slice->isIntra())
  {
    const Position posBL = pu.Cb().bottomLeft();
    const Position posTR = pu.Cb().topRight();
    const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_CHROMA);
    const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_CHROMA);
    bool isNeigh0LM = neigh0 && PU::isLMCMode(neigh0->intraDir[1]);
    bool isNeigh1LM = neigh1 && PU::isLMCMode(neigh1->intraDir[1]);

    if (isNeigh0LM && isNeigh1LM)
    {
      w0 = 1; w1 = 3;
    }
    else if (!isNeigh0LM && !isNeigh1LM)
    {
      w0 = 3; w1 = 1;
    }
  }

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int blend = pelPred[x] * w0;
      blend += pelLm[x] * w1;
      blend += 2;
      pelPred[x] = (Pel) (blend >> shift);
    }
    pelPred += piPred.stride;
    pelLm += predLmBuffer.stride;
  }
}
#endif

void IntraPrediction::xPredIntraBDPCM(const CPelBuf &pSrc, PelBuf &pDst, const uint32_t dirMode, const ClpRng& clpRng )
{
  const int wdt = pDst.width;
  const int hgt = pDst.height;

  const int strideP = pDst.stride;
  const int strideS = pSrc.stride;

  CHECK( !( dirMode == 1 || dirMode == 2 ), "Incorrect BDPCM mode parameter." );

  Pel* pred = &pDst.buf[0];
  if( dirMode == 1 )
  {
    Pel  val;
    for( int y = 0; y < hgt; y++ )
    {
      val = pSrc.buf[(y + 1) + strideS];
      for( int x = 0; x < wdt; x++ )
      {
        pred[x] = val;
      }
      pred += strideP;
    }
  }
  else
  {
    for( int y = 0; y < hgt; y++ )
    {
      for( int x = 0; x < wdt; x++ )
      {
        pred[x] = pSrc.buf[x + 1];
      }
      pred += strideP;
    }
  }
}

// Explicit instanciation since the implementation is in cpp file
template void IntraPrediction::geneWeightedPred<false>( const ComponentID compId, AreaBuf<Pel>& pred, const PredictionUnit &pu, const AreaBuf<Pel> &interPred, const AreaBuf<Pel> &intraPred, const Pel* pLUT );
template void IntraPrediction::geneWeightedPred<true>( const ComponentID compId, AreaBuf<Pel>& pred, const PredictionUnit &pu, const AreaBuf<Pel> &interPred, const AreaBuf<Pel> &intraPred, const Pel* pLUT );

template<bool lmcs>
void IntraPrediction::geneWeightedPred( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT )
{
  const int            width = pred.width;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK(width == 2, "Width of 2 is not supported");
#endif
  const int            height = pred.height;
  Pel*                 interPredBuf = interPred.buf;
  const int            interPredStride = interPred.stride;
  Pel*                 intraPredBuf = intraPred.buf;
  const int            intraPredStride = intraPred.stride;
  const int            dstStride = pred.stride;

  Pel*                 dstBuf = pred.buf;
#if CIIP_PDPC
  if (!pu.ciipPDPC)
  {
#endif
  int wIntra, wMerge;

  const Position posBL = pu.Y().bottomLeft();
  const Position posTR = pu.Y().topRight();
  const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);
  const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_LUMA);
  bool isNeigh0Intra = neigh0 && (CU::isIntra(*neigh0->cu));
  bool isNeigh1Intra = neigh1 && (CU::isIntra(*neigh1->cu));

  if (isNeigh0Intra && isNeigh1Intra)
  {
    wIntra = 3; wMerge = 1;
  }
  else
  {
    if (!isNeigh0Intra && !isNeigh1Intra)
    {
      wIntra = 1; wMerge = 3;
    }
    else
    {
      wIntra = 2; wMerge = 2;
    }
  }
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  const ChannelType chType = toChannelType(compId);
  int dirMode = PU::getFinalIntraMode(pu, chType);
  if (dirMode == PLANAR_IDX || dirMode == DC_IDX || width < 4 || height < 4)
  {
    for (int y = 0; y < height; y++)
    {
      for( int x = 0; x < width; x++ )
      {
        if( lmcs )
        {
          dstBuf[x] = ( wMerge * pLUT[interPredBuf[x]] + wIntra * intraPredBuf[x] + 2 ) >> 2;
        }
        else
        {
          dstBuf[x] = ( wMerge * interPredBuf[x] + wIntra * intraPredBuf[x] + 2 ) >> 2;
        }
      }
      dstBuf += dstStride;
      interPredBuf += interPredStride;
      intraPredBuf += intraPredStride;
    }
  }
  else if (dirMode < DIA_IDX)
  {
    int interval = (width >> 2);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        if (x < interval)
        {
          if (lmcs)
          {
            dstBuf[x] = (2 * pLUT[interPredBuf[x]] + 6 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (2 * interPredBuf[x] + 6 * intraPredBuf[x] + 4) >> 3;
          }
        }
        else if (x >= interval && x < (2 * interval))
        {
          if (lmcs)
          {
            dstBuf[x] = (3 * pLUT[interPredBuf[x]] + 5 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (3 * interPredBuf[x] + 5 * intraPredBuf[x] + 4) >> 3;
          }
        }
        else if (x >= (interval * 2) && x < (3 * interval))
        {
          if (lmcs)
          {
            dstBuf[x] = (5 * pLUT[interPredBuf[x]] + 3 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (5 * interPredBuf[x] + 3 * intraPredBuf[x] + 4) >> 3;
          }
        }
        else
        {
          if (lmcs)
          {
            dstBuf[x] = (6 * pLUT[interPredBuf[x]] + 2 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (6 * interPredBuf[x] + 2 * intraPredBuf[x] + 4) >> 3;
          }
        }
      }
      dstBuf += dstStride;
      interPredBuf += interPredStride;
      intraPredBuf += intraPredStride;
    }
  }
  else
  {
    int interval = (height >> 2);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        if (y < interval)
        {
          if (lmcs)
          {
            dstBuf[x] = (2 * pLUT[interPredBuf[x]] + 6 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (2 * interPredBuf[x] + 6 * intraPredBuf[x] + 4) >> 3;
          }
        }
        else if (y >= interval && y < (2 * interval))
        {
          if (lmcs)
          {
            dstBuf[x] = (3 * pLUT[interPredBuf[x]] + 5 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (3 * interPredBuf[x] + 5 * intraPredBuf[x] + 4) >> 3;
          }
        }
        else if (y >= (interval * 2) && y < (3 * interval))
        {
          if (lmcs)
          {
            dstBuf[x] = (5 * pLUT[interPredBuf[x]] + 3 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (5 * interPredBuf[x] + 3 * intraPredBuf[x] + 4) >> 3;
          }
        }
        else
        {
          if (lmcs)
          {
            dstBuf[x] = (6 * pLUT[interPredBuf[x]] + 2 * intraPredBuf[x] + 4) >> 3;
          }
          else
          {
            dstBuf[x] = (6 * interPredBuf[x] + 2 * intraPredBuf[x] + 4) >> 3;
          }
        }
      }
      dstBuf += dstStride;
      interPredBuf += interPredStride;
      intraPredBuf += intraPredStride;
    }
  }
#else
  for (int y = 0; y < height; y++)
  {
    for( int x = 0; x < width; x++ )
    {
      if( lmcs )
      {
        dstBuf[x] = ( wMerge * pLUT[interPredBuf[x]] + wIntra * intraPredBuf[x] + 2 ) >> 2;
      }
      else
      {
        dstBuf[x] = ( wMerge * interPredBuf[x] + wIntra * intraPredBuf[x] + 2 ) >> 2;
      }
    }

    dstBuf += dstStride;
    interPredBuf += interPredStride;
    intraPredBuf += intraPredStride;
  }
#endif
#if CIIP_PDPC
  }
  else
  {
    const int scale = ((floorLog2(width) - 2 + floorLog2(height) - 2 + 2) >> 2);
    for (int y = 0; y < height; y++)
    {
      const int wT = 32 >> std::min(31, ((y << 1) >> scale));
      for (int x = 0; x < width; x++)
      {
        const int wL = 32 >> std::min(31, ((x << 1) >> scale));

        if( lmcs )
        {
          dstBuf[x] = ( Pel ) ClipPel( ( ( int( intraPredBuf[x] ) << 6 ) + ( 64 - wT - wL ) * int( pLUT[interPredBuf[x]] ) + 32 ) >> 6, pu.cs->slice->clpRng( compId ) );
        }
        else
        {
          dstBuf[x] = ( Pel ) ClipPel( ( ( int( intraPredBuf[x] ) << 6 ) + ( 64 - wT - wL ) * int( interPredBuf[x] ) + 32 ) >> 6, pu.cs->slice->clpRng( compId ) );
        }
      }

      dstBuf += dstStride;
      interPredBuf += interPredStride;
      intraPredBuf += intraPredStride;
    }
  }
#endif
}
#if JVET_AG0135_AFFINE_CIIP
template void IntraPrediction::geneWeightedCIIPAffinePred<false>(const ComponentID compId, AreaBuf<Pel>& pred, const PredictionUnit &pu, const AreaBuf<Pel> &interPred, const AreaBuf<Pel> &intraPred, const Pel* pLUT);
template void IntraPrediction::geneWeightedCIIPAffinePred<true>(const ComponentID compId, AreaBuf<Pel>& pred, const PredictionUnit &pu, const AreaBuf<Pel> &interPred, const AreaBuf<Pel> &intraPred, const Pel* pLUT);

template<bool lmcs>
void IntraPrediction::geneWeightedCIIPAffinePred(const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred, const Pel* pLUT)
{
  const int            width = pred.width;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK(width == 2, "Width of 2 is not supported");
#endif
  const int            height = pred.height;
  Pel*                 interPredBuf = interPred.buf;
  const int            interPredStride = interPred.stride;
  Pel*                 intraPredBuf = intraPred.buf;
  const int            intraPredStride = intraPred.stride;
  const int            dstStride = pred.stride;
  Pel*                 dstBuf = pred.buf;

  const Position posBL = pu.Y().bottomLeft();
  const Position posTR = pu.Y().topRight();
  const PredictionUnit *neigh0 = pu.cs->getPURestricted(posBL.offset(-1, 0), pu, CHANNEL_TYPE_LUMA);
  const PredictionUnit *neigh1 = pu.cs->getPURestricted(posTR.offset(0, -1), pu, CHANNEL_TYPE_LUMA);
  bool isNeigh0Intra = neigh0 && (CU::isIntra(*neigh0->cu));
  bool isNeigh1Intra = neigh1 && (CU::isIntra(*neigh1->cu));
  int wIntra, wMerge;
  if (isNeigh0Intra && isNeigh1Intra)
  {
    wIntra = 3; wMerge = 1;
  }
  else
  {
    if (!isNeigh0Intra && !isNeigh1Intra)
    {
      wIntra = 1; wMerge = 3;
    }
    else
    {
      wIntra = 2; wMerge = 2;
    }
  }

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      if (lmcs)
      {
        dstBuf[x] = (wMerge * pLUT[interPredBuf[x]] + wIntra * intraPredBuf[x] + 2) >> 2;
      }
      else
      {
        dstBuf[x] = (wMerge * interPredBuf[x] + wIntra * intraPredBuf[x] + 2) >> 2;
      }
    }
    dstBuf += dstStride;
    interPredBuf += interPredStride;
    intraPredBuf += intraPredStride;
  }
}
#endif
#if JVET_AC0112_IBC_CIIP
void IntraPrediction::geneWeightedPred( const ComponentID compId, PelBuf& pred, const PredictionUnit &pu, const PelBuf& interPred, const PelBuf& intraPred)
{
  const int            width = pred.width;
  const int            height = pred.height;
  const Pel*           interPredBuf = interPred.buf;
  const int            interPredStride = interPred.stride;
  Pel*                 intraPredBuf = intraPred.buf;
  const int            intraPredStride = intraPred.stride;
  const int            dstStride = pred.stride;
  Pel*                 dstBuf = pred.buf;
  int wMerge = 13;
  int wIntra = 3;
  int shift = 4;
  if (!pu.mergeFlag)
  {
    wMerge = 1;
    wIntra = 1;
    shift = 1;
  }
  if (width < 4)
  {
    ibcCiipBlending(dstBuf, dstStride, interPredBuf, interPredStride, intraPredBuf, intraPredStride, wMerge, wIntra, shift, width, height);
  }
  else
  {
    m_ibcCiipBlending(dstBuf, dstStride, interPredBuf, interPredStride, intraPredBuf, intraPredStride, wMerge, wIntra, shift, width, height);
  }
}
#endif

void IntraPrediction::geneIntrainterPred(const CodingUnit &cu, PelStorage& pred)
{
#if JVET_AC0112_IBC_CIIP
  if (!cu.firstPU->ciipFlag && !cu.firstPU->ibcCiipFlag)
#else
  if (!cu.firstPU->ciipFlag)
#endif
  {
    return;
  }

  const PredictionUnit* pu = cu.firstPU;

  initIntraPatternChType(cu, pu->Y());

  const UnitArea localUnitArea(pu->cs->area.chromaFormat, Area(0, 0, pu->Y().width, pu->Y().height));
  PelBuf ciipBuff = pred.getBuf(localUnitArea.Y());
  predIntraAng(COMPONENT_Y, ciipBuff, *pu);
#if JVET_AC0112_IBC_CIIP
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const bool chroma = !(CS::isDualITree(*pu->cs));
#else
  const bool chroma = !pu.cu->isSepTree();
#endif
  if (cu.firstPU->ibcCiipFlag && !chroma)
  {
    return;
  }
#endif

  if (isChromaEnabled(pu->chromaFormat))
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (pu->chromaSize().width > 2)
  {
#endif
    initIntraPatternChType(cu, pu->Cb());
    PelBuf ciipBuffCb = pred.getBuf(localUnitArea.Cb());
    predIntraAng(COMPONENT_Cb, ciipBuffCb, *pu);

    initIntraPatternChType(cu, pu->Cr());
    PelBuf ciipBuffCr = pred.getBuf(localUnitArea.Cr());
    predIntraAng(COMPONENT_Cr, ciipBuffCr, *pu);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  }
#endif
  }
}

#if !MERGE_ENC_OPT
void IntraPrediction::geneWeightedPred( const ComponentID compId, PelBuf &pred, const PredictionUnit &pu, Pel *srcBuf )
{
  const int            width = pred.width;
  CHECK( width == 2, "Width of 2 is not supported" );
  const int            height = pred.height;
  const int            srcStride = width;
  const int            dstStride = pred.stride;

  Pel*                 dstBuf = pred.buf;
  int wIntra, wMerge;

  const Position posBL = pu.Y().bottomLeft();
  const Position posTR = pu.Y().topRight();
  const PredictionUnit *neigh0 = pu.cs->getPURestricted( posBL.offset( -1, 0 ), pu, CHANNEL_TYPE_LUMA );
  const PredictionUnit *neigh1 = pu.cs->getPURestricted( posTR.offset( 0, -1 ), pu, CHANNEL_TYPE_LUMA );
  bool isNeigh0Intra = neigh0 && ( CU::isIntra( *neigh0->cu ) );
  bool isNeigh1Intra = neigh1 && ( CU::isIntra( *neigh1->cu ) );

  if( isNeigh0Intra && isNeigh1Intra )
  {
    wIntra = 3; wMerge = 1;
  }
  else
  {
    if( !isNeigh0Intra && !isNeigh1Intra )
    {
      wIntra = 1; wMerge = 3;
    }
    else
    {
      wIntra = 2; wMerge = 2;
    }
  }
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      dstBuf[y*dstStride + x] = ( wMerge * dstBuf[y*dstStride + x] + wIntra * srcBuf[y*srcStride + x] + 2 ) >> 2;
    }
  }
}

void IntraPrediction::switchBuffer( const PredictionUnit &pu, ComponentID compID, PelBuf srcBuff, Pel *dst )
{
  Pel  *src = srcBuff.bufAt( 0, 0 );
  int compWidth = compID == COMPONENT_Y ? pu.Y().width : pu.Cb().width;
  int compHeight = compID == COMPONENT_Y ? pu.Y().height : pu.Cb().height;
  for( int i = 0; i < compHeight; i++ )
  {
    memcpy( dst, src, compWidth * sizeof( Pel ) );
    src += srcBuff.stride;
    dst += compWidth;
  }
}
#endif

#if JVET_AD0120_LBCCP
inline bool isPosAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &refPos);
#endif
inline bool isAboveLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT );
inline int  isAboveAvailable      ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags );
inline int  isLeftAvailable       ( const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *validFlags );
inline int  isAboveRightAvailable ( const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );
inline int  isBelowLeftAvailable  ( const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *validFlags );

#if JVET_AB0155_SGPM
void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag,
                                             const int partIdx
#if JVET_AB0157_INTRA_FUSION
                                             , bool applyFusion
#endif
                                             )
#else
#if JVET_AB0157_INTRA_FUSION
void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag, bool applyFusion)
#else
void IntraPrediction::initIntraPatternChType(const CodingUnit &cu, const CompArea &area, const bool forceRefFilterFlag)
#endif
#endif
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK(area.width == 2, "Width of 2 is not supported");
#endif
  const CodingStructure& cs   = *cu.cs;

#if JVET_AB0157_INTRA_FUSION
  auto mrlIndex2D = cu.firstPU->multiRefIdx;
  m_ipaParam.fetchRef2nd   = area.compID == COMPONENT_Y;
  m_ipaParam.fetchRef2nd  &= area.width * area.height >= SIZE_CONSTRAINT;
  m_ipaParam.fetchRef2nd  &= !(cu.dimd && cu.dimdBlending);
  m_ipaParam.fetchRef2nd  &= !(mrlIndex2D == MULTI_REF_LINE_IDX[MRL_NUM_REF_LINES - 1] && ((cu.block(COMPONENT_Y).y) % ((cu.cs->sps)->getMaxCUWidth()) <= MULTI_REF_LINE_IDX[MRL_NUM_REF_LINES - 1]));
  m_ipaParam.fetchRef2nd  &= !(mrlIndex2D == 0 && ((cu.block(COMPONENT_Y).y) % ((cu.cs->sps)->getMaxCUWidth()) == 0));
#if JVET_AC0112_IBC_CIIP
  m_ipaParam.fetchRef2nd &= !cu.firstPU->ibcCiipFlag;
#endif
#if JVET_AC0112_IBC_GPM
  m_ipaParam.fetchRef2nd &= !cu.firstPU->ibcGpmFlag;
#endif
#endif

  if (!forceRefFilterFlag)
  {
#if JVET_AB0155_SGPM
    initPredIntraParams(*cu.firstPU, area, *cs.sps, partIdx);
#else
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
#endif
  }

  Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
  Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];

  setReferenceArrayLengths( area );

#if JVET_AB0155_SGPM
  if (!partIdx) 
  {
    // ----- Step 1: unfiltered reference samples -----
    xFillReferenceSamples(cs.picture->getRecoBuf(area), refBufUnfiltered, area, cu);
  }
#else
  // ----- Step 1: unfiltered reference samples -----
  xFillReferenceSamples( cs.picture->getRecoBuf( area ), refBufUnfiltered, area, cu );
#endif

#if JVET_AB0157_INTRA_FUSION
#if JVET_AB0155_SGPM
  if (m_ipaParam.fetchRef2nd && applyFusion && !partIdx)
#else
  if (m_ipaParam.fetchRef2nd && applyFusion)
#endif
  {
    cu.firstPU->multiRefIdx  = mrlIndex2D + 1;
    Pel *refBufUnfiltered2nd = m_refBuffer2nd[area.compID];
    auto backup              = m_refBufferStride[area.compID];
    xFillReferenceSamples(cs.picture->getRecoBuf(area), refBufUnfiltered2nd, area, cu);
    cu.firstPU->multiRefIdx  = mrlIndex2D;
    m_refBufferStride[area.compID] = backup;
  }
#endif
  // ----- Step 2: filtered reference samples -----
  if( m_ipaParam.refFilterFlag || forceRefFilterFlag )
  {
    xFilterReferenceSamples( refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx );
  }
}

void IntraPrediction::initIntraPatternChTypeISP(const CodingUnit& cu, const CompArea& area, PelBuf& recBuf, const bool forceRefFilterFlag)
{
  const CodingStructure& cs = *cu.cs;

  if (!forceRefFilterFlag)
  {
    initPredIntraParams(*cu.firstPU, area, *cs.sps);
  }

  const Position posLT = area;
  bool           isLeftAvail  = (cs.getCURestricted(posLT.offset(-1, 0), cu, CHANNEL_TYPE_LUMA) != NULL) && cs.isDecomp(posLT.offset(-1, 0), CHANNEL_TYPE_LUMA);
  bool           isAboveAvail = (cs.getCURestricted(posLT.offset(0, -1), cu, CHANNEL_TYPE_LUMA) != NULL) && cs.isDecomp(posLT.offset(0, -1), CHANNEL_TYPE_LUMA);
  // ----- Step 1: unfiltered reference samples -----
  if (cu.blocks[area.compID].x == area.x && cu.blocks[area.compID].y == area.y)
  {
    Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
    // With the first subpartition all the CU reference samples are fetched at once in a single call to xFillReferenceSamples
    if (cu.ispMode == HOR_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height << 1;
      m_topRefLength = cu.Y().width + area.width;
    }
    else //if (cu.ispMode == VER_INTRA_SUBPARTITIONS)
    {
      m_leftRefLength = cu.Y().height + area.height;
      m_topRefLength = cu.Y().width << 1;
    }


    xFillReferenceSamples(cs.picture->getRecoBuf(cu.Y()), refBufUnfiltered, cu.Y(), cu);

    // After having retrieved all the CU reference samples, the number of reference samples is now adjusted for the current subpartition
    m_topRefLength = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;
  }
  else
  {
    m_topRefLength = cu.blocks[area.compID].width + area.width;
    m_leftRefLength = cu.blocks[area.compID].height + area.height;

    const int predSizeHor = m_topRefLength;
    const int predSizeVer = m_leftRefLength;
    if (cu.ispMode == HOR_INTRA_SUBPARTITIONS)
    {
      Pel* src = recBuf.bufAt(0, -1);
      Pel *ref = m_refBuffer[area.compID][PRED_BUF_UNFILTERED] + m_refBufferStride[area.compID];
      if (isLeftAvail)
      {
        for (int i = 0; i <= 2 * cu.blocks[area.compID].height - area.height; i++)
        {
          ref[i] = ref[i + area.height];
        }
      }
      else
      {
        for (int i = 0; i <= predSizeVer; i++)
        {
          ref[i] = src[0];
        }
      }
      Pel *dst = m_refBuffer[area.compID][PRED_BUF_UNFILTERED] + 1;
      dst[-1]  = ref[0];
      for (int i = 0; i < area.width; i++)
      {
        dst[i] = src[i];
      }
      Pel sample = src[area.width - 1];
      dst += area.width;
      for (int i = 0; i < predSizeHor - area.width; i++)
      {
        dst[i] = sample;
      }
    }
    else
    {
      Pel* src = recBuf.bufAt(-1, 0);
      Pel *ref = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
      if (isAboveAvail)
      {
        for (int i = 0; i <= 2 * cu.blocks[area.compID].width - area.width; i++)
        {
          ref[i] = ref[i + area.width];
        }
      }
      else
      {
        for (int i = 0; i <= predSizeHor; i++)
        {
          ref[i] = src[0];
        }
      }
      Pel *dst = m_refBuffer[area.compID][PRED_BUF_UNFILTERED] + m_refBufferStride[area.compID] + 1;
      dst[-1]  = ref[0];
      for (int i = 0; i < area.height; i++)
      {
        *dst = *src;
        src += recBuf.stride;
        dst++;
      }
      Pel sample = src[-recBuf.stride];
      for (int i = 0; i < predSizeVer - area.height; i++)
      {
        *dst = sample;
        dst++;
      }
    }
  }
  // ----- Step 2: filtered reference samples -----
  if (m_ipaParam.refFilterFlag || forceRefFilterFlag)
  {
    Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
    Pel *refBufFiltered   = m_refBuffer[area.compID][PRED_BUF_FILTERED];
    xFilterReferenceSamples(refBufUnfiltered, refBufFiltered, area, *cs.sps, cu.firstPU->multiRefIdx);
  }
}

#if JVET_V0130_INTRA_TMP
#if JVET_W0069_TMP_BOUNDARY
RefTemplateType IntraPrediction::getRefTemplateType(CodingUnit& cu, CompArea& area)
#else
bool IntraPrediction::isRefTemplateAvailable(CodingUnit& cu, CompArea& area)
#endif
{
  const ChannelType      chType = toChannelType(area.compID);
  const CodingStructure& cs = *cu.cs;
  const SPS& sps = *cs.sps;
  const PreCalcValues& pcv = *cs.pcv;
  setReferenceArrayLengths( area );

  const int  tuWidth = area.width;
  const int  tuHeight = area.height;
  const int  predSize = m_topRefLength;
  const int  predHSize = m_leftRefLength;

  const int  unitWidth = pcv.minCUWidth >> getComponentScaleX(area.compID, sps.getChromaFormatIdc());
  const int  unitHeight = pcv.minCUHeight >> getComponentScaleY(area.compID, sps.getChromaFormatIdc());

  const int  totalAboveUnits = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits = std::max<int>(tuWidth / unitWidth, 1);
  const int  numLeftUnits = std::max<int>(tuHeight / unitHeight, 1);
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits = totalLeftUnits - numLeftUnits;

  if( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0 )
  {
#if JVET_W0069_TMP_BOUNDARY
    return NO_TEMPLATE;
#else
    return false;
#endif
  }

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];

  memset(neighborFlags, 0, totalUnits);


#if JVET_W0069_TMP_BOUNDARY
  if (isAboveLeftAvailable(cu, chType, posLT) && isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1)) && isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1)))
  {
    return L_SHAPE_TEMPLATE;
  }
  else if (isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1)))
  {
    return LEFT_TEMPLATE;
  }
  else if (isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1)))
  {
    return ABOVE_TEMPLATE;
  }
  else
  {
    return NO_TEMPLATE;
  }
  CHECK(1, "un defined template type");
#else
  return isAboveLeftAvailable(cu, chType, posLT) && isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1)) && isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1));
#endif

}
#endif
void IntraPrediction::xFillReferenceSamples( const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu )
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int multiRefIdx         = (area.compID == COMPONENT_Y) ? cu.firstPU->multiRefIdx : 0;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
  const int predStride = predSize + 1 + multiRefIdx;
  m_refBufferStride[area.compID] = predStride;

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth          = tuWidth  <= 2 && cu.ispMode && isLuma(area.compID) ? tuWidth  : pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int  unitHeight         = tuHeight <= 2 && cu.ispMode && isLuma(area.compID) ? tuHeight : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

 #if JVET_Y0116_EXTENDED_MRL_LIST
  int leftMrlUnitNum  = multiRefIdx / unitHeight;
  int aboveMrlUnitNum = multiRefIdx / unitWidth;
#endif

  const int  totalAboveUnits    = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (predHSize + (unitHeight - 1)) / unitHeight;
#if JVET_Y0116_EXTENDED_MRL_LIST
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1 + leftMrlUnitNum + aboveMrlUnitNum; //+1 for top-left
#else
  const int  totalUnits         = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
#endif
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  CHECK( numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported" );

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

#if JVET_Y0116_EXTENDED_MRL_LIST
#if JVET_AC0094_REF_SAMPLES_OPT
  bool neighborFlags[4 * (MAX_NUM_PART_IDXS_IN_CTU_WIDTH << 2) + MAX_REF_LINE_IDX + 1];
#else
  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + MAX_REF_LINE_IDX + 1];
#endif
#else
  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
#endif
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

#if JVET_Y0116_EXTENDED_MRL_LIST
  neighborFlags[totalLeftUnits+leftMrlUnitNum] = isAboveLeftAvailable( cu, chType, posLT.offset(-multiRefIdx, -multiRefIdx) );
  numIntraNeighbor += neighborFlags[totalLeftUnits+leftMrlUnitNum] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT.offset(-aboveMrlUnitNum*unitWidth, -multiRefIdx), aboveMrlUnitNum,      unitWidth,  (neighborFlags + totalLeftUnits + 1 + leftMrlUnitNum) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT.offset(-multiRefIdx, -leftMrlUnitNum*unitHeight), leftMrlUnitNum,       unitHeight, (neighborFlags + totalLeftUnits - 1 + leftMrlUnitNum) );
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT.offset(0, -multiRefIdx), numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1 + leftMrlUnitNum + aboveMrlUnitNum) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT.offset(0, -multiRefIdx), numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + leftMrlUnitNum + aboveMrlUnitNum + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT.offset(-multiRefIdx, 0), numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB.offset(-multiRefIdx, 0), numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );

#else
  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT );
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT, numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT, numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT, numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB, numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );
#endif

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);

  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize + multiRefIdx; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);

    std::memcpy( ptrDst, ptrSrc, sizeof( *ptrDst ) * ( predSize + multiRefIdx + 1 ) );

    for (int i = 0; i <= predHSize + multiRefIdx; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + multiRefIdx) * srcStride - (1 + multiRefIdx);
    ptrDst = refBufUnfiltered;
#if JVET_Y0116_EXTENDED_MRL_LIST
    if (neighborFlags[totalLeftUnits+leftMrlUnitNum])
#else
    if (neighborFlags[totalLeftUnits])
#endif
    {
      ptrDst[0] = ptrSrc[0];
      ptrDst[predStride] = ptrSrc[0];
#if JVET_Y0116_EXTENDED_MRL_LIST
      for (int i = 1; i <= multiRefIdx-leftMrlUnitNum*unitHeight; i++)
#else
      for (int i = 1; i <= multiRefIdx; i++)
#endif
      {
        ptrDst[i] = ptrSrc[i];
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
#if JVET_Y0116_EXTENDED_MRL_LIST
    ptrSrc += (1 + multiRefIdx-leftMrlUnitNum*unitHeight) * srcStride;
    ptrDst += (1 + multiRefIdx-leftMrlUnitNum*unitHeight) + predStride;
#else
    ptrSrc += (1 + multiRefIdx) * srcStride;
    ptrDst += (1 + multiRefIdx) + predStride;
#endif
#if JVET_Y0116_EXTENDED_MRL_LIST
    for (int unitIdx = totalLeftUnits + leftMrlUnitNum - 1; unitIdx > 0; unitIdx--)
#else
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
#endif
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = (predHSize % unitHeight == 0) ? unitHeight : predHSize % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
#if JVET_Y0116_EXTENDED_MRL_LIST
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx ) - aboveMrlUnitNum*unitWidth;
    ptrDst = refBufUnfiltered + 1 + multiRefIdx - aboveMrlUnitNum*unitWidth;
#else
    ptrSrc = srcBuf - srcStride * (1 + multiRefIdx);
    ptrDst = refBufUnfiltered + 1 + multiRefIdx;
#endif
#if JVET_Y0116_EXTENDED_MRL_LIST
    for (int unitIdx = totalLeftUnits + leftMrlUnitNum + 1; unitIdx < totalUnits - 1; unitIdx++)
#else
    for (int unitIdx = totalLeftUnits + 1; unitIdx < totalUnits - 1; unitIdx++)
#endif
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = (predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;

#if JVET_Y0116_EXTENDED_MRL_LIST
      if (firstAvailUnit < totalLeftUnits + leftMrlUnitNum)
      {
        firstAvailRow = (totalLeftUnits + leftMrlUnitNum - firstAvailUnit) * unitHeight + multiRefIdx - leftMrlUnitNum * unitHeight;
      }
      else if (firstAvailUnit == totalLeftUnits + leftMrlUnitNum)
      {
        firstAvailRow = multiRefIdx - leftMrlUnitNum * unitHeight;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - leftMrlUnitNum - 1) * unitWidth + 1 + multiRefIdx - aboveMrlUnitNum * unitWidth;
      }
#else
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + multiRefIdx;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = multiRefIdx;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - totalLeftUnits - 1) * unitWidth + 1 + multiRefIdx;
      }
#endif
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize + multiRefIdx;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit]) // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
#if JVET_Y0116_EXTENDED_MRL_LIST
        if (lastAvailUnit < totalLeftUnits + leftMrlUnitNum)
        {
          lastAvailRow = (totalLeftUnits + leftMrlUnitNum - lastAvailUnit - 1) * unitHeight + multiRefIdx - leftMrlUnitNum * unitHeight + 1;
        }
        else if (lastAvailUnit == totalLeftUnits + leftMrlUnitNum)
        {
          lastAvailCol = multiRefIdx- leftMrlUnitNum * unitHeight;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits - leftMrlUnitNum) * unitWidth + multiRefIdx - aboveMrlUnitNum * unitWidth;
        }
#else
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + multiRefIdx + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = multiRefIdx;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - totalLeftUnits) * unitWidth + multiRefIdx;
        }
#endif
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
#if JVET_Y0116_EXTENDED_MRL_LIST
        if (currUnit < totalLeftUnits + leftMrlUnitNum)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits + leftMrlUnitNum)
        {
          for (int i = 0; i < multiRefIdx - leftMrlUnitNum * unitHeight + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx - aboveMrlUnitNum * unitWidth + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
#else
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 0; i < multiRefIdx + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < multiRefIdx + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1) ? ((predSize % unitWidth == 0) ? unitWidth : predSize % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
#endif
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
}

void IntraPrediction::xFilterReferenceSamples(const Pel *refBufUnfiltered, Pel *refBufFiltered, const CompArea &area,
                                              const SPS &sps, int multiRefIdx)
{
  if (area.compID != COMPONENT_Y)
  {
    multiRefIdx = 0;
  }
  const int predSize = m_topRefLength + multiRefIdx;
  const int predHSize = m_leftRefLength + multiRefIdx;
  const size_t predStride = m_refBufferStride[area.compID];

  const Pel topLeft =
    (refBufUnfiltered[0] + refBufUnfiltered[1] + refBufUnfiltered[predStride] + refBufUnfiltered[predStride + 1] + 2)
    >> 2;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predSize] = refBufUnfiltered[predSize];

  refBufFiltered += predStride;
  refBufUnfiltered += predStride;

  refBufFiltered[0] = topLeft;

  for (int i = 1; i < predHSize; i++)
  {
    refBufFiltered[i] = (refBufUnfiltered[i - 1] + 2 * refBufUnfiltered[i] + refBufUnfiltered[i + 1] + 2) >> 2;
  }
  refBufFiltered[predHSize] = refBufUnfiltered[predHSize];
}

#if JVET_W0123_TIMD_FUSION
Pel IntraPrediction::xGetPredTimdValDc( const CPelBuf &pSrc, const Size &dstSize, TemplateType eTempType, int iTempHeight, int iTempWidth )
{
  int idx, sum = 0;
  Pel dcVal;
  const int width  = dstSize.width;
  const int height = dstSize.height;
  auto denom     = (width == height) ? (width << 1) : std::max(width,height);
  auto divShift  = floorLog2(denom);
  auto divOffset = (denom >> 1);

  if (eTempType == LEFT_NEIGHBOR)
  {
    denom = height;
    divShift = floorLog2(denom);
    divOffset = (denom >> 1);
    for(idx = 0; idx < height; idx++)
      sum += pSrc.at(1 + idx, 1);
    dcVal = (sum + divOffset) >> divShift;
    return dcVal;
  }
  else if (eTempType == ABOVE_NEIGHBOR)
  {
    denom = width;
    divShift = floorLog2(denom);
    divOffset = (denom >> 1);
    for(idx = 0; idx < width; idx++)
      sum += pSrc.at( 1 + idx, 0);
    dcVal = (sum + divOffset) >> divShift;
    return dcVal;
  }

  if ( width >= height )
  {
    for( idx = 0; idx < width; idx++ )
    {
      sum += pSrc.at(iTempWidth + 1 + idx, 0);
    }
  }
  if ( width <= height )
  {
    for( idx = 0; idx < height; idx++ )
    {
      sum += pSrc.at(iTempHeight + 1 + idx, 1);
    }
  }
  dcVal = (sum + divOffset) >> divShift;
  return dcVal;
}

void IntraPrediction::predTimdIntraAng( const ComponentID compId, const PredictionUnit &pu, uint32_t uiDirMode, Pel* pPred, uint32_t uiStride, uint32_t iWidth, uint32_t iHeight, TemplateType eTempType, int32_t iTemplateWidth, int32_t iTemplateHeight)
{
  const ComponentID compID       = MAP_CHROMA( compId );

  const int srcStride  = m_refBufferStride[compID];
  const int srcHStride = 2;

  const CPelBuf & srcBuf = CPelBuf(getPredictorPtr(compID), srcStride, srcHStride);
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compID));

  switch (uiDirMode)
  {
    case(PLANAR_IDX): xPredTimdIntraPlanar(srcBuf, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight); break;
    case(DC_IDX):     xPredTimdIntraDc(pu, srcBuf, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight); break;
#if JVET_AC0119_LM_CHROMA_FUSION
    default:          xPredTimdIntraAng(srcBuf, clpRng, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight, uiDirMode, toChannelType(compID)); break;
#else
    default:          xPredTimdIntraAng(srcBuf, clpRng, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight, uiDirMode); break;
#endif
  }

  if (m_ipaParam.applyPDPC && (uiDirMode == PLANAR_IDX || uiDirMode == DC_IDX))
  {
    xIntraPredTimdPlanarDcPdpc(srcBuf, pPred, uiStride, iWidth, iHeight, eTempType, iTemplateWidth, iTemplateHeight);
  }
}

void IntraPrediction::xPredTimdIntraPlanar( const CPelBuf &pSrc, Pel* rpDst, int iDstStride, int width, int height, TemplateType eTempType, int iTemplateWidth, int iTemplateHeight )
{
  static int leftColumn[MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE+1] = {0}, topRow[MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE+1] ={0}, bottomRow[MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE] = {0}, rightColumn[MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE]={0};
  if(eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    //predict above template
    {
      uint32_t w = width - iTemplateWidth;
      const uint32_t log2W = floorLog2( w );
      const uint32_t log2H = floorLog2( iTemplateHeight );
      const uint32_t offset = 1 << (log2W + log2H);
      for(int k = 0; k < w + 1; k++)
      {
        topRow[k] = pSrc.at( k + iTemplateWidth + 1, 0 );
      }
      for (int k=0; k < iTemplateHeight + 1; k++)
      {
        leftColumn[k] = pSrc.at( k + 1, 1 );
      }

      int bottomLeft = leftColumn[iTemplateHeight];
      int topRight = topRow[w];
      for(int k = 0; k < w; k++)
      {
        bottomRow[k]  = bottomLeft - topRow[k];
        topRow[k]     = topRow[k] << log2H;
      }
      for(int k = 0; k < iTemplateHeight; k++)
      {
        rightColumn[k]  = topRight - leftColumn[k];
        leftColumn[k]   = leftColumn[k] << log2W;
      }

      const uint32_t finalShift = 1 + log2W + log2H;
      for (int y = 0; y < iTemplateHeight; y++)
      {
        int horPred = leftColumn[y];
        for (int x = 0; x < w; x++)
        {
          horPred   += rightColumn[y];
          topRow[x] += bottomRow[x];
          int vertPred = topRow[x];
          rpDst[y*iDstStride+x + iTemplateWidth] = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
        }
      }
    }

    //predict left template
    {
      uint32_t h = height - iTemplateHeight;
      const uint32_t log2W = floorLog2( iTemplateWidth );
      const uint32_t log2H = floorLog2( h );
      const uint32_t offset = 1 << (log2W + log2H);
      for (int k = 0; k < h + 1; k++)
      {
        leftColumn[k] = pSrc.at( k + iTemplateHeight + 1, 1 );
      }
      for(int k = 0; k < iTemplateWidth + 1; k++)
      {
        topRow[k] = pSrc.at( k + 1, 0 );
      }
      int bottomLeft = leftColumn[h];
      int topRight = topRow[iTemplateWidth];
      for(int k = 0; k < iTemplateWidth; k++)
      {
        bottomRow[k]  = bottomLeft - topRow[k];
        topRow[k]     = topRow[k] << log2H;
      }
      for(int k = 0; k < h; k++)
      {
        rightColumn[k]  = topRight - leftColumn[k];
        leftColumn[k]   = leftColumn[k] << log2W;
      }
      const uint32_t finalShift = 1 + log2W + log2H;
      for (int y = 0; y < height; y++)
      {
        int horPred = leftColumn[y];
        for (int x = 0; x < iTemplateWidth; x++)
        {
          horPred   += rightColumn[y];
          topRow[x] += bottomRow[x];
          int vertPred = topRow[x];
          rpDst[(y + iTemplateHeight)*iDstStride+x] = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
        }
      }
    }
  }
  else if(eTempType == LEFT_NEIGHBOR)
  {
    const uint32_t log2W = floorLog2( iTemplateWidth );
    const uint32_t log2H = floorLog2( height );
    const uint32_t offset = 1 << (log2W + log2H);
    for (int k = 0; k < height + 1; k++)
    {
      leftColumn[k] = pSrc.at( k + iTemplateHeight + 1, 1 );
    }
    for(int k = 0; k < iTemplateWidth + 1; k++)
    {
      topRow[k] = pSrc.at( k + 1, 0 );
    }

    int bottomLeft = leftColumn[height];
    int topRight = topRow[iTemplateWidth];
    for(int k = 0; k < iTemplateWidth; k++)
    {
      bottomRow[k]  = bottomLeft - topRow[k];
      topRow[k]     = topRow[k] << log2H;
    }
    for(int k = 0; k < height; k++)
    {
      rightColumn[k]  = topRight - leftColumn[k];
      leftColumn[k]   = leftColumn[k] << log2W;
    }

    const uint32_t finalShift = 1 + log2W + log2H;
    for (int y = 0; y < height; y++)
    {
      int horPred = leftColumn[y];
      for (int x = 0; x < iTemplateWidth; x++)
      {
        horPred   += rightColumn[y];
        topRow[x] += bottomRow[x];
        int vertPred = topRow[x];
        rpDst[y*iDstStride+x] = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
      }
    }
  }
  else if(eTempType == ABOVE_NEIGHBOR)
  {
    const uint32_t log2W = floorLog2( width );
    const uint32_t log2H = floorLog2( iTemplateHeight );
    const uint32_t offset = 1 << (log2W + log2H);
    for(int k = 0; k < width + 1; k++)
    {
      topRow[k] = pSrc.at( k + iTemplateWidth + 1, 0 );
    }
    for (int k=0; k < iTemplateHeight + 1; k++)
    {
      leftColumn[k] = pSrc.at( k + 1, 1 );
    }

    int bottomLeft = leftColumn[iTemplateHeight];
    int topRight = topRow[width];
    for(int k=0;k<width;k++)
    {
      bottomRow[k]  = bottomLeft - topRow[k];
      topRow[k]     = topRow[k] << log2H;
    }
    for(int k = 0; k < iTemplateHeight; k++)
    {
      rightColumn[k]  = topRight - leftColumn[k];
      leftColumn[k]   = leftColumn[k] << log2W;
    }

    const uint32_t finalShift = 1 + log2W + log2H;
    for (int y = 0; y < iTemplateHeight; y++)
    {
      int horPred = leftColumn[y];
      for (int x = 0; x < width; x++)
      {
        horPred   += rightColumn[y];
        topRow[x] += bottomRow[x];
        int vertPred = topRow[x];
        rpDst[y*iDstStride+x] = ( ( horPred << log2H ) + ( vertPred << log2W ) + offset ) >> finalShift;
      }
    }
  }
  else
  {
    assert(0);
  }
}

void IntraPrediction::xPredTimdIntraDc( const PredictionUnit &pu, const CPelBuf &pSrc, Pel* pDst, int iDstStride, int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth, int iTemplateHeight )
{
#if JVET_AC0119_LM_CHROMA_FUSION
  const Size& dstSize = Size(iWidth - iTemplateWidth, iHeight - iTemplateHeight);
#else
  const Size &dstSize = Size(pu.lwidth(), pu.lheight());
#endif
  const Pel dcval = xGetPredTimdValDc( pSrc, dstSize, eTempType, iTemplateHeight, iTemplateWidth );
  if(eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    for (int y = 0; y < iHeight; y++,pDst += iDstStride)
    {
      if(y < iTemplateHeight)
      {
        for (int x = iTemplateWidth; x < iWidth; x++)
        {
          pDst[x] = dcval;
        }
      }
      else
      {
        for (int x = 0; x < iTemplateWidth; x++)
        {
          pDst[x] = dcval;
        }
      }
    }
  }
  else if(eTempType == LEFT_NEIGHBOR)
  {
    for (int y = 0; y < iHeight; y++, pDst += iDstStride)
    {
      for (int x = 0; x < iTemplateWidth; x++)
      {
        pDst[x] = dcval;
      }
    }
  }
  else if(eTempType == ABOVE_NEIGHBOR)
  {
    for (int y = 0; y < iTemplateHeight; y++, pDst+=iDstStride)
    {
      for (int x = 0; x < iWidth; x++)
      {
        pDst[x] = dcval;
      }
    }
  }
  else
  {
    assert(0);
  }
}

#if JVET_AB0155_SGPM
void IntraPrediction::initPredTimdIntraParams(const PredictionUnit &pu, const CompArea area, int dirMode, bool bSgpm
#if JVET_AC0094_REF_SAMPLES_OPT
                                              , bool checkWideAngle
#endif
)
#else
void IntraPrediction::initPredTimdIntraParams(const PredictionUnit & pu, const CompArea area, int dirMode)
#endif
{
  const Size   puSize    = Size( area.width, area.height );
  const Size&  blockSize = puSize;
#if JVET_AB0155_SGPM
#if JVET_AC0094_REF_SAMPLES_OPT
  const int predMode = checkWideAngle ? getWideAngleExt(blockSize.width, blockSize.height, dirMode, bSgpm)
                                      : getTimdWideAngleExt(blockSize.width, blockSize.height, dirMode);
#else
  const int predMode = getWideAngleExt(blockSize.width, blockSize.height, dirMode, bSgpm);
#endif
#else
  const int     predMode = getWideAngleExt( blockSize.width, blockSize.height, dirMode );
#endif
  m_ipaParam.isModeVer            = predMode >= EXT_DIA_IDX;
  m_ipaParam.refFilterFlag        = false;
  m_ipaParam.interpolationFlag    = false;
  m_ipaParam.applyPDPC            = puSize.width >= MIN_TB_SIZEY && puSize.height >= MIN_TB_SIZEY;
  const int    intraPredAngleMode = (m_ipaParam.isModeVer) ? predMode - EXT_VER_IDX : -(predMode - EXT_HOR_IDX);

  int absAng = 0;
  static const int extAngTable[64]    = { 0, 1, 2, 3, 4, 5, 6,7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 74, 78, 84, 90, 96, 102, 108, 114, 121, 128, 137, 146, 159, 172, 188, 204, 230, 256, 299, 342, 427, 512, 597, 682, 853, 1024, 1536, 2048, 3072 };
  static const int extInvAngTable[64] = { 0, 32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3277, 2731, 2341, 2048, 1820, 1638, 1489, 1365, 1260, 1170, 1092, 1024, 964, 910, 862, 819, 762, 712, 669, 630, 596, 565, 537, 512, 489, 468, 443, 420, 390, 364, 341, 321, 303, 287, 271, 256, 239, 224, 206, 191, 174, 161, 142, 128, 110, 96, 77, 64, 55, 48, 38, 32, 21, 16, 11 }; // (512 * 64) / Angle

  const int     absAngMode         = abs(intraPredAngleMode);
  const int     signAng            = intraPredAngleMode < 0 ? -1 : 1;
                absAng             = extAngTable  [absAngMode];

  m_ipaParam.absInvAngle              = extInvAngTable[absAngMode];
  m_ipaParam.intraPredAngle        = signAng * absAng;

  if (dirMode > 1)
  {
    if (intraPredAngleMode < 0)
    {
      m_ipaParam.applyPDPC = false;
    }
    else if (intraPredAngleMode > 0)
    {
      const int sideSize = m_ipaParam.isModeVer ? puSize.height : puSize.width;
      const int maxScale = 2;
#if GRAD_PDPC
      m_ipaParam.useGradPDPC = false;
#endif
      m_ipaParam.angularScale = std::min(maxScale, floorLog2(sideSize) - (floorLog2(3 * m_ipaParam.absInvAngle - 2) - 8));
#if GRAD_PDPC
      if (m_ipaParam.angularScale < 0)
      {
        m_ipaParam.angularScale = (floorLog2(puSize.width) + floorLog2(puSize.height) - 2) >> 2;
        m_ipaParam.useGradPDPC = true;
      }
#endif
      m_ipaParam.applyPDPC &= m_ipaParam.angularScale >= 0;
    }
  }
}

void IntraPrediction::xPredTimdIntraAng( const CPelBuf &pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride, int iWidth, int iHeight, TemplateType eTempType, int iTemplateWidth , int iTemplateHeight, uint32_t dirMode
#if JVET_AC0119_LM_CHROMA_FUSION
  , const ChannelType channelType
#endif
)
{
  int width = iWidth;
  int height = iHeight;
  const bool bIsModeVer     = m_ipaParam.isModeVer;
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  invAngle       = m_ipaParam.absInvAngle;
  Pel* refMain;
  Pel* refSide;
#if JVET_AC0094_REF_SAMPLES_OPT
  Pel * refAbove = tempRefAbove;
  Pel * refLeft  = tempRefLeft;
#else
  static Pel  refAbove[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];
  static Pel  refLeft[2 * MAX_CU_SIZE + 5 + 33 * MAX_REF_LINE_IDX];
#endif

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for (int x = 0; x <= width + 1; x++)
    {
      refAbove[x + height] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= height + 1; y++)
    {
      refLeft[y + width] = pSrc.at(y, 1);
    }
    refMain = bIsModeVer ? refAbove + height : refLeft + width;
    refSide = bIsModeVer ? refLeft + width : refAbove + height;
    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? height : width;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * invAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength; y++)
    {
      refLeft[y] = pSrc.at(y, 1);
    }
    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;
    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(width - iTemplateWidth) - floorLog2(height - iTemplateHeight);
    const int s         = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex  = (std::max(iTemplateWidth, iTemplateHeight) << s) + 2 + std::max(iTemplateWidth, iTemplateHeight);
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val       = refMain[refLength];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + z] = val;
    }
  }

  // swap width/height if we are doing a horizontal mode:
  static Pel tempArray[(MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE)*(MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE)];  ///< buffer size may not be big enough
  const int dstStride = bIsModeVer ? iDstStride : (MAX_CU_SIZE+DIMD_MAX_TEMP_SIZE);
  Pel *pDst = bIsModeVer ? pTrueDst : tempArray;
  if (!bIsModeVer)
  {
    std::swap(width, height);
    std::swap(iTemplateWidth, iTemplateHeight);
  }

  if( intraPredAngle == 0 )  // pure vertical or pure horizontal
  {
    if(eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      if (m_ipaParam.applyPDPC)
      {
        int scale = (floorLog2(width) + floorLog2(height) - 2) >> 2;
        xIntraPredTimdHorVerPdpc(pDst, dstStride, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, refMain, clpRng);
        xIntraPredTimdHorVerPdpc(pDst+iTemplateHeight*dstStride, dstStride, refSide, iTemplateWidth, height, 0, iTemplateHeight, scale, refMain, clpRng);
      }
      else
      {
        for (int y = 0; y < iTemplateHeight; y++)
        {
          memcpy(pDst + y * dstStride + iTemplateWidth, &refMain[iTemplateWidth + 1], (width - iTemplateWidth) * sizeof(Pel));
        }
        for (int y = iTemplateHeight; y < height; y++)
        {
          memcpy(pDst + y * dstStride, &refMain[1], iTemplateWidth * sizeof(Pel));
        }
      }
    }
    else if(eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR)
    {
      if((eTempType == LEFT_NEIGHBOR && bIsModeVer)||(eTempType == ABOVE_NEIGHBOR && !bIsModeVer))
      {
        if (m_ipaParam.applyPDPC)
        {
          const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
          xIntraPredTimdHorVerPdpc(pDst, dstStride, refSide, iTemplateWidth, height, 0, 0, scale, refMain, clpRng);
        }
        else
        {
          for (int y = 0; y < height; y++)
          {
            for (int x = 0; x < iTemplateWidth; x++)
            {
              pDst[y * dstStride+x] = refMain[x + 1];
            }
          }
        }
      }
      else
      {
        if (m_ipaParam.applyPDPC)
        {
          const int scale   = (floorLog2(width) + floorLog2(height) - 2) >> 2;
          xIntraPredTimdHorVerPdpc(pDst, dstStride, refSide, width, iTemplateHeight, 0, 0, scale, refMain, clpRng);
        }
        else
        {
          for (int y = 0; y < iTemplateHeight; y++)
          {
            memcpy(pDst + y * dstStride, &refMain[1], width * sizeof(Pel));
          }
        }
      }
    }
    else
    {
      assert(0);
    }
  }
  else
  {
    Pel *pDsty=pDst;
#if JVET_AC0119_LM_CHROMA_FUSION
    bool bExtIntraDir = isLuma(channelType);
    bool bIntSlope = isLuma(channelType) ? isIntegerSlopeExt(abs(intraPredAngle)) : isIntegerSlope(abs(intraPredAngle));
    if (!bIntSlope)
#else
    if (!isIntegerSlopeExt(abs(intraPredAngle)))
#endif
    {
      int deltaPos = intraPredAngle;
      if (eTempType == LEFT_ABOVE_NEIGHBOR)
      {
        Pel *pDsty=pDst;
        // Above template
#if JVET_AC0119_LM_CHROMA_FUSION
        if (isLuma(channelType))
        {
#endif
        xIntraPredTimdAngLuma(pDsty, dstStride, refMain, width, iTemplateHeight, deltaPos, intraPredAngle, clpRng, iTemplateWidth, 0);
        // Left template
        for (int y = 0; y < iTemplateHeight; y++)
        {
          deltaPos += intraPredAngle;
        }
        xIntraPredTimdAngLuma(pDsty, dstStride, refMain, iTemplateWidth, height, deltaPos, intraPredAngle, clpRng, 0, iTemplateHeight);
#if JVET_AC0119_LM_CHROMA_FUSION
        }
        else
        {
          xIntraPredTimdAngChroma(pDsty, dstStride, refMain, width, iTemplateHeight, deltaPos, intraPredAngle, clpRng, iTemplateWidth, 0);
          // Left template
          for (int y = 0; y < iTemplateHeight; y++)
          {
            deltaPos += intraPredAngle;
          }
          xIntraPredTimdAngChroma(pDsty, dstStride, refMain, iTemplateWidth, height, deltaPos, intraPredAngle, clpRng, 0, iTemplateHeight);
        }
#endif
#if GRAD_PDPC
        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int deltaPos2 = intraPredAngle;
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, deltaPos2, intraPredAngle, clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
            , bExtIntraDir
#endif
          );
          for (int y = 0; y < iTemplateHeight; y++)
            deltaPos2 += intraPredAngle;
          xIntraPredTimdAngGradPdpc(pDst+iTemplateHeight*dstStride, dstStride, refMain, refSide, iTemplateWidth, height, 0, iTemplateHeight, scale, deltaPos2, intraPredAngle, clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
            , bExtIntraDir
#endif
          );
        }
        else
#endif
        if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, invAngle);
          xIntraPredTimdAngPdpc(pDst+iTemplateHeight*dstStride, dstStride, refSide, iTemplateWidth, height, 0, iTemplateHeight, scale, invAngle);
        }
      }
      else if (eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR)
      {
        int iRegionWidth, iRegionHeight;
        if((eTempType == LEFT_NEIGHBOR && bIsModeVer)||(eTempType == ABOVE_NEIGHBOR && !bIsModeVer))
        {
          iRegionWidth  = iTemplateWidth;
          iRegionHeight = height;
        }
        else
        {
          iRegionWidth  = width;
          iRegionHeight = iTemplateHeight;
        }
#if JVET_AC0119_LM_CHROMA_FUSION
        if (isLuma(channelType))
        {
#endif
        xIntraPredTimdAngLuma(pDsty, dstStride, refMain, iRegionWidth, iRegionHeight, deltaPos, intraPredAngle, clpRng, 0, 0);
#if JVET_AC0119_LM_CHROMA_FUSION
        }
        else
        {
          xIntraPredTimdAngChroma(pDsty, dstStride, refMain, iRegionWidth, iRegionHeight, deltaPos, intraPredAngle, clpRng, 0, 0);
        }
#endif
#if GRAD_PDPC
        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int deltaPos2 = intraPredAngle;
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, iRegionWidth, iRegionHeight, 0, 0, scale, deltaPos2, intraPredAngle, clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
            , bExtIntraDir
#endif
          );
        }
        else
#endif
        if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, iRegionWidth, iRegionHeight, 0, 0, scale, invAngle);
        }
      }
    }
    else
    {
      if(eTempType == LEFT_ABOVE_NEIGHBOR)
      {
        Pel *pDsty=pDst;
        for (int y = 0, deltaPos = intraPredAngle; y<height; y++, deltaPos += intraPredAngle, pDsty += dstStride)
        {
#if JVET_AC0119_LM_CHROMA_FUSION
          const int deltaInt = isLuma(channelType) ? deltaPos >> 6 : deltaPos >> 5;
#else
          const int deltaInt = deltaPos >> 6;
#endif
          int iStartIdx, iEndIdx;
          if(y < iTemplateHeight)
          {
            iStartIdx = iTemplateWidth;
            iEndIdx   = width - 1;
          }
          else
          {
            iStartIdx = 0;
            iEndIdx   = iTemplateWidth - 1;
          }
          memcpy(pDsty + iStartIdx, &refMain[iStartIdx + deltaInt + 1], (iEndIdx - iStartIdx + 1) * sizeof(Pel));
        }
#if GRAD_PDPC
        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int deltaPos2 = intraPredAngle;
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, deltaPos2, intraPredAngle, clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
            , bExtIntraDir
#endif
          );
          for (int y = 0; y < iTemplateHeight; y++)
            deltaPos2 += intraPredAngle;
          xIntraPredTimdAngGradPdpc(pDst+iTemplateHeight*dstStride, dstStride, refMain, refSide, iTemplateWidth, height, 0, iTemplateHeight, scale, deltaPos2, intraPredAngle, clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
            , bExtIntraDir
#endif
          );
        }
        else
#endif
        if (m_ipaParam.applyPDPC)
        {
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, width, iTemplateHeight, iTemplateWidth, 0, scale, invAngle);
          xIntraPredTimdAngPdpc(pDst+iTemplateHeight*dstStride, dstStride, refSide, iTemplateWidth, height, 0, iTemplateHeight, scale, invAngle);
        }
      }
      else // if (eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR)
      {
        Pel *pDsty=pDst;
        assert(eTempType == LEFT_NEIGHBOR || eTempType == ABOVE_NEIGHBOR);
        int iRegionWidth, iRegionHeight;
        if((eTempType == LEFT_NEIGHBOR && bIsModeVer)||(eTempType == ABOVE_NEIGHBOR && !bIsModeVer))
        {
          iRegionWidth  = iTemplateWidth;
          iRegionHeight = height;
        }
        else
        {
          iRegionWidth  = width;
          iRegionHeight = iTemplateHeight;
        }
        for (int y = 0, deltaPos = intraPredAngle; y<iRegionHeight; y++, deltaPos += intraPredAngle, pDsty += dstStride)
        {
#if JVET_AC0119_LM_CHROMA_FUSION
          const int deltaInt = isLuma(channelType) ? deltaPos >> 6 : deltaPos >> 5;
#else
          const int deltaInt = deltaPos >> 6;
#endif
          memcpy(pDsty, &refMain[deltaInt + 1], iRegionWidth * sizeof(Pel));
        }
#if GRAD_PDPC
        if (m_ipaParam.applyPDPC && m_ipaParam.useGradPDPC)
        {
          int deltaPos2 = intraPredAngle;
          const int scale = m_ipaParam.angularScale;
          xIntraPredTimdAngGradPdpc(pDst, dstStride, refMain, refSide, iRegionWidth, iRegionHeight, 0, 0, scale, deltaPos2, intraPredAngle, clpRng
#if JVET_AC0119_LM_CHROMA_FUSION
            , bExtIntraDir
#endif
          );
        }
        else
#endif
        if (m_ipaParam.applyPDPC)
        {
          const int scale   = m_ipaParam.angularScale;
          xIntraPredTimdAngPdpc(pDst, dstStride, refSide, iRegionWidth, iRegionHeight, 0, 0, scale, invAngle);
        }
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    if(eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      for (int y = 0; y < height; y++)
      {
        int iStartIdx, iEndIdx;
        if(y < iTemplateHeight)
        {
          iStartIdx = iTemplateWidth;
          iEndIdx   = width - 1;
        }
        else
        {
          iStartIdx = 0;
          iEndIdx   = iTemplateWidth - 1;
        }
        for (int x = iStartIdx; x <= iEndIdx; x++)
        {
          pTrueDst[x*iDstStride+y] = pDst[y*dstStride+x];
        }
      }
    }
    else if(eTempType == LEFT_NEIGHBOR)
    {
      for (int y = 0; y < iTemplateHeight; y++)
      {
        for (int x = 0; x < width; x++)
        {
          pTrueDst[x*iDstStride+y] = pDst[y*dstStride+x];
        }
      }
    }
    else if(eTempType == ABOVE_NEIGHBOR)
    {
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < iTemplateWidth; x++)
        {
          pTrueDst[x*iDstStride+y] = pDst[y*dstStride+x];
        }
      }
    }
    else
    {
      assert(0);
    }
  }
}

void IntraPrediction::initTimdIntraPatternLuma(const CodingUnit &cu, const CompArea &area, int iTemplateWidth, int iTemplateHeight, uint32_t uiRefWidth, uint32_t uiRefHeight)
{
  const CodingStructure& cs   = *cu.cs;
  Pel *refBufUnfiltered = m_refBuffer[area.compID][PRED_BUF_UNFILTERED];
#if JVET_AC0094_REF_SAMPLES_OPT
  bool bLeftAbove = iTemplateHeight > 0 && iTemplateWidth > 0;
  m_leftRefLength = bLeftAbove ? (uiRefHeight << 3) : ((uiRefHeight + iTemplateHeight) << 3);
  m_topRefLength  = bLeftAbove ? (uiRefWidth << 3) : ((uiRefWidth + iTemplateWidth) << 3);
#else
  bool bLeftAbove = iTemplateHeight > 0 && iTemplateWidth > 0;
  m_leftRefLength     = bLeftAbove ? (uiRefHeight << 1) : ((uiRefHeight + iTemplateHeight) << 1);
  m_topRefLength      = bLeftAbove ? (uiRefWidth << 1) : ((uiRefWidth + iTemplateWidth) << 1);
#endif
  xFillTimdReferenceSamples(cs.picture->getRecoBuf(area), refBufUnfiltered, area, cu, iTemplateWidth, iTemplateHeight);
}

void IntraPrediction::xFillTimdReferenceSamples(const CPelBuf &recoBuf, Pel* refBufUnfiltered, const CompArea &area, const CodingUnit &cu, int iTemplateWidth, int iTemplateHeight)
{
  const ChannelType      chType = toChannelType( area.compID );
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int  tuWidth            = area.width;
  const int  tuHeight           = area.height;
  const int  predSize           = m_topRefLength;
  const int  predHSize          = m_leftRefLength;
  const int predStride = predSize + 1;
  m_refBufferStride[area.compID] = predStride;

  const bool noShift            = pcv.noChroma2x2 && area.width == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth          = tuWidth  <= 2 && cu.ispMode && isLuma(area.compID) ? tuWidth  : pcv.minCUWidth  >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int  unitHeight         = tuHeight <= 2 && cu.ispMode && isLuma(area.compID) ? tuHeight : pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));
  int leftTempUnitNum = 0;
  int aboveTempUnitNum = 0;
  if (iTemplateHeight >= 4)
  {
    leftTempUnitNum = iTemplateHeight / unitHeight;
  }
  if (iTemplateWidth >= 4)
  {
    aboveTempUnitNum = iTemplateWidth / unitWidth;
  }

  const int  totalAboveUnits = (predSize + (unitWidth - 1)) / unitWidth - aboveTempUnitNum;
  const int  totalLeftUnits = (predHSize + (unitHeight - 1)) / unitHeight - leftTempUnitNum;
  const int  totalUnits = totalAboveUnits + totalLeftUnits + 1 + aboveTempUnitNum + leftTempUnitNum; //+1 for top-left
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  // ----- Step 1: analyze neighborhood -----
  const Position posLT          = area;
  const Position posRT          = area.topRight();
  const Position posLB          = area.bottomLeft();

#if JVET_AC0094_REF_SAMPLES_OPT
  bool neighborFlags[4 * (MAX_NUM_PART_IDXS_IN_CTU_WIDTH << 2) + 1];
#else
  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
#endif
  int   numIntraNeighbor = 0;

  memset( neighborFlags, 0, totalUnits );

  neighborFlags[totalLeftUnits] = isAboveLeftAvailable( cu, chType, posLT.offset(-iTemplateWidth, -iTemplateHeight) );
  neighborFlags[totalLeftUnits + leftTempUnitNum] = neighborFlags[totalLeftUnits];
  neighborFlags[totalLeftUnits + leftTempUnitNum + aboveTempUnitNum] = neighborFlags[totalLeftUnits];
  numIntraNeighbor += neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += leftTempUnitNum > 0 && neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += aboveTempUnitNum > 0 && neighborFlags[totalLeftUnits] ? 1 : 0;
  numIntraNeighbor += isAboveAvailable     ( cu, chType, posLT.offset(0, -iTemplateHeight), numAboveUnits,      unitWidth,  (neighborFlags + totalLeftUnits + 1 + leftTempUnitNum + aboveTempUnitNum) );
  numIntraNeighbor += isAboveRightAvailable( cu, chType, posRT.offset(0, -iTemplateHeight), numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + leftTempUnitNum + aboveTempUnitNum + numAboveUnits) );
  numIntraNeighbor += isLeftAvailable      ( cu, chType, posLT.offset(-iTemplateWidth, 0), numLeftUnits,       unitHeight, (neighborFlags + totalLeftUnits - 1) );
  numIntraNeighbor += isBelowLeftAvailable ( cu, chType, posLB.offset(-iTemplateWidth, 0), numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );

  // ----- Step 2: fill reference samples (depending on neighborhood) -----

  const Pel*  srcBuf    = recoBuf.buf;
  const int   srcStride = recoBuf.stride;
        Pel*  ptrDst    = refBufUnfiltered;
  const Pel*  ptrSrc;
  const Pel   valueDC   = 1 << (sps.getBitDepth( chType ) - 1);


  if( numIntraNeighbor == 0 )
  {
    // Fill border with DC value
    for (int j = 0; j <= predSize; j++)
    {
      ptrDst[j] = valueDC;
    }
    for (int i = 0; i <= predHSize; i++)
    {
      ptrDst[i + predStride] = valueDC;
    }
  }
  else if( numIntraNeighbor == totalUnits )
  {
    // Fill top-left border and top and top right with rec. samples
    ptrSrc = srcBuf - (1 + iTemplateHeight) * srcStride - (1 + iTemplateWidth);
    for (int j = 0; j <= predSize; j++)
    {
      ptrDst[j] = ptrSrc[j];
    }
    for (int i = 0; i <= predHSize; i++)
    {
      ptrDst[i + predStride] = ptrSrc[i * srcStride];
    }
  }
  else // reference samples are partially available
  {
    // Fill top-left sample(s) if available
    ptrSrc = srcBuf - (1 + iTemplateHeight) * srcStride - (1 + iTemplateWidth);
    ptrDst = refBufUnfiltered;
    if (neighborFlags[totalLeftUnits])
    {
      for( int i = 0; i <= iTemplateWidth; i++ )
      {
        ptrDst[i] = ptrSrc[i];
      }
      for( int i = 0; i <= iTemplateHeight; i++ )
      {
        ptrDst[i + predStride] = ptrSrc[i * srcStride];
      }
    }

    // Fill left & below-left samples if available (downwards)
    ptrSrc += (1 + iTemplateHeight) * srcStride;
    ptrDst += (1 + iTemplateHeight) + predStride;
    for (int unitIdx = totalLeftUnits - 1; unitIdx > 0; unitIdx--)
    {
      if (neighborFlags[unitIdx])
      {
        for (int i = 0; i < unitHeight; i++)
        {
          ptrDst[i] = ptrSrc[i * srcStride];
        }
      }
      ptrSrc += unitHeight * srcStride;
      ptrDst += unitHeight;
    }
    // Fill last below-left sample(s)
    if (neighborFlags[0])
    {
      int lastSample = ((predHSize - iTemplateHeight) % unitHeight == 0) ? unitHeight : (predHSize - iTemplateHeight) % unitHeight;
      for (int i = 0; i < lastSample; i++)
      {
        ptrDst[i] = ptrSrc[i * srcStride];
      }
    }

    // Fill above & above-right samples if available (left-to-right)
    ptrSrc = srcBuf - srcStride * (1 + iTemplateHeight);
    ptrDst = refBufUnfiltered + 1 + iTemplateWidth;
    for (int unitIdx = totalLeftUnits + 1 + leftTempUnitNum + aboveTempUnitNum; unitIdx < totalUnits - 1; unitIdx++)
    {
      if (neighborFlags[unitIdx])
      {
        for (int j = 0; j < unitWidth; j++)
        {
          ptrDst[j] = ptrSrc[j];
        }
      }
      ptrSrc += unitWidth;
      ptrDst += unitWidth;
    }
    // Fill last above-right sample(s)
    if (neighborFlags[totalUnits - 1])
    {
      int lastSample = ((predSize - iTemplateWidth) % unitWidth == 0) ? unitWidth : (predSize - iTemplateWidth) % unitWidth;
      for (int j = 0; j < lastSample; j++)
      {
        ptrDst[j] = ptrSrc[j];
      }
    }

    // pad from first available down to the last below-left
    ptrDst = refBufUnfiltered;
    int lastAvailUnit = 0;
    if (!neighborFlags[0])
    {
      int firstAvailUnit = 1;
      while (firstAvailUnit < totalUnits && !neighborFlags[firstAvailUnit])
      {
        firstAvailUnit++;
      }

      // first available sample
      int firstAvailRow = -1;
      int firstAvailCol = 0;
      if (firstAvailUnit < totalLeftUnits)
      {
        firstAvailRow = (totalLeftUnits - firstAvailUnit) * unitHeight + iTemplateHeight;
      }
      else if (firstAvailUnit == totalLeftUnits)
      {
        firstAvailRow = iTemplateHeight;
      }
      else
      {
        firstAvailCol = (firstAvailUnit - (totalLeftUnits + leftTempUnitNum + aboveTempUnitNum) - 1) * unitWidth + 1 + iTemplateWidth;
      }
      const Pel firstAvailSample = ptrDst[firstAvailRow < 0 ? firstAvailCol : firstAvailRow + predStride];

      // last sample below-left (n.a.)
      int lastRow = predHSize;

      // fill left column
      for (int i = lastRow; i > firstAvailRow; i--)
      {
        ptrDst[i + predStride] = firstAvailSample;
      }
      // fill top row
      if (firstAvailCol > 0)
      {
        for (int j = 0; j < firstAvailCol; j++)
        {
          ptrDst[j] = firstAvailSample;
        }
      }
      lastAvailUnit = firstAvailUnit;
    }

    // pad all other reference samples.
    int currUnit = lastAvailUnit + 1;
    while (currUnit < totalUnits)
    {
      if (!neighborFlags[currUnit]) // samples not available
      {
        // last available sample
        int lastAvailRow = -1;
        int lastAvailCol = 0;
        if (lastAvailUnit < totalLeftUnits)
        {
          lastAvailRow = (totalLeftUnits - lastAvailUnit - 1) * unitHeight + iTemplateHeight + 1;
        }
        else if (lastAvailUnit == totalLeftUnits)
        {
          lastAvailCol = iTemplateWidth;
        }
        else
        {
          lastAvailCol = (lastAvailUnit - (totalLeftUnits + leftTempUnitNum + aboveTempUnitNum)) * unitWidth + iTemplateWidth;
        }
        const Pel lastAvailSample = ptrDst[lastAvailRow < 0 ? lastAvailCol : lastAvailRow + predStride];

        // fill current unit with last available sample
        if (currUnit < totalLeftUnits)
        {
          for (int i = lastAvailRow - 1; i >= lastAvailRow - unitHeight; i--)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
        }
        else if (currUnit == totalLeftUnits)
        {
          for (int i = 0; i < iTemplateHeight + 1; i++)
          {
            ptrDst[i + predStride] = lastAvailSample;
          }
          for (int j = 0; j < iTemplateWidth + 1; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
        else
        {
          int numSamplesInUnit = (currUnit == totalUnits - 1) ? (((predSize - iTemplateWidth) % unitWidth == 0) ? unitWidth : (predSize - iTemplateWidth) % unitWidth) : unitWidth;
          for (int j = lastAvailCol + 1; j <= lastAvailCol + numSamplesInUnit; j++)
          {
            ptrDst[j] = lastAvailSample;
          }
        }
      }
      lastAvailUnit = currUnit;
      currUnit++;
    }
  }
}

#if JVET_AB0155_SGPM
void IntraPrediction::deriveSgpmModeOrdered(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu,
                                            static_vector<SgpmInfo, SGPM_NUM> &candModeList,
                                            static_vector<double, SGPM_NUM> &  candCostList)
{
  SizeType uiWidth         = cu.lwidth();
  SizeType uiHeight        = cu.lheight();

  int      iCurX = cu.lx();
  int      iCurY = cu.ly();
  int      iRefX = -1, iRefY = -1;
  uint32_t uiRefWidth = 0, uiRefHeight = 0;

  const int iTempWidth = SGPM_TEMPLATE_SIZE, iTempHeight = SGPM_TEMPLATE_SIZE;

  TemplateType eTempType = CU::deriveTimdRefType(iCurX, iCurY, uiWidth, uiHeight, iTempWidth, iTempHeight, iRefX,
                                                  iRefY, uiRefWidth, uiRefHeight);
  auto &        pu        = *cu.firstPU;
  uint32_t      uiRealW   = uiRefWidth + (eTempType == LEFT_NEIGHBOR ? iTempWidth : 0);
  uint32_t      uiRealH   = uiRefHeight + (eTempType == ABOVE_NEIGHBOR ? iTempHeight : 0);

  const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, uiRealW, uiRealH));
  uint32_t       uiPredStride = m_sgpmBuffer[0].getBuf(localUnitArea.Y()).stride;
  CHECK(eTempType != LEFT_ABOVE_NEIGHBOR, "left and above both should exist");
  
  const CodingStructure &cs = *cu.cs;
  m_ipaParam.multiRefIndex  = iTempWidth;

  initTimdIntraPatternLuma(cu, area, eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0,
                           eTempType != LEFT_NEIGHBOR ? iTempHeight : 0, uiRefWidth, uiRefHeight);

  Distortion sadWholeTM[NUM_LUMA_MODE];
  Distortion sadPartsTM[NUM_LUMA_MODE][GEO_NUM_PARTITION_MODE];
  uint8_t    ipmList[GEO_NUM_PARTITION_MODE][2][SGPM_NUM_MPM];
  bool       sadPartsNeeded[NUM_LUMA_MODE][GEO_NUM_PARTITION_MODE] = {};
  bool       ipmNeeded[NUM_LUMA_MODE]                                 = {};

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    if (!g_sgpmSplitDir[splitDir])
    {
      continue;
    }

    int16_t angle = g_geoParams[splitDir][0];
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      PU::getSgpmIntraMPMs(pu, ipmList[splitDir][partIdx], splitDir, g_geoTmShape[partIdx][angle]);
      for (int modeIdx = 0; modeIdx < SGPM_NUM_MPM; modeIdx++)
      {
        int ipmIdx                       = ipmList[splitDir][partIdx][modeIdx];
        ipmNeeded[ipmIdx]                = true;
        sadPartsNeeded[ipmIdx][splitDir] = true;
      }
    }
  }

  for (int ipmIdx = 0; ipmIdx < NUM_LUMA_MODE; ipmIdx++)
  {
    if (ipmNeeded[ipmIdx])
    {
      int iMode = MAP67TO131(ipmIdx);
      initPredTimdIntraParams(pu, area, iMode, true);
      Pel *tempPred = m_sgpmBuffer[0].getBuf(localUnitArea.Y()).buf;
      predTimdIntraAng(COMPONENT_Y, pu, iMode, tempPred, uiPredStride, uiRealW, uiRealH, eTempType,
                       (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth, (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);

      PelBuf predBuf = m_sgpmBuffer[0].getBuf(localUnitArea.Y());
      PelBuf recBuf  = cs.picture->getRecoBuf(area);
      PelBuf adBuf   = m_sgpmBuffer[0].getBuf(localUnitArea.Y());

      sadWholeTM[ipmIdx] =
        m_if.m_sadTM(pu, uiWidth, uiHeight, iTempWidth, iTempHeight, COMPONENT_Y, predBuf, recBuf, adBuf);

      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        if (sadPartsNeeded[ipmIdx][splitDir])
        {
          sadPartsTM[ipmIdx][splitDir] =
            m_if.m_sgpmSadTM(pu, uiWidth, uiHeight, iTempWidth, iTempHeight, COMPONENT_Y, splitDir, adBuf);
        }
      }
    }
  }
  // check every possible combination
  uint32_t cntComb = 0;
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    if (!g_sgpmSplitDir[splitDir])
    {
      continue;
    }

    for (int mode0Idx = 0; mode0Idx < SGPM_NUM_MPM; mode0Idx++)
    {
      for (int mode1Idx = 0; mode1Idx < SGPM_NUM_MPM; mode1Idx++)
      {
        int ipm0Idx = ipmList[splitDir][0][mode0Idx];
        int ipm1Idx = ipmList[splitDir][1][mode1Idx];
        if (ipm0Idx == ipm1Idx)
        {
          continue;
        }

        double cost = static_cast<double>(sadPartsTM[ipm0Idx][splitDir]) + static_cast<double>(sadWholeTM[ipm1Idx])
                      - static_cast<double>(sadPartsTM[ipm1Idx][splitDir]);

        cntComb++;

        if ((cntComb > SGPM_NUM && cost < candCostList[SGPM_NUM - 1]) || cntComb <= SGPM_NUM)
        {
          updateCandList(SgpmInfo(splitDir, ipm0Idx, ipm1Idx), cost, candModeList, candCostList, SGPM_NUM);
        }
      }
    }
  }
}
#endif

#if JVET_AD0085_MPM_SORTING
void IntraPrediction::deriveMPMSorted(const PredictionUnit& pu, uint8_t* mpm, int& sortedSize, int iStartIdx)
{
  SizeType uiWidth = pu.lwidth();
  SizeType uiHeight = pu.lheight();
  const CompArea area = pu.Y();
  int channelBitDepth = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);

  int      iCurX = pu.lx();
  int      iCurY = pu.ly();
  int      iRefX = -1, iRefY = -1;
  uint32_t uiRefWidth = 0, uiRefHeight = 0;
  int      iTempWidth = 1, iTempHeight = 1;

  static_vector<uint8_t, NUM_MOST_PROBABLE_MODES> uiModeList;
  static_vector<uint64_t, NUM_MOST_PROBABLE_MODES> uiCostList;
  int iBestN = std::min(NUM_PRIMARY_MOST_PROBABLE_MODES - 1, sortedSize);
  if (!pu.cs->pcv->isEncoder && pu.mpmFlag && pu.ipredIdx < iBestN)
  {
    iBestN = pu.ipredIdx;
  }

  TemplateType eTempType = CU::deriveTimdRefType(iCurX, iCurY, uiWidth, uiHeight, iTempWidth, iTempHeight, iRefX,
    iRefY, uiRefWidth, uiRefHeight);

  uint32_t      uiRealW2   = uiRefWidth + (eTempType == LEFT_NEIGHBOR ? iTempWidth : 0);
  uint32_t      uiRealH2   = uiRefHeight + (eTempType == ABOVE_NEIGHBOR ? iTempHeight : 0);
  const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, uiRealW2, uiRealH2));
  uint32_t       uiPredStride = m_intraPredBuffer[0].getBuf(localUnitArea.Y()).stride;
  Pel *piPred = m_intraPredBuffer[0].getBuf(localUnitArea.Y()).buf;

  if (eTempType == NO_NEIGHBOR)
  {
    return;
  }

  const CodingStructure& cs = *pu.cs;
  m_ipaParam.multiRefIndex = iTempWidth;
  Pel* piOrg = cs.picture->getRecoBuf(area).buf;
  int  iOrgStride = cs.picture->getRecoBuf(area).stride;
  piOrg += (iRefY - iCurY) * iOrgStride + (iRefX - iCurX);
  DistParam distParamSad[2];   // above, left
  distParamSad[0].applyWeight = false;
  distParamSad[0].useMR = false;
  distParamSad[1].applyWeight = false;
  distParamSad[1].useMR = false;
  if (eTempType == LEFT_ABOVE_NEIGHBOR)
  {
    m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg + iTempWidth, piPred + iTempWidth, iOrgStride,
      uiPredStride, channelBitDepth, COMPONENT_Y, uiWidth, iTempHeight, 0, 1, false);   // Use HAD (SATD) cost
    m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg + iTempHeight * iOrgStride,
      piPred + iTempHeight * uiPredStride, iOrgStride, uiPredStride, channelBitDepth,
      COMPONENT_Y, iTempWidth, uiHeight, 0, 1, false);
  }
  else if (eTempType == LEFT_NEIGHBOR)
  {
    m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth,
      COMPONENT_Y, iTempWidth, uiHeight, 0, 1, false);
  }
  else if (eTempType == ABOVE_NEIGHBOR)
  {
    m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth,
      COMPONENT_Y, uiWidth, iTempHeight, 0, 1, false);
  }
  initTimdIntraPatternLuma(*pu.cu, area, eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0,
    eTempType != LEFT_NEIGHBOR ? iTempHeight : 0, uiRefWidth, uiRefHeight);

  uint32_t uiRealW = uiRefWidth + (eTempType == LEFT_NEIGHBOR ? iTempWidth : 0);
  uint32_t uiRealH = uiRefHeight + (eTempType == ABOVE_NEIGHBOR ? iTempHeight : 0);

  for (int i = iStartIdx; i < sortedSize; i++)
  {
    uint64_t uiCost = 0;
    int      iMode = mpm[i];
    if (iMode > DC_IDX)
    {
      iMode = MAP67TO131(iMode);
    }
    initPredTimdIntraParams(pu, area, iMode);
    predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType,
      (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth,
      (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);
    if (eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      uiCost += distParamSad[0].distFunc(distParamSad[0]);
      uiCost += distParamSad[1].distFunc(distParamSad[1]);
    }
    else if (eTempType == LEFT_NEIGHBOR)
    {
      uiCost = distParamSad[1].distFunc(distParamSad[1]);
    }
    else if (eTempType == ABOVE_NEIGHBOR)
    {
      uiCost += distParamSad[0].distFunc(distParamSad[0]);
    }
    else
    {
      assert(0);
    }

    if (uiCostList.size() < iBestN || (uiCostList.size() >= iBestN && uiCost < uiCostList.back()))
    {
      updateCandList(mpm[i], uiCost, uiModeList, uiCostList, iBestN);
    }
  }

  sortedSize = int(uiModeList.size()) + iStartIdx;
  for (int i = 0; i < uiModeList.size(); i++)
  {
    mpm[i + iStartIdx] = uiModeList[i];
  }
}
#endif

#if JVET_AB0155_SGPM
int IntraPrediction::deriveTimdMode(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu, bool bFull, bool bHorVer)
{
  int      channelBitDepth = cu.slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  SizeType uiWidth         = cu.lwidth();
  SizeType uiHeight        = cu.lheight();

  int      iCurX = cu.lx();
  int      iCurY = cu.ly();
  int      iRefX = -1, iRefY = -1;
  uint32_t uiRefWidth = 0, uiRefHeight = 0;

  int iTempWidth = 4, iTempHeight = 4;
  if (uiWidth <= 8)
  {
    iTempWidth = 2;
  }
  if (uiHeight <= 8)
  {
    iTempHeight = 2;
  }

  TemplateType eTempType = CU::deriveTimdRefType(iCurX, iCurY, uiWidth, uiHeight, iTempWidth, iTempHeight, iRefX,
                                                  iRefY, uiRefWidth, uiRefHeight);
  auto &        pu        = *cu.firstPU;
  uint32_t      uiRealW   = uiRefWidth + (eTempType == LEFT_NEIGHBOR ? iTempWidth : 0);
  uint32_t      uiRealH   = uiRefHeight + (eTempType == ABOVE_NEIGHBOR ? iTempHeight : 0);
  const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, uiRealW, uiRealH));
  uint32_t       uiPredStride = m_intraPredBuffer[0].getBuf(localUnitArea.Y()).stride;
  Pel *piPred = m_intraPredBuffer[0].getBuf(localUnitArea.Y()).buf;

  if (eTempType != NO_NEIGHBOR)
  {
    const CodingStructure &cs = *cu.cs;
    m_ipaParam.multiRefIndex  = iTempWidth;
    Pel *piOrg                = cs.picture->getRecoBuf(area).buf;
    int  iOrgStride           = cs.picture->getRecoBuf(area).stride;
    piOrg += (iRefY - iCurY) * iOrgStride + (iRefX - iCurX);
    DistParam distParamSad[2];   // above, left
    distParamSad[0].applyWeight = false;
    distParamSad[0].useMR       = false;
    distParamSad[1].applyWeight = false;
    distParamSad[1].useMR       = false;
    if (eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg + iTempWidth, piPred + iTempWidth, iOrgStride,
                                       uiPredStride, channelBitDepth, COMPONENT_Y, uiWidth, iTempHeight, 0, 1,
                                       true);   // Use HAD (SATD) cost
      m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg + iTempHeight * iOrgStride,
                                       piPred + iTempHeight * uiPredStride, iOrgStride, uiPredStride, channelBitDepth,
                                       COMPONENT_Y, iTempWidth, uiHeight, 0, 1, true);   // Use HAD (SATD) cost
    }
    else if (eTempType == LEFT_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth,
                                       COMPONENT_Y, iTempWidth, uiHeight, 0, 1, true);
    }
    else if (eTempType == ABOVE_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth,
                                       COMPONENT_Y, uiWidth, iTempHeight, 0, 1, true);
    }
    initTimdIntraPatternLuma(cu, area, eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0,
                             eTempType != LEFT_NEIGHBOR ? iTempHeight : 0, uiRefWidth, uiRefHeight);

    uint32_t uiIntraDirNeighbor[5] = { 0 }, modeIdx = 0;

    bool     includedMode[EXT_VDIA_IDX + 1];
    memset(includedMode, false, (EXT_VDIA_IDX + 1) * sizeof(bool));
    auto &   pu      = *cu.firstPU;
    uint32_t uiRealW = uiRefWidth + (eTempType == LEFT_NEIGHBOR ? iTempWidth : 0);
    uint32_t uiRealH = uiRefHeight + (eTempType == ABOVE_NEIGHBOR ? iTempHeight : 0);
    uint64_t maxCost = (uint64_t)(iTempWidth * cu.lheight() + iTempHeight * cu.lwidth());

    uint64_t uiBestCost      = MAX_UINT64;
    int      iBestMode       = PLANAR_IDX;
    uint64_t uiSecondaryCost = MAX_UINT64;
    int      iSecondaryMode  = PLANAR_IDX;
#if JVET_AC0094_REF_SAMPLES_OPT
    bool bBestModeCheckWA      = true;
    bool bSecondaryModeCheckWA = true;
#endif

    uint64_t uiBestCostHor = MAX_UINT64;
    uint64_t uiBestCostVer = MAX_UINT64;
    int      iBestModeHor  = PLANAR_IDX;
    int      iBestModeVer  = PLANAR_IDX;

    const Position posLTx = pu.Y().topLeft();
    const Position posRTx = pu.Y().topRight();
    const Position posLBx = pu.Y().bottomLeft();
#if JVET_AC0094_REF_SAMPLES_OPT
    bool increaseMaxClose = true;
    bool decreaseMinClose = true;
    bool increaseMaxFar   = true;
    bool decreaseMinFar   = true;
#endif

    // left
    const PredictionUnit *puLeftx = pu.cs->getPURestricted(posLBx.offset(-1, 0), pu, pu.chType);
    if (puLeftx && CU::isIntra(*puLeftx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma(*puLeftx);
      if (!puLeftx->cu->timd
#if JVET_AD0085_TMRL_EXTENSION
        && !puLeftx->cu->tmrlFlag
#endif
        )
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if (!includedMode[uiIntraDirNeighbor[modeIdx]])
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }

    // above
    const PredictionUnit *puAbovex = pu.cs->getPURestricted(posRTx.offset(0, -1), pu, pu.chType);
    if (puAbovex && CU::isIntra(*puAbovex->cu) && CU::isSameCtu(*pu.cu, *puAbovex->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma(*puAbovex);
      if (!puAbovex->cu->timd
#if JVET_AD0085_TMRL_EXTENSION
        && !puAbovex->cu->tmrlFlag
#endif
        )
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if (!includedMode[uiIntraDirNeighbor[modeIdx]])
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }

    // below left
    const PredictionUnit *puLeftBottomx = cs.getPURestricted(posLBx.offset(-1, 1), pu, pu.chType);
    if (puLeftBottomx && CU::isIntra(*puLeftBottomx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma(*puLeftBottomx);
      if (!puLeftBottomx->cu->timd
#if JVET_AD0085_TMRL_EXTENSION
        && !puLeftBottomx->cu->tmrlFlag
#endif
        )
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if (!includedMode[uiIntraDirNeighbor[modeIdx]])
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }

#if JVET_AC0094_REF_SAMPLES_OPT
    if (!puLeftx)
    {
      decreaseMinClose = false;
    }
    const PredictionUnit *puLeftBottomFarx = cs.getPURestricted(posLBx.offset(-1, uiHeight), pu, pu.chType);
    if (!puLeftBottomFarx)
    {
      decreaseMinFar = false;
    }
#endif

    // above right
    const PredictionUnit *puAboveRightx = cs.getPURestricted(posRTx.offset(1, -1), pu, pu.chType);
    if (puAboveRightx && CU::isIntra(*puAboveRightx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma(*puAboveRightx);
      if (!puAboveRightx->cu->timd
#if JVET_AD0085_TMRL_EXTENSION
        && !puAboveRightx->cu->tmrlFlag
#endif
        )
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if (!includedMode[uiIntraDirNeighbor[modeIdx]])
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }
#if JVET_AC0094_REF_SAMPLES_OPT
    if (!puAbovex)
    {
      increaseMaxClose = false;
    }
    const PredictionUnit *puAboveRightFarx = cs.getPURestricted(posRTx.offset(uiWidth, -1), pu, pu.chType);
    if (!puAboveRightFarx)
    {
      increaseMaxFar = false;
    }
#endif
    // above left
    const PredictionUnit *puAboveLeftx = cs.getPURestricted(posLTx.offset(-1, -1), pu, pu.chType);
    if (puAboveLeftx && CU::isIntra(*puAboveLeftx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma(*puAboveLeftx);
      if (!puAboveLeftx->cu->timd
#if JVET_AD0085_TMRL_EXTENSION
        && !puAboveLeftx->cu->tmrlFlag
#endif
        )
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if (!includedMode[uiIntraDirNeighbor[modeIdx]])
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }

    bool bNoAngular = false;
    if (modeIdx >= 2)
    {
      bNoAngular = true;
      for (uint32_t i = 0; i < modeIdx; i++)
      {
        if (uiIntraDirNeighbor[i] > DC_IDX)
        {
          bNoAngular = false;
          break;
        }
      }
    }

    if (bNoAngular)
    {
      if (bFull)
      {
        for (int iMode = 0; iMode <= 1; iMode++)
        {
          uint64_t uiCost = 0;
          initPredTimdIntraParams(pu, area, iMode);
          predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType,
                           (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth,
                           (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);
          if (eTempType == LEFT_ABOVE_NEIGHBOR)
          {
            uiCost += distParamSad[0].distFunc(distParamSad[0]);
            uiCost += distParamSad[1].distFunc(distParamSad[1]);
          }
          else if (eTempType == LEFT_NEIGHBOR)
          {
            uiCost = distParamSad[1].distFunc(distParamSad[1]);
          }
          else if (eTempType == ABOVE_NEIGHBOR)
          {
            uiCost += distParamSad[0].distFunc(distParamSad[0]);
          }
          else
          {
            assert(0);
          }

          if (uiCost < uiBestCost)
          {
            uiBestCost = uiCost;
            iBestMode  = iMode;
          }
          if (uiBestCost <= maxCost)
          {
            break;
          }
        }
        cu.timdMode      = iBestMode;
#if JVET_AC0094_REF_SAMPLES_OPT
        cu.timdModeCheckWA = true;
#endif
        cu.timdIsBlended = false;
      }
      if (bHorVer)
      {
        cu.timdHor = PLANAR_IDX;
        cu.timdVer = PLANAR_IDX;
      }
      return iBestMode;
    }
#if SECONDARY_MPM
    uint8_t mpmList[NUM_MOST_PROBABLE_MODES];
    uint8_t intraNonMPM[NUM_NON_MPM_MODES];
    PU::getIntraMPMs(pu, mpmList, intraNonMPM
#if JVET_AC0094_REF_SAMPLES_OPT
                     , true
#endif
    );
#else
    unsigned mpmList[NUM_MOST_PROBABLE_MODES];
    PU::getIntraMPMs(pu, mpmList);
#endif
    unsigned mpmExtraList[NUM_MOST_PROBABLE_MODES + 3];   // +DC/VER/HOR
    int      maxModeNum      = NUM_MOST_PROBABLE_MODES;
    unsigned modeCandList[3] = { DC_IDX, HOR_IDX, VER_IDX };
    bool     bNotExist[3]    = { true, true, true };
    for (int i = 0; i < NUM_MOST_PROBABLE_MODES; i++)
    {
      mpmExtraList[i] = mpmList[i];
      if (bNotExist[0] && mpmList[i] == DC_IDX)
      {
        bNotExist[0] = false;
      }
      if (bNotExist[1] && mpmList[i] == HOR_IDX)
      {
        bNotExist[1] = false;
      }
      if (bNotExist[2] && mpmList[i] == VER_IDX)
      {
        bNotExist[2] = false;
      }
    }
    for (int i = 0; i < 3; i++)
    {
      if (bNotExist[i])
      {
        mpmExtraList[maxModeNum++] = modeCandList[i];
      }
    }
    bool updateFull = true;
    for (int i = 0; i < maxModeNum; i++)
    {
      uint64_t uiCost    = 0;
      int      iMode     = mpmExtraList[i];
      uint64_t uiCostVer = UINT64_MAX;
      uint64_t uiCostHor = UINT64_MAX;
      uint64_t tmpCost0  = 0;
      uint64_t tmpCost1  = 0;
      if (iMode > DC_IDX)
      {
        iMode = MAP67TO131(iMode);
      }
      else
      {
        if (!bFull && bHorVer)
        {
          continue;
        }
      }
      initPredTimdIntraParams(pu, area, iMode);
      predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType,
                       (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth, (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);
      if (eTempType == LEFT_ABOVE_NEIGHBOR)
      {
        if (bFull && updateFull)
        {
          tmpCost0 = distParamSad[0].distFunc(distParamSad[0]);
          tmpCost1 = distParamSad[1].distFunc(distParamSad[1]);
        }
        else
        {
          if (iMode > EXT_DIA_IDX)
          {
            tmpCost0 = distParamSad[0].distFunc(distParamSad[0]);
          }
          else
          {
            tmpCost1 = distParamSad[1].distFunc(distParamSad[1]);
          }
        }
      }
      else if (eTempType == LEFT_NEIGHBOR)
      {
        tmpCost0 = distParamSad[1].distFunc(distParamSad[1]);
      }
      else if (eTempType == ABOVE_NEIGHBOR)
      {
        tmpCost1 = distParamSad[0].distFunc(distParamSad[0]);
      }
      else
      {
        assert(0);
      }

      if (bFull && updateFull)
      {
        uiCost = tmpCost0 + tmpCost1;
        if (uiCost < uiBestCost)
        {
          uiSecondaryCost = uiBestCost;
          iSecondaryMode  = iBestMode;
          uiBestCost      = uiCost;
          iBestMode       = iMode;
        }
        else if (uiCost < uiSecondaryCost)
        {
          uiSecondaryCost = uiCost;
          iSecondaryMode  = iMode;
        }
        if (uiSecondaryCost <= maxCost)
        {
          updateFull = false;
          if (!bHorVer)
          {
            break;
          }
        }
      }
      if (bHorVer && iMode > DC_IDX)
      {
        if (eTempType == LEFT_ABOVE_NEIGHBOR)
        {
          if (iMode > EXT_DIA_IDX)
          {
            uiCostVer = tmpCost0;
          }
          else
          {
            uiCostHor = tmpCost1;
          }
        }
        else if (eTempType == LEFT_NEIGHBOR)
        {
          uiCostHor = tmpCost1;
        }
        else if (eTempType == ABOVE_NEIGHBOR)
        {
          uiCostVer = tmpCost0;
        }
        if (uiCostHor < uiBestCostHor)
        {
          uiBestCostHor = uiCostHor;
          iBestModeHor  = iMode;
        }
        if (uiCostVer < uiBestCostVer)
        {
          uiBestCostVer = uiCostVer;
          iBestModeVer  = iMode;
        }
      }
    }
#if JVET_AC0094_REF_SAMPLES_OPT
    // Check modes removed due to WAIP
    int modeShiftExt[] = { 0, 11, 19, 23, 27, 29 };
    int deltaSize      = abs(floorLog2(uiWidth) - floorLog2(uiHeight));

    // Compute max allowed mode and min allowed mode to not exceed ref
    int maxModeOrig =
      uiWidth >= uiHeight ? EXT_VDIA_IDX + modeShiftExt[deltaSize] : EXT_VDIA_IDX - modeShiftExt[deltaSize];
    int minModeOrig = uiWidth >= uiHeight ? DC_IDX + 1 + modeShiftExt[deltaSize] : DC_IDX + 1 - modeShiftExt[deltaSize];

    int newMinMode = minModeOrig;
    int newMaxMode = maxModeOrig;

    if (uiWidth >= uiHeight)
    {
      if (increaseMaxFar)
      {
        newMaxMode += modeShiftExt[std::min(deltaSize + 2, 5)] - modeShiftExt[deltaSize];
      }
      else if (increaseMaxClose)
      {
        newMaxMode += modeShiftExt[std::min(deltaSize + 1, 5)] - modeShiftExt[deltaSize];
      }
    }
    if (uiWidth <= uiHeight)
    {
      if (decreaseMinFar)
      {
        newMinMode -= modeShiftExt[std::min(deltaSize + 2, 5)] - modeShiftExt[deltaSize];
      }
      else if (decreaseMinClose)
      {
        newMinMode -= modeShiftExt[std::min(deltaSize + 1, 5)] - modeShiftExt[deltaSize];
      }
    }
    int previous = deltaSize;
    int extRef   = (decreaseMinFar || increaseMaxFar) ? 2 : (decreaseMinClose || increaseMaxClose) ? 1 : 0;
    for (int d = 1; d <= extRef; d++)
    {
      if (uiWidth > uiHeight)
      {
        newMinMode -= abs(modeShiftExt[abs(previous)] - modeShiftExt[std::min(abs(deltaSize - d), 5)]);
      }
      if (uiWidth < uiHeight)
      {
        newMaxMode += abs(modeShiftExt[abs(previous)] - modeShiftExt[std::min(abs(deltaSize - d), 5)]);
      }
      previous = deltaSize - d;
    }

    {
      for (int i = newMinMode + 1; i < newMaxMode; i += 5)
      {
        // Mode is a regular mode (already tested)
        if (i >= minModeOrig && i <= maxModeOrig)
        {
          i = maxModeOrig - 1;
          continue;
        }
        uint64_t uiCost = 0;
        int      iMode  = i;
        iMode           = getTimdRegularAngleExt(uiWidth, uiHeight, iMode);
        initPredTimdIntraParams(pu, area, iMode, false, false);
        iMode = getTimdWideAngleExt(uiWidth, uiHeight, iMode);
        predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType,
                         (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth,
                         (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);
        if (eTempType == LEFT_ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
          uiCost += distParamSad[1].distFunc(distParamSad[1]);
        }
        else if (eTempType == LEFT_NEIGHBOR)
        {
          uiCost = distParamSad[1].distFunc(distParamSad[1]);
        }
        else if (eTempType == ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
        }
        else
        {
          assert(0);
        }

        if (uiCost < uiBestCost)
        {
          uiSecondaryCost = uiBestCost;
          iSecondaryMode  = iBestMode;
          uiBestCost      = uiCost;
          iBestMode             = iMode;
          bSecondaryModeCheckWA = bBestModeCheckWA;
          bBestModeCheckWA      = false;
        }
        else if (uiCost < uiSecondaryCost)
        {
          uiSecondaryCost = uiCost;
          iSecondaryMode        = iMode;
          bSecondaryModeCheckWA = false;
        }
        if (uiSecondaryCost <= maxCost)
        {
          break;
        }
      }
    }
#endif
    if(bFull)
    {
      int midMode = iBestMode;
#if JVET_AC0094_REF_SAMPLES_OPT
      bool checkWideAngle = bBestModeCheckWA;
      if ((midMode > DC_IDX && uiBestCost > maxCost && checkWideAngle) || (uiBestCost > maxCost && !checkWideAngle))
#else
      if (midMode > DC_IDX && uiBestCost > maxCost)
#endif
      {
        for (int i = -1; i <= 1; i += 2)
        {
          int iMode = midMode + i;
#if JVET_AC0094_REF_SAMPLES_OPT
          if (checkWideAngle && (iMode <= DC_IDX || iMode > EXT_VDIA_IDX))
          {
            continue;
          }
          if (!checkWideAngle)
          {
            if (iMode >= minModeOrig && iMode <= maxModeOrig)
            {
              continue;
            }
            iMode = getTimdRegularAngleExt(uiWidth, uiHeight, iMode);
          }
#else
          if (iMode <= DC_IDX || iMode > EXT_VDIA_IDX)
          {
            continue;
          }
#endif
          initPredTimdIntraParams(pu, area, iMode
#if JVET_AC0094_REF_SAMPLES_OPT
              , false, checkWideAngle
#endif
          );
#if JVET_AC0094_REF_SAMPLES_OPT
          if (!checkWideAngle)
          {
            iMode = getTimdWideAngleExt(uiWidth, uiHeight, iMode);
          }
#endif
          predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType,
                           (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth,
                           (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);
          uint64_t uiCost = 0;
          if (eTempType == LEFT_ABOVE_NEIGHBOR)
          {
            uiCost += distParamSad[0].distFunc(distParamSad[0]);
            uiCost += distParamSad[1].distFunc(distParamSad[1]);
          }
          else if (eTempType == LEFT_NEIGHBOR)
          {
            uiCost = distParamSad[1].distFunc(distParamSad[1]);
          }
          else if (eTempType == ABOVE_NEIGHBOR)
          {
            uiCost += distParamSad[0].distFunc(distParamSad[0]);
          }
          else
          {
            assert(0);
          }

          if (uiCost < uiBestCost)
          {
            uiBestCost = uiCost;
            iBestMode  = iMode;
          }
          if (uiBestCost <= maxCost)
          {
            break;
          }
        }
      }

      midMode = iSecondaryMode;
#if JVET_AC0094_REF_SAMPLES_OPT
      checkWideAngle = bSecondaryModeCheckWA;
      if ((midMode > DC_IDX && uiBestCost > maxCost && checkWideAngle) || (uiBestCost > maxCost && !checkWideAngle))
#else
      if (midMode > DC_IDX && uiSecondaryCost > maxCost)
#endif
      {
        for (int i = -1; i <= 1; i += 2)
        {
          int iMode = midMode + i;
#if JVET_AC0094_REF_SAMPLES_OPT
          if (checkWideAngle && (iMode <= DC_IDX || iMode > EXT_VDIA_IDX))
#else
          if (iMode <= DC_IDX || iMode > EXT_VDIA_IDX)
#endif
          {
            continue;
          }
#if JVET_AC0094_REF_SAMPLES_OPT
          if (!checkWideAngle)
          {
            if (iMode >= minModeOrig && iMode <= maxModeOrig)
            {
              continue;
            }
            iMode = getTimdRegularAngleExt(uiWidth, uiHeight, iMode);
          }
#endif
          initPredTimdIntraParams(pu, area, iMode
#if JVET_AC0094_REF_SAMPLES_OPT
        , false, checkWideAngle
#endif
        );
#if JVET_AC0094_REF_SAMPLES_OPT
          if (!checkWideAngle)
          {
            iMode = getTimdWideAngleExt(uiWidth, uiHeight, iMode);
          }
#endif
          predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType,
                           (eTempType == ABOVE_NEIGHBOR) ? 0 : iTempWidth,
                           (eTempType == LEFT_NEIGHBOR) ? 0 : iTempHeight);
          uint64_t uiCost = 0;
          if (eTempType == LEFT_ABOVE_NEIGHBOR)
          {
            uiCost += distParamSad[0].distFunc(distParamSad[0]);
            uiCost += distParamSad[1].distFunc(distParamSad[1]);
          }
          else if (eTempType == LEFT_NEIGHBOR)
          {
            uiCost = distParamSad[1].distFunc(distParamSad[1]);
          }
          else if (eTempType == ABOVE_NEIGHBOR)
          {
            uiCost += distParamSad[0].distFunc(distParamSad[0]);
          }
          else
          {
            assert(0);
          }

          if (uiCost < uiSecondaryCost)
          {
            uiSecondaryCost = uiCost;
            iSecondaryMode  = iMode;
          }
          if (uiSecondaryCost <= maxCost)
          {
            break;
          }
        }
      }

#if JVET_AC0094_REF_SAMPLES_OPT
      if (!bBestModeCheckWA)
      {
        iBestMode = getTimdRegularAngleExt(uiWidth, uiHeight, iBestMode);
      }
      if (!bSecondaryModeCheckWA)
      {
        iSecondaryMode = getTimdRegularAngleExt(uiWidth, uiHeight, iSecondaryMode);
      }
#endif
      // if( uiSecondaryCost < 2 * uiBestCost ), 2 * uiBestCost can overflow uint64_t
      if (uiSecondaryCost < uiBestCost || (uiSecondaryCost - uiBestCost < uiBestCost))
      {
        cu.timdMode          = iBestMode;
        cu.timdIsBlended     = true;
        cu.timdModeSecondary = iSecondaryMode;
#if JVET_AC0094_REF_SAMPLES_OPT
        cu.timdModeCheckWA          = bBestModeCheckWA;
        cu.timdModeSecondaryCheckWA = bSecondaryModeCheckWA;
#endif

        const int blend_sum_weight = 6;
        int       sum_weight       = 1 << blend_sum_weight;

#if JVET_X0149_TIMD_DIMD_LUT
        int      g_gradDivTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
        uint64_t s0                 = uiSecondaryCost;
        // uiBestCost + uiSecondaryCost can overlow uint64_t
        uint64_t s1 = (MAX_UINT64 - uiSecondaryCost < uiBestCost) ? MAX_UINT64 : (uiBestCost + uiSecondaryCost);
        int      x  = floorLog2Uint64(s1);
        CHECK(x < 0, "floor log2 value should be no negative");
        int norm_s1 = int(s1 << 4 >> x) & 15;
        int v       = g_gradDivTable[norm_s1] | 8;
        x += (norm_s1 != 0);
        int shift  = x + 3;
        int add    = (1 << (shift - 1));
        int iRatio = int((s0 * v * sum_weight + add) >> shift);

        if (iRatio > sum_weight)
        {
          iRatio = sum_weight;
        }

        CHECK(iRatio > sum_weight, "Wrong DIMD ratio");
#else
        double dRatio = 0.0;
        dRatio        = (double) uiSecondaryCost / (double) (uiBestCost + uiSecondaryCost);
        int iRatio    = static_cast<int>(dRatio * sum_weight + 0.5);
#endif
        cu.timdFusionWeight[0] = iRatio;
        cu.timdFusionWeight[1] = sum_weight - iRatio;
      }
      else
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        cu.timdModeCheckWA          = bBestModeCheckWA;
        cu.timdModeSecondaryCheckWA = true;
#endif
        cu.timdMode      = iBestMode;
        cu.timdIsBlended = false;
      }
    }
    if (bHorVer)
    {
      cu.timdHor = iBestModeHor;
      cu.timdVer = iBestModeVer;
    }

    return iBestMode;
  }
  else
  {
#if JVET_AC0094_REF_SAMPLES_OPT
    cu.timdModeCheckWA          = true;
    cu.timdModeSecondaryCheckWA = true;
#endif
    if (bFull)
    {
      cu.timdMode      = PLANAR_IDX;
      cu.timdIsBlended = false;
    }
    if (bHorVer)
    {
      cu.timdHor = PLANAR_IDX;
      cu.timdVer = PLANAR_IDX;
    }
    return PLANAR_IDX;
  }
}
#else   // SGPM

int IntraPrediction::deriveTimdMode( const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu )
{
  int channelBitDepth = cu.slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  SizeType uiWidth = cu.lwidth();
  SizeType uiHeight = cu.lheight();

  static Pel predLuma[(MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE) * (MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE)];
  memset(predLuma, 0, (MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE) * (MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE) * sizeof(Pel));
  Pel* piPred = predLuma;
  uint32_t uiPredStride = MAX_CU_SIZE + DIMD_MAX_TEMP_SIZE;

  int  iCurX  = cu.lx();
  int  iCurY  = cu.ly();
  int  iRefX  = -1, iRefY = -1;
  uint32_t uiRefWidth = 0, uiRefHeight = 0;

  int iTempWidth = 4, iTempHeight = 4;
  if(uiWidth <= 8)
  {
    iTempWidth = 2;
  }
  if(uiHeight <= 8)
  {
    iTempHeight = 2;
  }

  TemplateType eTempType = CU::deriveTimdRefType(iCurX, iCurY, uiWidth, uiHeight, iTempWidth, iTempHeight, iRefX, iRefY, uiRefWidth, uiRefHeight);

  if (eTempType != NO_NEIGHBOR)
  {
    const CodingStructure& cs   = *cu.cs;
    m_ipaParam.multiRefIndex        = iTempWidth;
    Pel* piOrg = cs.picture->getRecoBuf( area ).buf;
    int iOrgStride = cs.picture->getRecoBuf( area ).stride;
    piOrg += (iRefY - iCurY) * iOrgStride + (iRefX - iCurX);
    DistParam distParamSad[2]; // above, left
    distParamSad[0].applyWeight = false;
    distParamSad[0].useMR = false;
    distParamSad[1].applyWeight = false;
    distParamSad[1].useMR = false;
    if(eTempType == LEFT_ABOVE_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg + iTempWidth, piPred + iTempWidth, iOrgStride, uiPredStride, channelBitDepth, COMPONENT_Y, uiWidth, iTempHeight, 0, 1, true); // Use HAD (SATD) cost
      m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg + iTempHeight * iOrgStride, piPred + iTempHeight * uiPredStride, iOrgStride, uiPredStride, channelBitDepth, COMPONENT_Y, iTempWidth, uiHeight, 0, 1, true); // Use HAD (SATD) cost
    }
    else if(eTempType == LEFT_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth, COMPONENT_Y, iTempWidth, uiHeight, 0, 1, true);
    }
    else if(eTempType == ABOVE_NEIGHBOR)
    {
      m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg, piPred, iOrgStride, uiPredStride, channelBitDepth, COMPONENT_Y, uiWidth, iTempHeight, 0, 1, true);
    }
    initTimdIntraPatternLuma(cu, area, eTempType != ABOVE_NEIGHBOR ? iTempWidth : 0, eTempType != LEFT_NEIGHBOR ? iTempHeight : 0, uiRefWidth, uiRefHeight);

    uint32_t uiIntraDirNeighbor[5] = {0}, modeIdx = 0;
    bool includedMode[EXT_VDIA_IDX + 1];
    memset(includedMode, false, (EXT_VDIA_IDX + 1) * sizeof(bool));
    auto &pu = *cu.firstPU;
    uint32_t uiRealW = uiRefWidth + (eTempType == LEFT_NEIGHBOR? iTempWidth : 0);
    uint32_t uiRealH = uiRefHeight + (eTempType == ABOVE_NEIGHBOR? iTempHeight : 0);
    uint64_t maxCost = (uint64_t)(iTempWidth * cu.lheight() + iTempHeight * cu.lwidth());

    uint64_t uiBestCost = MAX_UINT64;
    int iBestMode = PLANAR_IDX;
    uint64_t uiSecondaryCost = MAX_UINT64;
    int iSecondaryMode = PLANAR_IDX;

    const Position posLTx = pu.Y().topLeft();
    const Position posRTx = pu.Y().topRight();
    const Position posLBx = pu.Y().bottomLeft();

    // left
    const PredictionUnit *puLeftx = pu.cs->getPURestricted(posLBx.offset(-1, 0), pu, pu.chType);
    if (puLeftx && CU::isIntra(*puLeftx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma( *puLeftx );
      if (!puLeftx->cu->timd)
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if( !includedMode[uiIntraDirNeighbor[modeIdx]] )
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }
    // above
    const PredictionUnit *puAbovex = pu.cs->getPURestricted(posRTx.offset(0, -1), pu, pu.chType);
    if (puAbovex && CU::isIntra(*puAbovex->cu) && CU::isSameCtu(*pu.cu, *puAbovex->cu))
    {
      uiIntraDirNeighbor[modeIdx] =PU::getIntraDirLuma( *puAbovex );
      if (!puAbovex->cu->timd)
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if( !includedMode[uiIntraDirNeighbor[modeIdx]] )
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }
    // below left
    const PredictionUnit *puLeftBottomx = cs.getPURestricted( posLBx.offset( -1, 1 ), pu, pu.chType );
    if (puLeftBottomx && CU::isIntra(*puLeftBottomx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma( *puLeftBottomx );
      if (!puLeftBottomx->cu->timd)
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if( !includedMode[uiIntraDirNeighbor[modeIdx]] )
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }
    // above right
    const PredictionUnit *puAboveRightx = cs.getPURestricted( posRTx.offset( 1, -1 ), pu, pu.chType );
    if (puAboveRightx && CU::isIntra(*puAboveRightx->cu))
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma( *puAboveRightx );
      if (!puAboveRightx->cu->timd)
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if( !includedMode[uiIntraDirNeighbor[modeIdx]] )
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }
    //above left
    const PredictionUnit *puAboveLeftx = cs.getPURestricted( posLTx.offset( -1, -1 ), pu, pu.chType );
    if( puAboveLeftx && CU::isIntra(*puAboveLeftx->cu) )
    {
      uiIntraDirNeighbor[modeIdx] = PU::getIntraDirLuma( *puAboveLeftx );
      if (!puAboveLeftx->cu->timd)
      {
        uiIntraDirNeighbor[modeIdx] = MAP67TO131(uiIntraDirNeighbor[modeIdx]);
      }
      if( !includedMode[uiIntraDirNeighbor[modeIdx]] )
      {
        includedMode[uiIntraDirNeighbor[modeIdx]] = true;
        modeIdx++;
      }
    }
    bool bNoAngular = false;
    if(modeIdx >= 2)
    {
      bNoAngular = true;
      for(uint32_t i = 0; i < modeIdx; i++)
      {
        if(uiIntraDirNeighbor[i] > DC_IDX)
        {
          bNoAngular = false;
          break;
        }
      }
    }

    if (bNoAngular)
    {
      for(int iMode = 0; iMode <= 1; iMode ++)
      {
        uint64_t uiCost = 0;
        initPredTimdIntraParams(pu, area, iMode);
        predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType, (eTempType == ABOVE_NEIGHBOR)? 0: iTempWidth, (eTempType == LEFT_NEIGHBOR)? 0: iTempHeight);
        if(eTempType == LEFT_ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
          uiCost += distParamSad[1].distFunc(distParamSad[1]);
        }
        else if(eTempType == LEFT_NEIGHBOR)
        {
          uiCost = distParamSad[1].distFunc(distParamSad[1]);
        }
        else if(eTempType == ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
        }
        else
        {
          assert(0);
        }

        if(uiCost < uiBestCost)
        {
          uiBestCost = uiCost;
          iBestMode = iMode;
        }
        if(uiBestCost <= maxCost)
        {
          break;
        }
      }
      cu.timdMode = iBestMode;
      cu.timdIsBlended = false;

      return iBestMode;
    }
#if SECONDARY_MPM
    uint8_t mpmList[NUM_MOST_PROBABLE_MODES];
    uint8_t intraNonMPM[NUM_NON_MPM_MODES];
    PU::getIntraMPMs(pu, mpmList, intraNonMPM);
#else
    unsigned mpmList[NUM_MOST_PROBABLE_MODES];
    PU::getIntraMPMs(pu, mpmList);
#endif
    unsigned mpmExtraList[NUM_MOST_PROBABLE_MODES + 3]; // +DC/VER/HOR
    int maxModeNum = NUM_MOST_PROBABLE_MODES;
    unsigned modeCandList[3] = {DC_IDX, HOR_IDX, VER_IDX};
    bool bNotExist[3] = {true, true, true};
    for (int i = 0; i < NUM_MOST_PROBABLE_MODES; i++)
    {
      mpmExtraList[i] = mpmList[i];
      if (bNotExist[0] && mpmList[i] == DC_IDX)
      {
        bNotExist[0] = false;
      }
      if (bNotExist[1] && mpmList[i] == HOR_IDX)
      {
        bNotExist[1] = false;
      }
      if (bNotExist[2] && mpmList[i] == VER_IDX)
      {
        bNotExist[2] = false;
      }
    }
    for (int i = 0; i < 3; i++)
    {
      if (bNotExist[i])
      {
        mpmExtraList[maxModeNum++] = modeCandList[i];
      }
    }
    for(int i = 0; i < maxModeNum; i ++)
    {
      uint64_t uiCost = 0;
      int iMode = mpmExtraList[i];
      if (iMode > DC_IDX)
      {
        iMode = MAP67TO131(iMode);
      }
      initPredTimdIntraParams(pu, area, iMode);
      predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType, (eTempType == ABOVE_NEIGHBOR)? 0: iTempWidth, (eTempType == LEFT_NEIGHBOR)? 0: iTempHeight);
      if(eTempType == LEFT_ABOVE_NEIGHBOR)
      {
        uiCost += distParamSad[0].distFunc(distParamSad[0]);
        uiCost += distParamSad[1].distFunc(distParamSad[1]);
      }
      else if(eTempType == LEFT_NEIGHBOR)
      {
        uiCost = distParamSad[1].distFunc(distParamSad[1]);
      }
      else if(eTempType == ABOVE_NEIGHBOR)
      {
        uiCost += distParamSad[0].distFunc(distParamSad[0]);
      }
      else
      {
        assert(0);
      }

      if( uiCost < uiBestCost )
      {
        uiSecondaryCost = uiBestCost;
        iSecondaryMode  = iBestMode;
        uiBestCost  = uiCost;
        iBestMode = iMode;
      }
      else if (uiCost < uiSecondaryCost)
      {
        uiSecondaryCost = uiCost;
        iSecondaryMode  = iMode;
      }
      if (uiSecondaryCost <= maxCost)
      {
        break;
      }
    }

    int midMode = iBestMode;
    if (midMode > DC_IDX && uiBestCost > maxCost)
    {
      for (int i = -1; i <= 1; i+=2)
      {
        int iMode = midMode + i;
        if (iMode <= DC_IDX || iMode > EXT_VDIA_IDX)
        {
          continue;
        }
        initPredTimdIntraParams(pu, area, iMode);
        predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType, (eTempType == ABOVE_NEIGHBOR)? 0: iTempWidth, (eTempType == LEFT_NEIGHBOR)? 0: iTempHeight);
        uint64_t uiCost = 0;
        if(eTempType == LEFT_ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
          uiCost += distParamSad[1].distFunc(distParamSad[1]);
        }
        else if(eTempType == LEFT_NEIGHBOR)
        {
          uiCost = distParamSad[1].distFunc(distParamSad[1]);
        }
        else if(eTempType == ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
        }
        else
        {
          assert(0);
        }

        if(uiCost < uiBestCost)
        {
          uiBestCost  = uiCost;
          iBestMode = iMode;
        }
        if(uiBestCost <= maxCost)
        {
          break;
        }
      }
    }

    midMode = iSecondaryMode;
    if (midMode > DC_IDX && uiSecondaryCost > maxCost)
    {
      for (int i = -1; i <= 1; i+=2)
      {
        int iMode = midMode + i;
        if (iMode <= DC_IDX || iMode > EXT_VDIA_IDX)
        {
          continue;
        }
        initPredTimdIntraParams(pu, area, iMode);
        predTimdIntraAng(COMPONENT_Y, pu, iMode, piPred, uiPredStride, uiRealW, uiRealH, eTempType, (eTempType == ABOVE_NEIGHBOR)? 0: iTempWidth, (eTempType == LEFT_NEIGHBOR)? 0: iTempHeight);
        uint64_t uiCost = 0;
        if(eTempType == LEFT_ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
          uiCost += distParamSad[1].distFunc(distParamSad[1]);
        }
        else if(eTempType == LEFT_NEIGHBOR)
        {
          uiCost = distParamSad[1].distFunc(distParamSad[1]);
        }
        else if(eTempType == ABOVE_NEIGHBOR)
        {
          uiCost += distParamSad[0].distFunc(distParamSad[0]);
        }
        else
        {
          assert(0);
        }

        if(uiCost < uiSecondaryCost)
        {
          uiSecondaryCost  = uiCost;
          iSecondaryMode = iMode;
        }
        if(uiSecondaryCost <= maxCost)
        {
          break;
        }
      }
    }

    // if( uiSecondaryCost < 2 * uiBestCost ), 2 * uiBestCost can overflow uint64_t
    if( uiSecondaryCost < uiBestCost || (uiSecondaryCost - uiBestCost < uiBestCost) )
  {
    cu.timdMode         = iBestMode;
    cu.timdIsBlended    = true;
    cu.timdModeSecondary = iSecondaryMode;

    const int blend_sum_weight = 6;
    int       sum_weight       = 1 << blend_sum_weight;

#if JVET_X0149_TIMD_DIMD_LUT
    int g_gradDivTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
    uint64_t s0 = uiSecondaryCost;
    // uiBestCost + uiSecondaryCost can overlow uint64_t
    uint64_t s1 = (MAX_UINT64 - uiSecondaryCost < uiBestCost) ? MAX_UINT64 : (uiBestCost + uiSecondaryCost);
    int x = floorLog2Uint64(s1);
    CHECK(x < 0, "floor log2 value should be no negative");
    int norm_s1 = int(s1 << 4 >> x) & 15;
    int v = g_gradDivTable[norm_s1] | 8;
    x += (norm_s1 != 0);
    int shift = x + 3;
    int add = (1 << (shift - 1));
    int iRatio = int((s0 * v * sum_weight + add) >> shift);

    if( iRatio > sum_weight )
    {
      iRatio = sum_weight;
    }

    CHECK( iRatio > sum_weight, "Wrong DIMD ratio" );
#else
    double dRatio       = 0.0;
    dRatio              = (double) uiSecondaryCost / (double) (uiBestCost + uiSecondaryCost);
    int iRatio          = static_cast<int>(dRatio * sum_weight + 0.5);
#endif
    cu.timdFusionWeight[0] = iRatio;
    cu.timdFusionWeight[1] = sum_weight - iRatio;
  }
  else
  {
    cu.timdMode      = iBestMode;
    cu.timdIsBlended = false;
  }

    return iBestMode;
  }
  else
  {
    cu.timdMode = PLANAR_IDX;
    cu.timdIsBlended = false;

    return PLANAR_IDX;
  }
}
#endif
#if INTRA_TRANS_ENC_OPT
void IntraPrediction::timdBlending(Pel *pDst, int strideDst, Pel *pSrc, int strideSrc, int w0, int w1, int width, int height)
{
  const int log2WeightSum = 6;
  Pel *pelPred = pDst;
  Pel *pelPredFusion = pSrc;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int blend = pelPred[x] * w0;
      blend += pelPredFusion[x] * w1;
      pelPred[x] = (Pel)(blend >> log2WeightSum);
    }

    pelPred += strideDst;
    pelPredFusion += strideSrc;
  }
}
#endif
#endif

#if JVET_AC0112_IBC_CIIP
void IntraPrediction::ibcCiipBlending(Pel *pDst, int strideDst, const Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int shift, int width, int height)
{
  Pel *pelPred = pDst;
  const Pel *pelPlanar = pSrc0;
  Pel *pelPredAng = pSrc1;
  int offset = 1 << (shift - 1);
  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int blend = pelPlanar[x] * w0;
      blend += pelPredAng[x] * w1;
      pelPred[x] = (Pel)((blend + offset) >> shift);
    }
    pelPred += strideDst;
    pelPlanar += strideSrc0;
    pelPredAng += strideSrc1;
  }
}
#endif

#if ENABLE_DIMD

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
int IntraPrediction::deriveDimdIntraTmpModePred(const CodingUnit cu, CPelBuf predBuf)
{
  int sigcnt = 0;
  const Pel* pPred = predBuf.buf;
  const int iStride = predBuf.stride;
  int height = predBuf.height;
  int width = predBuf.width;

  int piHistogramClean[NUM_LUMA_MODE] = { 0 };

  pPred = pPred + iStride + 1;
  sigcnt += buildHistogram(pPred, iStride, height - 2, width - 2, piHistogramClean, 0, width - 2, height - 2);

  int firstAmp = 0, curAmp = 0;
  int firstMode = 0, curMode = 0;
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    curAmp = piHistogramClean[i];
    curMode = i;
    if (curAmp > firstAmp)
    {
      firstAmp = curAmp;
      firstMode = curMode;
    }
  }
  return firstMode;
}
#endif

void IntraPrediction::deriveDimdMode(const CPelBuf &recoBuf, const CompArea &area, CodingUnit &cu)
{
  if( !cu.slice->getSPS()->getUseDimd() )
  {
    return;
  }

  const CodingStructure  &cs = *cu.cs;
  const SPS             &sps = *cs.sps;
  const PreCalcValues   &pcv = *cs.pcv;
  const ChannelType   chType = toChannelType(area.compID);

  const Pel *pReco = recoBuf.buf;
  const uint32_t uiWidth = area.width;
  const uint32_t uiHeight = area.height;
  const int iStride = recoBuf.stride;
#if JVET_AC0098_LOC_DEP_DIMD
  const int predSize = uiWidth + 1;
  const int predHSize = uiHeight + 1;
#else
  const int predSize = (uiWidth << 1);
  const int predHSize = (uiHeight << 1);
#endif

  const bool noShift = pcv.noChroma2x2 && uiWidth == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth = pcv.minCUWidth >> (noShift ? 0 : getComponentScaleX(area.compID, sps.getChromaFormatIdc()));
  const int  unitHeight = pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(area.compID, sps.getChromaFormatIdc()));

  const int  totalAboveUnits = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits = std::max<int>(uiWidth / unitWidth, 1);
  const int  numLeftUnits = std::max<int>(uiHeight / unitHeight, 1);
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported");

  // ----- Step 1: analyze neighborhood -----
  const Position posLT = area;

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  int numIntraAbove = isAboveAvailable(cu, chType, posLT, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1));
  int numIntraLeft = isLeftAvailable(cu, chType, posLT, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1));
#if JVET_AC0094_REF_SAMPLES_OPT || JVET_AC0098_LOC_DEP_DIMD
  const int numIntraAboveRight = isAboveRightAvailable(cu, chType, area.topRight(), numAboveRightUnits, unitWidth, neighborFlags + totalLeftUnits + 1 + numAboveUnits);
  const int numIntraBottomLeft = isBelowLeftAvailable(cu, chType, area.bottomLeft(), numLeftBelowUnits, unitHeight, neighborFlags + totalLeftUnits - 1 - numLeftUnits);
#endif

  // ----- Step 2: build histogram of gradients -----
#if JVET_AC0098_LOC_DEP_DIMD
  int histogramTop[NUM_LUMA_MODE] = { 0 };
  int histogramLeft[NUM_LUMA_MODE] = { 0 };

  int histogramTopLeft[NUM_LUMA_MODE] = { 0 };
#endif
  int histogram[NUM_LUMA_MODE] = { 0 };

  if (numIntraLeft)
  {
#if JVET_AC0094_REF_SAMPLES_OPT || JVET_AC0098_LOC_DEP_DIMD
    const uint32_t uiHeightLeft = (numIntraLeft + numIntraBottomLeft) * unitHeight - 1 - (!numIntraAbove ? 1 : 0);
#else
    uint32_t uiHeightLeft = numIntraLeft * unitHeight - 1 - (!numIntraAbove ? 1 : 0);
#endif
    const Pel *pRecoLeft = pReco - 2 + iStride * (!numIntraAbove ? 1 : 0);
#if JVET_AC0098_LOC_DEP_DIMD
    buildHistogram(pRecoLeft, iStride, uiHeightLeft, 1, histogramLeft, 1, uiWidth, uiHeight);
#else
    buildHistogram(pRecoLeft, iStride, uiHeightLeft, 1, histogram, 1, uiWidth, uiHeight);
#endif
  }

  if (numIntraAbove)
  {
#if JVET_AC0094_REF_SAMPLES_OPT || JVET_AC0098_LOC_DEP_DIMD
    const uint32_t uiWidthAbove = (numIntraAbove + numIntraAboveRight)*unitWidth - 1 - (!numIntraLeft ? 1 : 0);
#else
    uint32_t uiWidthAbove = numIntraAbove * unitWidth - 1 - (!numIntraLeft ? 1 : 0);
#endif
    const Pel *pRecoAbove = pReco - iStride * 2 + (!numIntraLeft ? 1 : 0);
#if JVET_AC0098_LOC_DEP_DIMD
    buildHistogram(pRecoAbove, iStride, 1, uiWidthAbove, histogramTop, 2, uiWidth, uiHeight);
#else
    buildHistogram(pRecoAbove, iStride, 1, uiWidthAbove, histogram, 2, uiWidth, uiHeight);
#endif
  }

  if (numIntraLeft && numIntraAbove)
  {
    const Pel *pRecoAboveLeft = pReco - 2 - iStride * 2;
#if JVET_AC0098_LOC_DEP_DIMD
    buildHistogram(pRecoAboveLeft, iStride, 2, 2, histogramTopLeft, 3, uiWidth, uiHeight);
#else
    buildHistogram(pRecoAboveLeft, iStride, 2, 2, histogram, 3, uiWidth, uiHeight);
#endif
  }
#if JVET_AC0098_LOC_DEP_DIMD
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    histogram[i] = histogramTop[i] + histogramLeft[i] + histogramTopLeft[i];
  }
#endif

#if JVET_AB0157_INTRA_FUSION
  int amp[DIMD_FUSION_NUM-1] = { 0 };
  int curAmp         = 0;
  int mode[DIMD_FUSION_NUM-1] = { 0 };
  int curMode         = 0;
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    curAmp = histogram[i];
    curMode = i;
    for (int j = 0; j < DIMD_FUSION_NUM - 1; j++)
    {
      if (curAmp > amp[j])
      {
        for (int k = DIMD_FUSION_NUM - 2; k > j; k--)
        {
          amp[k] = amp[k - 1];
          mode[k] = mode[k - 1];
        }
        amp[j] = curAmp;
        mode[j] = curMode;
        break;
      }
    }
  }

#if JVET_AC0098_LOC_DEP_DIMD
  for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
  {
    cu.dimdLocDep[i] = 0;
  }
  for(int i = 0; i < DIMD_FUSION_NUM - 1; i++)
  {
    int secondMode = mode[i];
    if (secondMode > DC_IDX)
    {
      cu.dimdLocDep[i] = 0;

      int ampSecond = histogram[secondMode];
      int ampSecondLeft = histogramLeft[secondMode];
      int ampSecondAbove = histogramTop[secondMode];
        
      if (ampSecondLeft < (ampSecond / 3))
      {
        cu.dimdLocDep[i] = 1;
      }
      else if (ampSecondAbove < (ampSecond / 3))
      {
        cu.dimdLocDep[i] = 2;
      }
    }
  }
#endif


  cu.dimdMode = mode[0];

  cu.dimdBlending = true;
  cu.dimdBlending &= amp[1] > 0;
  cu.dimdBlending &= mode[1] > DC_IDX;
  cu.dimdBlending &= mode[0] > DC_IDX;

#if JVET_AC0098_LOC_DEP_DIMD
  if (cu.dimdLocDep[0] != 0 && amp[1] == 0)
  {
    cu.dimdBlending = true;
    cu.dimdBlending &= mode[0] > DC_IDX;
    mode[1] = 0;
    CHECK( mode[2] != 0, "Wrong logic" );
  } 
#endif

  int countBlendMode = 2;

  if( cu.dimdBlending )
  {
    for (int i = 0; i < DIMD_FUSION_NUM - 2; i++)
    {
      cu.dimdBlendMode[i] = mode[i+1];
      if (cu.dimdBlendMode[i] != PLANAR_IDX)
      {
        countBlendMode++;
      }
    }
  }

#if JVET_X0149_TIMD_DIMD_LUT
  int log2BlendWeight = 6;
  int dimdPlanarWeight = 64/4;
  int sumWeight = (1 << log2BlendWeight);
#else
  const int blendSumWeight = 6;
  int sumWeight = 1 << blendSumWeight;
#endif
  if (cu.dimdBlending)
  {
#if JVET_X0149_TIMD_DIMD_LUT
    int g_gradDivTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
#if JVET_AC0098_LOC_DEP_DIMD
    if (mode[1] == 0)
    {    
      CHECK( cu.dimdLocDep[0] == 0, "Wrong logic" );
      cu.dimdRelWeight[0] = sumWeight - 21;
      cu.dimdRelWeight[1] = 21;
      cu.dimdRelWeight[2] = 0;
      cu.dimdRelWeight[3] = 0;
      cu.dimdRelWeight[4] = 0;
      cu.dimdRelWeight[5] = 0;
    }
    else
    {
#endif
    sumWeight = sumWeight - dimdPlanarWeight;
    int s1 = 0;
    for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
    {
      s1 = s1 + amp[i];
    }
    int x = floorLog2(s1);
    CHECK(x < 0, "floor log2 value should be no negative");
    int normS1 = (s1 << 4 >> x) & 15;
    int v = g_gradDivTable[normS1] | 8;
    x += (normS1 != 0);
    int shift = x + 3;
    int add = (1 << (shift - 1));
    int iRatio[DIMD_FUSION_NUM - 1] = { 0 };
    for (int i = 0; i < DIMD_FUSION_NUM - 1; i++)
    {
      iRatio[i] = (amp[i] * v * sumWeight + add) >> shift;
      if (amp[i] == 0)
      {
        iRatio[i] = 0;
      }
      if( iRatio[i] > sumWeight )
      {
        iRatio[i] = sumWeight;
      }
      CHECK( iRatio[i] > sumWeight, "Wrong DIMD ratio" );
    }
#else
    double dRatio = 0.0;
    sumWeight -= static_cast<int>((double)sumWeight / 3);
    dRatio = (double)firstAmp / (double)(firstAmp + secondAmp);
    int iRatio = static_cast<int>(dRatio * sumWeight);
#endif
    cu.dimdRelWeight[0] = iRatio[0];
#if JVET_AC0098_LOC_DEP_DIMD
    int sumWeightReal = iRatio[0];
    int countBlendModeNew = countBlendMode;
    int lastFilled = 0;
#endif
    for (int i = 1; i < countBlendMode - 2; i++)
    {
      cu.dimdRelWeight[i+1] = iRatio[i];
#if JVET_AC0098_LOC_DEP_DIMD
      sumWeightReal += iRatio[i];
      if(sumWeightReal > sumWeight)
      {
        lastFilled = i;
        break;
      }
#endif
    }
#if JVET_AC0098_LOC_DEP_DIMD
    if(sumWeightReal > sumWeight)
    {
      for (int i = lastFilled + 1; i <= countBlendMode - 2; i++)
      {
        iRatio[i] = 0;
        cu.dimdRelWeight[i + 1] = 0;
        cu.dimdBlendMode[i - 1] = 0;
        countBlendModeNew--;  
      }
    }
    countBlendMode = countBlendModeNew;
    if (sumWeightReal > sumWeight)
    {
      int diff = sumWeightReal - sumWeight;
      for (int j = 0; j < sumWeight; j++)
      {
        for (int i = lastFilled; i >= 1; i--)
        {
          iRatio[i] -=1;
          cu.dimdRelWeight[Clip3(0, DIMD_FUSION_NUM - 1, i+1)] = iRatio[i];
          if (cu.dimdRelWeight[i+1] == 0)
          {
            cu.dimdBlendMode[i-1] = 0;
            countBlendMode--;
          }
          diff--;
          if(diff == 0)
          {
            break;
          }
        }
        if(diff == 0)
        {
          break;
        }
      }
    }
#endif
    cu.dimdRelWeight[countBlendMode-1] = sumWeight;
    for (int i = 0; i < countBlendMode - 2; i++)
    {
      cu.dimdRelWeight[countBlendMode-1] = cu.dimdRelWeight[countBlendMode-1] - iRatio[i];
#if JVET_AC0098_LOC_DEP_DIMD
      if (cu.dimdRelWeight[countBlendMode-1] == 0)
      {
        cu.dimdBlendMode[countBlendMode-3] = 0;
      }
#endif
    }
#if JVET_X0149_TIMD_DIMD_LUT
    cu.dimdRelWeight[1] = dimdPlanarWeight;
#else
    cu.dimdRelWeight[1] = (1 << blendSumWeight) - sumWeight;
#endif
#if JVET_AC0098_LOC_DEP_DIMD
    }
#endif
  }
  else
  {
    cu.dimdRelWeight[0] = sumWeight;
    for (int i = 1; i < DIMD_FUSION_NUM; i++)
    {
      cu.dimdRelWeight[i] = 0;
    }
  }
#else
  int firstAmp = 0, secondAmp = 0, curAmp = 0;
  int firstMode = 0, secondMode = 0, curMode = 0;

  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    curAmp = histogram[i];
    curMode = i;
    if (curAmp > firstAmp)
    {
      secondAmp = firstAmp;
      secondMode = firstMode;
      firstAmp = curAmp;
      firstMode = curMode;
    }
    else
    {
      if (curAmp > secondAmp)
      {
        secondAmp = curAmp;
        secondMode = curMode;
      }
    }
  }

#if JVET_AC0098_LOC_DEP_DIMD
  cu.dimdLocDep[0] = 0;
  cu.dimdLocDep[1] = 0;

  if (firstMode > DC_IDX)
  {
    cu.dimdLocDep[0] = 0;

    int ampFirst = histogram[firstMode];
    int ampFirstLeft = histogramLeft[firstMode];
    int ampFirstAbove = histogramTop[firstMode];

    if (ampFirstLeft == 0)
    {
      cu.dimdLocDep[0] = 1;
    }  
    else if (ampFirstAbove == 0)
    {
      cu.dimdLocDep[0] = 2;
    }
    else if (ampFirstLeft < (ampFirst / 3))
    {
      cu.dimdLocDep[0] = 3;
    }
    else if (ampFirstAbove < (ampFirst / 3))
    {
      cu.dimdLocDep[0] = 4;
    }
  }

  if (secondMode > DC_IDX)
  {
    cu.dimdLocDep[1] = 0;

    int ampSecond = histogram[secondMode];
    int ampSecondLeft = histogramLeft[secondMode];
    int ampSecondAbove = histogramTop[secondMode];

    if  (ampSecondLeft < (ampSecond / 3))
    {
      cu.dimdLocDep[1] = 1;
    }
    else if (ampSecondAbove < (ampSecond / 3))
    {
      cu.dimdLocDep[1] = 2;
    }
  }
#endif

  // ----- Step 3: derive best mode from histogram of gradients -----
  cu.dimdMode = firstMode;

  cu.dimdBlending = true;
  cu.dimdBlending &= secondAmp > 0;
  cu.dimdBlending &= secondMode > DC_IDX;
  cu.dimdBlending &= firstMode > DC_IDX;

#if JVET_AC0098_LOC_DEP_DIMD
  if (cu.dimdLocDep[0] != 0 && secondMode == 0)
  {
    cu.dimdBlending = true;
    cu.dimdBlending &= firstMode > DC_IDX;
    secondAmp = 0;
  } 
#endif

  if( cu.dimdBlending )
  {
    cu.dimdBlendMode[0] = secondMode;
  }

#if JVET_X0149_TIMD_DIMD_LUT
  const int log2BlendWeight = 6;
  const int planarWeight = 21;
  int sumWeight = 1 << log2BlendWeight;
#else
  const int blendSumWeight = 6;
  int sumWeight = 1 << blend_sum_weight;
#endif
  if (cu.dimdBlending)
  {
#if JVET_AC0098_LOC_DEP_DIMD
    if (secondMode == 0)
    {
      cu.dimdRelWeight[0] = sumWeight - planarWeight;
      cu.dimdRelWeight[2] = 0;
      cu.dimdRelWeight[1] = planarWeight;
    }
    else
    {
#endif
#if JVET_X0149_TIMD_DIMD_LUT
    sumWeight = sumWeight - planarWeight;
    int s0 = firstAmp;
    int s1 = firstAmp + secondAmp;
    int x = floorLog2(s1);

    CHECK(x < 0, "floor log2 value should be no negative");

    int norm = (s1 << 4 >> x) & 15;
    int v = g_gradDivTable[norm] | 8;
    x += (norm != 0);
    int shift = x + 3;
    int add = (1 << (shift - 1));
    int ratio = (s0 * v * sumWeight + add) >> shift;

    if( ratio > sumWeight )
    {
      ratio = sumWeight;
    }

    CHECK( ratio > sumWeight, "Wrong DIMD ratio" );
#else
    double dRatio = 0.0;
    sum_weight -= static_cast<int>((double)sumWeight / 3); // ~ 1/3 of the weight to be reserved for planar
    dRatio = (double)firstAmp / (double)(firstAmp + secondAmp);
    int ratio = static_cast<int>(dRatio * sumWeight);
#endif
    cu.dimdRelWeight[0] = ratio;
    cu.dimdRelWeight[2] = sumWeight - ratio;
#if JVET_X0149_TIMD_DIMD_LUT
    cu.dimdRelWeight[1] = planarWeight;
#else
    cu.dimdRelWeight[1] = (1 << blendSumWeight) - sumWeight;
#endif
#if JVET_AC0098_LOC_DEP_DIMD
    }
#endif
  }
  else
  {
    cu.dimdRelWeight[0] = sumWeight;
    cu.dimdRelWeight[1] = 0;
    cu.dimdRelWeight[2] = 0;
  }
#endif
}

#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
void IntraPrediction::deriveDimdChromaMode(const CPelBuf &recoBufY, const CPelBuf &recoBufCb, const CPelBuf &recoBufCr, const CompArea &areaY, const CompArea &areaCb, const CompArea &areaCr, CodingUnit &cu)
{
  if (!cu.slice->getSPS()->getUseDimd())
  {
    return;
  }

  const CodingStructure  &cs = *cu.cs;
  const SPS             &sps = *cs.sps;
  const PreCalcValues   &pcv = *cs.pcv;

  const Pel *pRecoY = recoBufY.buf;
  const uint32_t uiWidthY = areaY.width;
  const uint32_t uiHeightY = areaY.height;
  const int iStrideY = recoBufY.stride;

  const Pel *pRecoCb = recoBufCb.buf;
  const uint32_t uiWidthCb = areaCb.width;
  const uint32_t uiHeightCb = areaCb.height;
  const int iStrideCb = recoBufCb.stride;

  const Pel *pRecoCr = recoBufCr.buf;
  const uint32_t uiWidthCr = areaCr.width;
  const uint32_t uiHeightCr = areaCr.height;
  const int iStrideCr = recoBufCr.stride;

  // get the availability of the neighboring chroma samples
  const int predSize = (uiWidthCb << 1);
  const int predHSize = (uiHeightCb << 1);

  const bool noShift = pcv.noChroma2x2 && uiWidthCb == 4; // don't shift on the lowest level (chroma not-split)
  const int  unitWidth = pcv.minCUWidth >> (noShift ? 0 : getComponentScaleX(areaCb.compID, sps.getChromaFormatIdc()));
  const int  unitHeight = pcv.minCUHeight >> (noShift ? 0 : getComponentScaleY(areaCb.compID, sps.getChromaFormatIdc()));

  const int  totalAboveUnits = (predSize + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits = (predHSize + (unitHeight - 1)) / unitHeight;
  const int  totalUnits = totalAboveUnits + totalLeftUnits + 1; //+1 for top-left
  const int  numAboveUnits = std::max<int>(uiWidthCb / unitWidth, 1);
  const int  numLeftUnits = std::max<int>(uiHeightCb / unitHeight, 1);
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits = totalLeftUnits - numLeftUnits;

  CHECK(numAboveUnits <= 0 || numLeftUnits <= 0 || numAboveRightUnits <= 0 || numLeftBelowUnits <= 0, "Size not supported");

  const Position posLT = areaCb;

  bool  neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  Position pos = posLT.offset(0, -2); // get the availability of the third neighboring row
  int numIntraAbove = isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, pos, numAboveUnits, unitWidth, (neighborFlags + totalLeftUnits + 1));
  pos = posLT.offset(-2, 0); // get the availability of the third neighboring column
  int numIntraLeft = isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, pos, numLeftUnits, unitHeight, (neighborFlags + totalLeftUnits - 1));
#if JVET_AC0094_REF_SAMPLES_OPT
  const int numIntraAboveRight = std::min( isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, areaCb.topRight().offset(0, -2), numAboveRightUnits, unitWidth, neighborFlags + totalLeftUnits + 1 + numAboveUnits)
    , isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, areaCb.topRight(), numAboveRightUnits, unitWidth, neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  const int numIntraBottomLeft = std::min( isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, areaCb.bottomLeft().offset(-2, 0), numLeftBelowUnits, unitHeight, neighborFlags + totalLeftUnits - 1 - numLeftUnits)
    , isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, areaCb.bottomLeft(), numLeftBelowUnits, unitHeight, neighborFlags + totalLeftUnits - 1 - numLeftUnits) );
#endif

  int piHistogram[NUM_LUMA_MODE] = { 0 };
#if JVET_AC0094_REF_SAMPLES_OPT
  const uint32_t uiHeightLeftY = (numLeftUnits*unitHeight << getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, sps.getChromaFormatIdc())) - 2;
  const Pel* const pRecoLeftY = pRecoY + 1 + iStrideY;
  buildHistogram(pRecoLeftY, iStrideY, uiHeightLeftY, 2, piHistogram, 1, uiWidthY, uiHeightY);
#endif
  if (numIntraLeft)
  {
#if JVET_AC0094_REF_SAMPLES_OPT
    const uint32_t uiHeightLeftC = (numIntraLeft + numIntraBottomLeft)*unitHeight - 1 - (!numIntraAbove ? 1 : 0);
#else
    uint32_t uiHeightLeftY = ((numIntraLeft * unitHeight) << getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, sps.getChromaFormatIdc())) - 1 - (!numIntraAbove ? 1 : 0);
    uint32_t uiHeightLeftC = numIntraLeft * unitHeight - 1 - (!numIntraAbove ? 1 : 0);
#endif
#if !JVET_AC0094_REF_SAMPLES_OPT
    const Pel *pRecoLeftY = pRecoY - 2 + iStrideY * (!numIntraAbove ? 1 : 0);
#endif
    const Pel *pRecoLeftCb = pRecoCb - 2 + iStrideCb * (!numIntraAbove ? 1 : 0);
    const Pel *pRecoLeftCr = pRecoCr - 2 + iStrideCr * (!numIntraAbove ? 1 : 0);
#if !JVET_AC0094_REF_SAMPLES_OPT
    buildHistogram(pRecoLeftY, iStrideY, uiHeightLeftY, 1, piHistogram, 1, uiWidthY, uiHeightY);
#endif
    buildHistogram(pRecoLeftCb, iStrideCb, uiHeightLeftC, 1, piHistogram, 1, uiWidthCb, uiHeightCb);
    buildHistogram(pRecoLeftCr, iStrideCr, uiHeightLeftC, 1, piHistogram, 1, uiWidthCr, uiHeightCr);
  }
#if JVET_AC0094_REF_SAMPLES_OPT
  const uint32_t uiWidthAboveY = (numAboveUnits*unitWidth << getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, sps.getChromaFormatIdc())) - 4;
  const Pel* const pRecoAboveY = pRecoY + 3 + iStrideY;
  buildHistogram(pRecoAboveY, iStrideY, 2, uiWidthAboveY, piHistogram, 2, uiWidthY, uiHeightY);
#endif
  if (numIntraAbove)
  {
#if JVET_AC0094_REF_SAMPLES_OPT
    const uint32_t uiWidthAboveC = (numIntraAbove + numIntraAboveRight)*unitWidth - 1 - (!numIntraLeft ? 1 : 0);
#else
    uint32_t uiWidthAboveY = ((numIntraAbove * unitWidth) << getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, sps.getChromaFormatIdc())) - 1 - (!numIntraLeft ? 1 : 0);
    uint32_t uiWidthAboveC = numIntraAbove * unitWidth - 1 - (!numIntraLeft ? 1 : 0);
#endif
#if !JVET_AC0094_REF_SAMPLES_OPT
    const Pel *pRecoAboveY = pRecoY - iStrideY * 2 + (!numIntraLeft ? 1 : 0);
#endif
    const Pel *pRecoAboveCb = pRecoCb - iStrideCb * 2 + (!numIntraLeft ? 1 : 0);
    const Pel *pRecoAboveCr = pRecoCr - iStrideCr * 2 + (!numIntraLeft ? 1 : 0);
#if !JVET_AC0094_REF_SAMPLES_OPT
    buildHistogram(pRecoAboveY, iStrideY, 1, uiWidthAboveY, piHistogram, 2, uiWidthY, uiHeightY);
#endif
    buildHistogram(pRecoAboveCb, iStrideCb, 1, uiWidthAboveC, piHistogram, 2, uiWidthCb, uiHeightCb);
    buildHistogram(pRecoAboveCr, iStrideCr, 1, uiWidthAboveC, piHistogram, 2, uiWidthCr, uiHeightCr);
  }
  if (numIntraLeft && numIntraAbove)
  {
#if !JVET_AC0094_REF_SAMPLES_OPT
    const Pel *pRecoAboveLeftY = pRecoY - 2 - iStrideY * 2;
#endif
    const Pel *pRecoAboveLeftCb = pRecoCb - 2 - iStrideCb * 2;
    const Pel *pRecoAboveLeftCr = pRecoCr - 2 - iStrideCr * 2;
#if !JVET_AC0094_REF_SAMPLES_OPT
    buildHistogram(pRecoAboveLeftY, iStrideY, 2, 2, piHistogram, 3, uiWidthY, uiHeightY);
#endif
    buildHistogram(pRecoAboveLeftCb, iStrideCb, 2, 2, piHistogram, 3, uiWidthCb, uiHeightCb);
    buildHistogram(pRecoAboveLeftCr, iStrideCr, 2, 2, piHistogram, 3, uiWidthCr, uiHeightCr);
  }

  int firstAmp = 0, secondAmp = 0, curAmp = 0;
  int firstMode = 0, secondMode = 0, curMode = 0;

  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    curAmp = piHistogram[i];
    curMode = i;
    if (curAmp > firstAmp)
    {
      secondAmp = firstAmp;
      secondMode = firstMode;
      firstAmp = curAmp;
      firstMode = curMode;
    }
    else
    {
      if (curAmp > secondAmp)
      {
        secondAmp = curAmp;
        secondMode = curMode;
      }
    }
  }

  cu.dimdChromaMode = firstMode;
#if JVET_AC0094_REF_SAMPLES_OPT
  cu.dimdChromaModeSecond = secondMode;
#else
  int dmMode = PU::getCoLocatedIntraLumaMode(*cu.firstPU);
  if (dmMode == firstMode)
  {
    cu.dimdChromaMode = secondMode;
    if (firstMode == secondMode)
    {
      cu.dimdChromaMode = DC_IDX;
    }
  }
#endif
}
#endif

#if JVET_AB0067_MIP_DIMD_LFNST && ENABLE_DIMD
int IntraPrediction::deriveDimdMipMode(PelBuf& reducedPred, int width, int height, CodingUnit& cu)
{
  if (!cu.slice->getSPS()->getUseDimd())
  {
    return PLANAR_IDX;
  }
  const Pel* pPred = reducedPred.buf;
  const int iStride = reducedPred.stride;

  int histogram[NUM_LUMA_MODE] = { 0 };

  pPred = pPred + iStride + 1;
  buildHistogram(pPred, iStride, height - 2, width - 2, histogram, 0, width - 2, height - 2);

  int firstAmp = 0, curAmp = 0;
  int firstMode = 0, curMode = 0;
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    curAmp = histogram[i];
    curMode = i;
    if (curAmp > firstAmp)
    {
      firstAmp = curAmp;
      firstMode = curMode;
    }
  }
  return firstMode;
}
#endif

int IntraPrediction::buildHistogram(const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int* piHistogram, int direction, int bw, int bh)
{
  const int wStep = 1, hStep = 1;
  int angTable[17] = { 0, 2048, 4096, 6144, 8192, 12288, 16384, 20480, 24576, 28672, 32768, 36864, 40960, 47104, 53248, 59392, 65536 };
  int offsets[4] = { HOR_IDX, HOR_IDX, VER_IDX, VER_IDX };
  int dirs[4] = { -1, 1, -1, 1 };
  int mapXgrY1[2][2] = { { 1, 0 },{ 0, 1 } };
  int mapXgrY0[2][2] = { { 2, 3 },{ 3, 2 } };

  for (uint32_t y = 0; y < uiHeight; y += hStep)
  {
    for (uint32_t x = 0; x < uiWidth; x += wStep)
    {
      if( (direction == 3) && x == (uiWidth - 1) && y == (uiHeight - 1) )
      {
        continue;
      }

      const Pel *pRec = pReco + y * iStride + x;

      int iDy = pRec[-iStride - 1] + 2 * pRec[-1] + pRec[iStride - 1] - pRec[-iStride + 1] - 2 * pRec[+1] - pRec[iStride + 1];
      int iDx = pRec[iStride - 1] + 2 * pRec[iStride] + pRec[iStride + 1] - pRec[-iStride - 1] - 2 * pRec[-iStride] - pRec[-iStride + 1];

      if( iDy == 0 && iDx == 0 )
      {
        continue;
      }

      int iAmp = (int)(abs(iDx) + abs(iDy));
      int iAngUneven = -1;
      // for determining region
      if (iDx != 0 && iDy != 0) // pure angles are not concerned
      {
        // get the region
        int signx = iDx < 0 ? 1 : 0;
        int signy = iDy < 0 ? 1 : 0;
        int absx = iDx < 0 ? -iDx : iDx;
        int absy = iDy < 0 ? -iDy : iDy;
        int gtY = absx > absy ? 1 : 0;
        int region = gtY ? mapXgrY1[signy][signx] : mapXgrY0[signy][signx];
        //region = (region == 1 ? 2 : (region == 2 ? 1 : (region == 3 ? 4 : 3)));
#if JVET_X0149_TIMD_DIMD_LUT
        int s0 = gtY ? absy : absx;
        int s1 = gtY ? absx : absy;
        int x = floorLog2(s1);
        int norm = (s1 << 4 >> x) & 15;
        int v = g_gradDivTable[norm] | 8;
        x += (norm != 0);
        int shift = 13 - x;
        int ratio;
        if (shift < 0)
        {
          shift = -shift;
          int add = (1 << (shift - 1));
          ratio = (s0 * v + add) >> shift;
        }
        else
        {
          ratio = (s0 * v) << shift;
        }

        // iRatio after integerization can go beyond 2^16
#else
        float fRatio = gtY ? static_cast<float>(absy) / static_cast<float>(absx) : static_cast<float>(absx) / static_cast<float>(absy);
        float fRatioScaled = fRatio * (1 << 16);
        int ratio = static_cast<int>(fRatioScaled);
#endif
        // get ang_idx
        int idx = 16;
        for( int i = 1; i < 17; i++ )
        {
          if( ratio <= angTable[i] )
          {
            idx = ratio - angTable[i - 1] < angTable[i] - ratio ? i - 1 : i;
            break;
          }
        }

        iAngUneven = offsets[region] + dirs[region] * idx;
        //iAngUneven = offsets[region - 1] + dirs[region - 1] * idx;
      }
      else
      {
        iAngUneven = iDx == 0 ? VER_IDX : HOR_IDX;
      }

      CHECK( iAngUneven < 0, "Wrong mode in DIMD histogram" );
      CHECK( iAngUneven >= NUM_LUMA_MODE, "Wrong mode in DIMD histogram" );

      piHistogram[iAngUneven] += iAmp;
    }
  }
  return 0;
}

#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
void xDimdLocationdepBlending(Pel *pDst, int strideDst, Pel *pVer, int strideVer, Pel *pHor, int strideHor,Pel *pNonLocDep, int strideNonLocDep, int width, int height, int mode, int wVer, int wHor, int wNonLocDep)
{
  int maxWeight = (1 << 6);
  int weightShift = 6;
  int weightOffset = 1 << (weightShift - 1);
  int fixedRange = 10;
  int sizeThreshold = 64;
  int heightMinusOne = (height - 1);
  int widthMinusOne = (width - 1);
  if (mode == 0) // diagonal blending
  {
    int clipRangeVer = 64;
    int clipRangeHor = 64;
    {
      int totRange = wNonLocDep;
      int totDirWeight = wVer + wHor;
      clipRangeVer = wVer +  (int)(((double)wVer / (double)totDirWeight)* (double)totRange);
      clipRangeHor = wHor +  (int)(((double)wHor / (double)totDirWeight)* (double)totRange);
    }
    int rangeVer = fixedRange;
    int rangeHor = fixedRange;  

    if(height > sizeThreshold) 
    {
      rangeVer *= 2;
    }
    if(width > sizeThreshold)
    {
      rangeHor *= 2;
    }

    bool needClipVer = (((wVer + rangeVer) > clipRangeVer) || ((wVer - rangeVer) < 0));
    bool needClipHor = (((wHor + rangeHor) > clipRangeHor) || ((wHor - rangeHor) < 0));
    if(needClipVer && needClipHor)
    {
      for (int y = 0; y < height; y++)
      {
        int weightVer = Clip3(0, clipRangeVer, (((wVer + rangeVer) * heightMinusOne - ((rangeVer*y) << 1)) / heightMinusOne));
        for (int x = 0; x < width; x++)
        {
          int weightHor =  Clip3(0, clipRangeHor, (((wHor + rangeHor) * widthMinusOne - ((rangeHor*x) << 1)) / widthMinusOne));
          int weightNonLocDep =  maxWeight - weightVer - weightHor;
          int blend = (int)pVer[x]  * weightVer + (int)pHor[x]  * weightHor + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pVer += strideVer;
        pHor += strideHor;
        pNonLocDep += strideNonLocDep;
      }
    }
    else if (needClipVer)
    {
      for (int y = 0; y < height; y++)
      {
        int weightVer = Clip3(0, clipRangeVer, (((wVer + rangeVer) * heightMinusOne - ((rangeVer*y) << 1)) / heightMinusOne));
        for (int x = 0; x < width; x++)
        {
          int weightHor =  (((wHor + rangeHor) * widthMinusOne - ((rangeHor*x) << 1)) / widthMinusOne);
          int weightNonLocDep =  maxWeight - weightVer - weightHor;
          int blend = (int)pVer[x]  * weightVer + (int)pHor[x]  * weightHor + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pVer += strideVer;
        pHor += strideHor;
        pNonLocDep += strideNonLocDep;
      }
    }
    else if(needClipHor)
    {
      for (int y = 0; y < height; y++)
      {
        int weightVer =  (((wVer + rangeVer) * heightMinusOne - ((rangeVer*y) << 1)) / heightMinusOne);
        for (int x = 0; x < width; x++)
        {
          int weightHor =  Clip3(0, clipRangeHor, (((wHor + rangeHor) * widthMinusOne - ((rangeHor*x) << 1)) / widthMinusOne));
          int weightNonLocDep =  maxWeight - weightVer - weightHor;
          int blend = (int)pVer[x]  * weightVer + (int)pHor[x]  * weightHor + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pVer += strideVer;
        pHor += strideHor;
        pNonLocDep += strideNonLocDep;
      }
    }
    else
    {
      for (int y = 0; y < height; y++)
      {
        int weightVer =  (((wVer + rangeVer) * heightMinusOne - ((rangeVer*y) << 1)) / heightMinusOne);
        for (int x = 0; x < width; x++)
        {
          int weightHor =  (((wHor + rangeHor) * widthMinusOne - ((rangeHor*x) << 1)) / widthMinusOne);
          int weightNonLocDep =  maxWeight - weightVer - weightHor;
          int blend = (int)pVer[x]  * weightVer + (int)pHor[x]  * weightHor + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pVer += strideVer;
        pHor += strideHor;
        pNonLocDep += strideNonLocDep;
      }
    }
  }
  else if(mode == 1) //ver blending
  {
    int clipRangeVer = 64; 
    int rangeVer = fixedRange;

    if(height > sizeThreshold)
    {
      rangeVer *= 2;
    }
    bool needClipVer = (((wVer + rangeVer) > clipRangeVer) || ((wVer - rangeVer) < 0));

    if (needClipVer)
    {
      for (int y = 0; y < height; y++)
      {
        int weightVer = Clip3(0, clipRangeVer, (((wVer + rangeVer) * heightMinusOne - ((rangeVer*y) << 1)) / heightMinusOne));
        for (int x = 0; x < width; x++)
        {
          int weightNonLocDep =  maxWeight - weightVer;
          int blend = (int)pVer[x]  * weightVer + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pVer += strideVer;
        pNonLocDep += strideNonLocDep;
      }
    }
    else
    {
      for (int y = 0; y < height; y++)
      {
        int weightVer = (((wVer + rangeVer) * heightMinusOne - ((rangeVer*y) << 1)) / heightMinusOne);
        for (int x = 0; x < width; x++)
        {
          int weightNonLocDep =  maxWeight - weightVer;
          int blend = (int)pVer[x]  * weightVer + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pVer += strideVer;
        pNonLocDep += strideNonLocDep;
      }
    }
  }
  else// if(mode == 2) //ver blending
  {
    int clipRangeHor = 64; 
    int rangeHor = fixedRange;

    if(width > sizeThreshold)
    {
      rangeHor *= 2;
    }
    bool needClipHor = (((wHor + rangeHor) > clipRangeHor) || ((wHor - rangeHor) < 0));

    if (needClipHor)
    {
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          int weightHor =  Clip3(0, clipRangeHor, (((wHor + rangeHor) * widthMinusOne - ((rangeHor*x) << 1)) / widthMinusOne));
          int weightNonLocDep =  maxWeight - weightHor;
          int blend = (int)pHor[x]  * weightHor + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pHor += strideHor;
        pNonLocDep += strideNonLocDep;
      }
    }
    else
    {
      for (int y = 0; y < height; y++)
      {
        for (int x = 0; x < width; x++)
        {
          int weightHor =  (((wHor + rangeHor) * widthMinusOne - ((rangeHor*x) << 1)) / widthMinusOne);
          int weightNonLocDep =  maxWeight - weightHor;
          int blend = (int)pHor[x]  * weightHor + (int)pNonLocDep[x]  * weightNonLocDep + weightOffset;
          pDst[x] = (Pel)(blend >> weightShift);
        }
        pDst += strideDst;
        pHor += strideHor;
        pNonLocDep += strideNonLocDep;
      }
    }
  }
}
#else
void xDimdLocationdepBlending(Pel *pDst, int strideDst, Pel *pMainAng, int strideMainAng, Pel *pSecondAng, int strideSecondAng,Pel *pPlanar, int stridePlanar, int width, int height, int sideMain, int sideSecond, int wMain, int wSecond, int wPlanar)
{
  int smallerWeightsTh = 2;

  int maxWeight = (1 << 6);
  int weightShift = 6;
  int weightOffset = 32;

  if(sideMain != 0 && sideSecond != 0)
  {
    wPlanar += 8;
    wMain -= 4;
    wSecond = 64 - wMain - wPlanar;
  }

  int weightMain = wMain;
  int weightSecond = wSecond; 
  int weightPlanar = wPlanar; 

  int mainRange = 0, secondRange = 0;
  

  if(sideMain != 0)
  {
    int tempRange = (sideSecond == 0 ? 20 : 15);
    mainRange = (weightMain < tempRange) ? weightMain : tempRange;
    if(sideMain > smallerWeightsTh)
    {
      int tempRange = (sideSecond == 0 ? 10 : 5);
      mainRange = (weightMain < tempRange) ? weightMain : tempRange;
      sideMain -= smallerWeightsTh;
    }
    mainRange = Clip3(0, (64 - wMain), mainRange);
  }
  if(sideSecond != 0)
  {
    int tempRange = (sideMain == 0 ? 10 : 5);
    secondRange = (weightSecond < tempRange) ? weightSecond : tempRange;
    if(sideSecond > smallerWeightsTh)
    {
      sideSecond -= smallerWeightsTh;
    }
    secondRange = Clip3(0, (64 - wSecond), secondRange);
  }
  int blend;
    
  if(sideMain == 1 && sideSecond == 0)
  {
    for (int y = 0; y < height; y++)
    {
      weightMain = ((wMain + mainRange) * (height - 1) - ((mainRange*y) << 1)) / (height - 1);
      weightPlanar = wPlanar + (( wMain - weightMain ) >> 1);
      weightSecond = maxWeight - weightMain - weightPlanar;
      for (int x = 0; x < width; x++)
      {
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 2 && sideSecond == 0)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        weightMain = ((wMain + mainRange) * (width - 1) - ((mainRange*x) << 1)) / (width - 1);
        weightPlanar = wPlanar + (( wMain - weightMain ) >> 1);
        weightSecond = maxWeight - weightMain - weightPlanar;
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 0 && sideSecond == 1)
  {
    for (int y = 0; y < height; y++)
    {
      weightSecond = ((wSecond + secondRange) * (height - 1) - ((secondRange*y) << 1)) / (height - 1);
      weightPlanar = wPlanar + (( wSecond - weightSecond ) >> 1);
      weightMain = maxWeight - weightSecond - weightPlanar;
      for (int x = 0; x < width; x++)
      {
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 0 && sideSecond == 2)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        weightSecond = ((wSecond + secondRange) * (width - 1) - ((secondRange*x) << 1)) / (width - 1);
        weightPlanar = wPlanar + (( wSecond - weightSecond ) >> 1);
        weightMain = maxWeight - weightSecond - weightPlanar;
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 1 && sideSecond == 1)
  {
    for (int y = 0; y < height; y++)
    {
      weightMain = ((wMain + mainRange) * (height - 1) - ((mainRange*y) << 1)) / (height - 1);
      weightSecond = ((wSecond + secondRange) * (height - 1) - ((secondRange*y) << 1)) / (height - 1);
      weightPlanar =  maxWeight - weightSecond - weightMain;
      for (int x = 0; x < width; x++)
      {
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 1 && sideSecond == 2)
  {
    for (int y = 0; y < height; y++)
    {
      weightMain = ((wMain + mainRange) * (height - 1) - ((mainRange*y) << 1)) / (height - 1);
      for (int x = 0; x < width; x++)
      {
        weightSecond = ((wSecond + secondRange) * (width - 1) - ((secondRange*x) << 1)) / (width - 1);
        weightPlanar =  maxWeight - weightSecond - weightMain;
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 2 && sideSecond == 1)
  {
    for (int y = 0; y < height; y++)
    {
      weightSecond = ((wSecond + secondRange) * (height - 1) - ((secondRange*y) << 1)) / (height - 1);
      for (int x = 0; x < width; x++)
      {
        weightMain = ((wMain + mainRange) * (width - 1) - ((mainRange*x) << 1)) / (width - 1);
        weightPlanar =  maxWeight - weightSecond - weightMain;
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
  else if(sideMain == 2 && sideSecond == 2)
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x++)
      {
        weightMain = ((wMain + mainRange) * (width - 1) - ((mainRange*x) << 1)) / (width - 1);
        weightSecond = ((wSecond + secondRange) * (width - 1) - ((secondRange*x) << 1)) / (width - 1);
        weightPlanar =  maxWeight - weightSecond - weightMain;
        blend = (int)pMainAng[x]  * weightMain + (int)pSecondAng[x]  * weightSecond + (int)pPlanar[x]  * weightPlanar + weightOffset;
        pDst[x] = (Pel)(blend >> weightShift);
      }
      pDst += strideDst;
      pMainAng += strideMainAng;
      pSecondAng += strideSecondAng;
      pPlanar += stridePlanar;
    }
  }
}
#endif
#endif

#if INTRA_TRANS_ENC_OPT
void IntraPrediction::dimdBlending(Pel *pDst, int strideDst, Pel *pSrc0, int strideSrc0, Pel *pSrc1, int strideSrc1, int w0, int w1, int w2, int width, int height)
{
  Pel *pelPred = pDst;
  Pel *pelPlanar = pSrc0;
  Pel *pelPredAng = pSrc1;

  for (int y = 0; y < height; y++)
  {
    for (int x = 0; x < width; x++)
    {
      int blend = pelPred[x] * w0;
      blend += pelPlanar[x] * w1;
      blend += pelPredAng[x] * w2;
      pelPred[x] = (Pel)(blend >> 6);
    }

    pelPred += strideDst;
    pelPlanar += strideSrc0;
    pelPredAng += strideSrc1;
  }
}
#endif
#endif
#if JVET_AD0120_LBCCP
bool isPosAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &refPos)
{
  const CodingStructure& cs = *cu.cs;

  if (!cs.isDecomp(refPos, chType))
  {
    return false;
  }

  return (cs.getCURestricted(refPos, cu, chType) != NULL);
}
#endif
bool isAboveLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT)
{
  const CodingStructure& cs = *cu.cs;
  const Position refPos = posLT.offset(-1, -1);

  if (!cs.isDecomp(refPos, chType))
  {
    return false;
  }

  return (cs.getCURestricted(refPos, cu, chType) != NULL);
}

int isAboveAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDx      = uiNumUnitsInPU * unitWidth;

  for (int dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posLT.offset(dx, -1);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags++;
  }

  return numIntra;
}

int isLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLT, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags)
{
  const CodingStructure& cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDy      = uiNumUnitsInPU * unitHeight;

  for (int dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLT.offset(-1, dy);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags--;
  }

  return numIntra;
}

int isAboveRightAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posRT, const uint32_t uiNumUnitsInPU, const uint32_t unitWidth, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDx      = uiNumUnitsInPU * unitWidth;

  for (int dx = 0; dx < maxDx; dx += unitWidth)
  {
    const Position refPos = posRT.offset(unitWidth + dx, -1);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags++;
  }

  return numIntra;
}

int isBelowLeftAvailable(const CodingUnit &cu, const ChannelType &chType, const Position &posLB, const uint32_t uiNumUnitsInPU, const uint32_t unitHeight, bool *bValidFlags )
{
  const CodingStructure& cs = *cu.cs;

  bool *    validFlags = bValidFlags;
  int       numIntra   = 0;
  const int maxDy      = uiNumUnitsInPU * unitHeight;

  for (int dy = 0; dy < maxDy; dy += unitHeight)
  {
    const Position refPos = posLB.offset(-1, unitHeight + dy);

    if (!cs.isDecomp(refPos, chType))
    {
      break;
    }

    const bool valid = (cs.getCURestricted(refPos, cu, chType) != NULL);
    numIntra += valid ? 1 : 0;
    *validFlags = valid;

    validFlags--;
  }

  return numIntra;
}

#if JVET_AA0126_GLM
Pel IntraPrediction::xGlmGetLumaVal(const int s[6], const int c[6], const int glmIdx, const Pel val) const
{
  Pel grad = c[0] * s[0] + c[1] * s[1] + c[2] * s[2] 
           + c[3] * s[3] + c[4] * s[4] + c[5] * s[5];
#if NUM_GLM_WEIGHT && !JVET_AB0092_GLM_WITH_LUMA
  return (glmIdx >= NUM_GLM_PATTERN ? val + grad : grad);
#else
  return grad;
#endif
}

void IntraPrediction::xGetLumaRecPixelsGlmAll(const PredictionUnit &pu, CompArea chromaArea)
{
#if JVET_AB0092_GLM_WITH_LUMA
  const CPelBuf recoLuma = pu.cs->picture->getRecoBuf(COMPONENT_Y);
  const int  maxPosPicX = pu.cs->picture->chromaSize().width - 1;
  const int  maxPosPicY = pu.cs->picture->chromaSize().height - 1;

  xGlmCalcRefArea(pu, chromaArea); // Find the reference area

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refGrad[NUM_GLM_IDC];
  for (int i = 0; i < NUM_GLM_IDC; i++)
  {
    refGrad[i] = xGlmGetGradRefBuf(pu, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, i);
  }

  int puBorderX = refSizeX + chromaArea.width;
  int puBorderY = refSizeY + chromaArea.height;

  // Generate down-sampled luma and luma gradients
  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if ((x >= puBorderX && y >= refSizeY) || (y >= puBorderY && x >= refSizeX))
      {
        continue;
      }

      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      for (int i = 0; i < NUM_GLM_IDC; i++)
      {
        refGrad[i].at(x, y) = xGlmGetGradVal(pu, i, recoLuma, chromaPosPicX, chromaPosPicY);
      }
    }
  }
#endif
  int c[6] = { 0 };

  int iDstStride = 2 * MAX_CU_SIZE + 1;
  Pel* pDst0Cb[NUM_GLM_IDC];
  Pel* pDst0Cr[NUM_GLM_IDC];

  for (int k = 0; k < NUM_GLM_IDC; k++)
  {
    pDst0Cb[k] = m_glmTempCb[k] + iDstStride + 1;
    pDst0Cr[k] = m_glmTempCr[k] + iDstStride + 1;
  }

  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(), recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)

  CHECK(lumaArea.width == chromaArea.width && CHROMA_444 != pu.chromaFormat, "");
  CHECK(lumaArea.height == chromaArea.height && CHROMA_444 != pu.chromaFormat && CHROMA_422 != pu.chromaFormat, "");

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  int iRecStride        = Src.stride;
  int logSubWidthC  = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
  int logSubHeightC = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);

  int iRecStride2       = iRecStride << logSubHeightC;

  const CodingUnit& lumaCU = isChroma( pu.chType ) ? *pu.cs->picture->cs->getCU( lumaArea.pos(), CH_L ) : *pu.cu;
  const CodingUnit&     cu = *pu.cu;

  const CompArea& area = isChroma( pu.chType ) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  const int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const int  iUnitHeight = iBaseUnitSize >> getComponentScaleY(area.compID, area.chromaFormat);

  const int  iTUWidthInUnits = uiTuWidth / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
  const int  chromaUnitWidth = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, area.chromaFormat);
  const int  chromaUnitHeight = iBaseUnitSize >> getComponentScaleY(COMPONENT_Cb, area.chromaFormat);
  const int  topTemplateSampNum = 2 * uiCWidth; // for MDLM, the number of template samples is 2W or 2H.
  const int  leftTemplateSampNum = 2 * uiCHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  const int  totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth;
  const int  totalLeftUnits = (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight;
  const int  totalUnits = totalLeftUnits + totalAboveUnits + 1;
  const int  aboveRightUnits = totalAboveUnits - iAboveUnits;
  const int  leftBelowUnits = totalLeftUnits - iLeftUnits;

  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(bNeighborFlags, 0, totalUnits);
  bool aboveIsAvailable, leftIsAvailable;

  int availlableUnit = isLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                       iLeftUnits, iUnitHeight, (bNeighborFlags + iLeftUnits + leftBelowUnits - 1));

  leftIsAvailable = availlableUnit == iTUHeightInUnits;

  availlableUnit = isAboveAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                    iAboveUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + 1));

  aboveIsAvailable = availlableUnit == iTUWidthInUnits;

  if (leftIsAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.bottomLeftComp(area.compID), leftBelowUnits, iUnitHeight, (bNeighborFlags + leftBelowUnits - 1));
  }

  if (aboveIsAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.topRightComp(area.compID), aboveRightUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1));
  }

  Pel*       pDstCb[NUM_GLM_IDC];
  Pel*       pDstCr[NUM_GLM_IDC];
  Pel const* piSrc = nullptr;

  if (aboveIsAvailable)
  {
    for (int k = 0; k < NUM_GLM_IDC; k++)
    {
      pDstCb[k]  = pDst0Cb[k]    - iDstStride;
      pDstCr[k]  = pDst0Cr[k]    - iDstStride;
    }

    int addedAboveRight = avaiAboveRightUnits*chromaUnitWidth;

    for (int i = 0; i < uiCWidth + addedAboveRight; i++)
    {
      const int l = (i == 0 && !leftIsAvailable) ? 0 : 1;
      {
        piSrc = pRecSrc0 - iRecStride2;
        int s[6] = { piSrc[2 * i              - l], piSrc[2 * i                 ], piSrc[2 * i              + 1],
                     piSrc[2 * i + iRecStride - l], piSrc[2 * i + iRecStride    ], piSrc[2 * i + iRecStride + 1] };
        int val = (1 * s[0] + 2 * s[1] + 1 * s[2] 
                 + 1 * s[3] + 2 * s[4] + 1 * s[5] + 4) >> 3;
        pDstCb[0][i] = val;
        pDstCr[0][i] = val;

        for (int k = 1; k < NUM_GLM_IDC; k++)
        {
          int p = (k - 1 < NUM_GLM_PATTERN) ? (k - 1) : (k - 1 - NUM_GLM_PATTERN);
          c[0] = g_glmPattern[p][0], c[1] = g_glmPattern[p][1], c[2] = g_glmPattern[p][2];
          c[3] = g_glmPattern[p][3], c[4] = g_glmPattern[p][4], c[5] = g_glmPattern[p][5];
          int grad = c[0] * s[0] + c[1] * s[1] + c[2] * s[2] 
                   + c[3] * s[3] + c[4] * s[4] + c[5] * s[5];
          pDstCb[k][i] = (k - 1 < NUM_GLM_PATTERN) ? grad : val + grad;
          pDstCr[k][i] = (k - 1 < NUM_GLM_PATTERN) ? grad : val + grad;
        }
      }
    }
  }

  if (leftIsAvailable)
  {
    for (int k = 0; k < NUM_GLM_IDC; k++)
    {
      pDstCb[k]  = pDst0Cb[k]    - 1;
      pDstCr[k]  = pDst0Cr[k]    - 1;
    }
    piSrc = pRecSrc0 - 1 - logSubWidthC;

    int addedLeftBelow = avaiLeftBelowUnits*chromaUnitHeight;

    for (int j = 0; j < uiCHeight + addedLeftBelow; j++)
    {
      {
        int s[6] = { piSrc[           - 1], piSrc[             0], piSrc[             1],
                     piSrc[iRecStride - 1], piSrc[iRecStride    ], piSrc[iRecStride + 1] };
        int val = (1 * s[0] + 2 * s[1] + 1 * s[2] 
                 + 1 * s[3] + 2 * s[4] + 1 * s[5] + 4) >> 3;
        pDstCb[0][0] = val;
        pDstCr[0][0] = val;

        for (int k = 1; k < NUM_GLM_IDC; k++)
        {
          int p = (k - 1 < NUM_GLM_PATTERN) ? (k - 1) : (k - 1 - NUM_GLM_PATTERN);
          c[0] = g_glmPattern[p][0], c[1] = g_glmPattern[p][1], c[2] = g_glmPattern[p][2];
          c[3] = g_glmPattern[p][3], c[4] = g_glmPattern[p][4], c[5] = g_glmPattern[p][5];
          int grad = c[0] * s[0] + c[1] * s[1] + c[2] * s[2] 
                   + c[3] * s[3] + c[4] * s[4] + c[5] * s[5];
          pDstCb[k][0] = (k - 1 < NUM_GLM_PATTERN) ? grad : val + grad;
          pDstCr[k][0] = (k - 1 < NUM_GLM_PATTERN) ? grad : val + grad;
        }
      }

      piSrc += iRecStride2;
      for (int k = 0; k < NUM_GLM_IDC; k++)
      {
        pDstCb[k] += iDstStride;
        pDstCr[k] += iDstStride;
      }
    }
  }

  // inner part from reconstructed picture buffer
  for( int j = 0; j < uiCHeight; j++ )
  {
    for( int i = 0; i < uiCWidth; i++ )
    {
      const int l = (i == 0 && !leftIsAvailable) ? 0 : 1;
      {
        int s[6] = { pRecSrc0[2 * i              - l], pRecSrc0[2 * i                 ], pRecSrc0[2 * i              + 1],
                     pRecSrc0[2 * i + iRecStride - l], pRecSrc0[2 * i + iRecStride    ], pRecSrc0[2 * i + iRecStride + 1] };
        int val = (1 * s[0] + 2 * s[1] + 1 * s[2] 
                 + 1 * s[3] + 2 * s[4] + 1 * s[5] + 4) >> 3;
        pDst0Cb[0][i] = val;
        pDst0Cr[0][i] = val;

        for (int k = 1; k < NUM_GLM_IDC; k++)
        {
          int p = (k - 1 < NUM_GLM_PATTERN) ? (k - 1) : (k - 1 - NUM_GLM_PATTERN);
          c[0] = g_glmPattern[p][0], c[1] = g_glmPattern[p][1], c[2] = g_glmPattern[p][2];
          c[3] = g_glmPattern[p][3], c[4] = g_glmPattern[p][4], c[5] = g_glmPattern[p][5];
          int grad = c[0] * s[0] + c[1] * s[1] + c[2] * s[2] 
                   + c[3] * s[3] + c[4] * s[4] + c[5] * s[5];
          pDst0Cb[k][i] = (k - 1 < NUM_GLM_PATTERN) ? grad : val + grad;
          pDst0Cr[k][i] = (k - 1 < NUM_GLM_PATTERN) ? grad : val + grad;
        }
      }
    }

    for (int k = 0; k < NUM_GLM_IDC; k++)
    {
      pDst0Cb[k]    += iDstStride;
      pDst0Cr[k]    += iDstStride;
    }
    pRecSrc0 += iRecStride2;
  }
}
#endif

// LumaRecPixels
void IntraPrediction::xGetLumaRecPixels(const PredictionUnit &pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
  , int downsFilterIdx
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  , bool isTemplate
#endif
)
{
#if JVET_AF0073_INTER_CCP_MERGE
  if(!isTemplate)
  {
#endif
#if JVET_AA0057_CCCM
  if ( pu.cccmFlag )
  {
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
    if( pu.cccmNoSubFlag )
    {
      xCccmCreateLumaNoSubRef( pu, chromaArea );
      return;
    }
#endif
    xCccmCreateLumaRef(pu, chromaArea
#if JVET_AD0202_CCCM_MDF
      , downsFilterIdx
#endif
    );
#if JVET_AE0100_BVGCCCM
    if (pu.bvgCccmFlag)
    {
      CHECK(downsFilterIdx > 0, "downsFilterIdx in bvgCccm must be zero!");
      xBvgCccmCreateLumaRef(pu, chromaArea
#if JVET_AD0202_CCCM_MDF
      , downsFilterIdx
#endif
                            );
    }
#endif
    return;
  }
#endif

#if JVET_AB0092_GLM_WITH_LUMA
  if (pu.glmIdc.getIdc(chromaArea.compID, 0) > NUM_GLM_PATTERN)
  {
    xGlmCreateGradRef(pu, chromaArea);
    return;
  }
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
  if (pu.isChromaFusion > 1)
  {
    xCflmCreateLumaRef(pu, chromaArea);
    return;
  }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  }
#if JVET_AB0092_GLM_WITH_LUMA
  else
  {
    if (pu.glmIdc.getIdc(chromaArea.compID, 0) > NUM_GLM_PATTERN)
    {
      xGlmCreateGradRef(pu, chromaArea, true);
      return;
    }
  }
#endif
#endif

#if JVET_AA0126_GLM
  ComponentID compID = chromaArea.compID;
  int glmIdc = pu.glmIdc.getIdc(compID, 0);
  int c[6] = { 0 };
  CHECK(glmIdc < 0 || glmIdc >= NUM_GLM_IDC, "glmIdc out of range");

  if (glmIdc != 0)
  {
    int glmIdx = glmIdc - 1;
    int p = glmIdx >= NUM_GLM_PATTERN ? glmIdx - NUM_GLM_PATTERN : glmIdx;
    CHECK(p < 0 || p >= NUM_GLM_PATTERN, "glmPattern out of range");
    c[0] = g_glmPattern[p][0], c[1] = g_glmPattern[p][1], c[2] = g_glmPattern[p][2];
    c[3] = g_glmPattern[p][3], c[4] = g_glmPattern[p][4], c[5] = g_glmPattern[p][5];
  }
#endif

  int iDstStride = 0;
  Pel* pDst0 = 0;
  int curChromaMode = pu.intraDir[1];
#if JVET_AA0126_GLM
  if (pu.glmIdc.isActive())
  {
    iDstStride = 2 * MAX_CU_SIZE + 1;
    Pel* glmTemp = compID == COMPONENT_Cb ? m_glmTempCb[glmIdc] : m_glmTempCr[glmIdc];
    pDst0 = glmTemp + iDstStride + 1;
  }
  else
  {
#endif
#if MMLM
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX) || (curChromaMode == MMLM_L_IDX) || (curChromaMode == MMLM_T_IDX))
#else
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
#endif
  {
    iDstStride = 2 * MAX_CU_SIZE + 1;
    pDst0 = m_pMdlmTemp + iDstStride + 1;
  }
  else
  {
    iDstStride = MAX_CU_SIZE + 1;
    pDst0 = m_piTemp + iDstStride + 1; //MMLM_SAMPLE_NEIGHBOR_LINES;
  }
#if JVET_AA0126_GLM
  }
#endif
  //assert 420 chroma subsampling
  CompArea lumaArea = CompArea( COMPONENT_Y, pu.chromaFormat, chromaArea.lumaPos(), recalcSize( pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, chromaArea.size() ) );//needed for correct pos/size (4x4 Tus)

  CHECK(lumaArea.width == chromaArea.width && CHROMA_444 != pu.chromaFormat, "");
  CHECK(lumaArea.height == chromaArea.height && CHROMA_444 != pu.chromaFormat && CHROMA_422 != pu.chromaFormat, "");

  const SizeType uiCWidth = chromaArea.width;
  const SizeType uiCHeight = chromaArea.height;

  const CPelBuf Src = pu.cs->picture->getRecoBuf( lumaArea );
  Pel const* pRecSrc0   = Src.bufAt( 0, 0 );
  int iRecStride        = Src.stride;
  int logSubWidthC  = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
  int logSubHeightC = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);

  int iRecStride2       = iRecStride << logSubHeightC;

  const CodingUnit& lumaCU = isChroma( pu.chType ) ? *pu.cs->picture->cs->getCU( lumaArea.pos(), CH_L ) : *pu.cu;
  const CodingUnit&     cu = *pu.cu;

  const CompArea& area = isChroma( pu.chType ) ? chromaArea : lumaArea;

  const uint32_t uiTuWidth  = area.width;
  const uint32_t uiTuHeight = area.height;

  int iBaseUnitSize = ( 1 << MIN_CU_LOG2 );

  const int  iUnitWidth       = iBaseUnitSize >> getComponentScaleX( area.compID, area.chromaFormat );
  const int  iUnitHeight = iBaseUnitSize >> getComponentScaleY(area.compID, area.chromaFormat);

  const int  iTUWidthInUnits = uiTuWidth / iUnitWidth;
  const int  iTUHeightInUnits = uiTuHeight / iUnitHeight;
  const int  iAboveUnits      = iTUWidthInUnits;
  const int  iLeftUnits       = iTUHeightInUnits;
  const int  chromaUnitWidth = iBaseUnitSize >> getComponentScaleX(COMPONENT_Cb, area.chromaFormat);
  const int  chromaUnitHeight = iBaseUnitSize >> getComponentScaleY(COMPONENT_Cb, area.chromaFormat);
  const int  topTemplateSampNum = 2 * uiCWidth; // for MDLM, the number of template samples is 2W or 2H.
  const int  leftTemplateSampNum = 2 * uiCHeight;
#if JVET_AF0073_INTER_CCP_MERGE
  assert((pu.cu->firstTU->interCcpMerge) || (m_topRefLength >= topTemplateSampNum));
  assert((pu.cu->firstTU->interCcpMerge) || (m_leftRefLength >= leftTemplateSampNum));
#else
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
#endif
  const int  totalAboveUnits = (topTemplateSampNum + (chromaUnitWidth - 1)) / chromaUnitWidth;
  const int  totalLeftUnits = (leftTemplateSampNum + (chromaUnitHeight - 1)) / chromaUnitHeight;
  const int  totalUnits = totalLeftUnits + totalAboveUnits + 1;
  const int  aboveRightUnits = totalAboveUnits - iAboveUnits;
  const int  leftBelowUnits = totalLeftUnits - iLeftUnits;

  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  bool  bNeighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(bNeighborFlags, 0, totalUnits);
  bool aboveIsAvailable, leftIsAvailable;

  int availlableUnit = isLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                       iLeftUnits, iUnitHeight, (bNeighborFlags + iLeftUnits + leftBelowUnits - 1));

  leftIsAvailable = availlableUnit == iTUHeightInUnits;

  availlableUnit = isAboveAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.pos(),
                                    iAboveUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + 1));

  aboveIsAvailable = availlableUnit == iTUWidthInUnits;

  if (leftIsAvailable)   // if left is not available, then the below left is not available
  {
    avaiLeftBelowUnits = isBelowLeftAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.bottomLeftComp(area.compID), leftBelowUnits, iUnitHeight, (bNeighborFlags + leftBelowUnits - 1));
  }

  if (aboveIsAvailable)   // if above is not available, then  the above right is not available.
  {
    avaiAboveRightUnits = isAboveRightAvailable(isChroma(pu.chType) ? cu : lumaCU, toChannelType(area.compID), area.topRightComp(area.compID), aboveRightUnits, iUnitWidth, (bNeighborFlags + iLeftUnits + leftBelowUnits + iAboveUnits + 1));
  }

  Pel*       pDst  = nullptr;
  Pel const* piSrc = nullptr;

  bool isFirstRowOfCtu = (lumaArea.y & ((pu.cs->sps)->getCTUSize() - 1)) == 0;

  if (aboveIsAvailable)
  {
    pDst  = pDst0    - iDstStride;
    int addedAboveRight = 0;
#if MMLM
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX) || (curChromaMode == MMLM_L_IDX) || (curChromaMode == MMLM_T_IDX))
#else
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
#endif
    {
      addedAboveRight = avaiAboveRightUnits*chromaUnitWidth;
    }
    for (int i = 0; i < uiCWidth + addedAboveRight; i++)
    {
      const bool leftPadding = i == 0 && !leftIsAvailable;
      if (pu.chromaFormat == CHROMA_444)
      {
        piSrc = pRecSrc0 - iRecStride;
        pDst[i] = piSrc[i];
      }
      else if (isFirstRowOfCtu)
      {
        piSrc   = pRecSrc0 - iRecStride;
        pDst[i] = (piSrc[2 * i] * 2 + piSrc[2 * i - (leftPadding ? 0 : 1)] + piSrc[2 * i + 1] + 2) >> 2;
      }
      else if (pu.chromaFormat == CHROMA_422)
      {
        piSrc = pRecSrc0 - iRecStride2;

        int s = 2;
        s += piSrc[2 * i] * 2;
        s += piSrc[2 * i - (leftPadding ? 0 : 1)];
        s += piSrc[2 * i + 1];
        pDst[i] = s >> 2;
      }
      else if (pu.cs->sps->getCclmCollocatedChromaFlag())
      {
        piSrc = pRecSrc0 - iRecStride2;

        int s = 4;
        s += piSrc[2 * i - iRecStride];
        s += piSrc[2 * i] * 4;
        s += piSrc[2 * i - (leftPadding ? 0 : 1)];
        s += piSrc[2 * i + 1];
        s += piSrc[2 * i + iRecStride];
        pDst[i] = s >> 3;
      }
      else
      {
        piSrc = pRecSrc0 - iRecStride2;
        int s = 4;
        s += piSrc[2 * i] * 2;
        s += piSrc[2 * i + 1];
        s += piSrc[2 * i - (leftPadding ? 0 : 1)];
        s += piSrc[2 * i + iRecStride] * 2;
        s += piSrc[2 * i + 1 + iRecStride];
        s += piSrc[2 * i + iRecStride - (leftPadding ? 0 : 1)];
        pDst[i] = s >> 3;
      }
#if JVET_AA0126_GLM
      if (glmIdc != 0)
      {
        piSrc = pRecSrc0 - iRecStride2;
        int l = leftPadding ? 0 : 1;
        int s[6] = { piSrc[2 * i              - l], piSrc[2 * i                 ], piSrc[2 * i              + 1],
                     piSrc[2 * i + iRecStride - l], piSrc[2 * i + iRecStride    ], piSrc[2 * i + iRecStride + 1] };
        pDst[i] = xGlmGetLumaVal(s, c, glmIdc - 1, pDst[i]);
      }
#endif
    }
  }

  if (leftIsAvailable)
  {
    pDst  = pDst0    - 1;
    piSrc = pRecSrc0 - 1 - logSubWidthC;

    int addedLeftBelow = 0;
#if MMLM
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX) || (curChromaMode == MMLM_L_IDX) || (curChromaMode == MMLM_T_IDX))
#else
    if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
#endif
    {
      addedLeftBelow = avaiLeftBelowUnits*chromaUnitHeight;
    }

    for (int j = 0; j < uiCHeight + addedLeftBelow; j++)
    {
      if (pu.chromaFormat == CHROMA_444)
      {
        pDst[0] = piSrc[0];
      }
      else if (pu.chromaFormat == CHROMA_422)
      {
        int s = 2;
        s += piSrc[0] * 2;
        s += piSrc[-1];
        s += piSrc[1];
        pDst[0] = s >> 2;
      }
      else if (pu.cs->sps->getCclmCollocatedChromaFlag())
      {
        const bool abovePadding = j == 0 && !aboveIsAvailable;

        int s = 4;
        s += piSrc[-(abovePadding ? 0 : iRecStride)];
        s += piSrc[0] * 4;
        s += piSrc[-1];
        s += piSrc[1];
        s += piSrc[iRecStride];
        pDst[0] = s >> 3;
      }
      else
      {
        int s = 4;
        s += piSrc[0] * 2;
        s += piSrc[1];
        s += piSrc[-1];
        s += piSrc[iRecStride] * 2;
        s += piSrc[iRecStride + 1];
        s += piSrc[iRecStride - 1];
        pDst[0] = s >> 3;
      }
#if JVET_AA0126_GLM
      if (glmIdc != 0)
      {
        int s[6] = { piSrc[           - 1], piSrc[             0], piSrc[             1],
                     piSrc[iRecStride - 1], piSrc[iRecStride    ], piSrc[iRecStride + 1] };
        pDst[0] = xGlmGetLumaVal(s, c, glmIdc - 1, pDst[0]);
      }
#endif

      piSrc += iRecStride2;
      pDst  += iDstStride;
    }
  }

#if JVET_AF0073_INTER_CCP_MERGE
  if (!isTemplate)
  {
#endif
  // inner part from reconstructed picture buffer
  for( int j = 0; j < uiCHeight; j++ )
  {
    for( int i = 0; i < uiCWidth; i++ )
    {
      if (pu.chromaFormat == CHROMA_444)
      {
        pDst0[i] = pRecSrc0[i];
      }
      else if (pu.chromaFormat == CHROMA_422)
      {
        const bool leftPadding  = i == 0 && !leftIsAvailable;

        int s = 2;
        s += pRecSrc0[2 * i] * 2;
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + 1];
        pDst0[i] = s >> 2;
      }
      else if (pu.cs->sps->getCclmCollocatedChromaFlag())
      {
        const bool leftPadding  = i == 0 && !leftIsAvailable;
        const bool abovePadding = j == 0 && !aboveIsAvailable;

        int s = 4;
        s += pRecSrc0[2 * i - (abovePadding ? 0 : iRecStride)];
        s += pRecSrc0[2 * i] * 4;
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + 1];
        s += pRecSrc0[2 * i + iRecStride];
        pDst0[i] = s >> 3;
      }
      else
      {
        CHECK(pu.chromaFormat != CHROMA_420, "Chroma format must be 4:2:0 for vertical filtering");
        const bool leftPadding = i == 0 && !leftIsAvailable;

        int s = 4;
        s += pRecSrc0[2 * i] * 2;
        s += pRecSrc0[2 * i + 1];
        s += pRecSrc0[2 * i - (leftPadding ? 0 : 1)];
        s += pRecSrc0[2 * i + iRecStride] * 2;
        s += pRecSrc0[2 * i + 1 + iRecStride];
        s += pRecSrc0[2 * i + iRecStride - (leftPadding ? 0 : 1)];
        pDst0[i] = s >> 3;
      }
#if JVET_AA0126_GLM
      if (glmIdc != 0)
      {
        const bool leftPadding = i == 0 && !leftIsAvailable;
        int l = leftPadding ? 0 : 1;
        int s[6] = { pRecSrc0[2 * i              - l], pRecSrc0[2 * i                 ], pRecSrc0[2 * i              + 1],
                     pRecSrc0[2 * i + iRecStride - l], pRecSrc0[2 * i + iRecStride    ], pRecSrc0[2 * i + iRecStride + 1] };
        pDst0[i] = xGlmGetLumaVal(s, c, glmIdc - 1, pDst0[i]);
      }
#endif
    }

    pDst0    += iDstStride;
    pRecSrc0 += iRecStride2;
  }
#if JVET_AF0073_INTER_CCP_MERGE
  }
#endif
}

#if !LMS_LINEAR_MODEL
void IntraPrediction::xGetLMParameters(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea, CclmModel &cclmModel)
{
  CHECK(compID == COMPONENT_Y, "");

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure & cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const SPS &        sps           = *cs.sps;
  const uint32_t     tuWidth     = chromaArea.width;
  const uint32_t     tuHeight    = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth    = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeight   = baseUnitSize >> getComponentScaleY(chromaArea.compID, nChromaFormat);

  const int tuWidthInUnits  = tuWidth / unitWidth;
  const int tuHeightInUnits = tuHeight / unitHeight;
  const int aboveUnits      = tuWidthInUnits;
  const int leftUnits       = tuHeightInUnits;
  int topTemplateSampNum = 2 * cWidth; // for MDLM, the template sample number is 2W or 2H;
  int leftTemplateSampNum = 2 * cHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits = (topTemplateSampNum + (unitWidth - 1)) / unitWidth;
  int totalLeftUnits = (leftTemplateSampNum + (unitHeight - 1)) / unitHeight;
  int totalUnits = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits = totalAboveUnits - aboveUnits;
  int leftBelowUnits = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  int avaiAboveUnits = 0;
  int avaiLeftUnits = 0;

  int curChromaMode = pu.intraDir[1];
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  bool aboveAvailable, leftAvailable;

  int availableUnit =
    isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, aboveUnits, unitWidth,
    (neighborFlags + leftUnits + leftBelowUnits + 1));
  aboveAvailable = availableUnit == tuWidthInUnits;

  availableUnit =
    isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, leftUnits, unitHeight,
    (neighborFlags + leftUnits + leftBelowUnits - 1));
  leftAvailable = availableUnit == tuHeightInUnits;
  if (leftAvailable) // if left is not available, then the below left is not available
  {
    avaiLeftUnits = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp(chromaArea.compID), leftBelowUnits, unitHeight, (neighborFlags + leftBelowUnits - 1));
  }
  if (aboveAvailable) // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits = isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits, unitWidth, (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1));
  }
  Pel *srcColor0, *curChroma0;
  int srcStride;

  PelBuf temp;
#if MMLM
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX) || (curChromaMode == MMLM_L_IDX) || (curChromaMode == MMLM_T_IDX)
    || (m_encPreRDRun && curChromaMode == MMLM_CHROMA_IDX))
#else
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
#endif
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp        = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  srcColor0 = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  unsigned internalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  int minLuma[2] = {  MAX_INT, 0 };
  int maxLuma[2] = { -MAX_INT, 0 };
#if MMLM
  int minLuma2[2][2] = { { MAX_INT, 0 },{ MAX_INT, 0 } };
  int maxLuma2[2][2] = { { -MAX_INT, 0 },{ -MAX_INT, 0 } };
  int minDim = 1;
#endif
  Pel *src = srcColor0 - srcStride;
  int actualTopTemplateSampNum = 0;
  int actualLeftTemplateSampNum = 0;
#if MMLM
  if (curChromaMode == MDLM_T_IDX || curChromaMode == MMLM_T_IDX)
#else
  if (curChromaMode == MDLM_T_IDX)
#endif
  {
    leftAvailable = 0;
    avaiAboveRightUnits = avaiAboveRightUnits > (cHeight/unitWidth) ?  cHeight/unitWidth : avaiAboveRightUnits;
    actualTopTemplateSampNum = unitWidth*(avaiAboveUnits + avaiAboveRightUnits);
#if MMLM
    minDim = actualTopTemplateSampNum;
#endif
  }
#if MMLM
  else if (curChromaMode == MDLM_L_IDX || curChromaMode == MMLM_L_IDX)
#else
  else if (curChromaMode == MDLM_L_IDX)
#endif
  {
    aboveAvailable = 0;
    avaiLeftBelowUnits = avaiLeftBelowUnits > (cWidth/unitHeight) ? cWidth/unitHeight : avaiLeftBelowUnits;
    actualLeftTemplateSampNum = unitHeight*(avaiLeftUnits + avaiLeftBelowUnits);
#if MMLM
    minDim = actualLeftTemplateSampNum;
#endif
  }
#if MMLM
  else if (curChromaMode == LM_CHROMA_IDX || curChromaMode == MMLM_CHROMA_IDX)
#else
  else if (curChromaMode == LM_CHROMA_IDX)
#endif
  {
    actualTopTemplateSampNum = cWidth;
    actualLeftTemplateSampNum = cHeight;
#if MMLM
    minDim = leftAvailable && aboveAvailable ? 1 << g_aucPrevLog2[std::min(actualLeftTemplateSampNum, actualTopTemplateSampNum)]
      : 1 << g_aucPrevLog2[leftAvailable ? actualLeftTemplateSampNum : actualTopTemplateSampNum];
#endif
  }
#if MMLM
  int numSteps = minDim;
  int yAvg = 0;
  int avgCnt = 0;
#endif
  int startPos[2]; //0:Above, 1: Left
  int pickStep[2];

  int aboveIs4 = leftAvailable  ? 0 : 1;
  int leftIs4 =  aboveAvailable ? 0 : 1;

  startPos[0] = actualTopTemplateSampNum >> (2 + aboveIs4);
  pickStep[0] = std::max(1, actualTopTemplateSampNum >> (1 + aboveIs4));

  startPos[1] = actualLeftTemplateSampNum >> (2 + leftIs4);
  pickStep[1] = std::max(1, actualLeftTemplateSampNum >> (1 + leftIs4));

  Pel selectLumaPix[4] = { 0, 0, 0, 0 };
  Pel selectChromaPix[4] = { 0, 0, 0, 0 };

  int cntT, cntL;
  cntT = cntL = 0;
  int cnt = 0;
  if (aboveAvailable)
  {
    cntT = std::min(actualTopTemplateSampNum, (1 + aboveIs4) << 1);
    src = srcColor0 - srcStride;
    const Pel *cur = curChroma0 + 1;
    for (int pos = startPos[0]; cnt < cntT; pos += pickStep[0], cnt++)
    {
      selectLumaPix[cnt] = src[pos];
      selectChromaPix[cnt] = cur[pos];
    }
#if MMLM
    for (int j = 0; j < numSteps; j++)
    {
      int idx = (j * actualTopTemplateSampNum) / minDim;

      if (minLuma2[0][0] > src[idx])
      {
        minLuma2[0][0] = src[idx];
        minLuma2[0][1] = cur[idx];
      }
      if (maxLuma2[1][0] < src[idx])
      {
        maxLuma2[1][0] = src[idx];
        maxLuma2[1][1] = cur[idx];
      }

      yAvg += src[idx];
      avgCnt++;
    }
#endif
  }

  if (leftAvailable)
  {
    cntL = std::min(actualLeftTemplateSampNum, ( 1 + leftIs4 ) << 1 );
    src = srcColor0 - 1;
    const Pel *cur = curChroma0 + m_refBufferStride[compID] + 1;
    for (int pos = startPos[1], cnt = 0; cnt < cntL; pos += pickStep[1], cnt++)
    {
      selectLumaPix[cnt + cntT] = src[pos * srcStride];
      selectChromaPix[cnt + cntT] = cur[pos];
    }
#if MMLM
    for (int i = 0; i < numSteps; i++)
    {
      int idx = (i * actualLeftTemplateSampNum) / minDim;


      if (minLuma2[0][0] > src[srcStride * idx])
      {
        minLuma2[0][0] = src[srcStride * idx];
        minLuma2[0][1] = cur[idx];
      }
      if (maxLuma2[1][0] < src[srcStride * idx])
      {
        maxLuma2[1][0] = src[srcStride * idx];
        maxLuma2[1][1] = cur[idx];
      }

      yAvg += src[srcStride * idx];
      avgCnt++;
    }
#endif
  }
  cnt = cntL + cntT;

  if (cnt == 2)
  {
    selectLumaPix[3] = selectLumaPix[0]; selectChromaPix[3] = selectChromaPix[0];
    selectLumaPix[2] = selectLumaPix[1]; selectChromaPix[2] = selectChromaPix[1];
    selectLumaPix[0] = selectLumaPix[1]; selectChromaPix[0] = selectChromaPix[1];
    selectLumaPix[1] = selectLumaPix[3]; selectChromaPix[1] = selectChromaPix[3];
  }

  int minGrpIdx[2] = { 0, 2 };
  int maxGrpIdx[2] = { 1, 3 };
  int *tmpMinGrp = minGrpIdx;
  int *tmpMaxGrp = maxGrpIdx;
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMinGrp[1]])
  {
    std::swap(tmpMinGrp[0], tmpMinGrp[1]);
  }
  if (selectLumaPix[tmpMaxGrp[0]] > selectLumaPix[tmpMaxGrp[1]])
  {
    std::swap(tmpMaxGrp[0], tmpMaxGrp[1]);
  }
  if (selectLumaPix[tmpMinGrp[0]] > selectLumaPix[tmpMaxGrp[1]])
  {
    std::swap(tmpMinGrp, tmpMaxGrp);
  }
  if (selectLumaPix[tmpMinGrp[1]] > selectLumaPix[tmpMaxGrp[0]])
  {
    std::swap(tmpMinGrp[1], tmpMaxGrp[0]);
  }

  minLuma[0] = (selectLumaPix[tmpMinGrp[0]] + selectLumaPix[tmpMinGrp[1]] + 1 )>>1;
  minLuma[1] = (selectChromaPix[tmpMinGrp[0]] + selectChromaPix[tmpMinGrp[1]] + 1) >> 1;
  maxLuma[0] = (selectLumaPix[tmpMaxGrp[0]] + selectLumaPix[tmpMaxGrp[1]] + 1 )>>1;
  maxLuma[1] = (selectChromaPix[tmpMaxGrp[0]] + selectChromaPix[tmpMaxGrp[1]] + 1) >> 1;
#if MMLM
  if (avgCnt)
  {
    int x = floorLog2(avgCnt);
    // 4bit significands - 8 ( MSB is omitted )
    const uint8_t divSigTable[1 << 4] = { 0,  7,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  1,  1,  0 };
    int normDiff = (avgCnt << 4 >> x) & 15;
    int v = divSigTable[normDiff] | 8;
    x += normDiff != 0;
    yAvg = (yAvg * v) >> (x + 3);
  }
  int cntMMLM[2] = { 0,0 };

  //minLuma2[0][0] = minLuma[0]; minLuma2[0][1] = minLuma[1];
  //maxLuma2[1][0] = maxLuma[0]; maxLuma2[1][1] = maxLuma[1];
  src = srcColor0 - srcStride;

  const Pel *curMMLM = curChroma0 + 1;
  if (aboveAvailable)
  {
    for (int j = 0; j < numSteps; j++)
    {
      int idx = (j * actualTopTemplateSampNum) / minDim;
      if (src[idx] >= yAvg)
      {
        if (minLuma2[1][0] > src[idx])
        {
          minLuma2[1][0] = src[idx];
          minLuma2[1][1] = curMMLM[idx];
        }
        cntMMLM[1]++;
      }
      else
      {
        if (maxLuma2[0][0] < src[idx])
        {
          maxLuma2[0][0] = src[idx];
          maxLuma2[0][1] = curMMLM[idx];
        }
        cntMMLM[0]++;
      }
    }
  }

  if (leftAvailable)
  {
    src = srcColor0 - 1;
    //curMMLM = curChroma0 - 1; // should check here -1 or not
    const Pel *curMMLM = curChroma0 + m_refBufferStride[compID] + 1;
    for (int i = 0; i < numSteps; i++)
    {
      int idx = (i * actualLeftTemplateSampNum) / minDim;

      if (src[srcStride * idx] >= yAvg)
      {
        if (minLuma2[1][0] > src[srcStride * idx])
        {
          minLuma2[1][0] = src[srcStride * idx];
          minLuma2[1][1] = curMMLM[idx];
        }
        cntMMLM[1]++;
      }
      else
      {
        if (maxLuma2[0][0] < src[srcStride * idx])
        {
          maxLuma2[0][0] = src[srcStride * idx];
          maxLuma2[0][1] = curMMLM[idx];
        }
        cntMMLM[0]++;
      }
    }
  }

  if (PU::isMultiModeLM(curChromaMode))
  {
    CHECK(cntMMLM[0] && minLuma2[0][0] > maxLuma2[0][0], "Invalid class");
    CHECK(cntMMLM[1] && minLuma2[1][0] > maxLuma2[1][0], "Invalid class");
    CHECK(cntMMLM[0] && cntMMLM[1] && maxLuma2[0][0] > minLuma2[1][0], "Invalid boundary");

    for (int i = 0; i < 2; i++)
    {
      int ax = 0, bx = 0, iShiftx = 0;
      if (cntMMLM[i])
      {
        int diff = maxLuma2[i][0] - minLuma2[i][0];
        if (diff > 0)
        {
          int diffC = maxLuma2[i][1] - minLuma2[i][1];
          int x = floorLog2(diff);
          // 4bit significands - 8 ( MSB is omitted )
          const uint8_t divSigTable[1 << 4] = { 0,  7,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  1,  1,  0 };
          int normDiff = (diff << 4 >> x) & 15;
          int v = divSigTable[normDiff] | 8;
          x += normDiff != 0;

          int y = floorLog2(abs(diffC)) + 1;
          int add = 1 << y >> 1;
          ax = (diffC * v + add) >> y;
          iShiftx = 3 + x - y;
          if (iShiftx < 1)
          {
            iShiftx = 1;
            ax = ((ax == 0) ? 0 : (ax < 0) ? -15 : 15);   // a=Sign(a)*15
          }
          bx = minLuma2[i][1] - ((ax * minLuma2[i][0]) >> iShiftx);
        }
        else
        {
          ax = 0;
          bx = minLuma2[i][1];
          iShiftx = 0;
        }
      }
      else
      {
        ax = 0; bx = 1 << (internalBitDepth - 1); iShiftx = 0;
      }
      if (i == 0)
      {
        cclmModel.setFirstModel ( ax, bx, iShiftx );
      }
      else
      {
        cclmModel.setSecondModel( ax, bx, iShiftx, yAvg );
      }
    }

#if JVET_Z0050_CCLM_SLOPE
    cclmModel.midLuma  = cntMMLM[0] ? ( maxLuma2[0][0] + minLuma2[0][0] ) >> 1 : 0;
    cclmModel.midLuma2 = cntMMLM[1] ? ( maxLuma2[1][0] + minLuma2[1][0] ) >> 1 : 0;
#endif
  }
  else
  {
#endif // Non MMLM mode
  if (leftAvailable || aboveAvailable)
  {
    int diff = maxLuma[0] - minLuma[0];
#if JVET_Z0050_CCLM_SLOPE
    cclmModel.midLuma = ( maxLuma[0] + minLuma[0] ) >> 1;
#endif

    if (diff > 0)
    {
      int diffC = maxLuma[1] - minLuma[1];
      int x = floorLog2( diff );
      // 4bit significands - 8 ( MSB is omitted )
      const uint8_t divSigTable[1 << 4] = { 0,  7,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  1,  1,  0 };
      int normDiff = (diff << 4 >> x) & 15;
      int v = divSigTable[normDiff] | 8;
      x += normDiff != 0;

      int y = floorLog2( abs( diffC ) ) + 1;
      int add = 1 << y >> 1;
      int a = (diffC * v + add) >> y;
      int b = 0;
      int iShift = 3 + x - y;

      if ( iShift < 1 )
      {
        iShift = 1;
        a = ( (a == 0)? 0: (a < 0)? -15 : 15 );   // a=Sign(a)*15
      }
      b = minLuma[1] - ((a * minLuma[0]) >> iShift);

      cclmModel.setFirstModel ( a, b, iShift );
    }
    else
    {
      cclmModel.setFirstModel ( 0, minLuma[1], 0 );
    }
  }
  else
  {
    cclmModel.setFirstModel ( 0, 1 << (internalBitDepth - 1), 0 );
  }
#if MMLM
  }
#endif
}
#endif

void IntraPrediction::initIntraMip( const PredictionUnit &pu, const CompArea &area )
{
  CHECK( area.width > MIP_MAX_WIDTH || area.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );

  // prepare input (boundary) data for prediction
  CHECK( m_ipaParam.refFilterFlag, "ERROR: unfiltered refs expected for MIP" );
  Pel       *ptrSrc     = getPredictorPtr(area.compID);
  const int  srcStride  = m_refBufferStride[area.compID];
  const int  srcHStride = 2;

  m_matrixIntraPred.prepareInputForPred(CPelBuf(ptrSrc, srcStride, srcHStride), area,
                                        pu.cu->slice->getSPS()->getBitDepth(toChannelType(area.compID)), area.compID);
}

#if JVET_AB0067_MIP_DIMD_LFNST
void IntraPrediction::predIntraMip(const ComponentID compId, PelBuf& piPred, const PredictionUnit& pu, bool useDimd)
#else
void IntraPrediction::predIntraMip( const ComponentID compId, PelBuf &piPred, const PredictionUnit &pu )
#endif
{
  CHECK( piPred.width > MIP_MAX_WIDTH || piPred.height > MIP_MAX_HEIGHT, "Error: block size not supported for MIP" );
  CHECK( piPred.width != (1 << floorLog2(piPred.width)) || piPred.height != (1 << floorLog2(piPred.height)), "Error: expecting blocks of size 2^M x 2^N" );

  // generate mode-specific prediction
  uint32_t modeIdx       = MAX_NUM_MIP_MODE;
  bool     transposeFlag = false;
  if (compId == COMPONENT_Y)
  {
    modeIdx       = pu.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = pu.mipTransposedFlag;
  }
  else
  {
    const PredictionUnit &coLocatedLumaPU = PU::getCoLocatedLumaPU(pu);

    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "Error: MIP is only supported for chroma with DM_CHROMA.");
    CHECK(!coLocatedLumaPU.cu->mipFlag, "Error: Co-located luma CU should use MIP.");

    modeIdx       = coLocatedLumaPU.intraDir[CHANNEL_TYPE_LUMA];
    transposeFlag = coLocatedLumaPU.mipTransposedFlag;
  }
  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth(toChannelType(compId));

  CHECK(modeIdx >= getNumModesMip(piPred), "Error: Wrong MIP mode index");

  static_vector<int, MIP_MAX_WIDTH* MIP_MAX_HEIGHT> predMip( piPred.width * piPred.height );
#if JVET_AB0067_MIP_DIMD_LFNST
  if (useDimd)
  {
    int sizeId = getMipSizeId(Size(piPred.width, piPred.height));
    int reducedPredSize = (sizeId < 2) ? 4 : 8;
    static_vector<int, MIP_MAX_WIDTH* MIP_MAX_HEIGHT> reducedPred(reducedPredSize * reducedPredSize);
    m_matrixIntraPred.predBlock(predMip.data(), modeIdx, transposeFlag, bitDepth, compId, reducedPred.data());
    int    iLumaStride = MAX_CU_SIZE + 1;
    PelBuf reducedPredTemp = PelBuf(m_pMipTemp + iLumaStride + 1, iLumaStride, Size(reducedPredSize, reducedPredSize));
    Pel* pReducePred = reducedPredTemp.buf;
    int idx = 0;
    for (int y = 0; y < reducedPredSize; y++)
    {
      for (int x = 0; x < reducedPredSize; x++)
      {
        pReducePred[x] = Pel(reducedPred[idx++]);
      }
      pReducePred += reducedPredTemp.stride;
    }
    int iMode = deriveDimdMipMode(reducedPredTemp, reducedPredSize, reducedPredSize, *pu.cu);
    pu.cu->mipDimdMode = iMode;
  }
  else
  {
    m_matrixIntraPred.predBlock(predMip.data(), modeIdx, transposeFlag, bitDepth, compId);
  }
#else
  m_matrixIntraPred.predBlock(predMip.data(), modeIdx, transposeFlag, bitDepth, compId);
#endif
  Pel *pred = piPred.buf;
  int idx = 0;

  for( int y = 0; y < piPred.height; y++ )
  {
    for( int x = 0; x < piPred.width; x++ )
    {
      pred[x] = Pel(predMip[idx++]);
    }

    pred += piPred.stride;
  }
}

void IntraPrediction::reorderPLT(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);

  uint8_t        reusePLTSizetmp = 0;
  uint8_t        pltSizetmp = 0;
  Pel            curPLTtmp[MAX_NUM_COMPONENT][MAXPLTSIZE];
  bool           curPLTpred[MAXPLTPREDSIZE];

  for (int idx = 0; idx < MAXPLTPREDSIZE; idx++)
  {
    curPLTpred[idx] = false;
    cu.reuseflag[compBegin][idx] = false;
  }
  for (int idx = 0; idx < MAXPLTSIZE; idx++)
  {
    curPLTpred[idx] = false;
  }

  for (int predidx = 0; predidx < cs.prevPLT.curPLTSize[compBegin]; predidx++)
  {
    bool match = false;
    int curidx = 0;

    for (curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
    {
      if( curPLTpred[curidx] )
      {
        continue;
      }
      bool matchTmp = true;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        matchTmp = matchTmp && (cu.curPLT[comp][curidx] == cs.prevPLT.curPLT[comp][predidx]);
      }
      if (matchTmp)
      {
        match = true;
        break;
      }
    }

    if (match)
    {
      cu.reuseflag[compBegin][predidx] = true;
      curPLTpred[curidx] = true;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
#else
      if (CS::isDualITree(*cu.cs))
#endif
      {
        cu.reuseflag[COMPONENT_Y][predidx] = true;
        for( int comp = COMPONENT_Y; comp < MAX_NUM_COMPONENT; comp++ )
        {
          curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
        }
      }
      else
      {
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          curPLTtmp[comp][reusePLTSizetmp] = cs.prevPLT.curPLT[comp][predidx];
        }
      }
      reusePLTSizetmp++;
      pltSizetmp++;
    }
  }
  cu.reusePLTSize[compBegin] = reusePLTSizetmp;
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
    if (!curPLTpred[curidx])
    {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        for( int comp = compBegin; comp < (compBegin + numComp); comp++ )
        {
          curPLTtmp[comp][pltSizetmp] = cu.curPLT[comp][curidx];
        }
        if( isLuma(partitioner.chType) )
        {
          curPLTtmp[COMPONENT_Cb][pltSizetmp] = 1 << (cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
          curPLTtmp[COMPONENT_Cr][pltSizetmp] = 1 << (cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
        }
        else
        {
          curPLTtmp[COMPONENT_Y][pltSizetmp] = 1 << (cs.sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
        }
      }
      else
      {
#endif
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          curPLTtmp[comp][pltSizetmp] = cu.curPLT[comp][curidx];
        }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      }
#endif
      pltSizetmp++;
    }
  }
  assert(pltSizetmp == cu.curPLTSize[compBegin]);
  for (int curidx = 0; curidx < cu.curPLTSize[compBegin]; curidx++)
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isLocalSepTree() )
#else
    if (CS::isDualITree(*cu.cs))
#endif
    {
      for( int comp = COMPONENT_Y; comp < MAX_NUM_COMPONENT; comp++ )
      {
        cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
      }
    }
    else
    {
    for (int comp = compBegin; comp < (compBegin + numComp); comp++)
    {
      cu.curPLT[comp][curidx] = curPLTtmp[comp][curidx];
    }
    }
  }
}

#if MMLM && LMS_LINEAR_MODEL
int IntraPrediction::xCalcLMParametersGeneralized(int x, int y, int xx, int xy, int count, int bitDepth, int &a, int &b, int &iShift)
{

  uint32_t uiInternalBitDepth = bitDepth;
  if (count == 0)
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
    return -1;
  }
  CHECK(count > 512, "");


  int iCountShift = g_aucLog2[count];

  int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if (iTempShift > 0)
  {
    x = (x + (1 << (iTempShift - 1))) >> iTempShift;
    y = (y + (1 << (iTempShift - 1))) >> iTempShift;
    xx = (xx + (1 << (iTempShift - 1))) >> iTempShift;
    xy = (xy + (1 << (iTempShift - 1))) >> iTempShift;
    iCountShift -= iTempShift;
  }
  /////// xCalcLMParameters

  int avgX = x >> iCountShift;
  int avgY = y >> iCountShift;

  int RErrX = x & ((1 << iCountShift) - 1);
  int RErrY = y & ((1 << iCountShift) - 1);

  int iB = 7;
  iShift = 13 - iB;

  if (iCountShift == 0)
  {
    a = 0;
    b = 1 << (uiInternalBitDepth - 1);
    iShift = 0;
  }
  else
  {
    int a1 = xy - (avgX * avgY << iCountShift) - avgX * RErrY - avgY * RErrX;
    int a2 = xx - (avgX * avgX << iCountShift) - 2 * avgX * RErrX;
    const int iShiftA1 = uiInternalBitDepth - 2;
    const int iShiftA2 = 5;
    const int iAccuracyShift = uiInternalBitDepth + 4;

    int iScaleShiftA2 = 0;
    int iScaleShiftA1 = 0;
    int a1s = a1;
    int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : floorLog2(abs(a1)) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : floorLog2(abs(a2)) - iShiftA2;

    if (iScaleShiftA1 < 0)
    {
      iScaleShiftA1 = 0;
    }

    if (iScaleShiftA2 < 0)
    {
      iScaleShiftA2 = 0;
    }

    int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      uint32_t a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if (iScaleShiftA < 0)
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
    a = a << iB;

    int16_t n = 0;
    if (a != 0)
    {
      n = floorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
    }

    iShift = (iShift + iB) - n;
    a = a >> n;

    b = avgY - ((a * avgX) >> iShift);
  }
  return 0;
}

int IntraPrediction::xLMSampleClassifiedTraining(int count, int mean, int meanC, int lumaSamples[], int chrmSamples[], int bitDepth, MMLMParameters parameters[])
{

  //Initialize

  for (int i = 0; i < 2; i++)
  {
    parameters[i].a = 0;
    parameters[i].b = 1 << (bitDepth - 1);
    parameters[i].shift = 0;
  }

  if (count < 4)//
  {
    return -1;
  }
  int groupCount[2] = { 0, 0 };

  CHECK(count > 512, "");

  int meanDiff = meanC - mean;
  mean = std::max(1, mean);

  int lumaPower2[2][128];
  int chromaPower2[2][128];
  //int GroupCount[2] = { 0, 0 };
  for (int i = 0; i < count; i++)
  {
    if (lumaSamples[i] <= mean)
    {
      lumaPower2[0][groupCount[0]] = lumaSamples[i];
      chromaPower2[0][groupCount[0]] = chrmSamples[i];
      groupCount[0]++;
    }
    else
    {
      lumaPower2[1][groupCount[1]] = lumaSamples[i];
      chromaPower2[1][groupCount[1]] = chrmSamples[i];
      groupCount[1]++;
    }
  }

  // Take power of two
  for (int group = 0; group < 2; group++)
  {
    int existSampNum = groupCount[group];
    if (existSampNum < 2)
    {
      continue;
    }

    int upperPower2 = 1 << (g_aucLog2[existSampNum - 1] + 1);
    int lowerPower2 = 1 << (g_aucLog2[existSampNum]);

    if (upperPower2 != lowerPower2)
    {
      int numPaddedSamples = std::min(existSampNum, upperPower2 - existSampNum);
      groupCount[group] = upperPower2;
      int step = (int)(existSampNum / numPaddedSamples);
      for (int i = 0; i < numPaddedSamples; i++)
      {
        lumaPower2[group][existSampNum + i] = lumaPower2[group][i * step];
        chromaPower2[group][existSampNum + i] = chromaPower2[group][i * step];

      }
    }
  }

  int x[2], y[2], xy[2], xx[2];
  for (int group = 0; group < 2; group++)
  {
    x[group] = y[group] = xy[group] = xx[group] = 0;
  }

  for (int group = 0; group < 2; group++)
  {

    for (int i = 0; i < groupCount[group]; i++)
    {
      x[group] += lumaPower2[group][i];
      y[group] += chromaPower2[group][i];
      xx[group] += lumaPower2[group][i] * lumaPower2[group][i];
      xy[group] += lumaPower2[group][i] * chromaPower2[group][i];
    }
  }
  for (int group = 0; group < 2; group++)
  {
    int a, b, iShift;
    if (groupCount[group] > 1)
    {
      xCalcLMParametersGeneralized(x[group], y[group], xx[group], xy[group], groupCount[group], bitDepth, a, b, iShift);

      parameters[group].a = a;
      parameters[group].b = b;
      parameters[group].shift = iShift;
    }
    else
    {
      parameters[group].a = 0;
      parameters[group].b = meanDiff;
      parameters[group].shift = 0;
    }
  }
  return 0;
}
#endif
#if LMS_LINEAR_MODEL
void IntraPrediction::xPadMdlmTemplateSample(Pel*pSrc, Pel*pCur, int cWidth, int cHeight, int existSampNum, int targetSampNum)
{
  int sampNumToBeAdd = targetSampNum - existSampNum;
  Pel*pTempSrc = pSrc + existSampNum;
  Pel*pTempCur = pCur + existSampNum;

  int step = (int)(existSampNum / sampNumToBeAdd);

  for (int i = 0; i < sampNumToBeAdd; i++)
  {
    pTempSrc[i] = pSrc[i * step];
    pTempCur[i] = pCur[i * step];
  }
}
void IntraPrediction::xGetLMParametersLMS(const PredictionUnit &pu, const ComponentID compID, const CompArea& chromaArea, CclmModel &cclmModel)
{
  CHECK(compID == COMPONENT_Y, "");
  const SizeType cWidth = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  const Position posLT = chromaArea;

  CodingStructure & cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const SPS &        sps = *cs.sps;
  const uint32_t     tuWidth = chromaArea.width;
  const uint32_t     tuHeight = chromaArea.height;
  const ChromaFormat nChromaFormat = sps.getChromaFormatIdc();

  const int baseUnitSize = 1 << MIN_CU_LOG2;
  const int unitWidth = baseUnitSize >> getComponentScaleX(chromaArea.compID, nChromaFormat);
  const int unitHeight = baseUnitSize >> getComponentScaleY(chromaArea.compID, nChromaFormat);

  const int tuWidthInUnits = tuWidth / unitWidth;
  const int tuHeightInUnits = tuHeight / unitHeight;
  const int aboveUnits = tuWidthInUnits;
  const int leftUnits = tuHeightInUnits;
  int topTemplateSampNum = 2 * cWidth; // for MDLM, the template sample number is 2W or 2H;
  int leftTemplateSampNum = 2 * cHeight;
  assert(m_topRefLength >= topTemplateSampNum);
  assert(m_leftRefLength >= leftTemplateSampNum);
  int totalAboveUnits = (topTemplateSampNum + (unitWidth - 1)) / unitWidth;
  int totalLeftUnits = (leftTemplateSampNum + (unitHeight - 1)) / unitHeight;
  int totalUnits = totalLeftUnits + totalAboveUnits + 1;
  int aboveRightUnits = totalAboveUnits - aboveUnits;
  int leftBelowUnits = totalLeftUnits - leftUnits;
  int avaiAboveRightUnits = 0;
  int avaiLeftBelowUnits = 0;
  int avaiAboveUnits = 0;
  int avaiLeftUnits = 0;

  int curChromaMode = pu.intraDir[1];
  bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1];
  memset(neighborFlags, 0, totalUnits);

  bool aboveAvailable, leftAvailable;

  int availableUnit =
    isAboveAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, aboveUnits, unitWidth,
    (neighborFlags + leftUnits + leftBelowUnits + 1));
  aboveAvailable = availableUnit == tuWidthInUnits;

  availableUnit =
    isLeftAvailable(cu, CHANNEL_TYPE_CHROMA, posLT, leftUnits, unitHeight,
    (neighborFlags + leftUnits + leftBelowUnits - 1));
  leftAvailable = availableUnit == tuHeightInUnits;
  if (leftAvailable) // if left is not available, then the below left is not available
  {
    avaiLeftUnits = tuHeightInUnits;
    avaiLeftBelowUnits = isBelowLeftAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.bottomLeftComp(chromaArea.compID), leftBelowUnits, unitHeight, (neighborFlags + leftBelowUnits - 1));
  }
  if (aboveAvailable) // if above is not available, then  the above right is not available.
  {
    avaiAboveUnits = tuWidthInUnits;
    avaiAboveRightUnits = isAboveRightAvailable(cu, CHANNEL_TYPE_CHROMA, chromaArea.topRightComp(chromaArea.compID), aboveRightUnits, unitWidth, (neighborFlags + leftUnits + leftBelowUnits + aboveUnits + 1));
  }
  Pel *srcColor0, *curChroma0;
  int srcStride;

  PelBuf temp;
#if JVET_AA0126_GLM
  if (pu.glmIdc.isActive())
  {
    int glmIdc = pu.glmIdc.getIdc(compID, 0);
    Pel* glmTemp = compID == COMPONENT_Cb ? m_glmTempCb[glmIdc] : m_glmTempCr[glmIdc];
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp = PelBuf(glmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
#endif
#if MMLM
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX) || (curChromaMode == MMLM_L_IDX) || (curChromaMode == MMLM_T_IDX)
    || (m_encPreRDRun && curChromaMode == MMLM_CHROMA_IDX))
#else
  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
#endif
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
#if JVET_AA0126_GLM
  }
#endif
  srcColor0 = temp.bufAt(0, 0);
  curChroma0 = getPredictorPtr(compID);

  int x = 0, y = 0, xx = 0, xy = 0;
  int iCountShift = 0;
  unsigned uiInternalBitDepth = sps.getBitDepth(CHANNEL_TYPE_CHROMA);

  Pel *src = srcColor0 - srcStride;
  int actualTopTemplateSampNum = 0;
  int actualLeftTemplateSampNum = 0;

  //get the temp buffer to store the downsampled luma and chroma
  Pel pTempBufferSrc[2 * MAX_CU_SIZE]; // for MDLM, use tempalte size 2W or 2H,
  Pel pTempBufferCur[2 * MAX_CU_SIZE];
  int minDim = 1;
  int cntT = 0; int cntL = 0;

#if MMLM
  if (curChromaMode == MDLM_T_IDX || curChromaMode == MMLM_T_IDX)
#else
  if (curChromaMode == MDLM_T_IDX)
#endif
  {
    leftAvailable = 0;
#if !LMS_LINEAR_MODEL
    avaiAboveRightUnits = avaiAboveRightUnits > (cHeight / unitWidth) ? cHeight / unitWidth : avaiAboveRightUnits;
#endif
    actualTopTemplateSampNum = unitWidth * (avaiAboveUnits + avaiAboveRightUnits);
    minDim = actualTopTemplateSampNum;
  }
#if MMLM
  else if (curChromaMode == MDLM_L_IDX || curChromaMode == MMLM_L_IDX)
#else
  else if (curChromaMode == MDLM_L_IDX)
#endif
  {
    aboveAvailable = 0;
#if !LMS_LINEAR_MODEL
    avaiLeftBelowUnits = avaiLeftBelowUnits > (cWidth / unitHeight) ? cWidth / unitHeight : avaiLeftBelowUnits;
#endif
    actualLeftTemplateSampNum = unitHeight * (avaiLeftUnits + avaiLeftBelowUnits);
    minDim = actualLeftTemplateSampNum;
  }
#if MMLM
  else if (curChromaMode == LM_CHROMA_IDX || curChromaMode == MMLM_CHROMA_IDX)
#else
  else if (curChromaMode == LM_CHROMA_IDX)
#endif
  {
    actualTopTemplateSampNum = cWidth;
    actualLeftTemplateSampNum = cHeight;
    minDim = leftAvailable && aboveAvailable ? 1 << g_aucPrevLog2[std::min(actualLeftTemplateSampNum, actualTopTemplateSampNum)]
      : 1 << g_aucPrevLog2[leftAvailable ? actualLeftTemplateSampNum : actualTopTemplateSampNum];
  }
  int numSteps = minDim;

#if JVET_Z0050_CCLM_SLOPE
  int sumLuma = 0;
  int numPels = aboveAvailable && leftAvailable ? 2*numSteps : aboveAvailable || leftAvailable ? numSteps : 0;
#endif

  if (aboveAvailable)
  {
    cntT = numSteps;
    src = srcColor0 - srcStride;
    const Pel *cur = curChroma0 + 1;

    for (int j = 0; j < numSteps; j++)
    {
      int idx = (j * actualTopTemplateSampNum) / minDim;

      pTempBufferSrc[j] = src[idx];
      pTempBufferCur[j] = cur[idx];
#if JVET_Z0050_CCLM_SLOPE
      sumLuma          += src[idx];
#endif
    }

  }

  if (leftAvailable)
  {
    cntL = numSteps;
    src = srcColor0 - 1;
    const Pel *cur = curChroma0 + m_refBufferStride[compID] + 1;

    for (int i = 0; i < numSteps; i++)
    {
      int idx = (i * actualLeftTemplateSampNum) / minDim;

      pTempBufferSrc[i + cntT] = src[srcStride * idx];
      pTempBufferCur[i + cntT] = cur[idx];
#if JVET_Z0050_CCLM_SLOPE
      sumLuma                 += src[srcStride * idx];
#endif
    }
  }
  
#if JVET_Z0050_CCLM_SLOPE
#if JVET_AD0184_REMOVAL_OF_DIVISION_OPERATIONS
  cclmModel.midLuma = numPels ? PU::getMeanValue( sumLuma + (numPels >> 1), numPels ) : 1 << (uiInternalBitDepth - 1);
#else
  cclmModel.midLuma = numPels ? ( sumLuma + numPels/2 ) / numPels : 1 << (uiInternalBitDepth - 1);
#endif
#endif

  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    //pad the temple sample to targetSampNum.
    int orgNumSample = (curChromaMode == MDLM_T_IDX) ? (avaiAboveUnits*unitWidth) : (avaiLeftUnits*unitHeight);
    int existSampNum = (curChromaMode == MDLM_T_IDX) ? actualTopTemplateSampNum : actualLeftTemplateSampNum;

    if( !orgNumSample || !existSampNum )
    {
      cclmModel.setFirstModel( 0, 1 << (uiInternalBitDepth - 1), 0 );
      return;
    }

    int targetSampNum = 1 << ( floorLog2( existSampNum - 1 ) + 1 );

    if (targetSampNum != existSampNum)//if existSampNum not a value of power of 2
    {
      xPadMdlmTemplateSample(pTempBufferSrc, pTempBufferCur, cWidth, cHeight, existSampNum, targetSampNum);
    }
    for (int j = 0; j < targetSampNum; j++)
    {
      x += pTempBufferSrc[j];
      y += pTempBufferCur[j];
      xx += pTempBufferSrc[j] * pTempBufferSrc[j];
      xy += pTempBufferSrc[j] * pTempBufferCur[j];
    }
    iCountShift = g_aucLog2[targetSampNum];

  }
  else if (curChromaMode == LM_CHROMA_IDX)
  {
    int       minStep = 1;
    //int       numSteps = minDim;

    if( aboveAvailable )
    {
      iCountShift = g_aucLog2[minDim / minStep];
    }

    if( leftAvailable )
    {
      iCountShift += aboveAvailable ? 1 : g_aucLog2[minDim / minStep];
    }

    for (int i = 0; i < (cntT + cntL); i++)
    {
      x += pTempBufferSrc[i];
      y += pTempBufferCur[i];
      xx += pTempBufferSrc[i] * pTempBufferSrc[i];
      xy += pTempBufferSrc[i] * pTempBufferCur[i];
    }
  }
#if MMLM
  if (PU::isMultiModeLM(pu.intraDir[1]))
  {
    // Classify and training
    MMLMParameters parameters[2];
    int lumaSamples[512];
    int chromaSamples[512];
    int meanC = 0; int mean = 0;
    int avgCnt = cntT + cntL;

    for (int i = 0; i < avgCnt; i++)
    {
      mean += pTempBufferSrc[i];
      meanC += pTempBufferCur[i];
      lumaSamples[i] = pTempBufferSrc[i];
      chromaSamples[i] = pTempBufferCur[i];
    }

    if (avgCnt)
    {
       int x = floorLog2(avgCnt);
       // 4bit significands - 8 ( MSB is omitted )
       const uint8_t divSigTable[1 << 4] = { 0,  7,  6,  5,  5,  4,  4,  3,  3,  2,  2,  1,  1,  1,  1,  0 };
       int normDiff = (avgCnt << 4 >> x) & 15;
       int v = divSigTable[normDiff] | 8;
       x += normDiff != 0;

       mean = (mean * v) >> (x + 3);
       meanC = (meanC * v) >> (x + 3);
    }

    xLMSampleClassifiedTraining(avgCnt, mean, meanC, lumaSamples, chromaSamples, uiInternalBitDepth, parameters);

    cclmModel.setFirstModel ( parameters[0].a, parameters[0].b, parameters[0].shift );
    cclmModel.setSecondModel( parameters[1].a, parameters[1].b, parameters[1].shift, mean );

#if JVET_Z0050_CCLM_SLOPE
    // Middle luma values for the two models
    int sumLuma0 = 0;
    int sumLuma1 = 0;
    int numPels0 = 0;
    int numPels1 = 0;

    for (int i = 0; i < avgCnt; i++)
    {
      if ( lumaSamples[i] <= mean )
      {
        sumLuma0 += lumaSamples[i];
        numPels0 += 1;
      }
      else
      {
        sumLuma1 += lumaSamples[i];
        numPels1 += 1;
      }
    }

#if JVET_AD0184_REMOVAL_OF_DIVISION_OPERATIONS
    cclmModel.midLuma  = numPels0 ? PU::getMeanValue( sumLuma0 + (numPels0 >> 1), numPels0 ) : mean;
    cclmModel.midLuma2 = numPels1 ? PU::getMeanValue( sumLuma1 + (numPels1 >> 1), numPels1 ) : mean;
#else
    cclmModel.midLuma  = numPels0 ? ( sumLuma0 + numPels0/2 ) / numPels0 : mean;
    cclmModel.midLuma2 = numPels1 ? ( sumLuma1 + numPels1/2 ) / numPels1 : mean;
#endif
#endif

    return;
  }
#endif

  if ((curChromaMode == MDLM_L_IDX) || (curChromaMode == MDLM_T_IDX))
  {
    if ((curChromaMode == MDLM_L_IDX) ? (!leftAvailable) : (!aboveAvailable))
    {
      cclmModel.setFirstModel( 0, 1 << (uiInternalBitDepth - 1), 0 );
      return;
    }
  }
  else
  {
    if (!leftAvailable && !aboveAvailable)
    {
      cclmModel.setFirstModel( 0, 1 << (uiInternalBitDepth - 1), 0 );
      return;
    }
  }

  int iTempShift = uiInternalBitDepth + iCountShift - 15;

  if (iTempShift > 0)
  {
    x = (x + (1 << (iTempShift - 1))) >> iTempShift;
    y = (y + (1 << (iTempShift - 1))) >> iTempShift;
    xx = (xx + (1 << (iTempShift - 1))) >> iTempShift;
    xy = (xy + (1 << (iTempShift - 1))) >> iTempShift;
    iCountShift -= iTempShift;
  }

  /////// xCalcLMParameters

  int avgX = x >> iCountShift;
  int avgY = y >> iCountShift;

  int RErrX = x & ((1 << iCountShift) - 1);
  int RErrY = y & ((1 << iCountShift) - 1);

  int iB = 7;
  int a      = 0;
  int b      = 0;
  int iShift = 13 - iB;

  if (iCountShift == 0)
  {
    cclmModel.setFirstModel( 0, 1 << (uiInternalBitDepth - 1), 0 );
  }
  else
  {
    int a1 = xy - (avgX * avgY << iCountShift) - avgX * RErrY - avgY * RErrX;
    int a2 = xx - (avgX * avgX << iCountShift) - 2 * avgX * RErrX;
    const int iShiftA1 = uiInternalBitDepth - 2;
    const int iShiftA2 = 5;
    const int iAccuracyShift = uiInternalBitDepth + 4;

    int iScaleShiftA2 = 0;
    int iScaleShiftA1 = 0;
    int a1s = a1;
    int a2s = a2;

    iScaleShiftA1 = a1 == 0 ? 0 : floorLog2(abs(a1)) - iShiftA1;
    iScaleShiftA2 = a2 == 0 ? 0 : floorLog2(abs(a2)) - iShiftA2;

    if (iScaleShiftA1 < 0)
    {
      iScaleShiftA1 = 0;
    }

    if (iScaleShiftA2 < 0)
    {
      iScaleShiftA2 = 0;
    }

    int iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iShift - iScaleShiftA1;

    a2s = a2 >> iScaleShiftA2;

    a1s = a1 >> iScaleShiftA1;

    if (a2s >= 32)
    {
      uint32_t a2t = m_auShiftLM[a2s - 32];
      a = a1s * a2t;
    }
    else
    {
      a = 0;
    }

    if (iScaleShiftA < 0)
    {
      a = a << -iScaleShiftA;
    }
    else
    {
      a = a >> iScaleShiftA;
    }
    a = Clip3(-(1 << (15 - iB)), (1 << (15 - iB)) - 1, a);
    a = a << iB;

    int16_t n = 0;
    if (a != 0)
    {
      n = floorLog2(abs(a) + ((a < 0 ? -1 : 1) - 1) / 2) - 5;
    }

    iShift = (iShift + iB) - n;
    a = a >> n;
    b = avgY - ((a * avgX) >> iShift);

    cclmModel.setFirstModel( a, b, iShift );
  }
}
#endif

#if JVET_V0130_INTRA_TMP
void insertNode( int diff, int& iXOffset, int& iYOffset, int& pDiff, int& pX, int& pY )
{
  pDiff = diff;
  pX = iXOffset;
  pY = iYOffset;
}

void clipMvIntraConstraint( CodingUnit* pcCU, int regionId, int& iHorMin, int& iHorMax, int& iVerMin, int& iVerMax, unsigned int uiTemplateSize, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int iCurrY, int iCurrX, int offsetLCUY, int offsetLCUX, RefTemplateType tempType )
{
#if JVET_AD0086_ENHANCED_INTRA_TMP
  int searchRangeWidth  = std::max(TMP_SEARCH_RANGE_MULT_FACTOR * (int) uiBlkWidth, TMP_MINSR);
  int searchRangeHeight = std::max(TMP_SEARCH_RANGE_MULT_FACTOR * (int) uiBlkHeight, TMP_MINSR);
#else  
  int searchRangeWidth = TMP_SEARCH_RANGE_MULT_FACTOR * uiBlkWidth;
  int searchRangeHeight = TMP_SEARCH_RANGE_MULT_FACTOR * uiBlkHeight;
#endif   
  int iMvShift = 0;
  int iTemplateSize = uiTemplateSize;
  int iBlkWidth = uiBlkWidth;
  int iBlkHeight = uiBlkHeight;
  int iCTUsize = pcCU->cs->sps->getCTUSize();

  iHorMin = iVerMin = 0;
  iHorMax = iVerMax = INT_MAX;

  if (regionId == 0) // Above outside CTU
  {
    iVerMax = (iCurrY - offsetLCUY - iBlkHeight) << iMvShift;
  }
  else if (regionId == 1) // Left outside and top-left within CTU
  {
#if JVET_AD0086_ENHANCED_INTRA_TMP
    iHorMax = (iCurrX - iBlkWidth) << iMvShift;
#if JVET_AE0077_EXT_INTRATMP
    iVerMin = (iCurrY - offsetLCUY - iBlkHeight + 1) << iMvShift;
#else
    iVerMin = (iCurrY - offsetLCUY - iBlkHeight) << iMvShift;
#endif
    iVerMax = (iCurrY) << iMvShift;
#else
    iHorMax = (iCurrX - offsetLCUX - iBlkWidth) << iMvShift;
    iVerMin = (iCurrY - offsetLCUY - iBlkHeight) << iMvShift;
    iVerMax = (iCurrY) << iMvShift;
#endif
  }
  else if (regionId == 2) // Left-bottom outside CTU
  {
    iHorMax = (iCurrX - offsetLCUX - iBlkWidth) << iMvShift;
    iVerMin = (iCurrY + 1) << iMvShift;
    iVerMax = (iCurrY - offsetLCUY + iCTUsize - iBlkHeight) << iMvShift;
  }
  else if (regionId == 3)   // Top within CTU
  {
    iVerMin = (iCurrY - offsetLCUY - iBlkHeight + 1) << iMvShift;
    iVerMax = (iCurrY - iBlkHeight) << iMvShift;
#if JVET_AD0086_ENHANCED_INTRA_TMP
    iHorMin = (iCurrX - iBlkWidth + 1) << iMvShift;
    iHorMax = (iCurrX) << iMvShift;
#else
    iHorMin = (iCurrX - offsetLCUX - iBlkWidth + 1) << iMvShift;
    iHorMax = (iCurrX - iBlkWidth) << iMvShift;
#endif
  }
#if JVET_AE0077_EXT_INTRATMP
  else if (regionId == 4)   // Bottom-Left within CTU
  {
    iHorMin = (iCurrX - offsetLCUX - iBlkWidth + 1) << iMvShift;
    iHorMax = (iCurrX - iBlkWidth) << iMvShift;
    iVerMin = (iCurrY + 1) << iMvShift;
    iVerMax = (iCurrY - offsetLCUY + iCTUsize - iBlkHeight) << iMvShift;
  }
  else if (regionId == 5)   // Top-right within CTU
  {
    iHorMin = (iCurrX + 1) << iMvShift;
    iHorMax = (iCurrX - offsetLCUX + iCTUsize - iBlkWidth) << iMvShift;
    iVerMin = (iCurrY - offsetLCUY - iBlkHeight + 1) << iMvShift;
    iVerMax = (iCurrY - iBlkHeight) << iMvShift;
  }
#endif

  // Clipping by frame boundaries
  iVerMin = std::max(iVerMin, (tempType != LEFT_TEMPLATE) ? iTemplateSize << iMvShift : 0);
  iVerMax = std::min(iVerMax, ((int)pcCU->cs->pps->getPicHeightInLumaSamples() - iBlkHeight) << iMvShift);
  iHorMin = std::max(iHorMin, (tempType != ABOVE_TEMPLATE) ? iTemplateSize << iMvShift : 0);
  iHorMax = std::min(iHorMax, ((int)pcCU->cs->pps->getPicWidthInLumaSamples() - iBlkWidth) << iMvShift);

  // Clipping by search range
#if !JVET_AE0077_EXT_INTRATMP
  if (regionId == 0)
  {
#endif
  iVerMin = std::max(iVerMin, (iCurrY - searchRangeHeight) << iMvShift);
  iVerMax = std::min(iVerMax, (iCurrY + searchRangeHeight) << iMvShift);
#if !JVET_AE0077_EXT_INTRATMP
  }
  if (regionId != 3)
  {
#endif
  iHorMin = std::max(iHorMin, (iCurrX - searchRangeWidth) << iMvShift);
  iHorMax = std::min(iHorMax, (iCurrX + searchRangeWidth) << iMvShift);
#if !JVET_AE0077_EXT_INTRATMP
  }
#endif

  iHorMin = iHorMin - iCurrX;
  iHorMax = iHorMax - iCurrX;
  iVerMax = iVerMax - iCurrY;
  iVerMin = iVerMin - iCurrY;
}

#if JVET_AB0130_ITMP_SAMPLING
void clipMvIntraConstraintRefine(int& iHorMin, int& iHorMax, int& iVerMin, int& iVerMax,int pX, int pY, int refinementRange)
{
#if JVET_AD0086_ENHANCED_INTRA_TMP
  iHorMin = std::max(iHorMin, pX - refinementRange + (TMP_SAMPLING % 2 ? 0 : 1));
  iHorMax = std::min(iHorMax, pX + refinementRange);
  iVerMin = std::max(iVerMin, pY - refinementRange + (TMP_SAMPLING % 2 ? 0 : 1));
  iVerMax = std::min(iVerMax, pY + refinementRange);
#else
  iHorMin = std::max(iHorMin, pX - refinementRange);
  iHorMax = std::min(iHorMax, pX + refinementRange);
  iVerMin = std::max(iVerMin, pY - refinementRange);
  iVerMax = std::min(iVerMax, pY + refinementRange);
#endif
}
#endif

TempLibFast::TempLibFast()
{
}

TempLibFast::~TempLibFast()
{
}

#if !JVET_AD0086_ENHANCED_INTRA_TMP
void TempLibFast::initTemplateDiff( unsigned int uiPatchWidth, unsigned int uiPatchHeight, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int bitDepth )
{
  int maxValue = ((1 << bitDepth) >> (INIT_THRESHOULD_SHIFTBITS)) * (uiPatchHeight * uiPatchWidth - uiBlkHeight * uiBlkWidth);
  m_diffMax = maxValue;
  {
    m_pDiff = maxValue;
  }
}
#endif

#if JVET_W0069_TMP_BOUNDARY
void IntraPrediction::getTargetTemplate( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType )
#else
void IntraPrediction::getTargetTemplate( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight )
#endif
{
  const ComponentID compID = COMPONENT_Y;
  unsigned int uiPatchWidth = uiBlkWidth + TMP_TEMPLATE_SIZE;
  unsigned int uiPatchHeight = uiBlkHeight + TMP_TEMPLATE_SIZE;
  unsigned int uiTarDepth = floorLog2( std::max( uiBlkHeight, uiBlkWidth ) ) - 2;
  Pel** tarPatch = m_pppTarPatch[uiTarDepth];
  CompArea area = pcCU->blocks[compID];
  Pel* pCurrStart = pcCU->cs->picture->getRecoBuf( area ).buf;
  unsigned int  uiPicStride = pcCU->cs->picture->getRecoBuf( compID ).stride;
  unsigned int uiY, uiX;

  //fill template
  //up-left & up 
  Pel* tarTemp;
#if JVET_W0069_TMP_BOUNDARY
  if( tempType == L_SHAPE_TEMPLATE )
  {
#endif
    Pel* pCurrTemp = pCurrStart - TMP_TEMPLATE_SIZE * uiPicStride - TMP_TEMPLATE_SIZE;
    for( uiY = 0; uiY < TMP_TEMPLATE_SIZE; uiY++ )
    {
      tarTemp = tarPatch[uiY];
      for( uiX = 0; uiX < uiPatchWidth; uiX++ )
      {
        tarTemp[uiX] = pCurrTemp[uiX];
      }
      pCurrTemp += uiPicStride;
    }
    //left
    for( uiY = TMP_TEMPLATE_SIZE; uiY < uiPatchHeight; uiY++ )
    {
      tarTemp = tarPatch[uiY];
      for( uiX = 0; uiX < TMP_TEMPLATE_SIZE; uiX++ )
      {
        tarTemp[uiX] = pCurrTemp[uiX];
      }
      pCurrTemp += uiPicStride;
    }
#if JVET_W0069_TMP_BOUNDARY
  }
  else if( tempType == ABOVE_TEMPLATE )
  {
    Pel* pCurrTemp = pCurrStart - TMP_TEMPLATE_SIZE * uiPicStride;
    for( uiY = 0; uiY < TMP_TEMPLATE_SIZE; uiY++ )
    {
      tarTemp = tarPatch[uiY];
      for( uiX = 0; uiX < uiBlkWidth; uiX++ )
      {
        tarTemp[uiX] = pCurrTemp[uiX];
      }
      pCurrTemp += uiPicStride;
    }
  }
  else if( tempType == LEFT_TEMPLATE )
  {
    Pel* pCurrTemp = pCurrStart - TMP_TEMPLATE_SIZE;
    for( uiY = TMP_TEMPLATE_SIZE; uiY < uiPatchHeight; uiY++ )
    {
      tarTemp = tarPatch[uiY];
      for( uiX = 0; uiX < TMP_TEMPLATE_SIZE; uiX++ )
      {
        tarTemp[uiX] = pCurrTemp[uiX];
      }
      pCurrTemp += uiPicStride;
    }
  }
#endif
}

#if JVET_W0069_TMP_BOUNDARY
void IntraPrediction::candidateSearchIntra( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight, RefTemplateType tempType )
#else
void IntraPrediction::candidateSearchIntra( CodingUnit* pcCU, unsigned int uiBlkWidth, unsigned int uiBlkHeight )
#endif
{
  unsigned int uiPatchWidth = uiBlkWidth + TMP_TEMPLATE_SIZE;
  unsigned int uiPatchHeight = uiBlkHeight + TMP_TEMPLATE_SIZE;
  unsigned int uiTarDepth = floorLog2( std::max( uiBlkWidth, uiBlkHeight ) ) - 2;

  Pel** tarPatch = getTargetPatch( uiTarDepth );
  //Initialize the library for saving the best candidates
#if !JVET_AD0086_ENHANCED_INTRA_TMP
  const ComponentID compID = COMPONENT_Y;
  const int channelBitDepth = pcCU->cs->sps->getBitDepth(toChannelType(compID));
  m_tempLibFast.initTemplateDiff( uiPatchWidth, uiPatchHeight, uiBlkWidth, uiBlkHeight, channelBitDepth );
#endif
#if JVET_W0069_TMP_BOUNDARY
  searchCandidateFromOnePicIntra( pcCU, tarPatch, uiPatchWidth, uiPatchHeight, tempType );
#else
  searchCandidateFromOnePicIntra( pcCU, tarPatch, uiPatchWidth, uiPatchHeight, );
#endif
#if !JVET_AD0086_ENHANCED_INTRA_TMP
  //count collected candidate number
  int pDiff = m_tempLibFast.getDiff();
  int maxDiff = m_tempLibFast.getDiffMax();


  if( pDiff < maxDiff )
  {
    m_uiVaildCandiNum = 1;
  }
  else
  {
    m_uiVaildCandiNum = 0;
  }
#if TMP_FAST_ENC
  pcCU->tmpNumCand = m_uiVaildCandiNum;
#endif
#endif   
}

#if JVET_W0069_TMP_BOUNDARY
void IntraPrediction::searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, RefTemplateType tempType )
#else
void IntraPrediction::searchCandidateFromOnePicIntra( CodingUnit* pcCU, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, )
#endif
{
#if JVET_AD0086_ENHANCED_INTRA_TMP
  m_mtmpCandList.clear();
  m_mtmpCostList.clear();
  static_vector<TempLibFast, MTMP_NUM_SPARSE> sparseMtmpCandList[3];
  static_vector<uint64_t, MTMP_NUM_SPARSE>    sparseMtmpCostList[3];
  int                                         mtmpNumSparse[3] = { MTMP_NUM_SPARSE, TL_NUM_SPARSE, TL_NUM_SPARSE };
#endif

  const ComponentID compID      = COMPONENT_Y;
  unsigned int      uiBlkWidth  = uiPatchWidth - TMP_TEMPLATE_SIZE;
  unsigned int      uiBlkHeight = uiPatchHeight - TMP_TEMPLATE_SIZE;

#if !JVET_AD0086_ENHANCED_INTRA_TMP
  int      pX        = m_tempLibFast.getX();
  int      pY        = m_tempLibFast.getY();
  int      pDiff     = m_tempLibFast.getDiff();
#endif
  CompArea area      = pcCU->blocks[compID];
  int      refStride = pcCU->cs->picture->getRecoBuf(compID).stride;

  Pel* ref = pcCU->cs->picture->getRecoBuf( area ).buf;

  setRefPicUsed( ref ); //facilitate the access of each candidate point 
  setStride( refStride );

  Mv cTmpMvPred;
  cTmpMvPred.setZero();

  unsigned int uiCUPelY = area.pos().y;
  unsigned int uiCUPelX = area.pos().x;
  int          blkX     = 0;
  int          blkY     = 0;
  int          iCurrY   = uiCUPelY + blkY;
  int          iCurrX   = uiCUPelX + blkX;

  Position ctuRsAddr  = CU::getCtuXYAddr(*pcCU);
  int      offsetLCUY = iCurrY - ctuRsAddr.y;
  int      offsetLCUX = iCurrX - ctuRsAddr.x;

  int iYOffset, iXOffset;
#if JVET_AD0086_ENHANCED_INTRA_TMP
  int numPixTopLeft = uiPatchWidth * TMP_TEMPLATE_SIZE + TMP_TEMPLATE_SIZE * uiBlkHeight;
  int numPixTop     = uiBlkWidth * TMP_TEMPLATE_SIZE;
  int numPixLeft    = TMP_TEMPLATE_SIZE * uiBlkHeight;
  const int channelBitDepth = pcCU->cs->sps->getBitDepth(toChannelType(compID));
  int       diff[3];
  int       pDiff[3];
  pDiff[0] = ((1 << channelBitDepth) >> (INIT_THRESHOULD_SHIFTBITS)) * numPixTopLeft;
  pDiff[1] = ((1 << channelBitDepth) >> (INIT_THRESHOULD_SHIFTBITS)) * numPixTop;
  pDiff[2] = ((1 << channelBitDepth) >> (INIT_THRESHOULD_SHIFTBITS)) * numPixLeft;
#else
  int diff;
#endif
  Pel* refCurr;

#if JVET_AD0086_ENHANCED_INTRA_TMP
#if JVET_AE0077_EXT_INTRATMP
  const int regionNum = 6;
#else
  const int regionNum = 4;
#endif
#else
  const int regionNum = 3;
#endif
#if JVET_AB0130_ITMP_SAMPLING && !JVET_AD0086_ENHANCED_INTRA_TMP
  int mvYMins[regionNum + 1];
  int mvYMaxs[regionNum + 1];
  int mvXMins[regionNum + 1];
  int mvXMaxs[regionNum + 1];
#else
  int mvYMins[regionNum];
  int mvYMaxs[regionNum];
  int mvXMins[regionNum];
  int mvXMaxs[regionNum];
#endif
  int regionId = 0;
#if JVET_AB0130_ITMP_SAMPLING
#if JVET_AE0077_EXT_INTRATMP
  int bestRegionId = regionNum;
#else
  int bestRegionId = 4;
#endif
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  bool needTopLeft =
    pcCU->cs->pcv->isEncoder
      ? true
      : (pcCU->tmpFusionFlag ? (((pcCU->tmpIdx % TMP_GROUP_IDX) + 1) * TMP_FUSION_NUM) : pcCU->tmpIdx + 1) > (MTMP_NUM - TL_NUM * 2) ? true : false;

  for (regionId = 0; regionId < regionNum; regionId++)
  {
    clipMvIntraConstraint(pcCU, regionId, mvXMins[regionId], mvXMaxs[regionId], mvYMins[regionId], mvYMaxs[regionId],
                          TMP_TEMPLATE_SIZE, uiBlkWidth, uiBlkHeight, iCurrY, iCurrX, offsetLCUY, offsetLCUX, tempType);
  }

  for (int checkIdx = 0; checkIdx < regionNum; checkIdx++)
  {
    regionId = (checkIdx + 3) % regionNum;   // 3->0->1->2

    int mvYMin = mvYMins[regionId];
    int mvYMax = mvYMaxs[regionId];
    int mvXMin = mvXMins[regionId];
    int mvXMax = mvXMaxs[regionId];
    if (mvYMax < mvYMin || mvXMax < mvXMin)
    {
      continue;
    }
#if JVET_AB0130_ITMP_SAMPLING
    for (iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset -= TMP_SAMPLING)
    {
      for (iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset -= TMP_SAMPLING)
      {
#if JVET_AE0077_EXT_INTRATMP
        if (regionId == 4 || regionId == 5)
        {
          Position bottomRight(iCurrX + iXOffset + uiBlkWidth - 1, iCurrY + iYOffset + uiBlkHeight - 1);
          if (!pcCU->cs->isDecomp(bottomRight, CHANNEL_TYPE_LUMA))
          {
            continue;
          }
        }
#endif
#else
    for (iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset--)
    {
      for (iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset--)
      {
#endif
        refCurr = ref + iYOffset * refStride + iXOffset;

        m_calcTemplateDiff(refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, diff, pDiff, tempType, needTopLeft ? 3 : 0);

        for (int temIdx = 0; temIdx < 3; temIdx++)
        {
          if (diff[temIdx] < pDiff[temIdx])
          {
            updateCandList(TempLibFast(iXOffset, iYOffset, regionId), diff[temIdx],
                           sparseMtmpCandList[temIdx], sparseMtmpCostList[temIdx], mtmpNumSparse[temIdx]);
            if (sparseMtmpCandList[temIdx].size() == mtmpNumSparse[temIdx])
            {
              pDiff[temIdx] = std::min((int) sparseMtmpCostList[temIdx][mtmpNumSparse[temIdx] - 1], pDiff[temIdx]);
            }
          }
        }
      }
    }
  }
#else
  //1. check the near pixels within LCU
  //above pixels in LCU
  int iTemplateSize = TMP_TEMPLATE_SIZE;
  int iBlkWidth = uiBlkWidth;
  int iBlkHeight = uiBlkHeight;

  //check within CTU pixels
  for( regionId = regionNum; regionId < regionNum+1; regionId++ )
  {
    clipMvIntraConstraint(pcCU, regionId, mvXMins[regionId], mvXMaxs[regionId], mvYMins[regionId], mvYMaxs[regionId],
      TMP_TEMPLATE_SIZE, uiBlkWidth, uiBlkHeight, iCurrY, iCurrX, offsetLCUY, offsetLCUX, tempType);

    int mvYMin = mvYMins[regionId];
    int mvYMax = mvYMaxs[regionId];
    int mvXMin = mvXMins[regionId];
    int mvXMax = mvXMaxs[regionId];
    if( mvYMax < mvYMin || mvXMax < mvXMin )
    {
      continue;
    }
#if JVET_AB0130_ITMP_SAMPLING
    for (iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset -= TMP_SAMPLING)
    {
      for (iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset -= TMP_SAMPLING)
      {
#else
    for( iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset-- )
    {
      for( iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset-- )
      {
#endif
        refCurr = ref + iYOffset * refStride + iXOffset;
#if JVET_W0069_TMP_BOUNDARY
        diff = m_calcTemplateDiff( refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, pDiff, tempType );
#else
        diff = m_calcTemplateDiff( refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, pDiff );
#endif
        if( diff < (pDiff) )
        {
          insertNode( diff, iXOffset, iYOffset, pDiff, pX, pY );
#if JVET_AB0130_ITMP_SAMPLING
          bestRegionId = regionId;
#endif
        }
        if( pDiff == 0 )
        {
          regionId++;
        }
      }
    }
  }

  //2. check the pixels outside CTU
  for( regionId = 0; regionId < regionNum; regionId++ )
  {
    // this function fills in the range the template matching for pixels outside the current CTU
    clipMvIntraConstraint(pcCU, regionId, mvXMins[regionId], mvXMaxs[regionId], mvYMins[regionId], mvYMaxs[regionId],
                          TMP_TEMPLATE_SIZE, uiBlkWidth, uiBlkHeight, iCurrY, iCurrX, offsetLCUY, offsetLCUX, tempType);
  }

  for( regionId = 0; regionId < regionNum; regionId++ )
  {
    int mvYMin = mvYMins[regionId];
    int mvYMax = mvYMaxs[regionId];
    int mvXMin = mvXMins[regionId];
    int mvXMax = mvXMaxs[regionId];
    if( mvYMax < mvYMin || mvXMax < mvXMin )
    {
      continue;
    }
#if JVET_AB0130_ITMP_SAMPLING
    for (iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset -= TMP_SAMPLING)
    {
      for (iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset -= TMP_SAMPLING)
      {
#else
    for( iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset-- )
    {
      for( iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset-- )
      {
#endif
        refCurr = ref + iYOffset * refStride + iXOffset;
#if JVET_W0069_TMP_BOUNDARY
        diff = m_calcTemplateDiff( refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, pDiff, tempType );
#else
        diff = m_calcTemplateDiff( refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, pDiff );
#endif

        if( diff < (pDiff) )
        {
          insertNode( diff, iXOffset, iYOffset, pDiff, pX, pY );
#if JVET_AB0130_ITMP_SAMPLING
          bestRegionId = regionId;
#endif
        }

        if( pDiff == 0 )
        {
          regionId = regionNum;
        }
      }
    }
  }
#endif 

#if JVET_AD0086_ENHANCED_INTRA_TMP
  static_vector<TempLibFast, MTMP_NUM> refineMtmpCandList[3];
  static_vector<uint64_t, MTMP_NUM>    refineMtmpCostList[3];
  int                                  mtmpNumRefine[3] = { MTMP_NUM, TL_NUM, TL_NUM };
  if (!needTopLeft)
  {
    mtmpNumRefine[0] = pcCU->tmpFusionFlag ? (((pcCU->tmpIdx % TMP_GROUP_IDX) + 1) * TMP_FUSION_NUM) : (pcCU->tmpIdx + 1); 
  }
  int pDiffSparse[3];
  for (int i = 0; i < 3; i++)
  {
    pDiffSparse[i] = pDiff[i]+(sparseMtmpCandList[i].size() < mtmpNumSparse[i] ? 0 : 1);
  }
  for (int temIdx = 0; temIdx < 3; temIdx++)
  {
    if ((tempType != L_SHAPE_TEMPLATE || !needTopLeft) && temIdx > 0)
    {
      continue;
    }
    for (int i = 0; i < 3; i++)
    {
      if (temIdx == i)
      {
        pDiff[i] = pDiffSparse[i];
      }
      else
      {
        pDiff[i] = 0;
      }
    }
    for (int candIdx = 0; candIdx < sparseMtmpCandList[temIdx].size(); candIdx++)
    {
      int iRefine      = 1;
      int iRefineRange = TMP_SAMPLING >> 1;
      bestRegionId     = sparseMtmpCandList[temIdx][candIdx].m_rId;
      int mvYMin       = mvYMins[bestRegionId];
      int mvYMax       = mvYMaxs[bestRegionId];
      int mvXMin       = mvXMins[bestRegionId];
      int mvXMax       = mvXMaxs[bestRegionId];
      clipMvIntraConstraintRefine(mvXMin, mvXMax, mvYMin, mvYMax, sparseMtmpCandList[temIdx][candIdx].m_pX, sparseMtmpCandList[temIdx][candIdx].m_pY, iRefineRange);

      if (!(mvYMax < mvYMin || mvXMax < mvXMin))
      {
        for (iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset -= iRefine)
        {
          for (iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset -= iRefine)
          {
#if JVET_AE0077_EXT_INTRATMP
            if (bestRegionId == 4 || bestRegionId == 5)
            {
              Position bottomRight(iCurrX + iXOffset + uiBlkWidth - 1, iCurrY + iYOffset + uiBlkHeight - 1);
              if (!pcCU->cs->isDecomp(bottomRight, CHANNEL_TYPE_LUMA))
              {
                continue;
              }
            }
#endif
            if (iXOffset == sparseMtmpCandList[temIdx][candIdx].m_pX
                && iYOffset == sparseMtmpCandList[temIdx][candIdx].m_pY)
            {
              diff[temIdx] = (int) sparseMtmpCostList[temIdx][candIdx];
            }
            else
            {
              refCurr = ref + iYOffset * refStride + iXOffset;
              m_calcTemplateDiff(refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, diff, pDiff, tempType,
                                 temIdx);
            }
            if (diff[temIdx] < pDiff[temIdx])
            {
              updateCandList(TempLibFast(iXOffset, iYOffset, bestRegionId), diff[temIdx],
                             refineMtmpCandList[temIdx], refineMtmpCostList[temIdx], mtmpNumRefine[temIdx]);
              if (refineMtmpCandList[temIdx].size() == mtmpNumRefine[temIdx])
              {
                pDiff[temIdx] = std::min((int) refineMtmpCostList[temIdx][mtmpNumRefine[temIdx] - 1], pDiff[temIdx]);
              }
            }
          }
        }
      }
    }
  }

  m_mtmpCandList = refineMtmpCandList[0];
  m_mtmpCostList = refineMtmpCostList[0];

  if (tempType == L_SHAPE_TEMPLATE && needTopLeft)
  {
    for (int temIdx = 2; temIdx >0; temIdx--)
    {
      int cnt = 0;
      for (int candIdx = 0; candIdx < refineMtmpCostList[temIdx].size() && cnt < TL_NUM; candIdx++)
      {
        // check redundancy
        bool bRedundant = false;

        int  mvXCur     = refineMtmpCandList[temIdx][candIdx].m_pX;
        int  mvYCur     = refineMtmpCandList[temIdx][candIdx].m_pY;
        for (int crIdx = 0; crIdx < m_mtmpCandList.size(); crIdx++)
        {
          if (mvXCur == m_mtmpCandList[crIdx].m_pX && mvYCur == m_mtmpCandList[crIdx].m_pY)
          {
            bRedundant = true;
          }
        }

        if (!bRedundant)
        {
          cnt++;
          int pos = MTMP_NUM - 1 - TL_NUM * temIdx + cnt;
          if (pos < m_mtmpCandList.size())
          {
            for (int updatePos = (int) m_mtmpCandList.size() - 1; updatePos > pos; updatePos--)
            {
              m_mtmpCandList[updatePos] = m_mtmpCandList[updatePos - 1];
            }
            m_mtmpCandList[pos] = refineMtmpCandList[temIdx][candIdx];
          }
        }
      }
    }
  }
#else
#if JVET_AB0130_ITMP_SAMPLING
  // now perform refinment with horizontal and vertical flips
  if (bestRegionId < 4)
  {
    for (int iRefineLog2 = LOG2_TMP_SAMPLING - 1; iRefineLog2 >= 0; iRefineLog2--)
    {
      int iRefine = 1 << iRefineLog2;
      int tmpRefineRange = std::min(uiBlkHeight, uiBlkHeight) / 2;
      int mvYMin = mvYMins[bestRegionId];
      int mvYMax = mvYMaxs[bestRegionId];
      int mvXMin = mvXMins[bestRegionId];
      int mvXMax = mvXMaxs[bestRegionId];
      clipMvIntraConstraintRefine(mvXMin, mvXMax, mvYMin, mvYMax, pX, pY, iRefine* tmpRefineRange);
      if (!(mvYMax < mvYMin || mvXMax < mvXMin))
      {
        for (iYOffset = mvYMax; iYOffset >= mvYMin; iYOffset -= iRefine)
        {
          for (iXOffset = mvXMax; iXOffset >= mvXMin; iXOffset -= iRefine)
          {
            refCurr = ref + iYOffset * refStride + iXOffset;
#if JVET_W0069_TMP_BOUNDARY
            diff = m_calcTemplateDiff(refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, pDiff, tempType);
#else
            diff = m_calcTemplateDiff(refCurr, refStride, tarPatch, uiPatchWidth, uiPatchHeight, pDiff);
#endif
            if (diff < (pDiff))
            {
              insertNode(diff, iXOffset, iYOffset, pDiff, pX, pY);
            }
          }
        }
      }
    }
  }
#endif
  m_tempLibFast.m_pX    = pX;
  m_tempLibFast.m_pY    = pY;
  m_tempLibFast.m_pDiff = pDiff;
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  m_tmpNumCand = (int) m_mtmpCandList.size();
  for (int i = 0; i < m_tmpNumCand; i++)
  {
    m_tmpXdisp[i] = m_mtmpCandList[i].m_pX;
    m_tmpYdisp[i] = m_mtmpCandList[i].m_pY;
  }

  if (m_tmpNumCand == 0)
  {
    return;
  }

  for (int i = m_tmpNumCand; i < MTMP_NUM; i++)
  {
    m_tmpXdisp[i] = m_tmpXdisp[i - m_tmpNumCand];
    m_tmpYdisp[i] = m_tmpYdisp[i - m_tmpNumCand];
    m_mtmpCostList.push_back(m_mtmpCostList[i - m_tmpNumCand]);
  }
#else 
#if TMP_FAST_ENC
  m_tmpXdisp = pX;
  m_tmpYdisp = pY;
#endif
#endif   
}

#if JVET_AD0086_ENHANCED_INTRA_TMP
int IntraPrediction::xCalTMPFusionNumber(const int maxNum, const int numIdx)
{
  int tmpFusionNum = 1;
  int offset       = maxNum * numIdx;
  int threshold    = numIdx ? int(m_mtmpCostList[offset] * 1.2) : int(m_mtmpCostList[offset] * 2);

  for (int i = 1; i < maxNum; i++)
  {
    if (m_mtmpCostList[offset + i] > threshold)
    {
      break;
    }
    tmpFusionNum++;
  }
  return tmpFusionNum;
}

void IntraPrediction::convertDiff2Weight(int *pDiff, int *weights, const int start, const int foundCandiNum)
{
  if (foundCandiNum <= 1)
  {
    weights[0] = 64;
    return;
  }
  int end = foundCandiNum + start - 1;

  const int blendSumWeight = 6;
  int       sumWeight      = 1 << blendSumWeight;   // 64
  int       sumW           = 0;
  int       w[FUSION_IDX_NUM];

  for (int i = start; i <= end; i++)
  {
    if (!pDiff[i])
    {
      pDiff[i]++;
    }
    sumW += pDiff[i];
  }
  for (int i = start; i <= end; i++)
  {
    pDiff[i] = sumW - pDiff[i];
  }

  int sumDiff = (foundCandiNum - 1) * sumW;
  int x       = floorLog2(sumDiff);

  int norm = (sumDiff << 4 >> x) & 15;
  int v    = g_gradDivTable[norm] | 8;
  x += (norm != 0);
  int shift = x + 3;
  int add   = (1 << (shift - 1));

  sumW = 0;
  for (int j = start; j <= end; j++)
  {
    w[j] = (pDiff[j] * v * sumWeight + add) >> shift;
    sumW += w[j];
  }
  while (sumW > sumWeight)
  {
    for (int j = end; j >= start && sumW > sumWeight; j--)
    {
      if (w[j])
      {
        w[j]--;
        sumW--;
      }
    }
  }
  while (sumW < sumWeight)
  {
    for (int j = start; j <= end && sumW < sumWeight; j++)
    {
      w[j]++;
      sumW++;
    }
  }
  for (int i = start; i <= end; i++)
  {
    weights[i - start] = w[i];
  }
}

void IntraPrediction::xTMPBuildFusionCandidate(CodingUnit &cu, RefTemplateType tempType)
{
  if (!m_tmpNumCand)
  {
    return;
  }

  m_tmpFusionInfo[0] = IntraTMPFusionInfo{ true, false, 0, TMP_FUSION_NUM };
  m_tmpFusionInfo[1] = IntraTMPFusionInfo{ true, false, TMP_FUSION_NUM, TMP_FUSION_NUM };
  m_tmpFusionInfo[2] = IntraTMPFusionInfo{ true, false, TMP_FUSION_NUM << 1, TMP_FUSION_NUM };
  m_tmpFusionInfo[3] = IntraTMPFusionInfo{ true, true, 0, TMP_FUSION_NUM };
  m_tmpFusionInfo[4] = IntraTMPFusionInfo{ true, true, TMP_FUSION_NUM, TMP_FUSION_NUM };
  m_tmpFusionInfo[5] = IntraTMPFusionInfo{ true, true, TMP_FUSION_NUM << 1, TMP_FUSION_NUM };
  int tmpIdx = cu.tmpIdx;
  int idx0   = cu.cs->pcv->isEncoder ? 0 : cu.tmpIdx;
  int idx1   = cu.cs->pcv->isEncoder ? TMP_GROUP_IDX << 1 : cu.tmpIdx + 1;

  for (int i = idx0; i < idx1; i++)
  {
    cu.tmpIdx                           = i;
    m_tmpFusionInfo[i].tmpFusionNumber = xCalTMPFusionNumber(m_tmpFusionInfo[i].tmpMaxNum, 0);

    if (m_tmpFusionInfo[i].bFilter)
    {
      xTMPFusionCalcModels(&cu, cu.lwidth(), cu.lheight(), tempType);
    }
    else
    {
      static int iDiff[TMP_FUSION_NUM];
      const int  offset       = m_tmpFusionInfo[i].tmpFusionIdx;
      const int  foundCandNum = m_tmpFusionInfo[i].tmpFusionNumber;
      for (int i = 0; i < foundCandNum; i++)
      {
        iDiff[i] = (int) m_mtmpCostList[offset + i];
      }
      convertDiff2Weight(iDiff, m_tmpFusionInfo[i].tmpFusionWeight, 0, foundCandNum);
    }
  }
  cu.tmpIdx = tmpIdx;
  return;
}

void IntraPrediction::xTMPFusionCalcParams(CodingUnit *cu, CompArea area, CccmModel& tmpFusionModel,
                                           int foundCandiNum, RefTemplateType tempType, Pel *curPointTemplate,
                                           Pel *refPointTemplate[])
{
  int sampleNum = 0;

  int uiHeight = area.height;
  int uiWidth  = area.width;

  int picStride = cu->cs->picture->getRecoBuf(area).stride;   // refTemplate and curTemplate

  int areaWidth  = uiWidth + TMP_TEMPLATE_SIZE;
  int areaHeight = uiHeight + TMP_TEMPLATE_SIZE;
  int refSizeX   = TMP_TEMPLATE_SIZE;
  int refSizeY   = TMP_TEMPLATE_SIZE;

  if (tempType == L_SHAPE_TEMPLATE)
  {
    refSizeX   = TMP_TEMPLATE_SIZE;
    refSizeY   = TMP_TEMPLATE_SIZE;
    areaWidth  = uiWidth + TMP_TEMPLATE_SIZE;
    areaHeight = uiHeight + TMP_TEMPLATE_SIZE;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    refSizeX   = 0;
    refSizeY   = TMP_TEMPLATE_SIZE;
    areaWidth  = uiWidth;
    areaHeight = uiHeight + TMP_TEMPLATE_SIZE;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    refSizeX   = TMP_TEMPLATE_SIZE;
    refSizeY   = 0;
    areaWidth  = uiWidth + TMP_TEMPLATE_SIZE;
    areaHeight = uiHeight;
  }

#if JVET_AB0174_CCCM_DIV_FREE
  int curTemplatelumaOffset = 1 << (cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  int refPosX               = 0;
  int refPosY               = 0;
  if (refSizeX || refSizeY)
  {
    refPosX               = refSizeX > 0 ? refSizeX - 1 : 0;
    refPosY               = refSizeY > 0 ? refSizeY - 1 : 0;
    curTemplatelumaOffset = curPointTemplate[refPosY * picStride + refPosX];
  }
#endif

  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if (x >= refSizeX && y >= refSizeY)
      {
        continue;
      }

      int i;
#if JVET_AB0174_CCCM_DIV_FREE
      for (i = 0; i < foundCandiNum; i++)
      {
        m_a[i][sampleNum] = refPointTemplate[i][y * picStride + x] - refPointTemplate[i][refPosY * picStride + refPosX];
      }
#else
      for (i = 0; i < foundCandiNum; i++)
      {
        m_a[i][sampleNum] = refPointTemplate[i][y * picStride + x];
      }
#endif
      for (; i < TMP_FUSION_PARAMS - 1; i++)
      {
        m_a[i][sampleNum] = 0;
      }

      m_a[i++][sampleNum] = tmpFusionModel.bias();

      m_cb[sampleNum++] = curPointTemplate[y * picStride + x];
    }
  }
  if (!sampleNum)   // Number of samples can go to zero in the multimode case
  {
    tmpFusionModel.clearModel();
  }
  else
  {
#if JVET_AB0174_CCCM_DIV_FREE
    m_cccmSolver.solve1(m_a, m_cb, sampleNum, curTemplatelumaOffset, tmpFusionModel);
#else
    m_cccmSolver.solve1(m_a, m_cb, sampleNum, tmpFusionModel);
#endif
  }
}

void IntraPrediction::xTMPFusionCalcModels(CodingUnit *cu, unsigned int uiBlkWidth, unsigned int uiBlkHeight,
                                           RefTemplateType tempType)
{
  int foundCandiNum = m_tmpFusionInfo[cu->tmpIdx].tmpFusionNumber;

  if (foundCandiNum < 1)
  {
    return;
  }
  int      iOffsetY, iOffsetX;
  CompArea area      = cu->Y();
  Pel *    ref       = cu->cs->picture->getRecoBuf(area).buf;
  int      picStride = cu->cs->picture->getRecoBuf(area).stride;

  Pel *refPointTemplate[TMP_BEST_CANDIDATES] = { NULL };
  Pel *curPointTemplate                      = nullptr;

  for (int i = 0; i < foundCandiNum; i++)
  {
    iOffsetX = m_tmpXdisp[i + m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx];
    iOffsetY = m_tmpYdisp[i + m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx];

    if (tempType == L_SHAPE_TEMPLATE)
    {
      refPointTemplate[i] = (ref + iOffsetY * picStride + iOffsetX) - picStride * TMP_TEMPLATE_SIZE - TMP_TEMPLATE_SIZE;
      curPointTemplate    = ref - TMP_TEMPLATE_SIZE * picStride - TMP_TEMPLATE_SIZE;
    }
    else if (tempType == ABOVE_TEMPLATE)
    {
      refPointTemplate[i] = (ref + iOffsetY * picStride + iOffsetX) - picStride * TMP_TEMPLATE_SIZE;
      curPointTemplate    = ref - TMP_TEMPLATE_SIZE * picStride;
    }
    else if (tempType == LEFT_TEMPLATE)
    {
      refPointTemplate[i] = (ref + iOffsetY * picStride + iOffsetX) - TMP_TEMPLATE_SIZE;
      curPointTemplate    = ref - TMP_TEMPLATE_SIZE;
    }
  }
  CccmModel tmpFusionModel( TMP_FUSION_PARAMS, cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );

  xTMPFusionCalcParams(cu, area, tmpFusionModel, foundCandiNum, tempType, curPointTemplate, refPointTemplate);

  for (int i = 0; i < TMP_FUSION_PARAMS; i++)
  {
    m_tmpFusionInfo[cu->tmpIdx].tmpFushionParams[i] = tmpFusionModel.params[i];
  }
}

void IntraPrediction::xCalTmpFlmParam(CodingUnit *cu, unsigned int uiBlkWidth, unsigned int uiBlkHeight,
                                      RefTemplateType tempType)
{
  int foundCandiNum = m_tmpNumCand;

  if (foundCandiNum < 1)
  {
    return;
  }

  CompArea area = cu->Y();

  xGetTmpFlmRefBuf(cu, uiBlkWidth, uiBlkHeight, tempType);

  int areaWidth, areaHeight, refSizeX, refSizeY;

  refSizeX         = m_tmpRefArea[cu->tmpIdx].x;
  refSizeY         = m_tmpRefArea[cu->tmpIdx].y;
  areaWidth        = m_tmpRefArea[cu->tmpIdx].width;
  areaHeight       = m_tmpRefArea[cu->tmpIdx].height;
  int    refStride = areaWidth + 2 * TMP_FILTER_PADDING;   // Including paddings required for the 2D filter
  int    refOrigin = refStride * TMP_FILTER_PADDING + TMP_FILTER_PADDING;
  PelBuf tmpRefBuf = PelBuf(m_tmpRefBuf[cu->tmpIdx] + refOrigin, refStride, areaWidth, areaHeight);

  int sampleNum = 0;

  Pel *ref       = cu->cs->picture->getRecoBuf(area).buf;   // cur Template
  int  picStride = cu->cs->picture->getRecoBuf(area).stride;

  ref = ref - refSizeY * picStride - refSizeX;
#if JVET_AB0174_CCCM_DIV_FREE
  int offset = 1 << (cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  if (refSizeX || refSizeY)
  {
    int  refPosX   = refSizeX > 0 ? refSizeX - 1 : 0;
    int  refPosY   = refSizeY > 0 ? refSizeY - 1 : 0;
    Pel *refOffset = ref + refPosY * picStride + refPosX;
    offset         = refOffset[0];
  }
#endif
  // Collect reference data to input matrix A and target vector Y
  Pel* Y = m_cb;

  CccmModel tmpModel( TMP_FLM_PARAMS, cu->cs->sps->getBitDepth( CHANNEL_TYPE_LUMA ) );

  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if (x >= refSizeX && y >= refSizeY)
      {
        continue;
      }

      m_a[0][sampleNum] = tmpRefBuf.at(x, y);       // C
      m_a[1][sampleNum] = tmpRefBuf.at(x, y - 1);   // N
      m_a[2][sampleNum] = tmpRefBuf.at(x, y + 1);   // S
      m_a[3][sampleNum] = tmpRefBuf.at(x - 1, y);   // W
      m_a[4][sampleNum] = tmpRefBuf.at(x + 1, y);   // E
      m_a[5][sampleNum] = tmpModel.bias();

      Y[sampleNum++] = ref[x];
    }
    ref += picStride;
  }

  if (!sampleNum)   // Number of samples can go to zero in the multimode case
  {
    tmpModel.clearModel();
  }
  else
  {
    m_cccmSolver.solve1(m_a, Y, sampleNum, offset, tmpModel);
  }

  for (int i = 0; i < TMP_FLM_PARAMS; i++)
  {
    m_tmpFlmParams[i][cu->tmpIdx] = tmpModel.params[i];
  }
}

void IntraPrediction::xCalcTmpFlmRefArea(CodingUnit *cu, unsigned int uiBlkWidth, unsigned int uiBlkHeight,
                                         RefTemplateType tempType, bool &leftPadding, bool &rightPadding,
                                         bool &abovePadding, bool &belowPadding)
{
  int regionId      = m_mtmpCandList[cu->tmpIdx].m_rId;

  int      iHorMax = 0, iHorMin = 0, iVerMax = 0, iVerMin = 0;
  CompArea area       = cu->blocks[COMPONENT_Y];
  int      iCurrY     = area.pos().y;
  int      iCurrX     = area.pos().x;
  Position ctuRsAddr  = CU::getCtuXYAddr(*cu);
  int      offsetLCUY = iCurrY - ctuRsAddr.y;
  int      offsetLCUX = iCurrX - ctuRsAddr.x;

  int pX = m_tmpXdisp[cu->tmpIdx];
  int pY = m_tmpYdisp[cu->tmpIdx];

  CHECK(regionId < 0 || regionId > 5, "region Id error\n");
  clipMvIntraConstraint(cu, regionId, iHorMin, iHorMax, iVerMin, iVerMax,
    TMP_TEMPLATE_SIZE, uiBlkWidth, uiBlkHeight, iCurrY, iCurrX, offsetLCUY, offsetLCUX, tempType);

  leftPadding  = !(pX > iHorMin);
  rightPadding = !(pX < iHorMax);
  abovePadding = !(pY > iVerMin);
  belowPadding = !(pY < iVerMax);
#if JVET_AE0077_EXT_INTRATMP
  int iBlkWidth = uiBlkWidth;
  int iBlkHeight = uiBlkHeight;
  int bestPosX = iCurrX + pX;
  int bestPosY = iCurrY + pY;
  if (regionId == 4 || regionId == 5)
  {
    if (!cu->cs->isDecomp(Position(bestPosX + iBlkWidth - 1, bestPosY + iBlkHeight), CHANNEL_TYPE_LUMA))
    {
      belowPadding = true;
    }
    if (!cu->cs->isDecomp(Position(bestPosX + iBlkWidth, bestPosY + iBlkHeight - 1), CHANNEL_TYPE_LUMA))
    {
      rightPadding = true;
    }
  }
#endif
}

void IntraPrediction::xGetTmpFlmRefBuf(CodingUnit *cu, unsigned int uiBlkWidth, unsigned int uiBlkHeight,
                                       RefTemplateType tempType)
{
  int pX = m_tmpXdisp[cu->tmpIdx];
  int pY = m_tmpYdisp[cu->tmpIdx];

  int      iOffsetY, iOffsetX;
  Pel *    refTarget;
  CompArea area     = cu->Y();
  int      uiHeight = area.height;
  int      uiWidth  = area.width;

  Pel *ref       = cu->cs->picture->getRecoBuf(area).buf;
  int  picStride = cu->cs->picture->getRecoBuf(area).stride;

  iOffsetY       = pY;
  iOffsetX       = pX;
  refTarget      = ref + iOffsetY * picStride + iOffsetX;   // refTarget
  int areaWidth  = uiWidth + TMP_TEMPLATE_SIZE;
  int areaHeight = uiHeight + TMP_TEMPLATE_SIZE;
  int refSizeX   = TMP_TEMPLATE_SIZE;
  int refSizeY   = TMP_TEMPLATE_SIZE;

  Pel *refTemp    = nullptr;
  bool paddingTop = false, paddingLeft = false, paddingRight = false, paddingBottom = false;
  xCalcTmpFlmRefArea(cu, uiBlkWidth, uiBlkHeight, tempType, paddingLeft, paddingRight, paddingTop, paddingBottom);

  if (tempType == L_SHAPE_TEMPLATE)
  {
    refSizeX   = TMP_TEMPLATE_SIZE;
    refSizeY   = TMP_TEMPLATE_SIZE;
    areaWidth  = uiWidth + TMP_TEMPLATE_SIZE;
    areaHeight = uiHeight + TMP_TEMPLATE_SIZE;
    refTemp    = refTarget - TMP_TEMPLATE_SIZE * picStride - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    refSizeX   = 0;
    refSizeY   = TMP_TEMPLATE_SIZE;
    areaWidth  = uiWidth;
    areaHeight = uiHeight + TMP_TEMPLATE_SIZE;
    refTemp    = refTarget - TMP_TEMPLATE_SIZE * picStride;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    refSizeX   = TMP_TEMPLATE_SIZE;
    refSizeY   = 0;
    areaWidth  = uiWidth + TMP_TEMPLATE_SIZE;
    areaHeight = uiHeight;
    refTemp    = refTarget - TMP_TEMPLATE_SIZE;
  }
  m_tmpRefArea[cu->tmpIdx] = Area(refSizeX, refSizeY, areaWidth, areaHeight);

  int refStride = areaWidth + 2 * TMP_FILTER_PADDING;   // Including paddings required for the 2D filter
  int refOrigin = refStride * TMP_FILTER_PADDING + TMP_FILTER_PADDING;

  PelBuf tmpRefBuf = PelBuf(m_tmpRefBuf[cu->tmpIdx] + refOrigin, refStride, areaWidth, areaHeight);
  PelBuf srcRefBuf = PelBuf(refTemp, picStride, areaWidth, areaHeight);
#if JVET_AB0174_CCCM_DIV_FREE
  int offset = 1 << (cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  if (refSizeX || refSizeY)
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;
    offset      = srcRefBuf.at(refPosX, refPosY);
  }
#endif
  for (int y = (paddingTop ? 0 : -1); y < (paddingBottom ? areaHeight : areaHeight + 1); y++)
  {
    for (int x = (paddingLeft ? 0 : -1); x < (paddingRight ? areaWidth : areaWidth + 1); x++)
    {
#if JVET_AB0174_CCCM_DIV_FREE
      tmpRefBuf.at(x, y) = srcRefBuf.at(x, y) - offset;
#else
      tmpRefBuf.at(x, y) = srcRefBuf.at(x, y);
#endif
    }
  }

  // Pad top area
  if (paddingTop)
  {
    for (int x = (paddingLeft ? 0 : -1); x < (paddingRight ? areaWidth : areaWidth + 1); x++)
    {
      tmpRefBuf.at(x, -1) = tmpRefBuf.at(x, 0);
    }
  }
  // Pad bottom area
  if (paddingBottom)
  {
    for (int x = (paddingLeft ? 0 : -1); x < (paddingRight ? areaWidth : areaWidth + 1); x++)
    {
      tmpRefBuf.at(x, areaHeight) = tmpRefBuf.at(x, areaHeight - 1);
    }
  }

  // Pad right area
  if (paddingRight)
  {
    for (int y = -1; y <= areaHeight; y++)
    {
      tmpRefBuf.at(areaWidth, y) = tmpRefBuf.at(areaWidth - 1, y);
    }
  }
  // Pad left area
  if (paddingLeft)
  {
    for (int y = -1; y <= areaHeight; y++)
    {
      tmpRefBuf.at(-1, y) = tmpRefBuf.at(0, y);
    }
  }
}

void IntraPrediction::xTMPFusionApplyModel(PelBuf &piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight,
                                           RefTemplateType tempType, CodingUnit *cu, bool bDeriveDimdMode)
{
  bool bTmpFusion = cu->tmpFusionFlag && m_tmpFusionInfo[cu->tmpIdx].bValid;
  if (!bTmpFusion || !m_tmpFusionInfo[cu->tmpIdx].bFilter)
  {
    return;
  }
  int foundCandiNum = m_tmpFusionInfo[cu->tmpIdx].tmpFusionNumber;

  if (foundCandiNum < 1)
  {
    return;
  }

  CccmModel tmpFusionModel( TMP_FUSION_PARAMS, cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
  for (int i = 0; i < TMP_FUSION_PARAMS; i++)
  {
    tmpFusionModel.params[i] = m_tmpFusionInfo[cu->tmpIdx].tmpFushionParams[i];
  }
  CompArea      area      = cu->Y();
  Pel *         ref       = cu->cs->picture->getRecoBuf(area).buf;
  int           picStride = cu->cs->picture->getRecoBuf(area).stride;
  const ClpRng &clpRng(cu->cs->slice->clpRng(COMPONENT_Y));
  int           iOffsetY, iOffsetX;
  Pel*          samples = m_samples;
  Pel *         refPointPatch[TMP_BEST_CANDIDATES] = { NULL };

  memset( samples, 0, sizeof( Pel ) * TMP_FUSION_PARAMS);

  for (int i = 0; i < foundCandiNum; i++)
  {
    iOffsetY         = m_tmpYdisp[i + m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx];
    iOffsetX         = m_tmpXdisp[i + m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx];
    refPointPatch[i] = ref + iOffsetY * picStride + iOffsetX;
  }
  Pel *pPred    = piPred.buf;
  int  uiStride = piPred.stride;
  for (int y = 0; y < uiBlkHeight; y++)
  {
    for (int x = 0; x < uiBlkWidth; x++)
    {
      int i = 0;
      if (tempType == L_SHAPE_TEMPLATE)
      {
        for (i = 0; i < foundCandiNum; i++)
        {
          samples[i] = refPointPatch[i][y * picStride + x] - refPointPatch[i][-picStride - 1];
        }
      }
      else if (tempType == ABOVE_TEMPLATE)
      {
        for (i = 0; i < foundCandiNum; i++)
        {
          samples[i] = refPointPatch[i][y * picStride + x] - refPointPatch[i][-picStride];
        }
      }
      else if (tempType == LEFT_TEMPLATE)
      {
        for (i = 0; i < foundCandiNum; i++)
        {
          samples[i] = refPointPatch[i][y * picStride + x] - refPointPatch[i][-1];
        }
      }
      for (; i < TMP_FUSION_PARAMS - 1; i++)
      {
        samples[i] = 0;
      }
      samples[i++]            = tmpFusionModel.bias();
      pPred[y * uiStride + x] = ClipPel<Pel>(tmpFusionModel.convolve(samples), clpRng);
    }
  }
  int pX = m_tmpXdisp[m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx];
  int pY = m_tmpYdisp[m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx];

  cu->firstPU->interDir               = 1;
  cu->firstPU->refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
  cu->firstPU->mv->set(pX << MV_FRACTIONAL_BITS_INTERNAL, pY << MV_FRACTIONAL_BITS_INTERNAL);
  cu->firstPU->bv.set(pX, pY);

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  if (bDeriveDimdMode)
  {
    CPelBuf predBuf      = piPred;
    cu->intraTmpDimdMode = deriveDimdIntraTmpModePred(*cu, predBuf);
  }
#endif
  return;
}

void IntraPrediction::xGenerateTmpFlmPred(PelBuf &piPred, unsigned int uiBlkWidth, unsigned int uiBlkHeight,
                                          RefTemplateType tempType, CodingUnit *cu, bool bDeriveDimdMode)
{
  if (!cu->tmpFlmFlag)
  {
    return;
  }
  const ClpRng &            clpRng(cu->cs->slice->clpRng(COMPONENT_Y));
  Pel* samples = m_samples;
  CccmModel tmpModel( TMP_FLM_PARAMS, cu->cs->sps->getBitDepth(CHANNEL_TYPE_LUMA));

  for (int i = 0; i < TMP_FLM_PARAMS; i++)
  {
    tmpModel.params[i] = m_tmpFlmParams[i][cu->tmpIdx];
  }

  int refStride = m_tmpRefArea[cu->tmpIdx].width + 2 * TMP_FILTER_PADDING;   // Including paddings required for the 2D filter
  int refOrigin = refStride * (m_tmpRefArea[cu->tmpIdx].y + TMP_FILTER_PADDING) + m_tmpRefArea[cu->tmpIdx].x + TMP_FILTER_PADDING;
  PelBuf tmpRefBuf = PelBuf(m_tmpRefBuf[cu->tmpIdx] + refOrigin, refStride, uiBlkWidth, uiBlkHeight);

  for (int y = 0; y < tmpRefBuf.height; y++)
  {
    for (int x = 0; x < tmpRefBuf.width; x++)
    {
      samples[0] = tmpRefBuf.at(x, y);       // C
      samples[1] = tmpRefBuf.at(x, y - 1);   // N
      samples[2] = tmpRefBuf.at(x, y + 1);   // S
      samples[3] = tmpRefBuf.at(x - 1, y);   // W
      samples[4] = tmpRefBuf.at(x + 1, y);   // E
      samples[5] = tmpModel.bias();

      piPred.at(x, y) = ClipPel<Pel>(tmpModel.convolve(samples), clpRng);
    }
  }
  int pX = m_tmpXdisp[cu->tmpIdx];
  int pY = m_tmpYdisp[cu->tmpIdx];

  cu->firstPU->interDir               = 1;
  cu->firstPU->refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
  cu->firstPU->mv->set(pX << MV_FRACTIONAL_BITS_INTERNAL, pY << MV_FRACTIONAL_BITS_INTERNAL);
  cu->firstPU->bv.set(pX, pY);

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  if (bDeriveDimdMode)
  {
    CPelBuf predBuf      = piPred;
    cu->intraTmpDimdMode = deriveDimdIntraTmpModePred(*cu, predBuf);
  }
#endif
  return;
}

void IntraPrediction::xPadForInterpolation(CodingUnit *cu)
{
  CodingStructure &cs       = *cu->cs;
  int              uiWidth  = cu->lwidth();
  int              uiHeight = cu->lheight();

  Position pos = cu->Y().offset(m_tmpXdisp[cu->tmpIdx], m_tmpYdisp[cu->tmpIdx]);

  const UnitArea localUnitArea(cu->firstPU->chromaFormat,
                               Area(0, 0, uiWidth + 2 * TMP_SUBPEL_PAD_NUM, uiHeight + 2 * TMP_SUBPEL_PAD_NUM));
  PelBuf         dstBuffer = m_tempBuffer[0].getBuf(localUnitArea.Y());
  int            dstStride = dstBuffer.stride;
  Pel *          dst0      = dstBuffer.buf + TMP_SUBPEL_PAD_NUM + TMP_SUBPEL_PAD_NUM * dstStride;
  Pel *          dst       = dst0;

  int  srcStride = cu->cs->picture->getRecoBuf(cu->firstPU->Y()).stride;
  Pel *src0 =
    cu->cs->picture->getRecoBuf(cu->firstPU->Y()).buf + m_tmpXdisp[cu->tmpIdx] + m_tmpYdisp[cu->tmpIdx] * srcStride;
  Pel *src = src0;

  // block
  for (int j = 0; j < uiHeight; j++)
  {
    for (int i = 0; i < uiWidth; i++)
    {
      dst[i + j * dstStride] = src[i + j * srcStride];
    }
  }

  // above
  for (int j = -1; j >= -TMP_SUBPEL_PAD_NUM; j--)
  {
    for (int i = 0; i < uiWidth; i++)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i + (j + 1) * dstStride];
    }
  }

  // bottom
  for (int j = uiHeight; j < uiHeight + TMP_SUBPEL_PAD_NUM; j++)
  {
    for (int i = 0; i < uiWidth; i++)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i + (j - 1) * dstStride];
    }
  }

  // left
  for (int j = 0; j < uiHeight; j++)
  {
    for (int i = -1; i >= -TMP_SUBPEL_PAD_NUM; i--)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i + 1 + j * dstStride];
    }
  }

  // right
  for (int j = 0; j < uiHeight; j++)
  {
    for (int i = uiWidth; i < uiWidth + TMP_SUBPEL_PAD_NUM; i++)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i - 1 + j * dstStride];
    }
  }

  // aboveleft
  for (int j = -1; j >= -TMP_SUBPEL_PAD_NUM; j--)
  {
    for (int i = -1; i >= -TMP_SUBPEL_PAD_NUM; i--)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i + 1 + j * dstStride];
    }
  }

  // aboveright
  for (int j = -1; j >= -TMP_SUBPEL_PAD_NUM; j--)
  {
    for (int i = uiWidth; i < uiWidth + TMP_SUBPEL_PAD_NUM; i++)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i - 1 + j * dstStride];
    }
  }

  // bottomleft
  for (int j = uiHeight; j < uiHeight + TMP_SUBPEL_PAD_NUM; j++)
  {
    for (int i = -1; i >= -TMP_SUBPEL_PAD_NUM; i--)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i + 1 + j * dstStride];
    }
  }

  // bottomright
  for (int j = uiHeight; j < uiHeight + TMP_SUBPEL_PAD_NUM; j++)
  {
    for (int i = uiWidth; i < uiWidth + TMP_SUBPEL_PAD_NUM; i++)
    {
      dst[i + j * dstStride] =
        cs.isDecomp(pos.offset(i, j), CHANNEL_TYPE_LUMA) ? src[i + j * srcStride] : dst[i - 1 + j * dstStride];
    }
  }
}
#endif

#if !JVET_AD0086_ENHANCED_INTRA_TMP
#if TMP_FAST_ENC
bool IntraPrediction::generateTMPrediction(Pel* piPred, unsigned int uiStride, CompArea area, int& foundCandiNum, CodingUnit* cu)
#else
bool IntraPrediction::generateTMPrediction( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int& foundCandiNum )
#endif
{
  bool bSucceedFlag = true;
#if !TMP_FAST_ENC
  unsigned int uiPatchWidth = uiBlkWidth + TMP_TEMPLATE_SIZE;
  unsigned int uiPatchHeight = uiBlkHeight + TMP_TEMPLATE_SIZE;
#endif

#if TMP_FAST_ENC
  foundCandiNum = cu->tmpNumCand;
#else
  foundCandiNum = m_uiVaildCandiNum;
#endif
  if( foundCandiNum < 1 )
  {
    return false;
  }

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  Pel* pPred = piPred;
#endif

#if TMP_FAST_ENC
  int pX = cu->tmpXdisp;
  int pY = cu->tmpYdisp;
#else
  int pX = m_tempLibFast.getX();
  int pY = m_tempLibFast.getY();
  Pel* ref;
  int picStride = getStride();
#endif
  int iOffsetY, iOffsetX;
  Pel* refTarget;
#if TMP_FAST_ENC
  int uiHeight = area.height;
  int uiWidth = area.width;
  Pel* ref = cu->cs->picture->getRecoBuf(area).buf;
  int picStride = cu->cs->picture->getRecoBuf(area).stride;
#else
  unsigned int uiHeight = uiPatchHeight - TMP_TEMPLATE_SIZE;
  unsigned int uiWidth = uiPatchWidth - TMP_TEMPLATE_SIZE;

  //the data center: we use the prediction block as the center now.
  //collect the candidates
  ref = getRefPicUsed();
#endif
  iOffsetY  = pY;
  iOffsetX  = pX;
  refTarget = ref + iOffsetY * picStride + iOffsetX;
  for (unsigned int uiY = 0; uiY < uiHeight; uiY++)
  {
    for (unsigned int uiX = 0; uiX < uiWidth; uiX++)
    {
      piPred[uiX] = refTarget[uiX];
    }
    refTarget += picStride;
    piPred += uiStride;
  }

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  CPelBuf predBuf(pPred, uiStride, uiWidth, uiHeight);
  cu->intraTmpDimdMode = deriveDimdIntraTmpModePred(*cu, predBuf);
#endif
  return bSucceedFlag;
}
#endif

#if JVET_AB0061_ITMP_BV_FOR_IBC
#if JVET_AD0086_ENHANCED_INTRA_TMP
bool IntraPrediction::generateTMPrediction(Pel *piPred, unsigned int uiStride, int &foundCandiNum, PredictionUnit &pu, bool bDeriveDimdMode)
#else
bool IntraPrediction::generateTMPrediction(Pel *piPred, unsigned int uiStride, int &foundCandiNum, PredictionUnit &pu)
#endif
{
  bool         bSucceedFlag  = true;
#if !TMP_FAST_ENC
  unsigned int uiPatchWidth  = pu.lwidth() + TMP_TEMPLATE_SIZE;
  unsigned int uiPatchHeight = pu.lheight() + TMP_TEMPLATE_SIZE;
#endif

#if TMP_FAST_ENC
  foundCandiNum = m_tmpNumCand;
#else
  foundCandiNum = m_uiVaildCandiNum;
#endif
  if (foundCandiNum < 1)
  {
    return false;
  }

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  Pel* pPred = piPred;
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
  CodingUnit *cu         = pu.cu;
  CompArea    area       = pu.Y();
  int         uiHeight   = area.height;
  int         uiWidth    = area.width;
  Pel *       ref        = cu->cs->picture->getRecoBuf(area).buf;
  int         picStride  = cu->cs->picture->getRecoBuf(area).stride;
  int         pX         = m_tmpXdisp[0];
  int         pY         = m_tmpYdisp[0];

  bool bTmpFusion   = cu->tmpFusionFlag;
  int  tmpFusionNum = 0;
  int  tmpFusionIdx = 0;
  if (bTmpFusion)
  {
    bool bFilter      = m_tmpFusionInfo[cu->tmpIdx].bFilter;
    tmpFusionNum      = m_tmpFusionInfo[cu->tmpIdx].tmpFusionNumber;
    if (tmpFusionNum < 1 || bFilter)
    {
      return bSucceedFlag;
    }
    tmpFusionIdx      = m_tmpFusionInfo[cu->tmpIdx].tmpFusionIdx;
    pX         = m_tmpXdisp[tmpFusionIdx];
    pY         = m_tmpYdisp[tmpFusionIdx];
  }

  if (bTmpFusion)
  {
    const int log2WeightSum = 6;
    int *pDiff = m_tmpFusionInfo[cu->tmpIdx].tmpFusionWeight;
    Pel *refTarget[TMP_FUSION_NUM];
    for (int i = 0; i < tmpFusionNum; i++)
    {
      refTarget[i] = ref + m_tmpYdisp[i + tmpFusionIdx] * picStride + m_tmpXdisp[i + tmpFusionIdx];
    }
    const int shift = 1 << (log2WeightSum - 1);
    for (unsigned int uiY = 0; uiY < uiHeight; uiY++)
    {
      for (unsigned int uiX = 0; uiX < uiWidth; uiX++)
      {
        int blend = shift;

        for (int i = 0; i < tmpFusionNum; i++)
        {
          blend += (pDiff[i] * refTarget[i][uiX]);
        }
        piPred[uiX] = (Pel)(blend >> log2WeightSum);
      }
      for (int i = 0; i < tmpFusionNum; i++)
      {
        refTarget[i] += picStride;
      }
      piPred += uiStride;
    }
  }
  else
  {
    pX                            = m_tmpXdisp[cu->tmpIdx];
    pY                            = m_tmpYdisp[cu->tmpIdx];
    Pel *            refTarget    = ref + pY * picStride + pX;
    int              tmpIsSubPel  = cu->tmpIsSubPel;
    int              tmpSubPelIdx = cu->tmpSubPelIdx;
    TmpSubPelDirType tmpSubPelDir = tmpIsSubPel ? (TmpSubPelDirType) tmpSubPelIdx : LEFT_POS;

    const UnitArea localUnitArea(cu->firstPU->chromaFormat,
                                 Area(0, 0, uiWidth + 2 * TMP_SUBPEL_PAD_NUM, uiHeight + 2 * TMP_SUBPEL_PAD_NUM));
    PelBuf         predBuffer = m_tempBuffer[0].getBuf(localUnitArea.Y());
    int            dstStride  = predBuffer.stride;
    Pel *          dst0       = predBuffer.buf + TMP_SUBPEL_PAD_NUM + TMP_SUBPEL_PAD_NUM * dstStride;
    Pel *          dst        = dst0;
    const ClpRng & clpRng(cu->cs->slice->clpRng(COMPONENT_Y));

    const TFilterCoeff *const f0 = InterpolationFilter::getChromaFilterTable(16);   // 1/2
    const TFilterCoeff *const f1 = InterpolationFilter::getChromaFilterTable(8);    // 1/4
    const TFilterCoeff *const f2 = InterpolationFilter::getChromaFilterTable(24);   // 3/4

    if (tmpIsSubPel == 0) // int-pel
    {
      for (unsigned int uiY = 0; uiY < uiHeight; uiY++)
      {
        memcpy(piPred, refTarget, uiWidth * sizeof(Pel));
        refTarget += picStride;
        piPred += uiStride;
      }
    }
    else if (tmpSubPelDir < ABOVE_LEFT_POS)
    {
      const TFilterCoeff *p = f0;
      if (tmpSubPelDir == LEFT_POS || tmpSubPelDir == ABOVE_POS)
      {
        p = (tmpIsSubPel == 1) ? f0 : ((tmpIsSubPel == 2) ? f2 : f1);
      }
      else
      {
        p = (tmpIsSubPel == 1) ? f0 : ((tmpIsSubPel == 2) ? f1 : f2);
      }

      if (tmpSubPelDir == LEFT_POS) // left
      {
        m_if.m_filterHor[3][1][true](clpRng, dst - 1, dstStride, piPred, uiStride, uiWidth, uiHeight, p, false);
      }
      else if (tmpSubPelDir == RIGHT_POS) // right
      {
        m_if.m_filterHor[3][1][true](clpRng, dst, dstStride, piPred, uiStride, uiWidth, uiHeight, p, false);
      }
      else if (tmpSubPelDir == ABOVE_POS) // top
      {
        m_if.m_filterVer[3][true][true](clpRng, dst - dstStride, dstStride, piPred, uiStride, uiWidth, uiHeight, p, false);
      }
      else // bottom
      {
        m_if.m_filterVer[3][true][true](clpRng, dst, dstStride, piPred, uiStride, uiWidth, uiHeight, p, false);
      }
    }
    else
    {
      PelBuf tmpBuffer = m_tempBuffer[1].getBuf(localUnitArea.Y());
      int    tmpStride = tmpBuffer.stride;
      Pel *  tmp0      = tmpBuffer.buf + TMP_SUBPEL_PAD_NUM + TMP_SUBPEL_PAD_NUM * tmpStride;
      Pel *  tmp       = tmp0 - TMP_SUBPEL_PAD_NUM * tmpStride;
      dst              = dst0 - TMP_SUBPEL_PAD_NUM * dstStride;

      const TFilterCoeff *p = f0;
      // hor
      if (tmpSubPelDir == ABOVE_LEFT_POS || tmpSubPelDir == LEFT_BOTTOM_POS) // left
      {
        p = (tmpIsSubPel == 1) ? f0 : ((tmpIsSubPel == 2) ? f2 : f1);
        m_if.m_filterHor[3][1][true](clpRng, dst - 1, dstStride, tmp, tmpStride, uiWidth, uiHeight + 2 * TMP_SUBPEL_PAD_NUM, p, false);
      }
      else // right
      {
        p = (tmpIsSubPel == 1) ? f0 : ((tmpIsSubPel == 2) ? f1 : f2);
        m_if.m_filterHor[3][1][true](clpRng, dst, dstStride, tmp, tmpStride, uiWidth, uiHeight + 2 * TMP_SUBPEL_PAD_NUM, p, false);
      }

      // ver
      tmp = tmp0;
      if (tmpSubPelDir == ABOVE_LEFT_POS || tmpSubPelDir == ABOVE_RIGHT_POS) // top
      {
        p = (tmpIsSubPel == 1) ? f0 : ((tmpIsSubPel == 2) ? f2 : f1);
        m_if.m_filterVer[3][true][true](clpRng, tmp - tmpStride, tmpStride, piPred, uiStride, uiWidth, uiHeight, p, false);
      }
      else // bottom
      {
        p = (tmpIsSubPel == 1) ? f0 : ((tmpIsSubPel == 2) ? f1 : f2);
        m_if.m_filterVer[3][true][true](clpRng, tmp, tmpStride, piPred, uiStride, uiWidth, uiHeight, p, false);
      }
    }
#if JVET_AF0079_STORING_INTRATMP
    if (tmpIsSubPel != 0)   
    {
      int absDistance = (tmpIsSubPel == 1) ? 8 : (tmpIsSubPel == 2) ? 4 : 12;
      int xDistance = 0, yDistance = 0;
      if ((tmpSubPelIdx == LEFT_POS) || (tmpSubPelIdx == ABOVE_LEFT_POS) || (tmpSubPelIdx == LEFT_BOTTOM_POS))
      {
        xDistance = -absDistance;
      }
      if ((tmpSubPelIdx == RIGHT_POS) || (tmpSubPelIdx == ABOVE_RIGHT_POS) || (tmpSubPelIdx == RIGHT_BOTTOM_POS))
      {
        xDistance = absDistance;
      }
      if ((tmpSubPelIdx == ABOVE_POS) || (tmpSubPelIdx == ABOVE_LEFT_POS) || (tmpSubPelIdx == ABOVE_RIGHT_POS))
      {
        yDistance = -absDistance;
      }
      if ((tmpSubPelIdx == BOTTOM_POS) || (tmpSubPelIdx == LEFT_BOTTOM_POS) || (tmpSubPelIdx == RIGHT_BOTTOM_POS))
      {
        yDistance = absDistance;
      }
      const int iHor = (pX << MV_FRACTIONAL_BITS_INTERNAL) + xDistance;
      const int iVer = (pY << MV_FRACTIONAL_BITS_INTERNAL) + yDistance;
      pu.mv[0].set(iHor, iVer);

      pu.bv.set(pX, pY);
    }
#endif
  }
#else
#if TMP_FAST_ENC
  int pX = pu.cu->tmpXdisp;
  int pY = pu.cu->tmpYdisp;
#else
  int          pX = m_tempLibFast.getX();
  int          pY = m_tempLibFast.getY();
#endif
  Pel *        ref;
#if !TMP_FAST_ENC
  int          picStride = getStride();
#endif
  int          iOffsetY, iOffsetX;
  Pel *        refTarget;
#if TMP_FAST_ENC
  CompArea area = pu.Y();
  int uiHeight = area.height;
  int uiWidth = area.width;
  ref = pu.cu->cs->picture->getRecoBuf(area).buf;
  int picStride = pu.cu->cs->picture->getRecoBuf(area).stride;
#else
  unsigned int uiHeight = uiPatchHeight - TMP_TEMPLATE_SIZE;
  unsigned int uiWidth  = uiPatchWidth - TMP_TEMPLATE_SIZE;

  // the data center: we use the prediction block as the center now.
  // collect the candidates
  ref = getRefPicUsed();
#endif
  iOffsetY  = pY;
  iOffsetX  = pX;
  refTarget = ref + iOffsetY * picStride + iOffsetX;
  for (unsigned int uiY = 0; uiY < uiHeight; uiY++)
  {
    for (unsigned int uiX = 0; uiX < uiWidth; uiX++)
    {
      piPred[uiX] = refTarget[uiX];
    }
    refTarget += picStride;
    piPred += uiStride;
  }
#endif

  pu.interDir               = 1;
  pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
#if JVET_AF0079_STORING_INTRATMP
  if ((cu->tmpIsSubPel == 0) || (cu->tmpIsSubPel == -1))
  {
    pu.mv[0].set(pX << MV_FRACTIONAL_BITS_INTERNAL, pY << MV_FRACTIONAL_BITS_INTERNAL);
    pu.bv.set(pX, pY);
  }
#else
  pu.mv->set(pX << MV_FRACTIONAL_BITS_INTERNAL, pY << MV_FRACTIONAL_BITS_INTERNAL);
  pu.bv.set(pX, pY);
#endif

#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
#if JVET_AD0086_ENHANCED_INTRA_TMP
  if (bDeriveDimdMode)
  {
#endif
    CPelBuf predBuf(pPred, uiStride, uiWidth, uiHeight);
    pu.cu->intraTmpDimdMode = deriveDimdIntraTmpModePred(*pu.cu, predBuf);
#if JVET_AD0086_ENHANCED_INTRA_TMP
  }
#endif
#endif

  return bSucceedFlag;
}
#endif

#if JVET_W0069_TMP_BOUNDARY
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST 
bool IntraPrediction::generateTmDcPrediction(Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val, CodingUnit* cu)
#else
bool IntraPrediction::generateTmDcPrediction( Pel* piPred, unsigned int uiStride, unsigned int uiBlkWidth, unsigned int uiBlkHeight, int DC_Val )
#endif
{
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  Pel* pPred = piPred;
#endif
  bool bSucceedFlag = true;
  {
    for( unsigned int uiY = 0; uiY < uiBlkHeight; uiY++ )
    {
      for( unsigned int uiX = 0; uiX < uiBlkWidth; uiX++ )
      {
        piPred[uiX] = DC_Val;
      }
      piPred += uiStride;
    }
  }
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  CPelBuf predBuf(pPred, uiStride, uiBlkWidth, uiBlkHeight);
  cu->intraTmpDimdMode = deriveDimdIntraTmpModePred(*cu, predBuf);
#endif
  return bSucceedFlag;
}
#endif

#if JVET_AD0086_ENHANCED_INTRA_TMP
void IntraPrediction::calcTemplateDiff(Pel *ref, unsigned int uiStride, Pel **tarPatch, unsigned int uiPatchWidth,
                                       unsigned int uiPatchHeight, int *diff, int *iMax, RefTemplateType tempType, int requiredTemplate)
{
  int diffSum = 0;
  int topDiff  = MAX_INT;
  int leftDiff = MAX_INT;
#if JVET_W0069_TMP_BOUNDARY
  Pel *refPatchRow;
  if (tempType == L_SHAPE_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride;
  }
#else
  Pel *refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
#endif
  Pel *tarPatchRow;

#if JVET_W0069_TMP_BOUNDARY
  if (tempType == L_SHAPE_TEMPLATE)
  {
#endif

    topDiff  = 0;
    leftDiff = 0;

    // horizontal difference
    if(requiredTemplate == 3)//all
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow = tarPatch[iY];
        for (int iX = 0; iX < uiPatchWidth; iX++)
        {
          diffSum += abs(refPatchRow[iX] - tarPatchRow[iX]);

          if (iX >= TMP_TEMPLATE_SIZE)
          {
            topDiff += abs(refPatchRow[iX] - tarPatchRow[iX]);
          }
        }

        if (diffSum > iMax[0] && topDiff > iMax[1])
        {
          break;
        }

        refPatchRow += uiStride;
      }

      refPatchRow = ref - TMP_TEMPLATE_SIZE;

      // vertical difference
      for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow = tarPatch[iY];
        for (int iX = 0; iX < TMP_TEMPLATE_SIZE; iX++)
        {
          diffSum += abs(refPatchRow[iX] - tarPatchRow[iX]);
          leftDiff += abs(refPatchRow[iX] - tarPatchRow[iX]);
        }

        if (diffSum > iMax[0] && leftDiff > iMax[2])
        {
          break;
        }

        refPatchRow += uiStride;
      }
    }
    else if (requiredTemplate == 0)//TL
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow = tarPatch[iY];
        for (int iX = 0; iX < uiPatchWidth; iX++)
        {
          diffSum += abs(refPatchRow[iX] - tarPatchRow[iX]);
        }

        if (diffSum > iMax[0])
        {
          break;
        }

        refPatchRow += uiStride;
      }

      refPatchRow = ref - TMP_TEMPLATE_SIZE;

      // vertical difference
      for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow = tarPatch[iY];
        for (int iX = 0; iX < TMP_TEMPLATE_SIZE; iX++)
        {
          diffSum += abs(refPatchRow[iX] - tarPatchRow[iX]);
        }

        if (diffSum > iMax[0])
        {
          break;
        }

        refPatchRow += uiStride;
      }
    }
    else if(requiredTemplate == 1) //T  
    {
      for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
      {
        tarPatchRow = tarPatch[iY];
        for (int iX = TMP_TEMPLATE_SIZE; iX < uiPatchWidth; iX++)
        {
          topDiff += abs(refPatchRow[iX] - tarPatchRow[iX]);
        }

        if (topDiff > iMax[1])
        {
          break;
        }

        refPatchRow += uiStride;
      }
    }
    else // L
    {
      refPatchRow = ref - TMP_TEMPLATE_SIZE;

      // vertical difference
      for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
      {
        tarPatchRow = tarPatch[iY];
        for (int iX = 0; iX < TMP_TEMPLATE_SIZE; iX++)
        {
          leftDiff += abs(refPatchRow[iX] - tarPatchRow[iX]);
        }

        if (leftDiff > iMax[2])
        {
          break;
        }

        refPatchRow += uiStride;
      }
    }
#if JVET_W0069_TMP_BOUNDARY
  }
  else if (tempType == ABOVE_TEMPLATE)
  {
    // top  template difference
    for (int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++)
    {
      tarPatchRow = tarPatch[iY];
      for (int iX = 0; iX < uiPatchWidth - TMP_TEMPLATE_SIZE; iX++)
      {
        diffSum += abs(refPatchRow[iX] - tarPatchRow[iX]);
      }
      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }
      refPatchRow += uiStride;
    }
  }
  else if (tempType == LEFT_TEMPLATE)
  {
    // left template difference
    for (int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++)
    {
      tarPatchRow = tarPatch[iY];
      for (int iX = 0; iX < TMP_TEMPLATE_SIZE; iX++)
      {
        diffSum += abs(refPatchRow[iX] - tarPatchRow[iX]);
      }
      if (diffSum > iMax[0])   // for speeding up
      {
        break;
      }
      refPatchRow += uiStride;
    }
  }
#endif

  diff[0] = diffSum;
  diff[1] = topDiff;
  diff[2] = leftDiff;
}
#else
#if JVET_W0069_TMP_BOUNDARY
int IntraPrediction::calcTemplateDiff( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax, RefTemplateType tempType )
#else
int IntraPrediction::calcTemplateDiff( Pel* ref, unsigned int uiStride, Pel** tarPatch, unsigned int uiPatchWidth, unsigned int uiPatchHeight, int iMax )
#endif
{
  int diffSum = 0;
#if JVET_W0069_TMP_BOUNDARY
  Pel* refPatchRow;
  if( tempType == L_SHAPE_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
  }
  else if( tempType == LEFT_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE;
  }
  else if( tempType == ABOVE_TEMPLATE )
  {
    refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride;
  }
#else
  Pel* refPatchRow = ref - TMP_TEMPLATE_SIZE * uiStride - TMP_TEMPLATE_SIZE;
#endif
  Pel* tarPatchRow;

#if JVET_W0069_TMP_BOUNDARY
  if( tempType == L_SHAPE_TEMPLATE )
  {
#endif
    // horizontal difference
    for( int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++ )
    {
      tarPatchRow = tarPatch[iY];
      for( int iX = 0; iX < uiPatchWidth; iX++ )
      {
        diffSum += abs( refPatchRow[iX] - tarPatchRow[iX] );
      }
      if( diffSum > iMax ) //for speeding up
      {
        return diffSum;
      }
      refPatchRow += uiStride;
    }

    // vertical difference
    for( int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++ )
    {
      tarPatchRow = tarPatch[iY];
      for( int iX = 0; iX < TMP_TEMPLATE_SIZE; iX++ )
      {
        diffSum += abs( refPatchRow[iX] - tarPatchRow[iX] );
      }
      if( diffSum > iMax ) //for speeding up
      {
        return diffSum;
      }
      refPatchRow += uiStride;
    }
#if JVET_W0069_TMP_BOUNDARY
  }
  else if( tempType == ABOVE_TEMPLATE )
  {
    // top  template difference
    for( int iY = 0; iY < TMP_TEMPLATE_SIZE; iY++ )
    {
      tarPatchRow = tarPatch[iY];
      for( int iX = 0; iX < uiPatchWidth - TMP_TEMPLATE_SIZE; iX++ )
      {
        diffSum += abs( refPatchRow[iX] - tarPatchRow[iX] );
      }
      if( diffSum > iMax ) //for speeding up
      {
        return diffSum;
      }
      refPatchRow += uiStride;
    }
  }
  else if( tempType == LEFT_TEMPLATE )
  {
    // left template difference
    for( int iY = TMP_TEMPLATE_SIZE; iY < uiPatchHeight; iY++ )
    {
      tarPatchRow = tarPatch[iY];
      for( int iX = 0; iX < TMP_TEMPLATE_SIZE; iX++ )
      {
        diffSum += abs( refPatchRow[iX] - tarPatchRow[iX] );
      }
      if( diffSum > iMax ) //for speeding up
      {
        return diffSum;
      }
      refPatchRow += uiStride;
    }
  }
#endif

  return diffSum;
}
#endif
#endif


#if JVET_AD0188_CCP_MERGE
void IntraPrediction::xGlmApplyModelOffset(const PredictionUnit &pu, const ComponentID compId,
                                           const CompArea &chromaArea, CccmModel& glmModel, int glmIdc,
                                           PelBuf &piPred, int lumaOffset, int chromaOffset)
{
  const ClpRng &clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;

  CPelBuf refLumaBlk = xGlmGetGradPuBuf(pu, chromaArea, 0);
  CPelBuf refGradBlk = xGlmGetGradPuBuf(pu, chromaArea, glmIdc);

  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      samples[0] = refGradBlk.at(x, y);                // luma gradient
      samples[1] = refLumaBlk.at(x, y) + lumaOffset;   // luma value
      samples[2] = glmModel.bias();

      piPred.at(x, y) = ClipPel<Pel>(glmModel.convolve(samples) + chromaOffset, clpRng);
    }
  }
}

void IntraPrediction::xCccmApplyModelOffset(const PredictionUnit &pu, const ComponentID compId,
                                            CccmModel& cccmModel, int modelId, int modelThr,
                                            PelBuf &piPred, int lumaOffset, int chromaOffset[], int type, int refSizeX, int refSizeY)
{
  const ClpRng &clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;

  int stepX = 1;
  int stepY = 1;
  int chromaScaleX = 0;
  int chromaScaleY = 0;

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if( pu.cccmNoSubFlag )
  {
    chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
    chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );

    stepX = 1 << chromaScaleX;
    stepY = 1 << chromaScaleY;
  }
#endif

  CPelBuf refLumaBlk;
#if JVET_AD0202_CCCM_MDF
  CPelBuf refLumaBlk1, refLumaBlk2, refLumaBlk3;

  if( ( type & CCP_TYPE_MDFCCCM ) && pu.curCand.cccmMultiFilterIdx == 1 )
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu, 0, 3, &refLumaBlk1, &refLumaBlk3, &refLumaBlk2 );
  }
  else if( ( type & CCP_TYPE_MDFCCCM ) && pu.curCand.cccmMultiFilterIdx > 1 )
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu, 0, 2, &refLumaBlk1, &refLumaBlk3 );
  }
  else
#endif
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu );
  }

  int offset = modelId == 2 ? chromaOffset[1] : chromaOffset[0];

  if (type & CCP_TYPE_CCCM)
  {
    for (int y = 0; y < refLumaBlk.height; y += stepY )
    {
      for (int x = 0; x < refLumaBlk.width; x += stepX )
      {
        if (modelId == 1 && (refLumaBlk.at(x, y) + lumaOffset) > modelThr)   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if (modelId == 2 && (refLumaBlk.at(x, y) + lumaOffset) <= modelThr)   // Model 2: Include only samples above the threshold
        {
          continue;
        }

        // 7-tap cross
        samples[0] = refLumaBlk.at(x, y) + lumaOffset;       // C
        samples[1] = refLumaBlk.at(x, y - 1) + lumaOffset;   // N
        samples[2] = refLumaBlk.at(x, y + 1) + lumaOffset;   // S
        samples[3] = refLumaBlk.at(x - 1, y) + lumaOffset;   // W
        samples[4] = refLumaBlk.at(x + 1, y) + lumaOffset;   // E
        samples[5] = cccmModel.nonlinear(refLumaBlk.at(x, y) + lumaOffset);
        samples[6] = cccmModel.bias();

        piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>(cccmModel.convolve(samples) + offset, clpRng);
      }
    }
  }
#if JVET_AC0054_GLCCCM
  else if (type & CCP_TYPE_GLCCCM)
  {
    for( int y = 0; y < refLumaBlk.height; y += stepY )
    {
      for( int x = 0; x < refLumaBlk.width; x += stepX )
      {
        if (modelId == 1 && (refLumaBlk.at(x, y) + lumaOffset) > modelThr)   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if (modelId == 2 && (refLumaBlk.at(x, y) + lumaOffset) <= modelThr)   // Model 2: Include only samples above the threshold
        {
          continue;
        }

        samples[0] = refLumaBlk.at(x, y) + lumaOffset;   // C
        samples[1] = (2 * refLumaBlk.at(x, y - 1) + refLumaBlk.at(x - 1, y - 1) + refLumaBlk.at(x + 1, y - 1))
                   - (2 * refLumaBlk.at(x, y + 1) + refLumaBlk.at(x - 1, y + 1) + refLumaBlk.at(x + 1, y + 1));   // Vertical gradient
        samples[2] = (2 * refLumaBlk.at(x - 1, y) + refLumaBlk.at(x - 1, y - 1) + refLumaBlk.at(x - 1, y + 1))
                   - (2 * refLumaBlk.at(x + 1, y) + refLumaBlk.at(x + 1, y - 1) + refLumaBlk.at(x + 1, y + 1));   // Horizontal gradient
        samples[3] = ( y + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT;   // Y coordinate
        samples[4] = ( x + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT;   // X coordinate
        samples[5] = cccmModel.nonlinear( refLumaBlk.at( x, y ) + lumaOffset );
        samples[6] = cccmModel.bias();

        piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>( cccmModel.convolve(samples) + offset, clpRng);
      }
    }
  }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  else if( ( type & CCP_TYPE_NSCCCM ) && pu.cccmNoSubFlag )
  {
    for( int y = 0; y < refLumaBlk.height; y += stepY )
    {
      for( int x = 0; x < refLumaBlk.width; x += stepX )
      {
        const Pel *src0 = refLumaBlk.bufAt( x, y );
        const Pel *src1 = refLumaBlk.bufAt( x, y + 1 );

        if( modelId == 1 && ( src0[0] + lumaOffset ) > modelThr )   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if( modelId == 2 && ( src0[0] + lumaOffset ) <= modelThr )   // Model 2: Include only samples above the threshold
        {
          continue;
        }

        samples[0] = src0[0] + lumaOffset;
        samples[1] = src0[-1] + lumaOffset;
        samples[2] = src0[1] + lumaOffset;
        samples[3] = src1[0] + lumaOffset;
        samples[4] = src1[-1] + lumaOffset;
        samples[5] = src1[1] + lumaOffset;
        samples[6] = cccmModel.nonlinear( src0[0] + lumaOffset );
        samples[7] = cccmModel.nonlinear( src1[0] + lumaOffset );
        samples[8] = cccmModel.nonlinear( src0[1] + lumaOffset );
        samples[9] = cccmModel.nonlinear( src0[-1] + lumaOffset );
        samples[10] = cccmModel.bias();

        piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>( cccmModel.convolve( samples ) + offset, clpRng );
      }
    }
  }
  else
#endif
#if JVET_AD0202_CCCM_MDF
  if( ( type & CCP_TYPE_MDFCCCM ) && pu.curCand.cccmMultiFilterIdx == 1 )
  {
    for( int y = 0; y < refLumaBlk.height; y += stepY )
    {
      for( int x = 0; x < refLumaBlk.width; x += stepX )
      {
        if( modelId == 1 && ( refLumaBlk.at( x, y ) + lumaOffset ) > modelThr )   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if( modelId == 2 && ( refLumaBlk.at( x, y ) + lumaOffset ) <= modelThr )   // Model 2: Include only samples above the threshold
        {
          continue;
        }

        // 7-tap cross
        samples[0] = refLumaBlk.at( x, y ) + lumaOffset; // C
        samples[1] = refLumaBlk1.at( x, y ) + lumaOffset; // W
        samples[2] = refLumaBlk2.at( x, y ) + lumaOffset; // E
        samples[3] = refLumaBlk3.at( x, y ) + lumaOffset;
        samples[4] = cccmModel.nonlinear( refLumaBlk.at( x, y ) + lumaOffset );
        samples[5] = cccmModel.nonlinear( refLumaBlk1.at( x, y ) + lumaOffset );
        samples[6] = cccmModel.nonlinear( refLumaBlk2.at( x, y ) + lumaOffset );
        samples[7] = ( y + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        samples[8] = ( x + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        samples[9] = cccmModel.bias();

        piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>( cccmModel.convolve( samples ) + offset, clpRng );
      }
    }
  }
  else if( ( type & CCP_TYPE_MDFCCCM ) && pu.curCand.cccmMultiFilterIdx == 2 )
  {
    for( int y = 0; y < refLumaBlk.height; y += stepY )
    {
      for( int x = 0; x < refLumaBlk.width; x += stepX )
      {
        if( modelId == 1 && ( refLumaBlk.at( x, y ) + lumaOffset ) > modelThr )   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if( modelId == 2 && ( refLumaBlk.at( x, y ) + lumaOffset ) <= modelThr )   // Model 2: Include only samples above the threshold
        {
          continue;
        }

        // 7-tap cross
        samples[0] = refLumaBlk.at( x, y ) + lumaOffset; // C
        samples[1] = refLumaBlk.at( x - 1, y ) + lumaOffset; // W
        samples[2] = refLumaBlk.at( x + 1, y ) + lumaOffset; // E
        samples[3] = refLumaBlk1.at( x, y ) + lumaOffset; // C
        samples[4] = refLumaBlk1.at( x - 1, y ) + lumaOffset; // W
        samples[5] = refLumaBlk1.at( x + 1, y ) + lumaOffset; // E
        samples[6] = cccmModel.nonlinear( refLumaBlk.at( x, y ) + lumaOffset );
        samples[7] = cccmModel.nonlinear( refLumaBlk.at( x - 1, y ) + lumaOffset );
        samples[8] = cccmModel.nonlinear( refLumaBlk.at( x + 1, y ) + lumaOffset );
        samples[9] = ( x + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        samples[10] = cccmModel.bias();

        piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>( cccmModel.convolve( samples ) + offset, clpRng );
      }
    }
  }
  else if( ( type & CCP_TYPE_MDFCCCM ) && pu.curCand.cccmMultiFilterIdx == 3 )
  {
    for( int y = 0; y < refLumaBlk.height; y += stepY )
    {
      for( int x = 0; x < refLumaBlk.width; x += stepX )
      {
        if( modelId == 1 && ( refLumaBlk.at( x, y ) + lumaOffset ) > modelThr )   // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if( modelId == 2 && ( refLumaBlk.at( x, y ) + lumaOffset ) <= modelThr )   // Model 2: Include only samples above the threshold
        {
          continue;
        }

        // 7-tap cross
        samples[0] = refLumaBlk.at( x, y ) + lumaOffset; // C
        samples[1] = refLumaBlk.at( x + 1, y - 1 ) + lumaOffset; // EN
        samples[2] = refLumaBlk.at( x - 1, y + 1 ) + lumaOffset; // WS
        samples[3] = refLumaBlk3.at( x, y ) + lumaOffset; // C
        samples[4] = refLumaBlk3.at( x + 1, y - 1 ) + lumaOffset; // EN
        samples[5] = refLumaBlk3.at( x - 1, y + 1 ) + lumaOffset; // WS
        samples[6] = cccmModel.nonlinear( refLumaBlk.at( x, y ) + lumaOffset );
        samples[7] = cccmModel.nonlinear( refLumaBlk.at( x + 1, y - 1 ) + lumaOffset );
        samples[8] = cccmModel.nonlinear( refLumaBlk.at( x - 1, y + 1 ) + lumaOffset );
        samples[9] = ( y + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        samples[10] = cccmModel.bias();

        piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>( cccmModel.convolve( samples ) + offset, clpRng );
      }
    }
  }
#endif
  else
  {
    THROW("Invalid type");
  }
}

template <const bool updateOffsets>
int  IntraPrediction::xUpdateOffsetsAndGetCostCCLM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea,CclmModel &cclmModel, int modelNum, int glmIdc)
{
  int    srcStride = 0;
  PelBuf temp;

  if (glmIdc > 0)
  {
    Pel *glmTemp = m_glmTempCb[glmIdc];
    srcStride    = 2 * MAX_CU_SIZE + 1;
    temp         = PelBuf(glmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp      = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure  &cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const bool aboveAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID)) ? true : false;
  const bool leftAvailable  = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID)) ? true : false;

  Pel *srcColor0, *curChroma0;

  srcColor0 = temp.bufAt(0, 0);

  PelBuf chromaReco = cs.picture->getRecoBuf(chromaArea);

  Pel      *curChromaBuf = chromaReco.buf;
  const int curStride    = chromaReco.stride;

  int sad = 0;
  int totalOffset[2] = { 0, 0 };
  int count[2] = { 0, 0 };

  if( aboveAvailable )
  {
    curChroma0 = curChromaBuf - curStride;
    Pel *src = srcColor0 - srcStride;
    Pel predChroma;
    for( int pos = 0; pos < cWidth; pos++ )
    {
      if( modelNum == 2 && src[pos] > cclmModel.yThres )
      {
        predChroma = rightShift( cclmModel.a2 * src[pos], cclmModel.shift2 ) + cclmModel.b2;

        if( updateOffsets )
        {
          totalOffset[1] += curChroma0[pos] - predChroma;
          count[1]++;
        }
      }
      else
      {
        predChroma = rightShift( cclmModel.a * src[pos], cclmModel.shift ) + cclmModel.b;

        if( updateOffsets )
        {
          totalOffset[0] += curChroma0[pos] - predChroma;
          count[0]++;
        }
      }
      sad += abs( curChroma0[pos] - predChroma );
    }
  }

  if( leftAvailable )
  {
    curChroma0 = curChromaBuf - 1;
    Pel *src = srcColor0 - 1;

    Pel predChroma;
    for( int pos = 0; pos < cHeight; pos++ )
    {
      if( modelNum == 2 && src[pos * srcStride] > cclmModel.yThres )
      {
        predChroma = rightShift( cclmModel.a2 * src[pos * srcStride], cclmModel.shift2 ) + cclmModel.b2;

        if( updateOffsets )
        {
          totalOffset[1] += curChroma0[pos * curStride] - predChroma;
          count[1]++;
        }
      }
      else
      {
        predChroma = rightShift( cclmModel.a * src[pos * srcStride], cclmModel.shift ) + cclmModel.b;

        if( updateOffsets )
        {
          totalOffset[0] += curChroma0[pos * curStride] - predChroma;
          count[0]++;
        }        
      }
      sad += abs( curChroma0[pos * curStride] - predChroma );
    }
  }

  if( updateOffsets )
  {
    if( count[0] )
    {
      cclmModel.b += PU::getMeanValue( totalOffset[0], count[0] );   // totalOffset[0] / count[0];
    }

    if( modelNum == 2 && count[1] )
    {
      cclmModel.b2 += PU::getMeanValue( totalOffset[1], count[1] );   //totalOffset[1] / count[1];
    }
  }

  return sad;
}

template <const bool updateOffsets>
int IntraPrediction::xUpdateOffsetsAndGetCostCCCM(const PredictionUnit &pu, const ComponentID compID,
                                  const CompArea &chromaArea, CccmModel cccmModel[2],
                                  int modelThr, int lumaOffset, int chromaOffset[2], int type, int refSizeX, int refSizeY, const int cccmMultiFilterIdx)
{
  const ClpRng &clpRng(pu.cu->cs->slice->clpRng(compID));

  CHECK(compID != chromaArea.compID, "Invalid component ID");

  Pel* samples = m_samples;

  CPelBuf refLumaBlk;
#if JVET_AD0202_CCCM_MDF
  CPelBuf refLumaBlk1, refLumaBlk2, refLumaBlk3;

  if( cccmMultiFilterIdx == 1 )
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu, 0, 3, &refLumaBlk1, &refLumaBlk3, &refLumaBlk2 );
  }
  else if( cccmMultiFilterIdx > 1 )
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu, 0, 2, &refLumaBlk1, &refLumaBlk3 );
  }
  else
#endif
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu );
  }

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure  &cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const bool aboveAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID)) ? true : false;
  const bool leftAvailable  = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID)) ? true : false;

  PelBuf chromaReco = cs.picture->getRecoBuf(chromaArea);

  const Pel *curChromaBuf[2] = { nullptr, }, *srcBuf[2] = { nullptr, };
  int curStrideBuf[2] = { 0, }, srcStrideBuf[2] = { 0, };
  int sad = 0, start = 0, end = 0, posEnd[2] = { 0, }, step[2] = { 0, };
  const int srcStride = refLumaBlk.stride;

  int totalOffset[2] = { 0, 0 };
  int count[2] = { 0, 0 };

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
  const int stepX = 1 << chromaScaleX;
  const int stepY = 1 << chromaScaleY;
#endif

  Pel predChroma;

  if( aboveAvailable )
  {
    curChromaBuf[0] = chromaReco.bufAt( 0, -1 );
    srcBuf[0] = refLumaBlk.bufAt( 0, -1 );
    curStrideBuf[0] = 1;
    srcStrideBuf[0] = 1;
    posEnd[0] = cWidth;
    step[0] = stepX;
    start = 0;
    end = 1;
  }

  if( leftAvailable )
  {
    curChromaBuf[1] = chromaReco.bufAt( -1, 0 );
    srcBuf[1] = refLumaBlk.bufAt( -1, 0 );
    curStrideBuf[1] = chromaReco.stride;
    srcStrideBuf[1] = srcStride;
    posEnd[1] = cHeight;
    step[1] = stepY;
    start += !aboveAvailable;
    end = 2;
  }

  for( int i = start; i < end; i++ )
  {
    const Pel* curChroma0 = curChromaBuf[i];
    const Pel* src = srcBuf[i];    
    const int curStride = curStrideBuf[i];

    if( type & CCP_TYPE_CCCM )
    {
      for( int pos = 0; pos < posEnd[i]; pos++, src += srcStrideBuf[i] )
      {
        samples[0] = *src + lumaOffset;                 // C
        samples[1] = *( src - srcStride ) + lumaOffset;   // N
        samples[2] = *( src + srcStride ) + lumaOffset;   // S
        samples[3] = *( src - 1 ) + lumaOffset;           // W
        samples[4] = *( src + 1 ) + lumaOffset;           // E
        samples[5] = cccmModel[0].nonlinear( *src + lumaOffset );
        samples[6] = cccmModel[0].bias();

#if MMLM
        if( ( type & CCP_TYPE_MMLM ) && *src + lumaOffset > modelThr )
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[1].convolve( samples );
            totalOffset[1] += curChroma0[pos * curStride] - predChroma;
            count[1]++;
          }
          else
          {
            predChroma = cccmModel[1].convolve( samples ) + chromaOffset[1];
          }
        }
        else
#endif
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[0].convolve( samples );
            totalOffset[0] += curChroma0[pos * curStride] - predChroma;
            count[0]++;
          }
          else
          {
            predChroma = cccmModel[0].convolve( samples ) + chromaOffset[0];
          }
        }

        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );
      }
    }
#if JVET_AC0054_GLCCCM
    else if( type & CCP_TYPE_GLCCCM )
    {
      for( int pos = 0; pos < posEnd[i]; pos++, src += srcStrideBuf[i] )
      {
        samples[0] = *src + lumaOffset;   // C
        samples[1] = ( 2 * ( *( src - srcStride ) ) + ( *( src - 1 - srcStride ) ) + ( *( src - srcStride + 1 ) ) )
                   - ( 2 * ( *( src + srcStride ) ) + ( *( src - 1 + srcStride ) ) + ( *( src + srcStride + 1 ) ) );   // Vertical gradient
        samples[2] = ( 2 * ( *( src - 1 ) ) + ( *( src - 1 - srcStride ) ) + ( *( src - 1 + srcStride ) ) )
                   - ( 2 * ( *( src + 1 ) ) + ( *( src + 1 - srcStride ) ) + ( *( src + 1 + srcStride ) ) );   // Horizontal gradient
        samples[3] = ( ( ( i ? pos : -1 ) + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT );            // Y coordinate
        samples[4] = ( ( ( i ? -1 : pos ) + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT );            // X coordinate
        samples[5] = cccmModel[0].nonlinear( *src + lumaOffset );
        samples[6] = cccmModel[0].bias();

#if MMLM
        if( ( type & CCP_TYPE_MMLM ) && *src + lumaOffset > modelThr )
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[1].convolve( samples );
            totalOffset[1] += curChroma0[pos * curStride] - predChroma;
            count[1]++;
          }
          else
          {
            predChroma = cccmModel[1].convolve( samples ) + chromaOffset[1];
          }
        }
        else
#endif
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[0].convolve( samples );
            totalOffset[0] += curChroma0[pos * curStride] - predChroma;
            count[0]++;
          }
          else
          {
            predChroma = cccmModel[0].convolve( samples ) + chromaOffset[0];
          }
        }

        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );
      }
    }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
    else if( type & CCP_TYPE_NSCCCM )
    {
      const int stride = step[i] * srcStrideBuf[i];
      const Pel *src0 = i ? refLumaBlk.bufAt( -stepX, 0 ) : refLumaBlk.bufAt( 0, -stepY );
      const Pel *src1 = i ? refLumaBlk.bufAt( -stepX, 1 ) : refLumaBlk.bufAt( 0, -stepY + 1 );

      for( int pos = 0; pos < posEnd[i]; pos++ )
      {
        samples[0] = src0[0] + lumaOffset;
        samples[1] = src0[-1] + lumaOffset;
        samples[2] = src0[1] + lumaOffset;
        samples[3] = src1[0] + lumaOffset;
        samples[4] = src1[-1] + lumaOffset;
        samples[5] = src1[1] + lumaOffset;
        samples[6] = cccmModel[0].nonlinear( src0[0] + lumaOffset );
        samples[7] = cccmModel[0].nonlinear( src1[0] + lumaOffset );
        samples[8] = cccmModel[0].nonlinear( src0[1] + lumaOffset );
        samples[9] = cccmModel[0].nonlinear( src0[-1] + lumaOffset );
        samples[10] = cccmModel[0].bias();

#if MMLM
        if( ( type & CCP_TYPE_MMLM ) && src0[0] + lumaOffset > modelThr )
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[1].convolve( samples );
            totalOffset[1] += curChroma0[pos * curStride] - predChroma;
            count[1]++;
          }
          else
          {
            predChroma = cccmModel[1].convolve( samples ) + chromaOffset[1];
          }
        }
        else
#endif
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[0].convolve( samples );
            totalOffset[0] += curChroma0[pos * curStride] - predChroma;
            count[0]++;
          }
          else
          {
            predChroma = cccmModel[0].convolve( samples ) + chromaOffset[0];
          }
        }

        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );

        src0 += stride;
        src1 += stride;
      }
    }
#endif
#if JVET_AD0202_CCCM_MDF
    else if( ( type & CCP_TYPE_MDFCCCM ) && cccmMultiFilterIdx == 1 )
    {
      for( int pos = 0; pos < posEnd[i]; pos++ )
      {
        Position p = Position( ( i ? -1 : pos ), ( i ? pos : -1 ) );

        samples[0] = refLumaBlk.at( p ) + lumaOffset; // C
        samples[1] = refLumaBlk1.at( p ) + lumaOffset; // W
        samples[2] = refLumaBlk2.at( p ) + lumaOffset; // E
        samples[3] = refLumaBlk3.at( p ) + lumaOffset;
        samples[4] = cccmModel[0].nonlinear( refLumaBlk.at( p ) + lumaOffset );
        samples[5] = cccmModel[0].nonlinear( refLumaBlk1.at( p ) + lumaOffset );
        samples[6] = cccmModel[0].nonlinear( refLumaBlk2.at( p ) + lumaOffset );
        samples[7] = ( p.y + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        samples[8] = ( p.x + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        samples[9] = cccmModel[0].bias();

#if MMLM
        if( ( type & CCP_TYPE_MMLM ) && refLumaBlk.at( p ) + lumaOffset > modelThr )
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[1].convolve( samples );
            totalOffset[1] += curChroma0[pos * curStride] - predChroma;
            count[1]++;
          }
          else
          {
            predChroma = cccmModel[1].convolve( samples ) + chromaOffset[1];
          }
        }
        else
#endif
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[0].convolve( samples );
            totalOffset[0] += curChroma0[pos * curStride] - predChroma;
            count[0]++;
          }
          else
          {
            predChroma = cccmModel[0].convolve( samples ) + chromaOffset[0];
          }
        }

        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );
      }
    }
    else if( ( type & CCP_TYPE_MDFCCCM ) && cccmMultiFilterIdx == 2 )
    {
      const Pel* src0 = i ? refLumaBlk.bufAt( 0, 0 ) : refLumaBlk.bufAt( 1, -1 );
      const Pel* src1 = i ? refLumaBlk1.bufAt( 0, 0 ) : refLumaBlk1.bufAt( 1, -1 );

      const int stride0 = srcStrideBuf[i];
      const int stride1 = i ? refLumaBlk1.stride : 1;

      for( int pos = 0; pos < posEnd[i]; pos++ )
      {
        samples[0] = src0[-1] + lumaOffset; // C
        samples[1] = src0[-2] + lumaOffset; // W
        samples[2] = src0[0] + lumaOffset; // E
        samples[3] = src1[-1] + lumaOffset; // C
        samples[4] = src1[-2] + lumaOffset; // W
        samples[5] = src1[0] + lumaOffset; // E
        samples[6] = cccmModel[0].nonlinear( src0[-1] + lumaOffset );
        samples[7] = cccmModel[0].nonlinear( src0[-2] + lumaOffset );
        samples[8] = cccmModel[0].nonlinear( src0[0] + lumaOffset );
        samples[9] = ( ( i ? -1 : pos ) + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        samples[10] = cccmModel[0].bias();

#if MMLM
        if( ( type & CCP_TYPE_MMLM ) && src0[-1] + lumaOffset > modelThr )
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[1].convolve( samples );
            totalOffset[1] += curChroma0[pos * curStride] - predChroma;
            count[1]++;
          }
          else
          {
            predChroma = cccmModel[1].convolve( samples ) + chromaOffset[1];
          }
        }
        else
#endif
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[0].convolve( samples );
            totalOffset[0] += curChroma0[pos * curStride] - predChroma;
            count[0]++;
          }
          else
          {
            predChroma = cccmModel[0].convolve( samples ) + chromaOffset[0];
          }
        }

        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );

        src0 += stride0;
        src1 += stride1;
      }
    }
    else if( ( type & CCP_TYPE_MDFCCCM ) && cccmMultiFilterIdx == 3 )
    {
      const Pel* src0[3] = { i ?  refLumaBlk.bufAt( -1, 0 ) :  refLumaBlk.bufAt( 0, -1 ), i ?  refLumaBlk.bufAt( 0, -1 ) :  refLumaBlk.bufAt( 1, -2 ), i ?  refLumaBlk.bufAt( -2, 1 ) :  refLumaBlk.bufAt( -1, 0 ) };
      const Pel* src1[3] = { i ? refLumaBlk3.bufAt( -1, 0 ) : refLumaBlk3.bufAt( 0, -1 ), i ? refLumaBlk3.bufAt( 0, -1 ) : refLumaBlk3.bufAt( 1, -2 ), i ? refLumaBlk3.bufAt( -2, 1 ) : refLumaBlk3.bufAt( -1, 0 ) };

      const int stride0 = srcStrideBuf[i];
      const int stride1 = i ? refLumaBlk3.stride : 1;

      for( int pos = 0; pos < posEnd[i]; pos++ )
      {
        samples[0] = *src0[0] + lumaOffset; // C
        samples[1] = *src0[1] + lumaOffset; // EN
        samples[2] = *src0[2] + lumaOffset; // WS
        samples[3] = *src1[0] + lumaOffset; // C
        samples[4] = *src1[1] + lumaOffset; // EN
        samples[5] = *src1[2] + lumaOffset; // WS
        samples[6] = cccmModel[0].nonlinear( *src0[0] + lumaOffset );
        samples[7] = cccmModel[0].nonlinear( *src0[1] + lumaOffset );
        samples[8] = cccmModel[0].nonlinear( *src0[2] + lumaOffset );
        samples[9] = ( ( i ? pos : -1 ) + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        samples[10] = cccmModel[0].bias();
        
#if MMLM
        if( ( type & CCP_TYPE_MMLM ) && *src0[0] + lumaOffset > modelThr )
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[1].convolve( samples );
            totalOffset[1] += curChroma0[pos * curStride] - predChroma;
            count[1]++;
          }
          else
          {
            predChroma = cccmModel[1].convolve( samples ) + chromaOffset[1];
          }
        }
        else
#endif
        {
          if( updateOffsets )
          {
            predChroma = cccmModel[0].convolve( samples );
            totalOffset[0] += curChroma0[pos * curStride] - predChroma;
            count[0]++;
          }
          else
          {
            predChroma = cccmModel[0].convolve( samples ) + chromaOffset[0];
          }
        }

        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );

        src0[0] += stride0;
        src0[1] += stride0;
        src0[2] += stride0;
        src1[0] += stride1;
        src1[1] += stride1;
        src1[2] += stride1;
      }
    }
#endif
    else
    {
      THROW( "Invalid type" );
    }
  }

  if( updateOffsets )
  {
    chromaOffset[0] = chromaOffset[1] = 0;

    if( count[0] )
    {
      chromaOffset[0] = PU::getMeanValue( totalOffset[0], count[0] );   // totalOffset[0] / count[0];
    }
    if( ( type & CCP_TYPE_MMLM ) && count[1] )
    {
      chromaOffset[1] = PU::getMeanValue( totalOffset[1], count[1] );   // totalOffset[1] / count[1];
    }
  }

  return sad;
}

template <const bool updateOffsets>
int IntraPrediction::xUpdateOffsetsAndGetCostGLM(const PredictionUnit &pu, const ComponentID compID, const CompArea &chromaArea,
                                 CccmModel& glmModel, int glmIdc, int lumaOffset, int &chromaOffset)
{
  const ClpRng &clpRng(pu.cu->cs->slice->clpRng(compID));
  CHECK(compID != chromaArea.compID, "Invalid component ID");

  Pel* samples = m_samples;

  CPelBuf refLumaBlk = xGlmGetGradPuBuf(pu, chromaArea, 0);
  CPelBuf refGradBlk = xGlmGetGradPuBuf(pu, chromaArea, glmIdc);

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure  &cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const bool aboveAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID)) ? true : false;
  const bool leftAvailable  = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID)) ? true : false;

  const Pel *srcColor0, *srcColor1, *curChroma0;

  srcColor0      = refGradBlk.bufAt(0, 0);
  int srcStride0 = refGradBlk.stride;
  srcColor1      = refLumaBlk.bufAt(0, 0);
  int srcStride1 = refLumaBlk.stride;

  PelBuf chromaReco = cs.picture->getRecoBuf(chromaArea);

  Pel      *curChromaBuf = chromaReco.buf;
  const int curStride    = chromaReco.stride;

  int sad = 0;
  int totalOffset = 0;
  int count = 0;

  if (aboveAvailable)
  {
    curChroma0      = curChromaBuf - curStride;
    const Pel *src0 = srcColor0 - srcStride0;
    const Pel *src1 = srcColor1 - srcStride1;

    for( int pos = 0; pos < cWidth; pos++, src0++, src1++ )
    {
      samples[0] = *src0;                // luma gradient
      samples[1] = *src1 + lumaOffset;   // luma value
      samples[2] = glmModel.bias();

      if( updateOffsets )
      {
        Pel predChroma = glmModel.convolve( samples );
        totalOffset += curChroma0[pos] - predChroma;
        count++;
      }
      else
      {
        Pel predChroma = glmModel.convolve( samples ) + chromaOffset;
        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos] - predChroma );
      }
    }
  }

  if (leftAvailable)
  {
    curChroma0 = curChromaBuf - 1;

    const Pel *src0 = srcColor0 - 1;
    const Pel *src1 = srcColor1 - 1;

    for (int pos = 0; pos < cHeight; pos++, src0 += srcStride0, src1 += srcStride1)
    {
      samples[0] = *src0;                // luma gradient
      samples[1] = *src1 + lumaOffset;   // luma value
      samples[2] = glmModel.bias();

      if( updateOffsets )
      {
        Pel predChroma = glmModel.convolve( samples );
        totalOffset += curChroma0[pos * curStride] - predChroma;
        count++;
      }
      else
      {
        Pel predChroma = glmModel.convolve( samples ) + chromaOffset;
        predChroma = ClipPel<Pel>( predChroma, clpRng );
        sad += abs( curChroma0[pos * curStride] - predChroma );
      }
    }
  }

  if( updateOffsets )
  {
    chromaOffset = 0;

    if( count )
    {
      chromaOffset = PU::getMeanValue( totalOffset, count );   // totalOffset / count;
    }
  }

  return sad;
}
void IntraPrediction::xCclmApplyModel(const PredictionUnit &pu, const ComponentID compId,
                                      CccmModel& cccmModel, int modelId, int modelThr,
                                      PelBuf &piPred)
{
  const ClpRng &clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;

  CPelBuf refLumaBlk = xCccmGetLumaPuBuf(pu);

  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      if (modelId == 1 && refLumaBlk.at(x, y) > modelThr)   // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if (modelId == 2 && refLumaBlk.at(x, y) <= modelThr)   // Model 2: Include only samples above the threshold
      {
        continue;
      }
      samples[0] = refLumaBlk.at(x, y);   // C
      samples[1] = cccmModel.bias();

      piPred.at(x, y) = ClipPel<Pel>(Pel((cccmModel.params[0] * samples[0] + cccmModel.params[1] * samples[1] + CCCM_DECIM_ROUND) >> CCCM_DECIM_BITS), clpRng);
    }
  }
}

void IntraPrediction::reorderCCPCandidates(PredictionUnit &pu, CCPModelCandidate candList[], int reorderlistSize)
{
  int candCost[MAX_CCP_CAND_LIST_SIZE];
  for (int i = 0; i < reorderlistSize; i++)
  {
    candCost[i] = xGetOneCCPCandCost(pu, candList[i]);
  }

  //Inserting sorting
  for (int i = 1; i < reorderlistSize; i++)
  {
    for (int j = 0; j < i; j++)
    {
      if (candCost[i] < candCost[j])
      {
        CCPModelCandidate tmpCand = candList[i];
        int tmpCost = candCost[i];
        for (int k = i; k > j; k--)
        {
          candList[k] = candList[k - 1];
          candCost[k] = candCost[k - 1];
        }
        candList[j] = tmpCand;
        candCost[j] = tmpCost;
        break;
      }
    }
  }
}

int IntraPrediction::xGetOneCCPCandCost( PredictionUnit &pu, CCPModelCandidate &ccpCand )
{
  CompArea chromaArea = pu.Cb();
  int cost = 0;
  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  int offsetCb[2] = { 0, 0 };
  int offsetCr[2] = { 0, 0 };

#if JVET_AF0073_INTER_CCP_MERGE
  CHECK((ccpCand.type & CCP_TYPE_INTER_CCCM) && (ccpCand.type & CCP_TYPE_MMLM), "wrong type");
#endif
  CHECK(ccpCand.type == CCP_TYPE_NONE, "CCP model with no type")
  {
    if (ccpCand.type & (CCP_TYPE_CCCM | CCP_TYPE_GLCCCM))
    {
#if JVET_AB0174_CCCM_DIV_FREE
      int lumaOffset = m_cccmLumaOffset - ccpCand.lumaOffset;
#else
      int lumaOffset = 0;
#endif
      int refSizeX = ccpCand.corOffX;
      int refSizeY = ccpCand.corOffY;

      CccmModel cccmModelCb[2] = { CccmModel( CCCM_NUM_PARAMS, bitDepth ), CccmModel( CCCM_NUM_PARAMS, bitDepth ) };
      CccmModel cccmModelCr[2] = { CccmModel( CCCM_NUM_PARAMS, bitDepth ), CccmModel( CCCM_NUM_PARAMS, bitDepth ) };

      PU::ccpParamsToCccmModel(ccpCand, cccmModelCb, cccmModelCr);

      cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cb, pu.Cb(), cccmModelCb, ccpCand.yThres, lumaOffset, offsetCb, ccpCand.type, refSizeX, refSizeY);
      cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cr, pu.Cr(), cccmModelCr, ccpCand.yThres, lumaOffset, offsetCr, ccpCand.type, refSizeX, refSizeY);
    }
#if JVET_AF0073_INTER_CCP_MERGE
    else if (ccpCand.type & (CCP_TYPE_INTER_CCCM))
    {
#if JVET_AB0174_CCCM_DIV_FREE
      int lumaOffset = m_cccmLumaOffset - ccpCand.lumaOffset;
#else
      int lumaOffset = 0;
#endif
      pu.cccmNoSubFlag = 1;

      CccmModel interCccmModels[] = { CccmModel( INTER_CCCM_NUM_PARAMS, bitDepth ), CccmModel( INTER_CCCM_NUM_PARAMS, bitDepth ) };

      PU::ccpParamsToCccmModel(ccpCand, interCccmModels[0], interCccmModels[1]);

      cost += xGetCostInterCccm(pu, COMPONENT_Cb, pu.Cb(), interCccmModels[0], lumaOffset, offsetCb[0]);
      cost += xGetCostInterCccm(pu, COMPONENT_Cr, pu.Cr(), interCccmModels[1], lumaOffset, offsetCr[0]);

      pu.cccmNoSubFlag = 0;
    }
#endif
#if JVET_AD0202_CCCM_MDF
    else if (ccpCand.type & (CCP_TYPE_MDFCCCM))
    {
      pu.cccmMultiFilterIdx = ccpCand.cccmMultiFilterIdx;
#if JVET_AB0174_CCCM_DIV_FREE
      int lumaOffset = m_cccmLumaOffset - ccpCand.lumaOffset;
#else
      int lumaOffset = 0;
#endif
      int refSizeX = ccpCand.corOffX;
      int refSizeY = ccpCand.corOffY;

      if (ccpCand.cccmMultiFilterIdx == 1)
      {
        CccmModel cccmModelCb[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth ), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth) };
        CccmModel cccmModelCr[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth ), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth) };

        PU::ccpParamsToCccmModel(ccpCand, cccmModelCb, cccmModelCr);

        cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cb, pu.Cb(), cccmModelCb, ccpCand.yThres, lumaOffset, offsetCb, ccpCand.type, refSizeX, refSizeY, ccpCand.cccmMultiFilterIdx );
        cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cr, pu.Cr(), cccmModelCr, ccpCand.yThres, lumaOffset, offsetCr, ccpCand.type, refSizeX, refSizeY, ccpCand.cccmMultiFilterIdx );
      }
      else
      {
        CHECK(ccpCand.cccmMultiFilterIdx != 2 && ccpCand.cccmMultiFilterIdx != 3, "Unexpected cccmMultiFilterIdx");

        CccmModel cccmModelCb[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth) };
        CccmModel cccmModelCr[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth) };

        PU::ccpParamsToCccmModel(ccpCand, cccmModelCb, cccmModelCr);

        cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cb, pu.Cb(), cccmModelCb, ccpCand.yThres, lumaOffset, offsetCb, ccpCand.type, refSizeX, refSizeY, ccpCand.cccmMultiFilterIdx );
        cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cr, pu.Cr(), cccmModelCr, ccpCand.yThres, lumaOffset, offsetCr, ccpCand.type, refSizeX, refSizeY, ccpCand.cccmMultiFilterIdx );
      }

      pu.cccmMultiFilterIdx = 0;
    }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
    else if (ccpCand.type & CCP_TYPE_NSCCCM)
    {
#if JVET_AB0174_CCCM_DIV_FREE
      int lumaOffset = m_cccmLumaOffset - ccpCand.lumaOffset;
#else
      int lumaOffset = 0;
#endif
      pu.cccmNoSubFlag = 1;

      CccmModel cccmModelCb[2] = { CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ), CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ) };
      CccmModel cccmModelCr[2] = { CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ), CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ) };
        
      PU::ccpParamsToCccmModel(ccpCand, cccmModelCb, cccmModelCr);
        
      cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cb, pu.Cb(), cccmModelCb, ccpCand.yThres, lumaOffset, offsetCb, ccpCand.type );
      cost += xUpdateOffsetsAndGetCostCCCM<false>(pu, COMPONENT_Cr, pu.Cr(), cccmModelCr, ccpCand.yThres, lumaOffset, offsetCr, ccpCand.type );

      pu.cccmNoSubFlag = 0;
    }
#endif
    else if (ccpCand.type & (CCP_TYPE_CCLM | CCP_TYPE_GLM0123))
    {
      CPelBuf temp;
      int     lumaStride;

      if (ccpCand.type & CCP_TYPE_GLM0123)
      {
        Pel *glmTemp = m_glmTempCb[ccpCand.glmIdc];
        lumaStride   = 2 * MAX_CU_SIZE + 1;
        temp         = PelBuf(glmTemp + lumaStride + 1, lumaStride, Size(chromaArea));
      }
      else
      {
        lumaStride = MAX_CU_SIZE + 1;
        temp       = PelBuf(m_piTemp + lumaStride + 1, lumaStride, Size(chromaArea));
      }

      CclmModel cclmModels;
      PU::ccpParamsToCclmModel(COMPONENT_Cb, ccpCand, cclmModels);
      cost += xUpdateOffsetsAndGetCostCCLM<false>(pu, COMPONENT_Cb, pu.Cb(), cclmModels, 1, ccpCand.glmIdc);
      PU::ccpParamsToCclmModel(COMPONENT_Cr, ccpCand, cclmModels);
      cost += xUpdateOffsetsAndGetCostCCLM<false>(pu, COMPONENT_Cr, pu.Cr(), cclmModels, 1, ccpCand.glmIdc);
    }
    else if (ccpCand.type & CCP_TYPE_GLM4567)
    {
      int glmIdc       = ccpCand.glmIdc;
      int chromaOffset = 0;
#if JVET_AB0174_CCCM_DIV_FREE
      int lumaOffset = m_glmLumaOffset - ccpCand.lumaOffset;
#else
      int lumaOffset = 0;
#endif
      CccmModel glmModel( GLM_NUM_PARAMS, bitDepth );
      PU::ccpParamsToGlmModel(COMPONENT_Cb, ccpCand, glmModel);
      cost += xUpdateOffsetsAndGetCostGLM<false>(pu, COMPONENT_Cb, pu.Cb(), glmModel, glmIdc, lumaOffset, chromaOffset);
      PU::ccpParamsToGlmModel(COMPONENT_Cr, ccpCand, glmModel);
      cost += xUpdateOffsetsAndGetCostGLM<false>(pu, COMPONENT_Cr, pu.Cr(), glmModel, glmIdc, lumaOffset, chromaOffset);
    }
    else
    {
      THROW("Invalid type!");
    }
  }
  return cost;
}

void IntraPrediction::predCCPCandidate(PredictionUnit &pu, PelBuf &predCb, PelBuf &predCr)
{
  const int bitDepth = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);

  if (pu.idxNonLocalCCP)
  {
    CompArea                   chromaArea = pu.Cb();
    int                        cWidth     = chromaArea.width;
    int                        cHeight    = chromaArea.height;
    CccmModel cccmModelCb[2] = { CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth ) };
    CccmModel cccmModelCr[2] = { CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth ) };

    CHECK(pu.curCand.type == CCP_TYPE_NONE, "CCP model with no type")
    {
      if (pu.curCand.type & (CCP_TYPE_CCCM | CCP_TYPE_GLCCCM))
      {
#if JVET_AB0174_CCCM_DIV_FREE
        int lumaOffset = m_cccmLumaOffset - pu.curCand.lumaOffset;
#else
        int lumaOffset = 0;
#endif
        int refSizeX = pu.curCand.corOffX;
        int refSizeY = pu.curCand.corOffY;
        int offsetCb[2] = { 0, 0 };
        int offsetCr[2] = { 0, 0 };

        PU::ccpParamsToCccmModel(pu.curCand, cccmModelCb, cccmModelCr);

        if (!(pu.curCand.type & CCP_TYPE_MMLM))
        {
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), cccmModelCb, 0, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY);
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), cccmModelCr, 0, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY);
#endif
          xCccmApplyModelOffset(pu, COMPONENT_Cb, cccmModelCb[0], 0, 0, predCb, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY);
          xCccmApplyModelOffset(pu, COMPONENT_Cr, cccmModelCr[0], 0, 0, predCr, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY);
        }
        else
        {
          // Multimode case
          int modelThr = pu.curCand.yThres;
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), cccmModelCb, modelThr, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY);
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), cccmModelCr, modelThr, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY);
#endif
          xCccmApplyModelOffset(pu, COMPONENT_Cb, cccmModelCb[0], 1, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY);
          xCccmApplyModelOffset(pu, COMPONENT_Cr, cccmModelCr[0], 1, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY);

          xCccmApplyModelOffset(pu, COMPONENT_Cb, cccmModelCb[1], 2, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY);
          xCccmApplyModelOffset(pu, COMPONENT_Cr, cccmModelCr[1], 2, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY);
        }
      }
#if JVET_AF0073_INTER_CCP_MERGE
      else if (pu.curCand.type & CCP_TYPE_INTER_CCCM)
      {
        CccmModel interCccmModels[] = { CccmModel( INTER_CCCM_NUM_PARAMS, bitDepth ), CccmModel( INTER_CCCM_NUM_PARAMS, bitDepth ) };

#if JVET_AB0174_CCCM_DIV_FREE
        int lumaOffset = m_cccmLumaOffset - pu.curCand.lumaOffset;
#else
        int lumaOffset = 0;
#endif
        pu.cccmNoSubFlag = 1;
        int offsetCb = 0;
        int offsetCr = 0;

        PU::ccpParamsToCccmModel(pu.curCand, interCccmModels[0], interCccmModels[1]);

        xInterCccmApplyModelOffset(pu, COMPONENT_Cb, interCccmModels[0], predCb, lumaOffset, offsetCb);
        xInterCccmApplyModelOffset(pu, COMPONENT_Cr, interCccmModels[1], predCr, lumaOffset, offsetCr);
        pu.cccmNoSubFlag = 0;
      }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
      else if (pu.curCand.type & CCP_TYPE_NSCCCM)
      {
        CccmModel nscccmModelCb[2] = { CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ), CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ) };
        CccmModel nscccmModelCr[2] = { CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ), CccmModel( CCCM_NO_SUB_NUM_PARAMS, bitDepth ) };
#if JVET_AB0174_CCCM_DIV_FREE
        int lumaOffset = m_cccmLumaOffset - pu.curCand.lumaOffset;
#else
        int lumaOffset = 0;
#endif
        pu.cccmNoSubFlag = 1;
        int offsetCb[2] = { 0, 0 };
        int offsetCr[2] = { 0, 0 };

        PU::ccpParamsToCccmModel(pu.curCand, nscccmModelCb, nscccmModelCr);

        if (!(pu.curCand.type & CCP_TYPE_MMLM))
        {
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), nscccmModelCb, 0, lumaOffset, offsetCb, pu.curCand.type );
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), nscccmModelCr, 0, lumaOffset, offsetCr, pu.curCand.type );
#endif
          xCccmApplyModelOffset(pu, COMPONENT_Cb, nscccmModelCb[0], 0, 0, predCb, lumaOffset, offsetCb, pu.curCand.type );
          xCccmApplyModelOffset(pu, COMPONENT_Cr, nscccmModelCr[0], 0, 0, predCr, lumaOffset, offsetCr, pu.curCand.type );
        }
        else
        {
          // Multimode case
          int modelThr = pu.curCand.yThres;
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), nscccmModelCb, modelThr, lumaOffset, offsetCb, pu.curCand.type );
          xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), nscccmModelCr, modelThr, lumaOffset, offsetCr, pu.curCand.type );
#endif
          xCccmApplyModelOffset(pu, COMPONENT_Cb, nscccmModelCb[0], 1, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type );
          xCccmApplyModelOffset(pu, COMPONENT_Cr, nscccmModelCr[0], 1, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type );

          xCccmApplyModelOffset(pu, COMPONENT_Cb, nscccmModelCb[1], 2, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type );
          xCccmApplyModelOffset(pu, COMPONENT_Cr, nscccmModelCr[1], 2, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type );
        }

        pu.cccmNoSubFlag = 0;
      }
#endif
#if JVET_AD0202_CCCM_MDF
      else if (pu.curCand.type & CCP_TYPE_MDFCCCM)
      {
        pu.cccmMultiFilterIdx = pu.curCand.cccmMultiFilterIdx;
#if JVET_AB0174_CCCM_DIV_FREE
        int lumaOffset = m_cccmLumaOffset - pu.curCand.lumaOffset;
#else
        int lumaOffset = 0;
#endif
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
        int refSizeX = pu.curCand.corOffX;
        int refSizeY = pu.curCand.corOffY;
#endif
        int offsetCb[2] = { 0, 0 };
        int offsetCr[2] = { 0, 0 };

        if (pu.curCand.cccmMultiFilterIdx == 1)
        {
          CccmModel mfcccmModelCb[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth) };
          CccmModel mfcccmModelCr[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, bitDepth) };

          PU::ccpParamsToCccmModel(pu.curCand, mfcccmModelCb, mfcccmModelCr);

          if (!(pu.curCand.type & CCP_TYPE_MMLM))
          {
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), mfcccmModelCb, 0, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), mfcccmModelCr, 0, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
#endif
            xCccmApplyModelOffset(pu, COMPONENT_Cb, mfcccmModelCb[0], 0, 0, predCb, lumaOffset, offsetCb, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
            xCccmApplyModelOffset(pu, COMPONENT_Cr, mfcccmModelCr[0], 0, 0, predCr, lumaOffset, offsetCr, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
          }
          else
          {
            // Multimode case
            int modelThr = pu.curCand.yThres;
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), mfcccmModelCb, modelThr, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), mfcccmModelCr, modelThr, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
#endif
            xCccmApplyModelOffset(pu, COMPONENT_Cb, mfcccmModelCb[0], 1, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
            xCccmApplyModelOffset(pu, COMPONENT_Cr, mfcccmModelCr[0], 1, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );

            xCccmApplyModelOffset(pu, COMPONENT_Cb, mfcccmModelCb[1], 2, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
            xCccmApplyModelOffset(pu, COMPONENT_Cr, mfcccmModelCr[1], 2, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
          }
        }
        else // if (pu.curCand.cccmMultiFilterIdx == 2 || pu.curCand.cccmMultiFilterIdx == 3)
        {
          CccmModel mfcccmModelCb[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth) };
          CccmModel mfcccmModelCr[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, bitDepth) };

          PU::ccpParamsToCccmModel(pu.curCand, mfcccmModelCb, mfcccmModelCr);

          if (!(pu.curCand.type & CCP_TYPE_MMLM))
          {
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), mfcccmModelCb, 0, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), mfcccmModelCr, 0, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
#endif
            xCccmApplyModelOffset(pu, COMPONENT_Cb, mfcccmModelCb[0], 0, 0, predCb, lumaOffset, offsetCb, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
            xCccmApplyModelOffset(pu, COMPONENT_Cr, mfcccmModelCr[0], 0, 0, predCr, lumaOffset, offsetCr, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
          }
          else
          {
            // Multimode case
            int modelThr = pu.curCand.yThres;
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cb, pu.Cb(), mfcccmModelCb, modelThr, lumaOffset, offsetCb, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
            xUpdateOffsetsAndGetCostCCCM<true>(pu, COMPONENT_Cr, pu.Cr(), mfcccmModelCr, modelThr, lumaOffset, offsetCr, pu.curCand.type, refSizeX, refSizeY, pu.curCand.cccmMultiFilterIdx );
#endif
            xCccmApplyModelOffset(pu, COMPONENT_Cb, mfcccmModelCb[0], 1, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
            xCccmApplyModelOffset(pu, COMPONENT_Cr, mfcccmModelCr[0], 1, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );

            xCccmApplyModelOffset(pu, COMPONENT_Cb, mfcccmModelCb[1], 2, modelThr, predCb, lumaOffset, offsetCb, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
            xCccmApplyModelOffset(pu, COMPONENT_Cr, mfcccmModelCr[1], 2, modelThr, predCr, lumaOffset, offsetCr, pu.curCand.type, m_cccmBlkArea.x - m_cccmRefArea.x, m_cccmBlkArea.y - m_cccmRefArea.y );
          }
        }

        pu.cccmMultiFilterIdx = 0;
      }
#endif
      else if (pu.curCand.type & (CCP_TYPE_CCLM | CCP_TYPE_GLM0123))
      {
        CPelBuf temp;
        int     lumaStride;

        if (pu.curCand.type & CCP_TYPE_GLM0123)
        {
          Pel *glmTemp = m_glmTempCb[pu.curCand.glmIdc];
          lumaStride   = 2 * MAX_CU_SIZE + 1;
          temp         = PelBuf(glmTemp + lumaStride + 1, lumaStride, Size(chromaArea));
        }
        else
        {
          lumaStride = MAX_CU_SIZE + 1;
          temp       = PelBuf(m_piTemp + lumaStride + 1, lumaStride, Size(chromaArea));
        }

        CclmModel modelsCb, modelsCr;
        PU::ccpParamsToCclmModel(COMPONENT_Cb, pu.curCand, modelsCb);
        PU::ccpParamsToCclmModel(COMPONENT_Cr, pu.curCand, modelsCr);

        if (!(pu.curCand.type & CCP_TYPE_MMLM))
        {
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
          xUpdateOffsetsAndGetCostCCLM<true>(pu, COMPONENT_Cb, pu.Cb(), modelsCb, 1, pu.curCand.glmIdc);
          xUpdateOffsetsAndGetCostCCLM<true>(pu, COMPONENT_Cr, pu.Cr(), modelsCr, 1, pu.curCand.glmIdc);
#endif
          predCb.copyFrom(temp);
          predCr.copyFrom(temp);

          predCb.linearTransform(modelsCb.a, modelsCb.shift, modelsCb.b, true, pu.cs->slice->clpRng(COMPONENT_Cb));
          predCr.linearTransform(modelsCr.a, modelsCr.shift, modelsCr.b, true, pu.cs->slice->clpRng(COMPONENT_Cb));
        }
        else
        {
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
          xUpdateOffsetsAndGetCostCCLM<true>(pu, COMPONENT_Cb, pu.Cb(), modelsCb, 2, pu.curCand.glmIdc);
          xUpdateOffsetsAndGetCostCCLM<true>(pu, COMPONENT_Cr, pu.Cr(), modelsCr, 2, pu.curCand.glmIdc);
#endif
          auto applyMMCM = [&](PelBuf &predBuf, const CclmModel &cclmModel)
          {
            Pel       *chromaPred   = predBuf.bufAt(0, 0);
            const Pel *lumaReco     = temp.bufAt(0, 0);
            int        chromaStride = predBuf.stride;

            for (int i = 0; i < cHeight; i++)
            {
              for (int j = 0; j < cWidth; j++)
              {
                if (lumaReco[j] <= cclmModel.yThres)
                {
                  chromaPred[j] = (Pel) ClipPel(((cclmModel.a * lumaReco[j]) >> cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(COMPONENT_Cb));
                }
                else
                {
                  chromaPred[j] = (Pel) ClipPel(((cclmModel.a2 * lumaReco[j]) >> cclmModel.shift2) + cclmModel.b2, pu.cs->slice->clpRng(COMPONENT_Cb));
                }
              }
              chromaPred += chromaStride;
              lumaReco += lumaStride;
            }
          };
          applyMMCM(predCb, modelsCb);
          applyMMCM(predCr, modelsCr);
        }
        PU::cclmModelToCcpParams(COMPONENT_Cb, pu.curCand, modelsCb);
        PU::cclmModelToCcpParams(COMPONENT_Cr, pu.curCand, modelsCr);
      }
      else if (pu.curCand.type & CCP_TYPE_GLM4567)
      {
#if JVET_AB0174_CCCM_DIV_FREE
        int lumaOffset = m_glmLumaOffset - pu.curCand.lumaOffset;
#else
        int lumaOffset = 0;
#endif
        int glmIdc       = pu.curCand.glmIdc;
        int chromaOffset = 0;
        CccmModel glmModel( GLM_NUM_PARAMS, bitDepth);

        PU::ccpParamsToGlmModel(COMPONENT_Cb, pu.curCand, glmModel);
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
        xUpdateOffsetsAndGetCostGLM<true>(pu, COMPONENT_Cb, pu.Cb(), glmModel, glmIdc, lumaOffset, chromaOffset);
#endif
        xGlmApplyModelOffset(pu, COMPONENT_Cb, pu.Cb(), glmModel, glmIdc, predCb, lumaOffset, chromaOffset);

        PU::ccpParamsToGlmModel(COMPONENT_Cr, pu.curCand, glmModel);
#if !JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE
        xUpdateOffsetsAndGetCostGLM<true>(pu, COMPONENT_Cr, pu.Cr(), glmModel, glmIdc, lumaOffset, chromaOffset);
#endif
        xGlmApplyModelOffset(pu, COMPONENT_Cr, pu.Cr(), glmModel, glmIdc, predCr, lumaOffset, chromaOffset);
      }
      else
      {
        THROW("Invalid Type");
      }
    }
  }
}
#endif

#if JVET_AF0073_INTER_CCP_MERGE
void IntraPrediction::xInterCccmApplyModelOffset(const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, PelBuf& piPred, int lumaOffset, int chromaOffset)
{
  CHECK(!pu.cccmNoSubFlag, "cccmNoSubFlag shall be enabled");

  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compId));
  static Pel    samples[INTER_CCCM_NUM_PARAMS];

  const PelBuf  refLumaBlk = xCccmGetLumaPuBuf(pu);
  const int chromaScaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc());
  const int chromaScaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc());
  const int stepX = 1 << chromaScaleX;
  const int stepY = 1 << chromaScaleY;
  int offset = chromaOffset;

  for (int y = 0; y < refLumaBlk.height; y += stepY)
  {
    for (int x = 0; x < refLumaBlk.width; x += stepX)
    {
      const Pel* src0 = refLumaBlk.bufAt(x, y);
      const Pel* src1 = refLumaBlk.bufAt(x, y + 1);

      samples[0] = src0[ 0] + lumaOffset;
      samples[1] = src1[ 0] + lumaOffset;
      samples[2] = src0[-1] + lumaOffset;
      samples[3] = src0[ 1] + lumaOffset;
      samples[4] = src1[-1] + lumaOffset;
      samples[5] = src1[ 1] + lumaOffset;
      samples[6] = cccmModel.nonlinear((samples[0] + samples[1] + 1) >> 1);
      samples[7] = cccmModel.bias();
      piPred.at(x >> chromaScaleX, y >> chromaScaleY) = ClipPel<Pel>(cccmModel.convolve(samples) + offset, clpRng);
    }
  }
}
int IntraPrediction::xGetCostInterCccm(const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, CccmModel& cccmModel, int lumaOffset, int chromaOffset)
{
  CHECK(!pu.cccmNoSubFlag, "cccmNoSubFlag shall be enabled");
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(compID));
  static Pel samples[INTER_CCCM_NUM_PARAMS];

  CPelBuf   refLumaBlk = xCccmGetLumaPuBuf(pu);
  const int chromaScaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc());
  const int chromaScaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc());
  const int stepX = 1 << chromaScaleX;
  const int stepY = 1 << chromaScaleY;

  const SizeType cWidth = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure& cs = *(pu.cs);
  const CodingUnit& cu = *(pu.cu);

  const bool aboveAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID)) ? true : false;
  const bool leftAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID)) ? true : false;

  const Pel* curChroma0;

  PelBuf chromaReco = cs.picture->getRecoBuf(chromaArea);

  Pel* curChromaBuf = chromaReco.buf;
  const int curStride = chromaReco.stride;

  int sad = 0;

  Pel predChroma;

  if (aboveAvailable)
  {
    curChroma0 = curChromaBuf - curStride;
    for (int pos = 0, x = 0; pos < cWidth; pos++, x += stepX)
    {
      const Pel* src0 = refLumaBlk.bufAt(x, -stepY);
      const Pel* src1 = refLumaBlk.bufAt(x, -stepY + 1);
      samples[0] = src0[0] + lumaOffset;
      samples[1] = src1[0] + lumaOffset;
      samples[2] = src0[-1] + lumaOffset;
      samples[3] = src0[1] + lumaOffset;
      samples[4] = src1[-1] + lumaOffset;
      samples[5] = src1[1] + lumaOffset;
      samples[6] = cccmModel.nonlinear((samples[0] + samples[1] + 1) >> 1);
      samples[7] = cccmModel.bias();

      predChroma = cccmModel.convolve(samples) + chromaOffset;
      predChroma = ClipPel<Pel>(predChroma, clpRng);
      sad += abs(curChroma0[pos] - predChroma);
    }
  }

  if (leftAvailable)
  {
    curChroma0 = curChromaBuf - 1;
    for (int pos = 0, y = 0; pos < cHeight; pos++, y += stepY)
    {
      const Pel* src0 = refLumaBlk.bufAt(-stepX, y);
      const Pel* src1 = refLumaBlk.bufAt(-stepX, y + 1);
      samples[0] = src0[0] + lumaOffset;
      samples[1] = src1[0] + lumaOffset;
      samples[2] = src0[-1] + lumaOffset;
      samples[3] = src0[1] + lumaOffset;
      samples[4] = src1[-1] + lumaOffset;
      samples[5] = src1[1] + lumaOffset;
      samples[6] = cccmModel.nonlinear((samples[0] + samples[1] + 1) >> 1);
      samples[7] = cccmModel.bias();

      predChroma = cccmModel.convolve(samples) + chromaOffset;
      predChroma = ClipPel<Pel>(predChroma, clpRng);
      sad += abs(curChroma0[pos * curStride] - predChroma);
    }
  }
  return sad;
}
void IntraPrediction::xAddOnTheFlyCalcCCPCands4InterBlk(const PredictionUnit &pu, CompArea chromaArea, CCPModelCandidate candList[], int &validNum)
{
  if (!pu.cs->slice->getCheckLDC())
  {
    return;
  }

  int  maxCandIdx = validNum;
  bool isValid    = false;

#if JVET_AC0094_REF_SAMPLES_OPT
  m_leftRefLength = chromaArea.height << 3;
  m_topRefLength  = chromaArea.width << 3;
#else
  m_leftRefLength     = (height << 1);
  m_topRefLength      = (width << 1);
#endif

  // single model CCCM w/ downsampling
  xCccmCreateLumaRef(pu, chromaArea);
  const int bitDepth       = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  CccmModel cccmModelCb[2] = { CccmModel(CCCM_NUM_PARAMS, bitDepth), CccmModel(CCCM_NUM_PARAMS, bitDepth) };
  CccmModel cccmModelCr[2] = { CccmModel(CCCM_NUM_PARAMS, bitDepth), CccmModel(CCCM_NUM_PARAMS, bitDepth) };
  xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 0, 0, -1, true);
  for (int i = 0; i < (CCCM_NUM_PARAMS - 1); i++)
  {
    if (cccmModelCb[0].params[i] != 0 || cccmModelCr[0].params[i] != 0)
    {
      isValid = true;
      break;
    }
  }
  if (isValid)
  {
#if JVET_AC0054_GLCCCM
    candList[maxCandIdx].type    = (pu.glCccmFlag ? CCP_TYPE_GLCCCM : CCP_TYPE_CCCM);
    candList[maxCandIdx].corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
    candList[maxCandIdx].corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;
#else
    ccpCand.type = CCP_TYPE_CCCM;
#endif
    PU::cccmModelToCcpParams(candList[maxCandIdx], cccmModelCb, cccmModelCr, 0
#if JVET_AB0174_CCCM_DIV_FREE
                             , m_cccmLumaOffset
#endif
    );
    maxCandIdx++;
  }

  CHECK(maxCandIdx > MAX_CCP_CAND_LIST_SIZE, "Invlid number of InterCCP merge candidates");
  validNum = maxCandIdx;
}
void IntraPrediction::selectCcpMergeCand(PredictionUnit& pu, CCPModelCandidate candList[], int reorderlistSize)
{
  int candCost[MAX_CCP_CAND_LIST_SIZE];
  for (int i = 0; i < reorderlistSize; i++)
  {
    candCost[i] = xGetOneCCPCandCost(pu, candList[i]);
  }

  //Find the candidate with smallest template cost
  int minCost = candCost[0];
  int minIdx = 0;
  for (int i = 1; i < reorderlistSize; i++)
  {
    if (candCost[i] < minCost)
    {
      minCost = candCost[i];
      minIdx = i;
    }
  }
  if (minIdx != 0)
  {
    CCPModelCandidate tmpCand = candList[0];
    int tmpCost = candCost[0];
    candList[0] = candList[minIdx];
    candCost[0] = minCost;
    candList[minIdx] = tmpCand;
    candCost[minIdx] = tmpCost;
  }
  return;
}
void IntraPrediction::combineCcpAndInter(PredictionUnit& pu, PelBuf& inPredCb, PelBuf& inPredCr, PelBuf& outPredCb, PelBuf& outPredCr)
{
  CompArea chromaArea = pu.Cb();
  int predCbStride = MAX_CU_SIZE + 1;
  PelBuf predCbIntra = PelBuf(m_pCcpMerge[0] + predCbStride + 1, predCbStride, Size(chromaArea));
  int predCrStride = MAX_CU_SIZE + 1;
  PelBuf predCrIntra = PelBuf(m_pCcpMerge[1] + predCrStride + 1, predCrStride, Size(chromaArea));
  predCCPCandidate(pu, predCbIntra, predCrIntra);

  Pel* predCbIntraBuf = predCbIntra.buf;
  const int predCbIntraStride = predCbIntra.stride;
  Pel* predCrIntraBuf = predCrIntra.buf;
  const int predCrIntraStride = predCrIntra.stride;

  Pel* predCbInterBuf = inPredCb.buf;
  const int predCbInterStride = inPredCb.stride;
  Pel* predCrInterBuf = inPredCr.buf;
  const int predCrInterStride = inPredCr.stride;

  Pel* predCbDstBuf = outPredCb.buf;
  const int predCbDstStride = outPredCb.stride;
  Pel* predCrDstBuf = outPredCr.buf;
  const int predCrDstStride = outPredCr.stride;

  const ClpRng& clpRngCb = pu.cu->cs->slice->clpRng(COMPONENT_Cb);
  const ClpRng& clpRngCr = pu.cu->cs->slice->clpRng(COMPONENT_Cr);
  for (int cntH = 0; cntH < outPredCb.height; cntH++)
  {
    for (int cntW = 0; cntW < outPredCb.width; cntW++)
    {
      predCbDstBuf[cntH * predCbDstStride + cntW] = ClipPel(((3 * predCbIntraBuf[cntH * predCbIntraStride + cntW] + predCbInterBuf[cntH * predCbInterStride + cntW] + 2) >> 2), clpRngCb);
      predCrDstBuf[cntH * predCrDstStride + cntW] = ClipPel(((3 * predCrIntraBuf[cntH * predCrIntraStride + cntW] + predCrInterBuf[cntH * predCrInterStride + cntW] + 2) >> 2), clpRngCr);
    }
  }
}
#endif

#if JVET_AB0174_CCCM_DIV_FREE
#define DIV_PREC_BITS       14
#define DIV_PREC_BITS_POW2  8
#define DIV_SLOT_BITS       3
#define DIV_INTR_BITS      (DIV_PREC_BITS - DIV_SLOT_BITS)
#define DIV_INTR_ROUND     (1 << DIV_INTR_BITS >> 1)

int64_t xDivide(int64_t num, int64_t denom) // Note: assumes positive denominator
{
  static const int pow2W[8] = {   214,   153,   113,    86,    67,    53,    43,    35  }; // DIV_PREC_BITS_POW2
  static const int pow2O[8] = {  4822,  5952,  6624,  6792,  6408,  5424,  3792,  1466  }; // DIV_PREC_BITS
  static const int pow2B[8] = { 12784, 12054, 11670, 11583, 11764, 12195, 12870, 13782  }; // DIV_PREC_BITS

  int shift     = floorLog2Uint64(denom);
  int round     = 1 << shift >> 1;
  int normDiff  = (((denom << DIV_PREC_BITS) + round) >> shift) & ((1 << DIV_PREC_BITS) - 1);
  int diffFull  = normDiff >> DIV_INTR_BITS;
  int normDiff2 = normDiff - pow2O[diffFull];

  int scale     = ((pow2W[diffFull] * ((normDiff2 * normDiff2) >> DIV_PREC_BITS)) >> DIV_PREC_BITS_POW2) - (normDiff2 >> 1) + pow2B[diffFull];

  return ( (num << (CCCM_DECIM_BITS - DIV_PREC_BITS)) * scale + round) >> shift;
}

#if JVET_AC0053_GAUSSIAN_SOLVER
void xGetDivScaleRoundShift(int64_t denom, int &scale, int &round, int &shift) // Note: assumes positive denominator
{
  static const int pow2W[8] = {   214,   153,   113,    86,    67,    53,    43,    35  }; // DIV_PREC_BITS_POW2
  static const int pow2O[8] = {  4822,  5952,  6624,  6792,  6408,  5424,  3792,  1466  }; // DIV_PREC_BITS
  static const int pow2B[8] = { 12784, 12054, 11670, 11583, 11764, 12195, 12870, 13782  }; // DIV_PREC_BITS

  shift         = floorLog2Uint64(denom);
  round         = 1 << shift >> 1;
  int normDiff  = (((denom << DIV_PREC_BITS) + round) >> shift) & ((1 << DIV_PREC_BITS) - 1);
  int diffFull  = normDiff >> DIV_INTR_BITS;
  int normDiff2 = normDiff - pow2O[diffFull];

  scale         = ((pow2W[diffFull] * ((normDiff2 * normDiff2) >> DIV_PREC_BITS)) >> DIV_PREC_BITS_POW2) - (normDiff2 >> 1) + pow2B[diffFull];
  scale       <<= CCCM_DECIM_BITS - DIV_PREC_BITS;
}
#endif

#undef DIV_PREC_BITS
#undef DIV_PREC_BITS_POW2
#undef DIV_SLOT_BITS
#undef DIV_INTR_BITS
#undef DIV_INTR_ROUND

int xCccmDivideLowPrec(int64_t num, int64_t denom)
{
  if ( num < 0 )
  {
    return -int(xDivide(-num, denom) >> CCCM_DECIM_BITS);
  }
  else
  {
    return int(xDivide(num, denom) >> CCCM_DECIM_BITS);
  }
}

int64_t xCccmDivide(int64_t num, int64_t denom) // Note: assumes positive denominator
{
  return xDivide(num, denom);
}
#endif

#if JVET_AD0120_LBCCP
#if JVET_AA0057_CCCM
uint32_t IntraPrediction::xCalculateCCLMcost(const PredictionUnit &pu, const ComponentID compID, int intraDir, const CompArea &chromaArea, const CclmModel &cclmModel)
{
  int    srcStride = 0;
  PelBuf temp;

#if MMLM
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX) || (intraDir == MMLM_L_IDX) || (intraDir == MMLM_T_IDX) || (m_encPreRDRun && intraDir == MMLM_CHROMA_IDX))
#else
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
#endif
  {
    srcStride = 2 * MAX_CU_SIZE + 1;
    temp      = PelBuf(m_pMdlmTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  else
  {
    srcStride = MAX_CU_SIZE + 1;
    temp      = PelBuf(m_piTemp + srcStride + 1, srcStride, Size(chromaArea));
  }
  uint32_t totalSAD = 0;

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure  &cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const bool aboveAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID)) ? true : false;
  const bool leftAvailable  = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID)) ? true : false;

  Pel *srcColor0, *curChroma0;

  srcColor0 = temp.bufAt(0, 0);

  PelBuf chromaReco = cs.picture->getRecoBuf(chromaArea);

  Pel      *curChromaBuf = chromaReco.buf;
  const int curStride    = chromaReco.stride;

  if (aboveAvailable)
  {
    curChroma0 = curChromaBuf - curStride;
    Pel *src   = srcColor0 - srcStride;
#if MMLM
    if (PU::isMultiModeLM(pu.intraDir[1]))
    {
      Pel predChroma;
      for (int pos = 0; pos < cWidth; pos++)
      {
        if (src[pos] <= cclmModel.yThres)
        {
          predChroma = ClipPel(rightShift(cclmModel.a * src[pos], cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(compID));
        }
        else
        {
          predChroma = ClipPel(rightShift(cclmModel.a2 * src[pos], cclmModel.shift2) + cclmModel.b2, pu.cs->slice->clpRng(compID));
        }
        totalSAD += abs(predChroma - curChroma0[pos]);
      }
    }
    else
#endif
    {
      for (int pos = 0; pos < cWidth; pos++)
      {
        Pel predChroma = ClipPel(rightShift(cclmModel.a * src[pos], cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(compID));
        totalSAD += abs(predChroma - curChroma0[pos]);
      }
    }
  }

  if (leftAvailable)
  {
    curChroma0 = curChromaBuf - 1;

    Pel *src = srcColor0 - 1;

#if MMLM
    if (PU::isMultiModeLM(pu.intraDir[1]))
    {
      Pel predChroma;
      for (int pos = 0; pos < cHeight; pos++)
      {
        if (src[pos * srcStride] <= cclmModel.yThres)
        {
          predChroma = ClipPel(rightShift(cclmModel.a * src[pos * srcStride], cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(compID));
        }
        else
        {
          predChroma = ClipPel(rightShift(cclmModel.a2 * src[pos * srcStride], cclmModel.shift2) + cclmModel.b2, pu.cs->slice->clpRng(compID));
        }
        totalSAD += abs(predChroma - curChroma0[pos * curStride]);
      }
    }
    else
#endif
    {
      for (int pos = 0; pos < cHeight; pos++)
      {
        Pel predChroma = ClipPel(rightShift(cclmModel.a * src[pos * srcStride], cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(compID));
        totalSAD += abs(predChroma - curChroma0[pos * curStride]);
      }
    }
  }

  return totalSAD;
}

void IntraPrediction::applyChromaLM(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu, const CompArea &chromaArea, int intraDir, const CclmModel &cclmModel)
{
  int    iLumaStride = 0;
  PelBuf temp;

#if MMLM
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX) || (intraDir == MMLM_L_IDX) || (intraDir == MMLM_T_IDX) || (m_encPreRDRun && intraDir == MMLM_CHROMA_IDX))
#else
  if ((intraDir == MDLM_L_IDX) || (intraDir == MDLM_T_IDX))
#endif
  {
    iLumaStride = 2 * MAX_CU_SIZE + 1;
    temp        = PelBuf(m_pMdlmTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }
  else
  {
    iLumaStride = MAX_CU_SIZE + 1;
    temp        = PelBuf(m_piTemp + iLumaStride + 1, iLumaStride, Size(chromaArea));
  }

  piPred.copyFrom(temp);
#if MMLM
  if (PU::isMultiModeLM(pu.intraDir[1]))
  {
    Pel *pPred        = piPred.bufAt(0, 0);
    Pel *pLuma        = temp.bufAt(0, 0);
    int  uiPredStride = piPred.stride;
    int  uiCWidth     = chromaArea.width;
    int  uiCHeight    = chromaArea.height;

    for (int i = 0; i < uiCHeight; i++)
    {
      for (int j = 0; j < uiCWidth; j++)
      {
        if (pLuma[j] <= cclmModel.yThres)
        {
          pPred[j] = (Pel) ClipPel(((cclmModel.a * pLuma[j]) >> cclmModel.shift) + cclmModel.b, pu.cs->slice->clpRng(compID));
        }
        else
        {
          pPred[j] = (Pel) ClipPel(((cclmModel.a2 * pLuma[j]) >> cclmModel.shift2) + cclmModel.b2, pu.cs->slice->clpRng(compID));
        }
      }
      pPred += uiPredStride;
      pLuma += iLumaStride;
    }
  }
  else
  {
#endif
    piPred.linearTransform(cclmModel.a, cclmModel.shift, cclmModel.b, true, pu.cs->slice->clpRng(compID));
#if MMLM
  }
#endif
}

uint32_t IntraPrediction::xCalculateCCCMcost(const PredictionUnit &pu, const ComponentID compID, int intraDir, const CompArea &chromaArea, CccmModel cccmModel[2], int modelThr)
{
  int    srcStride = 0;

  CHECK( compID != chromaArea.compID, "Invalid component ID");

  const ClpRng &clpRng(pu.cu->cs->slice->clpRng(compID));
  Pel* samples = m_samples;

  CPelBuf temp = xCccmGetLumaPuBuf(pu);

  uint32_t totalSAD = 0;

  const SizeType cWidth  = chromaArea.width;
  const SizeType cHeight = chromaArea.height;

  CodingStructure  &cs = *(pu.cs);
  const CodingUnit &cu = *(pu.cu);

  const bool aboveAvailable = cu.cs->getCU(cu.blocks[compID].pos().offset(0, -1), toChannelType(compID)) ? true : false;
  const bool leftAvailable  = cu.cs->getCU(cu.blocks[compID].pos().offset(-1, 0), toChannelType(compID)) ? true : false;
  const bool checkAbove = aboveAvailable && (pu.cccmFlag != 2);
  const bool checkLeft  = leftAvailable && (pu.cccmFlag != 3);

  const Pel *srcColor0, *curChroma0;

  srcColor0 = temp.bufAt(0, 0);
  srcStride = temp.stride;

  PelBuf chromaReco = cs.picture->getRecoBuf(chromaArea);

  Pel      *curChromaBuf = chromaReco.buf;
  const int curStride    = chromaReco.stride;

  if (checkAbove)
  {
    curChroma0     = curChromaBuf - curStride;
    const Pel *src = srcColor0 - srcStride;

    Pel predChroma;
    for (int pos = 0; pos < cWidth; pos++, src++)
    {
      samples[0] = *src;                 // C
      samples[1] = *(src - srcStride);   // N
      samples[2] = *(src + srcStride);   // S
      samples[3] = *(src - 1);           // W
      samples[4] = *(src + 1);           // E
      samples[5] = cccmModel[0].nonlinear(*src);
      samples[6] = cccmModel[0].bias();

#if MMLM
      if (PU::isMultiModeLM(pu.intraDir[1]))
      {
        if (*src <= modelThr)
        {
          predChroma = ClipPel<Pel>(cccmModel[0].convolve(samples), clpRng);
        }
        else
        {
          predChroma = ClipPel<Pel>(cccmModel[1].convolve(samples), clpRng);
        }
      }
      else
#endif
        predChroma = ClipPel<Pel>(cccmModel[0].convolve(samples), clpRng);

      totalSAD += abs(predChroma - curChroma0[pos]);
    }
  }

  if (checkLeft)
  {
    curChroma0 = curChromaBuf - 1;

    const Pel *src = srcColor0 - 1;

    Pel predChroma;
    for (int pos = 0; pos < cHeight; pos++, src += srcStride)
    {
      samples[0] = *src;                 // C
      samples[1] = *(src - srcStride);   // N
      samples[2] = *(src + srcStride);   // S
      samples[3] = *(src - 1);           // W
      samples[4] = *(src + 1);           // E
      samples[5] = cccmModel[0].nonlinear(*src);
      samples[6] = cccmModel[0].bias();

#if MMLM
      if (PU::isMultiModeLM(pu.intraDir[1]))
      {
        if (*src <= modelThr)
        {
          predChroma = ClipPel<Pel>(cccmModel[0].convolve(samples), clpRng);
        }
        else
        {
          predChroma = ClipPel<Pel>(cccmModel[1].convolve(samples), clpRng);
        }
      }
      else
#endif
        predChroma = ClipPel<Pel>(cccmModel[0].convolve(samples), clpRng);
      totalSAD += abs(predChroma - curChroma0[pos * curStride]);
    }
  }

  return totalSAD;
}
#endif

void IntraPrediction::filterPredInside(const ComponentID compID, PelBuf &piPred, const PredictionUnit &pu)
{
  CodingUnit   &cu          = *pu.cu;
  ChannelType   channelType = toChannelType(compID);
  int           W           = pu.blocks[compID].width;
  int           H           = pu.blocks[compID].height;
  const CPelBuf recoBuf     = pu.cs->picture->getRecoBuf(compID);

  const int  recStride = recoBuf.stride;
  const Pel *pRec      = &recoBuf.buf[pu.blocks[compID].y * recStride + pu.blocks[compID].x];
  const int  tmpStride = MAX_CU_SIZE + 2;
  Pel       *pTmp      = m_pCCFilterTemp;
  const int  preStride = piPred.stride;
  Pel       *pPre      = piPred.buf;

  const bool aboveAvailable      = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(0, -1)) ? true : false;
  const bool leftAvailable       = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(-1, 0)) ? true : false;
  const bool leftAboveAvailable  = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(-1, -1)) ? true : false;
  const bool aboveRightAvailable = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(W, -1)) ? true : false;
  const bool leftBelowAvailable  = isPosAvailable(cu, channelType, cu.blocks[compID].pos().offset(-1, H)) ? true : false;

  for (int y = 0; y < H; y++)
  {
    memcpy(&pTmp[1 + (y + 1) * tmpStride], &pPre[y * preStride], sizeof(Pel) * W);
    pTmp[(y + 1) * tmpStride + 1 + W] = pTmp[(y + 1) * tmpStride + W];
  }
  memcpy(&pTmp[1 + (H + 1) * tmpStride], &pTmp[1 + H * tmpStride], sizeof(Pel) * (W + 1));

  if (aboveAvailable)
  {
    memcpy(&pTmp[1], &pRec[-1 * recStride], sizeof(Pel) * W);
  }
  else
  {
    memcpy(&pTmp[1], &pTmp[1 + tmpStride], sizeof(Pel) * W);
  }

  if (leftAvailable)
  {
    for (int y = 0; y < H; y++)
    {
      pTmp[(y + 1) * tmpStride] = pRec[y * recStride - 1];
    }
  }
  else
  {
    for (int y = 0; y < H; y++)
    {
      pTmp[(y + 1) * tmpStride] = pTmp[(y + 1) * tmpStride + 1];
    }
  }

  if (leftAboveAvailable)
  {
    pTmp[0] = pRec[-recStride - 1];
  }
  else
  {
    pTmp[0] = (pTmp[1] + pTmp[tmpStride] + 1) >> 1;
  }

  if (aboveRightAvailable)
  {
    pTmp[W + 1] = pRec[-recStride + W];
  }
  else
  {
    pTmp[W + 1] = pTmp[W];
  }

  if (leftBelowAvailable)
  {
    pTmp[(H + 1) * tmpStride] = pRec[-1 + H * recStride];
  }
  else
  {
    pTmp[(H + 1) * tmpStride] = pTmp[H * tmpStride];
  }

  pTmp = &m_pCCFilterTemp[1 + tmpStride];


  for (int y = 0; y < H; y++)
  {
    for (int x = 0; x < W; x++)
    {
      int sum                 = pTmp[(y -1)* tmpStride + x-1  ] +  pTmp[(y -1)* tmpStride + x]  +  pTmp[(y -1)* tmpStride + x + 1] + pTmp[y * tmpStride + x - 1]  + 8 * pTmp[y * tmpStride + x] + pTmp[y * tmpStride + x + 1] + pTmp[(y + 1) * tmpStride + x - 1] + pTmp[(y + 1) * tmpStride + x] + pTmp[(y + 1) * tmpStride + x + 1];
      pPre[y * preStride + x] = (sum + 8) >> 4;
    }
  }
}
#endif

#if JVET_AA0057_CCCM || JVET_AC0119_LM_CHROMA_FUSION
#if JVET_AB0174_CCCM_DIV_FREE
void IntraPrediction::xCccmSetLumaRefValue(const PredictionUnit& pu)
{
  int lumaPosX = m_cccmBlkArea.x << getComponentScaleX(COMPONENT_Cb, pu.cu->chromaFormat);
  int lumaPosY = m_cccmBlkArea.y << getComponentScaleY(COMPONENT_Cb, pu.cu->chromaFormat);

  if (lumaPosX || lumaPosY)
  {
    lumaPosX = lumaPosX ? lumaPosX - 1 : 0;
    lumaPosY = lumaPosY ? lumaPosY - 1 : 0;

    m_cccmLumaOffset = pu.cs->picture->getRecoBuf(COMPONENT_Y).at(lumaPosX, lumaPosY);
  }
  else
  {
    m_cccmLumaOffset = 1 << (pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  }
}
#endif

// Calculate a single downsampled luma reference value (copied from IntraPrediction::xGetLumaRecPixels)
Pel IntraPrediction::xCccmGetLumaVal(const PredictionUnit& pu, const CPelBuf pi, const int x, const int y
#if JVET_AD0202_CCCM_MDF
  , int downsFilterIdx
#endif
) const
{
  const Pel* piSrc = pi.buf;
  const int iRecStride = pi.stride;
  Pel ypval = 0;
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if (pu.cccmNoSubFlag || pu.chromaFormat == CHROMA_444)
#else
  if (pu.chromaFormat == CHROMA_444)
#endif
  {
    ypval = piSrc[x + iRecStride * y];
  }
  else if (pu.chromaFormat == CHROMA_422)
  {
    int s = 2;
    int offLeft = x > 0 ? -1 : 0;
    s += piSrc[2 * x + iRecStride * y] * 2;
    s += piSrc[2 * x + offLeft + iRecStride * y];
    s += piSrc[2 * x + 1 + iRecStride * y];
    ypval = s >> 2;
  }
  else if (pu.cs->sps->getCclmCollocatedChromaFlag())
  {
    int s = 4;
    int offLeft = x > 0 ? -1 : 0;
    int offAbove = y > 0 ? -1 : 0;
    s += piSrc[2 * x + iRecStride * 2 * y] * 4;
    s += piSrc[2 * x + offLeft + iRecStride * 2 * y];
    s += piSrc[2 * x + 1 + iRecStride * 2 * y];
    s += piSrc[2 * x + iRecStride * (2 * y + 1)];
    s += piSrc[2 * x + iRecStride * (2 * y + offAbove)];
    ypval = s >> 3;
  }
  else
  {
#if JVET_AD0202_CCCM_MDF
    const int lumaPosPicX1 = 2 * x;
    const int lumaPosPicY1 = 2 * y;
    int lumaPosPicX0 = lumaPosPicX1 - 1; lumaPosPicX0 = lumaPosPicX0 < 0 ? 0 : lumaPosPicX0;
    const int lumaPosPicX2 = lumaPosPicX1 + 1;
    const int lumaPosPicY2 = lumaPosPicY1 + 1;
    const int shift0 = iRecStride * lumaPosPicY1;
    const int shift1 = iRecStride * lumaPosPicY2;

    if (downsFilterIdx == 0)
    {
      int s = 4;

      s += piSrc[lumaPosPicX1 + shift0] * 2;
      s += piSrc[lumaPosPicX0 + shift0];
      s += piSrc[lumaPosPicX2 + shift0];
      s += piSrc[lumaPosPicX1 + shift1] * 2;
      s += piSrc[lumaPosPicX0 + shift1];
      s += piSrc[lumaPosPicX2 + shift1];
      ypval = s >> 3;
    }
    else if (downsFilterIdx == 1)
    {
      int s = 0;

      s += piSrc[lumaPosPicX0 + shift0];
      s -= piSrc[lumaPosPicX2 + shift0];
      s += piSrc[lumaPosPicX0 + shift1];
      s -= piSrc[lumaPosPicX2 + shift1];

      ypval = s < 0 ? 0 : s;
    }
    else if (downsFilterIdx == 2)
    {
      int s = 0;

      s += piSrc[lumaPosPicX0 + shift0];
      s += piSrc[lumaPosPicX1 + shift0] * 2;
      s += piSrc[lumaPosPicX2 + shift0];
      s -= piSrc[lumaPosPicX0 + shift1];
      s -= piSrc[lumaPosPicX1 + shift1] * 2;
      s -= piSrc[lumaPosPicX2 + shift1];

      ypval = s < 0 ? 0 : s;
    }
    else
    {
      int s = 0;

      s -= piSrc[lumaPosPicX0 + shift0];
      s += piSrc[lumaPosPicX1 + shift0];
      s += piSrc[lumaPosPicX2 + shift0] * 2;
      s -= piSrc[lumaPosPicX0 + shift1] * 2;
      s -= piSrc[lumaPosPicX1 + shift1];
      s += piSrc[lumaPosPicX2 + shift1];

      ypval = s < 0 ? 0 : s;
    }
#else
    int s = 4;
    int offLeft = x > 0 ? -1 : 0;
    s += piSrc[2 * x + iRecStride * y * 2] * 2;
    s += piSrc[2 * x + offLeft + iRecStride * y * 2];
    s += piSrc[2 * x + 1 + iRecStride * y * 2];
    s += piSrc[2 * x + iRecStride * (y * 2 + 1)] * 2;
    s += piSrc[2 * x + offLeft + iRecStride * (y * 2 + 1)];
    s += piSrc[2 * x + 1 + iRecStride * (y * 2 + 1)];
    ypval = s >> 3;
#endif
  }

#if JVET_AB0174_CCCM_DIV_FREE
  return ypval - m_cccmLumaOffset; // Note: this could have also been included in the rounding offset s to avoid the extra sample based operation
#else
  return ypval;
#endif
}
#endif

#if JVET_AA0057_CCCM
#if JVET_AD0188_CCP_MERGE
void IntraPrediction::predIntraCCCM( PredictionUnit &pu, PelBuf &predCb, PelBuf &predCr, int intraDir )
#else
void IntraPrediction::predIntraCCCM( const PredictionUnit &pu, PelBuf &predCb, PelBuf &predCr, int intraDir )
#endif
{
#if JVET_AE0100_BVGCCCM
  if (pu.bvgCccmFlag)
  {
    CccmModel cccmModelCb[2] = { CccmModel( BVG_CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( BVG_CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
    CccmModel cccmModelCr[2] = { CccmModel( BVG_CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( BVG_CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
    
    if (intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX)
    {
      int minVal = 0, maxVal = 0;
      xBvgCccmCalcBlkRange(pu, minVal, maxVal);
      
      xBvgCccmCalcModels(pu, cccmModelCb[0],  cccmModelCr[0], 0, 0, minVal, maxVal);
      xBvgCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[0], 0, 0, predCb);
      xBvgCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[0], 0, 0, predCr);
    }
    else
    {
      int modelThr = xBvgCccmCalcBlkAver(pu);
      int minVal = 0, maxVal = 0;
      xBvgCccmCalcBlkRange(pu, minVal, maxVal);
      
      xBvgCccmCalcModels(pu, cccmModelCb[0],  cccmModelCr[0], 1, modelThr, minVal, maxVal);
      xBvgCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[0], 1, modelThr, predCb);
      xBvgCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[0], 1, modelThr, predCr);
      
      xBvgCccmCalcModels(pu, cccmModelCb[1],  cccmModelCr[1], 2, modelThr, minVal, maxVal);
      xBvgCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[1], 2, modelThr, predCb);
      xBvgCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[1], 2, modelThr, predCr);
    }
    return;
  }
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if( pu.cccmNoSubFlag )
  {
#if JVET_AD0188_CCP_MERGE
    CccmModel cccmModelCb[2] = { CccmModel( CCCM_NO_SUB_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_NO_SUB_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
    CccmModel cccmModelCr[2] = { CccmModel( CCCM_NO_SUB_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_NO_SUB_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
#else
    CccmModel cccmModelCb( CCCM_NO_SUB_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) );
    CccmModel cccmModelCr( CCCM_NO_SUB_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth( CHANNEL_TYPE_LUMA ) );
#endif

#if JVET_AB0143_CCCM_TS
    if( intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX )
#else
    if( PU::cccmSingleModeAvail( pu, intraDir ) )
#endif
    {
#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 0, 0, predCr);

      pu.curCand.type = CCP_TYPE_NSCCCM;
      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, 0
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels( pu, cccmModelCb, cccmModelCr, 0, 0 );
      xCccmApplyModel( pu, COMPONENT_Cb, cccmModelCb, 0, 0, predCb );
      xCccmApplyModel( pu, COMPONENT_Cr, cccmModelCr, 0, 0, predCr );
#endif
    }
    else
    {
      // Multimode case
      int modelThr = xCccmCalcRefAver( pu );

#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels( pu, cccmModelCb[0], cccmModelCr[0], 1, modelThr );
      xCccmApplyModel( pu, COMPONENT_Cb,   cccmModelCb[0], 1, modelThr, predCb );
      xCccmApplyModel( pu, COMPONENT_Cr,   cccmModelCr[0], 1, modelThr, predCr );

      xCccmCalcModels( pu, cccmModelCb[1], cccmModelCr[1], 2, modelThr );
      xCccmApplyModel( pu, COMPONENT_Cb,   cccmModelCb[1], 2, modelThr, predCb );
      xCccmApplyModel( pu, COMPONENT_Cr,   cccmModelCr[1], 2, modelThr, predCr );

      pu.curCand.type = (CCP_TYPE_NSCCCM | CCP_TYPE_MMLM);
      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, modelThr
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels( pu, cccmModelCb, cccmModelCr, 1, modelThr );
      xCccmApplyModel( pu, COMPONENT_Cb, cccmModelCb, 1, modelThr, predCb );
      xCccmApplyModel( pu, COMPONENT_Cr, cccmModelCr, 1, modelThr, predCr );

      xCccmCalcModels( pu, cccmModelCb, cccmModelCr, 2, modelThr );
      xCccmApplyModel( pu, COMPONENT_Cb, cccmModelCb, 2, modelThr, predCb );
      xCccmApplyModel( pu, COMPONENT_Cr, cccmModelCr, 2, modelThr, predCr );
#endif
    }
  }
  else
#endif
#if JVET_AD0202_CCCM_MDF
  if (pu.cccmMultiFilterIdx == 1)
  {
#if JVET_AD0188_CCP_MERGE
    CccmModel cccmModelCb[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
    CccmModel cccmModelCr[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
#else
    CccmModel cccmModelCb( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
    CccmModel cccmModelCr( CCCM_MULTI_PRED_FILTER_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
#endif

#if JVET_AB0143_CCCM_TS
    if (intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX)
#else
    if (PU::cccmSingleModeAvail(pu, intraDir))
#endif
    {
#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 0, 0, predCr);

      pu.curCand.type = CCP_TYPE_MDFCCCM;
      pu.curCand.cccmMultiFilterIdx = pu.cccmMultiFilterIdx;
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;

      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, 0
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels(pu, cccmModelCb, cccmModelCr, 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 0, 0, predCr);
#endif
    }
    else
    {
      // Multimode case
      int modelThr = xCccmCalcRefAver(pu);

#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 1, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 1, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 1, modelThr, predCr);

      xCccmCalcModels(pu, cccmModelCb[1], cccmModelCr[1], 2, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[1], 2, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[1], 2, modelThr, predCr);

      pu.curCand.type = CCP_TYPE_MDFCCCM | CCP_TYPE_MMLM;
      pu.curCand.cccmMultiFilterIdx = pu.cccmMultiFilterIdx;
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;

      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, modelThr
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels(pu, cccmModelCb, cccmModelCr, 1, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 1, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 1, modelThr, predCr);

      xCccmCalcModels(pu, cccmModelCb, cccmModelCr, 2, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 2, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 2, modelThr, predCr);
#endif
    }
  }
  else if (pu.cccmMultiFilterIdx > 1)
  {
#if JVET_AD0188_CCP_MERGE
    CccmModel cccmModelCb[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
    CccmModel cccmModelCr[2] = { CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
#else
    CccmModel cccmModelCb(CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
    CccmModel cccmModelCr(CCCM_MULTI_PRED_FILTER_NUM_PARAMS2, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA));
#endif

#if JVET_AB0143_CCCM_TS
    if (intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX)
#else
    if (PU::cccmSingleModeAvail(pu, intraDir))
#endif
    {
#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 0, 0, predCr);

      pu.curCand.type = CCP_TYPE_MDFCCCM;
      pu.curCand.cccmMultiFilterIdx = pu.cccmMultiFilterIdx;
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;

      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, 0
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels(pu, cccmModelCb, cccmModelCr, 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 0, 0, predCr);
#endif
    }
    else
    {
      // Multimode case
      int modelThr = xCccmCalcRefAver(pu);

#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 1, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 1, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 1, modelThr, predCr);

      xCccmCalcModels(pu, cccmModelCb[1], cccmModelCr[1], 2, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[1], 2, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[1], 2, modelThr, predCr);

      pu.curCand.type = CCP_TYPE_MDFCCCM | CCP_TYPE_MMLM;
      pu.curCand.cccmMultiFilterIdx = pu.cccmMultiFilterIdx;
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;

      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, modelThr
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels(pu, cccmModelCb, cccmModelCr, 1, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 1, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 1, modelThr, predCr);

      xCccmCalcModels(pu, cccmModelCb, cccmModelCr, 2, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 2, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 2, modelThr, predCr);
#endif
    }
  }
  else
#endif
#if JVET_AD0120_LBCCP
    if (pu.cs->slice->isIntra() && pu.cccmFlag
#if JVET_AC0054_GLCCCM
        && !pu.glCccmFlag
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
        && !pu.cccmNoSubFlag
#endif
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && pu.cs->sps->getTMnoninterToolsEnableFlag()
#endif
    )
  {
    const int                  bitDepth       = pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
    CccmModel cccmModelCb[4] = { CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth) };
    CccmModel cccmModelCr[4] = { CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth), CccmModel( CCCM_NUM_PARAMS, bitDepth) };

#if JVET_AB0143_CCCM_TS
    if (intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX)
#else
    if (PU::cccmSingleModeAvail(pu, intraDir))
#endif
    {
      xCccmCalcModels(pu,cccmModelCb[0], cccmModelCr[0], 0, 0);
      int cccmSAD = xCalculateCCCMcost(pu, COMPONENT_Cb, intraDir, pu.blocks[COMPONENT_Cb], &cccmModelCb[0], 0);
      cccmSAD += xCalculateCCCMcost(pu, COMPONENT_Cr, intraDir, pu.blocks[COMPONENT_Cr], &cccmModelCr[0], 0);

      xCccmCalcModels(pu, cccmModelCb[2], cccmModelCr[2], 0, 0, 2);
      int cccmSADtmp = xCalculateCCCMcost(pu, COMPONENT_Cb, intraDir, pu.blocks[COMPONENT_Cb], &cccmModelCb[2], 0);
      cccmSADtmp += xCalculateCCCMcost(pu, COMPONENT_Cr, intraDir, pu.blocks[COMPONENT_Cr], &cccmModelCr[2], 0);
#if JVET_AD0188_CCP_MERGE
#if JVET_AC0054_GLCCCM
      pu.curCand.type = (pu.glCccmFlag ? CCP_TYPE_GLCCCM : CCP_TYPE_CCCM);
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;
#else
      pu.curCand.type = CCP_TYPE_CCCM;
#endif
#endif
      if (cccmSADtmp < cccmSAD)
      {
        xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[2], 0, 0, predCb);
        xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[2], 0, 0, predCr);
#if JVET_AD0188_CCP_MERGE
        PU::cccmModelToCcpParams(pu.curCand, &cccmModelCb[2], &cccmModelCr[2], 0
#if JVET_AB0174_CCCM_DIV_FREE
                                 , m_cccmLumaOffset
#endif
        );
#endif
      }
      else
      {
        xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[0], 0, 0, predCb);
        xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[0], 0, 0, predCr);
#if JVET_AD0188_CCP_MERGE
        PU::cccmModelToCcpParams(pu.curCand, &cccmModelCb[0], &cccmModelCr[0], 0
#if JVET_AB0174_CCCM_DIV_FREE
                                 , m_cccmLumaOffset
#endif
        );
#endif
      }
    }
    else
    {
      // Multimode case
      int modelThr = 0, modelThrTmp = 0;
      int cccmSAD = MAX_INT, cccmSADtmp = MAX_INT;

      modelThr = xCccmCalcRefAver(pu);
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 1, modelThr);
      xCccmCalcModels(pu, cccmModelCb[1], cccmModelCr[1], 2, modelThr);
      cccmSAD = xCalculateCCCMcost(pu, COMPONENT_Cb, pu.intraDir[1], pu.blocks[COMPONENT_Cb], &cccmModelCb[0], modelThr);
      cccmSAD += xCalculateCCCMcost(pu, COMPONENT_Cr, pu.intraDir[1], pu.blocks[COMPONENT_Cr], &cccmModelCr[0], modelThr);

      modelThrTmp = xCccmCalcRefAver(pu, 2);
      xCccmCalcModels(pu, cccmModelCb[2], cccmModelCr[2], 1, modelThrTmp);
      xCccmCalcModels(pu, cccmModelCb[3], cccmModelCr[3], 2, modelThrTmp);
      cccmSADtmp = xCalculateCCCMcost(pu, COMPONENT_Cb, pu.intraDir[1], pu.blocks[COMPONENT_Cb], &cccmModelCb[2], modelThrTmp);
      cccmSADtmp += xCalculateCCCMcost(pu, COMPONENT_Cr, pu.intraDir[1], pu.blocks[COMPONENT_Cr], &cccmModelCr[2], modelThrTmp);
#if JVET_AD0188_CCP_MERGE
#if JVET_AC0054_GLCCCM
      pu.curCand.type = ((pu.glCccmFlag ? CCP_TYPE_GLCCCM : CCP_TYPE_CCCM) | CCP_TYPE_MMLM);
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;
#else
      pu.curCand.type = (CCP_TYPE_CCCM | CCP_TYPE_MMLM);
#endif
#endif
      if (cccmSADtmp < cccmSAD)
      {
        xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[2], 1, modelThrTmp, predCb);
        xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[3], 2, modelThrTmp, predCb);

        xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[2], 1, modelThrTmp, predCr);
        xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[3], 2, modelThrTmp, predCr);
#if JVET_AD0188_CCP_MERGE
        PU::cccmModelToCcpParams(pu.curCand, &cccmModelCb[2], &cccmModelCr[2], modelThrTmp
#if JVET_AB0174_CCCM_DIV_FREE
                                 , m_cccmLumaOffset
#endif
        );
#endif
      }
      else
      {
        xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[0], 1, modelThr, predCb);
        xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb[1], 2, modelThr, predCb);

        xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[0], 1, modelThr, predCr);
        xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr[1], 2, modelThr, predCr);
#if JVET_AD0188_CCP_MERGE
        PU::cccmModelToCcpParams(pu.curCand, &cccmModelCb[0], &cccmModelCr[0], modelThr
#if JVET_AB0174_CCCM_DIV_FREE
                                 , m_cccmLumaOffset
#endif
        );
#endif
      }
    }
  }
  else
#endif
  if ( pu.cccmFlag )
  {
#if JVET_AD0188_CCP_MERGE
    CccmModel cccmModelCb[2] = { CccmModel( CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
    CccmModel cccmModelCr[2] = { CccmModel( CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)), CccmModel( CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA)) };
#else
    CccmModel cccmModelCb( CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );
    CccmModel cccmModelCr( CCCM_NUM_PARAMS, pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) );
#endif

#if JVET_AB0143_CCCM_TS
    if ( intraDir == LM_CHROMA_IDX || intraDir == MDLM_L_IDX || intraDir == MDLM_T_IDX )
#else
    if ( PU::cccmSingleModeAvail(pu, intraDir) )
#endif
    {
#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 0, 0, predCr);

#if JVET_AC0054_GLCCCM
      pu.curCand.type = (pu.glCccmFlag ? CCP_TYPE_GLCCCM : CCP_TYPE_CCCM);
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;
#else
      pu.curCand.type = CCP_TYPE_CCCM;
#endif
      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, 0
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels(pu, cccmModelCb,  cccmModelCr, 0, 0);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 0, 0, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 0, 0, predCr);
#endif
    }
    else
    {
      // Multimode case
      int modelThr = xCccmCalcRefAver(pu);
#if JVET_AD0188_CCP_MERGE
      xCccmCalcModels(pu, cccmModelCb[0], cccmModelCr[0], 1, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[0], 1, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[0], 1, modelThr, predCr);

      xCccmCalcModels(pu, cccmModelCb[1], cccmModelCr[1], 2, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb,   cccmModelCb[1], 2, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr,   cccmModelCr[1], 2, modelThr, predCr);

#if JVET_AC0054_GLCCCM
      pu.curCand.type = ((pu.glCccmFlag ? CCP_TYPE_GLCCCM : CCP_TYPE_CCCM) | CCP_TYPE_MMLM);
      pu.curCand.corOffX = m_cccmBlkArea.x - m_cccmRefArea.x;
      pu.curCand.corOffY = m_cccmBlkArea.y - m_cccmRefArea.y;
#else
      pu.curCand.type = (CCP_TYPE_CCCM | CCP_TYPE_MMLM);
#endif
      PU::cccmModelToCcpParams(pu.curCand, cccmModelCb, cccmModelCr, modelThr
#if JVET_AB0174_CCCM_DIV_FREE
                               , m_cccmLumaOffset
#endif
                               );
#else
      xCccmCalcModels(pu, cccmModelCb,  cccmModelCr, 1, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 1, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 1, modelThr, predCr);

      xCccmCalcModels(pu, cccmModelCb,  cccmModelCr, 2, modelThr);
      xCccmApplyModel(pu, COMPONENT_Cb, cccmModelCb, 2, modelThr, predCb);
      xCccmApplyModel(pu, COMPONENT_Cr, cccmModelCr, 2, modelThr, predCr);
#endif
    }
  }
#if JVET_AD0120_LBCCP
  if (pu.ccInsideFilter)
  {
    filterPredInside(COMPONENT_Cb, predCb, pu);
    filterPredInside(COMPONENT_Cr, predCr, pu);
  }
#endif
}

void IntraPrediction::xCccmApplyModel(const PredictionUnit& pu, const ComponentID compId, CccmModel& cccmModel, int modelId, int modelThr, PelBuf &piPred)
{
  const  ClpRng& clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;

  int stepX = 1;
  int stepY = 1;
  int chromaScaleX = 0;
  int chromaScaleY = 0;

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if( pu.cccmNoSubFlag )
  {
    chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
    chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );

    stepX = 1 << chromaScaleX;
    stepY = 1 << chromaScaleY;
  }
#endif

#if JVET_AC0054_GLCCCM
  int refSizeX   = m_cccmBlkArea.x - m_cccmRefArea.x; // Reference lines available left and above
  int refSizeY   = m_cccmBlkArea.y - m_cccmRefArea.y;
#endif

  CPelBuf refLumaBlk;
#if JVET_AD0202_CCCM_MDF
  CPelBuf refLumaBlk1, refLumaBlk2, refLumaBlk3;

  if( pu.cccmMultiFilterIdx == 1 )
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu, 0, 3, &refLumaBlk1, &refLumaBlk3, &refLumaBlk2 );
  }
  else if( pu.cccmMultiFilterIdx > 1 )
  {
    refLumaBlk = xCccmGetLumaPuBuf( pu, 0, 2, &refLumaBlk1, &refLumaBlk3 );
  }
  else
#endif  
  refLumaBlk = xCccmGetLumaPuBuf( pu );

  for (int y = 0; y < refLumaBlk.height; y += stepY )
  {
    for (int x = 0; x < refLumaBlk.width; x += stepX )
    {
      const Pel* src0 = refLumaBlk.bufAt( x, y );

      if ( modelId == 1 && src0[0] > modelThr ) // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if ( modelId == 2 && src0[0] <= modelThr) // Model 2: Include only samples above the threshold
      {
        continue;
      }

      const Pel* src1 = refLumaBlk.bufAt( x, y + 1 );
      const Pel* src2 = refLumaBlk.bufAt( x, y - 1 );
      
      // 7-tap cross
#if JVET_AC0054_GLCCCM
      if( pu.glCccmFlag )      
      {
        samples[0] = src0[0]; // C
        samples[1] = ( 2 * src2[0] + src2[-1] + src2[1] ) - ( 2 * src1[0] + src1[-1] + src1[1] ); // Vertical gradient
        samples[2] = ( 2 * src0[-1] + src2[-1] + src1[-1] ) - ( 2 * src0[1] + src2[1] + src1[1] ); // Horizontal gradient
        samples[3] = (y + refSizeY + CCCM_LOC_OFFSET) << CCCM_LOC_SHIFT; // Y coordinate
        samples[4] = (x + refSizeX + CCCM_LOC_OFFSET) << CCCM_LOC_SHIFT; // X coordinate
        samples[5] = cccmModel.nonlinear( src0[0] );
        samples[6] = cccmModel.bias();
      }
      else
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
      if( pu.cccmNoSubFlag )
      {
        samples[0] = src0[0];
        samples[1] = src0[-1];
        samples[2] = src0[1];
        samples[3] = src1[0];
        samples[4] = src1[-1];
        samples[5] = src1[1];
        samples[6] = cccmModel.nonlinear( src0[0] );
        samples[7] = cccmModel.nonlinear( src1[0] );
        samples[8] = cccmModel.nonlinear( src0[1] );
        samples[9] = cccmModel.nonlinear( src0[-1] );
        samples[10] = cccmModel.bias();
      }
      else
#endif
#if JVET_AD0202_CCCM_MDF
      if( pu.cccmMultiFilterIdx == 1 )
      {
        // 7-tap cross
        samples[0] = src0[0]; // C
        samples[1] = refLumaBlk1.at( x, y ); // W
        samples[2] = refLumaBlk2.at( x, y ); // E
        samples[3] = refLumaBlk3.at( x, y );
        samples[4] = cccmModel.nonlinear( src0[0] );
        samples[5] = cccmModel.nonlinear( refLumaBlk1.at( x, y ) );
        samples[6] = cccmModel.nonlinear( refLumaBlk2.at( x, y ) );
        samples[7] = ( ( y + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT ); // Y coordinate
        samples[8] = ( ( x + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT ); // X coordinate
        samples[9] = cccmModel.bias();
      }
      else if( pu.cccmMultiFilterIdx == 2 )
      {
        samples[0] = src0[0]; // C
        samples[1] = src0[-1]; // W
        samples[2] = src0[1]; // E
        samples[3] = refLumaBlk1.at( x, y ); // C
        samples[4] = refLumaBlk1.at( x - 1, y ); // W
        samples[5] = refLumaBlk1.at( x + 1, y ); // E
        samples[6] = cccmModel.nonlinear( src0[0] );
        samples[7] = cccmModel.nonlinear( src0[-1] );
        samples[8] = cccmModel.nonlinear( src0[1] );
        samples[9] = ( ( x + refSizeX + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT ); // X coordinate
        samples[10] = cccmModel.bias();
      }
      else if( pu.cccmMultiFilterIdx == 3 )
      {
        samples[0] = src0[0]; // C
        samples[1] = src2[1]; // EN
        samples[2] = src1[-1]; // WS
        samples[3] = refLumaBlk3.at( x, y ); // C
        samples[4] = refLumaBlk3.at( x + 1, y - 1 ); // EN
        samples[5] = refLumaBlk3.at( x - 1, y + 1 ); // WS
        samples[6] = cccmModel.nonlinear( src0[0] );
        samples[7] = cccmModel.nonlinear( src2[1] );
        samples[8] = cccmModel.nonlinear( src1[-1] );
        samples[9] = ( ( y + refSizeY + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT ); // Y coordinate
        samples[10] = cccmModel.bias();
      }
      else
#endif
      {
        samples[0] = src0[0]; // C
        samples[1] = src2[0]; // N
        samples[2] = src1[0]; // S
        samples[3] = src0[-1]; // W
        samples[4] = src0[1]; // E
        samples[5] = cccmModel.nonlinear( refLumaBlk.at( x, y ) );
        samples[6] = cccmModel.bias();
      }

      piPred.at( x >> chromaScaleX, y >> chromaScaleY ) = ClipPel<Pel>( cccmModel.convolve( samples ), clpRng );
    }
  }
}

void IntraPrediction::xCccmCalcModels(const PredictionUnit& pu, CccmModel& cccmModelCb, CccmModel& cccmModelCr, int modelId, int modelThr
#if JVET_AD0120_LBCCP
                                      , int trainingRange
#endif
#if JVET_AF0073_INTER_CCP_MERGE
                                      , bool useRefSampOnly
#endif
)
{
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  const CPelBuf recoCb  = pu.cs->picture->getRecoBuf(COMPONENT_Cb);
  const CPelBuf recoCr  = pu.cs->picture->getRecoBuf(COMPONENT_Cr);
  PelBuf        refLuma;

#if JVET_AD0202_CCCM_MDF
  PelBuf        refLuma1, refLuma2, refLuma3;

  if( pu.cccmMultiFilterIdx == 1 )
  {
    refLuma = xCccmGetLumaRefBuf( pu, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, 0, 3, &refLuma1, &refLuma3, &refLuma2 );
  }
  else if( pu.cccmMultiFilterIdx > 1 )
  {
    refLuma = xCccmGetLumaRefBuf( pu, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, 0, 2, &refLuma1, &refLuma3 );
  }
  else
#endif
  refLuma = xCccmGetLumaRefBuf( pu, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY );
  
  int sampleNum = 0;
  int stepX = 1;
  int stepY = 1;
  int chromaScaleX = 0;
  int chromaScaleY = 0;

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if( pu.cccmNoSubFlag )
  {
    chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
    chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );

    stepX = 1 << chromaScaleX;
    stepY = 1 << chromaScaleY;
  }
#endif
  
#if JVET_AB0174_CCCM_DIV_FREE
  int chromaOffsetCb = 1 << ( pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1 );
  int chromaOffsetCr = 1 << ( pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1 );
  
  if ( refSizeX || refSizeY )
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;

    refPosX = ( refPosX + refPosPicX ) >> chromaScaleX;
    refPosY = ( refPosY + refPosPicY ) >> chromaScaleY;

    chromaOffsetCb = recoCb.at( refPosX, refPosY );
    chromaOffsetCr = recoCr.at( refPosX, refPosY );
  }
#endif

#if JVET_AB0143_CCCM_TS
  int yStart = pu.cccmFlag == 2 ? refSizeY : 0;
  int yEnd = pu.cccmFlag == 3 ? refSizeY : areaHeight;
  int xStart = pu.cccmFlag == 3 ? refSizeX : 0;
  int xEnd = pu.cccmFlag == 2 ? refSizeX : areaWidth;
#if JVET_AD0120_LBCCP
  if( trainingRange != -1 )
  {
    if( pu.cccmFlag != 2 && ( refSizeY > trainingRange ) )
    {
      yStart = refSizeY - trainingRange;
    }
    if( pu.cccmFlag != 3 && ( refSizeX > trainingRange ) )
    {
      xStart = refSizeX - trainingRange;
    }
  }
#endif

  for (int y = yStart; y < yEnd; y += stepY )
  {
    for (int x = xStart; x < xEnd; x += stepX )
    {
#else
  for (int y = 0; y < areaHeight; y += stepY )
  {
    for (int x = 0; x < areaWidth; x += stepX )
    {
#endif
      if ( x >= refSizeX && y >= refSizeY )
      {
        continue;
      }
      
      const Pel* src0 = refLuma.bufAt( x, y );

      if( modelId == 1 && src0[0] > modelThr )   // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if( modelId == 2 && src0[0] <= modelThr )   // Model 2: Include only samples above the threshold
      {
        continue;
      }

      const Pel* src1 = refLuma.bufAt( x, y + 1 );
      const Pel* src2 = refLuma.bufAt( x, y - 1 );
#if JVET_AF0073_INTER_CCP_MERGE
      bool isBorderLeft = false, isBorderTop = false;
      if (useRefSampOnly)
      {
        if (y >= refSizeY && (x ==  pu.chromaPos().x - refPosPicX - 1))
        {
          isBorderLeft = true;
        }
        if(x >= refSizeX && (y ==  pu.chromaPos().y - refPosPicY - 1))
        {
          isBorderTop = true;
        }
      }
#endif

      // 7-tap cross
#if JVET_AC0054_GLCCCM
      if (pu.glCccmFlag)
      {        
        m_a[0][sampleNum] = src0[0]; // C
        m_a[1][sampleNum] = ( 2 * src2[0] + src2[-1] + src2[1] ) - ( 2 * src1[0] + src1[-1] + src1[1] ); // Vertical gradient
        m_a[2][sampleNum] = ( 2 * src0[-1] + src2[-1] + src1[-1] ) - ( 2 * src0[1] + src2[1] + src1[1] ); // Horizontal gradient
        m_a[3][sampleNum] = ( y + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        m_a[4][sampleNum] = ( x + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        m_a[5][sampleNum] = cccmModelCb.nonlinear( src0[0] );
        m_a[6][sampleNum] = cccmModelCb.bias();
      }
      else
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
      if( pu.cccmNoSubFlag )
      {
        m_a[0][sampleNum] = src0[0];
        m_a[1][sampleNum] = src0[-1];
        m_a[2][sampleNum] = src0[1];
        m_a[3][sampleNum] = src1[0];
        m_a[4][sampleNum] = src1[-1];
        m_a[5][sampleNum] = src1[1];
        m_a[6][sampleNum] = cccmModelCb.nonlinear( src0[0] );
        m_a[7][sampleNum] = cccmModelCb.nonlinear( src1[0] );
        m_a[8][sampleNum] = cccmModelCb.nonlinear( src0[1] );
        m_a[9][sampleNum] = cccmModelCb.nonlinear( src0[-1] );
        m_a[10][sampleNum] = cccmModelCb.bias();
      }
      else
#endif
#if JVET_AD0202_CCCM_MDF
      if( pu.cccmMultiFilterIdx == 1 )
      {
        // 7-tap cross
        m_a[0][sampleNum] = src0[0];
        m_a[1][sampleNum] = refLuma1.at( x, y );
        m_a[2][sampleNum] = refLuma2.at( x, y );
        m_a[3][sampleNum] = refLuma3.at( x, y );
        m_a[4][sampleNum] = cccmModelCb.nonlinear( src0[0] );
        m_a[5][sampleNum] = cccmModelCb.nonlinear( refLuma1.at( x, y ) );
        m_a[6][sampleNum] = cccmModelCb.nonlinear( refLuma2.at( x, y ) );
        m_a[7][sampleNum] = ( y + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        m_a[8][sampleNum] = ( x + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        m_a[9][sampleNum] = cccmModelCb.bias();
      }
      else if( pu.cccmMultiFilterIdx == 2 )
      {
        m_a[0][sampleNum] = src0[0]; // C
        m_a[1][sampleNum] = src0[-1]; // W
        m_a[2][sampleNum] = src0[1]; // E
        m_a[3][sampleNum] = refLuma1.at( x, y ); // C
        m_a[4][sampleNum] = refLuma1.at( x - 1, y ); // W
        m_a[5][sampleNum] = refLuma1.at( x + 1, y ); // E
        m_a[6][sampleNum] = cccmModelCb.nonlinear( src0[0] );
        m_a[7][sampleNum] = cccmModelCb.nonlinear( src0[-1] );
        m_a[8][sampleNum] = cccmModelCb.nonlinear( src0[1] );
        m_a[9][sampleNum] = ( x + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // X coordinate
        m_a[10][sampleNum] = cccmModelCb.bias();
      }
      else if( pu.cccmMultiFilterIdx == 3 )
      {
        m_a[0][sampleNum] = src0[0]; // C
        m_a[1][sampleNum] = src2[1]; // EN
        m_a[2][sampleNum] = src1[-1]; // WS
        m_a[3][sampleNum] = refLuma3.at( x, y ); // C
        m_a[4][sampleNum] = refLuma3.at( x + 1, y - 1 ); // EN
        m_a[5][sampleNum] = refLuma3.at( x - 1, y + 1 ); // WS
        m_a[6][sampleNum] = cccmModelCb.nonlinear( src0[0] );
        m_a[7][sampleNum] = cccmModelCb.nonlinear( src2[1] );
        m_a[8][sampleNum] = cccmModelCb.nonlinear( src1[-1] );
        m_a[9][sampleNum] = ( y + CCCM_LOC_OFFSET ) << CCCM_LOC_SHIFT; // Y coordinate
        m_a[10][sampleNum] = cccmModelCb.bias();
      }
      else
#endif
      {
        m_a[0][sampleNum] = src0[0]; // C
        m_a[1][sampleNum] = src2[0]; // N
#if JVET_AF0073_INTER_CCP_MERGE
        m_a[2][sampleNum] = isBorderTop ? src0[0] : src1[0]; // S
#else
        m_a[2][sampleNum] = src1[0]; // S
#endif
        m_a[3][sampleNum] = src0[-1]; // W
#if JVET_AF0073_INTER_CCP_MERGE
        m_a[4][sampleNum] = isBorderLeft ? src0[0] : src0[1]; // E
#else
        m_a[4][sampleNum] = src0[1]; // E
#endif
        m_a[5][sampleNum] = cccmModelCb.nonlinear( src0[0] );
        m_a[6][sampleNum] = cccmModelCb.bias();
      }
      
      const int refPosX = ( refPosPicX + x ) >> chromaScaleX;
      const int refPosY = ( refPosPicY + y ) >> chromaScaleY;

      m_cb[sampleNum] = recoCb.at( refPosX, refPosY );
      m_cr[sampleNum++] = recoCr.at( refPosX, refPosY );
    }
  }

  if( !sampleNum ) // Number of samples can go to zero in the multimode case
  {
    cccmModelCb.clearModel();
    cccmModelCr.clearModel();
  }
  else
  {
#if JVET_AB0174_CCCM_DIV_FREE
    m_cccmSolver.solve2( m_a, m_cb, m_cr, sampleNum, chromaOffsetCb, chromaOffsetCr, cccmModelCb, cccmModelCr );
#else
    m_cccmSolver.solve2( m_a, m_cb, m_cr, sampleNum, cccmModelCb, cccmModelCr );
#endif
  }
}

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
void IntraPrediction::xCccmCreateLumaNoSubRef( const PredictionUnit& pu, CompArea chromaArea 
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate
#endif
)
{
  const CPelBuf recoLuma = pu.cs->picture->getRecoBuf( COMPONENT_Y );
  const int  maxPosPicX = pu.cs->picture->lumaSize().width - 1;
  const int  maxPosPicY = pu.cs->picture->lumaSize().height - 1;

  xCccmCalcRefArea( pu, chromaArea ); // Find the reference area

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  CHECK( !pu.cccmNoSubFlag, "cccmNoSubFlag shall be enabled" );

  PelBuf refLuma = xCccmGetLumaRefBuf( pu, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY );

  const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
  const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );

  int puBorderX = refSizeX + (m_cccmBlkArea.width << chromaScaleX);
  int puBorderY = refSizeY + (m_cccmBlkArea.height << chromaScaleY);

#if JVET_AB0174_CCCM_DIV_FREE
  xCccmSetLumaRefValue( pu );
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  if (!isTemplate)
  {
#endif
  const int filterPaddingX = CCCM_FILTER_PADDING << chromaScaleX;
  const int filterPaddingY = CCCM_FILTER_PADDING << chromaScaleY;

  // luma for the area covering both the PU and the top/left reference areas (+ top and left paddings)
  for( int y = -filterPaddingY; y < areaHeight; y++ )
  {
    for( int x = -filterPaddingX; x < areaWidth; x++ )
    {
      if( (x >= puBorderX && y >= refSizeY) || (y >= puBorderY && x >= refSizeX) )
      {
        continue;
      }

      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at( x, y ) = xCccmGetLumaVal( pu, recoLuma, chromaPosPicX, chromaPosPicY );
    }
  }

  // Pad right of top reference area
  for( int x = 0; x < filterPaddingX; x++ )
  {
    for( int y = -filterPaddingY; y < refSizeY; y++ )
    {
      refLuma.at( areaWidth + x, y ) = refLuma.at( areaWidth - 1, y );
    }

    // Pad right of PU
    for( int y = refSizeY; y < puBorderY; y++ )
    {
      refLuma.at( puBorderX + x, y ) = refLuma.at( puBorderX - 1, y );
    }

    // Pad right of left reference area
    for( int y = puBorderY; y < areaHeight; y++ )
    {
      refLuma.at( refSizeX + x, y ) = refLuma.at( refSizeX - 1, y );
    }
  }

  for( int y = 0; y < filterPaddingY; y++ )
  {
    // Pad below left reference area
    for( int x = -filterPaddingX; x < refSizeX + filterPaddingX; x++ )
    {
      refLuma.at( x, areaHeight + y ) = refLuma.at( x, areaHeight - 1 );
    }

    // Pad below PU
    for( int x = refSizeX; x < puBorderX + filterPaddingX; x++ )
    {
      refLuma.at( x, puBorderY + y ) = refLuma.at( x, puBorderY - 1 );
    }

    // Pad below right reference area
    for( int x = puBorderX + filterPaddingX; x < areaWidth + filterPaddingX; x++ )
    {
      refLuma.at( x, refSizeY + y ) = refLuma.at( x, refSizeY - 1 );
    }
  }

  // In dualtree we can also use luma from the right and below (if not on CTU/picture boundary)
  if( CS::isDualITree( *pu.cs ) )
  {
    int ctuWidth = pu.cs->sps->getMaxCUWidth() >> getComponentScaleX( COMPONENT_Cb, pu.chromaFormat );
    int ctuHeight = pu.cs->sps->getMaxCUHeight() >> getComponentScaleY( COMPONENT_Cb, pu.chromaFormat );

    // Samples right of top reference area
    int padPosPicX = refPosPicX + areaWidth;

    if( padPosPicX <= maxPosPicX && (padPosPicX % ctuWidth) )
    {
      for( int y = -filterPaddingY; y < refSizeY; y++ )
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        for( int x = 0; x < filterPaddingX; x++ )
        {
          refLuma.at( areaWidth + x, y ) = xCccmGetLumaVal( pu, recoLuma, padPosPicX, chromaPosPicY );
        }
      }
    }

    // Samples right of PU
    padPosPicX = refPosPicX + puBorderX;

    if( padPosPicX <= maxPosPicX && (padPosPicX % ctuWidth) )
    {
      for( int y = refSizeY; y < puBorderY; y++ )
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        for( int x = 0; x < filterPaddingX; x++ )
        {
          refLuma.at( puBorderX + x, y ) = xCccmGetLumaVal( pu, recoLuma, padPosPicX, chromaPosPicY );
        }
      }
    }

    // Samples right of left reference area
    padPosPicX = refPosPicX + refSizeX;

    if( padPosPicX <= maxPosPicX )
    {
      for( int y = puBorderY; y < areaHeight; y++ )
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        for( int x = 0; x < filterPaddingX; x++ )
        {
          refLuma.at( refSizeX + x, y ) = xCccmGetLumaVal( pu, recoLuma, padPosPicX, chromaPosPicY );
        }
      }
    }

    // Samples below left reference area
    int padPosPicY = refPosPicY + areaHeight;

    if( padPosPicY <= maxPosPicY && (padPosPicY % ctuHeight) )
    {
      for( int x = -filterPaddingX; x < refSizeX + filterPaddingX; x++ )
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;

        for( int y = 0; y < filterPaddingY; y++ )
        {
          refLuma.at( x, areaHeight + y ) = xCccmGetLumaVal( pu, recoLuma, chromaPosPicX, padPosPicY );
        }
      }
    }

    // Samples below PU
    padPosPicY = refPosPicY + puBorderY;

    if( padPosPicY <= maxPosPicY && (padPosPicY % ctuHeight) )
    {
      for( int x = refSizeX; x < puBorderX; x++ ) // Just go to PU border as the next sample may be out of CTU (and not needed anyways)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;

        for( int y = 0; y < filterPaddingY; y++ )
        {
          refLuma.at( x, puBorderY + y ) = xCccmGetLumaVal( pu, recoLuma, chromaPosPicX, padPosPicY );
        }
      }
    }

    // Samples below right reference area
    padPosPicY = refPosPicY + refSizeY;

    if( padPosPicY <= maxPosPicY )
    {
      // Avoid going outside of right CTU border where these samples are not yet available
      int puPosPicX = pu.blocks[COMPONENT_Cb].x;
      int ctuRightEdgeDist = ctuWidth - (puPosPicX % ctuWidth) + refSizeX;
      int lastPosX = ctuRightEdgeDist < areaWidth ? ctuRightEdgeDist : areaWidth;

      for( int x = puBorderX + 1; x < lastPosX; x++ ) // Just go to ref area border as the next sample may be out of CTU (and not needed anyways)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;

        for( int y = 0; y < filterPaddingY; y++ )
        {
          refLuma.at( x, refSizeY + y ) = xCccmGetLumaVal( pu, recoLuma, chromaPosPicX, padPosPicY );
        }
      }
    }
  }
#if JVET_AF0073_INTER_CCP_MERGE
  }
  else
  {
    const int stepX        = 1 << chromaScaleX;
    const int stepY        = 1 << chromaScaleY;
    
    // Generate top template
    if (refSizeY > 0)
    {
      for (int y = refSizeY - stepY; y < refSizeY; y++)
      {
        for ( int x = refSizeX - 1; x < puBorderX; x++ )
        {
          int chromaPosPicX = refPosPicX + x;
          int chromaPosPicY = refPosPicY + y;

          chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
          chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

          refLuma.at( x, y ) = xCccmGetLumaVal( pu, recoLuma, chromaPosPicX, chromaPosPicY );
        }
      }
    }

    // Generate left template
    if (refSizeX > 0)
    {
      for (int x = refSizeX - stepX - 1; x < refSizeX; x++)
      {
        for( int y = refSizeY; y <= puBorderY; y++ )
        {
          int chromaPosPicX = refPosPicX + x;
          int chromaPosPicY = refPosPicY + y;

          chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
          chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

          refLuma.at( x, y ) = xCccmGetLumaVal( pu, recoLuma, chromaPosPicX, chromaPosPicY );
        }
      }
    }
  }
#endif  
}
#endif
#if JVET_AE0100_BVGCCCM
void IntraPrediction::xBvgCccmCalcModels( const PredictionUnit& pu, CccmModel& cccmModelCb, CccmModel &cccmModelCr, int modelId, int modelThr, int minVal, int maxVal )
{
  const CPelBuf recoCb = pu.cs->picture->getRecoBuf( COMPONENT_Cb );
  const CPelBuf recoCr = pu.cs->picture->getRecoBuf( COMPONENT_Cr );
  int chromaOffsetCb = 1 << ( pu.cu->slice->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ) - 1 );
  int chromaOffsetCr = 1 << ( pu.cu->slice->getSPS()->getBitDepth( CHANNEL_TYPE_CHROMA ) - 1 );
  
  int refSizeX   = m_cccmBlkArea.x - m_cccmRefArea.x; // Reference lines available left and above
  int refSizeY   = m_cccmBlkArea.y - m_cccmRefArea.y;
  int refPosPicX = m_cccmRefArea.x;                   // Position of the reference area in picture coordinates
  int refPosPicY = m_cccmRefArea.y;
#if JVET_AB0174_CCCM_DIV_FREE
  if ( refSizeX || refSizeY )
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;
    
    chromaOffsetCb = recoCb.at(refPosPicX + refPosX, refPosPicY + refPosY);
    chromaOffsetCr = recoCr.at(refPosPicX + refPosX, refPosPicY + refPosY);
  }
#endif
  int sampleNum = 0;
  int strX[NUM_BVG_CCCM_CANDS], endX[NUM_BVG_CCCM_CANDS], strY[NUM_BVG_CCCM_CANDS], endY[NUM_BVG_CCCM_CANDS];
  for (int candIdx = 0; candIdx < pu.numBvgCands; candIdx++)
  {
    Mv chromaBv = pu.bvList[candIdx];
    PelBuf refLuma = xBvgCccmGetLumaPuBuf(pu, candIdx);
    int refPosPicX = pu.blocks[COMPONENT_Cb].x + chromaBv.hor;
    int refPosPicY = pu.blocks[COMPONENT_Cb].y + chromaBv.ver;
    strX[candIdx] = refPosPicX;
    strY[candIdx] = refPosPicY;
    endX[candIdx] = refPosPicX + refLuma.width;
    endY[candIdx] = refPosPicY + refLuma.height;
  }
  
  for (int candIdx = 0; candIdx < pu.numBvgCands; candIdx++)
  {
    PelBuf refLuma = xBvgCccmGetLumaPuBuf(pu, candIdx);
    PelBuf refCb = xBvgCccmGetChromaPuBuf(pu, COMPONENT_Cb, candIdx);
    PelBuf refCr = xBvgCccmGetChromaPuBuf(pu, COMPONENT_Cr, candIdx);
      //-- collect data
    for( int y = 0; y < refLuma.height; y++ )
    {
      for( int x = 0; x < refLuma.width; x++ )
      {
        if ( modelId == 1 && refLuma.at(x, y) > modelThr ) // Model 1: Include only samples below or equal to the threshold
        {
          continue;
        }
        if ( modelId == 2 && refLuma.at(x, y) <= modelThr) // Model 2: Include only samples above the threshold
        {
          continue;
        }
        if (refLuma.at(x, y) < minVal || refLuma.at(x, y) > maxVal)
        {
          continue;
        }
        bool exist = false;
        if (candIdx > 0)
        {
          for (int i = 0; i < candIdx; i++)
          {
            int xx = strX[candIdx] + x;
            int yy = strY[candIdx] + y;
            if (xx >= strX[i] && xx < endX[i] && yy >= strY[i] && yy < endY[i])
            {
              exist = true;
              break;
            }
          }
        }
        if (exist)
        {
          continue;
        }
        // 11-tap filter
        m_a[0][sampleNum] = refLuma.at(x, y    ); // C
        m_a[1][sampleNum] = refLuma.at(x, y - 1); // N
        m_a[2][sampleNum] = refLuma.at(x, y + 1); // S
        m_a[3][sampleNum] = refLuma.at(x - 1, y); // W
        m_a[4][sampleNum] = refLuma.at(x + 1, y); // E
        m_a[5][sampleNum] = cccmModelCb.nonlinear( refLuma.at(x, y) );     // nonlinear(C)
        m_a[6][sampleNum] = cccmModelCb.nonlinear( refLuma.at(x, y - 1) ); // nonlinear(N)
        m_a[7][sampleNum] = cccmModelCb.nonlinear( refLuma.at(x, y + 1) ); // nonlinear(S)
        m_a[8][sampleNum] = cccmModelCb.nonlinear( refLuma.at(x - 1, y) ); // nonlinear(W)
        m_a[9][sampleNum] = cccmModelCb.nonlinear( refLuma.at(x + 1, y) ); // nonlinear(E)
        m_a[10][sampleNum] = cccmModelCb.bias();
        
        m_cb[sampleNum] = refCb.at( x, y );
        m_cr[sampleNum++] = refCr.at( x, y );
      }
    }
  }
  
  if( !sampleNum ) // Number of samples can go to zero in the multimode case
  {
    cccmModelCb.clearModel();
    cccmModelCr.clearModel();
  }
  else
  {
#if JVET_AB0174_CCCM_DIV_FREE
    m_cccmSolver.solve2( m_a, m_cb, m_cr, sampleNum, chromaOffsetCb, chromaOffsetCr, cccmModelCb, cccmModelCr );
#else
    m_cccmSolver.solve2( m_a, m_cb, m_cr, sampleNum, cccmModelCb, cccmModelCr );
#endif
  }
  return;
}
void IntraPrediction::xBvgCccmApplyModel( const PredictionUnit& pu, const ComponentID compId, CccmModel &cccmModel, int modelId, int modelThr, PelBuf &piPred )
{
  const  ClpRng& clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;
  
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  CHECK( pu.cccmNoSubFlag, "cccmNoSubFlag shall be disabled" );
#endif
  CPelBuf refLumaBlk = xCccmGetLumaPuBuf(pu);
  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      if ( modelId == 1 && refLumaBlk.at( x, y ) > modelThr ) // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if ( modelId == 2 && refLumaBlk.at( x, y ) <= modelThr) // Model 2: Include only samples above the threshold
      {
        continue;
      }
      // 11-tap filter
      samples[0] = refLumaBlk.at(x, y    ); // C
      samples[1] = refLumaBlk.at(x, y - 1); // N
      samples[2] = refLumaBlk.at(x, y + 1); // S
      samples[3] = refLumaBlk.at(x - 1, y); // W
      samples[4] = refLumaBlk.at(x + 1, y); // E
      samples[5] = cccmModel.nonlinear( refLumaBlk.at(x, y) );     // nonlinear(C)
      samples[6] = cccmModel.nonlinear( refLumaBlk.at(x, y - 1) ); // nonlinear(N)
      samples[7] = cccmModel.nonlinear( refLumaBlk.at(x, y + 1) ); // nonlinear(S)
      samples[8] = cccmModel.nonlinear( refLumaBlk.at(x - 1, y) ); // nonlinear(W)
      samples[9] = cccmModel.nonlinear( refLumaBlk.at(x + 1, y) ); // nonlinear(E)
      samples[10] = cccmModel.bias();
      
      piPred.at(x, y) = ClipPel<Pel>( cccmModel.convolve(samples), clpRng );
    }
  }
}
int IntraPrediction::xBvgCccmCalcBlkAver(const PredictionUnit& pu) const
{
  int numSamples = 0;
  int sumSamples = 0;
  CPelBuf refLumaBlk = xCccmGetLumaPuBuf( pu );
  
  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      sumSamples += refLumaBlk.at(x, y);
      numSamples += 1;
    }
  }
#if JVET_AB0174_CCCM_DIV_FREE
  return numSamples == 0 ? 512 : xCccmDivideLowPrec(sumSamples, numSamples);
#else
  return numSamples == 0 ? 512 : ( sumSamples + numSamples/2) / numSamples;
#endif
}
void IntraPrediction::xBvgCccmCalcBlkRange(const PredictionUnit& pu, int& minVal, int&maxVal) const
{
  minVal = MAX_INT, maxVal = -MAX_INT;
  CPelBuf refLumaBlk = xCccmGetLumaPuBuf(pu);
  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      if (refLumaBlk.at(x, y) < minVal)
      {
        minVal = refLumaBlk.at(x, y);
      }
      if (refLumaBlk.at(x, y) > maxVal)
      {
        maxVal = refLumaBlk.at(x, y);
      }
    }
  }
  int range = abs(maxVal - minVal) >> 4;
  minVal -= range;
  maxVal += range;
}
void IntraPrediction::xBvgCccmCalcRefArea(const PredictionUnit& pu, CompArea chromaArea)
{
  m_bvgCccmBlkArea = chromaArea;
  m_bvgCccmRefArea = chromaArea;
  int refWidth = chromaArea.width + 2 * CCCM_FILTER_PADDING;
  int refHeight = chromaArea.height + 2 * CCCM_FILTER_PADDING;
  m_bvgCccmRefArea = Area(chromaArea.x - CCCM_FILTER_PADDING, chromaArea.y - CCCM_FILTER_PADDING, refWidth, refHeight);
}
PelBuf IntraPrediction::xBvgCccmGetLumaPuBufFul(const PredictionUnit& pu, int candIdx) const
{
  int refStride = m_bvgCccmRefArea.width;
  int tuWidth = m_bvgCccmRefArea.width;
  int tuHeight = m_bvgCccmRefArea.height;
  return PelBuf( m_bvgCccmLumaBuf[candIdx], refStride, tuWidth, tuHeight );  // Points to the top-left corner
                                                                             //  return PelBuf( m_bvgCccmLumaBuf[0], refStride, tuWidth, tuHeight );  // Points to the top-left corner
}
PelBuf IntraPrediction::xBvgCccmGetLumaPuBuf(const PredictionUnit& pu, int candIdx) const
{
  int refSizeX  = 0;//m_bvgCccmBlkArea.x; // Reference lines available left and above
  int refSizeY  = 0;//m_bvgCccmBlkArea.y;
  int tuWidth   = m_bvgCccmBlkArea.width;
  int tuHeight  = m_bvgCccmBlkArea.height;
  int refStride = m_bvgCccmBlkArea.width + 2 * CCCM_FILTER_PADDING; // Including paddings required for the 2D filter
  int refOrigin = refStride * (refSizeY + CCCM_FILTER_PADDING) + refSizeX + CCCM_FILTER_PADDING;
  
  return PelBuf( m_bvgCccmLumaBuf[candIdx] + refOrigin, refStride, tuWidth, tuHeight );  // Points to the top-left corner of the block
                                                                                         //  return PelBuf( m_bvgCccmLumaBuf[0] + refOrigin, refStride, tuWidth, tuHeight );  // Points to the top-left corner of the block
}
PelBuf IntraPrediction::xBvgCccmGetChromaPuBuf(const PredictionUnit& pu, const ComponentID compID, int candIdx) const
{
  int refSizeX  = 0;//m_bvgCccmBlkArea.x; // Reference lines available left and above
  int refSizeY  = 0;//m_bvgCccmBlkArea.y;
  int tuWidth   = m_bvgCccmBlkArea.width;
  int tuHeight  = m_bvgCccmBlkArea.height;
  int refStride = m_bvgCccmBlkArea.width + 2 * CCCM_FILTER_PADDING; // Including paddings required for the 2D filter
  int refOrigin = refStride * (refSizeY + CCCM_FILTER_PADDING) + refSizeX + CCCM_FILTER_PADDING;
  if (compID == COMPONENT_Cb)
  {
    return PelBuf( m_bvgCccmChromaBuf[candIdx][0] + refOrigin, refStride, tuWidth, tuHeight );  // Points to the top-left corner of the block
  }
  else
  {
    return PelBuf( m_bvgCccmChromaBuf[candIdx][1] + refOrigin, refStride, tuWidth, tuHeight );  // Points to the top-left corner of the block
  }
}
void IntraPrediction::xBvgCccmCreateLumaRef(const PredictionUnit& pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
                                            , int downsFilterIdx
#endif
                                            )
{
  const CPelBuf recoLuma = pu.cs->picture->getRecoBuf(COMPONENT_Y);
  const CPelBuf recoCb = pu.cs->picture->getRecoBuf( COMPONENT_Cb );
  const CPelBuf recoCr = pu.cs->picture->getRecoBuf( COMPONENT_Cr );
  
  const int  maxPosPicX  = pu.cs->picture->chromaSize().width  - 1;
  const int  maxPosPicY  = pu.cs->picture->chromaSize().height - 1;
  int ctuWidth  = pu.cs->sps->getMaxCUWidth()  >> getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
  int ctuHeight = pu.cs->sps->getMaxCUHeight() >> getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);
  
  for (int candIdx = 0; candIdx < pu.numBvgCands; candIdx++)
  {
    Mv chromaBv = pu.bvList[candIdx];
    xBvgCccmCalcRefArea(pu, chromaArea); // Find the reference area
    PelBuf refLuma = xBvgCccmGetLumaPuBuf(pu, candIdx);
    PelBuf refCb = xBvgCccmGetChromaPuBuf(pu, COMPONENT_Cb, candIdx);
    PelBuf refCr = xBvgCccmGetChromaPuBuf(pu, COMPONENT_Cr, candIdx);
    
#if JVET_AB0174_CCCM_DIV_FREE
    xCccmSetLumaRefValue( pu );
#endif
    
    int refPosPicX = pu.blocks[COMPONENT_Cb].x + chromaBv.hor;
    int refPosPicY = pu.blocks[COMPONENT_Cb].y + chromaBv.ver;
    int maxWidth = refLuma.width;
    int maxHeight =  refLuma.height;
    PU::checkIsChromaBvCandidateValid(pu, chromaBv, maxWidth, maxHeight);
    // Generate down-sampled luma for the area covering both the PU and the top/left reference areas (+ top and left paddings)
    for (int y = 0; y < refLuma.height; y++)
    {
      for (int x = 0; x < refLuma.width; x++)
      {
        int chromaPosPicX = refPosPicX + x;
        int chromaPosPicY = refPosPicY + y;
        
        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
        
        refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
                                             , downsFilterIdx
#endif
                                             );
        refCb.at( x, y ) = recoCb.at( chromaPosPicX, chromaPosPicY );
        refCr.at( x, y ) = recoCr.at( chromaPosPicX, chromaPosPicY );
        // pad if not available
        if ( x >= maxWidth )
        {
          refLuma.at( x, y ) = refLuma.at( x - 1, y );
          refCb.at( x, y ) = refCb.at( x - 1, y );
          refCr.at( x, y ) = refCr.at( x - 1, y );
        }
        else if (y >= maxHeight)
        {
          refLuma.at( x, y ) = refLuma.at( x, y - 1);
          refCb.at( x, y ) = refCb.at( x, y - 1);
          refCr.at( x, y ) = refCr.at( x, y - 1);
        }
      }
    }
    //-- Now fill the out of block samples (North)
    for (int x = 0; x < refLuma.width; x++)
    {
      int y = -1;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;
      
      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
      
      refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
                                           , downsFilterIdx
#endif
                                           );
      if (chromaPosPicY < 0)
      {
        refLuma.at( x, y ) = refLuma.at( x, y + 1 );
      }
      if ( x >= maxWidth )
      {
        refLuma.at( x, y ) = refLuma.at( x - 1, y );
      }
    }
    //-- Now fill the out of block samples (South)
    for (int x = 0; x < refLuma.width; x++)
    {
      int y = refLuma.height;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;
      
      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
      
      refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
                                           , downsFilterIdx
#endif
                                           );
      if (chromaPosPicY >= maxPosPicY)
      {
        refLuma.at( x, y ) = refLuma.at( x, y - 1 );
      }
      if (!( chromaPosPicY <= maxPosPicY && (chromaPosPicY % ctuHeight) ))
      {
        refLuma.at( x, y ) = refLuma.at( x, y - 1 );
      }
      if (maxHeight < refLuma.height)
      {
        refLuma.at( x, y ) = refLuma.at( x, y - 1);
      }
      if ( x >= maxWidth )
      {
        refLuma.at( x, y ) = refLuma.at( x - 1, y );
      }
    }
    //-- Now fill the out of block samples (West)
    for (int y = 0; y < refLuma.height; y++)
    {
      int x = -1;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;
      
      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
      
      refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
                                           , downsFilterIdx
#endif
                                           );
      if (chromaPosPicX < 0)
      {
        refLuma.at( x, y ) = refLuma.at( x + 1, y );
      }
      if ( y >= maxHeight )
      {
        refLuma.at( x, y ) = refLuma.at( x, y - 1 );
      }
    }
    //-- Now fill the out of block samples (East)
    for (int y = 0; y < refLuma.height; y++)
    {
      int x = refLuma.width;
      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;
      
      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
      
      refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
                                           , downsFilterIdx
#endif
                                           );
      if (chromaPosPicX >= maxPosPicX)
      {
        refLuma.at( x, y ) = refLuma.at( x - 1, y );
      }
      if (!( chromaPosPicX <= maxPosPicX && (chromaPosPicX % ctuWidth) ))
      {
        refLuma.at( x, y ) = refLuma.at( x - 1, y );
      }
      if (maxWidth < refLuma.width)
      {
        refLuma.at( x, y ) = refLuma.at( x - 1, y );
      }
      if ( y >= maxHeight )
      {
        refLuma.at( x, y ) = refLuma.at( x, y - 1 );
      }
    }
    
    int rribcFlipType = pu.rrIbcList[candIdx];
    if (rribcFlipType > 0)
    {
      PelBuf refLumaExt = xBvgCccmGetLumaPuBufFul(pu, candIdx);
      refLumaExt.flipSignal(rribcFlipType == 1);
      refCb.flipSignal(rribcFlipType == 1);
      refCr.flipSignal(rribcFlipType == 1);
    }
  }
}
#endif

// Using the same availability checking as in IntraPrediction::xFillReferenceSamples
void IntraPrediction::xCccmCalcRefArea(const PredictionUnit& pu, CompArea chromaArea)
{
  const ChannelType     chType  = CHANNEL_TYPE_CHROMA;
  const CodingUnit&     cu      = *pu.cu;
  const CodingStructure &cs     = *cu.cs;
  const SPS             &sps    = *cs.sps;
  const PreCalcValues   &pcv    = *cs.pcv;

  const int tuWidth      = chromaArea.width;
  const int tuHeight     = chromaArea.height;

  const bool noShift     = pcv.noChroma2x2 && chromaArea.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int  compScaleX  = getComponentScaleX(chromaArea.compID, sps.getChromaFormatIdc());
  const int  compScaleY  = getComponentScaleY(chromaArea.compID, sps.getChromaFormatIdc());
  const int  unitWidth   = pcv.minCUWidth  >> (noShift ? 0 : compScaleX);
  const int  unitHeight  = pcv.minCUHeight >> (noShift ? 0 : compScaleY);

  const int  totalAboveUnits    = (2 * tuWidth + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits     = (2 * tuHeight + (unitHeight - 1)) / unitHeight;
  const int  numAboveUnits      = std::max<int>( tuWidth / unitWidth, 1 );
  const int  numLeftUnits       = std::max<int>( tuHeight / unitHeight, 1 );
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits  = totalLeftUnits - numLeftUnits;

  static bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1] = { false }; // Just a dummy array here, content not used

  int avaiAboveRightUnits = isAboveRightAvailable( cu, chType, chromaArea.topRight(),   numAboveRightUnits, unitWidth,  (neighborFlags + totalLeftUnits + 1 + numAboveUnits) );
  int avaiLeftBelowUnits  = isBelowLeftAvailable ( cu, chType, chromaArea.bottomLeft(), numLeftBelowUnits,  unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits) );
  
  int refSizeX, refSizeY;
  
  PU::getCccmRefLineNum(pu, chromaArea, refSizeX, refSizeY); // Reference lines available left and above

  int refWidth  = chromaArea.width  + refSizeX;              // Reference buffer size excluding paddings
  int refHeight = chromaArea.height + refSizeY;

  int extWidth  = avaiAboveRightUnits * unitWidth;
  int extHeight = avaiLeftBelowUnits  * unitHeight;
  
  refWidth  += refSizeY ? extWidth  : 0; // Add above right if above is available
  refHeight += refSizeX ? extHeight : 0; // Add below left if left is available

  m_cccmBlkArea = chromaArea;
  m_cccmRefArea = Area(chromaArea.x - refSizeX, chromaArea.y - refSizeY, refWidth, refHeight);
}

// Return downsampled luma buffer that contains PU and the reference areas above and left of the PU
PelBuf IntraPrediction::xCccmGetLumaRefBuf(const PredictionUnit& pu, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY
#if JVET_AD0202_CCCM_MDF
  , int cccmDownsamplesFilterIdx, int numBuffer, PelBuf* refLuma1, PelBuf* refLuma3, PelBuf* refLuma2
#endif
) const
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if( pu.cccmNoSubFlag )
  {
    const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
    const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );

    refPosPicX = m_cccmRefArea.x << chromaScaleX;                          // Position of the reference area in picture coordinates
    refPosPicY = m_cccmRefArea.y << chromaScaleY;
    refSizeX   = ((m_cccmBlkArea.x - m_cccmRefArea.x) << chromaScaleX);    // Reference lines available left and above
    refSizeY   = ((m_cccmBlkArea.y - m_cccmRefArea.y) << chromaScaleY);
    areaWidth  = m_cccmRefArea.width << chromaScaleX;                      // Reference buffer size excluding paddings
    areaHeight = m_cccmRefArea.height << chromaScaleY;

    int refStride = areaWidth + 2 * (CCCM_FILTER_PADDING << chromaScaleX); // Including paddings required for the 2D filter
    int refOrigin = refStride * (CCCM_FILTER_PADDING << chromaScaleY) + (CCCM_FILTER_PADDING << chromaScaleX);
#if JVET_AD0202_CCCM_MDF
    return PelBuf(m_cccmLumaBuf[0] + refOrigin, refStride, areaWidth, areaHeight); // Points to the top-left corner of the reference area 
#else
    return PelBuf( m_cccmLumaBuf[1] + refOrigin, refStride, areaWidth, areaHeight ); // Points to the top-left corner of the reference area 
#endif
  }
#endif

  refSizeX   = m_cccmBlkArea.x - m_cccmRefArea.x; // Reference lines available left and above
  refSizeY   = m_cccmBlkArea.y - m_cccmRefArea.y;
  areaWidth  = m_cccmRefArea.width;               // Reference buffer size excluding paddings
  areaHeight = m_cccmRefArea.height;
  refPosPicX = m_cccmRefArea.x;                   // Position of the reference area in picture coordinates
  refPosPicY = m_cccmRefArea.y;

  int refStride = areaWidth + 2 * CCCM_FILTER_PADDING; // Including paddings required for the 2D filter
  int refOrigin = refStride * CCCM_FILTER_PADDING + CCCM_FILTER_PADDING;

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  if (numBuffer == 0)
  {
    return PelBuf(m_cccmLumaBuf[cccmDownsamplesFilterIdx + 1] + refOrigin, refStride, areaWidth, areaHeight); // Points to the top-left corner of the reference area
}
  else if (numBuffer == 2)
  {
    *refLuma1 = PelBuf(m_cccmLumaBuf[2] + refOrigin, refStride, areaWidth, areaHeight);
    *refLuma3 = PelBuf(m_cccmLumaBuf[4] + refOrigin, refStride, areaWidth, areaHeight);
    return PelBuf(m_cccmLumaBuf[1] + refOrigin, refStride, areaWidth, areaHeight); // Points to the top-left corner of the reference area
  }
  else
  {
    *refLuma1 = PelBuf(m_cccmLumaBuf[2] + refOrigin, refStride, areaWidth, areaHeight);
    *refLuma2 = PelBuf(m_cccmLumaBuf[3] + refOrigin, refStride, areaWidth, areaHeight);
    *refLuma3 = PelBuf(m_cccmLumaBuf[4] + refOrigin, refStride, areaWidth, areaHeight);
    return PelBuf(m_cccmLumaBuf[1] + refOrigin, refStride, areaWidth, areaHeight); // Points to the top-left corner of the reference area
  }
#else
  return PelBuf( m_cccmLumaBuf[0] + refOrigin, refStride, areaWidth, areaHeight ); // Points to the top-left corner of the reference area
#endif
#else
  return PelBuf(m_cccmLumaBuf + refOrigin, refStride, areaWidth, areaHeight); // Points to the top-left corner of the reference area
#endif
}

// Return downsampled luma buffer for a PU
PelBuf IntraPrediction::xCccmGetLumaPuBuf(const PredictionUnit& pu
#if JVET_AD0202_CCCM_MDF
  , int cccmDownsamplesFilterIdx, int numBuffer, CPelBuf* refLuma1, CPelBuf* refLuma3, CPelBuf* refLuma2
#endif
) const
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING || JVET_AF0073_INTER_CCP_MERGE
  if( pu.cccmNoSubFlag )
  {
    const int chromaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );
    const int chromaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.cu->slice->getSPS()->getChromaFormatIdc() );

    int refSizeX = ((m_cccmBlkArea.x - m_cccmRefArea.x) << chromaScaleX); // Reference lines available left and above
    int refSizeY = ((m_cccmBlkArea.y - m_cccmRefArea.y) << chromaScaleY);
    int tuWidth  = m_cccmBlkArea.width << chromaScaleX;
    int tuHeight = m_cccmBlkArea.height << chromaScaleY;

    int refStride = (m_cccmRefArea.width + 2 * CCCM_FILTER_PADDING) << chromaScaleX; // Including paddings required for the 2D filter
    int refOrigin = refStride * (refSizeY + (CCCM_FILTER_PADDING << chromaScaleY)) + refSizeX + (CCCM_FILTER_PADDING << chromaScaleX);
#if JVET_AD0202_CCCM_MDF
    return PelBuf(m_cccmLumaBuf[0] + refOrigin, refStride, tuWidth, tuHeight);  // Points to the top-left corner of the block
#else
    return PelBuf( m_cccmLumaBuf[1] + refOrigin, refStride, tuWidth, tuHeight );  // Points to the top-left corner of the block
#endif
  }
#endif

  int refSizeX  = m_cccmBlkArea.x - m_cccmRefArea.x; // Reference lines available left and above
  int refSizeY  = m_cccmBlkArea.y - m_cccmRefArea.y;
  int tuWidth   = m_cccmBlkArea.width;
  int tuHeight  = m_cccmBlkArea.height;
  int refStride = m_cccmRefArea.width + 2 * CCCM_FILTER_PADDING; // Including paddings required for the 2D filter
  int refOrigin = refStride * (refSizeY + CCCM_FILTER_PADDING) + refSizeX + CCCM_FILTER_PADDING;

#if JVET_AC0147_CCCM_NO_SUBSAMPLING
#if JVET_AD0202_CCCM_MDF
  if (numBuffer == 0)
  {
    return PelBuf(m_cccmLumaBuf[1] + refOrigin, refStride, tuWidth, tuHeight);  // Points to the top-left corner of the block
  }
  else if (numBuffer == 2)
  {
    *refLuma1 = PelBuf(m_cccmLumaBuf[2] + refOrigin, refStride, tuWidth, tuHeight);
    *refLuma3 = PelBuf(m_cccmLumaBuf[4] + refOrigin, refStride, tuWidth, tuHeight);
    return PelBuf(m_cccmLumaBuf[1] + refOrigin, refStride, tuWidth, tuHeight);  // Points to the top-left corner of the block
  }
  else
  {
    *refLuma1 = PelBuf(m_cccmLumaBuf[2] + refOrigin, refStride, tuWidth, tuHeight);
    *refLuma2 = PelBuf(m_cccmLumaBuf[3] + refOrigin, refStride, tuWidth, tuHeight);
    *refLuma3 = PelBuf(m_cccmLumaBuf[4] + refOrigin, refStride, tuWidth, tuHeight);
    return PelBuf(m_cccmLumaBuf[1] + refOrigin, refStride, tuWidth, tuHeight);  // Points to the top-left corner of the block
  }
#else
  return PelBuf( m_cccmLumaBuf[0] + refOrigin, refStride, tuWidth, tuHeight );  // Points to the top-left corner of the block
#endif
#else
  return PelBuf(m_cccmLumaBuf + refOrigin, refStride, tuWidth, tuHeight);  // Points to the top-left corner of the block
#endif
}

int IntraPrediction::xCccmCalcRefAver(const PredictionUnit& pu
#if JVET_AD0120_LBCCP
                                      ,int trainingRange
#endif
) const
{
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refLuma = xCccmGetLumaRefBuf(pu, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);
  
  int numSamples = 0;
  int sumSamples = 0;
  
#if JVET_AB0143_CCCM_TS && MMLM
#if JVET_AD0120_LBCCP
  if (trainingRange != -1)
  {
    CPelBuf refLumaBlk = xCccmGetLumaPuBuf(pu);
    const auto divShift = floorLog2(refLumaBlk.height) + floorLog2(refLumaBlk.width);
    const auto offset   = 1 << (divShift - 1);
    for (int y = 0; y < refLumaBlk.height; y++)
    {
      for (int x = 0; x < refLumaBlk.width; x++)
      {
        sumSamples += refLumaBlk.at(x, y);
      }
    }
    return (sumSamples + offset) >> divShift;
  }
#endif
  if( pu.cccmFlag == 1 || pu.cccmFlag == 3 )
  {
    // above samples
    for( int y = 0; y < refSizeY; y++ )
    {
      for( int x = (pu.cccmFlag == 3 ? refSizeX : 0); x < areaWidth; x++ )
      {
        sumSamples += refLuma.at( x, y );
        numSamples++;
      }
    }
  }

  if( pu.cccmFlag == 1 || pu.cccmFlag == 2 )
  {
    // left samples
    for (int y = refSizeY; y < areaHeight; y++)
    {
      for (int x = 0; x < refSizeX; x++)
      {
        sumSamples += refLuma.at(x, y);
        numSamples++;
      }
    }
  }  
#else
  // Top samples
  for (int y = 0; y < refSizeY; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      sumSamples += refLuma.at(x, y);
      numSamples++;
    }
  }

  // Left samples
  for (int y = refSizeY; y < areaHeight; y++)
  {
    for (int x = 0; x < refSizeX; x++)
    {
      sumSamples += refLuma.at(x, y);
      numSamples++;
    }
  }
#endif

#if JVET_AB0174_CCCM_DIV_FREE
  return numSamples == 0 ? 512 : xCccmDivideLowPrec(sumSamples, numSamples);
#else
  return numSamples == 0 ? 512 : ( sumSamples + numSamples/2) / numSamples;
#endif
}

void IntraPrediction::xCccmCreateLumaRef(const PredictionUnit& pu, CompArea chromaArea
#if JVET_AD0202_CCCM_MDF
  , int downsFilterIdx
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate
#endif
)
{
  const CPelBuf recoLuma = pu.cs->picture->getRecoBuf(COMPONENT_Y);
  const int  maxPosPicX  = pu.cs->picture->chromaSize().width  - 1;
  const int  maxPosPicY  = pu.cs->picture->chromaSize().height - 1;

  xCccmCalcRefArea(pu, chromaArea); // Find the reference area
  
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refLuma = xCccmGetLumaRefBuf(pu, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY
#if JVET_AD0202_CCCM_MDF
    , downsFilterIdx
#endif
  );
  
  int puBorderX = refSizeX + m_cccmBlkArea.width;
  int puBorderY = refSizeY + m_cccmBlkArea.height;
  
#if JVET_AB0174_CCCM_DIV_FREE
  xCccmSetLumaRefValue( pu );
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  if (!isTemplate)
  {
#endif
  // Generate down-sampled luma for the area covering both the PU and the top/left reference areas (+ top and left paddings)
  for (int y = -CCCM_FILTER_PADDING; y < areaHeight; y++)
  {
    for (int x = -CCCM_FILTER_PADDING; x < areaWidth; x++)
    {
      if (( x >= puBorderX && y >= refSizeY ) ||
          ( y >= puBorderY && x >= refSizeX ))
      {
        continue;
      }

      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;
      
      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
      
      refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
        , downsFilterIdx
#endif
      );
    }
  }

  CHECK( CCCM_FILTER_PADDING != 1, "Only padding with one sample implemented" );

  // Pad right of top reference area
  for (int y = -1; y < refSizeY; y++)
  {
    refLuma.at( areaWidth, y ) = refLuma.at( areaWidth - 1, y );
  }

  // Pad right of PU
  for (int y = refSizeY; y < puBorderY; y++)
  {
    refLuma.at( puBorderX, y ) = refLuma.at( puBorderX - 1, y );
  }

  // Pad right of left reference area
  for (int y = puBorderY; y < areaHeight; y++)
  {
    refLuma.at( refSizeX, y ) = refLuma.at( refSizeX - 1, y );
  }

  // Pad below left reference area
  for (int x = -1; x < refSizeX + 1; x++)
  {
    refLuma.at( x, areaHeight ) = refLuma.at( x, areaHeight - 1 );
  }

  // Pad below PU
  for (int x = refSizeX; x < puBorderX + 1; x++)
  {
    refLuma.at( x, puBorderY ) = refLuma.at( x, puBorderY - 1 );
  }

  // Pad below right reference area
  for (int x = puBorderX + 1; x < areaWidth + 1; x++)
  {
    refLuma.at( x, refSizeY ) = refLuma.at( x, refSizeY - 1 );
  }
  
  // In dualtree we can also use luma from the right and below (if not on CTU/picture boundary)
  if ( CS::isDualITree( *pu.cs ) )
  {
    int ctuWidth  = pu.cs->sps->getMaxCUWidth()  >> getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
    int ctuHeight = pu.cs->sps->getMaxCUHeight() >> getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);

    // Samples right of top reference area
    int padPosPicX = refPosPicX + areaWidth;

    if ( padPosPicX <= maxPosPicX && (padPosPicX % ctuWidth) )
    {
      for (int y = -1; y < refSizeY; y++)
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at( areaWidth, y ) = xCccmGetLumaVal(pu, recoLuma, padPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
#endif
        );
      }
    }

    // Samples right of PU
    padPosPicX = refPosPicX + puBorderX;

    if ( padPosPicX <= maxPosPicX && (padPosPicX % ctuWidth) )
    {
      for (int y = refSizeY; y < puBorderY; y++)
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at( puBorderX, y ) = xCccmGetLumaVal(pu, recoLuma, padPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
#endif
        );
      }
    }

    // Samples right of left reference area
    padPosPicX = refPosPicX + refSizeX;

    if ( padPosPicX <= maxPosPicX )
    {
      for (int y = puBorderY; y < areaHeight; y++)
      {
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at( refSizeX, y ) = xCccmGetLumaVal(pu, recoLuma, padPosPicX, chromaPosPicY
#if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
#endif
        );
      }
    }
    
    // Samples below left reference area
    int padPosPicY = refPosPicY + areaHeight;
    
    if ( padPosPicY <= maxPosPicY && (padPosPicY % ctuHeight) )
    {
      for (int x = -1; x < refSizeX + 1; x++)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        
        refLuma.at( x, areaHeight ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, padPosPicY
#if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
#endif
        );
      }
    }
    
    // Samples below PU
    padPosPicY = refPosPicY + puBorderY;
    
    if ( padPosPicY <= maxPosPicY && (padPosPicY % ctuHeight) )
    {
      for (int x = refSizeX; x < puBorderX; x++) // Just go to PU border as the next sample may be out of CTU (and not needed anyways)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        
        refLuma.at( x, puBorderY ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, padPosPicY
#if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
#endif
        );
      }
    }

    // Samples below right reference area
    padPosPicY = refPosPicY + refSizeY;
    
    if ( padPosPicY <= maxPosPicY )
    {
      // Avoid going outside of right CTU border where these samples are not yet available
      int puPosPicX        = m_cccmBlkArea.x;
      int ctuRightEdgeDist = ctuWidth - (puPosPicX % ctuWidth) + refSizeX;
      int lastPosX         = ctuRightEdgeDist < areaWidth ? ctuRightEdgeDist : areaWidth;

      for (int x = puBorderX + 1; x < lastPosX; x++) // Just go to ref area border as the next sample may be out of CTU (and not needed anyways)
      {
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        
        refLuma.at( x, refSizeY ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, padPosPicY
#if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
#endif
        );
      }
    }
  }
#if JVET_AF0073_INTER_CCP_MERGE
  }
  else
  {
    // Generate down-sampled luma for area needed to compute template cost
    int startY = refSizeY > 0 ? refSizeY - 1 - CCCM_FILTER_PADDING : -CCCM_FILTER_PADDING;
    int startX = refSizeX > 0 ? refSizeX - 1 - CCCM_FILTER_PADDING : -CCCM_FILTER_PADDING;
    for (int y = startY; y < areaHeight; y++)
    {
      for (int x = startX; x < areaWidth; x++)
      {
        if ( x > refSizeX && y > refSizeY )
        {
          continue;
        }

        int chromaPosPicX = refPosPicX + x;
        int chromaPosPicY = refPosPicY + y;
        
        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;
        
        refLuma.at( x, y ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY
  #if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
  #endif
        );
      }
    }

    CHECK( CCCM_FILTER_PADDING != 1, "Only padding with one sample implemented" );

    // Pad right of top reference area
    for (int y = startY; y < refSizeY; y++)
    {
      refLuma.at( areaWidth, y ) = refLuma.at( areaWidth - 1, y );
    }

    // Pad right of PU
    refLuma.at( puBorderX, refSizeY ) = refLuma.at( puBorderX - 1, refSizeY );

    // Pad below left reference area
    for (int x = startX; x < refSizeX + 1; x++)
    {
      refLuma.at( x, areaHeight ) = refLuma.at( x, areaHeight - 1 );
    }

    // Pad below PU
    refLuma.at( refSizeX, puBorderY ) = refLuma.at( refSizeX, puBorderY - 1 );
  
    // In dualtree we can also use luma from the right and below (if not on CTU/picture boundary)
    if ( CS::isDualITree( *pu.cs ) )
    {
      int ctuWidth  = pu.cs->sps->getMaxCUWidth()  >> getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
      int ctuHeight = pu.cs->sps->getMaxCUHeight() >> getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);

      // Samples right of top reference area
      int padPosPicX = refPosPicX + areaWidth;

      if ( padPosPicX <= maxPosPicX && (padPosPicX % ctuWidth) )
      {
        for (int y = startY; y < refSizeY; y++)
        {
          int chromaPosPicY = refPosPicY + y;
          chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

          refLuma.at( areaWidth, y ) = xCccmGetLumaVal(pu, recoLuma, padPosPicX, chromaPosPicY
  #if JVET_AD0202_CCCM_MDF
            , downsFilterIdx
  #endif
          );
        }
      }

      // Samples right of PU
      padPosPicX = refPosPicX + puBorderX;

      if ( padPosPicX <= maxPosPicX && (padPosPicX % ctuWidth) )
      {
        int y = refSizeY;
        int chromaPosPicY = refPosPicY + y;
        chromaPosPicY     = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at( puBorderX, y ) = xCccmGetLumaVal(pu, recoLuma, padPosPicX, chromaPosPicY
  #if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
  #endif
        );
      }
      
      // Samples below left reference area
      int padPosPicY = refPosPicY + areaHeight;
      
      if ( padPosPicY <= maxPosPicY && (padPosPicY % ctuHeight) )
      {
        for (int x = startX; x < refSizeX + 1; x++)
        {
          int chromaPosPicX = refPosPicX + x;
          chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
          
          refLuma.at( x, areaHeight ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, padPosPicY
  #if JVET_AD0202_CCCM_MDF
            , downsFilterIdx
  #endif
          );
        }
      }
      
      // Samples below PU
      padPosPicY = refPosPicY + puBorderY;
      
      if ( padPosPicY <= maxPosPicY && (padPosPicY % ctuHeight) )
      {
        int x = refSizeX;
        int chromaPosPicX = refPosPicX + x;
        chromaPosPicX     = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        
        refLuma.at( x, puBorderY ) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, padPosPicY
  #if JVET_AD0202_CCCM_MDF
          , downsFilterIdx
  #endif
        );
      }
    }
  }
#endif
}
#endif

#if JVET_AC0119_LM_CHROMA_FUSION
void IntraPrediction::xCflmCalcRefArea(const PredictionUnit& pu, const CompArea& chromaArea)
{
  const ChannelType     chType = CHANNEL_TYPE_CHROMA;
  const CodingUnit& cu = *pu.cu;
  const CodingStructure& cs = *cu.cs;
  const SPS& sps = *cs.sps;
  const PreCalcValues& pcv = *cs.pcv;

  const int tuWidth = chromaArea.width;
  const int tuHeight = chromaArea.height;

  const bool noShift = pcv.noChroma2x2 && chromaArea.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int  compScaleX = getComponentScaleX(chromaArea.compID, sps.getChromaFormatIdc());
  const int  compScaleY = getComponentScaleY(chromaArea.compID, sps.getChromaFormatIdc());
  const int  unitWidth = pcv.minCUWidth >> (noShift ? 0 : compScaleX);
  const int  unitHeight = pcv.minCUHeight >> (noShift ? 0 : compScaleY);

  const int  totalAboveUnits = (2 * tuWidth + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits = (2 * tuHeight + (unitHeight - 1)) / unitHeight;
  const int  numAboveUnits = std::max<int>(tuWidth / unitWidth, 1);
  const int  numLeftUnits = std::max<int>(tuHeight / unitHeight, 1);
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits = totalLeftUnits - numLeftUnits;

  static bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1] = { false }; // Just a dummy array here, content not used

  int avaiAboveRightUnits = isAboveRightAvailable(cu, chType, chromaArea.topRight(), numAboveRightUnits, unitWidth, (neighborFlags + totalLeftUnits + 1 + numAboveUnits));
  int avaiLeftBelowUnits = isBelowLeftAvailable(cu, chType, chromaArea.bottomLeft(), numLeftBelowUnits, unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits));

  int refSizeX, refSizeY;

  PU::getCccmRefLineNum(pu, chromaArea, refSizeX, refSizeY); // Reference lines available left and above

  int refWidth = chromaArea.width + refSizeX;              // Reference buffer size excluding paddings
  int refHeight = chromaArea.height + refSizeY;

  int extWidth = avaiAboveRightUnits * unitWidth;
  int extHeight = avaiLeftBelowUnits * unitHeight;

  refWidth += refSizeY ? extWidth : 0; // Add above right if above is available
  refHeight += refSizeX ? extHeight : 0; // Add below left if left is available

  m_cccmBlkArea = chromaArea;
  m_cflmRefArea = Area(chromaArea.x - refSizeX, chromaArea.y - refSizeY, refWidth, refHeight); // Position with respect to the PU
}

PelBuf IntraPrediction::xCflmGetRefBuf(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, int& areaWidth, int& areaHeight, int& refSizeX, int& refSizeY, int& refPosPicX, int& refPosPicY) const
{
  refSizeX = std::min(chromaArea.x - m_cflmRefArea.x, 2);  // Reference lines available left and above
  refSizeY = std::min(chromaArea.y - m_cflmRefArea.y, 2);
  areaWidth = chromaArea.width + refSizeX;
  areaHeight = chromaArea.height + refSizeY;
  refPosPicX = chromaArea.x - refSizeX; // Position of the reference area in picture coordinates
  refPosPicY = chromaArea.y - refSizeY;

  return PelBuf(m_cflmBuf[compId], areaWidth, areaWidth, areaHeight); // Points to the top-left corner of the reference area
}

PelBuf IntraPrediction::xCflmGetPuBuf(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea) const
{
  int refSizeX = std::min(chromaArea.x - m_cflmRefArea.x, 2);  // Reference lines available left and above
  int refSizeY = std::min(chromaArea.y - m_cflmRefArea.y, 2);
  int tuWidth = chromaArea.width;
  int tuHeight = chromaArea.height;
  int refStride = chromaArea.width + refSizeX;
  int refOrigin = refStride * refSizeY + refSizeX;

  return PelBuf(m_cflmBuf[compId] + refOrigin, refStride, tuWidth, tuHeight);  // Points to the top-left corner of the block
}

void IntraPrediction::xCflmCreateLumaRef(const PredictionUnit& pu, const CompArea& chromaArea)
{
  const CPelBuf recoLuma = pu.cs->picture->getRecoBuf(COMPONENT_Y);
  const int  maxPosPicX = pu.cs->picture->chromaSize().width - 1;
  const int  maxPosPicY = pu.cs->picture->chromaSize().height - 1;

  xCflmCalcRefArea(pu, chromaArea); // Find the reference area

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refLuma = xCflmGetRefBuf(pu, COMPONENT_Y, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

#if JVET_AB0174_CCCM_DIV_FREE
  xCccmSetLumaRefValue(pu);
#endif

  // Generate down-sampled luma for the area covering both the PU and the top/left reference areas
  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if (x >= 0 && x < refSizeX && y >= 0 && y < refSizeY)
      {
        continue;
      }

      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xCccmGetLumaVal(pu, recoLuma, chromaPosPicX, chromaPosPicY);
    }
  }
}

bool IntraPrediction::xCflmCreateChromaPred(const PredictionUnit& pu, const ComponentID compId, PelBuf& piPred)
{
  uint32_t iMode = PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA);

  const CompArea& area = pu.blocks[compId];
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refChroma = xCflmGetRefBuf(pu, compId, area, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  // top/left reference areas
  TemplateType eTplType = (TemplateType)((refSizeX > 0 ? LEFT_NEIGHBOR : 0) + (refSizeY > 0 ? ABOVE_NEIGHBOR : 0));
  if (eTplType == NO_NEIGHBOR)
  {
    return false;
  }

  m_topRefLength = areaWidth << 1;
  m_leftRefLength = areaHeight << 1;
  xFillTimdReferenceSamples(pu.cs->picture->getRecoBuf(area), m_refBuffer[compId][PRED_BUF_UNFILTERED], area, *pu.cu, refSizeX, refSizeY);

  Pel* piRefPred = refChroma.bufAt(0, 0);
  initPredIntraParams(pu, area, *(pu.cs->sps));
  predTimdIntraAng(compId, pu, iMode, piRefPred, refChroma.stride, areaWidth, areaHeight, eTplType, refSizeX, refSizeY);

  // the PU reference areas
  PelBuf predChroma = xCflmGetPuBuf(pu, compId, area);
  Pel* piPredBuf = piPred.buf;
  for (int y = 0; y < piPred.height; y++)
  {
    for (int x = 0; x < piPred.width; x++)
    {
      predChroma.at(x, y) = piPredBuf[x];
    }

    piPredBuf += piPred.stride;
  }

#if JVET_AB0174_CCCM_DIV_FREE
  int chromaOffset = 1 << (pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
  const CPelBuf reco = pu.cs->picture->getRecoBuf(compId);

  if (refSizeX || refSizeY)
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;

    chromaOffset = reco.at(refPosPicX + refPosX, refPosPicY + refPosY);
  }

  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      piRefPred[x] -= chromaOffset;
    }

    piRefPred += refChroma.stride;
  }
#endif

  return true;
}

void IntraPrediction::xCflmCalcModels(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& cflmModel, int modelId, int modelThr)
{
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  const CPelBuf reco = pu.cs->picture->getRecoBuf(compId);
  PelBuf        refLuma = xCflmGetRefBuf(pu, COMPONENT_Y, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);
  PelBuf        refChroma = xCflmGetRefBuf(pu, compId, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  int sampleNum = 0;

#if JVET_AB0174_CCCM_DIV_FREE
  int chromaOffset = 1 << (pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);

  if (refSizeX || refSizeY)
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;

    chromaOffset = reco.at(refPosPicX + refPosX, refPosPicY + refPosY);
  }
#endif

  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if ((x >= refSizeX && y >= refSizeY)
        || (x >= 0 && x < refSizeX && y >= 0 && y < refSizeY))
      {
        continue;
      }

      if (modelId == 1 && refLuma.at(x, y) > modelThr) // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if (modelId == 2 && refLuma.at(x, y) <= modelThr) // Model 2: Include only samples above the threshold
      {
        continue;
      }

      if (pu.intraDir[1] == MDLM_L_IDX || pu.intraDir[1] == MMLM_L_IDX)
      {
        if (y < refSizeY)
        {
          continue;
        }
      }

      if (pu.intraDir[1] == MDLM_T_IDX || pu.intraDir[1] == MMLM_T_IDX)
      {
        if (x < refSizeX)
        {
          continue;
        }
      }

      m_a[0][sampleNum] = refLuma.at(x, y); // Luma
      m_a[1][sampleNum] = refChroma.at(x, y); // Chroma
      m_a[2][sampleNum] = cflmModel.bias();

      m_cb[sampleNum] = reco.at(refPosPicX + x, refPosPicY + y);
      sampleNum++;
    }
  }

  if (!sampleNum) // Number of samples can go to zero in the multimode case
  {
    cflmModel.clearModel();
    return;
  }
  else
  {
#if JVET_AB0174_CCCM_DIV_FREE
    m_cccmSolver.solve1(m_a, m_cb, sampleNum, chromaOffset, cflmModel);
#else
    m_cccmSolver.solve1(m_a, m_cb, sampleNum, cflmModel);
#endif
  }
}

void IntraPrediction::xCflmApplyModel(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& cflmModel, int modelId, int modelThr, PelBuf& piPred)
{
  const  ClpRng& clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;

  CPelBuf refLumaBlk = xCflmGetPuBuf(pu, COMPONENT_Y, chromaArea);
  CPelBuf refChromaBlk = xCflmGetPuBuf(pu, compId, chromaArea);

  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      if (modelId == 1 && refLumaBlk.at(x, y) > modelThr) // Model 1: Include only samples below or equal to the threshold
      {
        continue;
      }
      if (modelId == 2 && refLumaBlk.at(x, y) <= modelThr) // Model 2: Include only samples above the threshold
      {
        continue;
      }

      samples[0] = refLumaBlk.at(x, y); // Luma
      samples[1] = refChromaBlk.at(x, y); // Chroma
      samples[2] = cflmModel.bias();

      piPred.at(x, y) = ClipPel<Pel>(cflmModel.convolve(samples), clpRng);
    }
  }
}

int IntraPrediction::xCflmCalcRefAver(const PredictionUnit& pu, const CompArea& chromaArea)
{
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refLuma = xCflmGetRefBuf(pu, COMPONENT_Y, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY);

  int numSamples = 0;
  int sumSamples = 0;

  // Top samples
  if (pu.intraDir[1] != MDLM_L_IDX && pu.intraDir[1] != MMLM_L_IDX)
  {
    for (int y = 0; y < refSizeY; y++)
    {
      for (int x = refSizeX; x < areaWidth; x++)
      {
        sumSamples += refLuma.at(x, y);
        numSamples++;
      }
    }
  }

  // Left samples
  if (pu.intraDir[1] != MDLM_T_IDX && pu.intraDir[1] != MMLM_T_IDX)
  {
    for (int y = refSizeY; y < areaHeight; y++)
    {
      for (int x = 0; x < refSizeX; x++)
      {
        sumSamples += refLuma.at(x, y);
        numSamples++;
      }
    }
  }

#if JVET_AD0184_REMOVAL_OF_DIVISION_OPERATIONS
  return numSamples == 0 ? 512 : PU::getMeanValue( sumSamples + (numSamples >> 1), numSamples);
#else
  return numSamples == 0 ? 512 : (sumSamples + numSamples / 2) / numSamples;
#endif
}
#endif

#if JVET_AA0057_CCCM || JVET_AB0092_GLM_WITH_LUMA || JVET_AC0119_LM_CHROMA_FUSION
    
#if JVET_AC0053_GAUSSIAN_SOLVER
void CccmCovariance::gaussBacksubstitution( TCccmCoeff* x, int numEq, int col )
{
  x[numEq-1] = C[numEq-1][col];

  for( int i = numEq-2; i >= 0; i-- )
  {
    x[i] = C[i][col];

    for( int j = i+1; j < numEq; j++ )
    {
      x[i] -= FIXED_MULT(C[i][j], x[j]);
    }
  }
}

void CccmCovariance::gaussElimination( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y0, TCccmCoeff* x0, TCccmCoeff* y1, TCccmCoeff* x1, int numEq, int numFilters, int bd
#if JVET_AE0059_INTER_CCCM
  ,const bool interCccmMode
#endif
)
{ 
  int colChr0 = numEq;
  int colChr1 = numEq + 1;
#if JVET_AE0059_INTER_CCCM
  int reg = interCccmMode ? 1 : 2 << (bd - 8);
#else
  int reg = 2 << (bd - 8);
#endif
  
  // Create an [M][M+2] matrix system (could have been done already when calculating auto/cross-correlations)
  for( int i = 0; i < numEq; i++ )
  {
    for( int j = 0; j < numEq; j++ )
    {
      C[i][j] = j >= i ? A[i][j] : A[j][i];
    }
    
    C[i][i]      += reg; // Regularization
    C[i][colChr0] = y0[i];
    C[i][colChr1] = numFilters == 2 ? y1[i] : 0; // Only applicable if solving for 2 filters at the same time
  }

  for( int i = 0; i < numEq; i++ )
  {
    TCccmCoeff *src = C[i];
    TCccmCoeff diag = src[i] < 1 ? 1 : src[i];

#if JVET_AB0174_CCCM_DIV_FREE
    int scale, round, shift;
    
    xGetDivScaleRoundShift(diag, scale, round, shift);
#endif

    for( int j = i+1; j < numEq+numFilters; j++ )
    {
#if JVET_AB0174_CCCM_DIV_FREE
      src[j] = ( int64_t(src[j]) * scale + round ) >> shift;
#else
      src[j] = FIXED_DIV(src[j], diag);
#endif
    }
    
    for( int j = i + 1; j < numEq; j++ )
    {
      TCccmCoeff *dst  = C[j];
      TCccmCoeff scale = dst[i];

      // On row j all elements with k < i+1 are now zero (not zeroing those here as backsubstitution does not need them)
      for( int k = i + 1; k < numEq+numFilters; k++ )
      {
         dst[k] -= FIXED_MULT(scale, src[k]);
      }
    }
  }

  // Solve with backsubstitution
  if ( numFilters == 2 )
  {
    gaussBacksubstitution(x0, numEq, colChr0);
    gaussBacksubstitution(x1, numEq, colChr1);
  }
  else
  {
    gaussBacksubstitution(x0, numEq, colChr0);
  }
}

#else

// LDL decomposing A to U'*diag*U
bool CccmCovariance::ldlDecomp( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, int numEq) const
{
  for (int i = 0; i < numEq; i++)
  {
    diag[i] = A[i][i];
    
    for (int k = i - 1; k >= 0; k--)
    {
      TCccmCoeff tmp = FIXED_MULT(U[k][i], U[k][i]);
      diag[i]       -= FIXED_MULT(tmp, diag[k]);
    }

    if ( diag[i] <= 0) // A is singular
    {
      return false;
    }

    for (int j = i + 1; j < numEq; j++)
    {
      TCccmCoeff scale = A[i][j];
      
      for (int k = i - 1; k >= 0; k--)
      {
        TCccmCoeff tmp = FIXED_MULT(U[k][j], U[k][i]);
        scale         -= FIXED_MULT(tmp, diag[k]);
      }

#if JVET_AB0174_CCCM_DIV_FREE
      U[i][j] = xCccmDivide(scale, diag[i]);
#else
      U[i][j] = FIXED_DIV(scale, diag[i]);
#endif
    }
  }

  return true;
}

// Solve U'z = y for z
void CccmCovariance::ldlTransposeBacksubstitution( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* y, TCccmCoeff* z, int numEq) const
{
  z[0] = y[0];
  
  for (int i = 1; i < numEq; i++)
  {
    TCccmCoeff sum = 0;

    for (int j = 0; j < i; j++)
    {
      sum += FIXED_MULT(z[j], U[j][i]);
    }

    z[i] = y[i] - sum;
  }
}

// Solve Ux = z for x
void CccmCovariance::ldlBacksubstitution( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* z, TCccmCoeff* x, int numEq) const
{
  x[numEq - 1] = z[numEq - 1];

  for (int i = numEq - 2; i >= 0; i--)
  {
    TCccmCoeff sum = 0;

    for (int j = i + 1; j < numEq; j++)
    {
      sum += FIXED_MULT(U[i][j], x[j]);
    }

    x[i] = z[i] - sum;
  }
}

bool CccmCovariance::ldlDecompose( TCccmCoeff A[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, int numEq) const
{
  // Compute upper triangular U and diagonal D such that U'*D*U = A
  // (U being the tranpose of L in LDL decomposition: L*D*L' = A)

  // Regularize A to reduce singularities
  for (int i = 0; i < numEq; i++)
  {
    A[i][i] += 1;
  }
  
  return ldlDecomp(A, U, diag, numEq);
}

void CccmCovariance::ldlSolve( TCccmCoeff U[CCCM_NUM_PARAMS_MAX][CCCM_NUM_PARAMS_MAX], TCccmCoeff* diag, TCccmCoeff* y, TCccmCoeff* x, int numEq, bool decompOk) const
{
  if ( decompOk )
  {
    // Now, the equation is  U'*D*U*x = y, where U is upper triangular
    // Solve U'*aux = y for aux
    Ty aux;
    
    ldlTransposeBacksubstitution(U, y, aux, numEq);

    // The equation is now D*U*x = aux, remove diagonal by scaling
    for (int i = 0; i < numEq; i++)
    {
#if JVET_AB0174_CCCM_DIV_FREE
      aux[i] = xCccmDivide(aux[i], diag[i]);
#else
      aux[i] = FIXED_DIV(aux[i], diag[i]);
#endif
    }
    
    // The equation is now U*x = aux, solve it for x (filter coefficients)
    ldlBacksubstitution(U, aux, x, numEq);
  }
  else // A was singular
  {
    std::memset(x, 0, sizeof(TCccmCoeff) * numEq);
  }
}
#endif

#if JVET_AB0174_CCCM_DIV_FREE
void CccmCovariance::solve1( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* C, const int sampleNum, const int chromaOffset, CccmModel& model )
#else
void CccmCovariance::solve1( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* C, const int sampleNum, CccmModel& model )
#endif
{
  const int numParams = model.getNumParams();

  CHECK( CCCM_REF_SAMPLES_MAX < sampleNum, "Insufficient buffer size" );
  CHECK( CCCM_NUM_PARAMS_MAX < numParams, "Insufficient buffer size" );

  for( int i = 0; i < numParams; i++ )
  {
    memset( ATA[i], 0x00, sizeof( TCccmCoeff ) * numParams );
  }
  memset( ATCb, 0x00, sizeof( TCccmCoeff ) * numParams );

  for( int coli0 = 0; coli0 < numParams; coli0++ )
  {
    for( int coli1 = coli0; coli1 < numParams; coli1++ )
    {
      const Pel *col0 = A[coli0];
      const Pel *col1 = A[coli1];

      for( int rowi = 0; rowi < sampleNum; rowi++ )
      {
        ATA[coli0][coli1] += col0[rowi] * col1[rowi];
      }
    }
  }

  for( int coli = 0; coli < numParams; coli++ )
  {
    const Pel *col = A[coli];

    for( int rowi = 0; rowi < sampleNum; rowi++ )
    {
      ATCb[coli] += col[rowi] * C[rowi];
    }
  }

#if JVET_AB0174_CCCM_DIV_FREE
  // Remove chromaOffset from stats to update cross-correlation
  for( int coli = 0; coli < numParams; coli++ )
  {
    ATCb[coli] = ATCb[coli] - ((ATA[coli][numParams - 1] * chromaOffset) >> (model.bd - 1));
  }
#endif

  // Scale the matrix and vector to selected dynamic range
  int matrixShift = 28 - 2 * model.bd - ceilLog2( sampleNum );

  if( matrixShift > 0 )
  {
    for( int coli0 = 0; coli0 < numParams; coli0++ )
    {
      for( int coli1 = coli0; coli1 < numParams; coli1++ )
      {
        ATA[coli0][coli1] <<= matrixShift;
      }
    }

    for( int coli = 0; coli < numParams; coli++ )
    {
      ATCb[coli] <<= matrixShift;
    }
  }
  else if( matrixShift < 0 )
  {
    matrixShift = -matrixShift;

    for( int coli0 = 0; coli0 < numParams; coli0++ )
    {
      for( int coli1 = coli0; coli1 < numParams; coli1++ )
      {
        ATA[coli0][coli1] >>= matrixShift;
      }
    }

    for( int coli = 0; coli < numParams; coli++ )
    {
      ATCb[coli] >>= matrixShift;
    }
  }

#if JVET_AC0053_GAUSSIAN_SOLVER
  // Solve the filter coefficients
  gaussElimination(ATA, ATCb, model.params.data(), nullptr, nullptr, numParams, 1, model.bd);
#else
  // Solve the filter coefficients using LDL decomposition
  TE U;       // Upper triangular L' of ATA's LDL decomposition
  Ty diag;    // Diagonal of D

  bool decompOk = ldlDecompose( ATA, U, diag, M );
  ldlSolve( U, diag, ATCb, model.params, M, decompOk );
#endif

#if JVET_AB0174_CCCM_DIV_FREE
  // Add the chroma offset to bias term (after shifting up by CCCM_DECIM_BITS and down by cccmModelCb.bd - 1)
  model.params[numParams - 1] += chromaOffset << (CCCM_DECIM_BITS - (model.bd - 1));
#endif
}

#if JVET_AB0174_CCCM_DIV_FREE
void CccmCovariance::solve2( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* Cb, const Pel* Cr, const int sampleNum, const int chromaOffsetCb, const int chromaOffsetCr, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode
#endif
)
#else
void CccmCovariance::solve2( const Pel A[CCCM_NUM_PARAMS_MAX][CCCM_REF_SAMPLES_MAX], const Pel* Cb, const Pel* Cr, const int sampleNum, CccmModel& modelCb, CccmModel& modelCr
#if JVET_AE0059_INTER_CCCM
    , const bool interCccmMode
#endif
)
#endif
{

  const int numParams = modelCb.getNumParams();

  CHECK( modelCr.getNumParams() != numParams, "Chroma number of parameters don't match" );
  CHECK( CCCM_REF_SAMPLES_MAX < sampleNum, "Insufficient buffer size" );
  CHECK( CCCM_NUM_PARAMS_MAX < numParams, "Insufficient buffer size" );

  // Calculate autocorrelation matrix and cross-correlation vector
  for( int i = 0; i < numParams; i++ )
  {
    memset( ATA[i], 0x00, sizeof( TCccmCoeff ) * numParams );
  }
  memset( ATCb, 0x00, sizeof( TCccmCoeff ) * numParams );
  memset( ATCr, 0x00, sizeof( TCccmCoeff ) * numParams );

  for( int coli0 = 0; coli0 < numParams; coli0++ )
  {
    for( int coli1 = coli0; coli1 < numParams; coli1++ )
    {
      const Pel *col0 = A[coli0];
      const Pel *col1 = A[coli1];

      for( int rowi = 0; rowi < sampleNum; rowi++ )
      {
        ATA[coli0][coli1] += col0[rowi] * col1[rowi];
      }
    }
  }

  for( int coli = 0; coli < numParams; coli++ )
  {
    const Pel *col = A[coli];

    for( int rowi = 0; rowi < sampleNum; rowi++ )
    {
      ATCb[coli] += col[rowi] * Cb[rowi];
      ATCr[coli] += col[rowi] * Cr[rowi];
    }
  }

#if JVET_AB0174_CCCM_DIV_FREE
  // Remove chromaOffset from stats to update cross-correlation
  for( int coli = 0; coli < numParams; coli++ )
  {
    ATCb[coli] = ATCb[coli] - ((ATA[coli][numParams - 1] * chromaOffsetCb) >> (modelCb.bd - 1));
    ATCr[coli] = ATCr[coli] - ((ATA[coli][numParams - 1] * chromaOffsetCr) >> (modelCr.bd - 1));
  }
#endif

  // Scale the matrix and vector to selected dynamic range
  CHECK( modelCb.bd != modelCr.bd, "Bitdepth of Cb and Cr is different" );
#if JVET_AE0059_INTER_CCCM
  int matrixShift = (interCccmMode ? 28 : CCCM_MATRIX_BITS) - 2 * modelCb.bd - ceilLog2( sampleNum );
#else
  int matrixShift = CCCM_MATRIX_BITS - 2 * modelCb.bd - ceilLog2( sampleNum );
#endif

  if( matrixShift > 0 )
  {
    for( int coli0 = 0; coli0 < numParams; coli0++ )
    {
      for( int coli1 = coli0; coli1 < numParams; coli1++ )
      {
        ATA[coli0][coli1] <<= matrixShift;
      }
    }

    for( int coli = 0; coli < numParams; coli++ )
    {
      ATCb[coli] <<= matrixShift;
    }

    for( int coli = 0; coli < numParams; coli++ )
    {
      ATCr[coli] <<= matrixShift;
    }
  }
  else if( matrixShift < 0 )
  {
    matrixShift = -matrixShift;

    for( int coli0 = 0; coli0 < numParams; coli0++ )
    {
      for( int coli1 = coli0; coli1 < numParams; coli1++ )
      {
        ATA[coli0][coli1] >>= matrixShift;
      }
    }

    for( int coli = 0; coli < numParams; coli++ )
    {
      ATCb[coli] >>= matrixShift;
    }

    for( int coli = 0; coli < numParams; coli++ )
    {
      ATCr[coli] >>= matrixShift;
    }
  }

#if JVET_AC0053_GAUSSIAN_SOLVER
  // Solve the filter coefficients
  gaussElimination(ATA, ATCb, modelCb.params.data(), ATCr, modelCr.params.data(), numParams, 2, modelCb.bd
#if JVET_AE0059_INTER_CCCM
    , interCccmMode
#endif
  );
#else
  // Solve the filter coefficients using LDL decomposition
  TE U;       // Upper triangular L' of ATA's LDL decomposition
  Ty diag;    // Diagonal of D

  bool decompOk = ldlDecompose( ATA, U, diag, M );

  ldlSolve( U, diag, ATCb, modelCb.params, M, decompOk );
  ldlSolve( U, diag, ATCr, modelCr.params, M, decompOk );
#endif

#if JVET_AB0174_CCCM_DIV_FREE
  // Add the chroma offset to bias term (after shifting up by CCCM_DECIM_BITS and down by cccmModelCb.bd - 1)
  modelCb.params[numParams - 1] += chromaOffsetCb << (CCCM_DECIM_BITS - (modelCb.bd - 1));
  modelCr.params[numParams - 1] += chromaOffsetCr << (CCCM_DECIM_BITS - (modelCr.bd - 1));
#endif
}
#endif

#if JVET_AB0092_GLM_WITH_LUMA
void IntraPrediction::xGlmApplyModel(const PredictionUnit& pu, const ComponentID compId, const CompArea& chromaArea, CccmModel& glmModel, PelBuf &piPred)
{
  const  ClpRng& clpRng(pu.cu->cs->slice->clpRng(compId));
  Pel* samples = m_samples;

  CPelBuf refLumaBlk = xGlmGetGradPuBuf(pu, chromaArea, 0);
  CPelBuf refGradBlk = xGlmGetGradPuBuf(pu, chromaArea, pu.glmIdc.getIdc(compId, 0));

  for (int y = 0; y < refLumaBlk.height; y++)
  {
    for (int x = 0; x < refLumaBlk.width; x++)
    {
      samples[0] = refGradBlk.at(x, y); // luma gradient
      samples[1] = refLumaBlk.at(x, y); // luma value
      samples[2] = glmModel.bias();

      piPred.at(x, y) = ClipPel<Pel>(glmModel.convolve(samples), clpRng);
    }
  }
}

void IntraPrediction::xGlmCalcModel(const PredictionUnit& pu, const ComponentID compID, const CompArea& chromaArea, CccmModel& glmModel)
{
  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  const CPelBuf reco = pu.cs->picture->getRecoBuf(compID);
  PelBuf        refLuma = xGlmGetGradRefBuf(pu, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, 0);
  PelBuf        refGrad = xGlmGetGradRefBuf(pu, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, pu.glmIdc.getIdc(compID, 0));

  int sampleNum = 0;

#if JVET_AB0174_CCCM_DIV_FREE
  int chromaOffset = 1 << (pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);

  if (refSizeX || refSizeY)
  {
    int refPosX = refSizeX > 0 ? refSizeX - 1 : 0;
    int refPosY = refSizeY > 0 ? refSizeY - 1 : 0;

    chromaOffset = reco.at(refPosPicX + refPosX, refPosPicY + refPosY);
  }
#endif

  int sizeX = refSizeX + chromaArea.width;
  int sizeY = refSizeY + chromaArea.height;

  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if (x >= refSizeX && y >= refSizeY)
      {
        continue;
      }
      if (pu.intraDir[1] == MDLM_L_IDX)
      {
        if (y < refSizeY)
        {
          continue;
        }
      }
      else if (pu.intraDir[1] == MDLM_T_IDX)
      {
        if (x < refSizeX)
        {
          continue;
        }
      }
      else
      {
        if (x >= sizeX || y >= sizeY)
        {
          continue;
        }
      }

      // 7-tap cross
      m_a[0][sampleNum] = refGrad.at(x, y); // luma gradient
      m_a[1][sampleNum] = refLuma.at(x, y); // luma value
      m_a[2][sampleNum] = glmModel.bias();

      m_cb[sampleNum] = reco.at(refPosPicX + x, refPosPicY + y);
      sampleNum++;
    }
  }

  if( !sampleNum ) // Number of sample can go to zero in the multimode case
  {
    glmModel.clearModel();
  }
  else
  {
#if JVET_AB0174_CCCM_DIV_FREE
    m_cccmSolver.solve1( m_a, m_cb, sampleNum, chromaOffset, glmModel );
#else
    m_cccmSolver.solve1( m_a, m_cb, sampleNum, glmModel );
#endif
  }
}

Pel IntraPrediction::xGlmGetGradVal(const PredictionUnit& pu, const int glmIdx, const CPelBuf pi, const int x, const int y) const
{
  const Pel* piSrc = pi.buf;
  const int iRecStride = pi.stride;
  Pel ypval = 0;
  if (glmIdx == 0)
  {
    if (pu.chromaFormat == CHROMA_444)
    {
      ypval = piSrc[x + iRecStride * y];
    }
    else if (pu.chromaFormat == CHROMA_422)
    {
      int s = 2;
      int offLeft = x > 0 ? -1 : 0;
      s += piSrc[2 * x + iRecStride * y] * 2;
      s += piSrc[2 * x + offLeft + iRecStride * y];
      s += piSrc[2 * x + 1 + iRecStride * y];
      ypval = s >> 2;
    }
    else if (pu.cs->sps->getCclmCollocatedChromaFlag())
    {
      int s = 4;
      int offLeft = x > 0 ? -1 : 0;
      int offAbove = y > 0 ? -1 : 0;
      s += piSrc[2 * x + iRecStride * 2 * y] * 4;
      s += piSrc[2 * x + offLeft + iRecStride * 2 * y];
      s += piSrc[2 * x + 1 + iRecStride * 2 * y];
      s += piSrc[2 * x + iRecStride * (2 * y + 1)];
      s += piSrc[2 * x + iRecStride * (2 * y + offAbove)];
      ypval = s >> 3;
    }
    else
    {
      int s = 4;
      int offLeft = x > 0 ? -1 : 0;
      s += piSrc[2 * x + iRecStride * y * 2] * 2;
      s += piSrc[2 * x + offLeft + iRecStride * y * 2];
      s += piSrc[2 * x + 1 + iRecStride * y * 2];
      s += piSrc[2 * x + iRecStride * (y * 2 + 1)] * 2;
      s += piSrc[2 * x + offLeft + iRecStride * (y * 2 + 1)];
      s += piSrc[2 * x + 1 + iRecStride * (y * 2 + 1)];
      ypval = s >> 3;
    }
#if JVET_AB0174_CCCM_DIV_FREE
    ypval -= m_glmLumaOffset;
#endif
  }
  else
  {
    int p = glmIdx > NUM_GLM_PATTERN ? glmIdx - NUM_GLM_PATTERN - 1 : glmIdx - 1;
    int c[6] = { 0 };
    c[0] = g_glmPattern[p][0], c[1] = g_glmPattern[p][1], c[2] = g_glmPattern[p][2];
    c[3] = g_glmPattern[p][3], c[4] = g_glmPattern[p][4], c[5] = g_glmPattern[p][5];

    int offLeft = x > 0 ? -1 : 0;
    int s[6] = { piSrc[2 * x + offLeft + iRecStride * y * 2], piSrc[2 * x + iRecStride * y * 2], piSrc[2 * x + 1 + iRecStride * y * 2],
                 piSrc[2 * x + offLeft + iRecStride * (y * 2 + 1)], piSrc[2 * x + iRecStride * (y * 2 + 1)], piSrc[2 * x + 1 + iRecStride * (y * 2 + 1)] };

    ypval = xGlmGetLumaVal(s, c, p + 1, 0);
  }

  return ypval;
}

void IntraPrediction::xGlmCalcRefArea(const PredictionUnit& pu, CompArea chromaArea)
{
  const ChannelType     chType = CHANNEL_TYPE_CHROMA;
  const CodingUnit&     cu = *pu.cu;
  const CodingStructure &cs = *cu.cs;
  const SPS             &sps = *cs.sps;
  const PreCalcValues   &pcv = *cs.pcv;

  const int tuWidth = chromaArea.width;
  const int tuHeight = chromaArea.height;

  const bool noShift = pcv.noChroma2x2 && chromaArea.width == 4;   // don't shift on the lowest level (chroma not-split)
  const int  compScaleX = getComponentScaleX(chromaArea.compID, sps.getChromaFormatIdc());
  const int  compScaleY = getComponentScaleY(chromaArea.compID, sps.getChromaFormatIdc());
  const int  unitWidth = pcv.minCUWidth >> (noShift ? 0 : compScaleX);
  const int  unitHeight = pcv.minCUHeight >> (noShift ? 0 : compScaleY);

  const int  totalAboveUnits = (2 * tuWidth + (unitWidth - 1)) / unitWidth;
  const int  totalLeftUnits = (2 * tuHeight + (unitHeight - 1)) / unitHeight;
  const int  numAboveUnits = std::max<int>(tuWidth / unitWidth, 1);
  const int  numLeftUnits = std::max<int>(tuHeight / unitHeight, 1);
  const int  numAboveRightUnits = totalAboveUnits - numAboveUnits;
  const int  numLeftBelowUnits = totalLeftUnits - numLeftUnits;

  static bool neighborFlags[4 * MAX_NUM_PART_IDXS_IN_CTU_WIDTH + 1] = { false }; // Just a dummy array here, content not used

  int avaiAboveRightUnits = isAboveRightAvailable(cu, chType, chromaArea.topRight(), numAboveRightUnits, unitWidth, (neighborFlags + totalLeftUnits + 1 + numAboveUnits));
  int avaiLeftBelowUnits = isBelowLeftAvailable(cu, chType, chromaArea.bottomLeft(), numLeftBelowUnits, unitHeight, (neighborFlags + totalLeftUnits - 1 - numLeftUnits));

  int refSizeX, refSizeY;

  PU::getCccmRefLineNum(pu, chromaArea, refSizeX, refSizeY); // Reference lines available left and above

  int refWidth = chromaArea.width + refSizeX;              // Reference buffer size excluding paddings
  int refHeight = chromaArea.height + refSizeY;

  int extWidth = avaiAboveRightUnits * unitWidth;
  int extHeight = avaiLeftBelowUnits * unitHeight;

  refWidth += refSizeY ? extWidth : 0; // Add above right if above is available
  refHeight += refSizeX ? extHeight : 0; // Add below left if left is available

  m_glmRefArea = Area(chromaArea.x - refSizeX, chromaArea.y - refSizeY, refWidth, refHeight); // Position with respect to the PU
}

PelBuf IntraPrediction::xGlmGetGradRefBuf(const PredictionUnit& pu, CompArea chromaArea, int &areaWidth, int &areaHeight, int &refSizeX, int &refSizeY, int &refPosPicX, int &refPosPicY, int glmIdx) const
{
  refSizeX = chromaArea.x - m_glmRefArea.x;                        // Reference lines available left and above
  refSizeY = chromaArea.y - m_glmRefArea.y;
  areaWidth = m_glmRefArea.width;                    // Reference buffer size excluding paddings
  areaHeight = m_glmRefArea.height;
  refPosPicX = m_glmRefArea.x; // Position of the reference area in picture coordinates
  refPosPicY = m_glmRefArea.y;

  int refStride = areaWidth;

  int idx = glmIdx > NUM_GLM_PATTERN ? glmIdx - NUM_GLM_PATTERN : glmIdx;
  return PelBuf(m_glmGradBuf[idx], refStride, areaWidth, areaHeight);
}

PelBuf IntraPrediction::xGlmGetGradPuBuf(const PredictionUnit& pu, CompArea chromaArea, int glmIdx) const
{
  int refSizeX = chromaArea.x - m_glmRefArea.x; // Reference lines available left and above
  int refSizeY = chromaArea.y - m_glmRefArea.y;
  int tuWidth = chromaArea.width;
  int tuHeight = chromaArea.height;
  int refStride = m_glmRefArea.width;
  int refOrigin = refStride * refSizeY + refSizeX;

  int idx = glmIdx > NUM_GLM_PATTERN ? glmIdx - NUM_GLM_PATTERN : glmIdx;
  return PelBuf(m_glmGradBuf[idx] + refOrigin, refStride, tuWidth, tuHeight);
}

void IntraPrediction::xGlmCreateGradRef(const PredictionUnit& pu, CompArea chromaArea
#if JVET_AF0073_INTER_CCP_MERGE
    , bool isTemplate
#endif
)
{
  const CPelBuf recoLuma = pu.cs->picture->getRecoBuf(COMPONENT_Y);
  const int  maxPosPicX = pu.cs->picture->chromaSize().width - 1;
  const int  maxPosPicY = pu.cs->picture->chromaSize().height - 1;

  xGlmCalcRefArea(pu, chromaArea); // Find the reference area

  int areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY;

  PelBuf refLuma = xGlmGetGradRefBuf(pu, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, 0);
  PelBuf refGrad = xGlmGetGradRefBuf(pu, chromaArea, areaWidth, areaHeight, refSizeX, refSizeY, refPosPicX, refPosPicY, pu.glmIdc.getIdc(chromaArea.compID, 0));

  int puBorderX = refSizeX + chromaArea.width;
  int puBorderY = refSizeY + chromaArea.height;

#if JVET_AB0174_CCCM_DIV_FREE
  xGlmSetLumaRefValue(pu, chromaArea);
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  if (!isTemplate)
  {
#endif
  for (int y = 0; y < areaHeight; y++)
  {
    for (int x = 0; x < areaWidth; x++)
    {
      if ((x >= puBorderX && y >= refSizeY) ||
        (y >= puBorderY && x >= refSizeX))
      {
        continue;
      }

      int chromaPosPicX = refPosPicX + x;
      int chromaPosPicY = refPosPicY + y;

      chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
      chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

      refLuma.at(x, y) = xGlmGetGradVal(pu, 0, recoLuma, chromaPosPicX, chromaPosPicY);
      refGrad.at(x, y) = xGlmGetGradVal(pu, pu.glmIdc.getIdc(chromaArea.compID, 0), recoLuma, chromaPosPicX, chromaPosPicY);
    }
  }
#if JVET_AF0073_INTER_CCP_MERGE
  }
  else
  {
    // Generate top template
    if (refSizeY > 0)
    {
      int y = refSizeY - 1;
      for (int x = refSizeX; x < puBorderX; x++)
      {
        int chromaPosPicX = refPosPicX + x;
        int chromaPosPicY = refPosPicY + y;

        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at(x, y) = xGlmGetGradVal(pu, 0, recoLuma, chromaPosPicX, chromaPosPicY);
        refGrad.at(x, y) = xGlmGetGradVal(pu, pu.glmIdc.getIdc(chromaArea.compID, 0), recoLuma, chromaPosPicX, chromaPosPicY);
      }
    }

    // Generate left template
    if (refSizeX > 0)
    {
      int x = refSizeX - 1;
      for (int y = refSizeY; y < puBorderY; y++)
      {
        int chromaPosPicX = refPosPicX + x;
        int chromaPosPicY = refPosPicY + y;

        chromaPosPicX = chromaPosPicX < 0 ? 0 : chromaPosPicX > maxPosPicX ? maxPosPicX : chromaPosPicX;
        chromaPosPicY = chromaPosPicY < 0 ? 0 : chromaPosPicY > maxPosPicY ? maxPosPicY : chromaPosPicY;

        refLuma.at(x, y) = xGlmGetGradVal(pu, 0, recoLuma, chromaPosPicX, chromaPosPicY);
        refGrad.at(x, y) = xGlmGetGradVal(pu, pu.glmIdc.getIdc(chromaArea.compID, 0), recoLuma, chromaPosPicX, chromaPosPicY);
      }
    }
  }
#endif
}

#if JVET_AB0174_CCCM_DIV_FREE
void IntraPrediction::xGlmSetLumaRefValue(const PredictionUnit& pu, CompArea chromaArea)
{
  int lumaPosX = chromaArea.x << getComponentScaleX(chromaArea.compID, pu.cu->chromaFormat);
  int lumaPosY = chromaArea.y << getComponentScaleY(chromaArea.compID, pu.cu->chromaFormat);

  if (lumaPosX || lumaPosY)
  {
    lumaPosX = lumaPosX ? lumaPosX - 1 : 0;
    lumaPosY = lumaPosY ? lumaPosY - 1 : 0;

    m_glmLumaOffset = pu.cs->picture->getRecoBuf(COMPONENT_Y).at(lumaPosX, lumaPosY);
  }
  else
  {
    m_glmLumaOffset = 1 << (pu.cu->slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
  }
}
#endif
#endif

#if JVET_AB0157_TMRL
void IntraPrediction::xPredTmrlIntraDc(const CPelBuf& pSrc, Pel* pDst, int iDstStride)
{
  const Pel dcval = xGetPredValDc(pSrc, Size(tmrlInfo.uiWidth, tmrlInfo.uiHeight));
  for (int y = 0; y < tmrlInfo.uiRefHeight; y++, pDst += iDstStride)
  {
    if (y < tmrlInfo.uiTemplateAbove)
    {
      for (int x = tmrlInfo.uiTemplateLeft; x < tmrlInfo.uiRefWidth; x++)
      {
        pDst[x] = dcval;
      }
    }
    else // y >= tmrlInfo.uiTemplateAbove
    {
      for (int x = 0; x < tmrlInfo.uiTemplateLeft; x++)
      {
        pDst[x] = dcval;
      }
    }
  }
}

Pel tmrlFiltering(Pel* pSrc, const int deltaFrac)
{
#if JVET_AD0085_TMRL_EXTENSION
  const TFilterCoeff* const f = InterpolationFilter::getExtIntraCubicFilter(deltaFrac);
  int val = 0;
  for (int i = 0; i < 4; i++)
  {
    val += pSrc[i] * f[i];
  }
  val = (val + 128) >> 8;
#else
  const TFilterCoeff IntraCubicFilter[32][4] =
  {
    {  0, 64,  0,  0 },
    { -1, 63,  2,  0 },
    { -2, 62,  4,  0 },
    { -2, 60,  7, -1 },
    { -2, 58, 10, -2 },
    { -3, 57, 12, -2 },
    { -4, 56, 14, -2 },
    { -4, 55, 15, -2 },
    { -4, 54, 16, -2 },
    { -5, 53, 18, -2 },
    { -6, 52, 20, -2 },
    { -6, 49, 24, -3 },
    { -6, 46, 28, -4 },
    { -5, 44, 29, -4 },
    { -4, 42, 30, -4 },
    { -4, 39, 33, -4 },
    { -4, 36, 36, -4 },
    { -4, 33, 39, -4 },
    { -4, 30, 42, -4 },
    { -4, 29, 44, -5 },
    { -4, 28, 46, -6 },
    { -3, 24, 49, -6 },
    { -2, 20, 52, -6 },
    { -2, 18, 53, -5 },
    { -2, 16, 54, -4 },
    { -2, 15, 55, -4 },
    { -2, 14, 56, -4 },
    { -2, 12, 57, -3 },
    { -2, 10, 58, -2 },
    { -1,  7, 60, -2 },
    {  0,  4, 62, -2 },
    {  0,  2, 63, -1 },
  };

  int val = 0;
  for (int i = 0; i < 4; i++)
  {
    val += pSrc[i] * IntraCubicFilter[deltaFrac][i];
  }
  val = (val + 32) >> 6;
#endif
  return Pel(val);
}

void IntraPrediction::xPredTmrlIntraAng(const CPelBuf& pSrc, const ClpRng& clpRng, Pel* pTrueDst, int iDstStride)
{
  uint32_t uiWidth = tmrlInfo.uiWidth;
  uint32_t uiHeight = tmrlInfo.uiHeight;
  uint32_t uiTemplateAbove = tmrlInfo.uiTemplateAbove;
  uint32_t uiTemplateLeft = tmrlInfo.uiTemplateLeft;
  uint32_t uiRefWidth = tmrlInfo.uiRefWidth;
  uint32_t uiRefHeight = tmrlInfo.uiRefHeight;

  const bool bIsModeVer = m_ipaParam.isModeVer;
  int  multiRefIdx = m_ipaParam.multiRefIndex;
  const int  lineOffset = multiRefIdx - TMRL_TPL_SIZE;
  const int  intraPredAngle = m_ipaParam.intraPredAngle;
  const int  absInvAngle = m_ipaParam.absInvAngle;

  Pel* refMain;
  Pel* refSide;

#if JVET_AC0094_REF_SAMPLES_OPT
  Pel * refAbove = tempRefAbove;
  Pel * refLeft  = tempRefLeft;
#else
  Pel  refAbove[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
  Pel  refLeft[2 * MAX_CU_SIZE + 3 + 33 * MAX_REF_LINE_IDX];
#endif

  // Initialize the Main and Left reference array.
  if (intraPredAngle < 0)
  {
    for (int x = 0; x <= uiWidth + 1 + multiRefIdx; x++)
    {
      refAbove[x + uiRefHeight] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= uiHeight + 1 + multiRefIdx; y++)
    {
      refLeft[y + uiRefWidth] = pSrc.at(y, 1);
    }
    refMain = bIsModeVer ? refAbove + uiRefHeight : refLeft + uiRefWidth;
    refSide = bIsModeVer ? refLeft + uiRefWidth : refAbove + uiRefHeight;
    // Extend the Main reference to the left.
    int sizeSide = bIsModeVer ? uiRefHeight : uiRefWidth;
    for (int k = -sizeSide; k <= -1; k++)
    {
      refMain[k] = refSide[std::min((-k * absInvAngle + 256) >> 9, sizeSide)];
    }
  }
  else
  {
    for (int x = 0; x <= m_topRefLength + multiRefIdx; x++)
    {
      refAbove[x] = pSrc.at(x, 0);
    }
    for (int y = 0; y <= m_leftRefLength + multiRefIdx; y++)
    {
      refLeft[y] = pSrc.at(y, 1);
    }

    refMain = bIsModeVer ? refAbove : refLeft;
    refSide = bIsModeVer ? refLeft : refAbove;

    // Extend main reference to right using replication
    const int log2Ratio = floorLog2(uiWidth) - floorLog2(uiHeight);
    const int s = std::max<int>(0, bIsModeVer ? log2Ratio : -log2Ratio);
    const int maxIndex = (multiRefIdx << s) + 2;
    const int refLength = bIsModeVer ? m_topRefLength : m_leftRefLength;
    const Pel val = refMain[refLength + multiRefIdx];
    for (int z = 1; z <= maxIndex; z++)
    {
      refMain[refLength + multiRefIdx + z] = val;
    }
  }

  if (!bIsModeVer)
  {
    std::swap(uiHeight, uiWidth);
    std::swap(uiRefHeight, uiRefWidth);
    std::swap(uiTemplateAbove, uiTemplateLeft);
  }
  const int iAreaSize = MAX_CU_SIZE + TMRL_TPL_SIZE;
  static Pel tempArray[iAreaSize * iAreaSize];
  const int dstStride = iDstStride;
  Pel* pDstBuf = bIsModeVer ? pTrueDst : tempArray;

  // compensate for relative line offset in reference line buffers
  refMain += lineOffset;
  refSide += lineOffset;

  Pel* pDsty = pDstBuf;

  if (intraPredAngle == 0)  // pure vertical or pure horizontal
  {
    for (auto y = 0; y < uiTemplateAbove; y++)
    {
      memcpy(pDsty + y * dstStride + uiTemplateLeft, &refMain[1 + uiTemplateLeft], uiWidth * sizeof(Pel));
    }
    for (auto y = uiTemplateAbove; y < uiRefHeight; y++)
    {
      memcpy(pDsty + y * dstStride, &refMain[1], uiTemplateLeft * sizeof(Pel));
    }
  }
  else
  {
    for (int y = 0, deltaPos = intraPredAngle * (1 + lineOffset); y < uiRefHeight; y++, deltaPos += intraPredAngle, pDsty += dstStride)
    {
      int iStartIdx, iEndIdx;
      if (y < uiTemplateAbove)
      {
        iStartIdx = uiTemplateLeft;
        iEndIdx = uiRefWidth;
      }
      else
      {
        iStartIdx = 0;
        iEndIdx = uiTemplateLeft;
      }
#if JVET_AD0085_TMRL_EXTENSION
      const int deltaInt = deltaPos >> 6;
      const int deltaFract = deltaPos & 63;
      if (!isIntegerSlopeExt(abs(intraPredAngle)))
#else
      const int deltaInt = deltaPos >> 5;
      const int deltaFract = deltaPos & 31;
      if (!isIntegerSlope(abs(intraPredAngle)))
#endif
      {
        CHECK(deltaInt + iStartIdx + lineOffset < -int(uiRefHeight), "over the prepared reference buffer.");
        for (int x = iStartIdx; x < iEndIdx; x++)
        {
          Pel val = tmrlFiltering(&refMain[deltaInt + x], deltaFract);
          pDsty[x] = ClipPel(val, clpRng);   // always clip even though not always needed
        }
      }

      else
      {
        // Just copy the integer samples
        ::memcpy(pDsty + iStartIdx, refMain + deltaInt + 1 + iStartIdx, (iEndIdx - iStartIdx) * sizeof(Pel));
      }
    }
  }

  // Flip the block if this is the horizontal mode
  if (!bIsModeVer)
  {
    for (int y = 0; y < uiRefHeight; y++)
    {
      int iStartIdx, iEndIdx;
      if (y < uiTemplateAbove)
      {
        iStartIdx = uiTemplateLeft;
        iEndIdx = uiRefWidth;
      }
      else
      {
        iStartIdx = 0;
        iEndIdx = uiTemplateLeft;
      }

      for (int x = iStartIdx; x < iEndIdx; x++)
      {
        pTrueDst[x * iDstStride + y] = pDstBuf[y * dstStride + x];
      }
    }
  }
}

void IntraPrediction::predTmrlIntraAng(const PredictionUnit& pu, Pel* pPred, uint32_t uiStride)
{
  const CPelBuf& srcBuf = CPelBuf(m_refBuffer[COMPONENT_Y][0], m_refBufferStride[COMPONENT_Y], 2);
  const ClpRng& clpRng(pu.cu->cs->slice->clpRng(COMPONENT_Y));

  switch (pu.intraDir[0])
  {
  case(DC_IDX):     xPredTmrlIntraDc(srcBuf, pPred, uiStride); break;
  default:          xPredTmrlIntraAng(srcBuf, clpRng, pPred, uiStride); break;
  }
}

void IntraPrediction::initTmrlIntraParams(const PredictionUnit& pu, const CompArea area, const SPS& sps)
{
  const ComponentID compId = area.compID;
  const ChannelType chType = toChannelType(compId);
  const Size& blockSize = Size(pu.cu->blocks[compId].width, pu.cu->blocks[compId].height);
  const int      dirMode = PU::getFinalIntraMode(pu, chType);
#if JVET_AD0085_TMRL_EXTENSION
  const int     predMode = getWideAngleExt(blockSize.width, blockSize.height, dirMode);
  m_ipaParam.isModeVer = predMode >= EXT_DIA_IDX;
  m_ipaParam.multiRefIndex = pu.multiRefIdx;
  m_ipaParam.refFilterFlag = false;
  m_ipaParam.applyPDPC = false;
  const int    intraPredAngleMode = (m_ipaParam.isModeVer) ? predMode - EXT_VER_IDX : -(predMode - EXT_HOR_IDX);
  if (dirMode > DC_IDX)
  {
    int absAng = 0;
    static const int extAngTable[64] = { 0, 1, 2, 3, 4, 5, 6,7, 8, 10, 12, 14, 16, 18, 20, 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 43, 46, 49, 52, 55, 58, 61, 64, 67, 70, 74, 78, 84, 90, 96, 102, 108, 114, 121, 128, 137, 146, 159, 172, 188, 204, 230, 256, 299, 342, 427, 512, 597, 682, 853, 1024, 1536, 2048, 3072 };
    static const int extInvAngTable[64] = { 0, 32768, 16384, 10923, 8192, 6554, 5461, 4681, 4096, 3277, 2731, 2341, 2048, 1820, 1638, 1489, 1365, 1260, 1170, 1092, 1024, 964, 910, 862, 819, 762, 712, 669, 630, 596, 565, 537, 512, 489, 468, 443, 420, 390, 364, 341, 321, 303, 287, 271, 256, 239, 224, 206, 191, 174, 161, 142, 128, 110, 96, 77, 64, 55, 48, 38, 32, 21, 16, 11 }; // (512 * 64) / Angle

    const int     absAngMode = abs(intraPredAngleMode);
    const int     signAng = intraPredAngleMode < 0 ? -1 : 1;
    absAng = extAngTable[absAngMode];

    m_ipaParam.absInvAngle = extInvAngTable[absAngMode];
    m_ipaParam.intraPredAngle = signAng * absAng;
  }
#else
  const int     predMode = getModifiedWideAngle(blockSize.width, blockSize.height, dirMode);
  m_ipaParam.isModeVer = predMode >= DIA_IDX;
  m_ipaParam.multiRefIndex = pu.multiRefIdx;
  m_ipaParam.refFilterFlag = false;
  m_ipaParam.applyPDPC = false;

  const int    intraPredAngleMode = (m_ipaParam.isModeVer) ? (predMode - VER_IDX) : (-(predMode - HOR_IDX));
  int absAng = 0;
  if (dirMode > DC_IDX && dirMode < NUM_LUMA_MODE) // intraPredAngle for directional modes
  {
    static const int angTable[32] = { 0,    1,    2,    3,    4,    6,     8,   10,   12,   14,   16,   18,   20,   23,   26,   29,   32,   35,   39,  45,  51,  57,  64,  73,  86, 102, 128, 171, 256, 341, 512, 1024 };
    static const int invAngTable[32] = {
      0,   16384, 8192, 5461, 4096, 2731, 2048, 1638, 1365, 1170, 1024, 910, 819, 712, 630, 565,
      512, 468,   420,  364,  321,  287,  256,  224,  191,  161,  128,  96,  64,  48,  32,  16
    };   // (512 * 32) / Angle

    const int     absAngMode = abs(intraPredAngleMode);
    const int     signAng = intraPredAngleMode < 0 ? -1 : 1;

    absAng = angTable[absAngMode];
    m_ipaParam.absInvAngle = invAngTable[absAngMode];
    m_ipaParam.intraPredAngle = signAng * absAng;
  }
#endif
}

void IntraPrediction::getTmrlSearchRange(const PredictionUnit& pu, int8_t* tmrlRefList, uint8_t* tmrlIntraList, uint8_t& sizeRef, uint8_t& sizeMode)
{
  CodingUnit& cu = *pu.cu;
  int aboveLines = (cu.block(COMPONENT_Y).y) % ((cu.cs->sps)->getMaxCUWidth());
  sizeRef = 0;

  for (; sizeRef < 5; sizeRef++)
  {
    tmrlRefList[sizeRef] = EXT_REF_LINE_IDX[sizeRef];
    if (EXT_REF_LINE_IDX[sizeRef] >= aboveLines)
    {
      break;
    }
  }

#if JVET_AD0085_TMRL_EXTENSION
  sizeMode = TMRL_MPM_SIZE;
  int numCand = getSpatialIpm(pu, tmrlIntraList, sizeMode
#if JVET_AC0094_REF_SAMPLES_OPT
                            , true
#endif
                            , true
  );
  fillMPMList(pu, tmrlIntraList, sizeMode, numCand, true);
#else
  // intra mode candidates
  sizeMode = 0;
  const CodingStructure& cs = *pu.cs;
  const int maxListSize = TMRL_MPM_SIZE;
  bool includedMode[NUM_LUMA_MODE]{ false };
  includedMode[PLANAR_IDX] = true;

  const CompArea& area = pu.block(COMPONENT_Y);
  const Position posA = area.topRight().offset(0, -1);
  const Position posAR = area.topRight().offset(1, -1);
  const Position posL = area.bottomLeft().offset(-1, 0);
  const Position posLB = area.bottomLeft().offset(-1, 1);
  const Position posAL = area.topLeft().offset(-1, -1);

  // left (Intra)
  const PredictionUnit* puLeft = cs.getPURestricted(posL, pu, pu.chType);
  if (puLeft && CU::isIntra(*puLeft->cu))
  {
    tmrlIntraList[sizeMode] = PU::getIntraDirLuma(*puLeft);
    if (puLeft->cu->timd)
    {
      tmrlIntraList[sizeMode] = MAP131TO67(tmrlIntraList[sizeMode]);
    }
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }
  // above (Intra)
  const PredictionUnit* puAbove = cs.getPURestricted(posA, pu, pu.chType);
  if (puAbove && CU::isIntra(*puAbove->cu))
  {
    tmrlIntraList[sizeMode] = PU::getIntraDirLuma(*puAbove);
    if (puAbove->cu->timd)
    {
      tmrlIntraList[sizeMode] = MAP131TO67(tmrlIntraList[sizeMode]);
    }
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }
  // left (Inter)
#if JVET_AC0112_IBC_CIIP
  if (puLeft && (CU::isInter(*puLeft->cu) || CU::isIBC(*puLeft->cu)))
#else
  if (puLeft && CU::isInter(*puLeft->cu))
#endif
  {
    tmrlIntraList[sizeMode] = puLeft->getIpmInfo(posL);
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }
  // above (Inter)
#if JVET_AC0112_IBC_CIIP
  if (puAbove && (CU::isInter(*puAbove->cu) || CU::isIBC(*puAbove->cu)))
#else
  if (puAbove && CU::isInter(*puAbove->cu))
#endif
  {
    tmrlIntraList[sizeMode] = puAbove->getIpmInfo(posA);
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }

  // above left (Intra)
  const PredictionUnit* puAboveLeft = cs.getPURestricted(posAL, pu, pu.chType);
  if (puAboveLeft && CU::isIntra(*puAboveLeft->cu))
  {
    tmrlIntraList[sizeMode] = PU::getIntraDirLuma(*puAboveLeft);
    if (puAboveLeft->cu->timd)
    {
      tmrlIntraList[sizeMode] = MAP131TO67(tmrlIntraList[sizeMode]);
    }
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }

  // left bottom (Intra)
  const PredictionUnit* puLeftBottom = cs.getPURestricted(posLB, pu, pu.chType);
  if (puLeftBottom && CU::isIntra(*puLeftBottom->cu))
  {
    tmrlIntraList[sizeMode] = PU::getIntraDirLuma(*puLeftBottom);
    if (puLeftBottom->cu->timd)
    {
      tmrlIntraList[sizeMode] = MAP131TO67(tmrlIntraList[sizeMode]);
    }
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }
  // above right (Intra)
  const PredictionUnit* puAboveRight = cs.getPURestricted(posAR, pu, pu.chType);
  if (puAboveRight && CU::isIntra(*puAboveRight->cu))
  {
    tmrlIntraList[sizeMode] = PU::getIntraDirLuma(*puAboveRight);
    if (puAboveRight->cu->timd)
    {
      tmrlIntraList[sizeMode] = MAP131TO67(tmrlIntraList[sizeMode]);
    }
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }

  // above left (Inter)
#if JVET_AC0112_IBC_CIIP
  if (puAboveLeft && (CU::isInter(*puAboveLeft->cu) || CU::isIBC(*puAboveLeft->cu)))
#else
  if (puAboveLeft && CU::isInter(*puAboveLeft->cu))
#endif
  {
    tmrlIntraList[sizeMode] = puAboveLeft->getIpmInfo(posAL);
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }

  // left bottom (Inter)
#if JVET_AC0112_IBC_CIIP
  if (puLeftBottom && (CU::isInter(*puLeftBottom->cu) || CU::isIBC(*puLeftBottom->cu)))
#else
  if (puLeftBottom && CU::isInter(*puLeftBottom->cu))
#endif
  {
    tmrlIntraList[sizeMode] = puLeftBottom->getIpmInfo(posLB);
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }

  // above right (Inter)
#if JVET_AC0112_IBC_CIIP
  if (puAboveRight && (CU::isInter(*puAboveRight->cu) || CU::isIBC(*puAboveRight->cu)))
#else
  if (puAboveRight && CU::isInter(*puAboveRight->cu))
#endif
  {
    tmrlIntraList[sizeMode] = puAboveRight->getIpmInfo(posAR);
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }

#if SECONDARY_MPM
  //adding dimd modes
  if (pu.cu->slice->getSPS()->getUseDimd())
  {
    if (pu.cu->dimdMode != -1)
    {
      tmrlIntraList[sizeMode] = pu.cu->dimdMode;
      if (!includedMode[tmrlIntraList[sizeMode]])
      {
        includedMode[tmrlIntraList[sizeMode++]] = true;
      }
    }

    if (pu.cu->dimdBlendMode[0] != -1)
    {
      tmrlIntraList[sizeMode] = pu.cu->dimdBlendMode[0];
      if (!includedMode[tmrlIntraList[sizeMode]])
      {
        includedMode[tmrlIntraList[sizeMode++]] = true;
      }
    }
  }
#endif

  tmrlIntraList[sizeMode] = DC_IDX;
  if (!includedMode[tmrlIntraList[sizeMode]])
  {
    includedMode[tmrlIntraList[sizeMode++]] = true;
  }

  const int offset = (int)NUM_LUMA_MODE - 6;
  const int mod = offset + 3;
  const int numCands = sizeMode;

  for (int deltaAngular = 0; deltaAngular < 4 && sizeMode < maxListSize; deltaAngular++)
  {
    for (int i = 0; i < numCands && sizeMode < maxListSize; i++)
    {
      if (tmrlIntraList[i] <= DC_IDX)
      {
        continue;
      }

      // try to fill mode - (delta + 1)
      tmrlIntraList[sizeMode] = ((tmrlIntraList[i] + offset - deltaAngular) % mod) + 2;
      if (!includedMode[tmrlIntraList[sizeMode]])
      {
        includedMode[tmrlIntraList[sizeMode++]] = true;
      }

      if (sizeMode >= maxListSize)
      {
        break;
      }

      // try to fill mode + delta + 1
      tmrlIntraList[sizeMode] = ((tmrlIntraList[i] - 1 + deltaAngular) % mod) + 2;
      if (!includedMode[tmrlIntraList[sizeMode]])
      {
        includedMode[tmrlIntraList[sizeMode++]] = true;
      }
    }
  }

  uint8_t mpmDefault[] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, 14, 22, 42, 58, 10, 26,
                            38, 62, 6, 30, 34, 66, 2, 48, 52, 16 };
  for (int idx = 0; sizeMode < maxListSize; idx++)
  {
    tmrlIntraList[sizeMode] = mpmDefault[idx];
    if (!includedMode[tmrlIntraList[sizeMode]])
    {
      includedMode[tmrlIntraList[sizeMode++]] = true;
    }
  }
#endif
}

void IntraPrediction::getTmrlList(CodingUnit& cu)
{
  // step-1. prepare buffers, cost functions, initialize size.
  const CompArea& area = cu.Y();
  PredictionUnit& pu = *cu.firstPU;
  int channelBitDepth = cu.slice->getSPS()->getBitDepth(CHANNEL_TYPE_LUMA);
  SizeType uiWidth = cu.lwidth();
  SizeType uiHeight = cu.lheight();
  tmrlInfo.uiWidth = uiWidth;
  tmrlInfo.uiHeight = uiHeight;
  tmrlInfo.uiTemplateAbove = TMRL_TPL_SIZE;
  tmrlInfo.uiTemplateLeft = TMRL_TPL_SIZE;
  tmrlInfo.uiRefWidth = uiWidth + TMRL_TPL_SIZE;
  tmrlInfo.uiRefHeight = uiHeight + TMRL_TPL_SIZE;
  uint32_t      uiRealW   = tmrlInfo.uiRefWidth + TMRL_TPL_SIZE;
  uint32_t      uiRealH   = tmrlInfo.uiRefHeight + TMRL_TPL_SIZE;
  const UnitArea localUnitArea(pu.chromaFormat, Area(0, 0, uiRealW, uiRealH));
  uint32_t       uiPredStride = m_intraPredBuffer[0].getBuf(localUnitArea.Y()).stride;
  Pel *piPred = m_intraPredBuffer[0].getBuf(localUnitArea.Y()).buf;
  const CodingStructure& cs = *cu.cs;
  Pel* piOrg = cs.picture->getRecoBuf(area).buf;
  int iOrgStride = cs.picture->getRecoBuf(area).stride;
  piOrg -= (tmrlInfo.uiTemplateAbove * iOrgStride + tmrlInfo.uiTemplateLeft);

  DistParam distParamSad[2]; // above, left
  distParamSad[0].applyWeight = false;
  distParamSad[0].useMR = false;
  distParamSad[1].applyWeight = false;
  distParamSad[1].useMR = false;

  m_timdSatdCost->setTimdDistParam(distParamSad[0], piOrg + tmrlInfo.uiTemplateLeft, piPred + tmrlInfo.uiTemplateLeft, iOrgStride, uiPredStride,
    channelBitDepth, COMPONENT_Y, uiWidth, tmrlInfo.uiTemplateAbove, 0, 1, false);
  if (cu.lx())
  {
    m_timdSatdCost->setTimdDistParam(distParamSad[1], piOrg + tmrlInfo.uiTemplateAbove * iOrgStride, piPred + tmrlInfo.uiTemplateAbove * uiPredStride,
      iOrgStride, uiPredStride, channelBitDepth, COMPONENT_Y, tmrlInfo.uiTemplateLeft, uiHeight, 0, 1, false);
  }

  // step-2. define search range.
  int8_t tmrlRefList[MRL_NUM_REF_LINES]{ 0 };
  uint8_t tmrlIntraModeList[NUM_LUMA_MODE]{ 0 };
  uint8_t sizeRef, sizeMode;
  getTmrlSearchRange(pu, tmrlRefList, tmrlIntraModeList, sizeRef, sizeMode);

  // step-3. search and sort
  static_vector<TmrlMode, MRL_LIST_SIZE> uiModeList;
  static_vector<uint64_t, MRL_LIST_SIZE> uiCostList;

  int iBestN = MRL_LIST_SIZE;
  if (!pu.cs->pcv->isEncoder)
  {
    iBestN = pu.cu->tmrlListIdx + 1;
  }

  for (uint8_t refIdx = 0; refIdx < sizeRef; refIdx++)
  {
    setReferenceArrayLengths(area);
    pu.multiRefIdx = tmrlRefList[refIdx];
    CHECK(!pu.multiRefIdx, "mrl idx shall not be 0.");
    xFillReferenceSamples(cs.picture->getRecoBuf(area), m_refBuffer[COMPONENT_Y][0], area, cu);

    for (uint8_t modeIdx = 0; modeIdx < sizeMode; modeIdx++)
    {
      pu.intraDir[0] = tmrlIntraModeList[modeIdx];
      initTmrlIntraParams(pu, pu.Y(), *(pu.cs->sps));
      predTmrlIntraAng(pu, piPred, uiPredStride);

      uint64_t uiCost = 0;
      uiCost += distParamSad[0].distFunc(distParamSad[0]);

      if (uiCostList.size() >= iBestN)
      {
        uint64_t uiCostMax = uiCostList[iBestN - 1];
        if (uiCost > uiCostMax)
        {
          continue;
        }
      }

      if (cu.lx())
      {
        uiCost += distParamSad[1].distFunc(distParamSad[1]);
      }
      updateCandList(TmrlMode(pu.multiRefIdx, pu.intraDir[0]), uiCost, uiModeList, uiCostList, iBestN);
    }
  }

  // step-4. fill the list
  for (auto i = 0; i < uiModeList.size(); i++)
  {
    m_tmrlList[i] = uiModeList[i];
  }
}
#endif
//! \}
