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

/** \file     DecSlice.h
    \brief    slice decoder class (header)
*/

#ifndef __DECSLICE__
#define __DECSLICE__

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "CommonLib/CommonDef.h"
#include "CommonLib/BitStream.h"
#include "DecCu.h"
#include "CABACReader.h"
#if JVET_AH0066_JVET_AH0202_CCP_MERGE_LUMACBF0
using namespace std;
#endif

//! \ingroup DecoderLib
//! \{

// ====================================================================================================================
// Class definition
// ====================================================================================================================

/// slice decoder class
class DecSlice
{
private:
  // access channel
  CABACDecoder*   m_CABACDecoder;
  DecCu*          m_pcCuDecoder;

  Ctx             m_entropyCodingSyncContextState;      ///< context storage for state of contexts at the wavefront/WPP/entropy-coding-sync second CTU of tile-row
  PLTBuf          m_palettePredictorSyncState;      /// palette predictor storage at wavefront/WPP
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  std::vector<BinStoreVector> m_binVectors;
#endif

public:
  DecSlice();
  virtual ~DecSlice();

  void  init              ( CABACDecoder* cabacDecoder, DecCu* pcMbDecoder );
#if JVET_AG0117_CABAC_SPATIAL_TUNING
  BinStoreVector* getBinVector ( int id ) { return &m_binVectors[id]; }
  void  create                 ( int width, int iMaxCUWidth );
#else
  void  create            ();
#endif
  void  destroy           ();

  void  decompressSlice   ( Slice* slice, InputBitstream* bitstream, int debugCTU );

#if JVET_AG0098_AMVP_WITH_SBTMVP
  std::map<int, uint32_t> m_amvpSbTmvpArea;
  void clearAmvpSbTmvpStatArea(const Slice* slice)
  {
    if (slice->getPendingRasInit() || slice->isInterGDR())
    {
      m_amvpSbTmvpArea.clear();
    }
  }
  void storeAmvpSbTmvpStatArea(const int Tlayer, const uint32_t enabledArea)
  {
    m_amvpSbTmvpArea[Tlayer] = enabledArea;
  }
  bool loadAmvpSbTmvpStatArea(const int Tlayer, uint32_t& enabledArea)
  {
    if (m_amvpSbTmvpArea.find(Tlayer) != m_amvpSbTmvpArea.end())
    {
      enabledArea = m_amvpSbTmvpArea[Tlayer];
      return true;
    }
    return false;
  }
#endif
};

//! \}

#endif

