/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2020, ITU/ISO/IEC
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

/**
 * \file
 * \brief Implementation of IbcHashMap class
 */

#include "CommonDefX86.h"
#include "../TrQuant.h"

#ifdef TARGET_SIMD_X86

#include <nmmintrin.h>

#if TRANSFORM_SIMD_OPT
template <TransType transType, int transSize>
void TrQuant::fastForwardTransform_SIMD( const TCoeff *block, TCoeff *coeff, int shift, int line, int iSkipLine, int iSkipLine2 )
{
  int i, j, k, sum;
  int rndFactor = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = m_forwardTransformKernels[transType][g_aucLog2[transSize] - 1];
  const int zoRow = line - iSkipLine;
  const int zoCol = transSize - iSkipLine2;

  TCoeff *pCoef;
  if( ( transSize & 0x03 ) == 0 && sizeof( TCoeff ) == 4 && sizeof( TMatrixCoeff ) == 2 )
  {
    if( iSkipLine ) // zeroout true
    {
      // zo represents number of lines to be processed
      // colShift: representes number of cols to processed
      for( i = 0; i < zoRow; i++ )
      {
        pCoef = coeff;
        const TMatrixCoeff *curT = iT;
        for( j = 0; j < zoCol; j++ )
        {
          __m128i tmpSum = _mm_setzero_si128();
          for( k = 0; k < transSize; k += 4 )
          {
            __m128i tmpBlk = _mm_loadu_si128( ( __m128i * )( block + k ) );
            __m128i tmpT = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curT + k ) ) );
            tmpSum = _mm_add_epi32( tmpSum, _mm_mullo_epi32( tmpBlk, tmpT ) );
          }
          tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
          tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
          sum = _mm_cvtsi128_si32( tmpSum );
          pCoef[i] = ( sum + rndFactor ) >> shift;
          pCoef += line;
          curT += transSize;
        }
        block += transSize;
      }
    }
    else
    {
      if( ( transType == 0 ) && ( ( transSize & 0xF ) == 0 ) )
      {
        for( i = 0; i < line; i++ )
        {
          TCoeff    splitBlockE[2048];
          const int halfSize = transSize >> 1;
          const int quarterSize = transSize >> 2;
          TCoeff  * splitBlockO = splitBlockE + halfSize;
          TCoeff  * splitBlockF = splitBlockE + quarterSize;
          TCoeff  * splitBlockG = splitBlockE + transSize;

          for( int m = 0, n = transSize - 1; m < n; m++, n-- )
          {
            splitBlockG[m] = block[m] + block[n];
            splitBlockO[m] = block[m] - block[n];
          }
          for( int m = 0, n = halfSize - 1; m < n; m++, n-- )
          {
            splitBlockE[m] = splitBlockG[m] + splitBlockG[n];
            splitBlockF[m] = splitBlockG[m] - splitBlockG[n];
          }

          pCoef = coeff;
          const TMatrixCoeff *curT = iT;
          for( j = 0; j < transSize; j++ )
          {
            const int    arraySize = ( j & 1 ? halfSize : quarterSize );
            const TCoeff * selPart = ( j & 1 ? splitBlockO : ( j & 2 ? splitBlockF : splitBlockE ) );
            __m128i tmpSum = _mm_setzero_si128();

            for( k = 0; k < arraySize; k += 4 )
            {
              __m128i tmpBlk = _mm_loadu_si128( ( __m128i * )( selPart + k ) );
              __m128i tmpT = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curT + k ) ) );
              tmpSum = _mm_add_epi32( tmpSum, _mm_mullo_epi32( tmpBlk, tmpT ) );
            }

            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
            sum = _mm_cvtsi128_si32( tmpSum );
            pCoef[i] = ( sum + rndFactor ) >> shift;
            pCoef += line;
            curT += transSize;
          }
          block += transSize;
        }
      }
      else
      {
        for( i = 0; i < line; i++ )
        {
          pCoef = coeff;
          const TMatrixCoeff *curT = iT;
          for( j = 0; j < transSize; j++ )
          {
            __m128i tmpSum = _mm_setzero_si128();
            for( k = 0; k < transSize; k += 4 )
            {
              __m128i tmpBlk = _mm_loadu_si128( ( __m128i * )( block + k ) );
              __m128i tmpT = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curT + k ) ) );
              tmpSum = _mm_add_epi32( tmpSum, _mm_mullo_epi32( tmpBlk, tmpT ) );
            }
            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 2, 3, 0, 1 ) ) );
            tmpSum = _mm_add_epi32( tmpSum, _mm_shuffle_epi32( tmpSum, _MM_SHUFFLE( 1, 0, 3, 2 ) ) );
            sum = _mm_cvtsi128_si32( tmpSum );
            pCoef[i] = ( sum + rndFactor ) >> shift;
            pCoef += line;
            curT += transSize;
          }
          block += transSize;
        }
      }
    }
  }
  else
  {
    if( iSkipLine )
    {
      TCoeff *tmp = coeff;
      int     numCoeff = line * transSize; // total number of coefficients
      for( i = 0; i < zoRow; i++ )
      {
        pCoef = coeff;
        const TMatrixCoeff *curT = iT;
        for( j = 0; j < zoCol; j++ )
        {
          sum = 0;
          for( k = 0; k < transSize; k++ )
          {
            sum += block[k] * curT[k];
          }
          pCoef[i] = ( sum + rndFactor ) >> shift;
          pCoef += line;
          curT += transSize;
        }
        block += transSize;
      }
      coeff += zoRow;
      int zeOutNumCoeffCols = line - zoRow;
      if( zoRow )
      {
        for( j = 0; j < zoCol; j++ )
        {
          memset( coeff, 0, sizeof( TCoeff ) * ( zeOutNumCoeffCols ) );
          coeff += line;
        }
      }
      coeff = tmp + line * zoCol;
      int zeOutNumCoeffRows = numCoeff - ( line * zoCol );
      memset( coeff, 0, sizeof( TCoeff ) * zeOutNumCoeffRows );
    }
    else
    {
      for( i = 0; i < line; i++ )
      {
        pCoef = coeff;
        const TMatrixCoeff *curT = iT;
        for( j = 0; j < transSize; j++ )
        {
          sum = 0;
          for( k = 0; k < transSize; k++ )
          {
            sum += block[k] * curT[k];
          }
          pCoef[i] = ( sum + rndFactor ) >> shift;
          pCoef += line;
          curT += transSize;
        }
        block += transSize;
      }
    }
  }
}

template <TransType transType, int transSize>
void TrQuant::fastInverseTransform_SIMD( const TCoeff *coeff, TCoeff *block, int shift, int line, int iSkipLine, int iSkipLine2, const TCoeff outputMinimum, const TCoeff outputMaximum )
{
  int i, j, k;
  int rndFactor = 1 << ( shift - 1 );

  const TMatrixCoeff *iT = m_inverseTransformKernels[transType][g_aucLog2[transSize] - 1];

  const int zoRow = line - iSkipLine;
  const int zoCol = transSize - iSkipLine2;

  if( !( transSize & 0x03 ) && sizeof( TCoeff ) == 4 && sizeof( TMatrixCoeff ) == 2 )
  {
    const __m128i minCoeff = _mm_set1_epi32( outputMinimum );
    const __m128i maxCoeff = _mm_set1_epi32( outputMaximum );
    const __m128i round = _mm_set1_epi32( rndFactor );

    if( transType == DCT2 && !( transSize & 0xF ) )
    {
      for( i = 0; i < line; i++, block += transSize )
      {
        TCoeff    splitBlockE[2048];
        const int halfSize = transSize >> 1;
        const int quarterSize = transSize >> 2;
        TCoeff  * splitBlockO = splitBlockE + halfSize;
        TCoeff  * splitBlockF = splitBlockE + quarterSize;
        TCoeff  * splitBlockG = splitBlockE + transSize;

        memset( splitBlockE, 0, 3 * halfSize * sizeof( TCoeff ) );

        const TCoeff *      curCoeff = coeff;
        const TMatrixCoeff *curIT = iT;
        for( k = 0; k < transSize; k++, curCoeff += line, curIT += transSize )
        {
          const int arraySize = ( k & 1 ? halfSize : quarterSize );
          __m128i    tmpCoeff = _mm_set1_epi32( curCoeff[i] );
          __m128i  * tmpBlock = ( __m128i * )( k & 1 ? splitBlockO : ( k & 2 ? splitBlockF : splitBlockE ) );

          for( j = 0; j < arraySize; j += 4, tmpBlock++ )
          {
            __m128i tmp = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curIT + j ) ) );
            __m128i sum = _mm_loadu_si128( tmpBlock );
            tmp = _mm_mullo_epi32( tmpCoeff, tmp );
            tmp = _mm_add_epi32( tmp, sum );
            _mm_storeu_si128( tmpBlock, tmp );
          }
        }

        for( int m = 0, n = halfSize - 1; m < n; m++, n-- )
        {
          splitBlockG[m] = splitBlockE[m] + splitBlockF[m];
          splitBlockG[n] = splitBlockE[m] - splitBlockF[m];
        }
        for( int m = 0, n = transSize - 1; m < n; m++, n-- )
        {
          block[m] = ( splitBlockG[m] + splitBlockO[m] );
          block[n] = ( splitBlockG[m] - splitBlockO[m] );
        }

        __m128i *tmpBlock = ( __m128i * )block;
        for( j = 0; j < transSize; j += 4, tmpBlock++ )
        {
          __m128i tmp = _mm_loadu_si128( tmpBlock );
          tmp = _mm_srai_epi32( _mm_add_epi32( tmp, round ), shift );
          tmp = _mm_min_epi32( _mm_max_epi32( tmp, minCoeff ), maxCoeff );
          _mm_storeu_si128( tmpBlock, tmp );
        }
      }
    }
    else
    {
      for( i = 0; i < line; i++, block += transSize )
      {
        memset( block, 0, transSize << 2 );
        const TCoeff *      curCoeff = coeff;
        const TMatrixCoeff *curIT = iT;
        for( k = 0; k < transSize; k++, curCoeff += line, curIT += transSize )
        {
          __m128i  tmpCoeff = _mm_set1_epi32( curCoeff[i] );
          __m128i *tmpBlock = ( __m128i * )block;
          for( j = 0; j < transSize; j += 4, tmpBlock++ )
          {
            __m128i tmp = _mm_cvtepi16_epi32( _mm_loadl_epi64( ( __m128i * )( curIT + j ) ) );
            __m128i sum = _mm_loadu_si128( tmpBlock );
            tmp = _mm_mullo_epi32( tmpCoeff, tmp );
            tmp = _mm_add_epi32( tmp, sum );
            _mm_storeu_si128( tmpBlock, tmp );
          }
        }
        __m128i *tmpBlock = ( __m128i * )block;
        for( j = 0; j < transSize; j += 4, tmpBlock++ )
        {
          __m128i tmp = _mm_loadu_si128( tmpBlock );
          tmp = _mm_srai_epi32( _mm_add_epi32( tmp, round ), shift );
          tmp = _mm_min_epi32( _mm_max_epi32( tmp, minCoeff ), maxCoeff );
          _mm_storeu_si128( tmpBlock, tmp );
        }
      }
    }
  }
  else
  {
    for( i = 0; i < zoRow; i++ )
    {
      for( j = 0; j < transSize; j++ )
      {
        int sum = 0;
        for( k = 0; k < zoCol; k++ )
        {
          sum += coeff[k * line + i] * iT[k * transSize + j];
        }
        block[i * transSize + j] = Clip3( outputMinimum, outputMaximum, ( int ) ( sum + rndFactor ) >> shift );
      }
    }

    if( iSkipLine )
    {
      memset( block + ( zoRow*transSize ), 0, ( iSkipLine*transSize ) * sizeof( TCoeff ) );
    }
  }
}
#endif

#if ENABLE_SIMD_SIGN_PREDICTION
template< X86_VEXT vext >
uint32_t computeSAD_SIMD( const Pel* ref, const Pel* cur, const int size )
{
  uint32_t dist = 0;

  if( vext >= AVX2 && ( size & 15 ) == 0 )
  {
#ifdef USE_AVX2
    // Do for width that multiple of 16
    __m256i vzero = _mm256_setzero_si256();
    __m256i vsum16 = vzero;

    for( int i = 0; i < size; i += 16 )
    {
      __m256i vsrc1 = _mm256_lddqu_si256( ( __m256i* )( &ref[i] ) );
      __m256i vsrc2 = _mm256_lddqu_si256( ( __m256i* )( &cur[i] ) );
      vsum16 = _mm256_add_epi16( vsum16, _mm256_abs_epi16( _mm256_sub_epi16( vsrc1, vsrc2 ) ) );
    }
    __m256i vsum32 = _mm256_add_epi32( _mm256_unpacklo_epi16( vsum16, vzero ), _mm256_unpackhi_epi16( vsum16, vzero ) );

    vsum32 = _mm256_hadd_epi32( vsum32, vzero );
    vsum32 = _mm256_hadd_epi32( vsum32, vzero );
    dist = _mm_cvtsi128_si32( _mm256_castsi256_si128( vsum32 ) ) + _mm_cvtsi128_si32( _mm256_castsi256_si128( _mm256_permute2x128_si256( vsum32, vsum32, 0x11 ) ) );
#endif
  }
  else if( ( size & 7 ) == 0 )
  {
    // Do with step of 8
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum16 = vzero;

    for( int i = 0; i < size; i += 8 )
    {
      __m128i vsrc1 = _mm_loadu_si128( ( const __m128i* )( &ref[i] ) );
      __m128i vsrc2 = _mm_lddqu_si128( ( const __m128i* )( &cur[i] ) );
      vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
    }
    __m128i vsum32 = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );

    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    dist = _mm_cvtsi128_si32( vsum32 );
  }
  else
  {
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum16 = vzero;

    CHECK( ( size & 3 ) != 0, "Not divisible by 4: " << size );

    for( int i = 0; i < size; i += 4 )
    {
      __m128i vsrc1 = _mm_loadl_epi64( ( const __m128i* )&ref[i] );
      __m128i vsrc2 = _mm_loadl_epi64( ( const __m128i* )&cur[i] );
      vsum16 = _mm_add_epi16( vsum16, _mm_abs_epi16( _mm_sub_epi16( vsrc1, vsrc2 ) ) );
    }
    __m128i vsum32 = _mm_add_epi32( _mm_unpacklo_epi16( vsum16, vzero ), _mm_unpackhi_epi16( vsum16, vzero ) );

    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    dist = _mm_cvtsi128_si32( vsum32 );
  }
  return dist;
}
#endif

#if ENABLE_SIMD_SIGN_PREDICTION || TRANSFORM_SIMD_OPT
template <X86_VEXT vext>
void TrQuant::_initTrQuantX86()
{
#if ENABLE_SIMD_SIGN_PREDICTION
  m_computeSAD = computeSAD_SIMD<vext>;
#endif

#if TRANSFORM_SIMD_OPT
#if TU_256
  m_forwardTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_FORWARD][0], g_trCoreDCT2P4[TRANSFORM_FORWARD][0], g_trCoreDCT2P8[TRANSFORM_FORWARD][0], g_trCoreDCT2P16[TRANSFORM_FORWARD][0], g_trCoreDCT2P32[TRANSFORM_FORWARD][0], g_trCoreDCT2P64[TRANSFORM_FORWARD][0], g_trCoreDCT2P128[TRANSFORM_FORWARD][0], g_trCoreDCT2P256[0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_FORWARD][0], g_trCoreDCT8P8[TRANSFORM_FORWARD][0], g_trCoreDCT8P16[TRANSFORM_FORWARD][0], g_trCoreDCT8P32[TRANSFORM_FORWARD][0], g_trCoreDCT8P64[TRANSFORM_FORWARD][0], g_trCoreDCT8P128[TRANSFORM_FORWARD][0], g_trCoreDCT8P256[0] },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_FORWARD][0], g_trCoreDST7P8[TRANSFORM_FORWARD][0], g_trCoreDST7P16[TRANSFORM_FORWARD][0], g_trCoreDST7P32[TRANSFORM_FORWARD][0], g_trCoreDST7P64[TRANSFORM_FORWARD][0], g_trCoreDST7P128[TRANSFORM_FORWARD][0], g_trCoreDST7P256[0] },
  } };

  m_inverseTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_INVERSE][0], g_trCoreDCT2P4[TRANSFORM_INVERSE][0], g_trCoreDCT2P8[TRANSFORM_INVERSE][0], g_trCoreDCT2P16[TRANSFORM_INVERSE][0], g_trCoreDCT2P32[TRANSFORM_INVERSE][0], g_trCoreDCT2P64[TRANSFORM_INVERSE][0], g_trCoreDCT2P128[TRANSFORM_INVERSE][0], g_trCoreDCT2P256[0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_INVERSE][0], g_trCoreDCT8P8[TRANSFORM_INVERSE][0], g_trCoreDCT8P16[TRANSFORM_INVERSE][0], g_trCoreDCT8P32[TRANSFORM_INVERSE][0], g_trCoreDCT8P64[TRANSFORM_INVERSE][0], g_trCoreDCT8P128[TRANSFORM_INVERSE][0], g_trCoreDCT8P256[0] },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_INVERSE][0], g_trCoreDST7P8[TRANSFORM_INVERSE][0], g_trCoreDST7P16[TRANSFORM_INVERSE][0], g_trCoreDST7P32[TRANSFORM_INVERSE][0], g_trCoreDST7P64[TRANSFORM_INVERSE][0], g_trCoreDST7P128[TRANSFORM_INVERSE][0], g_trCoreDST7P256[0] },
  } };
  
  fastFwdTrans[0][0] = fastForwardTransform_SIMD<DCT2, 2>;
  fastFwdTrans[0][1] = fastForwardTransform_SIMD<DCT2, 4>;
  fastFwdTrans[0][2] = fastForwardTransform_SIMD<DCT2, 8>;
  fastFwdTrans[0][3] = fastForwardTransform_SIMD<DCT2, 16>;
  fastFwdTrans[0][4] = fastForwardTransform_SIMD<DCT2, 32>;
  fastFwdTrans[0][5] = fastForwardTransform_SIMD<DCT2, 64>;
  fastFwdTrans[0][6] = fastForwardTransform_SIMD<DCT2, 128>;
  fastFwdTrans[0][7] = fastForwardTransform_SIMD<DCT2, 256>;

  fastFwdTrans[1][0] = nullptr;
  fastFwdTrans[1][1] = fastForwardTransform_SIMD<DCT8, 4>;
  fastFwdTrans[1][2] = fastForwardTransform_SIMD<DCT8, 8>;
  fastFwdTrans[1][3] = fastForwardTransform_SIMD<DCT8, 16>;
  fastFwdTrans[1][4] = fastForwardTransform_SIMD<DCT8, 32>;
  fastFwdTrans[1][5] = fastForwardTransform_SIMD<DCT8, 64>;
  fastFwdTrans[1][6] = fastForwardTransform_SIMD<DCT8, 128>;
  fastFwdTrans[1][7] = fastForwardTransform_SIMD<DCT8, 256>;

  fastFwdTrans[2][0] = nullptr;
  fastFwdTrans[2][1] = fastForwardTransform_SIMD<DST7, 4>;
  fastFwdTrans[2][2] = fastForwardTransform_SIMD<DST7, 8>;
  fastFwdTrans[2][3] = fastForwardTransform_SIMD<DST7, 16>;
  fastFwdTrans[2][4] = fastForwardTransform_SIMD<DST7, 32>;
  fastFwdTrans[2][5] = fastForwardTransform_SIMD<DST7, 64>;
  fastFwdTrans[2][6] = fastForwardTransform_SIMD<DST7, 128>;
  fastFwdTrans[2][7] = fastForwardTransform_SIMD<DST7, 256>;

  fastInvTrans[0][0] = fastInverseTransform_SIMD<DCT2, 2>;
  fastInvTrans[0][1] = fastInverseTransform_SIMD<DCT2, 4>;
  fastInvTrans[0][2] = fastInverseTransform_SIMD<DCT2, 8>;
  fastInvTrans[0][3] = fastInverseTransform_SIMD<DCT2, 16>;
  fastInvTrans[0][4] = fastInverseTransform_SIMD<DCT2, 32>;
  fastInvTrans[0][5] = fastInverseTransform_SIMD<DCT2, 64>;
  fastInvTrans[0][6] = fastInverseTransform_SIMD<DCT2, 128>;
  fastInvTrans[0][7] = fastInverseTransform_SIMD<DCT2, 256>;

  fastInvTrans[1][0] = nullptr;
  fastInvTrans[1][1] = fastInverseTransform_SIMD<DCT8, 4>;
  fastInvTrans[1][2] = fastInverseTransform_SIMD<DCT8, 8>;
  fastInvTrans[1][3] = fastInverseTransform_SIMD<DCT8, 16>;
  fastInvTrans[1][4] = fastInverseTransform_SIMD<DCT8, 32>;
  fastInvTrans[1][5] = fastInverseTransform_SIMD<DCT8, 64>;
  fastInvTrans[1][6] = fastInverseTransform_SIMD<DCT8, 128>;
  fastInvTrans[1][7] = fastInverseTransform_SIMD<DCT8, 256>;

  fastInvTrans[2][0] = nullptr;
  fastInvTrans[2][1] = fastInverseTransform_SIMD<DST7, 4>;
  fastInvTrans[2][2] = fastInverseTransform_SIMD<DST7, 8>;
  fastInvTrans[2][3] = fastInverseTransform_SIMD<DST7, 16>;
  fastInvTrans[2][4] = fastInverseTransform_SIMD<DST7, 32>;
  fastInvTrans[2][5] = fastInverseTransform_SIMD<DST7, 64>;
  fastInvTrans[2][6] = fastInverseTransform_SIMD<DST7, 128>;
  fastInvTrans[2][7] = fastInverseTransform_SIMD<DST7, 256>;
#else
  m_forwardTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_FORWARD][0], g_trCoreDCT2P4[TRANSFORM_FORWARD][0], g_trCoreDCT2P8[TRANSFORM_FORWARD][0], g_trCoreDCT2P16[TRANSFORM_FORWARD][0], g_trCoreDCT2P32[TRANSFORM_FORWARD][0], g_trCoreDCT2P64[TRANSFORM_FORWARD][0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_FORWARD][0], g_trCoreDCT8P8[TRANSFORM_FORWARD][0], g_trCoreDCT8P16[TRANSFORM_FORWARD][0], g_trCoreDCT8P32[TRANSFORM_FORWARD][0], nullptr },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_FORWARD][0], g_trCoreDST7P8[TRANSFORM_FORWARD][0], g_trCoreDST7P16[TRANSFORM_FORWARD][0], g_trCoreDST7P32[TRANSFORM_FORWARD][0], nullptr },
  } };

  m_inverseTransformKernels =
  { {
    { g_trCoreDCT2P2[TRANSFORM_INVERSE][0], g_trCoreDCT2P4[TRANSFORM_INVERSE][0], g_trCoreDCT2P8[TRANSFORM_INVERSE][0], g_trCoreDCT2P16[TRANSFORM_INVERSE][0], g_trCoreDCT2P32[TRANSFORM_INVERSE][0], g_trCoreDCT2P64[TRANSFORM_INVERSE][0] },
    { nullptr,                              g_trCoreDCT8P4[TRANSFORM_INVERSE][0], g_trCoreDCT8P8[TRANSFORM_INVERSE][0], g_trCoreDCT8P16[TRANSFORM_INVERSE][0], g_trCoreDCT8P32[TRANSFORM_INVERSE][0], nullptr },
    { nullptr,                              g_trCoreDST7P4[TRANSFORM_INVERSE][0], g_trCoreDST7P8[TRANSFORM_INVERSE][0], g_trCoreDST7P16[TRANSFORM_INVERSE][0], g_trCoreDST7P32[TRANSFORM_INVERSE][0], nullptr },
  } };

  fastFwdTrans[0][0] = fastForwardTransform_SIMD<DCT2, 2>;
  fastFwdTrans[0][1] = fastForwardTransform_SIMD<DCT2, 4>;
  fastFwdTrans[0][2] = fastForwardTransform_SIMD<DCT2, 8>;
  fastFwdTrans[0][3] = fastForwardTransform_SIMD<DCT2, 16>;
  fastFwdTrans[0][4] = fastForwardTransform_SIMD<DCT2, 32>;
  fastFwdTrans[0][5] = fastForwardTransform_SIMD<DCT2, 64>;

  fastFwdTrans[1][0] = nullptr;
  fastFwdTrans[1][1] = fastForwardTransform_SIMD<DCT8, 4>;
  fastFwdTrans[1][2] = fastForwardTransform_SIMD<DCT8, 8>;
  fastFwdTrans[1][3] = fastForwardTransform_SIMD<DCT8, 16>;
  fastFwdTrans[1][4] = fastForwardTransform_SIMD<DCT8, 32>;
  fastFwdTrans[1][5] = fastForwardTransform_SIMD<DCT8, 64>;

  fastFwdTrans[2][0] = nullptr;
  fastFwdTrans[2][1] = fastForwardTransform_SIMD<DST7, 4>;
  fastFwdTrans[2][2] = fastForwardTransform_SIMD<DST7, 8>;
  fastFwdTrans[2][3] = fastForwardTransform_SIMD<DST7, 16>;
  fastFwdTrans[2][4] = fastForwardTransform_SIMD<DST7, 32>;
  fastFwdTrans[2][5] = fastForwardTransform_SIMD<DST7, 64>;

  fastInvTrans[0][0] = fastInverseTransform_SIMD<DCT2, 2>;
  fastInvTrans[0][1] = fastInverseTransform_SIMD<DCT2, 4>;
  fastInvTrans[0][2] = fastInverseTransform_SIMD<DCT2, 8>;
  fastInvTrans[0][3] = fastInverseTransform_SIMD<DCT2, 16>;
  fastInvTrans[0][4] = fastInverseTransform_SIMD<DCT2, 32>;
  fastInvTrans[0][5] = fastInverseTransform_SIMD<DCT2, 64>;

  fastInvTrans[1][0] = nullptr;
  fastInvTrans[1][1] = fastInverseTransform_SIMD<DCT8, 4>;
  fastInvTrans[1][2] = fastInverseTransform_SIMD<DCT8, 8>;
  fastInvTrans[1][3] = fastInverseTransform_SIMD<DCT8, 16>;
  fastInvTrans[1][4] = fastInverseTransform_SIMD<DCT8, 32>;
  fastInvTrans[1][5] = fastInverseTransform_SIMD<DCT8, 64>;

  fastInvTrans[2][0] = nullptr;
  fastInvTrans[2][1] = fastInverseTransform_SIMD<DST7, 4>;
  fastInvTrans[2][2] = fastInverseTransform_SIMD<DST7, 8>;
  fastInvTrans[2][3] = fastInverseTransform_SIMD<DST7, 16>;
  fastInvTrans[2][4] = fastInverseTransform_SIMD<DST7, 32>;
  fastInvTrans[2][5] = fastInverseTransform_SIMD<DST7, 64>;
#endif
#endif
}

template void TrQuant::_initTrQuantX86<SIMDX86>();
#endif

#endif //#ifdef TARGET_SIMD_X86
//! \}
