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

/** \file     Common.h
 *  \brief    Common 2D-geometrical structures
 */

#ifndef __COMMON__
#define __COMMON__

#include "CommonDef.h"

typedef int PosType;
typedef uint32_t SizeType;
struct Position
{
  PosType x;
  PosType y;

  Position()                                   : x(0),  y(0)  { }
  Position(const PosType _x, const PosType _y) : x(_x), y(_y) { }

  bool operator!=(const Position &other)  const { return x != other.x || y != other.y; }
  bool operator==(const Position &other)  const { return x == other.x && y == other.y; }

  Position offset(const Position pos)                 const { return Position(x + pos.x, y + pos.y); }
  Position offset(const PosType _x, const PosType _y) const { return Position(x + _x   , y + _y   ); }
  void     repositionTo(const Position newPos)              { x  = newPos.x; y  = newPos.y; }
  void     relativeTo  (const Position origin)              { x -= origin.x; y -= origin.y; }

  Position operator-( const Position &other )         const { return{ x - other.x, y - other.y }; }
#if JVET_AA0096_MC_BOUNDARY_PADDING || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  PosType getX() { return x; }
  PosType getY() { return y; }
#endif
};
#if JVET_Y0141_SIGN_PRED_IMPROVE
struct PositionWithLevel
{
  uint16_t x;
  uint16_t y;
  TCoeff   level;
};
#endif
struct Size
{
  SizeType width;
  SizeType height;

  Size()                                              : width(0),      height(0)       { }
  Size(const SizeType _width, const SizeType _height) : width(_width), height(_height) { }

  bool operator!=(const Size &other)      const { return (width != other.width) || (height != other.height); }
  bool operator==(const Size &other)      const { return (width == other.width) && (height == other.height); }
  uint32_t area()                             const { return (uint32_t) width * (uint32_t) height; }
#if REUSE_CU_RESULTS_WITH_MULTIPLE_TUS || CONVERT_NUM_TU_SPLITS_TO_CFG
  void resizeTo(const Size newSize)             { width = newSize.width; height = newSize.height; }
#endif
};

struct Area : public Position, public Size
{
  Area()                                                                         : Position(),       Size()       { }
  Area(const Position &_pos, const Size &_size)                                  : Position(_pos),   Size(_size)  { }
  Area(const PosType _x, const PosType _y, const SizeType _w, const SizeType _h) : Position(_x, _y), Size(_w, _h) { }

        Position& pos()                           { return *this; }
  const Position& pos()                     const { return *this; }
        Size&     size()                          { return *this; }
  const Size&     size()                    const { return *this; }

  const Position& topLeft()                 const { return *this; }
        Position  topRight()                const { return { (PosType) (x + width - 1), y                          }; }
        Position  bottomLeft()              const { return { x                        , (PosType) (y + height - 1) }; }
        Position  bottomRight()             const { return { (PosType) (x + width - 1), (PosType) (y + height - 1) }; }
        Position  center()                  const { return { (PosType) (x + width / 2), (PosType) (y + height / 2) }; }

  bool contains(const Position &_pos)       const { return (_pos.x >= x) && (_pos.x < (x + width)) && (_pos.y >= y) && (_pos.y < (y + height)); }
  bool contains(const Area &_area)          const { return contains(_area.pos()) && contains(_area.bottomRight()); }
#if JVET_Z0118_GDR
  bool overlap(const Area &_area) const 
  { 
    Area thisArea = Area(pos(), size());

    if (contains(_area))
    {
      return false;
    }

    if (_area.contains(thisArea))
    {
      return false;
    }

    bool topLeft  = contains(_area.topLeft());
    bool topRight = contains(_area.topRight());
    bool botLeft  = contains(_area.bottomLeft());
    bool botRight = contains(_area.bottomRight());

    int sum = (topLeft ? 1 : 0) + (topRight ? 1 : 0) + (botLeft ? 1 : 0) + (botRight ? 1 : 0);

    if (0 < sum && sum < 4)
    {
      return true;
    }

    return false;
  }
#endif

  bool operator!=(const Area &other)        const { return (Size::operator!=(other)) || (Position::operator!=(other)); }
  bool operator==(const Area &other)        const { return (Size::operator==(other)) && (Position::operator==(other)); }
};

struct UnitScale
{
  UnitScale()                 : posx( 0), posy( 0), area(posx+posy) {}
  UnitScale( int sx, int sy ) : posx(sx), posy(sy), area(posx+posy) {}
  int posx;
  int posy;
  int area;

  template<typename T> T scaleHor ( const T &in ) const { return in >> posx; }
  template<typename T> T scaleVer ( const T &in ) const { return in >> posy; }
  template<typename T> T scaleArea( const T &in ) const { return in >> area; }

  Position scale( const Position &pos  ) const { return { pos.x >> posx, pos.y >> posy }; }
  Size     scale( const Size     &size ) const { return { size.width >> posx, size.height >> posy }; }
  Area     scale( const Area    &_area ) const { return Area( scale( _area.pos() ), scale( _area.size() ) ); }
};
namespace std
{
  template<> struct hash<Position>
  {
    size_t operator()( const Position &value ) const { return ( ( (size_t)value.x << 32 ) + value.y ); }
  };

  template<> struct hash<Size>
  {
    size_t operator()( const Size &value ) const { return ( ( (size_t)value.width << 32 ) + value.height ); }
  };
}

inline ptrdiff_t rsAddr( const Position &pos, const uint32_t stride, const UnitScale &unitScale )
{
  return (ptrdiff_t)( stride >> unitScale.posx ) * (ptrdiff_t)( pos.y >> unitScale.posy )
    + (ptrdiff_t)( pos.x >> unitScale.posx );
}

inline size_t rsAddr(const Position &pos, const Position &origin, const uint32_t stride, const UnitScale &unitScale )
{
  return (stride >> unitScale.posx) * ((pos.y - origin.y) >> unitScale.posy) + ((pos.x - origin.x) >> unitScale.posx);
}

inline ptrdiff_t rsAddr( const Position &pos, const ptrdiff_t stride )
{
  return stride * (ptrdiff_t)pos.y + (ptrdiff_t)pos.x;
}

inline size_t rsAddr(const Position &pos, const Position &origin, const uint32_t stride )
{
  return stride * (pos.y - origin.y) + (pos.x - origin.x);
}

inline Area clipArea(const Area &_area, const Area &boundingBox)
{
  Area area = _area;

  if (area.x + area.width > boundingBox.x + boundingBox.width)
  {
    area.width = boundingBox.x + boundingBox.width - area.x;
  }

  if (area.y + area.height > boundingBox.y + boundingBox.height)
  {
    area.height = boundingBox.y + boundingBox.height - area.y;
  }

  return area;
}


class SizeIndexInfo
{
public:
  SizeIndexInfo(){}
  virtual ~SizeIndexInfo(){}
  SizeType numAllWidths()               { return (SizeType)m_idxToSizeTab.size(); }
  SizeType numAllHeights()              { return (SizeType)m_idxToSizeTab.size(); }
  SizeType numWidths()                  { return (SizeType)m_numBlkSizes; }
  SizeType numHeights()                 { return (SizeType)m_numBlkSizes; }
  SizeType sizeFrom( SizeType idx )     { return m_idxToSizeTab[idx]; }
  SizeType idxFrom( SizeType size )     { CHECKD( m_sizeToIdxTab[size] == std::numeric_limits<SizeType>::max(), "Index of given size does NOT EXIST!" ); return m_sizeToIdxTab[size]; }
  bool     isCuSize( SizeType size )    { return m_isCuSize[size]; }
  virtual void init( SizeType maxSize ) {}

protected:

  void xInit()
  {
    m_isCuSize.resize( m_sizeToIdxTab.size(), false );

    std::vector<SizeType> grpSizes;

    for( int i = 0, n = 0; i < m_sizeToIdxTab.size(); i++ )
    {
      if( m_sizeToIdxTab[i] != std::numeric_limits<SizeType>::max() )
      {
        m_sizeToIdxTab[i] = n;
        m_idxToSizeTab.push_back( i );
        n++;
      }

      if( m_sizeToIdxTab[i] != std::numeric_limits<SizeType>::max() && m_sizeToIdxTab[i >> 1] != std::numeric_limits<SizeType>::max() && i >= 4 )
      {
        m_isCuSize[i] = true;
      }

      // collect group sizes (for coefficient group coding)
      SizeType grpSize = i >> ( ( i & 3 ) != 0 ? 1 : 2 );
      if( m_sizeToIdxTab[i] != std::numeric_limits<SizeType>::max() && m_sizeToIdxTab[grpSize] == std::numeric_limits<SizeType>::max() )
      {
        grpSizes.push_back( grpSize );
      }
    }

    m_numBlkSizes = (SizeType)m_idxToSizeTab.size();

    for( SizeType grpSize : grpSizes )
    {
      if( grpSize > 0 && m_sizeToIdxTab[grpSize] == std::numeric_limits<SizeType>::max() )
      {
        m_sizeToIdxTab[grpSize] = (SizeType)m_idxToSizeTab.size();
        m_idxToSizeTab.push_back( grpSize );
      }
    }
  };

  std::vector<bool    > m_isCuSize;
  int                   m_numBlkSizes; // as opposed to number all sizes, which also contains grouped sizes
  std::vector<SizeType> m_sizeToIdxTab;
  std::vector<SizeType> m_idxToSizeTab;
};

class SizeIndexInfoLog2 : public SizeIndexInfo
{
public:
  SizeIndexInfoLog2(){}
  ~SizeIndexInfoLog2(){};

  void init( SizeType maxSize )
  {
    for( int i = 0, n = 0; i <= maxSize; i++ )
    {
      SizeType val = std::numeric_limits<SizeType>::max();
      if( i == ( 1 << n ) )
      {
        n++;
        val = i;
      }
      m_sizeToIdxTab.push_back( val );
    }
    SizeIndexInfo::xInit();
  }
};

#if JVET_AD0188_CCP_MERGE
enum CCPType
{
  CCP_TYPE_NONE    = 0,
  CCP_TYPE_CCLM    = 1,
  CCP_TYPE_MMLM    = (1 << 1),
  CCP_TYPE_GLM0123 = (1 << 2),
  CCP_TYPE_GLM4567 = (1 << 3),
  CCP_TYPE_CCCM    = (1 << 4),
  CCP_TYPE_GLCCCM  = (1 << 5),
  CCP_TYPE_NSCCCM  = (1 << 6),
  CCP_TYPE_MDFCCCM = (1 << 7),
#if JVET_AF0073_INTER_CCP_MERGE
  CCP_TYPE_INTER_CCCM = (1 << 8),
#endif
  NUM_CCP_TYPE
};

struct CCPModelCandidate
{
  int64_t params[2][NUM_CCP_PARAMS] = { { 0 } };
#if MMLM
  int64_t params2[2][NUM_CCP_PARAMS] = { { 0 } };
  int     shift2[2] = { 0 };
  int     yThres = 0;
#endif
  int     shift[2] = { 0 };
  int     bd = 0;
  int     midVal = 0;
  int     type = 0;
#if JVET_AB0174_CCCM_DIV_FREE
  int     lumaOffset = 0;
#endif
#if JVET_AC0054_GLCCCM || JVET_AD0202_CCCM_MDF
  int     corOffX = 0;
  int     corOffY = 0;
#endif
#if JVET_AD0202_CCCM_MDF
  int     cccmMultiFilterIdx = 0;
#endif
#if JVET_AA0126_GLM
  int8_t  glmIdc = 0;
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  int ccInsideFilter = 0;
#endif

  template<int NUM>
  inline bool isTheSameParams(const CCPModelCandidate& p) const
  {
    for (int i = 0; i < NUM; ++i)
    {
      if (params[0][i] != p.params[0][i] || params[1][i] != p.params[1][i])
      {
        return false;
      }
    }
    return true;
  }

  template<int NUM>
  inline bool isTheSameParams2(const CCPModelCandidate& p) const
  {
    for (int i = 0; i < NUM; ++i)
    {
      if (params2[0][i] != p.params2[0][i] || params2[1][i] != p.params2[1][i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator==(const CCPModelCandidate& cand) const
  {
    if (type != cand.type)
    {
      return false;
    }

    if ((type & CCP_TYPE_MMLM) != (cand.type & CCP_TYPE_MMLM))
    {
      return false;
    }

    if (type & (CCP_TYPE_CCCM | CCP_TYPE_GLCCCM))
    {
#if JVET_AB0174_CCCM_DIV_FREE
      if (lumaOffset != cand.lumaOffset)
      {
        return false;
      }
#endif
      if (type & CCP_TYPE_MMLM)
      {
        if (yThres != cand.yThres)
        {
          return false;
        }
        if (isTheSameParams<CCCM_NUM_PARAMS>(cand) && isTheSameParams2<CCCM_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      else
      {
        if (isTheSameParams<CCCM_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      return false;
    }
    else if (type & (CCP_TYPE_CCLM | CCP_TYPE_GLM0123))
    {
      if (type & CCP_TYPE_MMLM)
      {
        if (yThres != cand.yThres)
        {
          return false;
        }
        if (params[0][0] == cand.params[0][0] && shift[0] == cand.shift[0]
          && params[1][0] == cand.params[1][0] && shift[1] == cand.shift[1]
          && params2[0][0] == cand.params2[0][0] && shift2[0] == cand.shift2[0]
          && params2[1][0] == cand.params2[1][0] && shift2[1] == cand.shift2[1])
        {
          return true;
        }
      }
      else
      {
        if (params[0][0] == cand.params[0][0] && shift[0] == cand.shift[0]
          && params[1][0] == cand.params[1][0] && shift[1] == cand.shift[1])
        {
          return true;
        }
      }
      return false;
    }
    else if (type & CCP_TYPE_GLM4567)
    {
#if JVET_AB0174_CCCM_DIV_FREE
      if (lumaOffset != cand.lumaOffset)
      {
        return false;
      }
#endif
      if (type & CCP_TYPE_MMLM)
      {
        if (yThres != cand.yThres)
        {
          return false;
        }
        if (isTheSameParams<GLM_NUM_PARAMS>(cand) && isTheSameParams2<GLM_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      else
      {
        if (isTheSameParams<GLM_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      return false;
    }
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
    else if (type & CCP_TYPE_NSCCCM)
    {
#if JVET_AB0174_CCCM_DIV_FREE
      if (lumaOffset != cand.lumaOffset)
      {
        return false;
      }
#endif
      if (type & CCP_TYPE_MMLM)
      {
        if (yThres != cand.yThres)
        {
          return false;
        }
        if (isTheSameParams<CCCM_NO_SUB_NUM_PARAMS>(cand) && isTheSameParams2<CCCM_NO_SUB_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      else
      {
        if (isTheSameParams<CCCM_NO_SUB_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      return false;
    }
#endif
#if JVET_AD0202_CCCM_MDF
    else if (type & CCP_TYPE_MDFCCCM)
    {
      if (cccmMultiFilterIdx != cand.cccmMultiFilterIdx)
      {
        return false;
      }
#if JVET_AB0174_CCCM_DIV_FREE
      if (lumaOffset != cand.lumaOffset)
      {
        return false;
      }
#endif
      if (type & CCP_TYPE_MMLM)
      {
        if (yThres != cand.yThres)
        {
          return false;
        }
        if (cccmMultiFilterIdx == 1)
        {
          if (isTheSameParams<CCCM_MULTI_PRED_FILTER_NUM_PARAMS>(cand) && isTheSameParams2<CCCM_MULTI_PRED_FILTER_NUM_PARAMS>(cand))
          {
            return true;
          }
        }
        else
        {
          if (isTheSameParams<CCCM_MULTI_PRED_FILTER_NUM_PARAMS2>(cand) && isTheSameParams2<CCCM_MULTI_PRED_FILTER_NUM_PARAMS2>(cand))
          {
            return true;
          }
        }
      }
      else
      {
        if (cccmMultiFilterIdx == 1)
        {
          if (isTheSameParams<CCCM_MULTI_PRED_FILTER_NUM_PARAMS>(cand))
          {
            return true;
          }
        }
        else
        {
          if (isTheSameParams<CCCM_MULTI_PRED_FILTER_NUM_PARAMS2>(cand))
          {
            return true;
          }
        }
      }
      return false;
    }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
    else if (type & CCP_TYPE_INTER_CCCM)
    {
#if JVET_AB0174_CCCM_DIV_FREE
      if (lumaOffset != cand.lumaOffset)
      {
        return false;
      }
#endif
      if (type & CCP_TYPE_MMLM)
      {
        if (yThres != cand.yThres)
        {
          return false;
        }
        if (isTheSameParams<INTER_CCCM_NUM_PARAMS>(cand) && isTheSameParams2<INTER_CCCM_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      else
      {
        if (isTheSameParams<INTER_CCCM_NUM_PARAMS>(cand))
        {
          return true;
        }
      }
      return false;
    }
#endif
    else
    {
      THROW("Wrong Type");
      // return pos == cand.pos;
    }
  }
};

struct LutCCP
{
#if JVET_Z0118_GDR  
  static_vector<CCPModelCandidate, MAX_NUM_HCCP_CANDS> lutCCP0;
  static_vector<CCPModelCandidate, MAX_NUM_HCCP_CANDS> lutCCP1;
#else
  static_vector<CCPModelCandidate, MAX_NUM_HCCP_CANDS> lutCCP;
#endif
  // Postions for future extensions
};
#endif
#if JVET_AG0058_EIP
struct EipModelCandidate
{
  int64_t params[EIP_FILTER_TAP] = { 0 };
  int     filterShape            = 0;
  int     eipDimdMode            = PLANAR_IDX;

  inline bool isTheSameParams(const EipModelCandidate& p) const
  {
    if (filterShape != p.filterShape) 
    {
      return false;
    }
    for (int i = 0; i < EIP_FILTER_TAP; ++i)
    {
      if (params[i] != p.params[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator==(const EipModelCandidate &cand) const 
  { 
    if (isTheSameParams(cand))
    {
      return true;
    }
    return false;
  }
};

#if JVET_AG0058_EIP
struct LutEIP
{
#if JVET_Z0118_GDR  
  static_vector<EipModelCandidate, MAX_NUM_HEIP_CANDS> lutEip0;
  static_vector<EipModelCandidate, MAX_NUM_HEIP_CANDS> lutEip1;
#else
  static_vector<CccmModel, MAX_NUM_HEIP_CANDS> lutEip;
#endif
  // Postions for future extensions
};
#endif
#endif

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
struct DecoderDerivedCcpCandidate
{
  CCPModelCandidate ddccpCand;
  int    cost;           // TM cost 
  int    lmIndex;        // LM_CHROMA_IDX, MMLM_CHROMA_IDX, MDLM_L_IDX, MDLM_T_IDX, MMLM_L_IDX, MMLM_T_IDX
  int    isCccm;         // 1: LM_CHROMA_IDX, MMLM_CHROMA_IDX; 2:MDLM_L_IDX, MMLM_L_IDX; 3: MDLM_T_IDX, MMLM_T_IDX
  int    isGlcccm;       // 1: if glCccmFlag true
  int    isInsideFilter; // 1: if LBCCP true
  bool   isFusion;       // enable two CCP fusion
};
#endif

#if JVET_AG0276_NLIC
struct AltLMInterUnit
{
  int scale[MAX_NUM_COMPONENT];
  int offset[MAX_NUM_COMPONENT];

  void resetAltLinearModel()
  {
    for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
      scale[comp] = 32;
      offset[comp] = 0;
    }
  }

  AltLMInterUnit()
  {
    resetAltLinearModel();
  }

  AltLMInterUnit &operator=(const AltLMInterUnit &other)
  {
    for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
    {
      scale[comp] = other.scale[comp];
      offset[comp] = other.offset[comp];
    }
    return *this;
  }
};
#endif
#endif
