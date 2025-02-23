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

/** \file     EncTemporalFilter.cpp
\brief    EncTemporalFilter class
*/

#include "EncTemporalFilter.h"
#include "Utilities/VideoIOYuv.h"
#include <math.h>


// ====================================================================================================================
// Constructor / destructor / initialization / destroy
// ====================================================================================================================

const double EncTemporalFilter::m_chromaFactor    =  0.55;
const double EncTemporalFilter::m_sigmaMultiplier =  9.0;
const double EncTemporalFilter::m_sigmaZeroPoint  = 10.0;
const int EncTemporalFilter::m_motionVectorFactor = 16;
const int EncTemporalFilter::m_padding = 128;
const int EncTemporalFilter::m_interpolationFilter[16][8] =
{
  {   0,   0,   0,  64,   0,   0,   0,   0 },   //0
  {   0,   1,  -3,  64,   4,  -2,   0,   0 },   //1 -->-->
  {   0,   1,  -6,  62,   9,  -3,   1,   0 },   //2 -->
  {   0,   2,  -8,  60,  14,  -5,   1,   0 },   //3 -->-->
  {   0,   2,  -9,  57,  19,  -7,   2,   0 },   //4
  {   0,   3, -10,  53,  24,  -8,   2,   0 },   //5 -->-->
  {   0,   3, -11,  50,  29,  -9,   2,   0 },   //6 -->
  {   0,   3, -11,  44,  35, -10,   3,   0 },   //7 -->-->
  {   0,   1,  -7,  38,  38,  -7,   1,   0 },   //8
  {   0,   3, -10,  35,  44, -11,   3,   0 },   //9 -->-->
  {   0,   2,  -9,  29,  50, -11,   3,   0 },   //10-->
  {   0,   2,  -8,  24,  53, -10,   3,   0 },   //11-->-->
  {   0,   2,  -7,  19,  57,  -9,   2,   0 },   //12
  {   0,   1,  -5,  14,  60,  -8,   2,   0 },   //13-->-->
  {   0,   1,  -3,   9,  62,  -6,   1,   0 },   //14-->
  {   0,   0,  -2,   4,  64,  -3,   1,   0 }    //15-->-->
};

const double EncTemporalFilter::m_refStrengths[2][4] =
{
  // abs(POC offset)
  //  1,    2     3     4
  { 0.85, 0.57, 0.41, 0.33 },   // random access
  { 1.13, 0.97, 0.81, 0.57 },   // low delay
};

#if JVET_Y0240_BIM
const int EncTemporalFilter::m_cuTreeThresh[4] =
  { 75, 60, 30, 15 };
#endif

EncTemporalFilter::EncTemporalFilter() :
  m_FrameSkip(0),
  m_chromaFormatIDC(NUM_CHROMA_FORMAT),
  m_sourceWidth(0),
  m_sourceHeight(0),
  m_QP(0),
  m_clipInputVideoToRec709Range(false),
  m_inputColourSpaceConvert(NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS)
{}

void EncTemporalFilter::init( const int frameSkip, const int inputBitDepth[MAX_NUM_CHANNEL_TYPE],
                              const int msbExtendedBitDepth[MAX_NUM_CHANNEL_TYPE],
                              const int internalBitDepth[MAX_NUM_CHANNEL_TYPE], const int width, const int height,
                              const int *pad, const bool rec709, const std::string &filename,
                              const ChromaFormat inputChromaFormatIDC, const InputColourSpaceConversion colorSpaceConv,
                              const int qp, const std::map<int, double> &temporalFilterStrengths, const int pastRefs,
                              const int futureRefs, const int firstValidFrame, const int lastValidFrame
#if JVET_Y0240_BIM
                              , const bool mctfEnabled, std::map<int, int*> *adaptQPmap, const bool bimEnabled, const int ctuSize
#endif
                              )
{
  m_FrameSkip = frameSkip;
  for (int i = 0; i < MAX_NUM_CHANNEL_TYPE; i++)
  {
    m_inputBitDepth[i]       = inputBitDepth[i];
    m_MSBExtendedBitDepth[i] = msbExtendedBitDepth[i];
    m_internalBitDepth[i]    = internalBitDepth[i];
  }

  m_sourceWidth  = width;
  m_sourceHeight = height;
  for (int i = 0; i < 2; i++)
  {
    m_pad[i] = pad[i];
  }
  m_clipInputVideoToRec709Range = rec709;
  m_inputFileName   = filename;
  m_chromaFormatIDC = inputChromaFormatIDC;
  m_inputColourSpaceConvert = colorSpaceConv;
  m_area = Area(0, 0, width, height);
  m_QP   = qp;
  m_temporalFilterStrengths = temporalFilterStrengths;

  m_pastRefs = pastRefs;
  m_futureRefs = futureRefs;
  m_firstValidFrame = firstValidFrame;
  m_lastValidFrame = lastValidFrame;
#if JVET_Y0240_BIM
  m_mctfEnabled = mctfEnabled;
  m_bimEnabled = bimEnabled;
  m_numCtu = ((width + ctuSize - 1) / ctuSize) * ((height + ctuSize - 1) / ctuSize);
  m_ctuSize = ctuSize;
  m_ctuAdaptedQP = adaptQPmap;
#endif
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

bool EncTemporalFilter::filter(PelStorage &orgPic, int receivedPoc)
{
  bool isFilterThisFrame = false;
  if (m_QP >= 17)  // disable filter for QP < 17
  {
    for (std::map<int, double>::iterator it = m_temporalFilterStrengths.begin(); it != m_temporalFilterStrengths.end(); ++it)
    {
      int filteredFrame = it->first;
      if (receivedPoc % filteredFrame == 0)
      {
        isFilterThisFrame = true;
        break;
      }
    }
  }

  if (isFilterThisFrame)
  {
    const int  currentFilePoc = receivedPoc + m_FrameSkip;
    const int  firstFrame = std::max( currentFilePoc - m_pastRefs, m_firstValidFrame );
    const int  lastFrame = std::min( currentFilePoc + m_futureRefs, m_lastValidFrame );
    VideoIOYuv yuvFrames;
    yuvFrames.open( m_inputFileName, false, m_inputBitDepth, m_MSBExtendedBitDepth, m_internalBitDepth );
    yuvFrames.skipFrames( firstFrame, m_sourceWidth - m_pad[0], m_sourceHeight - m_pad[1], m_chromaFormatIDC );

    std::deque<TemporalFilterSourcePicInfo> srcFrameInfo;

    // subsample original picture so it only needs to be done once
    PelStorage origPadded;

    origPadded.create(m_chromaFormatIDC, m_area, 0, m_padding);
    origPadded.copyFrom(orgPic);
    origPadded.extendBorderPel(m_padding, m_padding);

    PelStorage origSubsampled2;
    PelStorage origSubsampled4;

    subsampleLuma(origPadded, origSubsampled2);
    subsampleLuma(origSubsampled2, origSubsampled4);

    // determine motion vectors
    for (int poc = firstFrame; poc <= lastFrame; poc++)
    {
      if( poc == currentFilePoc )
      { // hop over frame that will be filtered
        yuvFrames.skipFrames(1, m_sourceWidth - m_pad[0], m_sourceHeight - m_pad[1], m_chromaFormatIDC);
        continue;
      }
      srcFrameInfo.push_back(TemporalFilterSourcePicInfo());
      TemporalFilterSourcePicInfo &srcPic = srcFrameInfo.back();

      PelStorage dummyPicBufferTO; // Only used temporary in yuvFrames.read
      srcPic.picBuffer.create(m_chromaFormatIDC, m_area, 0, m_padding);
      dummyPicBufferTO.create(m_chromaFormatIDC, m_area, 0, m_padding);
      if (!yuvFrames.read(srcPic.picBuffer, dummyPicBufferTO, m_inputColourSpaceConvert, m_pad, m_chromaFormatIDC, m_clipInputVideoToRec709Range))
      {
        // eof or read fail
        srcPic.picBuffer.destroy();
        srcFrameInfo.pop_back();
        break;
      }
      srcPic.picBuffer.extendBorderPel(m_padding, m_padding);
      srcPic.mvs.allocate(m_sourceWidth / 4, m_sourceHeight / 4);

      motionEstimation(srcPic.mvs, origPadded, srcPic.picBuffer, origSubsampled2, origSubsampled4);
      applyMotion(srcPic.mvs, srcPic.picBuffer, dummyPicBufferTO);
      srcPic.picBuffer.swap(dummyPicBufferTO);
      srcPic.origOffset = poc - currentFilePoc;
    }

    const int numRefs = int(srcFrameInfo.size());
    if (numRefs == 0)
    {
      yuvFrames.close();
      return false;
    }

    // filter
    PelStorage newOrgPic;
    newOrgPic.create(m_chromaFormatIDC, m_area, 0, m_padding);
    double overallStrength = -1.0;
    for (map<int, double>::iterator it = m_temporalFilterStrengths.begin(); it != m_temporalFilterStrengths.end(); ++it)
    {
      int frame = it->first;
      double strength = it->second;
      if (receivedPoc % frame == 0)
      {
        overallStrength = strength;
      }
    }
#if JVET_Y0240_BIM
    if ( m_bimEnabled )
    {
      const int bimFirstFrame = std::max(currentFilePoc - 2, firstFrame);
      const int bimLastFrame  = std::min(currentFilePoc + 2, lastFrame);
      std::vector<double> sumError(m_numCtu * 2, 0);
      std::vector<int>    blkCount(m_numCtu * 2, 0);

      int frameIndex = bimFirstFrame - firstFrame;

      int distFactor[2] = {3,3};

      int* qpMap = new int[m_numCtu];
      for (int poc = bimFirstFrame; poc <= bimLastFrame; poc++)
      {
        if ((poc < 0) || (poc == currentFilePoc) || (frameIndex >= numRefs))
        {
          continue; // frame not available or frame that is being filtered
        }
        int dist = abs(poc - currentFilePoc) - 1;
        distFactor[dist]--;
        TemporalFilterSourcePicInfo &srcPic = srcFrameInfo.at(frameIndex);
        for (int y = 0; y < srcPic.mvs.h() / 2; y++) // going over in 8x8 block steps
        {
          for (int x = 0; x < srcPic.mvs.w() / 2; x++)
          {
            int blocksPerRow = (srcPic.mvs.w() / 2 + (m_ctuSize / 8 - 1)) / (m_ctuSize / 8);
            int ctuX = x / (m_ctuSize / 8);
            int ctuY = y / (m_ctuSize / 8);
            int ctuId = ctuY * blocksPerRow + ctuX;
            sumError[dist * m_numCtu + ctuId] += srcPic.mvs.get(x, y).error;
            blkCount[dist * m_numCtu + ctuId] += 1;
          }
        }
        frameIndex++;
      }
      double weight = (receivedPoc % 16) ? 0.6 : 1;
      const double center = 45.0;
      for (int i = 0; i < m_numCtu; i++)
      {
        int avgErrD1 = (int)((sumError[i] / blkCount[i]) * distFactor[0]);
        int avgErrD2 = (int)((sumError[i + m_numCtu] / blkCount[i + m_numCtu]) * distFactor[1]);
        int weightedErr = std::max(avgErrD1, avgErrD2) + abs(avgErrD2 - avgErrD1) * 3;
        weightedErr = (int)(weightedErr * weight + (1 - weight) * center);
        if (weightedErr > m_cuTreeThresh[0])
        {
          qpMap[i] = 2;
        }
        else if (weightedErr > m_cuTreeThresh[1])
        {
          qpMap[i] = 1;
        }
        else if (weightedErr < m_cuTreeThresh[3])
        {
          qpMap[i] = -2;
        }
        else if (weightedErr < m_cuTreeThresh[2])
        {
          qpMap[i] = -1;
        }
        else
        {
          qpMap[i] = 0;
        }
      }
      m_ctuAdaptedQP->insert({ receivedPoc, qpMap });
    }
#endif

#if JVET_Y0240_BIM
    if ( m_mctfEnabled )
#endif
    {
      bilateralFilter(origPadded, srcFrameInfo, newOrgPic, overallStrength);

      // move filtered to orgPic
      orgPic.copyFrom(newOrgPic);
    }

    yuvFrames.close();
    return true;
  }
  return false;
}

// ====================================================================================================================
// Private member functions
// ====================================================================================================================

void EncTemporalFilter::subsampleLuma(const PelStorage &input, PelStorage &output, const int factor) const
{
  const int newWidth  = input.Y().width  / factor;
  const int newHeight = input.Y().height / factor;
  output.create(m_chromaFormatIDC, Area(0, 0, newWidth, newHeight), 0, m_padding);

  const Pel* srcRow   = input.Y().buf;
  const int srcStride = input.Y().stride;
  Pel *dstRow         = output.Y().buf;
  const int dstStride = output.Y().stride;

  for (int y = 0; y < newHeight; y++, srcRow += factor * srcStride, dstRow += dstStride)
  {
    const Pel *inRow      = srcRow;
    const Pel *inRowBelow = srcRow + srcStride;
    Pel *target           = dstRow;

    for (int x = 0; x < newWidth; x++)
    {
      target[x] = (inRow[0] + inRowBelow[0] + inRow[1] + inRowBelow[1] + 2) >> 2;
      inRow += 2;
      inRowBelow += 2;
    }
  }
  output.extendBorderPel(m_padding, m_padding);
}

int EncTemporalFilter::motionErrorLuma(const PelStorage &orig,
  const PelStorage &buffer,
  const int x,
  const int y,
  int dx,
  int dy,
  const int bs,
  const int besterror = 8 * 8 * 1024 * 1024) const
{
  const Pel* origOrigin = orig.Y().buf;
  const int  origStride = orig.Y().stride;
  const Pel* buffOrigin = buffer.Y().buf;
  const int  buffStride = buffer.Y().stride;

  int error = 0;
  if (((dx | dy) & 0xF) == 0)
  {
    dx /= m_motionVectorFactor;
    dy /= m_motionVectorFactor;
    for (int y1 = 0; y1 < bs; y1++)
    {
      const Pel* origRowStart   = origOrigin + (y + y1) * origStride + x;
      const Pel* bufferRowStart = buffOrigin + (y + y1 + dy) * buffStride + (x + dx);
      for (int x1 = 0; x1 < bs; x1 += 2)
      {
        int diff = origRowStart[x1] - bufferRowStart[x1];
        error += diff * diff;
        diff = origRowStart[x1 + 1] - bufferRowStart[x1 + 1];
        error += diff * diff;
      }
      if (error > besterror)
      {
        return error;
      }
    }
  }
  else
  {
    const int *xFilter = m_interpolationFilter[dx & 0xF];
    const int *yFilter = m_interpolationFilter[dy & 0xF];
    int tempArray[64 + 8][64];

    int sum, base;
    for (int y1 = 1; y1 < bs + 7; y1++)
    {
      const int yOffset = y + y1 + (dy >> 4) - 3;
      const Pel *sourceRow = buffOrigin + yOffset * buffStride + 0;
      for (int x1 = 0; x1 < bs; x1++)
      {
        sum = 0;
        base = x + x1 + (dx >> 4) - 3;
        const Pel *rowStart = sourceRow + base;

        sum += xFilter[1] * rowStart[1];
        sum += xFilter[2] * rowStart[2];
        sum += xFilter[3] * rowStart[3];
        sum += xFilter[4] * rowStart[4];
        sum += xFilter[5] * rowStart[5];
        sum += xFilter[6] * rowStart[6];

        tempArray[y1][x1] = sum;
      }
    }

    const Pel maxSampleValue = (1 << m_internalBitDepth[CHANNEL_TYPE_LUMA]) - 1;
    for (int y1 = 0; y1 < bs; y1++)
    {
      const Pel *origRow = origOrigin + (y + y1) * origStride;
      for (int x1 = 0; x1 < bs; x1++)
      {
        sum = 0;
        sum += yFilter[1] * tempArray[y1 + 1][x1];
        sum += yFilter[2] * tempArray[y1 + 2][x1];
        sum += yFilter[3] * tempArray[y1 + 3][x1];
        sum += yFilter[4] * tempArray[y1 + 4][x1];
        sum += yFilter[5] * tempArray[y1 + 5][x1];
        sum += yFilter[6] * tempArray[y1 + 6][x1];

        sum = (sum + (1 << 11)) >> 12;
        sum = sum < 0 ? 0 : (sum > maxSampleValue ? maxSampleValue : sum);

        error += (sum - origRow[x + x1]) * (sum - origRow[x + x1]);
      }
      if (error > besterror)
      {
        return error;
      }
    }
  }
  return error;
}

void EncTemporalFilter::motionEstimationLuma(Array2D<MotionVector> &mvs, const PelStorage &orig, const PelStorage &buffer, const int blockSize,
  const Array2D<MotionVector> *previous, const int factor, const bool doubleRes) const
{
#if JVET_V0056
  int range = doubleRes ? 0 : 5;
#else
  int range = 5;
#endif
  const int stepSize = blockSize;

  const int origWidth  = orig.Y().width;
  const int origHeight = orig.Y().height;

#if JVET_V0056
  for (int blockY = 0; blockY + blockSize <= origHeight; blockY += stepSize)
  {
    for (int blockX = 0; blockX + blockSize <= origWidth; blockX += stepSize)
#else
  for (int blockY = 0; blockY + blockSize < origHeight; blockY += stepSize)
  {
    for (int blockX = 0; blockX + blockSize < origWidth; blockX += stepSize)
#endif
    {
      MotionVector best;

      if (previous == NULL)
      {
        range = 8;
      }
      else
      {
#if JVET_V0056
        for (int py = -1; py <= 1; py++)
#else
        for (int py = -2; py <= 2; py++)
#endif
        {
          int testy = blockY / (2 * blockSize) + py;
#if JVET_V0056
          for (int px = -1; px <= 1; px++)
#else
          for (int px = -2; px <= 2; px++)
#endif
          {
            int testx = blockX / (2 * blockSize) + px;
            if ((testx >= 0) && (testx < origWidth / (2 * blockSize)) && (testy >= 0) && (testy < origHeight / (2 * blockSize)))
            {
              MotionVector old = previous->get(testx, testy);
              int error = motionErrorLuma(orig, buffer, blockX, blockY, old.x * factor, old.y * factor, blockSize, best.error);
              if (error < best.error)
              {
                best.set(old.x * factor, old.y * factor, error);
              }
            }
          }
        }
#if JVET_V0056
        int error = motionErrorLuma(orig, buffer, blockX, blockY, 0, 0, blockSize, best.error);
        if (error < best.error)
        {
          best.set(0, 0, error);
        }
#endif
      }
      MotionVector prevBest = best;
      for (int y2 = prevBest.y / m_motionVectorFactor - range; y2 <= prevBest.y / m_motionVectorFactor + range; y2++)
      {
        for (int x2 = prevBest.x / m_motionVectorFactor - range; x2 <= prevBest.x / m_motionVectorFactor + range; x2++)
        {
          int error = motionErrorLuma(orig, buffer, blockX, blockY, x2 * m_motionVectorFactor, y2 * m_motionVectorFactor, blockSize, best.error);
          if (error < best.error)
          {
            best.set(x2 * m_motionVectorFactor, y2 * m_motionVectorFactor, error);
          }
        }
      }
      if (doubleRes)
      {
        prevBest = best;
        int doubleRange = 3 * 4;
        for (int y2 = prevBest.y - doubleRange; y2 <= prevBest.y + doubleRange; y2 += 4)
        {
          for (int x2 = prevBest.x - doubleRange; x2 <= prevBest.x + doubleRange; x2 += 4)
          {
            int error = motionErrorLuma(orig, buffer, blockX, blockY, x2, y2, blockSize, best.error);
            if (error < best.error)
            {
              best.set(x2, y2, error);
            }
          }
        }

        prevBest = best;
        doubleRange = 3;
        for (int y2 = prevBest.y - doubleRange; y2 <= prevBest.y + doubleRange; y2++)
        {
          for (int x2 = prevBest.x - doubleRange; x2 <= prevBest.x + doubleRange; x2++)
          {
            int error = motionErrorLuma(orig, buffer, blockX, blockY, x2, y2, blockSize, best.error);
            if (error < best.error)
            {
              best.set(x2, y2, error);
            }
          }
        }
      }

#if JVET_V0056
      if (blockY > 0)
      {
        MotionVector aboveMV = mvs.get(blockX / stepSize, (blockY - stepSize) / stepSize);
        int error = motionErrorLuma(orig, buffer, blockX, blockY, aboveMV.x, aboveMV.y, blockSize, best.error);
        if (error < best.error)
        {
          best.set(aboveMV.x, aboveMV.y, error);
        }
      }
      if (blockX > 0)
      {
        MotionVector leftMV = mvs.get((blockX - stepSize) / stepSize, blockY / stepSize);
        int error = motionErrorLuma(orig, buffer, blockX, blockY, leftMV.x, leftMV.y, blockSize, best.error);
        if (error < best.error)
        {
          best.set(leftMV.x, leftMV.y, error);
        }
      }

      // calculate average
      double avg = 0.0;
      for (int x1 = 0; x1 < blockSize; x1++)
      {
        for (int y1 = 0; y1 < blockSize; y1++)
        {
          avg = avg + orig.Y().at(blockX + x1, blockY + y1);
        }
      }
      avg = avg / (blockSize * blockSize);

      // calculate variance
      double variance = 0;
      for (int x1 = 0; x1 < blockSize; x1++)
      {
        for (int y1 = 0; y1 < blockSize; y1++)
        {
          int pix = orig.Y().at(blockX + x1, blockY + y1);
          variance = variance + (pix - avg) * (pix - avg);
        }
      }
      best.error = (int)(20 * ((best.error + 5.0) / (variance + 5.0)) + (best.error / (blockSize * blockSize)) / 50);
#endif
      mvs.get(blockX / stepSize, blockY / stepSize) = best;
    }
  }
}

void EncTemporalFilter::motionEstimation(Array2D<MotionVector> &mv, const PelStorage &orgPic, const PelStorage &buffer, const PelStorage &origSubsampled2, const PelStorage &origSubsampled4) const
{
  const int width  = m_sourceWidth;
  const int height = m_sourceHeight;
  Array2D<MotionVector> mv_0(width / 16, height / 16);
  Array2D<MotionVector> mv_1(width / 16, height / 16);
  Array2D<MotionVector> mv_2(width / 16, height / 16);

  PelStorage bufferSub2;
  PelStorage bufferSub4;

  subsampleLuma(buffer, bufferSub2);
  subsampleLuma(bufferSub2, bufferSub4);

  motionEstimationLuma(mv_0, origSubsampled4, bufferSub4, 16);
  motionEstimationLuma(mv_1, origSubsampled2, bufferSub2, 16, &mv_0, 2);
  motionEstimationLuma(mv_2, orgPic, buffer, 16, &mv_1, 2);

  motionEstimationLuma(mv, orgPic, buffer, 8, &mv_2, 1, true);
}

void EncTemporalFilter::applyMotion(const Array2D<MotionVector> &mvs, const PelStorage &input, PelStorage &output) const
{
  static const int lumaBlockSize = 8;

  for(int c = 0; c < getNumberValidComponents(m_chromaFormatIDC); c++)
  {
    const ComponentID compID = (ComponentID)c;
    const int csx = getComponentScaleX(compID, m_chromaFormatIDC);
    const int csy = getComponentScaleY(compID, m_chromaFormatIDC);
    const int blockSizeX = lumaBlockSize >> csx;
    const int blockSizeY = lumaBlockSize >> csy;
    const int height = input.bufs[c].height;
    const int width  = input.bufs[c].width;

    const Pel maxValue = (1 << m_internalBitDepth[toChannelType(compID)]) - 1;

    const Pel *srcImage = input.bufs[c].buf;
    const int srcStride = input.bufs[c].stride;

    Pel *dstImage = output.bufs[c].buf;
    int dstStride = output.bufs[c].stride;

    for (int y = 0, blockNumY = 0; y + blockSizeY <= height; y += blockSizeY, blockNumY++)
    {
      for (int x = 0, blockNumX = 0; x + blockSizeX <= width; x += blockSizeX, blockNumX++)
      {
        const MotionVector &mv = mvs.get(blockNumX,blockNumY);
        const int dx = mv.x >> csx ;
        const int dy = mv.y >> csy ;
        const int xInt = mv.x >> (4 + csx) ;
        const int yInt = mv.y >> (4 + csy) ;

        const int *xFilter = m_interpolationFilter[dx & 0xf];
        const int *yFilter = m_interpolationFilter[dy & 0xf]; // will add 6 bit.
        const int numFilterTaps   = 7;
        const int centerTapOffset = 3;

        int tempArray[lumaBlockSize + numFilterTaps][lumaBlockSize];

        for (int by = 1; by < blockSizeY + numFilterTaps; by++)
        {
          const int yOffset = y + by + yInt - centerTapOffset;
          const Pel *sourceRow = srcImage + yOffset * srcStride;
          for (int bx = 0; bx < blockSizeX; bx++)
          {
            int base = x + bx + xInt - centerTapOffset;
            const Pel *rowStart = sourceRow + base;

            int sum = 0;
            sum += xFilter[1] * rowStart[1];
            sum += xFilter[2] * rowStart[2];
            sum += xFilter[3] * rowStart[3];
            sum += xFilter[4] * rowStart[4];
            sum += xFilter[5] * rowStart[5];
            sum += xFilter[6] * rowStart[6];

            tempArray[by][bx] = sum;
          }
        }

        Pel *dstRow = dstImage + y * dstStride;
        for (int by = 0; by < blockSizeY; by++, dstRow += dstStride)
        {
          Pel *dstPel = dstRow + x;
          for (int bx = 0; bx < blockSizeX; bx++, dstPel++)
          {
            int sum = 0;

            sum += yFilter[1] * tempArray[by + 1][bx];
            sum += yFilter[2] * tempArray[by + 2][bx];
            sum += yFilter[3] * tempArray[by + 3][bx];
            sum += yFilter[4] * tempArray[by + 4][bx];
            sum += yFilter[5] * tempArray[by + 5][bx];
            sum += yFilter[6] * tempArray[by + 6][bx];

            sum = (sum + (1 << 11)) >> 12;
            sum = sum < 0 ? 0 : (sum > maxValue ? maxValue : sum);
            *dstPel = sum;
          }
        }
      }
    }
  }
}

void EncTemporalFilter::bilateralFilter(const PelStorage &orgPic,
#if JVET_V0056
  std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo,
#else
  const std::deque<TemporalFilterSourcePicInfo> &srcFrameInfo,
#endif
  PelStorage &newOrgPic,
  double overallStrength) const
{
  const int numRefs = int(srcFrameInfo.size());

  const int refStrengthRow = m_futureRefs > 0 ? 0 : 1;

  const double lumaSigmaSq = (m_QP - m_sigmaZeroPoint) * (m_QP - m_sigmaZeroPoint) * m_sigmaMultiplier;
  const double chromaSigmaSq = 30 * 30;

  for(int c = 0; c < getNumberValidComponents(m_chromaFormatIDC); c++)
  {
    const ComponentID compID = (ComponentID)c;
    const int height = orgPic.bufs[c].height;
    const int width  = orgPic.bufs[c].width;
    const Pel* srcPelRow = orgPic.bufs[c].buf;
    const int  srcStride = orgPic.bufs[c].stride;
          Pel* dstPelRow = newOrgPic.bufs[c].buf;
    const int  dstStride = newOrgPic.bufs[c].stride;
    const double sigmaSq = isChroma(compID) ? chromaSigmaSq : lumaSigmaSq;
    const double weightScaling = overallStrength * (isChroma(compID) ? m_chromaFactor : 0.4);
    const Pel maxSampleValue   = (1 << m_internalBitDepth[toChannelType(compID)]) - 1;
    const double bitDepthDiffWeighting = 1024.0 / (maxSampleValue + 1);
#if JVET_V0056
    const int lumaBlockSize = 8;
    const int csx = getComponentScaleX(compID, m_chromaFormatIDC);
    const int csy = getComponentScaleY(compID, m_chromaFormatIDC);
    const int blockSizeX = lumaBlockSize >> csx;
    const int blockSizeY = lumaBlockSize >> csy;
#endif

    for (int y = 0; y < height; y++, srcPelRow += srcStride, dstPelRow += dstStride)
    {
      const Pel *srcPel = srcPelRow;
      Pel *dstPel = dstPelRow;
      for (int x = 0; x < width; x++, srcPel++, dstPel++)
      {
        const int orgVal = (int) *srcPel;
        double temporalWeightSum = 1.0;
        double newVal = (double) orgVal;
#if JVET_V0056
        if ((y % blockSizeY == 0) && (x % blockSizeX == 0))
        {
          for (int i = 0; i < numRefs; i++)
          {
            double variance = 0, diffsum = 0;
            const ptrdiff_t refStride = srcFrameInfo[i].picBuffer.bufs[c].stride;
            const Pel *     refPel = srcFrameInfo[i].picBuffer.bufs[c].buf + y * refStride + x;
            for( int y1 = 0; y1 < blockSizeY; y1++ )
            {
              for( int x1 = 0; x1 < blockSizeX; x1++ )
              {
                const Pel pix = *(srcPel + srcStride * y1 + x1);
                const Pel ref = *(refPel + refStride * y1 + x1);
                const int diff = pix - ref;
                variance += diff * diff;
                if( x1 != blockSizeX - 1 )
                {
                  const Pel pixR = *(srcPel + srcStride * y1 + x1 + 1);
                  const Pel refR = *(refPel + refStride * y1 + x1 + 1);
                  const int diffR = pixR - refR;
                  diffsum += (diffR - diff) * (diffR - diff);
                }
                if( y1 != blockSizeY - 1 )
                {
                  const Pel pixD = *(srcPel + srcStride * y1 + x1 + srcStride);
                  const Pel refD = *(refPel + refStride * y1 + x1 + refStride);
                  const int diffD = pixD - refD;
                  diffsum += (diffD - diff) * (diffD - diff);
                }
              }
            }
            const int cntV = blockSizeX * blockSizeY;
            const int cntD = 2 * cntV - blockSizeX - blockSizeY;
            srcFrameInfo[i].mvs.get( x / blockSizeX, y / blockSizeY ).noise =
              ( int ) round( (15.0 * cntD / cntV * variance + 5.0) / (diffsum + 5.0) );
          }
        }
        double minError = 9999999;
        for (int i = 0; i < numRefs; i++)
        {
          minError = std::min(minError, (double) srcFrameInfo[i].mvs.get(x / blockSizeX, y / blockSizeY).error);
        }
#endif
        for (int i = 0; i < numRefs; i++)
        {
#if JVET_V0056
          const int error = srcFrameInfo[i].mvs.get(x / blockSizeX, y / blockSizeY).error;
          const int noise = srcFrameInfo[i].mvs.get(x / blockSizeX, y / blockSizeY).noise;
#endif
          const Pel* pCorrectedPelPtr = srcFrameInfo[i].picBuffer.bufs[c].buf + (y * srcFrameInfo[i].picBuffer.bufs[c].stride + x);
          const int refVal = (int) *pCorrectedPelPtr;
          double diff = (double)(refVal - orgVal);
          diff *= bitDepthDiffWeighting;
          double diffSq = diff * diff;
#if JVET_V0056
          const int index = std::min(3, std::abs(srcFrameInfo[i].origOffset) - 1);
          double ww = 1, sw = 1;
          ww *= (noise < 25) ? 1.0 : 0.6;
          sw *= (noise < 25) ? 1.0 : 0.8;
          ww *= (error < 50) ? 1.2 : ((error > 100) ? 0.6 : 1.0);
          sw *= (error < 50) ? 1.0 : 0.8;
          ww *= ((minError + 1) / (error + 1));
          double weight = weightScaling * m_refStrengths[refStrengthRow][index] * ww * exp(-diffSq / (2 * sw * sigmaSq));
#else
          const int index = std::min(1, std::abs(srcFrameInfo[i].origOffset) - 1);
          const double weight = weightScaling * m_refStrengths[refStrengthRow][index] * exp(-diffSq / (2 * sigmaSq));
#endif
          newVal += weight * refVal;
          temporalWeightSum += weight;
        }
        newVal /= temporalWeightSum;
        Pel sampleVal = (Pel)round(newVal);
        sampleVal = (sampleVal < 0 ? 0 : (sampleVal > maxSampleValue ? maxSampleValue : sampleVal));
        *dstPel = sampleVal;
      }
    }
  }
}

//! \}

