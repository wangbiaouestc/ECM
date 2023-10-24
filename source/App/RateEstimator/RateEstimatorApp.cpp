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

#include <iostream>
#include <vector>

#include <boost/property_tree/xml_parser.hpp>
#include "ContextsEst.h"


#if defined(_WIN32) || defined(__WIN32__) || defined(WIN32)
#define PATH_SEP '\\'
#else
#define PATH_SEP '/'
#endif

#if EXTENSION_CABAC_TRAINING
struct BinStreamInfo
{
  int qp;
  int pos;
  int len;
};


struct CandidateBits
{
  CandidateBits() : candBits{} {}

  static const int shiftIdxList[13];
  static const int weightedAdaptRate[5];
  static const int rateOffset[25];
  static constexpr int maxNumCandidates = 64;

  uint64_t candBits[13][5][maxNumCandidates];
};


struct CtxStreams
{
  CtxStreams( std::string n, int i ) : name( n ), idx( i ) {}

  std::string name;
  int idx;
  std::vector<BinStreamInfo> initTypeStreams[3];
  CandidateBits initTypeBits[3];
};


const int CandidateBits::shiftIdxList[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13 }; // these are the valid values for shiftIdx;
const int CandidateBits::weightedAdaptRate[5] = { 4, 11, 18, 25, 32 };
const int CandidateBits::rateOffset[25] = { 153, 154, 155, 156, 157, 158, 169, 171, 185, 186, 187, 189, 190, 201, 202, 203, 217, 219, 221, 222, 233, 235, 236, 237, 238 };



void parseNextXML(const boost::property_tree::ptree& tree, std::vector<CtxStreams>& streams, std::string& origStreamBits )
{
  for( const auto &frameIter : tree.get_child("CabacBits") )
  {
    if(frameIter.first == "frame")
    {
      const boost::property_tree::ptree &frame = frameIter.second;
      int initType = frame.get<int>("initType");
      int curQP = frame.get<int>("QP");

      for( const auto &frameChildIter : frame.get_child("") )
      {
        if(frameChildIter.first == "BinProbModel")
        {
          const boost::property_tree::ptree &frameChild = frameChildIter.second;
          std::string curName = frameChild.get<std::string>("<xmlattr>.name");
          int curIndex = frameChild.get<int>("<xmlattr>.index");
          int len = frameChild.get<int>("binstream.<xmlattr>.len");
          int pos = 0;
          if( len != 0 )
          {
            pos = frameChild.get<int>("binstream.<xmlattr>.pos");
          }
          CtxStreams *currentStream = nullptr;
          for( auto &st : streams )
          {
            if( st.name == curName && st.idx == curIndex )
            {
              currentStream = &st;
              break;
            }
          }
          if( currentStream == nullptr )
          {
            streams.push_back( CtxStreams( curName, curIndex) );
            currentStream = &streams.back();
          }
          if( len > 0 )
          {
            currentStream->initTypeStreams[initType].push_back( BinStreamInfo( { curQP, pos, len } ) );
          }
        }
      }
    }
    else if(frameIter.first == "sequenceBits")
    {
      origStreamBits = (frameIter.second.data());
    }
  }
}


uint64_t getRate( char * storage, const BinStreamInfo &bsInfo, BinProbModel &probEst )
{
  uint64_t bits = 0;
  for(int pos = 0; pos < bsInfo.len; pos++)
  {
    bool bin = !!(storage[(pos >> 3)] & (128 >> (pos & 7)));
    probEst.estFracBitsUpdate( bin, bits );
  }
  return bits;
}


void updateAllRates( std::istream &binStream, const BinStreamInfo &bsInfo, CandidateBits &bits, BinProbModel &probEst )
{
  binStream.seekg( bsInfo.pos, std::ios::beg );
  unsigned int numOfBlocks = (bsInfo.len >> 3) + (bsInfo.len & 7 ? 1: 0);
  std::vector<char> storage( numOfBlocks );
  binStream.read( storage.data(), numOfBlocks );

  for (int i = 0; i < 13; ++i)
  {
    int shiftIdx = CandidateBits::shiftIdxList[i];
    for (int j = 0; j < 5; j++)
    {
      int adaWeight = CandidateBits::weightedAdaptRate[j];

      for (int initValue = 0; initValue < probEst.getNumInitVals(); ++initValue)
      {
        probEst.init(bsInfo.qp, initValue);
        probEst.setLog2WindowSize(shiftIdx);
        probEst.setAdaptRateWeight(adaWeight);
        bits.candBits[i][j][initValue] += getRate(storage.data(), bsInfo, probEst);
      }
    }
  }
}


std::string getBasename( std::string fn )
{
  while( fn.back() == PATH_SEP )
  {
    fn.pop_back();
  }
  std::string outFn;
  while( fn.length() > 0 && fn.back() != PATH_SEP )
  {
    outFn = fn.back() + outFn;
    fn.pop_back();
  }
  return outFn;
}


int main( int argc, char* argv[] )
{
  if( argc != 4 )
  {
    std::cout << "usage: ProbEst xml_file bin_file output_dir" << std::endl;
    std::exit( 0 );
  }

  clock_t lBefore = clock();

  std::string seqName = argv[1];
  std::string origStreamBits;

  std::vector<CtxStreams> ctxStreams;
  {
    std::fstream xmlFile( seqName, std::fstream::in );
    boost::property_tree::ptree pt;
    read_xml( xmlFile, pt );
    parseNextXML( pt, ctxStreams, origStreamBits );
  }

  std::fstream binFile;
  binFile.open(argv[2], std::fstream::in | std::fstream::binary );

  BinProbModel prEst;

  if( prEst.getNumInitVals() > CandidateBits::maxNumCandidates )
  {
    std::cerr << "Too small: CandidateBits::maxNumCandidates" << std::endl;
    std::exit( 0 );
  }

  seqName.erase( seqName.length() - 4 );
  seqName = getBasename( seqName );
  std::vector<std::string> fileList;
  std::string outFilePath = argv[3];
  if( outFilePath.back() != PATH_SEP )
  {
    outFilePath += PATH_SEP;
  }

  const std::vector<uint8_t> &rateOffsetInitTable0 = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES * 3));
  const std::vector<uint8_t> &rateOffsetInitTable1 = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES * 3) + 1);
  const std::vector<uint8_t>& initTableB       = ContextSetCfg::getInitTable(B_SLICE);
  const std::vector<uint8_t> &rateInitTableB   = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES + B_SLICE);
  const std::vector<uint8_t> &weightInitTableB = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES << 1) + B_SLICE);
  const std::vector<uint8_t>& initTableP       = ContextSetCfg::getInitTable(P_SLICE);
  const std::vector<uint8_t> &rateInitTableP   = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES + P_SLICE);
  const std::vector<uint8_t> &weightInitTableP = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES << 1) + P_SLICE);
  const std::vector<uint8_t>& initTableI       = ContextSetCfg::getInitTable(I_SLICE);
  const std::vector<uint8_t> &rateInitTableI   = ContextSetCfg::getInitTable(NUMBER_OF_SLICE_TYPES + I_SLICE);
  const std::vector<uint8_t> &weightInitTableI = ContextSetCfg::getInitTable((NUMBER_OF_SLICE_TYPES << 1) + I_SLICE);

  int initIdx = 0;


  for( auto &s : ctxStreams )
  {
    prEst.setAdaptRateOffset(rateOffsetInitTable0[initIdx], 0);
    prEst.setAdaptRateOffset(rateOffsetInitTable1[initIdx], 1);
    for (int i : { 0, 1, 2 })
    {
      std::cout << "Estimating rates for: " << s.name << " " << s.idx << "[" << i << "] initIdx:" << initIdx << " offset:[" << (int)rateOffsetInitTable0[initIdx]  << " " << (int)rateOffsetInitTable1[initIdx] << "]" << std::endl;
      for (const auto &c : s.initTypeStreams[i])
      {
        updateAllRates(binFile, c, s.initTypeBits[i], prEst);
      }
    }

    std::stringstream outFileName;
    outFileName << seqName << "_" << s.name << "_" << s.idx << ".txt";
    std::string outFilePathAndName = outFilePath + outFileName.str();
    fileList.push_back(outFileName.str());

    std::cout << "Writing: " << outFilePathAndName << std::endl;
    std::fstream outFile(outFilePathAndName, std::fstream::out);
    if (!outFile.good())
    {
      std::cerr << "Error! Could not open output file. Exiting." << std::endl;
      std::exit(0);
    }
    for (int i : { 0, 1, 2 })
    {
      for (int ws = 0; ws < 13; ++ws)
      {
        for (int adaw = 0; adaw < 5; ++adaw)
        {
          for (int init = 0; init < prEst.getNumInitVals(); ++init)
          {
            uint64_t currBits = s.initTypeBits[i].candBits[ws][adaw][init];
            outFile << currBits << ", ";
          }
          outFile << std::endl;
        }
      }
      outFile << std::endl;
    }
    outFile << (int)rateOffsetInitTable0[initIdx] << ", " << (int)rateOffsetInitTable1[initIdx] << ", ";
    outFile << (int)initTableB[initIdx] << ", " << (int)rateInitTableB[initIdx] << ", " << (int)weightInitTableB[initIdx] << ", ";
    outFile << (int)initTableP[initIdx] << ", " << (int)rateInitTableP[initIdx] << ", " << (int)weightInitTableP[initIdx] << ", ";
    outFile << (int)initTableI[initIdx] << ", " << (int)rateInitTableI[initIdx] << ", " << (int)weightInitTableI[initIdx] << std::endl;
    outFile.close();
    initIdx++;
  }

  std::string indexFileName = outFilePath + seqName + "_index.txt";
  std::fstream indexFile( indexFileName, std::fstream::out );
  if( !indexFile.good() )
  {
    std::cerr << "Error! Could not open index file file: " << indexFileName << std::endl;
    std::exit( 0 );
  }

  indexFile << origStreamBits << std::endl;
  for( auto i : fileList )
  {
    indexFile << i << std::endl;
  }
  indexFile.close();

  double dResult = (double)(clock()-lBefore) / CLOCKS_PER_SEC;
  printf("\n Total Time: %12.3f min.\n", dResult / 60.0 );

  return 0;
}

#endif