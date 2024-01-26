#include <iomanip>
#include <iostream>
#include <fstream>
#include <string>

#include "program_options_lite.h"
#include "Contexts.h"
#include "io.h"
#include "optimization.h"
#include "simulator.h"

using namespace std;
namespace po = df::program_options_lite;
static constexpr int CNU = 35;

int main( int argc, char **argv )
{
  // parsing
  bool        doHelp = false;
  bool        keepModel = false;
  std::string outfile;
  std::string inputFile = "";
  po::Options opts;
  // clang-format off
  opts.addOptions()
    ( "help", doHelp, false, "optimize cabac contexts. Usage:\ncat *.cabac_xxx | cabacTrain --output fileout.txt" )
    ( "keepModel", keepModel, false, "if no bin, keep old parameters instead of putting default parameters" )
    ( "output", outfile, std::string( "cabac_prm.txt" ), "filename of the resulting parameters" )
    ( "input", inputFile, std::string( "" ), "filename of the input bins" )
    ;
  // clang-format on
  po::setDefaults( opts );
  po::ErrorReporter         err;
  const list<const char *> &argvUnhandled = po::scanArgv( opts, argc, (const char **)argv, err );

  for( list<const char *>::const_iterator it = argvUnhandled.begin(); it != argvUnhandled.end(); it++ )
  {
    msg( ERROR, "Unhandled argument ignored: `%s'\n", *it );
  }

  if( doHelp )
  {
    po::doHelp( cout, opts );
    return -1;
  }

  if( err.is_errored )
  {
    return -1;
  }

  DataDb db;

  if( inputFile != "" )
  {
    ifstream in( inputFile.c_str() );
    db = loadDataFrame( in );
  }
  else
  {
    db = loadDataFrame();
  }

  std::vector<SliceType> toOpt = { B_SLICE, P_SLICE, I_SLICE };
  std::array<bool, 3>    sliceActivation = {};
  if( db.ctxidx < 0 )
  {
    std::cout << "[ERROR] no context to train found" << std::endl;
    return -1;
  }
  std::cout << "Ctx " << db.ctxidx << std::endl;
  bool hasBin = false;
  for( int k = 0; k < (int)toOpt.size(); ++k )
  {
    SliceType st = toOpt[k];
    sliceActivation[k] = hasBin1Db( db.v, st );
    hasBin = true;
  }

  std::array<ModelParameters, 3> modelsgreedy = db.modelsBPI;
  std::cout << "Before:\n";
  print( db.modelsBPI );
  std::cout << std::endl;
  if( hasBin )
  {
    for( int k = 0; k < (int)toOpt.size(); ++k )
    {
      SliceType st = toOpt[k];
      if( sliceActivation[k] )
      {
        std::cout << "Optimized parameter slice type " << k << std::endl;
        modelsgreedy[(int)st] = getBestGreedy( db, st );
        db.modelsBPI[(int)st] = modelsgreedy[(int)st];

#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
        std::cout << "Optimized rate offsets " << std::endl;
        auto drate = getBestGreedyDrate( db, st );

        db.modelsBPI[k].rateoffset[0] = drate[0];
        db.modelsBPI[k].rateoffset[1] = drate[1];
        modelsgreedy[k].rateoffset[0] = drate[0];
        modelsgreedy[k].rateoffset[1] = drate[1];
#endif
      }
      else
      {
        if( keepModel )
        {
          // do not change the prm
        }
        else
        {
          std::cout << "Default model (no bin) for slice type " << k << std::endl;
          modelsgreedy[k].initId = db.modelsBPI[(int)st].initId = CNU;
          modelsgreedy[k].log2windowsize = db.modelsBPI[(int)st].log2windowsize = DWS;
          modelsgreedy[k].adaptweight = db.modelsBPI[(int)st].adaptweight = DWE;
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
          modelsgreedy[k].rateoffset[0] = DWO;
          modelsgreedy[k].rateoffset[1] = DWO;
#endif
        }
      }
    }

#if !JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
    std::cout << "Optimized rate offsets " << std::endl;
    auto drate = getBestGreedyDrate( db, sliceActivation );
    for( int k = 0; k < 3; ++k )
    {
      db.modelsBPI[k].rateoffset[0] = drate[0];
      db.modelsBPI[k].rateoffset[1] = drate[1];
      modelsgreedy[k].rateoffset[0] = drate[0];
      modelsgreedy[k].rateoffset[1] = drate[1];
    }
#endif
  }
  else
  {
    if( keepModel )
    {
      std::cout << "Skipped (no bin)" << std::endl;
    }
    else
    {
      std::cout << "Default model (no bin)" << std::endl;
      for( int k = 0; k < 3; ++k )
      {
        modelsgreedy[k].initId = CNU;
        modelsgreedy[k].log2windowsize = DWS;
        modelsgreedy[k].adaptweight = DWE;
        modelsgreedy[k].rateoffset[0] = DWO;
        modelsgreedy[k].rateoffset[1] = DWO;
      }
    }
  }
  std::cout << "\nAfter:\n";
  print( modelsgreedy );

  if( !outfile.empty() )
  {
    ofstream file( outfile, ios::binary );
    file << db.ctxidx << " A " << modelsgreedy[0].initId << ' ' << modelsgreedy[1].initId << ' '
      << modelsgreedy[2].initId << ' ' << modelsgreedy[0].log2windowsize << ' ' << modelsgreedy[1].log2windowsize
      << ' ' << modelsgreedy[2].log2windowsize << ' ' << modelsgreedy[0].adaptweight << ' '
      << modelsgreedy[1].adaptweight << ' ' << modelsgreedy[2].adaptweight << ' ' << modelsgreedy[0].rateoffset[0]
      << ' ' << modelsgreedy[0].rateoffset[1] 
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
      << ' ' << modelsgreedy[1].rateoffset[0] << ' ' << modelsgreedy[1].rateoffset[1] << ' ' << modelsgreedy[2].rateoffset[0] << ' ' << modelsgreedy[2].rateoffset[1]
#endif
      << std::endl;
  }
}
