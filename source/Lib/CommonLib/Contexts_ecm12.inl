#pragma once
#include "CommonDef.h"
#if JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE
// CONTEXTS WSA START
const CtxSet ContextSetCfg::SplitFlag = ContextSetCfg::addCtxSet({
// ctx 0 to 8
 { 11, 20, 52, 19, 14, 30, CNU, 30, 31, },
 { 11, 20, 60, 12, 21, 30, 28, 30, 31, },
 { 27, 36, 38, 20, 29, 30, 28, 38, 31, },
 { 5, 6, 5, 5, 10, 13, 10, 7, 13, },
 { 9, 13, 6, 9, 10, 13, 10, 10, 13, },
 { 9, 9, 9, 9, 10, 12, 5, 6, 9, },
 { 4, 4, DWE, 4, 4, 11, 11, 4, DWE, },
 { 4, 25, DWE, 11, 4, 11, 11, 11, 11, },
 { 4, 4, DWE, 11, 4, 4, 11, 4, 4, },
 { 180, 213, 134, 246, 247, 234, 106, 130, 133, },
 { 140, 171, 118, 172, 250, 210, 213, 116, 88, },
 { 243, 150, 150, 135, 250, 235, 107, 116, 149, },
 { 151, 251, 86, 252, 251, 115, 123, 118, 84, },
 { 243, 139, 151, 164, 250, 235, 105, 115, 181, },
 { 251, 251, 87, 251, 250, 88, 118, 123, 84, },
});

const CtxSet ContextSetCfg::SplitQtFlag = ContextSetCfg::addCtxSet({
// ctx 9 to 14
 { 27, 22, 23, 11, 12, 6, },
 { 20, 22, 23, 11, 12, 21, },
 { 11, 13, 30, 33, 27, 22, },
 { 6, 6, 13, 12, 6, 5, },
 { 7, 9, 10, 12, 10, 9, },
 { 4, 9, 9, 12, 12, 9, },
 { 4, 4, DWE, 4, 4, 4, },
 { 11, 4, 4, 4, 4, 11, },
 { 11, DWE, DWE, 11, 11, 11, },
 { 241, 115, 137, 101, 122, 118, },
 { 134, 251, 130, 199, 180, 122, },
 { 129, 115, 244, 91, 139, 133, },
 { 122, 252, 117, 235, 251, 148, },
 { 116, 107, 133, 90, 167, 196, },
 { 135, 198, 121, 235, 235, 171, },
});

const CtxSet ContextSetCfg::SplitHvFlag = ContextSetCfg::addCtxSet({
// ctx 15 to 19
 { 43, CNU, 44, 34, 45, },
 { 36, CNU, 44, 34, 45, },
 { 43, 50, 44, 27, 52, },
 { 7, 6, 10, 5, 5, },
 { 10, 10, 13, 9, 6, },
 { 9, 9, 9, 5, 5, },
 { 4, 4, 4, 4, 11, },
 { 4, 4, 11, 4, 4, },
 { 11, 11, 11, 4, 4, },
 { 133, 148, 164, 117, 164, },
 { 122, 123, 147, 148, 124, },
 { 134, 132, 133, 148, 148, },
 { 123, 133, 133, 132, 124, },
 { 244, 148, 149, 149, 212, },
 { 117, 148, 120, 148, 133, },
});

const CtxSet ContextSetCfg::Split12Flag = ContextSetCfg::addCtxSet({
// ctx 20 to 23
 { 44, 37, 36, 37, },
 { 44, 37, 29, 37, },
 { 44, 45, 44, 45, },
 { 12, 10, 12, 13, },
 { 13, 13, 13, 13, },
 { 12, 13, 12, 13, },
 { DWE, 4, 11, 4, },
 { 11, 4, 11, 4, },
 { DWE, 4, 25, 4, },
 { 104, 106, 196, 106, },
 { 107, 90, 132, 100, },
 { 165, 116, 150, 106, },
 { 195, 89, 102, 131, },
 { 134, 100, 121, 100, },
 { 120, 99, 116, 180, },
});

const CtxSet ContextSetCfg::SkipFlag = ContextSetCfg::addCtxSet({
// ctx 24 to 26
 { 49, 59, 53, },
 { 57, 59, 53, },
 { 0, 57, 52, },
 { 6, 6, 10, },
 { 6, 7, 10, },
 { 4, 6, 10, },
 { DWE, 11, DWE, },
 { DWE, 11, 11, },
 { 4, 4, 11, },
 { 118, 118, 133, },
 { 118, 195, 242, },
 { 118, 124, 133, },
 { 117, 132, 147, },
 { 118, 148, 196, },
 { 118, 243, 243, },
});

const CtxSet ContextSetCfg::MergeFlag = ContextSetCfg::addCtxSet({
// ctx 27
 { 14, },
 { 7, },
 { 5, },
 { 5, },
 { 6, },
 { 5, },
 { DWE, },
 { 11, },
 { 4, },
 { 147, },
 { 108, },
 { 118, },
 { 109, },
 { 148, },
 { 104, },
});


#if JVET_AG0276_LIC_FLAG_SIGNALING
const CtxSet ContextSetCfg::MergeFlagOppositeLic = ContextSetCfg::addCtxSet
({
  {  12, },
  {   4, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  11, },
  {   4, },
  { DWE, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  });
const CtxSet ContextSetCfg::AffineFlagOppositeLic = ContextSetCfg::addCtxSet
({
  {  12, },
  {   4, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  11, },
  {   4, },
  { DWE, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  });
const CtxSet ContextSetCfg::TmMergeFlagOppositeLic = ContextSetCfg::addCtxSet
({
  {  12, },
  {   4, },
  { CNU, },
  {   5, },
  {   5, },
  { DWS, },
  {  11, },
  {   4, },
  { DWE, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  { 132, },
  });
#endif


const CtxSet ContextSetCfg::RegularMergeFlag = ContextSetCfg::addCtxSet({
// ctx 28 to 29
 { 30, 6, },
 { 23, 6, },
 { 0, 7, },
 { 6, 6, },
 { 7, 6, },
 { 7, 6, },
 { 4, DWE, },
 { 4, DWE, },
 { 4, DWE, },
 { 117, 107, },
 { 116, 115, },
 { 116, 107, },
 { 131, 115, },
 { 117, 107, },
 { 114, 114, },
});

#if JVET_AG0164_AFFINE_GPM
const CtxSet ContextSetCfg::AffineGPMFlag = ContextSetCfg::addCtxSet
({
  {  19,   6,   7, },
  {  19,   5,   6, },
  { CNU, CNU, CNU, },
  {   5,   1,   0, },
  {   5,   1,   1, },
  { DWS, DWS, DWS, },
  {  11,  11,   4, },
  {  11,  11,  18, },
  { DWE, DWE, DWE, },
  { 116, 119, 148, },
  { 117, 117, 116, },
  { 116, 119, 148, },
  { 117, 117, 116, },
  { 116, 119, 148, },
  { 117, 117, 116, },
});

const CtxSet ContextSetCfg::GpmMergeIdx = ContextSetCfg::addCtxSet
({
  {  26,  29,  44,  44,  29, CNU, CNU, CNU, CNU, CNU, },
  {   5,  29,  44,  37,  44, CNU, CNU, CNU, CNU, CNU, },
  {  33,  42,  43,  27,  18, CNU, CNU, CNU, CNU, CNU, },
  {   5,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
  {   6,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
  {   6,   7,   6,  13,  12, DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
});

const CtxSet ContextSetCfg::GpmAffMergeIdx = ContextSetCfg::addCtxSet
({
  {  26,  29,  44,  44,  29, CNU, CNU, CNU, CNU, CNU, },
  {   5,  29,  44,  37,  44, CNU, CNU, CNU, CNU, CNU, },
  {  33,  42,  43,  27,  18, CNU, CNU, CNU, CNU, CNU, },
  {   5,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
  {   6,   5,   5,   5,   9, DWS, DWS, DWS, DWS, DWS, },
  {   6,   7,   6,  13,  12, DWS, DWS, DWS, DWS, DWS, },
  {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,  18,  18,  11, DWE, DWE, DWE, DWE, DWE, },
  {  18,  18,   4,   4,   4, DWE, DWE, DWE, DWE, DWE, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
  { 119, 134, 126, 126, 198, 119, 119, 119, 119, 119, },
  { 117, 105, 117, 116, 238, 119, 119, 119, 119, 119, },
});
#endif

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
const CtxSet ContextSetCfg::GeoBlendFlag = ContextSetCfg::addCtxSet
({
  {  CNU, },
  {  CNU, },
  {  CNU, },
  {  DWS, },
  {  DWS, },
  {  DWS, },
  {  DWE, },
  {  DWE, },
  {  DWE, },
  {  DWO, },
  {  DWO, },
  {  DWO, },
  {  DWO, },
  {  DWO, },
  {  DWO, },
});
#endif

#if JVET_AG0135_AFFINE_CIIP
const CtxSet ContextSetCfg::CiipAffineFlag = ContextSetCfg::addCtxSet
({
  {  12,   6,  15, },
  {  19,   5,   6, },
  { CNU, CNU, CNU, },
  {   5,   1,   0, },
  {   5,   1,   8, },
  { DWS, DWS, DWS, },
  {  11,  11,   4, },
  {   4,   4,  11, },
  {  18,  18,  18, },
  { 108, 122, 133, },
  { 116, 117, 116, },
  { 108, 122, 133, },
  { 116, 117, 116, },
  { 108, 122, 133, },
  { 116, 117, 116, },
});
#endif

const CtxSet ContextSetCfg::MergeIdx = ContextSetCfg::addCtxSet({
// ctx 30 to 39
 { 26, 29, 36, 44, 29, CNU, CNU, CNU, CNU, CNU, },
 { 5, 29, 44, 44, 29, CNU, CNU, CNU, CNU, CNU, },
 { 34, 43, 51, 51, 43, CNU, CNU, CNU, CNU, CNU, },
 { 5, 5, 5, 6, 13, DWS, DWS, DWS, DWS, DWS, },
 { 7, 5, 5, 6, 10, DWS, DWS, DWS, DWS, DWS, },
 { 6, 5, 5, 6, 13, DWS, DWS, DWS, DWS, DWS, },
 { 11, 4, 4, 11, 4, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 11, 11, DWE, 4, DWE, DWE, DWE, DWE, DWE, },
 { 11, 4, 4, 11, 4, DWE, DWE, DWE, DWE, DWE, },
 { 116, 132, 117, 116, 214, DWO, DWO, DWO, DWO, DWO, },
 { 108, 117, 117, 118, 234, DWO, DWO, DWO, DWO, DWO, },
 { 117, 132, 117, 117, 165, DWO, DWO, DWO, DWO, DWO, },
 { 116, 116, 132, 118, 250, DWO, DWO, DWO, DWO, DWO, },
 { 131, 117, 118, 117, 234, DWO, DWO, DWO, DWO, DWO, },
 { 117, 118, 118, 118, 186, DWO, DWO, DWO, DWO, DWO, },
});

const CtxSet ContextSetCfg::TmMergeIdx = ContextSetCfg::addCtxSet({
// ctx 40 to 49
 { 28, 28, 42, 44, 43, CNU, CNU, CNU, CNU, CNU, },
 { 21, 20, 42, 5, 4, CNU, CNU, CNU, CNU, CNU, },
 { 22, 29, 44, 44, 43, CNU, CNU, CNU, CNU, CNU, },
 { 5, 5, 5, 7, 13, DWS, DWS, DWS, DWS, DWS, },
 { 5, 5, 5, 9, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 5, 5, 5, 13, 13, DWS, DWS, DWS, DWS, DWS, },
 { DWE, 11, 4, 11, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 11, 11, 4, 4, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 4, 4, 11, 11, DWE, DWE, DWE, DWE, DWE, },
 { 133, 117, 133, 250, 233, DWO, DWO, DWO, DWO, DWO, },
 { 116, DWO, 148, 99, 84, DWO, DWO, DWO, DWO, DWO, },
 { 133, 117, 120, 212, 201, DWO, DWO, DWO, DWO, DWO, },
 { 116, 133, 118, 104, 92, DWO, DWO, DWO, DWO, DWO, },
 { 133, 132, 117, 118, 234, DWO, DWO, DWO, DWO, DWO, },
 { 116, 122, 120, 83, 98, DWO, DWO, DWO, DWO, DWO, },
});

const CtxSet ContextSetCfg::GPMIntraFlag = ContextSetCfg::addCtxSet({
// ctx 50
 { 26, },
 { 33, },
 { 0, },
 { 2, },
 { 2, },
 { 2, },
 { 11, },
 { 4, },
 { 11, },
 { 244, },
 { 133, },
 { 244, },
 { 133, },
 { 148, },
 { 134, },
});

const CtxSet ContextSetCfg::MmvdFlag = ContextSetCfg::addCtxSet({
// ctx 51 to 54
 { 25, 34, CNU, CNU, },
 { 18, 19, CNU, CNU, },
 { 40, 42, CNU, CNU, },
 { 5, 5, DWS, DWS, },
 { 6, 5, DWS, DWS, },
 { 6, 5, DWS, DWS, },
 { 11, DWE, DWE, DWE, },
 { DWE, DWE, DWE, DWE, },
 { 11, DWE, DWE, DWE, },
 { 116, 131, DWO, DWO, },
 { 116, 115, DWO, DWO, },
 { 116, 118, DWO, DWO, },
 { 116, 116, DWO, DWO, },
 { 178, 131, DWO, DWO, },
 { 242, 116, DWO, DWO, },
});

const CtxSet ContextSetCfg::MmvdMergeIdx = ContextSetCfg::addCtxSet({
// ctx 55 to 58
 { 43, 43, CNU, CNU, },
 { 43, 43, CNU, CNU, },
 { 43, 43, CNU, CNU, },
 { 9, 10, DWS, DWS, },
 { 10, 10, DWS, DWS, },
 { 10, 10, DWS, DWS, },
 { 4, 4, DWE, DWE, },
 { 4, 4, DWE, DWE, },
 { 4, 4, DWE, DWE, },
 { 134, 242, DWO, DWO, },
 { 101, 147, DWO, DWO, },
 { 148, 131, DWO, DWO, },
 { 131, 148, DWO, DWO, },
 { 135, 121, DWO, DWO, },
 { 105, 116, DWO, DWO, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdx = ContextSetCfg::addCtxSet({
// ctx 59 to 63
 { 28, 36, 36, 28, CNU, },
 { 28, 21, 36, 28, CNU, },
 { 43, 28, 43, CNU, 42, },
 { 6, 5, 6, DWS, 7, },
 { 6, 5, 5, 5, 5, },
 { 6, 6, DWS, DWS, 7, },
 { 4, 4, 4, 4, 4, },
 { 11, 4, 4, 4, 4, },
 { 4, 4, 4, 4, 4, },
 { 147, 118, 117, 104, 120, },
 { 147, 124, 136, 102, 133, },
 { 132, 212, 244, 244, 135, },
 { 118, 118, DWO, 108, 118, },
 { 122, 117, 101, 91, 104, },
 { 117, 122, 123, 104, 133, },
});

const CtxSet ContextSetCfg::MmvdStepMvpIdxECM3 = ContextSetCfg::addCtxSet({
// ctx 64
 { CNU, },
 { CNU, },
 { CNU, },
 { DWS, },
 { DWS, },
 { DWS, },
 { DWE, },
 { DWE, },
 { DWE, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
});

const CtxSet ContextSetCfg::GeoMmvdFlag = ContextSetCfg::addCtxSet({
// ctx 65
 { 33, },
 { 26, },
 { 0, },
 { 3, },
 { 3, },
 { 3, },
 { 4, },
 { 4, },
 { 4, },
 { 115, },
 { 115, },
 { 115, },
 { 115, },
 { 115, },
 { 116, },
});

const CtxSet ContextSetCfg::GeoMmvdStepMvpIdx = ContextSetCfg::addCtxSet({
// ctx 66
 { 45, },
 { 60, },
 { 0, },
 { 1, },
 { 1, },
 { 2, },
 { 4, },
 { 4, },
 { 4, },
 { DWO, },
 { 125, },
 { DWO, },
 { 120, },
 { DWO, },
 { 133, },
});

const CtxSet ContextSetCfg::GeoBldFlag = ContextSetCfg::addCtxSet({
// ctx 67 to 70
 { 58, 22, 52, 21, },
 { 60, 51, 59, 44, },
 { 0, 0, 0, 0, },
 { 1, 1, 1, 1, },
 { 1, 1, 1, 1, },
 { 1, 1, 0, 1, },
 { 11, DWE, DWE, 11, },
 { 11, DWE, 11, 11, },
 { 11, DWE, 4, 11, },
 { 132, 116, 116, 117, },
 { DWO, 117, 132, 118, },
 { 118, 117, 117, 117, },
 { 122, 118, 125, 122, },
 { DWO, 116, 117, 117, },
 { 120, 117, 150, DWO, },
});

const CtxSet ContextSetCfg::GeoSubModeIdx = ContextSetCfg::addCtxSet({
// ctx 71 to 75
 { CNU, 36, 44, 29, 29, },
 { 13, 36, 44, 29, 29, },
 { 0, 0, 0, 0, 0, },
 { 6, 7, 5, 5, DWS, },
 { 6, 6, 5, 4, DWS, },
 { 7, 7, 5, 4, DWS, },
 { 4, 4, 4, 4, 4, },
 { 11, 11, 11, 11, 11, },
 { 4, 4, 4, 4, 4, },
 { DWO, 122, 121, 121, 122, },
 { 117, 116, 117, 116, 103, },
 { 122, 122, 120, 124, 124, },
 { 117, 131, 132, 117, 164, },
 { 121, DWO, 118, 136, 182, },
 { 118, 162, 117, 109, 117, },
});

const CtxSet ContextSetCfg::AfMmvdFlag = ContextSetCfg::addCtxSet({
// ctx 76
 { 11, },
 { 11, },
 { 26, },
 { 6, },
 { 6, },
 { 6, },
 { DWE, },
 { 25, },
 { DWE, },
 { 107, },
 { 116, },
 { 116, },
 { 117, },
 { 115, },
 { 116, },
});

const CtxSet ContextSetCfg::AfMmvdIdx = ContextSetCfg::addCtxSet({
// ctx 77 to 80
 { 29, CNU, 28, CNU, },
 { 21, 28, 28, CNU, },
 { 36, 28, 28, CNU, },
 { 9, 9, 9, DWS, },
 { DWS, 9, DWS, DWS, },
 { DWS, 10, DWS, DWS, },
 { DWE, 11, DWE, DWE, },
 { DWE, DWE, 11, DWE, },
 { 11, DWE, 4, DWE, },
 { 202, 164, 202, DWO, },
 { 99, 131, 101, DWO, },
 { 202, 149, 151, DWO, },
 { 100, 106, 102, DWO, },
 { 135, DWO, 121, DWO, },
 { 100, 105, 103, DWO, },
});

const CtxSet ContextSetCfg::AfMmvdOffsetStep = ContextSetCfg::addCtxSet({
// ctx 81 to 86
 { 21, 37, 37, 37, 44, 50, },
 { 13, 29, 29, 37, 52, 50, },
 { 21, 37, 37, 37, 52, 58, },
 { 5, 7, 5, 6, 6, 5, },
 { 6, 6, 5, 5, 5, 6, },
 { 6, 7, 6, 6, 7, 6, },
 { 11, DWE, 4, 11, 4, 25, },
 { DWE, DWE, 11, 11, 4, 25, },
 { 11, DWE, 4, 4, 4, 25, },
 { 118, 138, 166, 150, 235, 106, },
 { 116, 106, 101, 101, 123, 146, },
 { 132, 138, 230, 246, 252, 117, },
 { 116, 103, 100, 100, 117, DWO, },
 { 163, 136, 136, 196, 168, 114, },
 { 194, 106, 100, 115, 170, 129, },
});

const CtxSet ContextSetCfg::AfMmvdOffsetStepECM3 = ContextSetCfg::addCtxSet({
// ctx 87
 { CNU, },
 { CNU, },
 { CNU, },
 { DWS, },
 { DWS, },
 { DWS, },
 { DWE, },
 { DWE, },
 { DWE, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
});

const CtxSet ContextSetCfg::IbcMbvdFlag = ContextSetCfg::addCtxSet({
// ctx 88 to 89
 { 26, CNU, },
 { 26, CNU, },
 { 28, 52, },
 { 5, DWS, },
 { 5, DWS, },
 { 4, 10, },
 { 11, DWE, },
 { 11, DWE, },
 { 4, 11, },
 { 104, 179, },
 { 108, 200, },
 { 102, 179, },
 { 109, 200, },
 { 102, 117, },
 { 109, 162, },
});

const CtxSet ContextSetCfg::IbcMbvdMergeIdx = ContextSetCfg::addCtxSet({
// ctx 90
 { CNU, },
 { 27, },
 { 43, },
 { 7, },
 { 10, },
 { 10, },
 { 11, },
 { DWE, },
 { DWE, },
 { 115, },
 { 116, },
 { 114, },
 { 117, },
 { 145, },
 { 130, },
});

const CtxSet ContextSetCfg::IbcMbvdStepMvpIdx = ContextSetCfg::addCtxSet({
// ctx 91 to 95
 { 34, 43, 43, CNU, CNU, },
 { CNU, 51, 59, CNU, CNU, },
 { CNU, 43, 43, CNU, CNU, },
 { 6, 10, 9, DWS, DWS, },
 { 6, 10, 6, DWS, DWS, },
 { 9, 10, 13, DWS, DWS, },
 { 4, 4, 4, DWE, DWE, },
 { 11, DWE, 4, DWE, DWE, },
 { 11, 4, 4, DWE, DWE, },
 { 107, 117, 100, DWO, DWO, },
 { 118, 103, 137, DWO, DWO, },
 { 91, 168, 104, DWO, DWO, },
 { DWO, 102, DWO, DWO, DWO, },
 { 99, 250, 101, DWO, DWO, },
 { 139, 100, 170, DWO, DWO, },
});

const CtxSet ContextSetCfg::TMMergeFlag = ContextSetCfg::addCtxSet({
// ctx 96 to 97
 { 40, 40, },
 { 26, 25, },
 { 25, 40, },
 { 5, 6, },
 { 6, 6, },
 { 6, 6, },
 { 11, 11, },
 { DWE, 4, },
 { 11, 4, },
 { 117, 99, },
 { 118, 132, },
 { 117, 100, },
 { 118, 124, },
 { 178, 98, },
 { 147, 148, },
});

const CtxSet ContextSetCfg::CiipTMMergeFlag = ContextSetCfg::addCtxSet({
// ctx 98
 { 26, },
 { 26, },
 { 0, },
 { 5, },
 { 5, },
 { 6, },
 { 4, },
 { 4, },
 { 4, },
 { 138, },
 { 117, },
 { 136, },
 { 117, },
 { 118, },
 { 117, },
});

const CtxSet ContextSetCfg::PredMode = ContextSetCfg::addCtxSet({
// ctx 99 to 100
 { 33, 21, },
 { 40, CNU, },
 { 40, 50, },
 { 6, 1, },
 { 6, 1, },
 { 6, 2, },
 { DWE, 11, },
 { DWE, 11, },
 { 11, 11, },
 { 105, 117, },
 { 116, 132, },
 { 106, 117, },
 { 116, 118, },
 { 115, 117, },
 { 116, 147, },
});

const CtxSet ContextSetCfg::MultiRefLineIdx = ContextSetCfg::addCtxSet({
// ctx 101 to 107
 { CNU, CNU, CNU, CNU, CNU, 25, 59, },
 { CNU, CNU, CNU, CNU, CNU, 25, 51, },
 { CNU, CNU, CNU, CNU, CNU, 25, 60, },
 { DWS, DWS, DWS, DWS, DWS, 5, 4, },
 { DWS, DWS, DWS, DWS, DWS, 5, 4, },
 { DWS, DWS, DWS, DWS, DWS, 6, DWS, },
 { DWE, DWE, DWE, DWE, DWE, 11, DWE, },
 { DWE, DWE, DWE, DWE, DWE, 11, DWE, },
 { DWE, DWE, DWE, DWE, DWE, DWE, 25, },
 { DWO, DWO, DWO, DWO, DWO, 102, 116, },
 { DWO, DWO, DWO, DWO, DWO, 132, 195, },
 { DWO, DWO, DWO, DWO, DWO, 105, 147, },
 { DWO, DWO, DWO, DWO, DWO, 132, 242, },
 { DWO, DWO, DWO, DWO, DWO, 107, 131, },
 { DWO, DWO, DWO, DWO, DWO, 132, 162, },
});

const CtxSet ContextSetCfg::IntraLumaMpmFlag = ContextSetCfg::addCtxSet({
// ctx 108
 { 29, },
 { 29, },
 { 22, },
 { 6, },
 { 5, },
 { 6, },
 { 11, },
 { 4, },
 { DWE, },
 { 116, },
 { 115, },
 { 116, },
 { 117, },
 { 116, },
 { 116, },
});

const CtxSet ContextSetCfg::IntraLumaSecondMpmFlag = ContextSetCfg::addCtxSet({
// ctx 109
 { 37, },
 { 37, },
 { 37, },
 { 9, },
 { 9, },
 { 6, },
 { DWE, },
 { 25, },
 { 11, },
 { 136, },
 { 84, },
 { 117, },
 { 118, },
 { 117, },
 { 116, },
});

const CtxSet ContextSetCfg::IntraLumaMPMIdx = ContextSetCfg::addCtxSet({
// ctx 110 to 112
 { 27, CNU, 57, },
 { 28, CNU, CNU, },
 { 42, CNU, 49, },
 { 7, DWS, 6, },
 { 10, DWS, 5, },
 { 5, DWS, 6, },
 { DWE, DWE, 11, },
 { DWE, DWE, 4, },
 { 11, DWE, 4, },
 { 101, DWO, 130, },
 { 132, DWO, 123, },
 { 89, DWO, 123, },
 { 122, DWO, 117, },
 { 102, DWO, 107, },
 { 123, DWO, 121, },
});

const CtxSet ContextSetCfg::IntraLumaSecondMpmIdx = ContextSetCfg::addCtxSet({
// ctx 113 to 117
 { 43, CNU, CNU, CNU, CNU, },
 { 43, CNU, CNU, CNU, CNU, },
 { 59, CNU, CNU, CNU, CNU, },
 { 10, 10, DWS, DWS, DWS, },
 { 9, 9, 9, DWS, DWS, },
 { 9, 10, 13, DWS, DWS, },
 { 4, 4, 4, DWE, DWE, },
 { 11, 11, 11, DWE, DWE, },
 { DWE, 4, 11, DWE, DWE, },
 { 118, 86, 84, DWO, DWO, },
 { 99, 234, 252, DWO, DWO, },
 { 116, 105, 85, DWO, DWO, },
 { DWO, 123, 252, DWO, DWO, },
 { 116, 101, 83, DWO, DWO, },
 { 107, 122, 234, DWO, DWO, },
});

const CtxSet ContextSetCfg::IntraLumaPlanarFlag = ContextSetCfg::addCtxSet({
// ctx 118 to 121
 { 14, 6, 19, 43, },
 { 6, 13, 34, CNU, },
 { 31, 6, 19, 28, },
 { 1, 2, 9, 9, },
 { 3, 2, 9, 9, },
 { 4, 5, DWS, DWS, },
 { DWE, DWE, 25, DWE, },
 { DWE, DWE, 25, DWE, },
 { 11, DWE, DWE, DWE, },
 { 123, 115, 88, 107, },
 { 116, 115, 151, 147, },
 { 117, 115, 102, 117, },
 { 117, 115, 134, 117, },
 { 148, 115, 92, 117, },
 { 118, 242, 136, 103, },
});

const CtxSet ContextSetCfg::CclmModeFlag = ContextSetCfg::addCtxSet({
// ctx 122
 { 27, },
 { 26, },
 { 29, },
 { 1, },
 { 4, },
 { 5, },
 { DWE, },
 { DWE, },
 { 25, },
 { 117, },
 { 131, },
 { 116, },
 { 116, },
 { 108, },
 { 115, },
});

const CtxSet ContextSetCfg::CclmModeIdx = ContextSetCfg::addCtxSet({
// ctx 123
 { 26, },
 { 26, },
 { 36, },
 { 5, },
 { 5, },
 { 5, },
 { 11, },
 { 11, },
 { 11, },
 { 147, },
 { 115, },
 { 117, },
 { 115, },
 { 117, },
 { 115, },
});

const CtxSet ContextSetCfg::IntraChromaPredMode = ContextSetCfg::addCtxSet({
// ctx 124
 { 25, },
 { 25, },
 { 28, },
 { 5, },
 { 5, },
 { 5, },
 { DWE, },
 { 11, },
 { DWE, },
 { 105, },
 { 116, },
 { 117, },
 { 116, },
 { 117, },
 { 108, },
});

const CtxSet ContextSetCfg::DimdChromaMode = ContextSetCfg::addCtxSet({
// ctx 125
 { 41, },
 { 42, },
 { 34, },
 { 9, },
 { DWS, },
 { 5, },
 { 25, },
 { 25, },
 { DWE, },
 { 99, },
 { 116, },
 { 123, },
 { 117, },
 { 116, },
 { 116, },
});

const CtxSet ContextSetCfg::ChromaFusionMode = ContextSetCfg::addCtxSet({
// ctx 126
 { 36, },
 { 43, },
 { 34, },
 { DWS, },
 { DWS, },
 { 4, },
 { 25, },
 { 25, },
 { 11, },
 { 123, },
 { 99, },
 { 118, },
 { 117, },
 { 107, },
 { 106, },
});

const CtxSet ContextSetCfg::DbvChromaMode = ContextSetCfg::addCtxSet({
// ctx 127
 { 25, },
 { 33, },
 { 28, },
 { 1, },
 { 1, },
 { 1, },
 { 25, },
 { 25, },
 { DWE, },
 { 115, },
 { 125, },
 { 117, },
 { 118, },
 { 116, },
 { 117, },
});

const CtxSet ContextSetCfg::MipFlag = ContextSetCfg::addCtxSet({
// ctx 128 to 131
 { 48, 41, 49, 33, },
 { 48, 49, 50, 33, },
 { 40, 41, 34, 25, },
 { 9, 9, DWS, 5, },
 { 9, 10, DWS, 6, },
 { 9, 10, 9, 6, },
 { 11, 4, 11, 11, },
 { 11, 11, DWE, 11, },
 { 4, 4, 11, 4, },
 { 99, 100, 187, 99, },
 { 118, 118, 132, 135, },
 { 118, 100, 172, 99, },
 { 211, 121, 196, 133, },
 { 117, 99, 117, 98, },
 { 116, 122, 132, 134, },
});

const CtxSet ContextSetCfg::TmpFlag = ContextSetCfg::addCtxSet({
// ctx 132 to 138
 { 25, 19, 29, 25, CNU, 13, 49, },
 { 1, 26, 52, 1, 28, 20, 57, },
 { 25, 34, 28, 25, 28, 13, 48, },
 { 2, 1, 1, 2, 1, 9, DWS, },
 { 5, 4, 0, 5, 4, DWS, 7, },
 { 6, 5, 1, 2, 1, DWS, 4, },
 { 11, 11, 4, 11, 11, 25, 25, },
 { 4, DWE, 25, 4, DWE, DWE, 25, },
 { DWE, DWE, 4, DWE, DWE, DWE, 11, },
 { 116, DWO, 120, 115, 132, 251, 83, },
 { 242, 131, 131, 130, DWO, 84, 137, },
 { 99, 101, 106, 99, 132, 245, 85, },
 { 178, 162, 130, 130, 125, 92, 134, },
 { 99, 102, 125, 115, 147, 246, 101, },
 { 146, 162, 147, 130, 118, 86, 165, },
});

const CtxSet ContextSetCfg::TmpIdx = ContextSetCfg::addCtxSet({
// ctx 139 to 141
 { 20, 36, 36, },
 { 20, 20, 20, },
 { 20, 36, 36, },
 { 9, 10, 13, },
 { 6, 10, 9, },
 { 5, 9, 13, },
 { 11, DWE, 25, },
 { 4, 11, 11, },
 { 4, 4, 11, },
 { 118, 186, 234, },
 { 101, 98, 98, },
 { 124, 149, 131, },
 { 108, 101, 120, },
 { 124, 251, 200, },
 { 108, 91, 89, },
});

const CtxSet ContextSetCfg::TmpFusion = ContextSetCfg::addCtxSet({
// ctx 142 to 147
 { 49, 43, 21, 34, 13, 42, },
 { CNU, 28, 21, 19, 27, 50, },
 { 49, 44, 45, 19, 13, 34, },
 { 5, 5, 7, 5, 5, 5, },
 { 10, 10, 4, 5, 6, 5, },
 { 4, 7, 6, 1, 4, DWS, },
 { 25, 11, 4, 25, 25, DWE, },
 { 25, 25, 11, DWE, 11, 11, },
 { DWE, DWE, 11, 11, 11, DWE, },
 { 108, 148, 130, 101, 120, 106, },
 { 124, 101, 100, 117, 146, 118, },
 { 122, 125, 117, 102, 117, 108, },
 { 118, 116, 117, 124, 118, 117, },
 { DWO, 115, 115, 117, 148, 100, },
 { 132, 114, 133, DWO, 148, 117, },
});

#if JVET_AG0136_INTRA_TMP_LIC
const CtxSet ContextSetCfg::TmpLic = ContextSetCfg::addCtxSet
({
  { CNU },
  { CNU },
  { CNU },
  { DWS },
  { DWS },
  { DWS },
  { DWE },
  { DWE },
  { DWE },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  });

const CtxSet ContextSetCfg::ItmpLicIndex = ContextSetCfg::addCtxSet
({
  { CNU, CNU, },
  { CNU, CNU, },
  { CNU, CNU, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWS, DWS, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWE, DWE, },
  { DWO, DWO, },
  { DWO, DWO, },
  { DWO, DWO, },
  { DWO, DWO, },
  { DWO, DWO, },
  { DWO, DWO, },
  });
#endif
const CtxSet ContextSetCfg::MMLMFlag = ContextSetCfg::addCtxSet({
// ctx 148
 { 44, },
 { 45, },
 { 11, },
 { 5, },
 { 5, },
 { 5, },
 { 25, },
 { 25, },
 { 11, },
 { 102, },
 { 154, },
 { 101, },
 { 164, },
 { 102, },
 { 132, },
});

const CtxSet ContextSetCfg::DeltaQP = ContextSetCfg::addCtxSet({
// ctx 149 to 150
 { CNU, CNU, },
 { CNU, CNU, },
 { CNU, CNU, },
 { DWS, DWS, },
 { DWS, DWS, },
 { DWS, DWS, },
 { DWE, DWE, },
 { DWE, DWE, },
 { DWE, DWE, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
});

const CtxSet ContextSetCfg::InterDir = ContextSetCfg::addCtxSet({
// ctx 151 to 158
 { 30, 12, 4, 11, 25, 25, 10, CNU, },
 { 15, 6, 5, 11, 18, 18, 10, CNU, },
 { 0, 0, 0, 0, 0, 0, 0, CNU, },
 { 1, 3, 0, 1, 4, DWS, DWS, DWS, },
 { 2, 1, 1, 2, 5, 10, 12, DWS, },
 { 1, 0, 1, 2, 5, 10, DWS, DWS, },
 { DWE, 25, 11, 4, 4, 25, 25, DWE, },
 { 25, 25, 11, 4, 11, 25, 25, DWE, },
 { DWE, 25, 4, 4, 4, 25, 25, DWE, },
 { 115, 120, 118, 117, 104, 92, 82, DWO, },
 { 216, 114, 133, 164, 244, 131, 244, DWO, },
 { 115, 116, 117, 116, 102, 90, 66, DWO, },
 { 118, 117, 196, 133, 164, 132, 123, DWO, },
 { 115, 126, 124, 124, 104, 84, 82, DWO, },
 { 124, 116, 243, 131, 132, 133, 251, DWO, },
});

const CtxSet ContextSetCfg::RefPic = ContextSetCfg::addCtxSet({
// ctx 159 to 160
 { CNU, CNU, },
 { CNU, CNU, },
 { CNU, CNU, },
 { DWS, DWS, },
 { DWS, DWS, },
 { DWS, DWS, },
 { DWE, DWE, },
 { DWE, DWE, },
 { DWE, DWE, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
 { DWO, DWO, },
});

const CtxSet ContextSetCfg::RefPicLC = ContextSetCfg::addCtxSet({
// ctx 161 to 163
 { 34, 27, 20, },
 { 27, 42, 20, },
 { 27, 58, CNU, },
 { 5, 5, 9, },
 { DWS, 9, DWS, },
 { DWS, 10, DWS, },
 { 4, 4, 4, },
 { 4, DWE, 4, },
 { 4, DWE, 11, },
 { 118, 121, 251, },
 { DWO, 118, 139, },
 { 117, 121, 233, },
 { 122, 121, 124, },
 { 103, 99, 131, },
 { 107, 138, 252, },
});

const CtxSet ContextSetCfg::SubblockMergeFlag = ContextSetCfg::addCtxSet({
// ctx 164 to 166
 { 25, 20, 22, },
 { 40, 19, 21, },
 { 40, 27, 44, },
 { 6, 5, 5, },
 { 6, 5, 5, },
 { 6, 5, 5, },
 { 11, 11, DWE, },
 { 11, 11, DWE, },
 { 11, 4, DWE, },
 { 131, 117, 124, },
 { 131, 148, DWO, },
 { 116, 117, 124, },
 { DWO, 132, 123, },
 { 118, 117, 124, },
 { 117, 132, 121, },
});

const CtxSet ContextSetCfg::BMMergeFlag = ContextSetCfg::addCtxSet({
// ctx 167 to 170
 { 56, 50, 58, 28, },
 { 49, 50, 58, 28, },
 { 0, 0, 0, 0, },
 { 5, 5, 4, 5, },
 { 5, 6, 6, 5, },
 { 6, 6, 9, 5, },
 { DWE, 11, 4, 11, },
 { DWE, DWE, 11, 11, },
 { DWE, 11, 4, 11, },
 { 124, 164, 141, 116, },
 { 117, 163, 101, 116, },
 { 124, 123, 135, 117, },
 { 117, 116, 102, 116, },
 { 116, 162, 163, 117, },
 { 116, 120, 169, 107, },
});

const CtxSet ContextSetCfg::affBMFlag = ContextSetCfg::addCtxSet({
// ctx 171 to 172
 { 40, 50, },
 { 48, 50, },
 { 0, 0, },
 { 5, 1, },
 { 5, 4, },
 { 5, 1, },
 { DWE, 4, },
 { DWE, 11, },
 { 11, 4, },
 { 99, 120, },
 { 121, 117, },
 { 99, 133, },
 { 124, 117, },
 { 99, 118, },
 { 132, 118, },
});

const CtxSet ContextSetCfg::rribcFlipType = ContextSetCfg::addCtxSet({
// ctx 173 to 176
 { 39, 39, 39, CNU, },
 { 39, 39, 0, CNU, },
 { 39, 39, 39, CNU, },
 { 0, 0, 0, DWS, },
 { 0, 0, 0, DWS, },
 { 0, 0, 0, DWS, },
 { 25, 25, 25, DWE, },
 { 25, 25, 25, DWE, },
 { 25, 25, 25, DWE, },
 { 116, 116, 116, DWO, },
 { 116, 116, 116, DWO, },
 { 116, 116, 116, DWO, },
 { 116, 116, 116, DWO, },
 { 116, 116, 116, DWO, },
 { 116, 116, 116, DWO, },
});

const CtxSet ContextSetCfg::bvOneZeroComp = ContextSetCfg::addCtxSet({
// ctx 177 to 180
 { 41, 50, 51, 27, },
 { 49, 59, 0, 50, },
 { 34, 42, CNU, 19, },
 { 1, 6, 4, 5, },
 { 6, 7, 13, 7, },
 { 6, 6, 2, 5, },
 { 4, DWE, 25, 11, },
 { 25, 4, 4, 11, },
 { 25, 4, 4, 11, },
 { 117, 99, 99, 101, },
 { 118, 133, 137, 243, },
 { 107, 105, 99, 107, },
 { 118, 116, 202, 116, },
 { 101, 100, 115, 103, },
 { 115, 116, 235, 147, },
});

const CtxSet ContextSetCfg::AffineFlag = ContextSetCfg::addCtxSet({
// ctx 181 to 183
 { 26, 20, 14, },
 { 26, 12, 13, },
 { 34, 20, 36, },
 { 5, 2, 0, },
 { 5, 2, 1, },
 { 6, 3, 1, },
 { 11, 11, 4, },
 { 4, 11, DWE, },
 { 4, 4, 4, },
 { 116, 115, 117, },
 { 108, 116, 117, },
 { 101, 116, 117, },
 { 117, DWO, 117, },
 { 105, 117, 117, },
 { 116, 117, 117, },
});

const CtxSet ContextSetCfg::AffineType = ContextSetCfg::addCtxSet({
// ctx 184
 { 27, },
 { 27, },
 { 26, },
 { 0, },
 { 5, },
 { 7, },
 { 4, },
 { 4, },
 { DWE, },
 { DWO, },
 { 181, },
 { 104, },
 { 133, },
 { 98, },
 { 137, },
});

const CtxSet ContextSetCfg::AffMergeIdx = ContextSetCfg::addCtxSet({
// ctx 185 to 187
 { 22, 21, 22, },
 { 6, 28, 29, },
 { 6, 36, 37, },
 { 1, 5, 9, },
 { 0, DWS, 9, },
 { 1, 5, 9, },
 { DWE, 11, 11, },
 { DWE, DWE, DWE, },
 { DWE, 4, 11, },
 { DWO, 132, 132, },
 { 132, 117, 100, },
 { 126, 148, 133, },
 { 132, 117, 101, },
 { DWO, 180, 148, },
 { 132, 117, 102, },
});

const CtxSet ContextSetCfg::LICFlag = ContextSetCfg::addCtxSet({
#if JVET_AG0276_LIC_SLOPE_ADJUST
  {  11, CNU, CNU, },
  {  11, CNU, CNU, },
  {  11, CNU, CNU, },
  {   4, DWS, DWS, },
  {   5, DWS, DWS, },
  {   6, DWS, DWS, },
  {   4, DWE, DWE, },
  {   4, DWE, DWE, },
  {   4, DWE, DWE, },
  { 102, DWO, DWO, },
  { 133, DWO, DWO, },
  { 101, 119, 119, },
  { 133, 119, 119, },
  { 106, 119, 119, },
  { 211, 119, 119, },
#else
// ctx 188
 { 11, },
 { 11, },
 { 11, },
 { 4, },
 { 5, },
 { 6, },
 { 4, },
 { 4, },
 { 4, },
 { 102, },
 { 133, },
 { 101, },
 { 133, },
 { 106, },
 { 211, },
#endif
});

#if JVET_AG0276_LIC_SLOPE_ADJUST
const CtxSet ContextSetCfg::LicDelta = ContextSetCfg::addCtxSet
({
  {  CNU, CNU, },
  {  CNU, CNU, },
  {  CNU, CNU, },
  {  DWS, DWS, },
  {  DWS, DWS, },
  {  DWS, DWS, },
  {  DWE, DWE, },
  {  DWE, DWE, },
  {  DWE, DWE, },
  {  DWO, DWO, },
  {  DWO, DWO, },
  {  119, 119, },
  {  119, 119, },
  });
#endif

const CtxSet ContextSetCfg::BcwIdx = ContextSetCfg::addCtxSet({
// ctx 189
 { 5, },
 { 5, },
 { 0, },
 { 1, },
 { 1, },
 { 5, },
 { 11, },
 { 4, },
 { 11, },
 { 228, },
 { 117, },
 { 133, },
 { 117, },
 { 122, },
 { 131, },
});

const CtxSet ContextSetCfg::Mvd = ContextSetCfg::addCtxSet({
// ctx 190 to 191
 { 44, 44, },
 { 44, 36, },
 { 44, 58, },
 { 2, 2, },
 { 3, 2, },
 { 3, 2, },
 { 4, 4, },
 { 4, 4, },
 { 4, 4, },
 { 168, 116, },
 { 156, 115, },
 { 139, 116, },
 { 138, 115, },
 { 134, 116, },
 { 123, 115, },
});

const CtxSet ContextSetCfg::Bvd = ContextSetCfg::addCtxSet({
// ctx 192 to 203
 { 38, 38, 38, 37, 36, CNU, 28, 38, 37, 36, 27, 42, },
 { 38, 38, 38, 14, 20, 43, 29, 30, 6, 59, 51, 50, },
 { 53, 38, 38, 30, 37, 36, 29, 52, 29, 37, 28, 42, },
 { 5, 9, 5, 5, 2, 5, 2, 5, 1, 0, 1, 1, },
 { 7, 10, 7, 7, 7, 7, 3, 6, 5, 1, 3, 2, },
 { 6, 9, 10, 7, 7, 2, 1, 5, 5, 1, 3, 1, },
 { DWE, 11, 4, 11, 4, DWE, DWE, DWE, 11, 4, 11, 11, },
 { DWE, 11, 11, DWE, DWE, DWE, 25, 11, DWE, 11, 11, 11, },
 { 25, DWE, 25, DWE, DWE, 11, 11, 25, 25, DWE, DWE, DWE, },
 { 242, 117, 124, 148, 133, 118, 117, 116, DWO, DWO, 123, 118, },
 { 108, 84, 102, 115, 115, 108, 120, 116, 117, 117, 117, 117, },
 { 241, 131, 124, 121, 124, 132, 118, 118, 118, 126, 125, 126, },
 { 107, 100, 101, 102, 115, 102, 120, 109, 102, 117, 116, 117, },
 { 161, 242, 225, 120, 130, 227, 118, 117, 117, 125, 123, 125, },
 { 106, 91, 98, 103, 105, 117, 120, 108, 100, 117, 116, 116, },
});

const CtxSet ContextSetCfg::MvsdIdxMVDMSB = ContextSetCfg::addCtxSet({
// ctx 204 to 235
 { 34, 34, 41, 41, 33, 26, 26, 34, 41, 41, 48, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 34, 34, 34, 41, 41, 41, 34, 34, 41, 41, 48, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 42, 34, 34, 26, 26, 26, 0, 34, 56, 56, 58, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 12, 13, 13, 12, 10, 6, 9, 13, 13, 12, 10, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 12, 13, 13, 13, 13, 10, 13, 12, 13, 13, 12, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 12, 13, 13, 13, 13, 9, DWS, 13, 13, 10, 13, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 4, 4, DWE, DWE, 11, 11, 25, 4, 4, 11, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 11, 4, 4, 4, 25, DWE, 25, 4, 25, 4, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 11, 4, 4, 4, 25, 25, 25, 4, DWE, DWE, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 235, 89, 114, 98, 81, 98, 97, 86, 129, 83, 97, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 84, DWO, 99, 118, 116, 146, 249, 105, 152, 228, 146, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 219, 120, 84, 83, 67, 82, 96, 138, 81, 83, 65, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 85, 100, 115, 116, 146, 242, 170, 91, 234, 164, 195, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 187, 121, 86, 72, 69, 81, 84, 90, 89, 82, 65, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 99, 105, 106, 199, 117, 116, 131, 100, 134, 234, 212, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
});

const CtxSet ContextSetCfg::MvsdIdxBVDMSB = ContextSetCfg::addCtxSet({
// ctx 236 to 251
 { 34, 41, 26, 26, 42, 41, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 57, 41, 2, 40, 57, 33, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 34, 41, 34, 41, 42, 34, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 13, 9, 13, 12, 13, 13, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 13, 10, DWS, 13, 13, 13, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 13, 10, 13, 13, 13, 10, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 4, 4, 25, DWE, 4, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 4, 4, 11, 4, 4, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 4, 4, 4, 4, 4, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 100, 83, 202, 84, 101, 89, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 89, 100, 106, 117, 167, 102, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 202, 83, 252, 72, 135, 83, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 87, 115, 88, 164, 103, 181, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 201, 82, 218, 72, 106, 83, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 90, 99, 83, 132, 105, 215, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
});

const CtxSet ContextSetCfg::MvsdIdx = ContextSetCfg::addCtxSet({
// ctx 252 to 255
 { 34, 33, 34, 33, },
 { 41, 48, 41, 33, },
 { 41, 48, 41, 48, },
 { 13, 10, 12, 13, },
 { 13, 12, 13, 10, },
 { 13, 10, 13, 10, },
 { 11, 4, 4, DWE, },
 { 4, 25, DWE, 25, },
 { 4, 11, 11, 11, },
 { 73, 82, 67, 96, },
 { 102, 105, 102, 99, },
 { 74, 81, 68, 65, },
 { 106, 118, 118, 146, },
 { 85, 81, 68, 96, },
 { 100, 131, 99, 116, },
});

const CtxSet ContextSetCfg::MvsdIdxIBC = ContextSetCfg::addCtxSet({
// ctx 256 to 257
 { 41, 33, },
 { 41, 56, },
 { 34, 41, },
 { 13, 10, },
 { 13, 10, },
 { 12, 10, },
 { 4, 4, },
 { 4, 4, },
 { 11, 4, },
 { 234, 82, },
 { 115, 131, },
 { 121, 82, },
 { 131, 116, },
 { 105, 83, },
 { 211, 116, },
});

const CtxSet ContextSetCfg::MultiHypothesisFlag = ContextSetCfg::addCtxSet({
// ctx 258 to 260
 { 17, 50, 51, },
 { 10, 42, 51, },
 { 0, 0, 0, },
 { 1, 4, 1, },
 { 2, 5, 2, },
 { 2, 5, 2, },
 { 4, 4, 4, },
 { 11, 11, 4, },
 { 11, 11, 4, },
 { 117, 105, 117, },
 { 132, 148, 132, },
 { 117, 123, 103, },
 { 132, 116, 116, },
 { 116, 108, 116, },
 { 131, 131, 131, },
});

const CtxSet ContextSetCfg::MHRefPic = ContextSetCfg::addCtxSet({
// ctx 261 to 262
 { 43, 34, },
 { 43, 34, },
 { 0, 0, },
 { 1, 0, },
 { 1, 1, },
 { 4, 0, },
 { 11, 4, },
 { 4, 11, },
 { 4, 4, },
 { 180, 213, },
 { 117, 117, },
 { 133, 149, },
 { 116, 118, },
 { DWO, 150, },
 { 116, 117, },
});

const CtxSet ContextSetCfg::MHWeight = ContextSetCfg::addCtxSet({
// ctx 263 to 264
 { 51, CNU, },
 { 43, CNU, },
 { 0, CNU, },
 { 0, DWS, },
 { 0, DWS, },
 { 0, DWS, },
 { 4, DWE, },
 { 4, DWE, },
 { 4, DWE, },
 { 135, DWO, },
 { 150, DWO, },
 { 136, DWO, },
 { 229, DWO, },
 { 126, DWO, },
 { 230, DWO, },
});

const CtxSet ContextSetCfg::BDPCMMode = ContextSetCfg::addCtxSet({
// ctx 265 to 268
 { 25, 21, 0, 50, },
 { 0, CNU, 0, 59, },
 { 11, 50, 0, 34, },
 { 1, 0, 1, 0, },
 { 5, 4, 1, 13, },
 { 1, 1, DWS, 0, },
 { 11, DWE, 25, 25, },
 { DWE, DWE, 25, 4, },
 { DWE, 25, DWE, 4, },
 { 122, 118, 115, 132, },
 { 148, 117, 183, 116, },
 { 102, 109, 82, 155, },
 { DWO, 147, 149, 116, },
 { 118, 123, 82, 138, },
 { 122, 131, 151, 116, },
});

const CtxSet ContextSetCfg::QtRootCbf = ContextSetCfg::addCtxSet({
// ctx 269
 { 5, },
 { 5, },
 { 6, },
 { 1, },
 { 2, },
 { 1, },
 { 4, },
 { 4, },
 { 4, },
 { 118, },
 { 118, },
 { 117, },
 { 118, },
 { 132, },
 { 117, },
});

const CtxSet ContextSetCfg::ACTFlag = ContextSetCfg::addCtxSet({
// ctx 270
 { CNU, },
 { CNU, },
 { CNU, },
 { DWS, },
 { DWS, },
 { DWS, },
 { DWE, },
 { DWE, },
 { DWE, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
});

const CtxSet ContextSetCfg::QtCbf[3] = {
  ContextSetCfg::addCtxSet({
// ctx 271 to 274
 { 23, 38, 5, 7, },
 { 31, 7, 13, 7, },
 { 7, 34, 12, 7, },
 { 5, 1, 5, DWS, },
 { 6, 0, 5, 6, },
 { 5, 1, 5, 5, },
 { DWE, 11, 4, 11, },
 { DWE, 11, 4, 4, },
 { DWE, DWE, 4, 4, },
 { 131, 118, 100, 131, },
 { 108, 131, 164, 229, },
 { 131, 117, 100, 115, },
 { 116, 131, 243, 252, },
 { 131, 117, 100, 115, },
 { 116, 131, 243, 252, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 275 to 276
 { 25, 54, },
 { 25, 5, },
 { 4, 30, },
 { 5, 0, },
 { 5, 6, },
 { 5, 2, },
 { DWE, 25, },
 { 25, 25, },
 { DWE, 11, },
 { 101, 132, },
 { 242, 117, },
 { 101, 117, },
 { 131, 180, },
 { 101, 116, },
 { 131, 139, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 277 to 279
 { 17, 36, 30, },
 { 9, 28, 3, },
 { 33, 13, 22, },
 { 3, 1, 1, },
 { 2, 1, 10, },
 { 2, 2, 2, },
 { DWE, DWE, 25, },
 { DWE, DWE, DWE, },
 { 11, DWE, 11, },
 { 115, 118, 115, },
 { 116, 118, 115, },
 { 116, 118, 131, },
 { 117, 118, 114, },
 { 115, 118, 147, },
 { 131, DWO, 114, },
  }),
};

const CtxSet ContextSetCfg::SigCoeffGroup[2] = {
  ContextSetCfg::addCtxSet({
// ctx 280 to 281
 { 25, 30, },
 { 25, 45, },
 { 18, 31, },
 { DWS, 5, },
 { DWS, 5, },
 { 4, 5, },
 { 25, DWE, },
 { 25, DWE, },
 { 11, DWE, },
 { 92, 148, },
 { 108, 133, },
 { 108, 132, },
 { 108, DWO, },
 { 103, 132, },
 { 102, DWO, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 282 to 283
 { 9, 45, },
 { 25, 45, },
 { 25, 30, },
 { DWS, 9, },
 { 10, 13, },
 { 5, 9, },
 { 25, 11, },
 { DWE, DWE, },
 { 11, 25, },
 { 82, 251, },
 { 149, 235, },
 { 92, 252, },
 { 131, 140, },
 { 116, 218, },
 { 131, 131, },
  }),
};

#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const CtxSet ContextSetCfg::SigFlagL[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 284 to 301
 { 3, 26, 27, 28, 21, 29, 26, 27, 28, 37, 37, 30, CNU, 44, 37, 30, 30, 38, },
 { 18, 41, 27, 21, 21, 6, 19, 42, 28, 37, 37, 30, 27, 37, 44, 30, 38, 38, },
 { 34, 26, 19, 28, 13, 21, 42, 27, 28, 14, 22, 30, 43, 52, 37, 38, 38, 38, },
 { 13, 10, 10, 10, 13, 13, DWS, 13, 10, 10, 10, 13, 10, 10, DWS, 10, 10, 13, },
 { 12, 10, 9, 12, 10, 13, DWS, 10, 10, 10, 10, 12, 12, 10, 12, 10, 10, 13, },
 { 12, 9, 10, 10, 13, 12, DWS, 13, 13, 13, 13, 13, 12, 9, 9, 13, 13, 13, },
 { 25, 4, 4, 4, 4, 4, 25, 4, 4, 4, 4, 4, 25, 11, 4, 4, DWE, 25, },
 { 25, 4, 4, DWE, 4, 4, 25, 4, 4, 4, 4, 11, 25, DWE, 25, 11, 11, 25, },
 { 25, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 25, 4, 4, 4, 4, DWE, },
 { 234, 90, 101, 131, 90, 199, 252, 88, 170, 117, 218, 121, 250, 215, 166, 138, 250, 234, },
 { 113, 201, 134, 121, 101, 98, 99, 234, 122, 250, 90, 88, 208, 226, 102, 101, 83, 66, },
 { 210, 101, 107, 115, 90, 106, 252, 210, 243, 120, 164, 122, 252, 195, 118, 147, 243, 169, },
 { 179, 139, DWO, 155, 114, 146, 102, 153, 120, 136, 106, 132, 101, 155, 244, 154, 90, 67, },
 { 114, 91, 100, 99, 118, 202, 236, 84, 227, 147, 120, 136, 131, 101, 118, 115, 117, 234, },
 { 170, 155, 134, 218, 83, 67, 108, 234, 231, 218, 106, 89, 102, 251, 139, 234, 105, 66, },
    }),
  ContextSetCfg::addCtxSet
  ({
// ctx 302 to 313
 { 18, 42, 27, 13, 29, 44, 19, 52, 45, 38, 38, 46, },
 { 18, 27, 20, 21, 21, 29, 19, 52, 30, 30, 38, 46, },
 { 42, 34, 20, 21, 29, 37, 43, 53, 53, 38, 38, 46, },
 { 13, 13, 13, 12, 10, 1, 6, 5, DWS, DWS, 9, 9, },
 { 13, 13, 12, 12, DWS, 12, 7, DWS, DWS, DWS, DWS, 10, },
 { 12, 13, 9, 13, 13, DWS, DWS, DWS, 12, DWS, 13, 10, },
 { DWE, 11, 11, 25, 25, 4, 4, 4, DWE, 25, 25, 25, },
 { DWE, 11, 11, 25, 25, 4, 4, 11, DWE, 25, 25, 25, },
 { 25, 4, 4, DWE, 4, 4, DWE, 4, 11, 4, DWE, DWE, },
 { 234, 131, 151, 99, 99, 117, 122, 132, 118, 243, 133, 213, },
 { 163, 137, 233, 235, 234, 115, 212, 218, 245, 252, 152, 81, },
 { 163, 130, 102, 101, 108, 219, 122, 148, 116, 118, 136, 211, },
 { 118, 153, 251, 197, 148, 99, 221, 122, 204, 252, 120, 86, },
 { 148, 83, 101, 87, 106, 220, 140, 103, 116, 108, 165, 151, },
 { 150, 234, 251, 234, 117, 83, 252, 172, 235, 252, 101, 82, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 314 to 331
 { 28, 37, 45, 46, 46, 46, 52, 54, 61, 39, 39, 39, 5, 39, 39, 39, 39, 39, },
 { 27, 45, 30, 46, 46, 54, 43, 54, 54, 39, 39, 39, 7, 39, 39, 39, 39, 39, },
 { 28, 29, 45, 54, 54, 54, 29, 39, 39, 39, 39, 39, 31, 39, 39, 39, 39, 39, },
 { 9, 12, 13, 9, DWS, 9, DWS, DWS, DWS, 0, 0, 0, 3, 0, 0, 0, 0, 0, },
 { 12, 13, 13, 12, 10, 5, 10, DWS, DWS, 0, 4, 4, 7, 0, 0, 0, 0, 0, },
 { DWS, 12, 13, 12, 12, 9, 13, DWS, DWS, 4, 0, 0, 7, 0, 0, 0, 0, 0, },
 { 25, DWE, DWE, 4, 11, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 25, 25, 25, 11, 4, 4, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 11, 4, DWE, 11, 11, 25, 25, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, },
 { 251, 107, 185, 151, 236, 235, 252, 252, 252, 216, 250, 254, 216, 235, 238, 237, 237, 238, },
 { 99, 235, 179, 250, 83, 82, 86, 242, 114, 116, 116, 116, 113, 116, 116, 116, 116, 132, },
 { 252, 181, 180, 150, 186, 187, 250, 252, 185, 198, 169, 170, 180, 238, 252, 237, 183, 253, },
 { 105, 235, 202, 218, 85, 84, 106, 242, 178, 99, 99, 99, 96, 116, 132, 116, 116, 116, },
 { 252, 117, 163, 229, 200, 234, 234, 252, 250, 183, 252, 232, 168, 238, 253, 252, 252, 252, },
 { 103, 235, 234, 234, 85, 83, 105, 242, 114, 99, 116, 116, 96, 116, 116, 116, 116, 116, },
    }),
  ContextSetCfg::addCtxSet
  ({
// ctx 332 to 343
 { CNU, 45, 38, 31, 54, 31, 7, 39, 39, 39, 39, 39, },
 { 49, 38, 53, 54, 54, 54, 23, 39, 39, 39, 39, 39, },
 { 14, 46, 38, 39, 39, 39, 39, 39, 39, 39, 39, 39, },
 { DWS, 12, 4, 0, 0, 13, 0, 0, 0, 0, 0, 0, },
 { 13, 12, DWS, 4, 3, 13, 5, 0, 0, 0, 0, 0, },
 { DWS, 12, 13, 4, 0, 0, 4, 0, 0, 0, 0, 0, },
 { 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 25, DWE, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, },
 { 25, DWE, 25, DWE, 25, 4, 25, 25, 25, 25, 25, 25, },
 { 252, 235, 252, 222, 238, 232, 212, 238, 116, 238, 238, 238, },
 { 194, 233, 99, 116, 116, 130, 116, 116, 116, 116, 116, 132, },
 { 229, 202, 251, 168, 230, 252, 165, 253, 254, 132, 151, 252, },
 { 225, 235, 114, 99, 99, 114, 98, 116, 116, 132, 116, 116, },
 { 165, 186, 234, 201, 254, 254, 252, 254, 253, 254, 254, 252, },
 { 194, 235, 101, 99, 116, 116, 99, 116, 116, 132, 116, 116, },
    }),
  ContextSetCfg::addCtxSet
  ({
// ctx 344 to 361
 { 20, 54, 39, 39, 39, 39, 59, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { CNU, 39, 54, 39, 39, 39, 28, 39, 39, 39, 39, 39, CNU, 39, 39, 39, 39, 39, },
 { 27, 39, 39, 39, 39, 39, 51, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { 10, DWS, DWS, 12, 12, 7, 12, 4, 4, 1, 4, 1, 7, 0, 0, 0, 0, 0, },
 { 10, DWS, DWS, 12, 12, 13, 13, 4, 4, 4, 4, 4, DWS, 0, 0, 0, 0, 0, },
 { 10, DWS, 12, 12, 12, 13, 12, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
 { 25, 25, 25, 25, 25, 4, 25, DWE, 25, 25, 25, 25, 4, 25, 25, 25, 25, 25, },
 { 25, 25, DWE, 25, 25, 11, 25, DWE, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, },
 { 25, 25, 25, 25, 25, DWE, 4, 25, 25, 25, 25, 25, 4, 25, 25, 25, 25, 25, },
 { 231, 251, 198, 202, 235, 121, 219, 237, 202, 189, 253, 231, 98, 238, 253, 238, 253, 252, },
 { 144, 252, 181, 226, 97, 115, 97, 99, 99, 115, 99, 115, 96, 116, 116, 116, 116, 116, },
 { 210, 251, 251, 153, 234, 204, 233, 218, 167, 182, 171, 170, 98, 252, 253, 238, 122, 252, },
 { 123, 252, 228, 162, 144, 89, DWO, 99, 99, 99, 99, 99, 96, 116, 116, 116, 116, 116, },
 { 217, 252, 235, 182, 235, 234, 202, 254, 221, 205, 190, 250, 116, 238, 252, 254, 254, 238, },
 { 114, 252, 180, 226, 97, 128, 149, 116, 99, 116, 116, 116, 116, 116, 116, 116, 116, 116, },
    }),
  ContextSetCfg::addCtxSet
  ({
// ctx 362 to 373
 { 27, 38, 54, 39, 39, 39, 33, 39, 39, 39, 39, 39, },
 { 27, 38, 62, 39, 39, 39, 5, 39, 39, 39, 39, 39, },
 { 20, 39, 39, 39, 39, 39, 51, 39, 39, 39, 39, 39, },
 { 10, DWS, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, },
 { 10, DWS, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, },
 { 13, DWS, 4, 0, 0, 0, 6, 0, 0, 0, 0, 0, },
 { DWE, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, DWE, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, },
 { 4, DWE, DWE, 4, 4, 11, 25, 25, 25, 25, 25, 25, },
 { 233, 252, 205, 252, 236, 235, 245, 237, 253, 116, 235, 235, },
 { 113, 236, 116, 148, 148, 116, 115, 116, 116, 116, 116, 116, },
 { 231, 252, 204, 236, 254, 254, 164, 253, 252, 253, 254, 237, },
 { 129, 252, 103, 99, 116, 116, 113, 132, 132, 116, 116, 116, },
 { 217, 252, 253, 237, 254, 254, 215, 253, 252, 254, 252, 253, },
 { 70, 252, 101, 116, 116, 116, 113, 116, 116, 116, 116, 116, },
   }),
};
#else
const CtxSet ContextSetCfg::SigFlagL[6] = {
  ContextSetCfg::addCtxSet({
    { 10, 26, 27, 21, 25, 27, 28, 37, 34, 44, 44, 38 },
    { 10, 41, 42, 21, 19, 42, 28, 37, 27, 37, 44, 38 },
    { 26, 26, 19, 28, 34, 27, 28, 22, 43, 52, 37, 38 },
    { 12, 10, 10, 10, 8, 10, 10, 10, 8, 10, 8, 9 },
    { 12, 10, 9, 13, 8, 10, 10, 10, 8, 10, 8, 10 },
    { 12, 9, 9, 10, 8, 13, 13, 13, 9, 9, 9, 10 },
    { 25, 4, 11, 11, 25, 4, 11, 4, 25, 18, 11, 4 },
    { 25, 4, 4, 18, 25, 4, 4, 4, 25, 18, 11, 11 },
    { 25, 4, 4, 4, 4, 4, 11, 11, 18, 4, 4, 4 },
    { 231, 100, 114, 100, 251, 147, 152, 122, 201, 170, 186, 119 },
    { 102, 138, 138, 233, 100, 137, 122, 138, 101, 99, 100, 106 },
    { 234, 119, 227, 89, 252, 118, 199, 122, 252, 135, 251, 137 },
    { 84, 137, 121, 234, 114, 132, 227, 122, 83, 104, 115, 90 },
    { 131, 101, 117, 116, 252, 85, 211, 101, 131, 163, 132, 121 },
    { 148, 152, 118, 118, 132, 186, 164, 234, 182, 235, 135, 122 },
  }),
  ContextSetCfg::addCtxSet({
    { 18, 42, 35, 21, 34, 52, 53, 38 },
    { 11, 27, 20, 21, 19, 52, 30, 38 },
    { 26, 34, 20, 21, 43, 53, 53, 38 },
    { 8, 13, 9, 13, 6, 5, 8, 10 },
    { 13, 13, 12, 12, 5, 5, 8, 8 },
    { 12, 13, 9, 13, 8, 8, 9, 9 },
    { 11, 18, 4, 25, 18, 11, 18, 25 },
    { 18, 11, 11, 25, 4, 4, 18, 25 },
    { 25, 11, 4, 11, 25, 4, 4, 4 },
    { 251, 215, 123, 99, 138, 132, 117, 146 },
    { 118, 120, 171, 186, 131, 117, 139, 116 },
    { 233, 115, 101, 84, 140, 132, 117, 147 },
    { 106, 133, 235, 235, 138, 124, 153, 252 },
    { 235, 86, 104, 163, 118, 164, 132, 132 },
    { 107, 149, 251, 147, 252, 108, 251, 154 },
  }),
  ContextSetCfg::addCtxSet({
    { 35, 37, 45, 46, 28, 54, 61, 39, 43, 39, 39, 39 },
    { 27, 30, 38, 46, 51, 54, 54, 39, 22, 39, 39, 39 },
    { 20, 37, 45, 54, 36, 39, 39, 39, 38, 39, 39, 39 },
    { 12, 13, 13, 8, 10, 8, 8, 4, 0, 0, 0, 0 },
    { 12, 13, 12, 8, 7, 8, 8, 5, 0, 0, 0, 0 },
    { 8, 12, 13, 12, 13, 8, 8, 4, 0, 0, 0, 0 },
    { 25, 11, 18, 11, 25, 25, 25, 25, 18, 25, 25, 25 },
    { 25, 25, 25, 4, 25, 25, 25, 25, 25, 25, 25, 25 },
    { 11, 11, 25, 11, 25, 25, 25, 25, 25, 25, 25, 25 },
    { 235, 234, 232, 152, 250, 252, 235, 173, 212, 238, 116, 116 },
    { 162, 234, 162, 101, 112, 147, 99, 99, 116, 116, 116, 116 },
    { 235, 153, 195, 135, 250, 252, 220, 150, 230, 116, 238, 235 },
    { 82, 234, 235, 121, 97, 136, 115, 98, 116, 116, 116, 116 },
    { 187, 149, 228, 213, 234, 252, 231, 154, 133, 116, 123, 238 },
    { 228, 235, 181, 187, 98, 242, 242, 99, 148, 116, 116, 116 },
  }),
  ContextSetCfg::addCtxSet({
    { 34, 37, 38, 31, 6, 39, 39, 39 },
    { 34, 38, 53, 54, 38, 39, 39, 39 },
    { 13, 46, 38, 39, 54, 39, 39, 39 },
    { 8, 12, 8, 4, 0, 0, 0, 0 },
    { 10, 12, 8, 4, 12, 0, 0, 0 },
    { 8, 12, 13, 4, 0, 0, 0, 0 },
    { 25, 25, 18, 18, 25, 25, 25, 25 },
    { 18, 18, 25, 25, 25, 25, 25, 25 },
    { 25, 18, 25, 25, 25, 25, 25, 25 },
    { 235, 235, 252, 151, 164, 252, 116, 238 },
    { 115, 214, 83, 99, 132, 116, 116, 116 },
    { 202, 235, 252, 221, 235, 235, 116, 116 },
    { 129, 235, 83, 100, 65, 116, 116, 116 },
    { 252, 235, 186, 246, 148, 253, 254, 133 },
    { 242, 235, 101, 102, 132, 116, 116, 116 },
  }),
  ContextSetCfg::addCtxSet({
    { 35, 54, 39, 39, 35, 39, 39, 39, 0, 39, 39, 39 },
    { 43, 39, 54, 39, 35, 39, 39, 39, 35, 39, 39, 39 },
    { 19, 39, 39, 39, 43, 39, 39, 39, 0, 39, 39, 39 },
    { 12, 9, 8, 8, 13, 4, 4, 4, 0, 0, 0, 0 },
    { 12, 8, 8, 12, 13, 4, 4, 4, 8, 0, 0, 0 },
    { 5, 8, 12, 12, 13, 0, 0, 0, 0, 0, 0, 0 },
    { 25, 25, 25, 18, 4, 25, 25, 25, 4, 25, 25, 25 },
    { 25, 25, 11, 25, 25, 18, 25, 25, 18, 25, 25, 25 },
    { 18, 25, 25, 25, 18, 25, 18, 18, 25, 25, 25, 25 },
    { 197, 251, 252, 218, 233, 188, 205, 221, 116, 116, 235, 252 },
    { 114, 250, 118, 179, 146, 99, 99, 99, 116, 116, 116, 116 },
    { 235, 235, 236, 202, 234, 253, 215, 215, 119, 116, 235, 235 },
    { 68, 251, 251, 162, 65, 115, 99, 99, 119, 116, 116, 116 },
    { 119, 252, 235, 155, 184, 249, 174, 173, 116, 235, 135, 254 },
    { 123, 251, 227, 114, 209, 116, 116, 116, 116, 116, 132, 116 },
  }),
  ContextSetCfg::addCtxSet({
    { 19, 30, 54, 39, 42, 39, 39, 39 },
    { 27, 38, 62, 39, 33, 39, 39, 39 },
    { 20, 39, 39, 39, 61, 39, 39, 39 },
    { 10, 8, 4, 0, 0, 0, 0, 0 },
    { 10, 8, 0, 0, 1, 0, 0, 0 },
    { 9, 8, 8, 0, 1, 0, 0, 0 },
    { 25, 18, 25, 25, 25, 25, 25, 25 },
    { 18, 25, 25, 25, 11, 25, 25, 25 },
    { 18, 18, 25, 4, 25, 25, 25, 25 },
    { 216, 236, 253, 253, 212, 116, 253, 238 },
    { 192, 233, 100, 148, 116, 116, 116, 116 },
    { 248, 252, 254, 254, 115, 116, 238, 235 },
    { 100, 203, 117, 116, 221, 116, 116, 116 },
    { 123, 252, 204, 217, 115, 148, 116, 134 },
    { 210, 246, 162, 116, 251, 116, 116, 132 },
  }),
};
#endif

const CtxSet ContextSetCfg::ParFlagL[2] = {
  ContextSetCfg::addCtxSet({
// ctx 374 to 394
 { 33, 0, 17, 18, 0, 0, 25, 33, 26, 34, 19, 25, 34, 42, 42, CNU, 26, 27, CNU, 42, CNU, },
 { 33, 0, 0, 2, 0, 0, 25, 33, 26, 34, 19, 25, 34, 42, 42, CNU, 19, CNU, 42, CNU, CNU, },
 { 25, 0, 1, 9, 3, 9, 25, 18, 11, 34, 27, 33, 19, 19, 27, CNU, 34, 42, 20, 43, 20, },
 { 10, 13, 13, 0, 0, 0, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13, 12, 13, 13, 12, 13, },
 { 13, 0, DWS, 0, 0, 0, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13, 12, 13, 13, 13, 13, },
 { 13, 13, 3, 0, 0, 0, 10, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, },
 { 25, 25, 11, 25, 4, 4, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, DWE, DWE, 4, 4, 4, },
 { 25, 25, 25, 25, 4, 4, 4, 4, 11, 11, 4, 11, 4, 4, 4, 4, 11, 11, 11, 4, 4, },
 { 25, 25, DWE, 25, 25, 4, 4, 4, 11, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 11, 4, },
 { 66, 64, 195, 116, 116, 116, 66, 84, 75, 104, 87, 106, 86, 117, 122, 105, 122, 234, 100, 90, 122, },
 { 181, 183, 252, 116, 116, 116, 167, 235, 235, 170, 234, 227, 234, 136, 151, 234, 101, 106, 218, 235, 122, },
 { 66, 64, 195, 116, 116, 116, 66, 84, 75, 104, 87, 106, 86, 117, 122, 105, 122, 234, 100, 90, 122, },
 { 181, 183, 252, 116, 116, 116, 167, 235, 235, 170, 234, 227, 234, 136, 151, 234, 101, 106, 218, 235, 122, },
 { 66, 64, 195, 116, 116, 116, 66, 84, 75, 104, 87, 106, 86, 117, 122, 105, 122, 234, 100, 90, 122, },
 { 181, 183, 252, 116, 116, 116, 167, 235, 235, 170, 234, 227, 234, 136, 151, 234, 101, 106, 218, 235, 122, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 395 to 405
 { 33, 25, 26, 19, 42, 27, 33, 50, CNU, CNU, CNU, },
 { 25, 25, 18, 4, 42, 19, 33, 42, CNU, 12, 43, },
 { 33, 25, 26, 34, 19, 27, 26, 50, CNU, CNU, CNU, },
 { 13, 13, 12, 12, 13, 12, 12, 12, 10, 13, 10, },
 { 10, 12, 12, 7, 12, 13, 13, 12, 13, 13, 13, },
 { 13, 13, 12, 13, 13, 13, 12, 12, 13, 13, 13, },
 { 25, 4, DWE, 25, DWE, 25, 4, 11, 11, 25, 11, },
 { 25, DWE, 25, 25, 4, 4, DWE, 25, DWE, 25, DWE, },
 { 25, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, },
 { 64, 122, 85, 99, 99, 117, 219, 106, 243, 180, 249, },
 { 234, 117, 234, 187, 203, 123, 86, 104, 168, 165, 88, },
 { 64, 122, 85, 99, 99, 117, 219, 106, 243, 180, 249, },
 { 234, 117, 234, 187, 203, 123, 86, 104, 168, 165, 88, },
 { 64, 122, 85, 99, 99, 117, 219, 106, 243, 180, 249, },
 { 234, 117, 234, 187, 203, 123, 86, 104, 168, 165, 88, },
  }),
};

#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const CtxSet ContextSetCfg::GtxFlagL[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 406 to 434
 { 25, 0, 59, 0, CNU, CNU, CNU, CNU, 2, 10, 26, 34, 4, 12, 28, 25, 34, 27, 20, 28, 36, 29, 19, 20, 21, 29, 22, 45, 30, },
 { 17, 0, CNU, CNU, CNU, CNU, CNU, CNU, 17, 2, 18, 26, 27, 27, 20, 25, 26, 34, CNU, 28, 28, 29, 26, 27, 28, 36, 37, 22, 30, },
 { 17, 0, 0, 0, 0, 0, 39, CNU, 2, 10, 18, 26, 27, 27, 12, 18, 26, 27, CNU, 13, 28, 29, 27, 28, 36, 29, 22, 22, 30, },
 { 13, 0, 0, 0, DWS, DWS, DWS, DWS, 9, 13, 12, 13, 10, 7, 10, 12, 13, DWS, 9, DWS, 9, 9, 9, DWS, 9, 10, 9, 12, 10, },
 { 2, 0, DWS, DWS, DWS, DWS, DWS, DWS, 12, 12, 12, 12, 7, 7, 7, 12, 12, DWS, 12, DWS, DWS, DWS, DWS, DWS, DWS, DWS, 9, 12, 12, },
 { 2, 0, 0, 0, 0, 0, 0, DWS, 9, 13, 13, 12, 9, 13, 9, 9, 12, 9, 9, 10, 9, 12, 6, DWS, 9, 12, 9, 10, 9, },
 { 4, 4, 4, 4, DWE, DWE, DWE, DWE, DWE, 25, 25, DWE, 4, 4, 4, 25, 25, 11, 11, 4, 4, 11, 25, 25, 25, 25, DWE, DWE, DWE, },
 { 11, 4, DWE, DWE, DWE, DWE, DWE, DWE, 25, 25, 25, 25, 4, 4, DWE, 25, 25, 25, 25, 4, 4, 11, 25, 25, 25, 25, 25, 25, 25, },
 { 4, 4, 4, 4, 4, 4, 4, DWE, DWE, DWE, DWE, 11, 4, DWE, 11, 4, 11, 4, 4, 4, 4, 4, 4, 4, 11, DWE, 4, 4, 11, },
 { 234, 116, 116, 116, 116, 116, 116, DWO, 146, 234, 235, 218, 217, 217, 217, 233, 234, 252, 235, 252, 183, 249, 251, 252, 184, 234, 118, 105, 250, },
 { 215, 116, 116, 116, 116, 116, 116, DWO, 121, 101, 102, 99, 100, 121, 84, 99, 89, 101, 101, 86, 99, 84, 99, 92, 98, 84, 107, 102, 82, },
 { 86, 116, 116, 116, 116, 116, 116, DWO, 99, 99, 118, 235, 244, 185, 154, 115, 235, 251, 134, 120, 121, 136, 116, 246, 244, 252, 132, 116, 181, },
 { 136, 116, 116, 116, 116, 116, 116, DWO, 148, 124, 226, 99, 107, 193, 91, 107, 86, 102, 107, 115, 115, 99, 117, 101, 100, 85, 140, 245, 87, },
 { 252, 116, 116, 116, 116, 116, 116, DWO, 82, 234, 218, 235, 121, 234, 251, 162, 219, 203, 137, 115, 251, 229, 117, 166, 251, 227, 148, 117, 187, },
 { 165, 116, 116, 116, 116, 116, 116, DWO, 165, 130, 178, 84, 107, 193, 85, 117, 85, 103, 106, 121, 85, 83, 122, 101, 85, 100, 118, 115, 84, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 435 to 449
 { 33, 2, 3, 11, 4, 36, 5, 6, 2, 34, 27, 44, 37, 14, 38, },
 { 40, 9, 18, 11, 27, 59, 36, 37, 2, 18, 12, 29, 53, 45, 38, },
 { 25, 2, 26, 19, CNU, 20, 20, 21, 27, 28, 29, 29, 22, 22, 38, },
 { 10, 3, DWS, 1, 9, 4, 7, 4, 10, 9, 12, 5, 10, 3, DWS, },
 { 13, 10, 5, 9, 13, DWS, 7, DWS, 10, 13, DWS, 3, 10, 13, 7, },
 { 4, 6, DWS, 9, 10, 12, DWS, 13, 7, 10, 10, 10, DWS, 10, 9, },
 { 25, 4, DWE, 4, 25, 4, 25, 25, 4, 11, 4, 11, DWE, 25, 4, },
 { 25, 4, 11, 25, DWE, 4, 4, 25, 4, 25, DWE, 25, 4, 4, 11, },
 { 4, 11, 4, 11, DWE, 11, 4, DWE, 4, DWE, DWE, DWE, 4, 11, 4, },
 { 180, 131, 100, 136, 113, 102, 209, 181, 120, 103, 185, 178, 186, 251, 219, },
 { DWO, 132, 117, 117, 117, 123, 231, 116, 146, 100, 99, 124, 99, 250, 162, },
 { 99, 107, 91, 104, 129, 181, 133, 149, 99, 101, 103, 100, 132, 195, 120, },
 { DWO, 118, 123, 104, 216, 87, 116, 90, 121, 106, 180, 203, 140, 250, 83, },
 { 99, 99, 244, 147, 102, 163, 118, 186, 102, 114, 129, 115, 132, 132, 134, },
 { 122, 133, 133, 102, 178, 131, 87, 84, 117, 106, 249, 100, 220, 115, 83, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 450 to 478
 { 9, 1, 25, 33, 2, 0, CNU, CNU, 25, 34, 27, 28, 13, 29, 29, 26, 20, 36, 29, 37, 22, 30, 28, 29, 37, 45, 38, 38, 38, },
 { 1, 1, 25, 9, 58, 0, CNU, CNU, 25, 34, CNU, 28, 21, 29, 29, 26, 28, 36, 29, 37, 37, 30, 20, 21, 37, 37, 38, 38, 38, },
 { 25, 0, 17, 33, 48, 1, 4, 39, 25, 11, 27, 20, 13, 21, 21, 26, 20, 21, 21, 37, 22, 22, 28, 29, 45, 30, 38, 38, 23, },
 { 9, 5, 4, 12, 0, 0, DWS, DWS, 13, 13, 13, 13, 13, 9, 13, 13, 13, 13, 10, 13, 13, 13, 7, 9, DWS, 12, 13, 13, 9, },
 { 10, 4, 3, 13, 0, 0, DWS, DWS, 13, 13, 13, 12, 10, 10, 13, 13, 13, 13, 10, 13, 13, 10, 10, DWS, DWS, DWS, 13, 13, 9, },
 { 9, 13, 13, 13, 12, 0, 0, 0, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 10, 13, 12, 13, 9, 10, 10, 10, 12, 13, 13, },
 { 4, 4, DWE, 4, 4, 4, DWE, DWE, 4, 4, 4, 4, 11, 4, 4, DWE, 11, 4, 4, 4, 4, 4, DWE, 11, 4, 11, 11, 4, 4, },
 { 4, 4, 25, 25, 4, 25, DWE, DWE, 4, 4, 11, 11, DWE, DWE, 4, DWE, 11, 4, 4, 4, 4, 4, 25, DWE, 4, 4, 11, 4, 4, },
 { DWE, 4, 4, 11, 4, 4, 4, 4, 11, 11, 4, 4, 11, 11, 11, DWE, 11, 4, 4, 4, 4, 4, 11, 11, 11, 4, 11, 4, 4, },
 { 82, 98, 99, DWO, 116, 116, 116, 116, 106, 103, 200, 88, 100, 91, 202, 234, 234, 234, 152, 136, 131, 134, 152, 164, 108, 116, 100, 102, 155, },
 { 165, 233, 135, 171, 116, 116, 116, 116, 234, 234, 183, 234, 218, 251, 101, 234, 234, 234, 250, 234, 234, 104, 115, 100, 102, 101, 218, 218, 84, },
 { 82, 64, 67, 169, 133, 116, 116, 116, 90, 106, 133, 227, 107, 100, 235, 234, 198, 196, 138, 196, 132, 136, 147, 118, 108, 108, 117, 133, 165, },
 { 244, 137, 135, 104, 106, 116, 116, 116, 234, 234, 151, 235, 197, 248, 86, 234, 234, 235, 153, 218, 235, 85, 117, 131, 212, 244, 181, 101, 90, },
 { 81, 64, 67, 80, 65, 116, 116, 116, 90, 117, 199, 105, 100, 99, 234, 234, 233, 133, 169, 102, 101, 200, 131, 116, 100, 102, 227, 100, DWO, },
 { 155, 234, 234, 117, 169, 116, 116, 116, 234, 234, 136, 218, 234, 234, 83, 234, 234, 235, 234, 234, 235, 84, 117, 122, 250, 250, 235, 234, 104, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 479 to 493
 { 9, 25, 27, 20, 13, 37, 29, 30, 28, 45, 38, 46, 46, 54, 54, },
 { 9, 25, 27, 20, 13, 14, 6, 53, 20, 37, 15, 30, 38, 23, 31, },
 { 40, 33, 27, 20, 13, 21, 29, 37, 36, 37, 45, 30, 46, 46, 46, },
 { 9, 13, 12, DWS, DWS, DWS, DWS, 13, DWS, DWS, 4, DWS, 9, 5, 10, },
 { 9, 13, 12, DWS, 5, 4, 10, 10, 5, DWS, DWS, DWS, 13, DWS, 12, },
 { 13, 13, 12, 12, 9, 12, 13, 9, DWS, 12, DWS, 9, DWS, 9, 13, },
 { 4, DWE, 25, 25, 25, 25, 25, 25, DWE, DWE, 4, 25, 25, 11, 25, },
 { 4, DWE, 25, 25, 25, 4, 4, 4, 11, 25, 25, 25, 25, 4, DWE, },
 { DWE, 11, 11, DWE, 11, 11, 11, 4, 4, 25, 11, 11, 4, 4, 25, },
 { 86, 234, 210, 179, 242, 178, 188, 130, 164, 146, 164, 194, 195, 100, 250, },
 { 164, 234, 218, 132, 246, 220, 242, 84, 101, 250, 137, 215, 151, 165, 80, },
 { 64, 179, 116, 131, 115, 108, DWO, 199, 132, 99, 132, 131, 132, 100, 132, },
 { 164, 234, 235, 121, 252, 205, 108, 82, 108, 234, 108, 215, 204, 234, 73, },
 { 64, 168, 116, 179, 131, 212, 147, 235, 116, 114, 132, 115, 116, 115, 230, },
 { 184, 234, 235, 232, 251, 215, 121, 82, 103, 235, 123, 251, 252, 251, 66, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 494 to 522
 { 25, 48, 9, 0, 0, CNU, CNU, CNU, 25, 18, 19, 27, 20, 28, 36, 33, 27, 28, 36, 29, 29, 37, CNU, 36, 37, 45, 38, 38, 38, },
 { 25, 0, 0, 0, 0, CNU, CNU, CNU, 25, 26, 34, 12, 20, 28, 36, 26, 27, 28, 36, 29, 37, 22, CNU, 29, 37, 37, 38, 38, 38, },
 { 25, 1, 17, 0, 56, 2, 3, 0, 25, 3, 11, 12, 20, 5, 13, 26, 27, 28, 21, 29, 29, 22, 20, 29, 37, 22, 15, 38, 23, },
 { 6, 4, 0, 0, 0, DWS, DWS, DWS, 13, 13, 13, 13, 13, 13, 12, 13, 10, 13, 13, 13, 13, 12, 7, DWS, 9, 10, 12, 13, 9, },
 { 7, 0, 0, 0, 0, DWS, DWS, DWS, 13, 13, 13, 12, 13, 13, 12, 10, 10, 13, 10, 13, 12, 13, 13, DWS, 13, DWS, 12, 13, 9, },
 { 9, 12, 0, 0, 0, 0, 0, 0, 13, 12, 13, 13, 13, 12, 13, 9, 10, 10, 10, 13, 13, 13, 9, 10, 9, 13, 12, 12, 12, },
 { 4, 11, 4, 4, 4, DWE, DWE, DWE, 4, 4, 11, 4, 11, 4, 4, 4, 4, 11, 4, 11, 4, 4, 4, 4, 11, 11, 4, 11, 4, },
 { DWE, 4, 25, 25, 4, DWE, DWE, DWE, 4, 11, 4, DWE, 25, 11, 4, 4, 4, DWE, 4, 11, 4, 4, 25, 11, DWE, 4, 4, 4, 4, },
 { 11, 11, 25, 25, 4, 4, 4, 4, 11, 11, 11, DWE, DWE, 11, 4, 4, 4, 4, 4, 11, 4, 4, 4, DWE, 4, DWE, 11, 11, 4, },
 { 113, 99, 116, 116, 116, 116, 116, 116, 69, 102, 99, 90, 162, 88, 136, 202, DWO, 100, 103, 102, 105, 137, 132, 245, 104, 115, 85, 194, 139, },
 { 155, 219, 116, 116, 116, 116, 116, 116, 198, 234, 218, 184, 234, 185, 86, 163, 138, 234, 216, 234, 218, 85, 115, 179, 251, 148, 219, 233, 91, },
 { 82, 83, 102, 64, 116, 116, 116, 116, 69, 116, 198, 90, 100, 103, DWO, 100, 106, 102, 102, 118, 132, 118, 115, 120, 107, 107, 180, 117, 149, },
 { 137, 234, 168, 170, 116, 116, 116, 116, 136, 166, 132, 235, 235, 219, 101, 136, 136, 250, 250, 168, 138, 116, 120, 211, 235, 147, 90, 229, 107, },
 { 82, 72, 116, 116, 116, 116, 116, 116, 67, 84, 117, 84, 131, 86, 120, 91, 101, 99, 100, 131, 165, 105, 115, 115, 132, 100, 116, 89, 105, },
 { 152, 235, 238, 116, 116, 116, 116, 116, 153, 235, 231, 234, 197, 235, 101, 152, 170, 250, 250, 234, 117, 138, 123, 250, 147, 151, 235, 235, 235, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 523 to 537
 { 17, 33, 11, 12, 51, 13, 37, 45, 26, 36, 37, 45, 46, 38, 31, },
 { 25, 18, 19, 12, 13, 5, 59, 6, 26, 21, 14, 22, 38, 38, 31, },
 { 40, 33, 19, 12, 28, 13, 21, 37, 20, 29, 37, 30, 30, 38, 38, },
 { DWS, DWS, 12, DWS, 10, 6, 4, 10, 9, 10, 1, 12, 10, 6, 13, },
 { 10, 5, 5, 4, 12, 3, 0, 3, 5, 10, 1, 3, 13, 5, 10, },
 { 12, 12, 12, 9, 12, DWS, 13, 10, 9, 12, 4, 10, 12, 9, 13, },
 { 11, 11, 25, DWE, 25, 25, 11, 4, 11, 25, 4, 25, DWE, 4, 25, },
 { 4, 4, 25, 25, 25, 4, 4, DWE, 4, 25, 4, 4, DWE, 4, 11, },
 { DWE, 11, 25, 4, 11, 4, 25, 11, 11, 25, 4, DWE, 11, 4, 25, },
 { 252, 84, 226, 178, 210, 243, 100, 131, 85, 240, 149, 148, 195, 115, 150, },
 { 132, 117, 235, 136, 132, 165, 123, 89, 234, 249, 173, 131, 234, 249, 67, },
 { 250, 68, 84, 226, 210, 117, 116, 120, 86, 99, 133, 103, 163, 115, 233, },
 { 117, 104, 235, 218, 121, 186, 252, 89, 172, 218, 106, 218, 235, 250, 73, },
 { 218, 67, 83, 114, 162, 104, 116, 212, 84, 86, 133, 129, 90, 115, 233, },
 { 101, 120, 235, 251, 235, 252, 234, 83, 235, 235, DWO, 250, 146, 235, 65, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 538 to 566
 { 42, CNU, 0, CNU, CNU, CNU, CNU, CNU, 18, 26, 11, 19, 27, 12, 20, 26, 19, CNU, CNU, 13, 21, 29, 28, 36, 29, 14, 30, 30, 30, },
 { 2, CNU, CNU, CNU, CNU, CNU, CNU, CNU, 26, 34, 34, 42, 27, CNU, 20, 18, 19, 27, 28, 21, 21, 29, 27, 36, 36, 37, 30, 30, 38, },
 { 42, CNU, CNU, CNU, CNU, CNU, 0, CNU, 41, 26, 26, 34, 4, 4, 20, 11, 19, 12, 20, 28, 13, 29, CNU, 36, 29, 29, 22, 22, 30, },
 { 5, DWS, 0, DWS, DWS, DWS, DWS, DWS, 12, 13, 13, 13, 13, 13, 13, 12, 12, 13, 12, 13, 13, 13, 2, 9, 10, 12, 13, 13, 12, },
 { 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, 13, 12, 13, 13, 13, 13, 13, 12, DWS, DWS, 13, 13, 13, 13, 1, DWS, DWS, 12, 13, 13, 13, },
 { 10, DWS, DWS, DWS, DWS, DWS, 0, DWS, 13, 12, 12, 12, 13, 13, 13, 10, 13, 13, 13, 12, 13, 13, 9, 9, 10, 9, 12, 13, 12, },
 { 11, DWE, 4, DWE, DWE, DWE, DWE, DWE, DWE, 11, 11, 4, 4, 4, 4, 11, 11, 11, 4, 11, 4, 11, 4, 11, DWE, DWE, 11, 4, 4, },
 { 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, 4, 4, 4, 4, 4, 4, 11, 25, 4, 4, 11, 4, 4, 11, 4, 11, 25, DWE, 11, 11, 11, },
 { DWE, DWE, DWE, DWE, DWE, DWE, 4, DWE, 25, 11, 11, DWE, 4, 11, 4, 11, 11, 11, 4, 4, 11, 4, 4, 4, 4, 4, 4, DWE, 4, },
 { 146, DWO, 116, DWO, DWO, DWO, 116, DWO, 146, 234, 104, 151, 217, 90, 217, 235, 235, 234, 235, 234, 117, 234, 124, 250, 250, 235, 101, 163, 235, },
 { 252, DWO, 116, DWO, DWO, DWO, 116, DWO, 186, 234, 218, 217, 234, 234, 165, 163, 101, 137, 167, 234, 234, 101, 134, 131, 118, DWO, 234, 234, 84, },
 { 130, DWO, DWO, DWO, DWO, DWO, 116, DWO, 115, 153, 218, 235, 185, 147, 234, 87, 204, 203, 186, 234, 148, 234, 107, 170, 196, 251, 105, 211, 235, },
 { 183, DWO, DWO, DWO, DWO, DWO, 116, DWO, 166, 233, 229, 132, 234, 234, 153, 179, 243, 196, 234, 218, 234, 103, 133, 108, DWO, 106, 234, 234, 84, },
 { 231, DWO, DWO, DWO, DWO, DWO, 116, DWO, 169, 217, 233, 235, 163, 233, 234, 83, 234, 199, 234, 235, 234, 234, DWO, 155, 138, 251, 200, 217, 235, },
 { 134, DWO, DWO, DWO, DWO, DWO, 116, DWO, 195, 203, 102, 132, 234, 234, 234, 233, 116, 185, 233, 184, 153, 116, 131, 105, 138, 107, 132, 99, 84, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 567 to 581
 { 56, 10, 26, 19, 4, 19, 43, 36, 18, 34, 27, 20, 13, 21, 22, },
 { 57, 2, 11, 4, CNU, 27, 12, 28, 41, 57, 43, CNU, 13, 21, 14, },
 { 4, 26, 19, 19, CNU, 5, 5, 29, 20, 13, 29, 29, 22, 22, 30, },
 { 0, 7, 12, 13, 3, 10, 10, 5, 1, 13, 13, 13, 13, 13, 13, },
 { 1, 3, 13, 13, 7, 13, 13, 4, 3, 13, 13, 13, 13, 12, 13, },
 { 12, 10, 13, 10, 13, 9, 12, 13, 10, 13, 13, 13, 12, 12, 10, },
 { 25, 11, 25, 4, 11, 25, 25, 11, 4, 11, 4, 4, 4, 4, 4, },
 { 4, 11, 4, 4, 11, 4, 4, 4, 4, DWE, 4, 4, 4, 4, 25, },
 { 25, 4, 25, 4, 11, 4, 25, 4, 11, 25, DWE, DWE, 4, 11, 11, },
 { 250, 148, 152, 234, 247, 234, 248, 252, 227, 147, 170, 215, 217, 88, 234, },
 { 116, 134, 103, 185, 231, 231, 231, 103, 155, 183, 218, 234, 234, 217, 186, },
 { 115, 69, 144, 250, 151, 146, 130, 205, 177, 114, 147, 218, 195, 149, 217, },
 { 102, 163, 234, 103, 231, 234, 250, 101, 168, 250, 233, 99, 234, 187, 85, },
 { 130, 86, 83, 170, 217, 130, 162, 234, 161, 132, 117, 218, 88, 104, 250, },
 { 163, 195, 234, 102, 121, 251, 234, 202, 250, 151, 232, 146, 219, 235, 83, },
  }),
};
#else
const CtxSet ContextSetCfg::GtxFlagL[4] = {
  ContextSetCfg::addCtxSet({
    { 17, 0, 0, 0, 0, 0, 0, 17, 25, 25, 26, 9, 25, 18, 26, 12, 18, 26, 12, 20, 37 },
    { 1, 0, 0, 0, 0, 35, 0, 9, 25, 25, 18, 9, 25, 33, 26, 20, 18, 26, 12, 20, 22 },
    { 1, 0, 0, 0, 0, 0, 1, 17, 17, 25, 18, 17, 25, 18, 26, 12, 33, 19, 20, 28, 14 },
    { 4, 0, 0, 0, 0, 0, 12, 9, 13, 13, 6, 12, 10, 9, 9, 6, 9, 8, 5, 8, 9 },
    { 4, 0, 0, 0, 0, 8, 13, 12, 13, 13, 5, 13, 10, 9, 10, 9, 9, 4, 8, 8, 9 },
    { 4, 0, 0, 0, 0, 0, 12, 8, 12, 13, 6, 9, 10, 9, 10, 9, 6, 9, 10, 10, 9 },
    { 18, 25, 25, 25, 4, 4, 11, 11, 18, 11, 11, 4, 4, 11, 4, 4, 18, 25, 11, 11, 4 },
    { 4, 25, 25, 25, 4, 18, 18, 4, 11, 25, 4, 4, 4, 4, 11, 4, 18, 4, 25, 18, 4 },
    { 4, 25, 25, 25, 4, 4, 25, 4, 4, 18, 4, 4, 4, 4, 4, 4, 4, 18, 18, 18, 4 },
    { 99, 116, 116, 116, 116, 116, 66, 81, 65, 147, 100, 87, 121, 148, 132, 122, 118, 252, 155, 220, 118 },
    { 149, 116, 116, 116, 116, 116, 181, 215, 184, 103, 133, 132, 105, 118, 103, 104, 104, 100, 101, 86, 122 },
    { 99, 116, 116, 116, 116, 119, 66, 68, 66, 83, 100, 103, 138, 235, 242, 100, 235, 253, 252, 252, 123 },
    { 168, 116, 116, 116, 116, 119, 135, 151, 234, 234, 121, 103, 106, 132, 134, 100, 100, 101, 101, 115, 101 },
    { 99, 116, 116, 116, 116, 116, 65, 83, 99, 89, 123, 82, 90, 102, 194, 147, 107, 117, 103, 104, 118 },
    { 171, 116, 116, 116, 116, 116, 154, 156, 229, 117, 116, 149, 122, 136, 133, 102, 119, 116, 122, 121, 117 },
  }),
  ContextSetCfg::addCtxSet({
    { 17, 9, 25, 25, 41, 4, 17, 33, 34, 36, 45 },
    { 1, 1, 17, 10, 18, 4, 17, 33, 19, 43, 14 },
    { 0, 1, 25, 33, 18, 19, 18, 19, 20, 28, 14 },
    { 4, 9, 12, 8, 9, 4, 5, 5, 10, 6, 12 },
    { 5, 9, 12, 8, 12, 5, 5, 10, 5, 3, 7 },
    { 13, 12, 12, 8, 10, 5, 5, 5, 9, 8, 8 },
    { 25, 18, 25, 18, 25, 11, 4, 4, 18, 11, 25 },
    { 4, 11, 25, 25, 25, 4, 4, 18, 11, 4, 4 },
    { 4, 25, 25, 18, 18, 4, 4, 4, 25, 18, 4 },
    { 99, 82, 65, 150, 226, 116, 98, 99, 210, 100, 115 },
    { 122, 148, 232, 107, 155, 103, 134, 124, 250, 203, 234 },
    { 98, 83, 81, 82, 225, 132, 98, 85, 100, 116, 119 },
    { 252, 120, 120, 234, 164, 101, 185, 137, 203, 138, 129 },
    { 183, 66, 65, 102, 98, 122, 99, 101, 114, 101, 116 },
    { 122, 148, 154, 120, 150, 103, 119, 119, 155, 120, 200 },
  }),
  ContextSetCfg::addCtxSet({
    { 9, 17, 2, 10, 19, 0, 25, 34, 27, 28, 29, 26, 20, 36, 29, 22, 35, 29, 37, 45, 38 },
    { 1, 1, 17, 0, 20, 0, 25, 34, 27, 28, 29, 19, 28, 36, 29, 22, 20, 36, 30, 45, 38 },
    { 25, 0, 25, 33, 2, 2, 25, 11, 27, 20, 21, 26, 20, 36, 21, 22, 28, 29, 45, 30, 23 },
    { 9, 4, 7, 2, 0, 0, 13, 13, 13, 13, 10, 13, 10, 13, 10, 10, 6, 9, 9, 12, 13 },
    { 9, 4, 7, 13, 0, 0, 13, 13, 13, 12, 13, 13, 10, 13, 10, 10, 9, 8, 9, 8, 13 },
    { 9, 12, 10, 13, 13, 0, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 9, 10, 10, 10, 13 },
    { 4, 11, 25, 25, 25, 4, 4, 11, 11, 11, 4, 18, 4, 11, 4, 4, 18, 11, 11, 11, 11 },
    { 4, 11, 25, 25, 4, 4, 4, 4, 11, 18, 18, 18, 4, 4, 4, 4, 25, 18, 4, 4, 11 },
    { 18, 4, 25, 11, 4, 25, 11, 11, 4, 4, 4, 18, 11, 11, 4, 4, 11, 11, 11, 4, 4 },
    { 84, 99, 97, 114, 116, 116, 105, 106, 152, 90, 99, 234, 250, 197, 163, 133, 155, 245, 154, 166, 169 },
    { 148, 253, 208, 220, 116, 116, 200, 234, 197, 217, 234, 201, 149, 234, 250, 103, 107, 100, 85, 115, 70 },
    { 87, 99, 103, 64, 116, 116, 105, 102, 131, 90, 87, 234, 250, 182, 136, 122, 250, 235, 214, 202, 153 },
    { 180, 136, 216, 185, 116, 116, 234, 234, 234, 235, 234, 234, 234, 216, 186, 106, 99, 89, 115, 87, 72 },
    { 82, 97, 80, 215, 122, 116, 90, 117, 148, 227, 131, 234, 213, 227, 196, 106, 131, 117, 115, 116, 117 },
    { 137, 198, 217, 201, 168, 116, 234, 233, 197, 202, 234, 234, 234, 234, 202, 154, 117, 120, 234, 234, 185 },
  }),
  ContextSetCfg::addCtxSet({
    { 9, 18, 27, 36, 13, 37, 43, 45, 53, 46, 46 },
    { 9, 25, 27, 20, 13, 14, 20, 22, 30, 30, 23 },
    { 40, 33, 27, 20, 13, 29, 36, 37, 45, 30, 46 },
    { 9, 12, 12, 8, 12, 8, 9, 8, 5, 10, 9 },
    { 12, 13, 12, 8, 5, 13, 5, 9, 4, 0, 13 },
    { 13, 13, 12, 12, 9, 10, 8, 12, 8, 12, 13 },
    { 4, 18, 25, 18, 25, 25, 25, 11, 11, 18, 18 },
    { 4, 18, 25, 25, 25, 18, 11, 25, 18, 4, 18 },
    { 25, 11, 11, 25, 11, 11, 4, 25, 4, 25, 18 },
    { 103, 83, 131, 147, 210, 211, 117, 243, 119, 178, 212 },
    { 132, 134, 148, 252, 186, 108, 115, 252, 115, 250, 81 },
    { 87, 228, 225, 114, 146, 101, 135, 225, 148, 136, 231 },
    { 148, 151, 235, 251, 220, 150, 116, 181, 107, 132, 210 },
    { 64, 211, 179, 131, 100, 137, 102, 130, 148, 162, 120 },
    { 134, 234, 235, 138, 251, 100, 108, 235, 103, 182, 104 },
  }),
};
#endif

#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const CtxSet ContextSetCfg::SigFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 582 to 599
 { 17, 41, 42, 44, 37, 30, 9, 42, 43, 37, 45, 30, 33, 59, 51, 45, 30, 38, },
 { 17, 41, 42, 29, 37, 30, 25, 42, 43, 37, 45, 30, 33, 59, 51, 30, 30, 38, },
 { 25, 19, 20, 14, 22, 30, 18, 28, 29, 30, 30, 38, 26, 45, 30, 38, 38, 38, },
 { 13, 9, 9, 10, 10, 13, 9, 10, 10, 10, 10, 13, DWS, 9, 9, 9, 10, 9, },
 { 13, 9, 9, 10, 10, 13, 9, 10, 10, 10, 10, 13, DWS, 9, 9, 9, 10, 10, },
 { 13, 9, 9, 10, 13, 10, 9, 9, 9, 13, 13, 13, DWS, 9, 10, 10, 10, 9, },
 { 11, 4, DWE, DWE, 4, DWE, 11, DWE, DWE, 4, 4, DWE, 11, DWE, DWE, 4, 11, 4, },
 { 11, 11, DWE, DWE, 11, 25, DWE, DWE, DWE, 4, 4, DWE, DWE, DWE, DWE, 4, 11, 11, },
 { 11, 11, DWE, 4, 4, 4, DWE, 11, 11, 11, 4, 4, 11, DWE, DWE, 4, 4, 4, },
 { 74, 117, 116, 115, 136, 181, 103, 106, 115, 131, 137, 169, 117, 131, 117, 106, 135, 215, },
 { 149, 228, 117, 138, 104, 86, 117, 122, 117, 234, 106, 83, 117, 123, 118, 138, 90, 82, },
 { 87, 117, 117, 116, 122, 133, 102, 116, 116, 147, 181, 228, 118, 147, 117, 132, 132, 153, },
 { 122, 228, 117, 138, 122, 101, 118, 123, DWO, 250, 102, 90, 117, 123, 132, 133, 100, 82, },
 { 193, 117, 117, 242, 117, 138, 101, 147, 163, 131, 151, 151, 132, 131, 146, 163, 133, 251, },
 { 148, 212, 117, 138, 102, 90, 120, DWO, 118, 234, 89, 73, 196, DWO, 122, 133, 87, 82, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 600 to 611
 { 25, 27, 43, 44, 37, 38, 41, 45, 45, 30, 38, 23, },
 { 18, 27, 28, 21, 14, 14, 34, 45, 30, 30, 38, 23, },
 { 25, CNU, 28, 29, 37, 30, 26, 53, 53, 46, 46, 46, },
 { 12, 13, 9, 13, 12, 13, 4, 1, DWS, 5, 9, 13, },
 { 13, 13, 13, 13, 13, DWS, 5, 5, DWS, DWS, DWS, 12, },
 { 12, 12, 9, 9, 12, 9, 5, 5, DWS, DWS, DWS, DWS, },
 { DWE, DWE, 4, DWE, 25, 25, 11, 4, DWE, 4, 25, 11, },
 { 11, 11, 11, 4, 11, 25, 4, 4, 11, 11, 25, DWE, },
 { DWE, DWE, 11, 4, 11, 4, 4, 4, 11, 11, 4, 4, },
 { 69, 89, 243, 227, 234, 234, 116, 148, 101, 132, 116, 212, },
 { 118, DWO, 123, 104, 83, 81, DWO, 124, 135, 203, 218, 179, },
 { 90, 104, 116, 104, 105, 116, 117, 133, 132, 132, 118, 199, },
 { 108, 132, 123, 139, 116, 136, 118, 108, 135, 252, 234, 82, },
 { 114, 100, 116, 102, 120, 105, 100, 148, 116, 132, 118, 203, },
 { 107, 164, 121, 153, 116, 235, 124, 108, 235, 236, 204, 82, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 612 to 629
 { 26, 45, 53, 46, 46, 46, 26, 54, 61, 39, 39, 39, 34, 39, 39, 39, 39, 39, },
 { 26, 38, 38, 46, 46, 46, 41, 54, 54, 39, 39, 39, 5, 39, 39, 39, 39, 39, },
 { 26, 38, 46, 54, 54, 54, 11, 39, 39, 39, 39, 39, 28, 39, 39, 39, 39, 39, },
 { 9, 13, 12, DWS, DWS, DWS, DWS, DWS, DWS, 4, 4, 4, 0, 0, 0, 0, 0, 0, },
 { 9, 13, 12, DWS, DWS, DWS, DWS, DWS, DWS, 4, 4, 4, 0, 0, 0, 0, 0, 0, },
 { 9, 13, 12, DWS, DWS, DWS, DWS, DWS, 4, 0, 0, 4, 4, 0, 0, 0, 0, 0, },
 { 4, 4, DWE, DWE, DWE, DWE, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 4, 11, 25, 11, 11, DWE, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 4, 4, 25, 4, 4, 4, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 132, 234, 235, 252, 251, 251, 116, 252, 251, 201, 153, 184, 116, 237, 253, 252, 164, 254, },
 { 123, 234, 151, 92, 83, 82, 108, 243, 130, 99, 99, 99, 148, 116, 116, 116, 116, 116, },
 { 148, 234, 235, 252, 236, 236, 116, 253, 248, 168, 156, 215, 115, 252, 251, 252, 203, 253, },
 { 120, 234, 123, 87, 84, 83, 117, 243, 162, 99, 99, 99, 148, 116, 116, 116, 132, 116, },
 { 196, 234, 235, 252, 252, 236, 101, 250, 236, 190, 170, 252, 115, 252, 253, 253, 254, 254, },
 { 120, 234, 123, 85, 84, 82, 117, 243, 163, 116, 116, 99, 243, 132, 116, 132, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 630 to 641
 { 33, 30, 38, 31, 31, 31, 4, 39, 39, 39, 39, 39, },
 { 34, 38, 53, 54, 54, 62, 29, 39, 39, 39, 39, 39, },
 { 18, 46, 38, 39, 39, 39, 44, 39, 39, 39, 39, 39, },
 { DWS, 12, 13, DWS, 9, 9, 0, 0, 0, 0, 0, 0, },
 { DWS, 12, DWS, 4, DWS, 7, 4, 0, 0, 0, 0, 4, },
 { DWS, 12, 12, DWS, 4, 4, 0, 0, 0, 0, 0, 0, },
 { DWE, 11, 25, DWE, DWE, 11, 25, 25, 25, 25, 25, 25, },
 { 4, 4, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, 11, DWE, 25, DWE, 25, 25, 25, 25, 25, 25, 25, },
 { 100, 235, 218, 155, 203, 235, 116, 253, 253, 250, 132, 235, },
 { 118, 234, 225, 87, 83, 82, 134, 116, 132, 116, 116, 116, },
 { 148, 235, 235, 232, 246, 229, 116, 254, 252, 238, 164, 116, },
 { 117, 185, 129, 99, 83, 82, 227, 116, 116, 116, 116, 99, },
 { 179, 235, 235, 117, 253, 153, 116, 253, 254, 253, 229, 116, },
 { 117, 219, 129, 83, 99, 99, 228, 116, 116, 116, 132, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 642 to 659
 { 26, 62, 39, 39, 39, 39, 34, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { 19, 39, 54, 39, 39, 39, 19, 39, 39, 39, 39, 39, 40, 39, 39, 39, 39, 39, },
 { 18, 39, 39, 39, 39, 39, 11, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { DWS, DWS, DWS, 12, 12, 12, DWS, 4, 4, 4, 0, 1, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, DWS, 12, 12, 12, DWS, 4, 4, 4, 4, 4, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, 12, 12, 12, 12, DWS, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, },
 { DWE, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 118, 252, 233, 235, 235, 235, 116, 253, 253, 249, 238, 253, 116, 237, 238, 235, 235, 252, },
 { DWO, 114, 98, 66, 68, 70, 117, 99, 99, 99, 116, 115, 137, 116, 116, 116, 116, 116, },
 { 121, 252, 252, 235, 218, 235, 132, 203, 215, 168, 167, 169, 116, 237, 238, 212, 253, 236, },
 { 118, 130, 98, 75, 75, 73, 117, 99, 99, 99, 99, 99, 136, 116, 116, 116, 116, 116, },
 { 123, 252, 235, 235, 234, 235, 116, 254, 217, 201, 249, 204, 116, 238, 238, 117, 252, 254, },
 { 117, 130, 82, 75, 69, 72, 117, 116, 116, 116, 116, 99, 152, 116, 116, 116, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 660 to 671
 { 18, 38, 54, 39, 39, 39, 10, 39, 39, 39, 39, 39, },
 { 34, 38, 62, 39, 39, 39, 18, 39, 39, 39, 39, 39, },
 { 26, 39, 39, 39, 39, 39, 10, 39, 39, 39, 39, 39, },
 { DWS, 12, 12, 9, 4, DWS, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, DWS, 4, 4, DWS, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, DWS, 4, 0, 4, 0, 0, 0, 0, 0, 0, },
 { DWE, 25, 25, DWE, 4, 4, 25, 25, 25, 25, 25, 25, },
 { DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 25, 25, 25, 11, 4, 25, 25, 25, 25, 25, 25, 25, },
 { 100, 235, 216, 197, 135, 251, 116, 253, 253, 253, 252, 238, },
 { 118, 235, 120, 178, 103, 107, 133, 116, 116, 116, 116, 116, },
 { 131, 252, 252, 250, 157, 251, 115, 166, 254, 252, 253, 238, },
 { 118, 122, 83, 82, 100, 116, 134, 116, 132, 132, 116, 116, },
 { 115, 234, 251, 253, 254, 253, 116, 253, 252, 254, 254, 238, },
 { 117, 252, 82, 99, 116, 148, 152, 116, 116, 116, 132, 116, },
  }),
};
#else
const CtxSet ContextSetCfg::SigFlag[6] = {
  ContextSetCfg::addCtxSet({
    { 17, 41, 42, 29, 9, 42, 28, 37, 33, 59, 51, 45 },
    { 17, 41, 42, 29, 25, 42, 43, 37, 33, 59, 51, 30 },
    { 25, 19, 20, 14, 33, 28, 29, 30, 26, 45, 30, 38 },
    { 13, 9, 9, 10, 9, 10, 9, 10, 8, 9, 9, 9 },
    { 13, 9, 9, 10, 9, 10, 10, 10, 8, 9, 9, 9 },
    { 13, 9, 9, 10, 10, 9, 9, 13, 8, 9, 10, 10 },
    { 11, 11, 18, 18, 11, 18, 18, 11, 11, 18, 18, 11 },
    { 11, 11, 18, 18, 18, 18, 18, 11, 11, 18, 18, 4 },
    { 18, 11, 18, 11, 18, 11, 11, 11, 11, 18, 18, 11 },
    { 73, 118, 117, 117, 105, 106, 116, 122, 108, 147, 117, 117 },
    { 134, 228, 117, 119, 117, 121, 117, 116, 212, 119, 118, 103 },
    { 73, 107, 131, 116, 107, 115, 115, 120, 148, 147, 117, 132 },
    { 134, 244, 117, 117, 119, 122, 116, 117, 180, 118, 117, 116 },
    { 89, 117, 117, 147, 99, 116, 179, 118, 164, 131, 115, 133 },
    { 195, 133, 117, 131, 119, 134, 122, 122, 196, 139, 250, 99 },
  }),
  ContextSetCfg::addCtxSet({
    { 25, 27, 43, 37, 33, 60, 60, 38 },
    { 18, 27, 28, 22, 34, 45, 45, 38 },
    { 25, 35, 28, 37, 26, 53, 53, 46 },
    { 8, 13, 9, 12, 5, 4, 8, 8 },
    { 12, 13, 13, 12, 6, 5, 8, 8 },
    { 12, 12, 9, 9, 4, 5, 8, 8 },
    { 4, 25, 11, 18, 18, 11, 18, 18 },
    { 11, 18, 18, 4, 11, 11, 18, 18 },
    { 18, 18, 11, 4, 4, 4, 4, 4 },
    { 89, 101, 116, 122, 131, 148, 211, 117 },
    { 149, 115, 118, 100, 118, 117, 119, 137 },
    { 82, 105, 118, 151, 115, 243, 117, 117 },
    { 118, 120, 117, 103, 122, 108, 121, 236 },
    { 130, 100, 227, 179, 109, 164, 116, 133 },
    { 195, 148, 123, 135, 120, 107, 252, 236 },
  }),
  ContextSetCfg::addCtxSet({
    { 26, 45, 53, 46, 26, 54, 61, 39, 34, 39, 39, 39 },
    { 26, 38, 38, 46, 34, 54, 54, 39, 5, 39, 39, 39 },
    { 26, 38, 46, 54, 26, 39, 39, 39, 20, 39, 39, 39 },
    { 9, 13, 12, 8, 8, 8, 8, 4, 0, 0, 0, 0 },
    { 9, 13, 12, 8, 8, 8, 8, 4, 0, 0, 0, 0 },
    { 9, 13, 12, 8, 8, 4, 4, 0, 4, 0, 0, 0 },
    { 11, 4, 18, 18, 18, 25, 25, 18, 18, 25, 25, 25 },
    { 4, 11, 25, 11, 18, 25, 25, 25, 25, 25, 25, 25 },
    { 11, 4, 18, 4, 18, 18, 25, 18, 25, 25, 25, 25 },
    { 117, 234, 201, 218, 116, 247, 184, 170, 116, 238, 238, 253 },
    { 123, 234, 123, 84, 117, 163, 178, 99, 148, 116, 116, 116 },
    { 132, 234, 234, 236, 116, 246, 170, 198, 116, 237, 237, 237 },
    { 133, 234, 137, 85, 117, 243, 131, 99, 244, 116, 116, 116 },
    { 118, 234, 235, 252, 116, 220, 188, 173, 147, 238, 237, 124 },
    { 123, 234, 228, 85, 117, 196, 99, 116, 243, 116, 116, 116 },
  }),
  ContextSetCfg::addCtxSet({
    { 33, 45, 38, 31, 11, 39, 39, 39 },
    { 26, 38, 53, 54, 29, 39, 39, 39 },
    { 18, 46, 38, 39, 59, 39, 39, 39 },
    { 8, 12, 12, 12, 0, 0, 0, 0 },
    { 8, 12, 12, 8, 0, 0, 0, 0 },
    { 8, 12, 12, 8, 0, 0, 0, 0 },
    { 18, 18, 18, 25, 25, 25, 25, 25 },
    { 11, 4, 18, 4, 25, 25, 25, 25 },
    { 18, 11, 18, 25, 25, 25, 25, 25 },
    { 100, 235, 235, 170, 132, 248, 254, 138 },
    { 117, 187, 179, 85, 135, 116, 132, 116 },
    { 227, 235, 235, 183, 132, 236, 238, 142 },
    { 117, 235, 98, 90, 138, 116, 116, 116 },
    { 132, 235, 235, 216, 116, 254, 253, 157 },
    { 107, 235, 98, 101, 228, 116, 116, 116 },
  }),
  ContextSetCfg::addCtxSet({
    { 26, 54, 39, 39, 26, 39, 39, 39, 0, 39, 39, 39 },
    { 19, 39, 54, 39, 19, 39, 39, 39, 40, 39, 39, 39 },
    { 18, 39, 39, 39, 11, 39, 39, 39, 0, 39, 39, 39 },
    { 8, 8, 8, 12, 8, 4, 4, 4, 0, 0, 0, 0 },
    { 8, 8, 8, 12, 8, 4, 4, 4, 0, 0, 0, 0 },
    { 8, 8, 12, 12, 8, 0, 0, 4, 0, 0, 0, 0 },
    { 18, 25, 25, 25, 18, 25, 25, 25, 25, 25, 25, 25 },
    { 18, 25, 25, 25, 18, 18, 25, 25, 25, 25, 25, 25 },
    { 18, 25, 25, 25, 18, 25, 25, 25, 25, 25, 25, 25 },
    { 117, 252, 252, 235, 116, 253, 153, 246, 116, 116, 116, 124 },
    { 119, 98, 86, 82, 117, 99, 99, 99, 141, 116, 116, 116 },
    { 118, 234, 252, 235, 116, 253, 229, 188, 116, 116, 116, 235 },
    { 121, 98, 92, 82, 117, 99, 99, 99, 197, 116, 116, 116 },
    { 124, 252, 235, 235, 132, 249, 189, 172, 116, 235, 235, 253 },
    { 118, 130, 98, 75, 116, 116, 116, 99, 139, 116, 116, 116 },
  }),
  ContextSetCfg::addCtxSet({
    { 18, 38, 54, 39, 18, 39, 39, 39 },
    { 26, 38, 62, 39, 18, 39, 39, 39 },
    { 26, 39, 39, 39, 18, 39, 39, 39 },
    { 8, 12, 8, 12, 0, 0, 0, 0 },
    { 8, 8, 12, 13, 0, 0, 0, 0 },
    { 8, 8, 8, 4, 0, 0, 0, 0 },
    { 25, 25, 25, 25, 25, 25, 25, 25 },
    { 11, 4, 25, 25, 25, 25, 25, 25 },
    { 25, 25, 25, 11, 25, 25, 25, 25 },
    { 100, 235, 167, 151, 116, 253, 238, 253 },
    { 118, 235, 152, 203, 118, 116, 132, 116 },
    { 242, 252, 185, 180, 116, 212, 213, 252 },
    { 118, 188, 138, 122, 135, 132, 116, 116 },
    { 131, 252, 252, 234, 116, 238, 237, 157 },
    { 116, 246, 99, 100, 158, 116, 132, 116 },
  }),
};
#endif

const CtxSet ContextSetCfg::ParFlag[2] = {
  ContextSetCfg::addCtxSet({
// ctx 672 to 692
 { 33, 40, 33, 26, 34, 42, 25, 33, 34, 34, 27, 25, 34, 42, 42, CNU, 33, 27, CNU, 42, CNU, },
 { 33, 25, 33, 18, 26, 42, 25, 33, 34, 42, 27, 25, 34, 42, 42, CNU, 26, 27, 42, CNU, CNU, },
 { 33, 25, 18, 26, 34, 27, 25, 26, 19, 42, CNU, 33, 19, 27, CNU, CNU, 34, 42, 20, 43, 20, },
 { DWS, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 13, 13, 13, 13, 9, 13, 13, 12, 13, },
 { 10, 9, 10, 13, 13, 13, 10, 13, 13, 13, 13, 13, 13, 13, 13, 13, 10, 13, 13, 13, 13, },
 { DWS, 9, 9, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, },
 { 11, DWE, DWE, DWE, 4, 4, 11, 11, 11, 4, 4, 4, DWE, 11, 4, 4, 4, 25, 11, 4, 4, },
 { 25, 11, 4, 25, 11, 11, 4, 4, 11, 4, 4, 4, 11, 4, 4, 4, 11, 11, 4, 4, 4, },
 { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, },
 { 90, 88, 91, 89, 105, 106, 88, 104, 89, 185, 106, 235, 106, DWO, 104, 122, 101, 122, 117, 105, 138, },
 { 180, 132, DWO, 116, 118, 122, 148, 122, 234, 120, 185, 100, 148, 122, 232, 136, 227, 227, 168, 198, 120, },
 { 90, 88, 91, 89, 105, 106, 88, 104, 89, 185, 106, 235, 106, DWO, 104, 122, 101, 122, 117, 105, 138, },
 { 180, 132, DWO, 116, 118, 122, 148, 122, 234, 120, 185, 100, 148, 122, 232, 136, 227, 227, 168, 198, 120, },
 { 90, 88, 91, 89, 105, 106, 88, 104, 89, 185, 106, 235, 106, DWO, 104, 122, 101, 122, 117, 105, 138, },
 { 180, 132, DWO, 116, 118, 122, 148, 122, 234, 120, 185, 100, 148, 122, 232, 136, 227, 227, 168, 198, 120, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 693 to 703
 { 33, 33, 26, 34, 42, 27, 33, 34, CNU, 27, CNU, },
 { 33, 25, 11, 11, 19, 27, 33, 34, CNU, 27, 43, },
 { 33, 25, 26, 34, 19, 27, 26, 50, CNU, CNU, CNU, },
 { 5, 12, 12, 12, 12, 13, 10, DWS, 10, DWS, 13, },
 { 9, 12, 12, DWS, DWS, 13, 13, 12, 12, 12, 13, },
 { 9, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, },
 { 4, 25, DWE, 11, 4, 4, 11, 25, DWE, 11, 4, },
 { 11, DWE, 25, 25, 25, 25, 25, 25, 25, 11, 4, },
 { 4, 4, DWE, DWE, 11, 4, 11, DWE, 4, 11, 11, },
 { 124, 235, 102, 179, 100, 200, 72, 129, 115, 211, 234, },
 { 196, 117, DWO, 106, 246, 104, 138, 232, 153, 252, 87, },
 { 124, 235, 102, 179, 100, 200, 72, 129, 115, 211, 234, },
 { 196, 117, DWO, 106, 246, 104, 138, 232, 153, 252, 87, },
 { 124, 235, 102, 179, 100, 200, 72, 129, 115, 211, 234, },
 { 196, 117, DWO, 106, 246, 104, 138, 232, 153, 252, 87, },
  }),
};

#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const CtxSet ContextSetCfg::GtxFlag[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 704 to 732
 { 34, 33, 41, 34, 42, 27, CNU, 36, 25, 18, 34, 42, CNU, 20, 44, 18, 26, 27, CNU, 28, 36, 37, 26, 19, 20, 28, 29, 37, 30, },
 { 25, 25, 10, 10, 26, 27, 27, 20, 40, 25, 26, 34, CNU, 20, 21, 25, 18, 34, CNU, 28, 36, 29, 18, 26, 20, 20, 29, 37, 30, },
 { 27, 33, 26, 34, 19, 27, CNU, 36, 10, 26, 19, 27, 20, 28, 29, 3, 27, 20, 28, 21, 21, 22, 19, 28, 21, 44, 22, 22, 38, },
 { 3, DWS, 7, 9, 7, 4, 7, DWS, 10, 12, 13, 12, 7, 10, 10, 12, DWS, DWS, DWS, 5, DWS, DWS, 5, 6, DWS, 9, DWS, 10, 12, },
 { 2, DWS, 7, 12, DWS, 7, 7, 9, 12, 12, 12, 12, 6, 7, 6, 12, DWS, DWS, 12, DWS, 5, DWS, 5, 5, DWS, DWS, 9, 9, 13, },
 { 5, 9, 9, 9, 9, 9, 10, 9, 13, 13, 9, 9, 6, 7, 6, 13, 10, 9, 10, 9, 9, 9, 6, DWS, 9, DWS, 10, 10, 9, },
 { 25, 25, 25, 25, 11, 4, DWE, 25, DWE, DWE, DWE, DWE, 4, 11, DWE, DWE, 11, DWE, 11, 4, 4, DWE, 4, 4, 11, DWE, 11, DWE, 25, },
 { 11, 25, 11, 25, DWE, 11, DWE, 25, DWE, 25, 25, 25, 4, 11, 11, DWE, 11, 25, 25, 4, 4, 11, 4, 4, DWE, DWE, 11, 4, 25, },
 { DWE, DWE, 4, 4, 4, 4, 4, DWE, 25, DWE, 4, 4, 4, 4, 4, 25, DWE, 4, DWE, 11, 4, 4, 4, 11, 4, 4, 11, 11, 4, },
 { 123, 252, 202, 216, 216, 106, 250, 168, 249, 219, 134, 200, 122, 163, 117, 219, 134, 124, 123, 124, 107, 120, 121, 151, DWO, 151, 108, 241, 134, },
 { 129, 99, 144, 113, 129, 132, 209, 114, 177, 99, 114, 91, 146, 194, 90, 114, 195, 131, 163, 116, 179, 92, 107, 115, 101, 99, DWO, 122, 161, },
 { 99, 103, 130, 116, 116, 133, 132, 131, 116, 123, 182, 213, 132, 147, 118, 107, 123, 183, 124, 123, 118, 118, 124, 140, 169, 248, 133, 121, 118, },
 { 125, 105, 118, 102, 101, 116, 117, 130, 105, 99, 105, 100, 116, 118, 88, 130, 147, 102, 131, 131, 115, 115, 116, 101, 101, 100, 107, 101, 131, },
 { 99, 104, 114, 101, 101, 135, 120, 131, 146, DWO, 164, 183, 135, 133, DWO, 85, 116, 164, 120, 228, 137, 148, 115, 134, 148, 184, 135, 134, 136, },
 { 132, 242, 132, 107, 107, 100, 99, 194, 177, 99, 117, 100, 107, 116, 107, 115, 117, 132, 117, 131, 88, 100, 117, 101, 243, 89, 114, 99, 84, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 733 to 747
 { 27, 26, 34, 27, 43, 28, 13, 37, 18, 19, CNU, 29, 14, 45, 15, },
 { 2, 17, 18, 11, 4, 20, 28, 13, 25, 26, 4, 13, 37, 30, 7, },
 { 27, 18, 26, 27, 20, 13, 13, 29, 11, 12, 28, 21, 37, 45, 46, },
 { 3, 12, 10, 10, 4, 2, DWS, 9, 7, 10, 4, DWS, 5, 5, DWS, },
 { 7, 5, DWS, DWS, DWS, 1, 4, 4, 7, 13, DWS, 10, 7, 13, 10, },
 { 1, 9, DWS, 9, DWS, 4, 5, DWS, DWS, DWS, 10, 6, 7, 13, 12, },
 { 25, DWE, DWE, 25, 4, 4, 4, 4, 11, 25, 25, 25, DWE, 4, 25, },
 { DWE, DWE, 25, 25, 25, 11, 11, DWE, 4, 25, 11, DWE, 11, 25, 25, },
 { 4, 25, DWE, DWE, 11, 11, 11, 4, 4, 4, 25, 4, 4, 4, 11, },
 { 154, 235, 250, 217, 220, 156, 123, 122, 250, 166, 116, 104, 116, 195, 105, },
 { 115, 75, 100, 100, 103, 123, 86, 104, 113, 161, DWO, 130, 123, 100, 83, },
 { 99, 88, 197, 147, 88, 108, 106, 123, 242, 130, 107, 102, 131, 195, 147, },
 { 117, 146, 102, 100, 164, 120, 132, 115, 108, 121, 243, 236, 149, 231, 186, },
 { 115, 98, 220, 197, 227, 102, 131, 150, 98, 146, 98, 103, 194, 131, 195, },
 { 117, 146, 100, 100, 164, 125, 120, 84, 198, 252, 234, 168, 149, 87, 164, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 748 to 776
 { 9, 17, 26, 19, CNU, 28, 36, 29, 25, 19, 20, 36, 29, 29, 45, 33, CNU, 36, 29, 37, 37, 30, 42, 36, 37, 45, 38, 38, 38, },
 { 1, 17, 26, 19, CNU, 36, 44, 29, 25, 34, 20, 36, 37, 37, 45, 33, 20, 36, 29, 37, 37, 30, 34, 28, 37, 37, 38, 30, 38, },
 { 25, 25, 19, 27, 20, 36, 21, 29, 18, 12, 28, 21, 29, 37, 22, 34, 28, 29, 29, 22, 22, 30, 28, 29, 45, 30, 23, 38, 23, },
 { 9, DWS, 6, 10, 13, 13, 10, 9, 9, 10, 12, 12, 10, 12, 13, 10, 10, 10, 10, 12, 12, 13, 6, 9, 10, 12, 13, 12, 13, },
 { 9, 9, 6, 10, 13, 13, 10, 13, 9, 10, 13, 13, 10, 13, 13, 10, 10, 10, 10, 10, 10, 10, 9, 9, 10, 10, 12, 13, 13, },
 { 9, 9, 6, 10, 10, 10, 13, 9, 13, 10, 13, 13, 13, 13, 13, 9, 10, 9, 10, 13, 13, 13, 9, 10, 10, 10, 12, 13, 13, },
 { 11, 4, 4, DWE, DWE, DWE, 11, DWE, 11, 11, 11, 11, 11, 11, 11, DWE, DWE, 11, 11, 11, 11, DWE, 11, 4, DWE, DWE, 25, DWE, DWE, },
 { 11, 11, 4, 11, DWE, DWE, 4, 25, 11, 11, 11, 4, 4, 4, 4, DWE, DWE, 4, 4, 4, 4, 4, DWE, 11, 11, 11, 11, 11, 11, },
 { DWE, DWE, 4, 4, 4, 4, 4, 11, DWE, 11, 11, 11, 11, 4, 11, 11, 11, 4, 4, 4, 4, 11, 11, DWE, 11, 11, 11, DWE, 11, },
 { 82, 84, 106, 115, 146, 162, 147, 150, 88, 101, 147, 88, 99, 100, 115, 99, 106, 130, 103, 115, 91, 104, 115, 179, 115, 100, 100, 131, 117, },
 { 163, 133, 121, DWO, 106, 168, 154, 162, 211, 118, 149, 235, 250, 235, 147, 116, 122, 152, 234, 219, 165, 89, 117, 107, 135, 219, 234, 203, 74, },
 { 82, 99, 106, 131, 117, 117, 116, 148, 90, 131, 179, 131, 102, 147, 117, 103, 117, 117, 117, 227, 132, 120, 115, 132, 117, 116, 101, 227, 139, },
 { 132, 117, 117, 146, 106, 131, 122, 147, 116, 121, 118, 201, 250, 134, 226, 118, 116, 135, 134, 106, 105, 85, 131, 105, DWO, 153, 234, 90, 82, },
 { 82, 99, 107, 147, 117, 106, 90, 148, 98, 116, 163, 131, 211, 115, 117, 131, 147, 107, DWO, 115, 179, 149, 131, 133, 132, 116, 211, 131, 183, },
 { 179, 116, 118, 146, 105, DWO, 138, 147, 106, 121, 121, 234, 148, 234, 100, 117, 116, 153, 122, 201, 135, 83, 147, 103, 106, 134, 235, DWO, 67, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 777 to 791
 { 17, 33, 27, 28, 36, 29, 52, 30, 50, 29, 37, 30, 38, 46, 46, },
 { 9, 25, 12, 20, 13, 6, 14, 22, CNU, 29, 22, 22, 15, 46, 23, },
 { 40, 33, CNU, 28, 13, 29, 29, 22, 43, 37, 45, 38, 46, 46, 46, },
 { 5, 13, 9, 12, 13, DWS, 12, 13, 4, 5, 1, DWS, 5, 7, 13, },
 { 9, 13, 12, 12, 12, DWS, DWS, 10, 5, DWS, 4, 10, DWS, 4, 9, },
 { 12, 12, 9, 12, 12, DWS, DWS, 12, DWS, DWS, DWS, 12, DWS, 12, 9, },
 { 4, DWE, 4, DWE, 25, 4, 4, 11, 4, DWE, 4, 25, DWE, 25, 11, },
 { 4, 11, 11, 25, 25, 25, DWE, DWE, 4, DWE, 11, 25, 25, 4, 4, },
 { DWE, 11, 11, 25, 25, 4, 4, DWE, 11, 11, 4, 25, 4, 25, 4, },
 { 104, 84, 131, 103, 163, 245, 182, 201, 116, 130, 132, 114, 100, 103, 161, },
 { 132, 116, 122, 166, 135, 118, 87, 83, DWO, 156, 135, 235, 250, 201, 113, },
 { 69, 85, 115, 100, 130, 118, 107, 117, 116, 115, 180, 99, 101, 105, 217, },
 { 117, 131, 117, 122, 212, 149, 105, 106, 118, 252, DWO, 250, 188, 200, 65, },
 { 74, 100, 131, 194, 210, 196, 102, 131, 115, 99, 180, 130, 91, 100, 215, },
 { 147, 100, 117, 120, 148, 218, 103, 202, DWO, 252, 118, 235, 252, 219, 81, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 792 to 820
 { 18, 25, 18, 26, 27, CNU, 43, 44, 25, 26, 27, CNU, 36, 36, 29, 33, 19, 43, 36, 29, 37, 45, 26, 20, 29, 37, 22, 22, 38, },
 { 25, 25, 33, 26, 19, CNU, 43, 36, 25, 26, 27, 20, 36, 44, 37, 33, 19, CNU, 36, 37, 37, 45, 26, CNU, 44, 44, 30, 30, 38, },
 { 26, 25, 26, 34, 27, 20, 28, 21, 18, 19, 12, 28, 13, 21, 14, 26, 20, 21, 29, 22, 22, 30, 20, 29, 45, 30, 38, 38, 23, },
 { DWS, DWS, 9, 12, 9, 10, 7, 9, 10, 13, 10, 13, 10, 10, 9, 9, 9, 10, 13, 12, 10, 9, 7, 12, 9, 12, 12, 12, 13, },
 { DWS, 9, 10, 13, 10, 7, 10, 10, 10, 10, 10, 13, 10, 10, 9, 9, 9, 10, 10, 12, 10, 9, 9, 10, 10, 10, 13, 12, 13, },
 { DWS, 9, 6, 10, 10, 10, 13, 9, 12, 10, 12, 12, 9, 10, 9, 10, 10, 9, 9, 12, 12, 13, 9, 10, 9, 10, 12, 12, 13, },
 { 25, 11, DWE, 25, DWE, DWE, 4, 25, 4, DWE, 11, DWE, DWE, 11, 11, 4, 11, DWE, 25, 25, 11, 11, 4, DWE, DWE, DWE, 25, 25, 25, },
 { DWE, 11, 4, 25, 4, 11, 11, 25, 4, 4, 4, 11, 11, 4, 4, 4, 11, 11, 4, 11, 4, 4, 4, 11, 11, 11, DWE, 11, DWE, },
 { 25, 4, 4, 4, 4, 4, 4, DWE, 11, 4, 11, 4, 4, 4, 4, 4, 11, 4, 4, 11, 11, 11, 11, 11, 4, 4, DWE, 4, 11, },
 { 92, 100, 99, 101, 103, 115, 232, 123, 82, 83, 98, 99, 99, 100, 107, 99, 115, 100, 98, 99, 99, 117, 100, 114, 146, 91, 130, 194, 116, },
 { 210, 116, 131, 178, 116, 116, 106, 99, 147, 122, 133, 138, 249, 234, 117, 116, 227, 131, 149, 219, 250, 103, 122, 118, 250, 218, 184, 120, 90, },
 { 99, 85, 101, 115, 103, 115, 242, 116, 83, 100, 104, 116, 107, 103, 117, 100, 116, 107, 105, 116, 102, 118, 91, 146, 117, 131, 131, 228, 180, },
 { 163, 116, 118, 146, 103, 162, 131, 116, 117, 134, 117, 120, 165, 251, 107, 132, 117, 124, 155, 122, 133, 100, DWO, 116, 136, 217, 107, 91, 83, },
 { 92, 87, 101, 226, 101, 99, 146, 147, 68, 114, 163, 179, 117, 131, 132, 100, 116, 131, 101, 85, 90, 117, 87, 146, 243, 102, 131, 132, 180, },
 { DWO, 227, 123, 130, 101, 117, 104, 116, 133, 150, 118, 151, DWO, 214, 106, 117, DWO, 137, 251, 219, 235, 89, 122, 226, 251, 250, 203, 219, 83, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 821 to 835
 { 41, 33, 19, 27, 28, 36, 44, 37, 26, 43, 29, 14, 30, 38, 46, },
 { 25, 25, 4, 4, 5, 13, 5, 29, 18, 13, 29, 22, 22, 15, 38, },
 { 26, 33, 19, 28, 21, 21, 21, 14, 19, 36, 37, 30, 53, 38, 46, },
 { DWS, 12, DWS, 9, 9, DWS, 6, 10, 10, 4, 9, 6, 5, DWS, 7, },
 { 10, 12, DWS, DWS, 4, 4, 6, 7, 13, DWS, 12, 7, 4, 6, 7, },
 { 6, 12, DWS, DWS, DWS, 4, DWS, DWS, DWS, DWS, 5, 7, DWS, DWS, 9, },
 { DWE, DWE, 11, 11, 11, 4, 4, 4, 25, 25, 25, DWE, DWE, DWE, 25, },
 { 11, 4, 25, 25, 25, 25, 25, DWE, 25, 25, 25, DWE, 11, 25, 25, },
 { DWE, DWE, 11, 25, DWE, 4, DWE, 4, 4, 25, 11, 11, 4, 4, 4, },
 { 149, 227, 102, 180, 115, 102, 138, 167, 82, 115, 88, 145, 100, 132, 145, },
 { 242, 102, 121, 107, 103, 121, 117, 89, 134, 237, 181, 155, 201, 185, 101, },
 { 107, 83, 101, 100, 130, 104, 132, 132, 84, 100, 91, 109, 102, 148, 122, },
 { 147, 195, 181, 138, 185, 166, 125, 124, 136, 252, 235, 248, 235, 122, 98, },
 { 98, 91, 116, 100, 178, 107, 117, 116, 83, 99, 102, 100, 228, 102, 136, },
 { 132, 179, 121, 137, 252, 137, 118, 103, 236, 235, 252, 234, 115, 236, 82, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 836 to 864
 { 28, 19, 50, 50, 50, CNU, CNU, 36, 34, 19, 19, 27, CNU, 43, 44, 27, 27, CNU, 43, 36, 36, 37, 27, 27, 28, 28, 29, 29, 30, },
 { 42, 50, 11, 42, 19, CNU, CNU, 28, 34, 34, 19, 42, 20, 28, 29, 34, 19, 27, 28, 13, 21, 29, 19, 27, 20, 28, 29, 29, 30, },
 { 44, 34, 42, 27, 27, CNU, CNU, 36, 11, 19, 27, 27, 28, 28, 29, CNU, 12, 20, 28, 21, 29, 22, 43, 36, 29, 44, 30, 30, 38, },
 { 0, 9, 5, 0, 7, 4, 4, 5, 2, 9, 12, 13, 13, 13, 10, 6, 9, 13, 13, 13, 13, 9, 2, 9, 10, 13, 13, 13, 12, },
 { 10, 10, 10, 13, 13, 13, 10, 10, 2, 7, 12, 12, 10, 10, 10, 4, 12, DWS, 13, 10, 10, 9, 2, 9, 9, 13, 13, 13, 10, },
 { 2, 5, 10, 9, 10, 10, 10, 10, 4, 13, 13, 13, 13, 13, 10, 10, 10, 10, 10, 13, 10, 10, 3, 10, 10, 13, 12, 12, 13, },
 { 25, 25, 25, 4, 25, 25, DWE, DWE, 11, 4, 11, DWE, DWE, DWE, 25, 4, 11, 25, 25, 25, 25, DWE, 4, 11, 4, DWE, 25, 25, 11, },
 { 25, 25, DWE, 11, 4, 4, 4, 25, 4, 4, DWE, 4, 4, 4, DWE, 4, DWE, 4, DWE, 4, 4, 11, 4, 11, 4, DWE, DWE, DWE, 4, },
 { 11, DWE, DWE, 11, 4, 4, 4, 25, 4, DWE, DWE, 11, DWE, 11, DWE, 25, 4, 11, 11, DWE, 4, 4, 4, 4, 4, DWE, 11, 11, 11, },
 { 139, 184, 217, 170, 247, 252, 202, 252, 123, 130, 211, 136, 166, 218, 154, 106, 116, 136, 230, 154, 218, 155, 116, DWO, 101, 180, 117, 234, 216, },
 { 134, 136, 147, 246, 145, 245, 246, 211, 211, 166, 153, 202, 233, 234, 243, 227, 212, 169, 234, 234, 233, 211, 180, 132, 234, 234, 234, 234, 150, },
 { 106, 107, 83, 89, 114, 185, 250, 154, 88, 210, 251, 219, 148, 202, 135, 90, 107, 249, 183, 133, 166, 134, 123, 140, 122, 164, 118, 117, 155, },
 { 138, 195, 179, 171, 201, 167, 136, 195, 133, 139, 118, 227, 234, 234, 133, 148, 243, 132, 229, 249, 234, 227, 124, 132, 245, DWO, 235, 235, 101, },
 { 116, 117, 85, 85, 146, 132, 234, 154, 102, 90, 117, 234, 120, 233, 150, 88, 243, 250, 250, 234, 138, 153, 116, 199, 138, 234, 105, 103, 218, },
 { 133, 148, 147, 215, 244, 249, 153, 211, 165, 195, 234, 147, 234, 234, 164, 137, 243, 122, 136, 134, 250, 131, 123, 196, 250, 147, 235, 235, 83, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 865 to 879
 { 12, 27, 43, CNU, 43, 36, 44, 37, 42, 27, 12, 20, 21, 29, 22, },
 { 57, 58, 11, 43, 43, 44, 13, 29, 34, 19, 12, 20, 13, 29, 14, },
 { 13, 11, CNU, 20, 28, 13, 13, 29, 28, 20, 28, 36, 29, 29, 30, },
 { 3, 4, 4, 12, 12, 13, 10, 13, 3, 6, 4, DWS, 13, 13, 10, },
 { 7, 6, DWS, 13, DWS, 13, 10, 5, 13, 13, 13, 13, 13, 13, 13, },
 { 3, 4, 4, DWS, 12, 12, 12, 9, 1, 5, 6, 10, 13, 13, 10, },
 { DWE, 4, 4, 25, 4, 25, 11, 11, DWE, 25, DWE, 25, 25, 4, 25, },
 { 4, 4, 4, DWE, 4, 11, 11, 4, 11, 4, 4, 4, 25, 4, 25, },
 { DWE, 4, 4, DWE, 25, 25, 25, 11, 11, 11, 11, 11, 11, 4, 4, },
 { 202, 141, 120, 148, 152, 137, 233, 121, 138, 137, 132, 130, 231, 169, 179, },
 { 120, 164, 105, 89, 72, 103, 103, 131, 132, 131, 137, 155, 234, 234, 193, },
 { 146, 75, 74, 116, 101, 131, 147, 218, 99, 242, 89, 99, 230, 217, 234, },
 { 120, 117, 214, 197, 246, 234, 232, 115, 148, 138, 167, 249, 234, 200, 106, },
 { 135, 139, 108, 101, 103, 131, 117, 251, 115, 115, 105, 100, 234, 218, 250, },
 { 120, 105, 229, 235, 199, 234, 219, 114, 180, 120, 234, 180, 201, 154, 101, },
  }),
};
#else
const CtxSet ContextSetCfg::GtxFlag[4] = {
  ContextSetCfg::addCtxSet({
    { 25, 0, 40, 25, 33, 26, 0, 17, 25, 33, 19, 9, 25, 26, 34, 20, 25, 18, 19, 20, 37 },
    { 1, 0, 9, 17, 25, 26, 0, 9, 25, 33, 19, 9, 25, 33, 34, 20, 25, 33, 19, 27, 29 },
    { 25, 1, 40, 25, 33, 11, 9, 25, 25, 18, 12, 17, 33, 26, 19, 13, 33, 19, 20, 28, 22 },
    { 1, 9, 10, 10, 7, 5, 9, 9, 10, 9, 6, 9, 9, 10, 6, 5, 6, 9, 6, 10, 10 },
    { 2, 8, 6, 9, 7, 5, 12, 13, 10, 13, 3, 9, 9, 10, 10, 6, 6, 6, 10, 10, 10 },
    { 4, 12, 6, 9, 10, 5, 12, 9, 9, 9, 5, 13, 10, 10, 10, 10, 6, 9, 10, 10, 9 },
    { 11, 11, 25, 18, 11, 18, 4, 4, 11, 4, 18, 11, 11, 25, 11, 4, 11, 18, 11, 25, 18 },
    { 11, 11, 4, 4, 4, 18, 18, 11, 4, 11, 4, 4, 11, 18, 18, 11, 11, 4, 18, 18, 11 },
    { 4, 25, 4, 11, 11, 18, 18, 11, 4, 11, 11, 18, 18, 18, 18, 18, 4, 18, 18, 18, 4 },
    { 115, 99, 86, 85, 102, 227, 82, 84, 100, 103, 117, 87, 100, 102, 106, 107, 98, 103, 115, 102, 116 },
    { 164, 131, 193, 130, 116, 131, 212, 148, 116, 147, 116, 243, 131, 118, 147, 131, 195, 131, 116, 120, 122 },
    { 118, 84, 99, 131, 122, 117, 65, 69, 99, 85, 118, 82, 90, 104, 99, 115, 98, 101, 99, 103, 116 },
    { 131, 116, 243, 227, 178, 116, 148, 179, 147, 103, 117, 132, 131, 130, 146, 105, 243, 116, 116, 116, 115 },
    { 99, 70, 98, 84, 82, 147, 68, 83, 123, 102, 107, 88, 100, 116, 104, 115, 99, 117, 106, 116, 119 },
    { 119, 147, 121, 195, 116, 117, 119, 123, 132, 117, 117, 116, 121, 116, 120, 115, 123, 116, 118, 121, 107 },
  }),
  ContextSetCfg::addCtxSet({
    { 25, 9, 25, 33, 26, 12, 17, 33, 11, 28, 37 },
    { 1, 1, 25, 18, 11, 12, 17, 33, 11, 4, 22 },
    { 25, 9, 25, 18, 26, 4, 25, 26, 27, 28, 37 },
    { 2, 8, 8, 5, 8, 5, 6, 8, 7, 5, 5 },
    { 3, 10, 12, 9, 9, 7, 5, 12, 5, 7, 8 },
    { 1, 12, 8, 8, 8, 2, 5, 4, 6, 4, 7 },
    { 11, 4, 4, 4, 4, 4, 4, 25, 25, 11, 4 },
    { 4, 4, 4, 4, 4, 4, 4, 25, 18, 18, 25 },
    { 4, 25, 4, 18, 18, 4, 4, 4, 18, 4, 4 },
    { 115, 201, 135, 122, 148, 196, 98, 83, 97, 131, 131 },
    { 131, 108, 107, 119, 91, 105, 195, 140, 150, 137, 124 },
    { 119, 106, 87, 105, 105, 135, 98, 83, 99, 102, 130 },
    { 130, 119, 101, 118, 102, 121, 164, 123, 152, 114, 168 },
    { 115, 67, 100, 100, 211, 119, 98, 99, 98, 103, 117 },
    { 120, 163, 118, 123, 104, 116, 120, 183, 219, 137, 105 },
  }),
  ContextSetCfg::addCtxSet({
    { 9, 17, 26, 27, 35, 21, 25, 27, 43, 36, 37, 33, 35, 36, 29, 30, 42, 36, 37, 45, 38 },
    { 1, 17, 26, 42, 35, 44, 25, 34, 35, 36, 37, 33, 20, 36, 29, 22, 34, 28, 37, 37, 38 },
    { 25, 25, 19, 27, 20, 21, 33, 12, 28, 21, 22, 34, 28, 29, 29, 30, 28, 29, 45, 30, 23 },
    { 9, 8, 9, 10, 10, 9, 9, 10, 13, 13, 10, 9, 10, 9, 10, 10, 5, 9, 10, 13, 13 },
    { 9, 9, 6, 10, 10, 10, 9, 10, 10, 13, 10, 10, 10, 10, 10, 10, 9, 9, 10, 10, 13 },
    { 9, 9, 6, 10, 10, 9, 13, 10, 13, 13, 13, 9, 10, 10, 10, 13, 9, 10, 10, 10, 13 },
    { 11, 4, 11, 11, 11, 18, 11, 11, 18, 4, 4, 18, 18, 11, 11, 4, 11, 11, 18, 18, 18 },
    { 11, 11, 4, 11, 11, 18, 11, 11, 4, 4, 4, 18, 18, 11, 11, 4, 18, 11, 11, 11, 11 },
    { 18, 18, 4, 4, 4, 18, 18, 11, 11, 11, 18, 11, 11, 11, 4, 11, 18, 18, 11, 11, 11 },
    { 82, 85, 102, 146, 134, 122, 90, 115, 116, 227, 116, 107, 115, 116, 116, 131, 116, 116, 116, 116, 119 },
    { 211, 107, 106, 116, 101, 116, 243, 117, 117, 137, 131, 117, 116, 123, 136, 118, 117, 116, 119, 134, 85 },
    { 82, 83, 107, 121, 120, 116, 89, 116, 163, 147, 116, 99, 115, 115, 116, 116, 104, 227, 116, 122, 132 },
    { 211, 132, 117, 100, 105, 130, 227, 116, 117, 182, 117, 116, 117, 133, 135, 99, 116, 131, 120, 103, 85 },
    { 83, 99, 107, 131, 117, 119, 98, 116, 195, 101, 102, 131, 119, 115, 120, 116, 115, 132, 132, 122, 164 },
    { 122, 116, 118, 101, 106, 117, 116, 122, 121, 234, 138, 118, 117, 170, 121, 102, 120, 106, 106, 106, 86 },
  }),
  ContextSetCfg::addCtxSet({
    { 17, 33, 27, 28, 21, 37, 42, 37, 45, 38, 46 },
    { 9, 25, 27, 20, 13, 22, 35, 29, 22, 30, 23 },
    { 40, 33, 35, 28, 13, 37, 43, 37, 45, 38, 46 },
    { 9, 12, 9, 9, 12, 9, 5, 4, 4, 10, 8 },
    { 9, 13, 12, 13, 13, 13, 5, 8, 4, 5, 8 },
    { 12, 12, 9, 12, 12, 9, 8, 8, 8, 13, 10 },
    { 18, 11, 11, 11, 25, 4, 11, 4, 4, 25, 11 },
    { 4, 11, 4, 11, 11, 4, 4, 18, 11, 18, 11 },
    { 18, 11, 11, 25, 18, 11, 11, 18, 11, 25, 11 },
    { 106, 115, 101, 115, 227, 197, 147, 243, 244, 98, 116 },
    { 242, 211, 119, 203, 117, 123, 117, 251, 120, 249, 118 },
    { 86, 84, 87, 195, 179, 134, 116, 115, 132, 178, 132 },
    { 195, 104, 166, 202, 106, 88, 148, 252, 138, 118, 236 },
    { 75, 115, 131, 178, 131, 227, 179, 92, 164, 98, 243 },
    { 131, 91, 117, 120, 137, 116, 136, 252, 105, 198, 84 },
  }),
};
#endif

const CtxSet ContextSetCfg::LastX[2] = {
  ContextSetCfg::addCtxSet({
// ctx 880 to 915
 { 6, 6, 5, 6, 7, 12, 14, 7, 6, 5, 14, 7, 6, 6, 44, 14, 14, 6, 22, 22, 60, 14, 14, 38, 61, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 6, 5, 4, 6, 6, 4, 14, 14, 5, 4, 14, 14, 6, 6, 60, 14, 14, 13, 14, 38, 46, 14, 14, 46, 62, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 6, 13, 12, 14, 6, 4, 6, 6, 14, 4, 14, 14, 22, 6, 11, 14, 15, 15, 22, 13, 3, 7, 52, 6, 20, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 5, 6, 5, 6, 5, 1, 6, 5, 2, 1, 6, 1, 1, 1, 1, 2, 1, 1, 1, 0, 0, 1, 0, 1, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 6, 6, 5, 6, 5, 1, 6, 5, 1, 1, 6, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 1, 4, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 9, 5, 5, 6, 5, 4, 6, 5, 1, 1, 6, 1, 1, 1, 0, 1, 2, 1, 1, 0, 0, 2, 0, 1, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { DWE, DWE, DWE, DWE, DWE, 4, DWE, DWE, DWE, DWE, DWE, 4, DWE, 25, 25, 11, 4, DWE, 25, 25, 25, 11, DWE, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, DWE, DWE, DWE, DWE, 11, DWE, DWE, DWE, DWE, DWE, 11, DWE, 25, 25, 11, 11, DWE, DWE, 25, 25, DWE, 25, 25, 11, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 11, 11, DWE, DWE, DWE, DWE, 25, DWE, DWE, 25, DWE, 25, 25, 25, 11, DWE, DWE, 25, 25, 25, DWE, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 116, 116, 118, 123, 124, 117, 123, 117, 132, 117, 121, 118, 125, 141, 115, 132, 117, 118, 123, 164, 116, 212, 205, 244, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 115, 116, 147, 124, 132, 163, 132, 243, 116, 132, 133, 148, 149, 115, 133, 164, 132, 243, 181, 116, 148, 254, 249, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 117, 122, 123, 124, 117, 123, 122, 125, 117, 123, 124, 121, 244, 116, 133, 118, 117, 132, 244, 116, 228, 221, 245, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 118, 117, 137, 244, 102, 163, 244, 149, 116, 163, 245, 148, 245, 115, 133, 164, 148, 245, 245, 116, 164, 250, 244, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 242, 117, 117, 123, 124, 117, 123, 124, 125, 131, 123, 125, 125, 244, 116, 125, 118, 117, 131, 244, 116, 153, 170, 247, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 242, 117, 117, 139, 245, 103, 163, 165, 149, 116, 139, 197, 133, 245, 116, 212, 243, 132, 244, 254, 116, 213, 238, 249, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 916 to 918
 { 27, 28, 58, },
 { 42, 42, 33, },
 { 20, 4, 3, },
 { 1, 0, 1, },
 { 5, 5, 5, },
 { 6, 4, 1, },
 { 11, 11, DWE, },
 { 11, DWE, 11, },
 { DWE, DWE, 4, },
 { 117, 118, 117, },
 { 132, 133, 147, },
 { 117, DWO, DWO, },
 { 243, 133, 132, },
 { 121, DWO, 120, },
 { 195, 244, 117, },
  }),
};

const CtxSet ContextSetCfg::LastY[2] = {
  ContextSetCfg::addCtxSet({
// ctx 919 to 954
 { 5, 13, 20, 6, 14, 4, 6, 7, 13, 12, 6, 14, 13, 6, 29, 21, 7, 13, 13, 44, 29, 21, 6, 30, 38, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 5, 5, 4, 6, 6, 19, 6, 14, 5, 12, 21, 14, 13, 6, 22, 21, 7, 5, 13, 45, 7, 28, 13, 6, 31, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 13, 13, 12, 14, 6, 4, 14, 14, 5, 11, 14, 7, 6, 5, 18, 37, 38, 22, 14, 12, 18, 36, 30, 13, 20, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 5, 6, 6, 6, 6, 1, 6, 6, 1, 1, 2, 5, 1, 1, 0, 2, 2, 1, 1, 1, 0, 1, 4, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 5, 5, 5, 6, 6, 1, 6, 6, 6, 1, 6, 5, 1, 1, 0, 2, 2, 1, 1, 1, 0, 1, DWS, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 9, 5, DWS, 6, 5, 4, 6, 6, 1, 0, 6, 6, 1, 1, 0, 2, 1, 1, 1, 1, 0, 1, 4, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 11, DWE, DWE, DWE, DWE, 4, DWE, DWE, 11, DWE, 4, 11, 11, 25, 25, 11, 4, 11, DWE, 25, 25, DWE, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 4, 11, 11, DWE, DWE, 4, DWE, DWE, 25, DWE, DWE, 11, 11, 25, 25, DWE, 4, 11, DWE, 25, 25, DWE, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 4, 4, DWE, DWE, 4, DWE, DWE, 11, DWE, 25, DWE, DWE, 25, 25, DWE, 11, DWE, 25, 25, 25, 25, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 116, 117, 123, 116, 118, DWO, 116, 118, 134, 117, 118, 104, 125, 141, 116, 118, 117, 118, 125, 244, 116, 148, 172, 253, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 163, 115, 116, 123, 131, 117, 118, 123, 246, 117, 132, 116, 165, 253, 117, 117, DWO, 164, 243, 245, 117, 117, 252, 252, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 117, 118, 117, 123, 124, 118, 118, 123, 135, 118, 118, 116, 125, 156, 116, 125, 118, 118, 124, 244, 116, 164, 138, 254, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 118, 118, 123, 138, 117, 118, 163, 253, 117, 132, 148, 246, 253, 116, 118, 125, 244, 244, 245, 116, 117, 254, 245, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 118, 133, 123, 124, 107, 120, 123, 141, DWO, 123, 115, 125, 157, 116, 124, 118, 124, 125, 244, 116, 138, 133, 254, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 107, 118, 103, 123, 137, 117, DWO, 147, 253, 117, 123, 242, 251, 253, 116, 120, 134, 245, 229, 252, 116, 117, 252, 252, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 955 to 957
 { 26, 28, 42, },
 { 41, 19, 33, },
 { 20, 4, 11, },
 { 2, 4, 0, },
 { 2, 5, 2, },
 { 7, 5, 4, },
 { 11, 25, 11, },
 { 4, 11, 4, },
 { 11, DWE, 4, },
 { 116, 118, DWO, },
 { 242, 132, 148, },
 { 124, 125, 118, },
 { 242, 148, 132, },
 { DWO, 124, 118, },
 { 178, 243, 116, },
  }),
};


#if JVET_AG0143_INTER_INTRA
const CtxSet ContextSetCfg::SigCoeffGroupCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 958 to 959
 { 18, 31, },
 { 11, 31, },
 { 10, 38, },
 { 4, 5, },
 { 5, 5, },
 { DWS, 5, },
 { 11, DWE, },
 { 11, DWE, },
 { 25, DWE, },
 { 116, 148, },
 { 102, 101, },
 { 132, 132, },
 { 102, 117, },
 { 131, 195, },
 { 101, 132, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 960 to 961
 { 10, 23, },
 { 25, 38, },
 { 2, 37, },
 { 9, 10, },
 { 9, 13, },
 { 0, 0, },
 { 11, 11, },
 { 11, 25, },
 { 25, 25, },
 { 85, 234, },
 { 131, 101, },
 { 86, 234, },
 { 211, 106, },
 { 244, 252, },
 { 138, 164, },
  }),
};



#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const CtxSet ContextSetCfg::SigFlagCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 962 to 979
 { 25, 19, 28, 14, 22, 30, 18, 28, 29, 30, 30, 38, 26, 45, 30, 38, 38, 38, },
 { 25, 19, 20, 14, 22, 30, 18, 28, 29, 30, 30, 38, 26, 45, 30, 38, 38, 38, },
 { 17, 26, 27, 29, 37, 30, 9, 42, 28, 37, 45, 30, 33, 59, 36, 45, 30, 38, },
 { 13, 9, 9, 13, 13, 10, 10, 9, 10, 13, 13, 10, DWS, 9, 10, 10, 10, 9, },
 { 13, 9, 9, 10, 10, 10, 10, 9, 9, 13, 13, 13, DWS, 9, 10, 10, 10, 10, },
 { 13, 9, 9, 9, 10, 10, 9, 9, DWS, 13, 10, 10, DWS, 9, 9, 9, 10, 9, },
 { DWE, 4, 11, 11, 4, 4, DWE, 4, 4, 4, 4, 4, 4, DWE, 11, 4, 4, 11, },
 { DWE, 11, DWE, 4, 4, 4, DWE, 11, 11, 11, 4, 4, 11, DWE, DWE, 4, 4, 11, },
 { 11, 11, DWE, 11, 4, 4, 11, 11, 4, 11, 4, 4, 11, DWE, DWE, 4, 4, 11, },
 { 84, 138, 148, 117, 121, 151, 98, 179, 147, 105, 218, 169, 164, 117, 131, 134, 213, 235, },
 { 122, 164, 147, 106, 87, 85, 120, 121, 116, 234, 83, 83, 106, 118, 163, 106, 84, 82, },
 { 87, 117, 117, 116, 122, 133, 102, 116, 116, 147, 181, 228, 118, 147, 117, 132, 132, 153, },
 { 122, 228, 117, 138, 122, 101, 118, 123, DWO, 250, 102, 90, 117, 123, 132, 133, 100, 82, },
 { 83, 107, 117, 147, 250, 136, 103, 163, 132, 100, 231, 201, 131, 131, 117, DWO, 180, 219, },
 { 196, 133, 117, 218, 99, 88, 118, 227, 118, 234, 87, 85, 121, 120, 123, 117, 86, 81, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 980 to 991
 { 33, 12, 28, 29, 37, 22, 26, 53, 53, 46, 46, 46, },
 { 25, 27, 28, 29, 37, 22, 26, 53, 53, 46, 46, 46, },
 { 25, CNU, 28, 21, 37, 22, 33, 60, 60, 38, 46, 23, },
 { 13, 13, 10, 13, 13, 12, 10, 5, DWS, DWS, DWS, DWS, },
 { 12, 12, 9, 9, 13, 9, 6, 5, DWS, DWS, DWS, 9, },
 { 13, 13, 6, 10, 9, DWS, 7, 5, 5, DWS, 9, 13, },
 { DWE, DWE, 4, 11, DWE, 11, DWE, 4, 11, DWE, 11, 4, },
 { DWE, DWE, 11, 4, DWE, 4, 4, 4, 4, 11, 4, DWE, },
 { 25, 25, 4, 11, 4, 11, DWE, 11, 4, 4, 4, 11, },
 { 98, 117, 131, 100, 102, 228, 116, 196, 116, 147, 117, 252, },
 { 210, 179, 195, 196, 168, 101, 104, DWO, 236, 252, 252, 82, },
 { 90, 104, 116, 104, 105, 116, 117, 133, 132, 132, 118, 199, },
 { 108, 132, 123, 139, 116, 136, 118, 108, 135, 252, 234, 82, },
 { 144, 177, 147, 106, 115, 131, 136, 149, 107, 87, 231, 201, },
 { 130, 122, 122, 169, 85, 90, DWO, 99, 102, 153, 83, 68, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 992 to 1009
 { 26, 38, 46, 54, 54, 54, 11, 39, 39, 39, 39, 39, 28, 39, 39, 39, 39, 39, },
 { 26, 38, 46, 54, 54, 54, 11, 39, 39, 39, 39, 39, 28, 39, 39, 39, 39, 39, },
 { 26, 45, 53, 46, 46, 46, 11, 54, 61, 39, 39, 39, 26, 39, 39, 39, 39, 39, },
 { DWS, 13, 12, DWS, DWS, DWS, DWS, 4, 0, 0, 0, 4, 4, 0, 0, 0, 0, 0, },
 { 9, 13, 12, DWS, DWS, DWS, DWS, 4, 4, 0, 0, 4, 4, 0, 0, 0, 0, 0, },
 { DWS, 13, 12, DWS, DWS, DWS, DWS, DWS, DWS, 0, 0, 4, 0, 0, 0, 0, 0, 0, },
 { 11, 4, 25, 4, 4, 4, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 11, 4, 25, 4, 4, 4, DWE, DWE, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, },
 { 4, 4, DWE, DWE, DWE, 11, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 132, 234, 235, 252, 252, 252, 115, 253, 254, 233, 150, 253, 131, 252, 254, 238, 252, 116, },
 { 124, 218, 107, 85, 83, 82, 117, 99, 116, 116, 116, 99, 147, 116, 132, 132, 116, 116, },
 { 148, 234, 235, 252, 236, 236, 116, 253, 248, 168, 156, 215, 115, 252, 251, 252, 203, 253, },
 { 120, 234, 123, 87, 84, 83, 117, 243, 162, 99, 99, 99, 148, 116, 116, 116, 132, 116, },
 { 102, 234, 235, 236, 252, 252, 116, 252, 251, 218, 184, 137, 116, 237, 238, 254, 235, 252, },
 { 122, 233, 133, 85, 83, 82, 108, 210, 114, 116, 116, 99, 148, 116, 116, 116, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1010 to 1021
 { 18, 46, 38, 39, 39, 39, 44, 39, 39, 39, 39, 39, },
 { 18, 46, 38, 39, 39, 39, 44, 39, 39, 39, 39, 39, },
 { 33, 30, 46, 31, 31, 31, 19, 39, 39, 39, 39, 39, },
 { DWS, 12, 12, 4, 4, 4, 0, 0, 0, 0, 0, 0, },
 { DWS, 12, 12, DWS, DWS, 4, 0, 0, 0, 0, 0, 0, },
 { DWS, 12, 13, 0, 4, 9, 0, 0, 0, 0, 0, 1, },
 { DWE, 11, DWE, DWE, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, 11, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, DWE, 25, DWE, 25, 4, 25, 25, 25, 25, 25, 25, },
 { 180, 235, 235, 253, 253, 220, 116, 253, 116, 235, 252, 238, },
 { 118, 102, 81, 99, 99, 99, 164, 116, 116, 116, 116, 116, },
 { 148, 235, 235, 232, 246, 229, 116, 254, 252, 238, 164, 116, },
 { 117, 185, 129, 99, 83, 82, 227, 116, 116, 116, 116, 99, },
 { 163, 199, 199, 187, 252, 218, 116, 252, 235, 116, 116, 251, },
 { 132, 103, 162, 116, 99, 81, 244, 116, 116, 116, 116, 115, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1022 to 1039
 { 18, 39, 39, 39, 39, 39, 11, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { 18, 39, 39, 39, 39, 39, 11, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { 26, 54, 39, 39, 39, 39, 26, 39, 39, 39, 39, 39, 0, 39, 39, 39, 39, 39, },
 { DWS, DWS, 12, 12, 12, 12, DWS, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, 12, 12, 12, 12, DWS, 0, 0, 0, 0, 4, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, DWS, 12, 13, 12, DWS, 4, 0, 0, 1, 4, 0, 0, 0, 0, 0, 0, },
 { DWE, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { DWE, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 25, 25, 25, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 118, 252, 235, 235, 235, 235, 115, 253, 222, 154, 214, 249, 116, 116, 235, 116, 238, 253, },
 { 118, 82, 66, 65, 65, 71, 117, 116, 116, 116, 116, 116, 217, 116, 116, 116, 116, 116, },
 { 121, 252, 252, 235, 218, 235, 132, 203, 215, 168, 167, 169, 116, 237, 238, 212, 253, 236, },
 { 118, 130, 98, 75, 75, 73, 117, 99, 99, 99, 99, 99, 136, 116, 116, 116, 116, 116, },
 { 123, 252, 252, 235, 234, 149, 228, 253, 251, 236, 253, 253, 132, 116, 116, 116, 238, 253, },
 { 120, 82, 82, 65, 65, 69, 103, 99, 116, 116, 115, 99, 154, 116, 116, 116, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1040 to 1051
 { 11, 39, 39, 39, 39, 39, 10, 39, 39, 39, 39, 39, },
 { 11, 39, 39, 39, 39, 39, 10, 39, 39, 39, 39, 39, },
 { 18, 38, 54, 39, 39, 39, 18, 39, 39, 39, 39, 39, },
 { DWS, DWS, DWS, 4, 4, 4, 0, 0, 0, 0, 0, 0, },
 { DWS, DWS, DWS, 4, DWS, 4, 4, 0, 0, 0, 0, 0, },
 { DWS, 10, 4, 4, 0, 0, 1, 0, 0, 0, 0, 0, },
 { DWE, 25, 25, 25, 25, 4, 25, 25, 25, 25, 25, 25, },
 { DWE, 25, 25, 11, 25, 4, 25, 25, 25, 25, 25, 25, },
 { 25, 25, 11, 25, 25, 25, 25, 25, 25, 25, 25, 25, },
 { 131, 252, 252, 253, 141, 253, 116, 254, 252, 252, 238, 222, },
 { 118, 99, 82, 99, 100, 116, 126, 116, 116, 116, 132, 116, },
 { 131, 252, 252, 250, 157, 251, 115, 166, 254, 252, 253, 238, },
 { 118, 122, 83, 82, 100, 116, 134, 116, 132, 132, 116, 116, },
 { 146, 250, 253, 253, 237, 238, 115, 252, 235, 116, 116, 116, },
 { 163, 150, 99, 99, 116, 116, 125, 116, 116, 132, 116, 116, },
  }),
};
#else
const CtxSet ContextSetCfg::SigFlagCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
  { 25, 19, 28, 14, 18, 28, 29, 30, 19, 45, 30, 38, },
  { 25, 19, 20, 14, 18, 28, 29, 30, 19, 45, 30, 38, },
  { 17, 26, 27, 29, 9, 42, 28, 37, 33, 44, 36, 45, },
  { 13, 9, 10, 10, 10, 9, 9, 13, 10, 9, 10, 10, },
  { 13, 9, 9, 10, 10, 9, 10, 13, 9, 9, 10, 10, },
  { 13, 9, 9, 5, 9, 9, 9, 9, DWS, DWS, 9, 10, },
  { DWE, 4, DWE, 11, DWE, 4, 4, 4, DWE, DWE, 11, 11, },
  { 11, 4, 11, 4, 11, 4, 4, 4, 11, 11, 11, 11, },
  { 11, 11, DWE, 4, 4, DWE, DWE, 4, DWE, 11, DWE, DWE, },
  { 85, 117, 131, 133, 105, 107, 107, 123, 99, 116, 117, 149, },
  { 121, 180, 118, 124, 117, 123, 123, 102, 134, 136, 123, 90, },
  { 85, 117, 131, 133, 105, 107, 107, 123, 99, 116, 117, 149, },
  { 121, 180, 118, 124, 117, 123, 123, 102, 134, 136, 123, 90, },
  { 85, 117, 131, 133, 105, 107, 107, 123, 99, 116, 117, 149, },
  { 121, 180, 118, 124, 117, 123, 123, 102, 134, 136, 123, 90, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 25, 12, 28, 29, 26, 53, 53, 46, },
  { 25, 12, 28, 29, 34, 53, 45, 38, },
  { 25, 27, 43, 29, 33, 60, 60, 38, },
  { 13, 13, 10, 10, 6, 5, DWS, DWS, },
  { 13, 12, 10, 12, 5, 5, DWS, DWS, },
  { 9, 9, 5, 13, 4, 1, 4, 13, },
  { DWE, DWE, 11, 4, 11, 4, 4, 11, },
  { DWE, 4, 4, 4, 4, 4, 11, DWE, },
  { DWE, DWE, 4, 4, 25, 4, 11, 25, },
  { 131, 103, 117, 116, 117, 118, 102, 180, },
  { 107, 136, 123, 123, 122, 107, 200, 220, },
  { 131, 103, 117, 116, 117, 118, 102, 180, },
  { 107, 136, 123, 123, 122, 107, 200, 220, },
  { 131, 103, 117, 116, 117, 118, 102, 180, },
  { 107, 136, 123, 123, 122, 107, 200, 220, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 26, 38, 46, 54, 19, 39, 39, 39, 13, 39, 39, 39, },
  { 26, 38, 46, 54, 19, 39, 39, 39, 28, 39, 39, 39, },
  { 18, 45, 53, 46, 11, 54, 61, 39, 26, 39, 39, 39, },
  { 9, 13, 12, DWS, DWS, 4, 4, 0, 1, 0, 0, 0, },
  { 9, 13, 12, DWS, DWS, 4, 4, 0, 4, 0, 0, 0, },
  { 9, 13, 12, 9, DWS, DWS, DWS, 4, 0, 0, 0, 0, },
  { 11, 4, 25, 4, 25, 25, 25, 25, 25, 25, 25, 25, },
  { 4, 4, 25, 4, DWE, 25, 25, 25, 25, 25, 25, 25, },
  { 11, 11, DWE, 25, DWE, 25, 25, 25, 11, 25, 25, 25, },
  { 165, 234, 217, 252, 104, 236, 252, 216, 115, 253, 254, 140, },
  { 117, 234, 117, 83, 117, 146, 130, 99, 164, 116, 116, 116, },
  { 165, 234, 217, 252, 104, 236, 252, 216, 115, 253, 254, 140, },
  { 117, 234, 117, 83, 117, 146, 130, 99, 164, 116, 116, 116, },
  { 165, 234, 217, 252, 104, 236, 252, 216, 115, 253, 254, 140, },
  { 117, 234, 117, 83, 117, 146, 130, 99, 164, 116, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 26, 46, 38, 39, 44, 39, 39, 39, },
  { 26, 46, 38, 39, 44, 39, 39, 39, },
  { 33, 45, 38, 31, 4, 39, 39, 39, },
  { DWS, 12, 12, DWS, 0, 0, 0, 0, },
  { DWS, 12, 12, 4, 0, 0, 0, 0, },
  { DWS, 12, DWS, 7, 1, 0, 0, 0, },
  { DWE, 11, DWE, 25, 25, 25, 25, 25, },
  { DWE, 11, DWE, 25, 25, 25, 25, 25, },
  { 25, 25, 25, DWE, 25, 25, 25, 25, },
  { 164, 235, 235, 236, 115, 185, 253, 118, },
  { 117, 91, 97, 82, 212, 116, 116, 116, },
  { 164, 235, 235, 236, 115, 185, 253, 118, },
  { 117, 91, 97, 82, 212, 116, 116, 116, },
  { 164, 235, 235, 236, 115, 185, 253, 118, },
  { 117, 91, 97, 82, 212, 116, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 18, 39, 39, 39, 19, 39, 39, 39, 0, 39, 39, 39, },
  { 18, 39, 39, 39, 19, 39, 39, 39, 0, 39, 39, 39, },
  { 26, 54, 39, 39, 34, 39, 39, 39, 0, 39, 39, 39, },
  { DWS, DWS, 12, 12, DWS, 0, 0, 0, 0, 0, 0, 0, },
  { DWS, DWS, 12, 12, DWS, 0, 0, 4, 0, 0, 0, 0, },
  { DWS, 4, 4, DWS, DWS, 4, 0, 4, 0, 0, 0, 0, },
  { DWE, 25, 25, 25, 25, 25, 25, 25, DWE, 25, 25, 25, },
  { 11, 25, 25, 25, DWE, 25, 25, 25, 25, 25, 25, 25, },
  { DWE, 4, 25, DWE, DWE, 25, 25, 25, 25, 25, 25, 25, },
  { 148, 252, 235, 235, 115, 219, 187, 189, 116, 237, 252, 238, },
  { 118, 114, 82, 73, 132, 99, 116, 99, 182, 116, 116, 116, },
  { 148, 252, 235, 235, 115, 219, 187, 189, 116, 237, 252, 238, },
  { 118, 114, 82, 73, 132, 99, 116, 99, 182, 116, 116, 116, },
  { 148, 252, 235, 235, 115, 219, 187, 189, 116, 237, 252, 238, },
  { 118, 114, 82, 73, 132, 99, 116, 99, 182, 116, 116, 116, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 11, 39, 39, 39, 18, 39, 39, 39, },
  { 11, 39, 39, 39, 18, 39, 39, 39, },
  { 18, 38, 62, 39, 18, 39, 39, 39, },
  { DWS, DWS, DWS, DWS, 0, 0, 0, 0, },
  { DWS, DWS, DWS, 0, 0, 0, 0, 0, },
  { 6, DWS, 4, 0, 0, 0, 0, 0, },
  { DWE, 25, 25, 25, 25, 25, 25, 25, },
  { DWE, 25, 25, DWE, 25, 25, 25, 25, },
  { 25, DWE, 11, 4, 25, 25, 25, 25, },
  { 211, 252, 199, 252, 116, 252, 142, 252, },
  { 117, 146, 86, 82, 151, 116, 116, 116, },
  { 211, 252, 199, 252, 116, 252, 142, 252, },
  { 117, 146, 86, 82, 151, 116, 116, 116, },
  { 211, 252, 199, 252, 116, 252, 142, 252, },
  { 117, 146, 86, 82, 151, 116, 116, 116, },
  }),
};
#endif

const CtxSet ContextSetCfg::ParFlagCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 1052 to 1072
 { 33, 25, 18, 26, 34, 27, 25, 26, 19, 42, CNU, 33, 19, 27, CNU, CNU, 34, 42, 20, 43, 20, },
 { 25, 25, 18, 26, 34, 27, 25, 26, 19, 42, CNU, 33, 19, 27, CNU, CNU, 34, 42, 20, 43, 20, },
 { 18, 40, 33, 26, 26, 42, 25, 33, 34, 34, 27, 25, 34, 42, 42, CNU, 18, 27, CNU, 42, CNU, },
 { DWS, 10, 10, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, },
 { 9, 10, 10, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, },
 { DWS, 12, 12, 10, 7, DWS, 12, 12, 13, 10, 13, 12, 12, 13, 12, 13, 9, 12, 13, 12, 13, },
 { 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, },
 { 11, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 11, 4, },
 { 4, 11, DWE, DWE, 4, 4, 11, 4, 4, 4, 4, 4, 11, 4, 4, 4, 4, DWE, 11, 4, 4, },
 { 116, 91, 90, 103, 151, 120, 88, 203, 100, 233, 106, 235, 147, DWO, 107, 135, 118, 105, 101, 235, 234, },
 { 133, 115, 117, 164, 116, 164, 118, 180, 169, 106, 185, 100, 136, 134, 165, 135, 243, 216, 201, 227, 105, },
 { 116, 91, 90, 103, 151, 120, 88, 203, 100, 233, 106, 235, 147, DWO, 107, 135, 118, 105, 101, 235, 234, },
 { 133, 115, 117, 164, 116, 164, 118, 180, 169, 106, 185, 100, 136, 134, 165, 135, 243, 216, 201, 227, 105, },
 { 116, 91, 90, 103, 151, 120, 88, 203, 100, 233, 106, 235, 147, DWO, 107, 135, 118, 105, 101, 235, 234, },
 { 133, 115, 117, 164, 116, 164, 118, 180, 169, 106, 185, 100, 136, 134, 165, 135, 243, 216, 201, 227, 105, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1073 to 1083
 { 33, 25, 26, 19, 19, 27, 26, 42, CNU, CNU, CNU, },
 { 25, 25, 18, 26, 11, 19, 26, 42, 20, 27, 43, },
 { 41, 25, 18, 34, 34, 19, 33, 42, 51, 58, 51, },
 { 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, },
 { 13, 12, 12, 9, 12, 13, 13, 13, 9, 13, 10, },
 { DWS, 12, 13, 12, 13, 13, 4, DWS, 13, 13, 13, },
 { 25, 11, DWE, 25, DWE, 4, DWE, 25, 11, 4, 4, },
 { 25, 4, 25, 25, 25, 25, 25, 25, 25, DWE, 4, },
 { 25, 4, 11, 25, DWE, 4, 11, 4, 11, 11, 4, },
 { 88, 235, 131, 99, 162, 121, 88, 106, 99, 164, 115, },
 { 196, 101, 211, 243, 196, 106, 227, 227, 211, 102, 151, },
 { 88, 235, 131, 99, 162, 121, 88, 106, 99, 164, 115, },
 { 196, 101, 211, 243, 196, 106, 227, 227, 211, 102, 151, },
 { 88, 235, 131, 99, 162, 121, 88, 106, 99, 164, 115, },
 { 196, 101, 211, 243, 196, 106, 227, 227, 211, 102, 151, },
  }),
};


#if JVET_AG0100_TRANSFORM_COEFFICIENT_CODING
const CtxSet ContextSetCfg::GtxFlagCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
// ctx 1084 to 1112
 { 34, 10, 26, 34, 19, 27, 20, 13, 2, 26, 19, 27, 20, 28, 29, 3, 27, 20, 28, 21, 29, 22, 19, 13, 21, 44, 22, 22, 38, },
 { 34, 25, 41, 26, 34, 27, CNU, 28, 2, 18, 26, 19, 20, 28, 21, 10, 34, 27, 20, 21, 21, 14, 26, 20, 13, 36, 22, 22, 38, },
 { 27, 33, 41, 34, 42, 27, CNU, 5, 10, 18, 11, 19, 20, 20, 21, 18, 11, 27, 20, 28, 21, 14, 11, 27, 20, 28, 29, 37, 30, },
 { 7, 9, 7, 10, 10, 7, 10, 9, 9, 12, 9, 9, 7, 7, 10, 12, 9, 9, DWS, 9, 12, 9, 5, 9, DWS, DWS, 10, 10, 12, },
 { 4, DWS, 6, 9, 9, 6, 6, 9, DWS, 13, DWS, 9, 6, 6, 6, 13, 9, 9, 10, 10, 10, 9, 6, 9, 9, DWS, 10, 10, 10, },
 { 3, DWS, 7, 7, 3, 7, DWS, DWS, DWS, 12, DWS, 12, 9, 7, 9, 10, 9, 9, 9, DWS, 10, DWS, 5, DWS, 5, 5, 7, 10, DWS, },
 { DWE, DWE, 4, 4, 11, 4, 4, DWE, 11, DWE, 11, 4, 4, 4, 11, 25, 11, 11, 11, 11, 11, 4, 4, DWE, 11, 11, 11, 11, 4, },
 { 11, DWE, 4, 4, 4, 4, 4, DWE, 4, DWE, 4, 4, 4, 4, 4, 25, DWE, 11, DWE, DWE, 11, 11, 11, DWE, 11, 4, 11, 11, 4, },
 { 11, 25, 11, 4, 4, 4, 4, 25, 11, DWE, 4, 4, 4, 4, 11, DWE, DWE, DWE, DWE, 11, 11, 4, 4, DWE, 4, 4, 11, 11, 4, },
 { 122, 103, 130, 122, 122, 233, 118, 147, 251, 235, 251, 251, 217, 168, 117, 183, 251, 251, 252, 233, 134, 166, 220, 251, 236, 252, 185, 134, 152, },
 { 129, 226, 194, 99, 193, 115, 116, 130, 101, 90, 102, 89, 106, 106, 85, 100, 115, 100, 100, 242, 85, 114, 227, 99, 88, 85, 90, 85, 67, },
 { 99, 103, 130, 116, 116, 133, 132, 131, 116, 123, 182, 213, 132, 147, 118, 107, 123, 183, 124, 123, 118, 118, 124, 140, 169, 248, 133, 121, 118, },
 { 125, 105, 118, 102, 101, 116, 117, 130, 105, 99, 105, 100, 116, 118, 88, 130, 147, 102, 131, 131, 115, 115, 116, 101, 101, 100, 107, 101, 131, },
 { 122, 251, 234, 178, 133, 249, 90, 195, 115, 216, 106, 196, 104, 150, DWO, 193, 133, 218, 147, 235, 100, 213, 140, 235, 252, 156, 154, 226, 218, },
 { 146, 146, 145, 100, 122, 116, 116, 162, 148, 100, 121, 91, 116, 106, 84, 103, 116, 100, 103, 131, 210, 115, 106, 100, 101, 100, 99, 162, 83, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1113 to 1127
 { 34, 10, 11, 4, 12, 5, 5, 6, 18, 19, 20, 21, 29, 45, 46, },
 { 41, 25, 18, 19, 4, 5, 5, 21, 18, 19, 20, 21, 37, 45, 38, },
 { 12, 18, 11, 27, 20, 44, 5, 37, 11, CNU, 20, 60, 37, 30, 23, },
 { 7, 12, DWS, 9, 4, 6, 5, 6, 9, 9, 10, 5, DWS, 10, 12, },
 { 4, 9, DWS, 9, DWS, 5, 5, 5, DWS, 6, 10, 5, 9, 12, 12, },
 { 4, 3, DWS, 13, 10, 12, 4, 2, 5, 3, 0, 5, 3, 0, 5, },
 { 25, 25, 25, 25, 4, DWE, 4, 11, 11, 4, 25, 4, DWE, 4, DWE, },
 { DWE, 25, 4, DWE, DWE, DWE, 11, 4, 11, 4, 25, 11, 11, DWE, 4, },
 { DWE, 4, 4, 25, DWE, 11, 11, 4, DWE, 11, 4, 4, 25, 11, 25, },
 { 150, 218, 154, 227, 182, 115, 105, 118, 84, 88, 128, 100, 101, 148, 101, },
 { 100, 129, 131, 102, 102, 118, DWO, 115, 105, 104, 217, 219, 140, 247, 117, },
 { 99, 88, 197, 147, 88, 108, 106, 123, 242, 130, 107, 102, 131, 195, 147, },
 { 117, 146, 102, 100, 164, 120, 132, 115, 108, 121, 243, 236, 149, 231, 186, },
 { 99, 216, 180, 152, 82, 138, 101, 244, 131, 135, DWO, 99, 233, 148, 133, },
 { 173, 147, DWO, 105, 105, 187, 101, 115, 99, 129, 116, 102, 113, 116, 164, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1128 to 1156
 { 25, 25, 19, 27, 20, 36, 21, 29, 33, 12, 28, 21, 29, 37, 22, 34, 28, 29, 29, 22, 22, 30, 28, 29, 45, 30, 23, 38, 23, },
 { 25, 25, 19, 27, 20, 36, 21, 29, 33, 12, 28, 21, 37, 37, 22, 34, 28, 29, 29, 22, 22, 30, 28, 29, 45, 30, 23, 38, 23, },
 { 9, 17, 26, 27, 43, 36, 36, 29, 25, 27, 20, 36, 29, 29, 22, 33, CNU, 36, 21, 37, 22, 30, 27, 36, 37, 45, 38, 38, 46, },
 { 9, 9, 7, 10, 10, 10, 13, 9, 13, 10, 13, 13, 13, 13, 13, 9, 9, 10, 10, 13, 13, 13, 10, 10, 10, 10, 12, 13, 12, },
 { 9, 9, 6, 10, 10, 10, 10, 9, 13, 10, 13, 13, 13, 13, 13, 9, 10, 9, 10, 13, 13, 13, 10, 10, 10, 10, 12, 13, 13, },
 { 12, DWS, DWS, 10, 9, 7, 9, 9, 9, 9, 13, 13, 13, 13, 13, 9, 9, 9, 9, 12, 12, 9, 6, 9, 9, 10, 10, 12, 10, },
 { DWE, 11, 4, 4, 4, 4, 4, 11, 11, 4, 11, 4, 11, 4, 4, 4, 4, 4, 4, 11, 4, 11, 11, DWE, 11, 4, 4, DWE, 4, },
 { DWE, DWE, 4, 4, 4, 4, 4, 11, DWE, 11, 11, 11, DWE, 11, 11, 11, 11, 4, 4, 11, 11, 11, DWE, DWE, 11, 11, 4, 25, 11, },
 { DWE, 4, 4, 11, 4, 4, 4, DWE, 11, 4, 11, 4, DWE, 4, 11, 11, 11, 4, 4, DWE, 11, 4, 11, 11, 11, 11, 4, DWE, 11, },
 { 82, 99, 116, 165, 136, 116, 163, 244, 84, 134, 197, 105, 211, 163, 133, 107, 150, 147, 234, 117, 213, 167, 162, 137, 135, 243, 116, 211, 219, },
 { 133, 131, 117, 99, 101, 106, 102, 131, 106, 116, 117, 149, 185, 233, 114, 118, 103, 135, 105, 101, 100, 69, 178, 99, 99, 243, 219, 226, 66, },
 { 82, 99, 106, 131, 117, 117, 116, 148, 90, 131, 179, 131, 102, 147, 117, 103, 117, 117, 117, 227, 132, 120, 115, 132, 117, 116, 101, 227, 139, },
 { 132, 117, 117, 146, 106, 131, 122, 147, 116, 121, 118, 201, 250, 134, 226, 118, 116, 135, 134, 106, 105, 85, 131, 105, DWO, 153, 234, 90, 82, },
 { 99, 99, 103, 209, 105, 116, 131, 184, 91, 147, 115, 115, 100, 102, 104, 115, 243, 147, 123, 132, 116, 138, 130, 133, 243, 117, 242, 147, 195, },
 { 131, 107, 102, 147, 104, 162, 154, 115, 117, 122, 169, 200, 202, 185, 118, 118, 116, 170, 103, 107, 164, 88, 118, 100, 132, 117, 250, 187, 84, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1157 to 1171
 { 40, 18, 20, 5, 13, 6, 6, 22, 43, 29, 45, 38, 38, 46, 46, },
 { 40, 33, CNU, 28, 13, 29, 29, 14, 43, 37, 45, 38, 46, 46, 46, },
 { 17, 33, 12, 28, 36, 13, 21, 6, 50, 29, 53, 61, 54, 46, 54, },
 { 12, 12, 12, 12, 12, DWS, 12, 13, DWS, DWS, DWS, 12, 13, 13, 13, },
 { 12, 12, 9, 12, 12, DWS, DWS, 12, DWS, DWS, DWS, 13, 12, 12, 10, },
 { 12, 9, DWS, 12, 13, 3, 10, 12, 4, 2, 4, 6, 13, 13, DWS, },
 { 25, 4, DWE, 25, 25, DWE, 25, DWE, DWE, DWE, DWE, 25, 25, 25, 25, },
 { DWE, 11, 11, 25, 25, 11, 4, DWE, 4, 11, 11, 25, DWE, DWE, DWE, },
 { 25, 4, 11, 25, 11, DWE, 25, DWE, 11, 4, 4, 25, 25, 4, 25, },
 { 87, 90, 88, 146, 162, 117, 103, 163, 100, 242, 147, 90, 178, 103, 150, },
 { 131, 91, 116, 179, 148, 155, 235, 114, 124, 252, 166, 233, 232, 218, 66, },
 { 69, 85, 115, 100, 130, 118, 107, 117, 116, 115, 180, 99, 101, 105, 217, },
 { 117, 131, 117, 122, 212, 149, 105, 106, 118, 252, DWO, 250, 188, 200, 65, },
 { 66, 195, 99, 210, 163, 123, 240, 235, 132, 115, 132, 113, 234, 233, 250, },
 { DWO, 100, 138, 226, 129, 151, 241, 217, 118, 136, 109, 101, 84, 80, 82, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1172 to 1200
 { 18, 25, 26, 34, 27, 20, 28, 29, 33, 19, 12, 28, 21, 29, 14, 26, 20, 21, 29, 22, 22, 30, 20, 29, 45, 30, 38, 38, 23, },
 { 18, 25, 26, 34, 27, 20, 28, 21, 33, 19, 12, 28, 21, 29, 14, 26, 20, 21, 29, 22, 22, 30, 20, 29, 45, 30, 38, 38, 23, },
 { 26, 25, 18, 34, 19, 20, 20, 21, 25, 26, 27, CNU, 13, 21, 37, 33, 19, 43, 36, 29, 29, 45, 11, 20, 29, 37, 22, 45, 38, },
 { 9, 12, 7, 10, 10, 10, 10, 10, 13, 10, 13, 12, 9, 10, 9, 10, 10, 10, 10, 12, 13, 13, 9, 10, 9, 10, 13, 12, 13, },
 { 9, 9, 6, 10, 10, 10, 10, 9, 13, 10, 12, 12, 9, 9, 9, 10, 10, 9, 9, 12, 12, 13, 9, 10, 9, 10, 13, 12, 13, },
 { DWS, DWS, 5, 13, 10, 9, 10, DWS, 13, 9, 9, 13, 9, 9, 9, 9, 9, DWS, 9, 12, 10, DWS, 5, 10, DWS, 9, 12, 13, 9, },
 { 25, 4, 4, 4, 11, 4, 4, DWE, DWE, 4, 11, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 11, 4, DWE, 4, 4, DWE, 4, 11, },
 { DWE, 4, 4, 4, 4, 11, 4, DWE, DWE, 4, 11, 4, 4, 4, 4, 4, 11, 4, 4, 11, 4, 11, 11, DWE, 4, 4, 25, DWE, 11, },
 { DWE, 11, 11, DWE, 11, DWE, 4, 25, DWE, 4, 4, 11, 4, 4, 4, 4, 4, 11, 11, 11, 4, 4, 4, DWE, 4, 4, 25, 25, 11, },
 { 100, 163, 106, 116, 105, 162, 103, 130, 74, 100, 195, 101, 102, 178, 118, 102, 211, 116, 116, 100, 131, 136, 85, 116, 105, 163, 131, 164, 215, },
 { 131, 131, 122, 146, 100, 105, 106, 193, 118, 154, 103, 120, 251, 250, 101, 117, 106, 121, 137, 219, 234, 83, 120, 118, 167, 165, 233, 233, 68, },
 { 99, 85, 101, 115, 103, 115, 242, 116, 83, 100, 104, 116, 107, 103, 117, 100, 116, 107, 105, 116, 102, 118, 91, 146, 117, 131, 131, 228, 180, },
 { 163, 116, 118, 146, 103, 162, 131, 116, 117, 134, 117, 120, 165, 251, 107, 132, 117, 124, 155, 122, 133, 100, DWO, 116, 136, 217, 107, 91, 83, },
 { 147, 85, 102, 114, 99, 87, 90, 116, 86, 88, 131, 147, 100, 89, 106, 101, 107, 102, 100, 131, 147, 137, 108, 115, 116, 102, 131, 234, 245, },
 { 101, 180, 148, 101, 210, 180, 200, 102, 210, 136, 121, 147, 235, 203, 107, 132, 116, 121, 135, 183, 183, 85, 121, DWO, 204, 251, 153, 86, 82, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1201 to 1215
 { 33, 18, 4, 5, 13, 21, 21, 14, 11, 13, 29, 30, 30, 30, 46, },
 { 33, 33, 19, 28, 21, 29, 29, 14, 19, 21, 37, 30, 38, 38, 46, },
 { 49, 33, 19, 27, 52, 36, 5, 5, 34, 28, 37, 37, 53, 38, 61, },
 { DWS, 12, DWS, DWS, DWS, 4, DWS, 6, 12, DWS, DWS, 7, 9, 12, 12, },
 { 6, 12, DWS, DWS, DWS, 4, DWS, 5, 12, DWS, 4, 7, 12, 12, 13, },
 { 12, 12, 6, 10, 2, 0, 4, 10, 9, 1, 0, 0, 0, 3, 0, },
 { 25, 25, DWE, 25, 25, 4, DWE, 11, 25, 25, 11, 11, DWE, 11, DWE, },
 { 11, DWE, 11, 25, 25, 4, DWE, 4, DWE, DWE, 4, 4, 25, 11, 11, },
 { 25, 11, DWE, 4, 4, 4, 4, 25, 25, 4, 4, DWE, 25, 25, 25, },
 { 121, 85, 101, 146, 242, 164, 116, DWO, 82, 98, 115, 99, 103, 148, 200, },
 { 242, 147, 135, 135, 199, 125, 135, 116, 202, 252, 188, 233, 201, 181, 114, },
 { 107, 83, 101, 100, 130, 104, 132, 132, 84, 100, 91, 109, 102, 148, 122, },
 { 147, 195, 181, 138, 185, 166, 125, 124, 136, 252, 235, 248, 235, 122, 98, },
 { 86, 85, 99, 115, 114, 245, 116, 167, 89, 243, 118, 116, 140, 234, 155, },
 { 194, 179, 136, 179, 196, 120, 101, 112, 163, 198, 217, 118, 116, 209, 164, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1216 to 1244
 { 59, 34, 34, 27, 27, CNU, CNU, 13, 11, 19, 19, 27, 28, 28, 29, 27, 12, 28, 28, 21, 29, 22, 20, 36, 29, 44, 30, 30, 38, },
 { 59, 42, 42, 27, 27, CNU, CNU, 36, 3, 19, 19, 27, 28, 28, 29, 27, 12, 20, 28, 29, 29, 22, CNU, 36, 29, 44, 30, 30, 38, },
 { 29, 34, 42, 27, 19, 27, 43, 28, 19, 19, 19, 27, 12, 20, 21, 27, 27, CNU, 20, 36, 36, 22, 12, 12, 28, 13, 29, 29, 30, },
 { 1, DWS, 12, 7, 10, 13, 13, 10, 10, 13, 9, 13, 13, 13, 10, 10, 13, 13, 9, 13, 13, 10, 3, 13, 13, 13, 12, 12, 12, },
 { 3, 7, 10, 9, 10, 10, 10, 10, 10, 12, 9, 13, 13, 13, 10, 10, 10, 10, 10, 13, 10, 10, 3, 10, 10, 13, 12, 12, 13, },
 { 2, DWS, 10, 10, 6, 6, 12, DWS, 2, 9, 9, 10, 13, 13, 9, 10, 9, 9, 9, 13, 10, 10, 3, 10, 9, 13, 13, 13, 13, },
 { 4, 25, DWE, 4, 4, 4, 4, 25, 25, 11, 4, 11, 11, 11, DWE, DWE, 4, 4, 4, DWE, 4, 4, 4, 11, 4, DWE, 4, 4, 4, },
 { DWE, 25, DWE, 11, 4, 11, 11, 25, 25, 11, 4, 11, DWE, DWE, DWE, 25, 11, 11, 11, DWE, 4, 4, 11, 11, 4, DWE, 4, 4, 11, },
 { 4, 25, 25, DWE, 11, 25, 11, 25, 4, DWE, 4, 4, 25, 25, 11, DWE, 11, 11, 4, 25, 11, 4, 4, 4, 4, DWE, 11, 11, 4, },
 { 228, 131, 102, 102, 102, 106, 234, 201, 97, 98, 234, 234, 136, 218, 152, 98, 217, 217, 251, 234, 200, 234, 121, 218, 134, 233, 121, 132, 203, },
 { 220, 147, 210, 179, 121, 148, 164, 242, 162, 180, 107, 105, 218, 234, 133, 165, 165, 152, 136, 233, 234, 115, 132, 217, 217, 150, 235, 219, 85, },
 { 106, 107, 83, 89, 114, 185, 250, 154, 88, 210, 251, 219, 148, 202, 135, 90, 107, 249, 183, 133, 166, 134, 123, 140, 122, 164, 118, 117, 155, },
 { 138, 195, 179, 171, 201, 167, 136, 195, 133, 139, 118, 227, 234, 234, 133, 148, 243, 132, 229, 249, 234, 227, 124, 132, 245, DWO, 235, 235, 101, },
 { 122, 211, 192, 83, 114, 251, 98, 252, 133, 102, 138, 234, 152, 217, 186, 84, 133, 218, 135, 215, 250, 227, DWO, 116, DWO, 215, 185, 182, 218, },
 { 134, 104, 233, 233, 235, 251, 102, 212, 120, 133, 197, 170, 186, 234, 247, 249, 195, 132, 168, 199, 218, 250, 135, 164, 136, 154, 234, 234, 101, },
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1245 to 1259
 { 5, 11, 4, 20, 28, 13, 13, 6, 27, 12, 20, 28, 21, 21, 30, },
 { 13, 11, 27, 43, 20, 13, 13, 29, 43, 20, 28, 44, 37, 37, 30, },
 { 4, 11, 28, CNU, 51, 5, 12, 13, 60, 13, 36, 5, 21, 37, 30, },
 { 7, 4, 4, DWS, 12, 9, 12, 10, 5, DWS, 13, 12, 13, 13, 13, },
 { 2, 12, 5, 5, 13, 12, 12, 10, 1, 5, 3, 10, 13, 13, 13, },
 { 4, 5, 13, 4, 0, 13, 13, 13, 7, DWS, DWS, 7, 7, 13, DWS, },
 { DWE, 4, 4, DWE, 25, 25, 25, 25, DWE, 11, 11, 25, 25, 25, 4, },
 { 11, 25, 11, 11, 25, 25, 25, DWE, 11, 11, 4, 11, 25, 4, 11, },
 { 25, DWE, 25, 4, 4, 4, 11, 4, 25, 25, 11, 25, 25, 4, 25, },
 { 122, 201, 165, 118, 227, 244, 194, 168, 99, 83, 84, 87, 162, 166, 218, },
 { 161, 196, 136, 250, 232, 216, 218, 148, 155, 252, 234, 235, 234, 233, 231, },
 { 146, 75, 74, 116, 101, 131, 147, 218, 99, 242, 89, 99, 230, 217, 234, },
 { 120, 117, 214, 197, 246, 234, 232, 115, 148, 138, 167, 249, 234, 200, 106, },
 { 138, 210, 80, 196, 117, 201, 167, 201, 176, 87, 100, 118, 250, 234, 251, },
 { 163, 211, 122, 106, 117, 166, 232, 162, 249, 252, 169, 208, 230, 233, 179, },
  }),
};
#else
const CtxSet ContextSetCfg::GtxFlagCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
  { 25, 1, 40, 25, 33, 11, 9, 25, 25, 18, 12, 17, 33, 26, 19, 13, 18, 19, 20, 28, 22, },
  { 17, 0, 9, 25, 25, 26, 1, 25, 25, 18, 12, 17, 33, 26, 19, 13, 33, 19, 20, 28, 22, },
  { 25, 0, 40, 17, 33, 26, 0, 17, 25, 33, 19, 9, 25, 18, 34, 20, 25, 18, 11, 12, 29, },
  { 4, 13, 10, 10, 10, 5, 12, 9, 9, 9, 5, 13, 9, 9, 9, 9, 6, 9, 9, 9, 9, },
  { 4, 13, 10, 10, 10, 5, 12, 10, 10, 10, 6, 13, 10, 9, 10, 9, 6, DWS, DWS, 9, 10, },
  { 2, DWS, 5, 5, 9, 2, 9, DWS, 9, 12, 5, DWS, 12, DWS, 4, 5, 6, 9, 5, 10, 5, },
  { 11, 25, DWE, 11, 11, DWE, DWE, 11, 4, 4, 11, 11, 11, 11, 11, 11, 4, DWE, 11, 11, 4, },
  { 25, 25, 11, 11, 4, DWE, DWE, 11, 4, 4, 11, 11, 11, 11, DWE, 11, 4, 11, 11, 11, 4, },
  { 4, 4, 4, DWE, 25, DWE, 4, 4, 4, 11, 11, 4, 25, DWE, 4, 11, 11, 25, 11, 25, 4, },
  { 99, 71, 83, 84, 91, 163, 73, 83, 135, 121, 108, 85, 100, 148, 148, 116, 101, 107, 121, 118, 137, },
  { 125, 102, 118, 116, 115, 117, 164, 134, 117, 147, 117, 108, 116, 117, 118, 101, 163, 116, 116, 115, 89, },
  { 99, 71, 83, 84, 91, 163, 73, 83, 135, 121, 108, 85, 100, 148, 148, 116, 101, 107, 121, 118, 137, },
  { 125, 102, 118, 116, 115, 117, 164, 134, 117, 147, 117, 108, 116, 117, 118, 101, 163, 116, 116, 115, 89, },
  { 99, 71, 83, 84, 91, 163, 73, 83, 135, 121, 108, 85, 100, 148, 148, 116, 101, 107, 121, 118, 137, },
  { 125, 102, 118, 116, 115, 117, 164, 134, 117, 147, 117, 108, 116, 117, 118, 101, 163, 116, 116, 115, 89, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 40, 9, 25, 3, 3, 12, 25, 18, 19, 28, 37, },
  { 17, 1, 17, 2, 3, 4, 25, 18, 11, 12, 14, },
  { 41, 9, 25, 33, 42, 27, 2, 41, 50, 36, 45, },
  { DWS, 12, DWS, DWS, DWS, 4, 4, DWS, 10, 4, 10, },
  { 4, DWS, 12, DWS, 4, 4, 4, 4, 4, 4, DWS, },
  { 4, DWS, 9, 10, 4, 3, 0, 1, 0, 1, 3, },
  { 25, 25, DWE, 25, 25, 11, 4, 25, 25, 4, 4, },
  { 25, 25, 25, 25, 25, 25, 11, DWE, 25, 11, 4, },
  { DWE, 25, 25, 4, 4, 4, 4, 25, 25, 25, 25, },
  { 99, 82, 92, 177, 115, 117, 99, 83, 160, 228, 179, },
  { 242, 195, 212, 117, 118, 103, 149, 123, 180, 123, 122, },
  { 99, 82, 92, 177, 115, 117, 99, 83, 160, 228, 179, },
  { 242, 195, 212, 117, 118, 103, 149, 123, 180, 123, 122, },
  { 99, 82, 92, 177, 115, 117, 99, 83, 160, 228, 179, },
  { 242, 195, 212, 117, 118, 103, 149, 123, 180, 123, 122, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 25, 25, 19, 27, 20, 29, 33, 12, 28, 21, 22, 34, 28, 29, 29, 30, 28, 29, 45, 30, 23, },
  { 25, 25, 19, 27, 20, 21, 33, 12, 28, 21, 22, 34, 28, 29, 29, 30, 28, 37, 45, 30, 23, },
  { 9, 17, 26, 27, CNU, 21, 25, 19, CNU, 36, 37, 33, CNU, 28, 29, 30, 19, 36, 37, 45, 38, },
  { 9, 9, 7, 10, 10, 10, 13, 10, 13, 13, 13, 9, 9, 10, 10, 13, 10, DWS, 9, 10, 13, },
  { 9, 6, 7, 10, 10, 10, 13, 10, 13, 13, 13, 9, 10, 10, 13, 13, 9, 9, 10, 9, 13, },
  { 12, DWS, DWS, 12, 10, DWS, 9, 9, 13, 13, 10, 9, 10, 9, DWS, DWS, 5, DWS, 10, DWS, 9, },
  { DWE, 11, 4, 4, 4, DWE, 11, 4, 4, 4, DWE, 4, 4, 4, 4, 11, DWE, 11, 4, 11, 11, },
  { DWE, 4, 4, 4, 4, DWE, 4, 4, 4, 4, 11, 4, 4, 4, 4, 11, 11, 11, 11, 4, 11, },
  { DWE, DWE, 11, 25, 25, DWE, 11, 11, DWE, 4, 4, DWE, DWE, 11, 4, 4, 11, 4, DWE, 4, 11, },
  { 84, 83, 116, 147, 121, 118, 85, 163, 213, 117, 105, 104, 122, 121, 228, 132, 195, 181, 121, 181, 135, },
  { 179, 117, 116, 100, 101, 116, 117, DWO, 105, 185, 118, 120, 116, 131, 123, 120, 117, 101, 162, 102, 74, },
  { 84, 83, 116, 147, 121, 118, 85, 163, 213, 117, 105, 104, 122, 121, 228, 132, 195, 181, 121, 181, 135, },
  { 179, 117, 116, 100, 101, 116, 117, DWO, 105, 185, 118, 120, 116, 131, 123, 120, 117, 101, 162, 102, 74, },
  { 84, 83, 116, 147, 121, 118, 85, 163, 213, 117, 105, 104, 122, 121, 228, 132, 195, 181, 121, 181, 135, },
  { 179, 117, 116, 100, 101, 116, 117, DWO, 105, 185, 118, 120, 116, 131, 123, 120, 117, 101, 162, 102, 74, },
  }),
  ContextSetCfg::addCtxSet
  ({
  { 40, 18, 20, 5, 13, 14, 43, 29, 45, 38, 46, },
  { 40, 18, 12, 5, 5, 6, 20, 14, 37, 30, 46, },
  { 17, 33, CNU, 28, 13, 21, 42, 37, 53, 61, 46, },
  { 12, 13, 9, 12, 12, DWS, DWS, DWS, DWS, 13, 13, },
  { 12, 13, 12, 12, 12, DWS, DWS, DWS, DWS, 10, 12, },
  { DWS, 10, 10, DWS, 7, 12, 4, 3, 0, 0, 4, },
  { 25, DWE, 11, 25, 25, 4, DWE, DWE, 11, 25, DWE, },
  { 25, 11, DWE, 25, 25, 25, 11, 25, 25, 25, DWE, },
  { DWE, 25, DWE, 4, 4, 4, 25, 11, 11, 4, 25, },
  { 87, 86, 100, 115, 194, 228, 116, 226, 101, 90, 165, },
  { 116, 115, 148, 121, 228, 133, DWO, 219, 148, 213, 99, },
  { 87, 86, 100, 115, 194, 228, 116, 226, 101, 90, 165, },
  { 116, 115, 148, 121, 228, 133, DWO, 219, 148, 213, 99, },
  { 87, 86, 100, 115, 194, 228, 116, 226, 101, 90, 165, },
  { 116, 115, 148, 121, 228, 133, DWO, 219, 148, 213, 99, },
  }),
};
#endif

const CtxSet ContextSetCfg::TsSigCoeffGroupCtxSetSwitch = ContextSetCfg::addCtxSet
  ({
// ctx 1260 to 1262
 { 18, 28, 38, },
 { 18, 20, 45, },
 { 11, 20, 30, },
 { 5, 5, 5, },
 { 6, 9, 3, },
 { 6, 10, 9, },
 { 11, DWE, 11, },
 { DWE, DWE, 4, },
 { DWE, DWE, 11, },
 { 121, 156, 212, },
 { 148, 150, 100, },
 { 120, 140, 148, },
 { 132, 244, 85, },
 { 117, 154, 164, },
 { 147, 168, 84, },
  });


const CtxSet ContextSetCfg::TsSigFlagCtxSetSwitch = ContextSetCfg::addCtxSet
  ({
// ctx 1263 to 1265
 { 25, 28, 30, },
 { 25, 28, 38, },
 { 25, CNU, 37, },
 { 13, 13, 10, },
 { 13, 13, 6, },
 { 13, 9, 6, },
 { 4, DWE, DWE, },
 { 4, 11, 11, },
 { 4, 4, DWE, },
 { 71, 234, 99, },
 { 84, 234, 104, },
 { 74, 251, 107, },
 { 101, 251, 107, },
 { 74, 251, 107, },
 { 101, 251, 107, },
  });




const CtxSet ContextSetCfg::TsParFlagCtxSetSwitch = ContextSetCfg::addCtxSet
  ({
// ctx 1266
 { 11, },
 { 11, },
 { 19, },
 { DWS, },
 { 6, },
 { 9, },
 { 25, },
 { 4, },
 { 11, },
 { 116, },
 { 101, },
 { 108, },
 { 101, },
 { 131, },
 { 131, },
  });

const CtxSet ContextSetCfg::TsGtxFlagCtxSetSwitch = ContextSetCfg::addCtxSet
  ({
// ctx 1267 to 1271
 { CNU, 3, 4, 5, 5, },
 { CNU, 10, 11, 18, 3, },
 { CNU, 3, 3, 4, 4, },
 { DWS, 1, 1, 4, 4, },
 { DWS, 1, 3, 0, 1, },
 { DWS, 1, 1, 5, 4, },
 { DWE, 11, 11, DWE, DWE, },
 { DWE, 11, DWE, 11, 11, },
 { DWE, 4, 4, DWE, DWE, },
 { DWO, 164, 196, 243, 243, },
 { DWO, 116, 117, 102, 102, },
 { DWO, 132, 164, 163, 148, },
 { DWO, 117, 117, 107, 102, },
 { DWO, 148, 244, 242, 148, },
 { DWO, 117, 117, 101, 102, },
  });


const CtxSet ContextSetCfg::TsLrg1FlagCtxSetSwitch = ContextSetCfg::addCtxSet
  ({
// ctx 1272 to 1275
 { 12, 5, 5, 7, },
 { 11, 5, 5, 52, },
 { 4, 12, 5, 7, },
 { 4, 1, 1, 5, },
 { 5, 2, 1, 7, },
 { 1, 1, 1, 4, },
 { DWE, 11, 11, DWE, },
 { 11, DWE, 11, DWE, },
 { 4, 11, 11, 25, },
 { 132, 179, 211, 242, },
 { 163, 117, 116, 104, },
 { 117, 116, 146, 116, },
 { 117, 116, 116, 104, },
 { 118, 179, 227, 147, },
 { 117, 116, 116, 100, },
});

const CtxSet ContextSetCfg::TsResidualSignCtxSetSwitch = ContextSetCfg::addCtxSet
  ({
// ctx 1276 to 1281
 { 12, 17, 46, 27, 33, 38, },
 { 12, 17, 61, 12, 56, 15, },
 { 36, 10, 46, 26, 40, 53, },
 { 0, 1, 4, 5, 6, DWS, },
 { 1, 5, 4, 5, 6, 9, },
 { 1, 5, 5, 5, 4, 0, },
 { DWE, 4, 25, DWE, DWE, DWE, },
 { DWE, DWE, 11, 4, 4, 4, },
 { 4, DWE, DWE, 25, 25, 4, },
 { 116, 116, 117, 147, 118, 118, },
 { 116, 148, 100, 101, 120, 118, },
 { 117, 131, 117, 226, 152, 117, },
 { 117, 117, 104, 101, 146, 231, },
 { 117, 147, 116, 162, 243, 117, },
 { 117, 117, 104, 99, 102, 116, },
  });


const CtxSet ContextSetCfg::LastXCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TU_256
// ctx 1282 to 1317
 { 6, 5, 4, 14, 5, 19, 14, 6, 13, 19, 22, 6, 6, 20, 26, 22, 7, 22, 21, 19, 10, 7, 14, 21, 2, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 21, 5, 4, 14, 5, 11, 14, 13, 13, 18, 22, 6, 14, 12, 18, 22, 22, 22, 13, 4, 58, 7, 29, 43, 61, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 13, 6, 5, 6, 7, 5, 5, 7, 14, 4, 13, 14, 29, 21, 4, 21, 22, 14, 6, 7, 58, 11, 28, 39, 19, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 9, 5, 5, 6, 6, 4, 6, 6, 0, 0, 7, 1, 1, 1, 0, 2, 1, 1, 0, 1, 0, 1, 0, 1, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 9, 5, 5, 6, 7, 4, 6, 6, 1, 0, 6, 1, 1, 1, 0, 2, 3, 2, 1, 0, 0, 2, 10, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 5, 5, 4, 1, 5, 4, 1, 5, 1, 1, 5, 0, 1, 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 7, 13, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { DWE, 11, 11, 11, 25, 25, 11, 25, DWE, 25, DWE, DWE, 25, 25, 25, 11, DWE, 25, 25, 25, 25, 25, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 4, 4, 11, 25, DWE, 11, 25, DWE, DWE, DWE, 11, DWE, 25, 25, 11, DWE, 25, 25, 25, 25, DWE, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 11, 11, 11, DWE, DWE, 11, 25, DWE, 25, 25, 4, 25, 25, 25, 11, 11, 25, 25, 25, 25, 11, 25, 25, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 115, 122, 134, 133, 139, 117, 136, 123, 126, 117, 136, 139, 124, 229, 116, 137, 125, 118, 133, 253, 116, 151, 254, 253, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 130, 116, 108, 244, 244, 105, 243, 251, 197, 116, 163, 245, 243, 243, 116, 228, 245, 211, 148, 211, 116, 181, 212, 218, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 118, 121, 117, 117, 117, 124, 117, 125, 117, 131, 123, 118, 212, 116, 134, 123, 116, 147, 228, 116, 155, 254, 254, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 115, 117, 108, 123, 244, 104, 122, 250, 181, 116, 148, 246, 180, 229, 116, 134, 245, 243, 244, 252, 116, 244, 229, 180, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 211, DWO, 118, 117, 116, 133, 117, 125, 117, 131, DWO, 122, 139, 116, 228, 117, 116, 132, 165, 116, 150, 254, 233, 64, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 115, 147, 109, 125, 132, 109, 133, 123, 212, 115, 133, 229, 148, 243, 116, 136, 197, 244, 164, 132, 116, 244, 148, 232, 153, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
#else
  {  21,   5,   4,   6,   6,   4,   6,  14,  14,   4,  14,   7,  30,  14,   4,  22,  38,  15,  22,   6 },
  {  21,   5,   4,   6,   6,   4,   6,  14,  14,   4,  14,   7,  30,  14,   4,  22,  38,  15,  22,   6 },
  {  21,   6,  12,  14,   7,   4,  14,   7,   6,   4,  14,   7,  14,   6,   5,  14,   7,  14,  14,  22 },
  {   9,   5,   5,   6,   5,   4,   5,   5,   1,   1,   5,   1,   1,   0,   0,   1,   1,   1,   0,   0 },
  {   9,   5,   5,   6,   5,   4,   5,   5,   1,   1,   5,   1,   1,   0,   0,   1,   1,   1,   0,   0 },
  {   5,   5,   5,   6,   5,   1,   6,   5,   2,   1,   6,   1,   1,   1,   1,   2,   2,   1,   1,   1 },
  {  25,  18,  18,  18,  25,  25,  18,  25,  18,  32,  25,  18,  25,  18,  32,  18,  18,  25,  32,  32 },
  {  25,  18,  18,  18,  25,  25,  18,  25,  18,  32,  25,  18,  25,  18,  32,  18,  18,  25,  32,  32 },
  {  18,  18,  18,  18,  18,  11,  18,  18,  18,  18,  18,  11,  18,  25,  25,  11,  11,  18,  25,  32 },
  { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148 },
  { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228 },
  { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148 },
  { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228 },
  { 116, 117, 118, 125, 119, 118, 122, 118, 126, 132, 118, 121, 118, 228, 116, 133, 126, 118, 148, 148 },
  { 117, 118, 117, 132, 228, 103, 132, 228, 149, 117, 164, 149, 149, 229, 116, 126, 132, 227, 228, 228 },
#endif
}),
  ContextSetCfg::addCtxSet
  ({
// ctx 1318 to 1320
 { 12, 11, 33, },
 { CNU, 34, 33, },
 { 12, 5, 4, },
 { 5, 4, 4, },
 { 5, 4, 4, },
 { 4, 4, 4, },
 { 11, DWE, 11, },
 { 11, 11, 4, },
 { DWE, 25, 25, },
 { 227, 123, 117, },
 { 164, 117, 102, },
 { 116, DWO, 117, },
 { 132, 118, 102, },
 { 117, 124, DWO, },
 { 164, 117, 100, },
  }),
};

const CtxSet ContextSetCfg::LastYCtxSetSwitch[] =
{
  ContextSetCfg::addCtxSet
  ({
#if TU_256
// ctx 1321 to 1356
 { 21, 13, 12, 14, 6, 19, 22, 14, 12, 11, 22, 7, 13, 5, 11, 37, 15, 6, 6, 27, 18, 7, 6, 13, 50, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 21, 5, 12, 14, 6, 19, 22, 14, 12, 26, 22, 7, 5, 12, 18, 37, 15, 21, 13, 27, 41, 22, 13, 28, 62, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 5, 13, 20, 13, 6, 20, 13, 14, 5, 12, 13, 7, 13, 12, 19, 28, 30, 29, 13, 6, 4, 40, 0, 0, 0, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 9, 5, 9, 7, 5, 5, 7, 6, 4, 0, 6, 5, 2, 1, 0, 1, 2, 1, 1, 0, 0, 2, 4, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 9, 5, DWS, 6, 6, 5, 6, 6, 6, 1, 6, 6, 2, 2, 0, 1, 2, 3, 3, 0, 0, 2, 10, 0, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { 5, 4, 4, 1, 4, 4, 5, 5, 4, 1, 1, 2, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 9, 13, 0, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, DWS, },
 { DWE, 4, DWE, DWE, 11, 11, DWE, 11, DWE, 11, DWE, 11, DWE, 25, 25, 11, 4, DWE, 25, 25, 25, DWE, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 4, 4, 11, 11, 4, 11, 11, 25, DWE, DWE, DWE, DWE, 25, 25, 11, 11, 25, 25, 25, 25, DWE, 25, 25, 25, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { DWE, 4, 4, 4, 11, 11, DWE, DWE, 25, DWE, 11, 4, 11, 25, 25, 11, 11, DWE, DWE, DWE, 25, 4, 25, DWE, 4, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, DWE, },
 { 178, DWO, 123, 131, 124, 118, 132, 117, 141, 133, 138, 117, 124, 156, 132, 133, 117, 125, 122, 229, 232, 166, 219, 132, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 115, 117, 107, 132, 197, 117, 137, 244, 253, 117, 123, 244, 252, 253, 116, 135, 180, 252, 244, 132, 116, 147, 253, 228, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 133, 122, DWO, 133, 118, 131, 118, 125, 227, 132, 116, 126, 140, 116, 125, 117, 124, 117, 244, 116, 169, 151, 254, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 131, 117, 104, 118, 244, 132, 131, 164, 253, 117, 121, 149, 246, 245, 116, 133, 135, 252, 244, 244, 116, 132, 253, 244, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 116, 149, 135, 118, 118, 196, 131, 117, 123, 163, 164, 116, 126, 140, 132, DWO, 118, 125, 164, 132, 116, 188, 251, 208, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
 { 147, 164, 103, DWO, 164, 117, 117, 121, 244, 117, 118, 149, 245, 245, 116, 149, 132, 197, 244, 245, 116, DWO, 250, 105, 116, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, DWO, },
#else
  {  13,   5,   4,  14,   6,  11,  14,  14,   5,  11,  14,   7,   6,   5,   3,  22,  38,  22,  14,   5 },
  {  13,   5,   4,  14,   6,  11,  14,  14,   5,  11,  14,   7,   6,   5,   3,  22,  38,  22,  14,   5 },
  {   5,   5,  20,   6,   6,   4,   6,  14,   5,  12,  14,   7,  13,   5,  20,  21,   7,   6,   5,  28 },
  {   9,   5,   8,   6,   5,   4,   6,   5,   4,   0,   6,   6,   1,   4,   0,   1,   1,   1,   1,   0 },
  {   9,   5,   8,   6,   5,   4,   6,   5,   4,   0,   6,   6,   1,   4,   0,   1,   1,   1,   1,   0 },
  {   6,   5,   5,   6,   5,   5,   6,   6,   5,   1,   2,   6,   1,   1,   0,   2,   2,   1,   0,   0 },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  25,  25,  18,  32,  32,  18,  18,  25,  32,  32 },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,  25,  25,  18,  32,  32,  18,  18,  25,  32,  32 },
  {  18,  18,  18,  18,  18,  18,  18,  18,  18,  18,   4,  18,  11,  18,  25,  11,   4,  11,  11,  25 },
  { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196 },
  { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228 },
  { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196 },
  { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228 },
  { 116, 120, 118, 119, 123, 118, 118, 124, 119, 132, 117, 116, 124, 134, 116, 126, 118, 119, 118, 196 },
  { 116, 118, 118, 126, 228, 117, 116, 147, 230, 117, 132, 195, 165, 229, 116, 118, 123, 149, 228, 228 },
#endif
  }),
  ContextSetCfg::addCtxSet
  ({
// ctx 1357 to 1359
 { 19, 4, 41, },
 { 42, 19, 33, },
 { 4, 13, 43, },
 { 6, 6, 5, },
 { 6, 6, 9, },
 { 4, 4, 0, },
 { DWE, 25, 25, },
 { 11, DWE, 25, },
 { DWE, 25, 4, },
 { 116, 120, 136, },
 { 146, 117, 130, },
 { 117, DWO, 122, },
 { 131, 133, 115, },
 { 102, 121, 214, },
 { 195, 211, 116, },
  }),
};
#endif


const CtxSet ContextSetCfg::MVPIdx = ContextSetCfg::addCtxSet({
// ctx 1360
 { 34, },
 { 27, },
 { 41, },
 { 10, },
 { 9, },
 { 13, },
 { 11, },
 { 4, },
 { 25, },
 { 99, },
 { 100, },
 { 107, },
 { 101, },
 { 100, },
 { 99, },
});

const CtxSet ContextSetCfg::amFlagState = ContextSetCfg::addCtxSet({
// ctx 1361
 { 40, },
 { 26, },
 { 0, },
 { 5, },
 { 5, },
 { 5, },
 { 4, },
 { 4, },
 { 4, },
 { 196, },
 { 108, },
 { 132, },
 { 117, },
 { 106, },
 { 132, },
});

const CtxSet ContextSetCfg::SmvdFlag = ContextSetCfg::addCtxSet({
// ctx 1362
 { 43, },
 { 36, },
 { 0, },
 { 1, },
 { 4, },
 { 1, },
 { 4, },
 { 4, },
 { 4, },
 { 125, },
 { 117, },
 { 120, },
 { 102, },
 { 118, },
 { 125, },
});

#if JVET_AG0098_AMVP_WITH_SBTMVP
const CtxSet ContextSetCfg::amvpSbTmvpFlag = ContextSetCfg::addCtxSet({
  {  33,  33 },
  {  25,  49 },
  {  35,  35 },
  {   2,   1 },
  {   2,   1 },
  {   8,   8 },
  {  18,   4 },
  {  11,   4 },
  {  18,  18 },
  { 116, 134 },
  { 116, 134 },
  { 116, 134 },
  { 115, 117 },
  { 115, 117 },
  { 115, 117 },
});

const CtxSet ContextSetCfg::amvpSbTmvpMvdIdx = ContextSetCfg::addCtxSet({
  {   4,   4,  59 },
  {  58,  19,  33 },
  {  35,  35,  35 },
  {   3,   2,   3 },
  {   2,   3,   0 },
  {   8,   8,   8 },
  {   4,  11,  11 },
  {  11,   4,  11 },
  {  18,  18,  18 },
  { 151, 251, 251 },
  { 151, 251, 251 },
  { 151, 251, 251 },
  { 115, 115, 115 },
  { 115, 115, 115 },
  { 115, 115, 115 },
});
#endif

const CtxSet ContextSetCfg::SaoMergeFlag = ContextSetCfg::addCtxSet({
// ctx 1363
 { 10, },
 { 53, },
 { 61, },
 { 0, },
 { 0, },
 { 0, },
 { 11, },
 { 4, },
 { 4, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
 { 117, },
});

const CtxSet ContextSetCfg::SaoTypeIdx = ContextSetCfg::addCtxSet({
// ctx 1364
 { 10, },
 { 6, },
 { 4, },
 { 1, },
 { 10, },
 { 2, },
 { DWE, },
 { 25, },
 { 25, },
 { 253, },
 { 197, },
 { 253, },
 { 246, },
 { 252, },
 { 252, },
});

const CtxSet ContextSetCfg::BifCtrlFlags[3] = {
  ContextSetCfg::addCtxSet({
// ctx 1365
 { 30, },
 { 23, },
 { 38, },
 { 1, },
 { 5, },
 { 1, },
 { 4, },
 { 4, },
 { 4, },
 { 115, },
 { 115, },
 { 115, },
 { 115, },
 { 115, },
 { 115, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 1366
 { 30, },
 { 37, },
 { 37, },
 { 0, },
 { 0, },
 { 0, },
 { 4, },
 { 4, },
 { 4, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 1367
 { 30, },
 { 22, },
 { 29, },
 { 0, },
 { 0, },
 { 0, },
 { 4, },
 { 4, },
 { 4, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
 { 116, },
  }),
};

const CtxSet ContextSetCfg::CcSaoControlIdc = ContextSetCfg::addCtxSet({
// ctx 1368 to 1376
 { 11, 46, 54, 25, 53, 54, 25, 45, 46, },
 { 5, 53, 46, 18, 45, 46, 18, 29, 38, },
 { 43, 46, 54, 19, 46, 62, 58, 54, 54, },
 { 10, 0, 4, 7, 0, 0, 7, 0, 0, },
 { 5, 0, 4, 5, 0, 0, 4, 1, 0, },
 { 6, 0, 4, 6, 0, 4, 5, 1, 0, },
 { 25, DWE, DWE, 11, 25, DWE, 4, DWE, DWE, },
 { 25, 25, 25, DWE, 25, 25, 4, 25, 25, },
 { 25, DWE, DWE, 25, DWE, 4, DWE, 25, 11, },
 { 83, 116, 151, 97, 117, 182, 97, 132, 151, },
 { 212, 132, 99, 212, 148, 116, 243, 244, 116, },
 { 82, 132, 182, 97, 129, 230, 97, 132, 154, },
 { 217, 132, 99, 170, 132, 115, 154, 195, 99, },
 { 98, 148, 137, 97, 132, 150, 98, 120, 170, },
 { 251, 132, 99, 245, 164, 99, 213, 164, 116, },
});

const CtxSet ContextSetCfg::LFNSTIdx = ContextSetCfg::addCtxSet({
// ctx 1377 to 1380
 { 51, CNU, 43, 42, },
 { 36, CNU, 51, CNU, },
 { 0, 43, 50, 42, },
 { 10, DWS, 5, 12, },
 { 10, DWS, 6, 13, },
 { 10, 10, 5, 10, },
 { 4, DWE, 11, 4, },
 { 4, DWE, 11, 11, },
 { 4, 4, 4, 4, },
 { 116, 122, 117, 150, },
 { 118, 116, 116, 101, },
 { 131, 122, 117, 121, },
 { 118, 116, 116, 116, },
 { 103, 122, 117, 122, },
 { 163, 106, 116, 116, },
});

#if JVET_AG0061_INTER_LFNST_NSPT
const CtxSet ContextSetCfg::InterLFNSTIdx = ContextSetCfg::addCtxSet({
  {  CNU, CNU, CNU, },
  {  CNU, CNU, CNU, },
  {  CNU, CNU, CNU, },
  {  DWS, DWS, DWS, },
  {  DWS, DWS, DWS, },
  {  DWS, DWS, DWS, },
  {  DWE, DWE, DWE, },
  {  DWE, DWE, DWE, },
  {  DWE, DWE, DWE, },
  {  DWO, DWO, DWO, },
  {  DWO, DWO, DWO, },
  {  DWO, DWO, DWO, },
  {  DWO, DWO, DWO, },
  {  DWO, DWO, DWO, },
  {  DWO, DWO, DWO, },
});
#endif

const CtxSet ContextSetCfg::PLTFlag = ContextSetCfg::addCtxSet({
// ctx 1381
 { 0, },
 { 0, },
 { 25, },
 { 0, },
 { 0, },
 { 2, },
 { 25, },
 { 25, },
 { 25, },
 { 116, },
 { 116, },
 { 117, },
 { 126, },
 { 120, },
 { 124, },
});

const CtxSet ContextSetCfg::RotationFlag = ContextSetCfg::addCtxSet({
// ctx 1382
 { 58, },
 { 0, },
 { CNU, },
 { 4, },
 { 0, },
 { 2, },
 { 25, },
 { 25, },
 { 11, },
 { 115, },
 { 180, },
 { 130, },
 { 123, },
 { 130, },
 { 124, },
});

const CtxSet ContextSetCfg::RunTypeFlag = ContextSetCfg::addCtxSet({
// ctx 1383
 { 59, },
 { 0, },
 { 34, },
 { 2, },
 { 3, },
 { 9, },
 { 4, },
 { 25, },
 { 25, },
 { 116, },
 { 211, },
 { 123, },
 { 243, },
 { 107, },
 { 243, },
});

const CtxSet ContextSetCfg::IdxRunModel = ContextSetCfg::addCtxSet({
// ctx 1384 to 1388
 { 59, 53, 53, 46, 46, },
 { 0, 0, 0, 0, 0, },
 { 50, 37, 45, 30, 46, },
 { 0, 0, DWS, 0, 4, },
 { 0, 0, DWS, 2, 13, },
 { 5, 2, 6, 10, 5, },
 { 4, 4, 25, 11, 11, },
 { 25, DWE, DWE, 25, DWE, },
 { 11, 4, 4, DWE, DWE, },
 { 156, 118, 242, 149, 132, },
 { 154, 214, 178, 136, 116, },
 { 253, 118, 115, 105, 116, },
 { 254, 126, 118, 106, 116, },
 { 252, 118, 194, 104, 116, },
 { 252, 124, 116, 103, 116, },
});

const CtxSet ContextSetCfg::CopyRunModel = ContextSetCfg::addCtxSet({
// ctx 1389 to 1391
 { 31, 46, 39, },
 { 0, 0, 0, },
 { 39, 38, 54, },
 { 0, 1, DWS, },
 { 0, 0, DWS, },
 { 0, 5, DWS, },
 { 11, 4, DWE, },
 { 25, 25, 11, },
 { 25, 11, DWE, },
 { 135, 229, 151, },
 { 116, 133, 99, },
 { 150, 125, 134, },
 { 116, 116, 99, },
 { 116, 123, 134, },
 { 116, 147, 99, },
});

const CtxSet ContextSetCfg::TransformSkipFlag = ContextSetCfg::addCtxSet({
// ctx 1392 to 1393
 { 25, 1, },
 { 25, 0, },
 { 25, 9, },
 { 2, 5, },
 { 2, 6, },
 { 5, 5, },
 { DWE, DWE, },
 { 11, 25, },
 { 25, 25, },
 { 117, 99, },
 { 162, 130, },
 { 108, 101, },
 { 242, 145, },
 { 108, 101, },
 { 162, 130, },
});

const CtxSet ContextSetCfg::MTSIdx = ContextSetCfg::addCtxSet({
// ctx 1394 to 1397
 { 43, 53, 46, CNU, },
 { 36, 46, 46, CNU, },
 { 34, 53, 38, CNU, },
 { DWS, 9, 6, DWS, },
 { 12, 9, DWS, DWS, },
 { DWS, 9, DWS, DWS, },
 { 4, DWE, 4, DWE, },
 { DWE, 11, 4, DWE, },
 { 4, DWE, 4, DWE, },
 { 117, 123, 122, DWO, },
 { 117, 116, 100, DWO, },
 { 108, 134, 133, DWO, },
 { 244, 99, 83, DWO, },
 { 133, 133, 149, DWO, },
 { 118, 100, 83, DWO, },
});

const CtxSet ContextSetCfg::ISPMode = ContextSetCfg::addCtxSet({
// ctx 1398 to 1400
 { 33, 43, 25, },
 { 33, 43, 40, },
 { 33, 43, 33, },
 { 5, 1, 9, },
 { 5, 2, 9, },
 { 9, 1, 9, },
 { 11, 11, DWE, },
 { 11, 4, 25, },
 { DWE, 4, DWE, },
 { 103, 117, 98, },
 { 163, 132, 226, },
 { 100, 118, 82, },
 { 108, DWO, 180, },
 { 99, 117, 82, },
 { 162, 118, 135, },
});

const CtxSet ContextSetCfg::SbtFlag = ContextSetCfg::addCtxSet({
// ctx 1401 to 1402
 { 40, 48, },
 { 48, 56, },
 { 48, 56, },
 { 2, 6, },
 { 2, 6, },
 { 2, 7, },
 { 4, 4, },
 { 11, 4, },
 { 4, 4, },
 { 117, 101, },
 { 117, 116, },
 { DWO, 100, },
 { 117, 117, },
 { 117, 104, },
 { 118, 117, },
});

const CtxSet ContextSetCfg::SbtQuadFlag = ContextSetCfg::addCtxSet({
// ctx 1403
 { 42, },
 { 42, },
 { 57, },
 { 10, },
 { 10, },
 { 10, },
 { 4, },
 { 4, },
 { 11, },
 { 116, },
 { 130, },
 { 116, },
 { 103, },
 { 137, },
 { 100, },
});

const CtxSet ContextSetCfg::SbtHorFlag = ContextSetCfg::addCtxSet({
// ctx 1404 to 1406
 { 20, 58, 5, },
 { 20, 51, 19, },
 { 27, 51, 19, },
 { 5, 5, 2, },
 { 6, 5, 5, },
 { 6, 6, 5, },
 { 4, 11, 11, },
 { 11, 11, 11, },
 { 4, 11, 11, },
 { 116, 163, 117, },
 { 132, 108, 147, },
 { 116, 117, 116, },
 { 117, 107, 147, },
 { 115, 115, 101, },
 { 117, 106, 131, },
});

const CtxSet ContextSetCfg::SbtPosFlag = ContextSetCfg::addCtxSet({
// ctx 1407
 { 28, },
 { 28, },
 { 28, },
 { 13, },
 { 13, },
 { 13, },
 { 4, },
 { 4, },
 { 4, },
 { 84, },
 { 100, },
 { 90, },
 { 101, },
 { 85, },
 { 115, },
});

const CtxSet ContextSetCfg::ChromaQpAdjFlag = ContextSetCfg::addCtxSet({
// ctx 1408
 { CNU, },
 { CNU, },
 { CNU, },
 { DWS, },
 { DWS, },
 { DWS, },
 { DWE, },
 { DWE, },
 { DWE, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
});

const CtxSet ContextSetCfg::DimdFlag = ContextSetCfg::addCtxSet({
// ctx 1409 to 1411
 { 40, CNU, CNU, },
 { 48, CNU, CNU, },
 { 25, CNU, CNU, },
 { 9, DWS, DWS, },
 { 6, DWS, DWS, },
 { 6, DWS, DWS, },
 { 11, DWE, DWE, },
 { 4, DWE, DWE, },
 { 11, DWE, DWE, },
 { 116, DWO, DWO, },
 { 131, DWO, DWO, },
 { 107, DWO, DWO, },
 { 116, DWO, DWO, },
 { 107, DWO, DWO, },
 { 117, DWO, DWO, },
});

const CtxSet ContextSetCfg::TimdFlag = ContextSetCfg::addCtxSet({
// ctx 1412 to 1414
 { 41, 34, 42, },
 { 34, 34, 34, },
 { 57, 50, 58, },
 { 7, 7, 7, },
 { 7, 10, 7, },
 { 6, 6, 2, },
 { 4, 4, 25, },
 { 4, 4, 4, },
 { 4, 4, 4, },
 { 106, 101, 106, },
 { 135, 243, 177, },
 { 118, 117, 125, },
 { 133, 133, 118, },
 { 117, 116, 124, },
 { 134, 148, 121, },
});

const CtxSet ContextSetCfg::SgpmFlag = ContextSetCfg::addCtxSet({
// ctx 1415 to 1417
 { 26, 26, 26, },
 { 26, 26, 26, },
 { 41, 34, 34, },
 { 9, 9, 10, },
 { 9, 10, 7, },
 { 6, 9, 10, },
 { 4, 4, 4, },
 { 4, 11, 4, },
 { 4, DWE, 4, },
 { 85, 91, 162, },
 { 196, 164, 132, },
 { 91, 90, 117, },
 { 134, 165, 147, },
 { 107, 91, 83, },
 { 135, 196, 232, },
});

const CtxSet ContextSetCfg::ObmcFlag = ContextSetCfg::addCtxSet({
// ctx 1418 to 1419
 { 39, 60, },
 { 39, 53, },
 { 39, 61, },
 { 4, 4, },
 { 4, 5, },
 { 1, 6, },
 { DWE, DWE, },
 { 25, 11, },
 { 25, 11, },
 { 243, 131, },
 { 99, 147, },
 { 147, 115, },
 { 99, 131, },
 { 152, 102, },
 { 115, 115, },
});

const CtxSet ContextSetCfg::ChromaQpAdjIdc = ContextSetCfg::addCtxSet({
// ctx 1420
 { CNU, },
 { CNU, },
 { CNU, },
 { DWS, },
 { DWS, },
 { DWS, },
 { DWE, },
 { DWE, },
 { DWE, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
 { DWO, },
});

const CtxSet ContextSetCfg::ImvFlag = ContextSetCfg::addCtxSet({
// ctx 1421 to 1425
 { 60, 41, 50, 60, 46, },
 { 59, 33, 50, 53, 60, },
 { 52, 33, 0, 0, 60, },
 { 1, 4, 1, 0, 4, },
 { 1, 5, 1, 0, 5, },
 { 2, 5, 2, 0, 5, },
 { 4, 25, 11, 25, 4, },
 { 4, 11, 11, 25, 11, },
 { 4, 4, 11, 11, 4, },
 { 117, 102, 117, 116, 133, },
 { 117, 116, 118, 116, 105, },
 { 118, 108, 117, 116, 133, },
 { 117, 116, 118, 116, 104, },
 { 118, 102, 116, 116, 132, },
 { 117, 132, 124, 116, 118, },
});

const CtxSet ContextSetCfg::ImvFlagIBC = ContextSetCfg::addCtxSet({
// ctx 1426 to 1430
 { 54, 41, CNU, CNU, CNU, },
 { 38, 33, CNU, CNU, CNU, },
 { 43, 0, CNU, CNU, CNU, },
 { 2, 0, DWS, DWS, DWS, },
 { 2, DWS, DWS, DWS, DWS, },
 { 0, 0, DWS, DWS, DWS, },
 { DWE, DWE, DWE, DWE, DWE, },
 { DWE, 25, DWE, DWE, DWE, },
 { 25, 25, DWE, DWE, DWE, },
 { 146, 132, DWO, DWO, DWO, },
 { 116, 117, DWO, DWO, DWO, },
 { 126, 85, DWO, DWO, DWO, },
 { 117, 147, DWO, DWO, DWO, },
 { 126, 116, DWO, DWO, DWO, },
 { 118, 148, DWO, DWO, DWO, },
});

const CtxSet ContextSetCfg::ctbAlfFlag = ContextSetCfg::addCtxSet({
// ctx 1431 to 1439
 { 19, 31, 46, 18, 46, 62, 18, 46, 62, },
 { 5, 23, 31, 19, 46, 62, 12, 46, 62, },
 { 31, 39, 39, 54, 39, 39, 46, 39, 39, },
 { 12, 1, DWS, 10, 0, DWS, 12, 0, 9, },
 { 10, 5, DWS, 5, 4, 5, 5, 1, 5, },
 { 10, 5, 9, 9, 4, DWS, 7, 4, 10, },
 { 25, DWE, DWE, 25, 11, 11, 25, 11, DWE, },
 { 25, 25, DWE, 25, 25, 11, 25, DWE, 11, },
 { 25, 25, DWE, 25, 25, DWE, 25, DWE, 25, },
 { 81, 148, 118, 81, 132, 196, 65, 132, 118, },
 { 229, 250, 98, 250, 180, 114, 199, 228, 82, },
 { 84, 148, 133, 67, 148, 134, 67, 147, 123, },
 { 150, 248, 114, 234, 228, 98, 218, 212, 82, },
 { 112, 135, DWO, 83, 195, 180, 105, 116, 118, },
 { 244, 251, 130, 251, 231, 83, 250, 233, 83, },
});

const CtxSet ContextSetCfg::ctbAlfAlternative = ContextSetCfg::addCtxSet({
// ctx 1440 to 1442
 { 26, 34, 34, },
 { 19, 26, 19, },
 { 26, 26, 26, },
 { 0, 0, 0, },
 { 0, 0, 0, },
 { 0, 0, 0, },
 { DWE, 25, 25, },
 { DWE, 25, 25, },
 { 25, 25, 25, },
 { 132, 132, 132, },
 { 116, 116, 116, },
 { 213, 149, 149, },
 { 116, 116, 116, },
 { 151, 150, 151, },
 { 116, 116, 116, },
});

const CtxSet ContextSetCfg::AlfUseTemporalFilt = ContextSetCfg::addCtxSet({
// ctx 1443
 { 39, },
 { 39, },
 { 39, },
 { 4, },
 { 5, },
 { 5, },
 { 4, },
 { 4, },
 { DWE, },
 { 116, },
 { 100, },
 { 117, },
 { 99, },
 { 242, },
 { 101, },
});

const CtxSet ContextSetCfg::CcAlfFilterControlFlag = ContextSetCfg::addCtxSet({
// ctx 1444 to 1449
 { 18, 52, 46, 18, 52, 46, },
 { 3, 45, 54, 3, 60, 46, },
 { 3, 29, 46, 18, 52, 46, },
 { 5, 2, 5, 4, 2, 5, },
 { 1, 3, 2, 4, 2, 5, },
 { 4, 3, 5, 4, 3, 4, },
 { DWE, DWE, 4, 11, DWE, 11, },
 { 4, DWE, 4, 11, DWE, 11, },
 { DWE, DWE, 4, DWE, DWE, 4, },
 { 99, 115, 108, 100, 130, DWO, },
 { 137, 130, 99, 120, 146, 98, },
 { 99, 115, 117, 99, 115, 120, },
 { 138, 130, 98, 213, 146, 99, },
 { 99, 120, 116, 100, 115, 197, },
 { 137, 131, 146, 135, 146, 99, },
});

const CtxSet ContextSetCfg::CiipFlag = ContextSetCfg::addCtxSet({
// ctx 1450 to 1451
 { 56, 43, },
 { 49, 43, },
 { 0, 28, },
 { 2, 2, },
 { 2, 2, },
 { 2, 2, },
 { 11, 4, },
 { 11, 4, },
 { 11, 4, },
 { 117, 123, },
 { 117, 122, },
 { 117, 120, },
 { 117, 118, },
 { 116, 123, },
 { 131, 124, },
});

const CtxSet ContextSetCfg::IBCFlag = ContextSetCfg::addCtxSet({
// ctx 1452 to 1454
 { 25, CNU, 30, },
 { 0, 34, 29, },
 { 1, 26, 20, },
 { 9, 5, 9, },
 { DWS, 6, 9, },
 { 5, 5, DWS, },
 { 25, 11, DWE, },
 { 25, 11, DWE, },
 { 11, DWE, 25, },
 { 103, 117, 131, },
 { 162, 243, 146, },
 { 116, 118, 131, },
 { 132, 116, 115, },
 { 116, 117, 108, },
 { 132, 116, 115, },
});

const CtxSet ContextSetCfg::BiPredIbcFlag = ContextSetCfg::addCtxSet({
// ctx 1455 to 1456
 { CNU, CNU, },
 { CNU, CNU, },
 { 20, 18, },
 { DWS, DWS, },
 { DWS, DWS, },
 { 5, 1, },
 { DWE, DWE, },
 { DWE, DWE, },
 { 25, 4, },
 { 124, 122, },
 { 116, 164, },
 { 124, 122, },
 { 116, 164, },
 { DWO, 118, },
 { 108, 164, },
});

const CtxSet ContextSetCfg::IbcCiipFlag = ContextSetCfg::addCtxSet({
// ctx 1457 to 1459
 { 40, CNU, CNU, },
 { 41, CNU, CNU, },
 { 40, 0, CNU, },
 { 5, DWS, DWS, },
 { 10, DWS, DWS, },
 { 9, 4, DWS, },
 { 25, DWE, DWE, },
 { 25, DWE, DWE, },
 { 25, 4, DWE, },
 { 98, 99, DWO, },
 { 147, 137, DWO, },
 { 83, 99, DWO, },
 { 122, 137, DWO, },
 { 82, 99, DWO, },
 { 210, 139, DWO, },
});

const CtxSet ContextSetCfg::IbcCiipIntraIdx = ContextSetCfg::addCtxSet({
// ctx 1460
 { 50, },
 { 50, },
 { 50, },
 { DWS, },
 { 5, },
 { 12, },
 { 4, },
 { 25, },
 { DWE, },
 { 133, },
 { 149, },
 { 186, },
 { 116, },
 { 201, },
 { 91, },
});

const CtxSet ContextSetCfg::IbcGpmFlag = ContextSetCfg::addCtxSet({
// ctx 1461
 { 41, },
 { 26, },
 { 41, },
 { 5, },
 { 5, },
 { 7, },
 { 11, },
 { 11, },
 { DWE, },
 { 102, },
 { 116, },
 { 102, },
 { 117, },
 { 100, },
 { 116, },
});

const CtxSet ContextSetCfg::IbcGpmIntraFlag = ContextSetCfg::addCtxSet({
// ctx 1462 to 1463
 { 34, 42, },
 { CNU, 58, },
 { 42, 57, },
 { 3, 1, },
 { 6, 3, },
 { 3, 2, },
 { 11, 4, },
 { 4, 4, },
 { 11, DWE, },
 { 115, 116, },
 { 116, 117, },
 { 115, 116, },
 { 124, DWO, },
 { 115, 115, },
 { 123, 117, },
});

const CtxSet ContextSetCfg::IbcGpmSplitDirSetFlag = ContextSetCfg::addCtxSet({
// ctx 1464
 { 38, },
 { 43, },
 { 22, },
 { 5, },
 { 2, },
 { 5, },
 { 11, },
 { 4, },
 { DWE, },
 { 116, },
 { 116, },
 { 147, },
 { 91, },
 { 163, },
 { 108, },
});

const CtxSet ContextSetCfg::IbcGpmBldIdx = ContextSetCfg::addCtxSet({
// ctx 1465 to 1468
 { CNU, CNU, CNU, CNU, },
 { CNU, CNU, CNU, CNU, },
 { 38, 52, 42, 36, },
 { DWS, DWS, DWS, DWS, },
 { DWS, DWS, DWS, DWS, },
 { 5, 5, 10, 12, },
 { DWE, DWE, DWE, DWE, },
 { DWE, DWE, DWE, DWE, },
 { 25, 4, 11, 25, },
 { 226, DWO, 117, 131, },
 { 108, 99, DWO, 118, },
 { 226, DWO, 117, 131, },
 { 108, 99, DWO, 118, },
 { 131, 137, 195, 227, },
 { 107, 99, 146, 132, },
});

const CtxSet ContextSetCfg::IbcLicFlag = ContextSetCfg::addCtxSet({
// ctx 1469 to 1472
 { 26, CNU, CNU, CNU, },
 { 33, CNU, CNU, CNU, },
 { 4, 17, 33, 18, },
 { 1, DWS, DWS, DWS, },
 { 6, DWS, DWS, DWS, },
 { 1, 6, 5, 3, },
 { DWE, DWE, DWE, DWE, },
 { DWE, DWE, DWE, DWE, },
 { 25, 25, 25, DWE, },
 { 116, 98, 100, 118, },
 { 115, 162, 226, 115, },
 { 97, 98, 100, 118, },
 { 116, 162, 226, 115, },
 { 115, 98, 103, 115, },
 { 115, 242, 117, 115, },
});

const CtxSet ContextSetCfg::IbcLicIndex = ContextSetCfg::addCtxSet({
// ctx 1473 to 1474
 { 19, 43, },
 { 51, 43, },
 { 41, 41, },
 { 4, 1, },
 { 13, 4, },
 { 4, 0, },
 { 25, 25, },
 { 25, 25, },
 { 11, 4, },
 { 131, 117, },
 { 108, 147, },
 { 141, 124, },
 { 108, 133, },
 { 136, 122, },
 { 102, 133, },
});

const CtxSet ContextSetCfg::JointCbCrFlag = ContextSetCfg::addCtxSet({
// ctx 1475 to 1477
 { 34, 28, 59, },
 { 27, 28, 52, },
 { 12, 29, 58, },
 { 1, 0, 1, },
 { 1, 0, 2, },
 { 1, 1, 1, },
 { DWE, DWE, 25, },
 { DWE, DWE, 25, },
 { DWE, DWE, 25, },
 { 117, 132, 116, },
 { 117, 116, 116, },
 { 117, 147, 116, },
 { 118, 116, 116, },
 { 116, 147, 116, },
 { 118, 116, 116, },
});

const CtxSet ContextSetCfg::TsSigCoeffGroup = ContextSetCfg::addCtxSet({
// ctx 1478 to 1480
 { 18, 20, 37, },
 { 18, 27, 44, },
 { 18, 20, 38, },
 { 6, 9, 5, },
 { 6, 10, 6, },
 { 6, DWS, 6, },
 { 11, DWE, 4, },
 { 11, 11, 4, },
 { DWE, DWE, DWE, },
 { 116, 121, 133, },
 { 243, 251, 180, },
 { 116, 124, 212, },
 { 211, 165, 118, },
 { 121, 244, 243, },
 { 163, 165, 101, },
});

const CtxSet ContextSetCfg::TsSigFlag = ContextSetCfg::addCtxSet({
// ctx 1481 to 1483
 { 25, CNU, 37, },
 { 25, CNU, 44, },
 { 25, 28, 38, },
 { 13, 9, 5, },
 { 13, 13, 5, },
 { 13, 13, 10, },
 { 4, 4, DWE, },
 { 4, 11, 11, },
 { 4, DWE, DWE, },
 { 74, 251, 104, },
 { 117, 251, 108, },
 { 74, 251, 105, },
 { 101, 251, 108, },
 { 74, 234, 99, },
 { 90, 234, 102, },
});

const CtxSet ContextSetCfg::TsParFlag = ContextSetCfg::addCtxSet({
// ctx 1484
 { 3, },
 { 10, },
 { 11, },
 { 5, },
 { 6, },
 { 5, },
 { DWE, },
 { 4, },
 { 4, },
 { 116, },
 { 131, },
 { 117, },
 { 116, },
 { 164, },
 { 105, },
});

const CtxSet ContextSetCfg::TsGtxFlag = ContextSetCfg::addCtxSet({
// ctx 1485 to 1489
 { CNU, 3, 4, 5, 5, },
 { CNU, 17, 2, 18, 58, },
 { CNU, 3, 3, 4, 3, },
 { DWS, 1, 5, 4, 1, },
 { DWS, 2, 2, 1, 1, },
 { DWS, 1, 5, 1, 1, },
 { DWE, DWE, 25, DWE, DWE, },
 { DWE, 11, 11, 11, 11, },
 { DWE, DWE, 25, DWE, DWE, },
 { DWO, 118, 146, 243, 243, },
 { DWO, 118, 107, 107, DWO, },
 { DWO, 132, 242, 243, 179, },
 { DWO, 118, 101, 117, 116, },
 { DWO, 132, 146, 243, 179, },
 { DWO, 117, 100, 117, 116, },
});

const CtxSet ContextSetCfg::TsLrg1Flag = ContextSetCfg::addCtxSet({
// ctx 1490 to 1493
 { 26, 11, 4, CNU, },
 { 25, 11, 4, CNU, },
 { 4, 5, 5, 14, },
 { 1, 1, 1, DWS, },
 { 6, 3, 2, DWS, },
 { 5, 2, 2, 7, },
 { 11, 11, 11, DWE, },
 { 11, DWE, 11, DWE, },
 { DWE, DWE, DWE, DWE, },
 { 117, 116, 163, 129, },
 { 132, 116, 116, 100, },
 { 116, 116, 116, 129, },
 { 242, 116, 116, 100, },
 { 108, 116, 131, 129, },
 { 242, 116, 116, 99, },
});

const CtxSet ContextSetCfg::TsResidualSign = ContextSetCfg::addCtxSet({
// ctx 1494 to 1499
 { 20, 10, 53, CNU, CNU, CNU, },
 { 12, 10, 53, CNU, CNU, CNU, },
 { 5, 17, 46, 28, 25, 46, },
 { 1, 5, 5, DWS, DWS, DWS, },
 { 5, 6, 5, DWS, DWS, DWS, },
 { 1, 5, 1, 5, 9, DWS, },
 { 4, DWE, DWE, DWE, DWE, DWE, },
 { 11, DWE, DWE, DWE, DWE, DWE, },
 { 25, DWE, 4, DWE, 25, DWE, },
 { DWO, 117, 116, 162, 232, 107, },
 { DWO, 116, 116, 115, 130, 251, },
 { 118, 116, 117, 162, 232, 107, },
 { 117, 116, 116, 115, 130, 251, },
 { DWO, 116, 118, 242, 251, 107, },
 { 117, 116, 116, 115, 114, 236, },
});

const CtxSet ContextSetCfg::signPred[2] = {
  ContextSetCfg::addCtxSet({
// ctx 1500 to 1503
 { 34, 34, 34, 26, },
 { 34, 34, 19, 26, },
 { 34, 34, 34, 26, },
 { 13, 10, 10, 10, },
 { 13, 13, 13, 10, },
 { 13, 10, 13, 10, },
 { 4, 4, 4, 4, },
 { 4, 4, 4, 4, },
 { 4, 4, 4, 4, },
 { 102, 100, 106, 100, },
 { 89, 106, 106, 101, },
 { 102, 102, 102, 101, },
 { 100, 102, 106, 115, },
 { 101, 102, 100, 105, },
 { 100, 102, 100, 100, },
  }),
  ContextSetCfg::addCtxSet({
// ctx 1504 to 1507
 { 26, 41, 34, 26, },
 { 41, 41, 26, 41, },
 { 34, 34, 19, 26, },
 { 13, 13, 10, 6, },
 { 13, 13, 10, 10, },
 { 10, 10, 10, 10, },
 { 4, 11, 11, 4, },
 { 4, 11, 4, 4, },
 { 4, 4, 4, 4, },
 { 85, 67, 99, 102, },
 { 101, 106, 101, 115, },
 { 90, 67, 100, 85, },
 { 116, 106, 106, 115, },
 { 90, 82, 106, 90, },
 { 106, 106, 101, 178, },
  }),
};

const CtxSet ContextSetCfg::CclmDeltaFlags = ContextSetCfg::addCtxSet({
// ctx 1508 to 1512
 { 26, 42, CNU, 51, 51, },
 { 26, 42, 4, 43, 43, },
 { CNU, 42, 34, 36, 43, },
 { 4, 3, 7, 10, DWS, },
 { 4, 9, 3, 9, 7, },
 { 0, 9, 0, 13, 12, },
 { 25, DWE, 25, 25, 25, },
 { 25, 25, 4, 25, 25, },
 { 4, DWE, 11, DWE, 4, },
 { 99, 250, 213, 234, 99, },
 { 155, 131, 115, 106, 120, },
 { 100, 250, 254, 218, 130, },
 { 141, 103, 116, 103, 153, },
 { 116, 234, 252, 234, 71, },
 { 254, 101, 116, 116, 183, },
});

const CtxSet ContextSetCfg::GlmFlags = ContextSetCfg::addCtxSet({
// ctx 1513 to 1517
 { 25, 37, 60, 12, CNU, },
 { 18, 51, 36, 20, CNU, },
 { 25, 14, 52, 28, 43, },
 { 0, 0, 4, 1, 1, },
 { 0, DWS, 4, 4, DWS, },
 { 0, DWS, 0, 0, 0, },
 { 25, 25, 25, 25, DWE, },
 { 25, 25, 25, 25, 25, },
 { 4, 25, DWE, 25, 4, },
 { 116, 158, 131, 117, 120, },
 { 141, 116, 101, 163, 211, },
 { 117, 149, 117, 117, 212, },
 { 165, 84, 102, 118, DWO, },
 { 133, 148, 116, 117, 212, },
 { 244, 82, 118, 117, 118, },
});

const CtxSet ContextSetCfg::CccmFlag = ContextSetCfg::addCtxSet({
// ctx 1518 to 1520
 { 13, 51, 36, },
 { 28, 43, 28, },
 { 29, 42, 42, },
 { DWS, 4, 4, },
 { DWS, DWS, DWS, },
 { 4, 4, 4, },
 { 25, 25, 25, },
 { 25, 25, 25, },
 { 11, DWE, 11, },
 { 115, DWO, 132, },
 { 242, 131, 131, },
 { DWO, 118, 118, },
 { 117, 148, DWO, },
 { 118, 117, 117, },
 { 117, 132, 118, },
});

const CtxSet ContextSetCfg::CccmMpfFlag = ContextSetCfg::addCtxSet({
// ctx 1521 to 1523
 { 27, CNU, CNU, },
 { CNU, CNU, 27, },
 { 50, 45, 50, },
 { DWS, DWS, 4, },
 { DWS, DWS, 9, },
 { DWS, 4, 4, },
 { 25, 25, 25, },
 { 25, 25, 25, },
 { 25, 11, DWE, },
 { 242, 136, 108, },
 { 242, 242, 148, },
 { 117, 133, 116, },
 { 116, 132, 196, },
 { 117, 196, 131, },
 { 116, 148, 164, },
});

const CtxSet ContextSetCfg::BvgCccmFlag = ContextSetCfg::addCtxSet({
// ctx 1524
 { CNU, },
 { CNU, },
 { 33, },
 { DWS, },
 { DWS, },
 { 1, },
 { DWE, },
 { DWE, },
 { DWE, },
 { 125, },
 { 117, },
 { 125, },
 { 117, },
 { 125, },
 { 116, },
});

const CtxSet ContextSetCfg::CcInsideFilterFlag = ContextSetCfg::addCtxSet({
// ctx 1525
 { 52, },
 { 21, },
 { 53, },
 { 0, },
 { 4, },
 { 0, },
 { 25, },
 { 25, },
 { 25, },
 { 124, },
 { 117, },
 { 118, },
 { 109, },
 { 116, },
 { 117, },
});

const CtxSet ContextSetCfg::ChromaFusionType = ContextSetCfg::addCtxSet({
// ctx 1526
 { 6, },
 { 21, },
 { 28, },
 { DWS, },
 { DWS, },
 { 5, },
 { 25, },
 { 25, },
 { 11, },
 { 182, },
 { 98, },
 { 132, },
 { 118, },
 { 134, },
 { 104, },
});

const CtxSet ContextSetCfg::ChromaFusionCclm = ContextSetCfg::addCtxSet({
// ctx 1527
 { 50, },
 { 50, },
 { 58, },
 { DWS, },
 { DWS, },
 { 4, },
 { 25, },
 { 25, },
 { 11, },
 { 162, },
 { 165, },
 { 125, },
 { 148, },
 { 165, },
 { 132, },
});

const CtxSet ContextSetCfg::TmrlDerive = ContextSetCfg::addCtxSet({
// ctx 1528 to 1535
 { 41, 50, 43, CNU, CNU, CNU, CNU, CNU, },
 { 41, 50, CNU, CNU, CNU, CNU, CNU, CNU, },
 { 41, 50, 43, 43, CNU, CNU, 50, CNU, },
 { 9, DWS, 9, DWS, 9, DWS, DWS, DWS, },
 { 9, DWS, 9, 9, 9, DWS, 9, DWS, },
 { 5, DWS, 9, 10, 9, DWS, 9, DWS, },
 { 25, DWE, DWE, 4, 11, 11, 4, DWE, },
 { 25, DWE, DWE, 11, 4, 11, 11, DWE, },
 { DWE, DWE, DWE, 11, 4, DWE, DWE, DWE, },
 { 86, 136, 251, 252, 135, 107, 100, DWO, },
 { 130, 108, 100, 84, 116, 120, 200, DWO, },
 { 91, DWO, 138, 140, 182, 118, 116, DWO, },
 { 118, 118, 107, 115, 243, 124, 134, DWO, },
 { 108, 118, 184, 184, 251, 117, 106, DWO, },
 { 147, 117, 100, 146, 179, 134, 150, DWO, },
});

const CtxSet ContextSetCfg::nonLocalCCP = ContextSetCfg::addCtxSet({
// ctx 1536
 { 59, },
 { 59, },
 { 42, },
 { 4, },
 { DWS, },
 { 5, },
 { DWE, },
 { 25, },
 { DWE, },
 { 109, },
 { 243, },
 { 118, },
 { 163, },
 { 117, },
 { 147, },
});

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
const CtxSet ContextSetCfg::decoderDerivedCCP = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { CNU, },
  { DWS, },
  { DWS, },
  { DWS, },
  { DWE, },
  { DWE, },
  { DWE, },
  { DWO, },
  { DWO, },
  { DWO, },
  { DWO, },
  { DWO, },
  { DWO, },
	});

const CtxSet ContextSetCfg::ddNonLocalCCP = ContextSetCfg::addCtxSet
({
  { CNU, },
  { CNU, },
  { 50, },
  { DWS, },
  { DWS, },
  { 4, },
  { DWE, },
  { DWE, },
  { 11, },
  { DWO, },
  { DWO, },
  { DWO, },
  { DWO, },
  { DWO, },
  { DWO, },
	});
#endif

const CtxSet ContextSetCfg::InterCccmFlag = ContextSetCfg::addCtxSet({
// ctx 1537
 { 25, },
 { 40, },
 { 17, },
 { 5, },
 { 5, },
 { 5, },
 { 25, },
 { 25, },
 { DWE, },
 { 99, },
 { 146, },
 { 99, },
 { 147, },
 { 99, },
 { 131, },
});

#if JVET_AF0073_INTER_CCP_MERGE
const CtxSet ContextSetCfg::InterCcpMergeFlag = ContextSetCfg::addCtxSet({
// ctx 1538
 { 40, },
 { 48, },
 { 40, },
 { 5, },
 { 5, },
 { 2, },
 { 25, },
 { 25, },
 { DWE, },
 { 99, },
 { 114, },
 { 102, },
 { 210, },
 { 118, },
 { 178, },
});
#endif

#if JVET_AG0058_EIP
const CtxSet ContextSetCfg::EipFlag = ContextSetCfg::addCtxSet
  ({
     { CNU, CNU, },
     { CNU, CNU, },
     { CNU, CNU, },
     { DWS, DWS, },
     { DWS, DWS, },
     { DWS, DWS, },
     { DWE, DWE, },
     { DWE, DWE, },
     { DWE, DWE, },
     { DWO, DWO, },
     { DWO, DWO, },
     { DWO, DWO, },
     { DWO, DWO, },
     { DWO, DWO, },
     { DWO, DWO, },
     });
#endif

#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
const CtxSet ContextSetCfg::CCPMergeFusionFlag = ContextSetCfg::addCtxSet({
  { CNU },
  { CNU },
  { CNU },
  { DWS },
  { DWS },
  { DWS },
  { DWE },
  { DWE },
  { DWE },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
});
const CtxSet ContextSetCfg::CCPMergeFusionType = ContextSetCfg::addCtxSet({
  { CNU },
  { CNU },
  { CNU },
  { DWS },
  { DWS },
  { DWS },
  { DWE },
  { DWE },
  { DWE },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
  { DWO },
});
#endif
// CONTEXTS WSA STOP
#endif
