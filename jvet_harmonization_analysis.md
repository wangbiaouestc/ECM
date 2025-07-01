# Harmonization Analysis: JVET_AC0105_DIRECTIONAL_PLANAR and JVET_AK0061_PDP_MPM

## Executive Summary

This document analyzes how two ECM (Enhanced Compression Model) features are harmonized:
- **JVET_AC0105_DIRECTIONAL_PLANAR**: Directional planar prediction
- **JVET_AK0061_PDP_MPM**: MPM (Most Probable Mode) construction with matrix-based position dependent intra prediction

The key harmonization mechanism addresses the fundamental conflict that PDP MPM can change the traditional planar mode position (typically first) in the MPM list, while directional planar introduces directional variants of planar mode.

## Feature Overview

### JVET_AC0105_DIRECTIONAL_PLANAR
- Introduces directional variants of planar prediction
- Controlled by `plIdx` parameter in the CU (Coding Unit)
- Three modes:
  - `plIdx = 0`: Original planar prediction (bilinear interpolation)
  - `plIdx = 1`: Horizontal planar (only horizontal gradient)
  - `plIdx = 2`: Vertical planar (only vertical gradient)
- Only available when specific conditions are met (single reference, no special modes)

### JVET_AK0061_PDP_MPM
- Modifies MPM list construction when Position Dependent Prediction (PDP) is available
- Can move planar mode from its traditional first position in the MPM list
- Uses template-based analysis to determine PDP availability
- Affects both spatial candidate derivation and final MPM list filling

## Key Harmonization Mechanisms

### 1. Conditional Planar Insertion in MPM List

The core harmonization occurs in the `getIntraMPMs()` function in `UnitTools.cpp` (lines 2034-2045):

```cpp
#if JVET_AK0061_PDP_MPM
 #if JVET_AL0125_IMPROVEMENT_ON_MPM
  if (!(pdpRefAvailable||planarDisable)) 
 #else 
  if (!pdpRefAvailable) 
 #endif
  {
    mpm[numCand++] = PLANAR_IDX; 
  }
#else 
    mpm[numCand++] = PLANAR_IDX;
#endif
```

**Harmonization Logic:**
- When PDP is **NOT** available: Planar mode is inserted at the beginning of the MPM list (traditional behavior)
- When PDP **IS** available: Planar mode insertion is **skipped** initially, allowing PDP-derived modes to populate the list first

### 2. Guaranteed Planar Inclusion

In the `fillMPMList()` function (lines 40104-40113), there's a safeguard mechanism:

```cpp
#if JVET_AK0061_PDP_MPM 
#if JVET_AL0125_IMPROVEMENT_ON_MPM
  if (!planarDisable&&pdpRefAvailable && !includedMode[PLANAR_IDX]) 
#else
  if (pdpRefAvailable && !includedMode[PLANAR_IDX]) 
#endif
  {
    includedMode[PLANAR_IDX] = true;
    mpm[idx++] = PLANAR_IDX;
  }
#endif
```

**Harmonization Ensures:**
- Even when PDP changes the initial MPM construction, planar mode is guaranteed to be included
- This prevents directional planar modes from being completely excluded from consideration

### 3. Directional Planar Mode Selection

The directional planar selection happens in transform-level processing (`IntraPrediction.cpp` lines 1649, 2826, 3292):

```cpp
#if JVET_AC0105_DIRECTIONAL_PLANAR
case (PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred, isLuma(compID) ? pu.cu->plIdx : 0); break;
#else
case (PLANAR_IDX): xPredIntraPlanar(srcBuf, piPred); break;
#endif
```

**Harmonization Strategy:**
- Directional planar operates at the prediction level, not the mode signaling level
- The MPM list still contains `PLANAR_IDX`, but the actual prediction varies based on `plIdx`
- This separation allows PDP MPM changes to coexist with directional planar variants

### 4. PDP Mode Mapping

When PDP is available, there's a special mapping function `getPDPPredMode()` (lines 39672-39723) that:
- Maps traditional intra modes to PDP-compatible modes
- Preserves mode relationships while adapting to PDP constraints
- Handles planar mode specially within this mapping

## Implementation Details

### Availability Conditions

**Directional Planar** (`isDirectionalPlanarAvailable()` - lines 38638-38648):
```cpp
if (pu->multiRefIdx == 0 && cu.ispMode == 0 && cu.mipFlag == 0 && 
    cu.dimd == 0 && cu.timd == 0 && cu.tmpFlag == 0 && 
    cu.tmrlFlag == 0 && cu.sgpm == 0 && isLuma(cu.chType))
```

**PDP MPM** (`determinePDPTemp()` - lines 1987-2000):
- Requires sufficient template area (≥ `MPM_SORT_TEMPLATE_SIZE`)
- Needs valid PDP filters for the block size
- Must have adequate reference samples

### Mode Signaling Harmonization

The signaling is harmonized in `CABACWriter.cpp` and `CABACReader.cpp`:

```cpp
#if JVET_AC0105_DIRECTIONAL_PLANAR
if (CU::isDirectionalPlanarAvailable(cu) && !(enablePlanarSort || planarDisable) && 
    pu->ipredIdx == 0 && mpmFlag[k])
{
  // Signal directional planar index
}
#endif
```

**Key Points:**
- Directional planar signaling is conditional on both availability and MPM selection
- PDP MPM sorting (`enablePlanarSort`) affects when directional planar signaling occurs
- The harmonization ensures consistent decoder behavior regardless of MPM list changes

## Impact Analysis

### Coding Efficiency
1. **Synergistic Benefits**: PDP can identify better spatial candidates while directional planar provides better prediction for planar-like content
2. **Redundancy Mitigation**: The conditional insertion prevents double-counting of planar-related modes
3. **Adaptive Behavior**: The system adapts based on local content characteristics

### Complexity
1. **Encoder**: Moderate increase due to PDP template analysis and directional planar RD optimization
2. **Decoder**: Minimal increase - mostly conditional logic in MPM construction and prediction

### Compatibility
- Backward compatible: Both features can be disabled independently
- Forward compatible: The harmonization framework supports future extensions

## Conclusion

The harmonization between JVET_AC0105_DIRECTIONAL_PLANAR and JVET_AK0061_PDP_MPM is achieved through:

1. **Conditional MPM Construction**: PDP availability determines initial planar insertion
2. **Guaranteed Mode Inclusion**: Safeguards ensure planar mode availability for directional variants
3. **Layered Operation**: Directional planar works at prediction level, PDP at mode derivation level
4. **Consistent Signaling**: Harmonized syntax element coding based on both features' states

This design maintains the benefits of both features while preventing conflicts in MPM list construction and mode signaling. The separation of concerns allows each feature to operate in its optimal domain while providing a unified, coherent intra prediction framework.