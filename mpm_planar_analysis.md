# MPM List Analysis: Is Planar Mode Guaranteed?

## Executive Summary

**Answer: NO, planar mode is NOT guaranteed to be in the MPM list under all conditions.**

While there are several mechanisms that attempt to ensure planar mode inclusion, there are specific scenarios where planar mode can be completely excluded from the MPM list.

## Detailed Analysis

### MPM List Structure

The MPM list contains:
- **Primary MPM**: 6 modes (`NUM_PRIMARY_MOST_PROBABLE_MODES = 6`)
- **Secondary MPM**: 16 modes (`NUM_SECONDARY_MOST_PROBABLE_MODES = 16`) 
- **Total**: 22 modes (`NUM_MOST_PROBABLE_MODES = 22`)

### Planar Mode Inclusion Logic

#### 1. Initial Insertion (Conditional)

In `getIntraMPMs()` function (lines 2034-2054):

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

**Key Points:**
- Without PDP: Planar is inserted at position 0
- With PDP available: Planar insertion is **SKIPPED**
- With `planarDisable` (AL0125): Planar insertion is **SKIPPED**

#### 2. Safeguard Mechanism in fillMPMList()

In `fillMPMList()` function (lines 40141-40147):

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

**Critical Limitation:** This safeguard only applies when:
- PDP is available (`pdpRefAvailable == true`)
- Planar is not already included (`!includedMode[PLANAR_IDX]`)
- Planar is not disabled (`!planarDisable` - AL0125 only)

#### 3. Default Mode List

The `fillMPMList()` function uses a default mode array (lines 40232-40240):

```cpp
#if JVET_AK0059_MDIP
  uint8_t mpmDefault[] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, HOR_IDX - 4, HOR_IDX + 4, ...};
#else
  uint8_t mpmDefault[] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, 14, 22, 42, 58, 10, 26, ...};
#endif
```

**Critical Issue:** `PLANAR_IDX` is **NOT** in the default mode array!

### Scenarios Where Planar Mode Can Be Excluded

#### Scenario 1: PDP Disabled + Planar Not from Spatial Candidates

**Conditions:**
- `pdpRefAvailable == false`
- `planarDisable == false` (AL0125)
- Spatial candidates from neighbors don't include planar mode
- All spatial candidates fill the MPM list to capacity

**Result:** 
1. Planar is inserted at position 0 initially ✓
2. This scenario is safe

#### Scenario 2: PDP Enabled + Spatial Candidates Fill List

**Conditions:**
- `pdpRefAvailable == true`
- Spatial candidates completely fill the MPM list
- None of the spatial candidates is planar mode

**Result:**
1. Initial planar insertion is skipped ✗
2. Spatial candidates fill all positions
3. Safeguard mechanism activates and adds planar ✓
4. This scenario is safe due to safeguard

#### Scenario 3: PDP Disabled + Planar Disabled (AL0125)

**Conditions:**
- `pdpRefAvailable == false`
- `planarDisable == true` (e.g., when PNN mode is active)

**Result:**
1. Initial planar insertion is skipped ✗
2. Safeguard doesn't apply (only for PDP enabled case) ✗
3. Default modes don't include planar ✗
4. **Planar mode can be completely excluded!**

#### Scenario 4: All Spatial Candidates + Angular Derivations Fill List

**Conditions:**
- Large number of valid spatial candidates
- Angular mode derivations from spatial candidates
- Default modes start from position beyond list capacity

**Result:**
1. If planar isn't in spatial candidates
2. If angular derivations fill remaining positions
3. Default modes never get processed
4. **Planar mode can be excluded!**

### Code Evidence

The `fillMPMList()` function has this check at the end (line 40251):
```cpp
CHECK(idx != maxCands, "");
```

This confirms that the list must be completely filled, and if planar mode hasn't been added by the time the list is full, it will be excluded.

### Critical Gap in Implementation

The safeguard mechanism in `fillMPMList()` has a logical gap:

```cpp
#if JVET_AK0061_PDP_MPM 
  if (pdpRefAvailable && !includedMode[PLANAR_IDX]) 
  {
    // Add planar
  }
#endif
```

**Problem:** This safeguard only applies when `pdpRefAvailable == true`. When PDP is not available, there's no equivalent safeguard to ensure planar inclusion if it was missed during spatial candidate derivation.

## Conclusion

**Planar mode is NOT guaranteed to be in the MPM list.** 

The key failure scenarios are:
1. **When `planarDisable == true`** (AL0125 feature) and PDP is not available
2. **When spatial candidates and their angular derivations completely fill the MPM list** before default modes are processed
3. **The absence of a universal safeguard** that works regardless of PDP availability

### Recommendation

To guarantee planar mode inclusion, the code should have a universal safeguard like:

```cpp
// Universal planar safeguard (not currently implemented)
if (!planarDisable && !includedMode[PLANAR_IDX] && idx < maxCands) 
{
  includedMode[PLANAR_IDX] = true;
  mpm[idx++] = PLANAR_IDX;
}
```

This would ensure planar mode is always available for directional planar prediction when not explicitly disabled.