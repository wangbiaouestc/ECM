  const PreCalcValues &pcv = *cu.cs->pcv;
  const Area &area       = cu.Y();

  const bool isOnBoundary = (area.x == 0) || (area.y == 0) ||
                            (area.x + area.width  >= pcv.lumaWidth) ||
                            (area.y + area.height >= pcv.lumaHeight);

  if (!isOnBoundary && pu->multiRefIdx == 0 && cu.ispMode == 0 && cu.mipFlag == 0 && cu.dimd == 0 &&
      cu.timd == 0 && cu.tmpFlag == 0 && cu.tmrlFlag == 0 && cu.sgpm == 0 && isLuma(cu.chType))

