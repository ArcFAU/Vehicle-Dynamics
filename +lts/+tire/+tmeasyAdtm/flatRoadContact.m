function contact = flatRoadContact(tir, requestedNormalLoad_N, camberAngle_rad)
%FLATROADCONTACT Reproduce the ADTM flat-road contact geometry for QSS use.

b2_m = tir.b_0 / 2;
staticRadius_m = max(0, tir.r_0 - requestedNormalLoad_N / tir.c_z);
leftRadius_m = staticRadius_m + b2_m * tan(camberAngle_rad);
rightRadius_m = staticRadius_m - b2_m * tan(camberAngle_rad);

if leftRadius_m < tir.r_0 && rightRadius_m < tir.r_0
    rectangularDeflection_m = tir.r_0 - staticRadius_m;
    rectangularWidth_m = tir.b_0 / cos(camberAngle_rad);
elseif leftRadius_m >= tir.r_0 && rightRadius_m < tir.r_0 ...
        && camberAngle_rad ~= 0
    rectangularDeflection_m = (tir.r_0 - rightRadius_m) ^ 2 ...
        / (2 * tir.b_0 * tan(abs(camberAngle_rad)));
    rectangularWidth_m = (tir.r_0 - rightRadius_m) ...
        / tan(abs(camberAngle_rad));
elseif leftRadius_m < tir.r_0 && rightRadius_m >= tir.r_0 ...
        && camberAngle_rad ~= 0
    rectangularDeflection_m = (tir.r_0 - leftRadius_m) ^ 2 ...
        / (2 * tir.b_0 * tan(abs(camberAngle_rad)));
    rectangularWidth_m = (tir.r_0 - leftRadius_m) ...
        / tan(abs(camberAngle_rad));
else
    rectangularDeflection_m = 0;
    rectangularWidth_m = tir.b_0;
    camberAngle_rad = 0;
end

crossSectionRadius_m = b2_m;
circularDeflection_m = crossSectionRadius_m ...
    - (staticRadius_m - (tir.r_0 - crossSectionRadius_m)) ...
    * cos(camberAngle_rad);
radicand_m2 = max(0, crossSectionRadius_m ^ 2 ...
    - (crossSectionRadius_m - circularDeflection_m) ^ 2);
circularWidth_m = 2 * sqrt(radicand_m2);

contact.staticRadius_m = staticRadius_m;
contact.deflection_m = (1 - tir.R_N) * rectangularDeflection_m ...
    + tir.R_N * circularDeflection_m;
contact.width_m = (1 - tir.R_N) * rectangularWidth_m ...
    + tir.R_N * circularWidth_m;
contact.camberAngle_rad = camberAngle_rad;
end
