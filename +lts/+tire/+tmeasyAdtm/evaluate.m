function forces = evaluate(tir, normalLoad_N, camberAngle_rad, ...
        slipRatio, slipAngle_rad, speed_mps, muScale)
%EVALUATE Stationary ADTM TMeasy operating point for the QSS solver.

arguments
    tir (1, 1) struct
    normalLoad_N (1, 1) double {mustBePositive, mustBeFinite}
    camberAngle_rad (1, 1) double {mustBeFinite}
    slipRatio (1, 1) double {mustBeFinite, mustBeLessThan(slipRatio, 0.95)}
    slipAngle_rad (1, 1) double {mustBeFinite}
    speed_mps (1, 1) double {mustBePositive, mustBeFinite}
    muScale (1, 1) double {mustBePositive, mustBeFinite} = 1
end

contact = lts.tire.tmeasyAdtm.flatRoadContact(tir, normalLoad_N, ...
    camberAngle_rad);
transportSpeed_mps = (speed_mps + slipRatio * tir.v_n) ...
    / (1 - slipRatio);
dynamicRadius_m = 2 / 3 * tir.r_0 + 1 / 3 * contact.staticRadius_m;
wheelSpeed_radps = transportSpeed_mps / max(dynamicRadius_m, eps);
grownRadius_m = tir.r_0 + tir.dr_s * wheelSpeed_radps ^ 2;
dynamicRadius_m = 2 / 3 * grownRadius_m ...
    + 1 / 3 * contact.staticRadius_m;
wheelSpeed_radps = transportSpeed_mps / max(dynamicRadius_m, eps);
transportSpeedOffset_mps = abs(transportSpeed_mps) + tir.v_n;
lateralVelocity_mps = -tan(slipAngle_rad) ...
    * transportSpeedOffset_mps;

[forceVector_N, torqueVector_Nm, actualSlip, ~, dynamicRadius_m, ...
    camberForce_N, dFx0_N, FxM_N, dFy0_N, FyM_N] = ...
    lts.tire.tmeasyAdtm.core(tir, 0, ...
    [speed_mps; lateralVelocity_mps; 0], ...
    [0; wheelSpeed_radps; 0], contact.staticRadius_m, ...
    contact.deflection_m, contact.width_m, contact.camberAngle_rad, ...
    muScale, [0; 0; 0], false);

forces.longitudinal_N = forceVector_N(1);
forces.lateral_N = forceVector_N(2);
forces.normal_N = forceVector_N(3);
forces.overturningMoment_Nm = torqueVector_Nm(1);
forces.rollingResistanceMoment_Nm = torqueVector_Nm(2);
forces.aligningMoment_Nm = torqueVector_Nm(3);
forces.dynamicRadius_m = dynamicRadius_m;
forces.actualSlipRatio = actualSlip(1);
forces.actualLateralSlip = actualSlip(2);
forces.actualCamberSlip = actualSlip(3);
forces.actualBoreSlip = actualSlip(4);
forces.camberForce_N = camberForce_N;
forces.dFx0_N = dFx0_N;
forces.FxM_N = FxM_N;
forces.dFy0_N = dFy0_N;
forces.FyM_N = FyM_N;
forces.utilization = hypot(forces.longitudinal_N / max(abs(FxM_N), eps), ...
    forces.lateral_N / max(abs(FyM_N), eps));
forces.unconstrainedUtilization = forces.utilization;
slipVelocityX_mps = speed_mps - transportSpeed_mps;
forces.slipPower_W = max(0, -(forces.longitudinal_N * slipVelocityX_mps ...
    + forces.lateral_N * lateralVelocity_mps));
end
