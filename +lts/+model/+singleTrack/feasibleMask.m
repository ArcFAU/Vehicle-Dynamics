function feasible = feasibleMask(vehicle, longitudinalAcceleration_mps2, lateralAcceleration_mps2)
%FEASIBLEMASK Fast internal feasibility evaluation for boundary searches.
% Inputs are validated at the public model interface. This function keeps
% only the equations required by the iterative limit search.

mass_kg = vehicle.mass.total_kg;
gravity_mps2 = 9.81;
frontFraction = vehicle.geometry.frontMassFraction;
loadTransferPerAcceleration_kg = ...
    mass_kg * vehicle.geometry.cgHeight_m / vehicle.geometry.wheelbase_m;
frontNormalLoad_N = mass_kg * gravity_mps2 * frontFraction ...
    - loadTransferPerAcceleration_kg .* longitudinalAcceleration_mps2;
rearNormalLoad_N = mass_kg * gravity_mps2 * (1 - frontFraction) ...
    + loadTransferPerAcceleration_kg .* longitudinalAcceleration_mps2;

frontLateralForce_N = mass_kg .* lateralAcceleration_mps2 * frontFraction;
rearLateralForce_N = mass_kg .* lateralAcceleration_mps2 * (1 - frontFraction);
frontLongitudinalFraction = vehicle.powertrain.frontDriveForceFraction ...
    * ones(size(longitudinalAcceleration_mps2));
braking = longitudinalAcceleration_mps2 < 0;
frontLongitudinalFraction(braking) = vehicle.brakes.frontForceFraction;
totalLongitudinalForce_N = mass_kg .* longitudinalAcceleration_mps2;
frontLongitudinalForce_N = totalLongitudinalForce_N ...
    .* frontLongitudinalFraction;
rearLongitudinalForce_N = totalLongitudinalForce_N ...
    .* (1 - frontLongitudinalFraction);

tire = vehicle.tireEnvelope;
frontCapacity_N = axleCapacity(tire, frontNormalLoad_N);
rearCapacity_N = axleCapacity(tire, rearNormalLoad_N);
exponent = tire.combinedForceExponent;
frontUtilization = ((abs(frontLongitudinalForce_N) ./ frontCapacity_N) ...
    .^ exponent + (abs(frontLateralForce_N) ./ frontCapacity_N) ...
    .^ exponent) .^ (1 / exponent);
rearUtilization = ((abs(rearLongitudinalForce_N) ./ rearCapacity_N) ...
    .^ exponent + (abs(rearLateralForce_N) ./ rearCapacity_N) ...
    .^ exponent) .^ (1 / exponent);

feasible = frontNormalLoad_N > 0 & rearNormalLoad_N > 0 ...
    & frontUtilization <= 1 + 1e-12 & rearUtilization <= 1 + 1e-12;
end

function capacity_N = axleCapacity(tire, axleNormalLoad_N)
capacity_N = zeros(size(axleNormalLoad_N));
loaded = axleNormalLoad_N > 0;
loadRatio = axleNormalLoad_N(loaded) ./ (2 * tire.referenceNormalLoad_N);
frictionCoefficient = tire.frictionCoefficientAtReferenceLoad ...
    .* loadRatio .^ tire.loadSensitivityExponent;
capacity_N(loaded) = frictionCoefficient .* axleNormalLoad_N(loaded);
end
