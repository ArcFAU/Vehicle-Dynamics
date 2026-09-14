function model = create(vehicle)
%CREATE Build Level 0.2 with four equally loaded envelope tires.

lts.validation.validateVehicle(vehicle);
model.name = 'Level 0.2 Load-Sensitive Point Mass';
model.level = '0.2';
model.vehicle = vehicle;
model.maximumSpeedForCurvature = ...
    @(curvature_1pm) maximumSpeedForCurvature(vehicle, curvature_1pm);
model.accelerationLimits = ...
    @(speed_mps, lateralAcceleration_mps2) ...
    accelerationLimits(vehicle, speed_mps, lateralAcceleration_mps2);
end

function maximumSpeed_mps = maximumSpeedForCurvature(vehicle, curvature_1pm)
maximumLateralAcceleration_mps2 = totalAccelerationLimit(vehicle);
maximumSpeed_mps = Inf(size(curvature_1pm));
corner = abs(curvature_1pm) > eps;
maximumSpeed_mps(corner) = sqrt(maximumLateralAcceleration_mps2 ...
    ./ abs(curvature_1pm(corner)));
end

function limits = accelerationLimits(vehicle, speed_mps, lateralAcceleration_mps2)
validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'real', 'finite', 'size', size(speed_mps)});
accelerationLimit_mps2 = totalAccelerationLimit(vehicle);
exponent = vehicle.tireEnvelope.combinedForceExponent;
lateralFraction = abs(lateralAcceleration_mps2) ./ accelerationLimit_mps2;
remainingFraction = max(0, 1 - lateralFraction .^ exponent);
remainingFraction(lateralFraction >= 1 - 1e-12) = 0;
longitudinalLimit_mps2 = accelerationLimit_mps2 ...
    .* remainingFraction .^ (1 / exponent);
limits.maximumAcceleration_mps2 = longitudinalLimit_mps2;
limits.maximumDeceleration_mps2 = longitudinalLimit_mps2;
limits.feasible = lateralFraction <= 1 + 1e-12;
end

function accelerationLimit_mps2 = totalAccelerationLimit(vehicle)
gravity_mps2 = 9.81;
normalLoadPerTire_N = vehicle.mass.total_kg * gravity_mps2 / 4;
totalTireForce_N = 4 * lts.tire.maximumForce(...
    vehicle.tireEnvelope, normalLoadPerTire_N);
accelerationLimit_mps2 = totalTireForce_N / vehicle.mass.total_kg;
end
