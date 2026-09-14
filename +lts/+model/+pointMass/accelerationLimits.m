function limits = accelerationLimits(vehicle, speed_mps, lateralAcceleration_mps2)
%ACCELERATIONLIMITS Constant-friction point-mass grip-circle limits.
% speed_mps is part of the common interface and becomes active with aero,
% drag and power in Level 0.3.

validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'real', 'finite', 'size', size(speed_mps)});

gravity_mps2 = 9.81;
totalAccelerationLimit_mps2 = ...
    vehicle.tireApproximation.frictionCoefficient * gravity_mps2;
remainingSquared_m2ps4 = max(0, totalAccelerationLimit_mps2^2 ...
    - lateralAcceleration_mps2.^2);
longitudinalLimit_mps2 = sqrt(remainingSquared_m2ps4);

limits.maximumAcceleration_mps2 = longitudinalLimit_mps2;
limits.maximumDeceleration_mps2 = longitudinalLimit_mps2;
limits.feasible = abs(lateralAcceleration_mps2) ...
    <= totalAccelerationLimit_mps2 * (1 + 1e-12);
end
