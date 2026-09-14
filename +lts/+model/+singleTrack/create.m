function model = create(vehicle)
%CREATE Build the Level 1.1 single-track performance interface.
% Two identical tires represent each axle. Longitudinal load transfer is
% solved consistently with the requested acceleration.

lts.validation.validateVehicle(vehicle);
model.name = 'Level 1.1 Single Track';
model.level = '1.1';
model.vehicle = vehicle;
model.maximumSpeedForCurvature = ...
    @(curvature_1pm) maximumSpeedForCurvature(vehicle, curvature_1pm);
model.accelerationLimits = ...
    @(speed_mps, lateralAcceleration_mps2) ...
    accelerationLimits(vehicle, speed_mps, lateralAcceleration_mps2);
model.operatingPoint = @(longitudinalAcceleration_mps2, ...
    lateralAcceleration_mps2) lts.model.singleTrack.operatingPoint(...
    vehicle, longitudinalAcceleration_mps2, lateralAcceleration_mps2);
end

function maximumSpeed_mps = maximumSpeedForCurvature(vehicle, curvature_1pm)
maximumLateralAcceleration_mps2 = findMaximumMagnitude(vehicle, 0, 'lateral');
maximumSpeed_mps = Inf(size(curvature_1pm));
corner = abs(curvature_1pm) > eps;
maximumSpeed_mps(corner) = sqrt(maximumLateralAcceleration_mps2 ...
    ./ abs(curvature_1pm(corner)));
end

function limits = accelerationLimits(vehicle, speed_mps, lateralAcceleration_mps2)
validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'real', 'finite', 'size', size(speed_mps)});

feasible = lts.model.singleTrack.feasibleMask(vehicle, ...
    zeros(size(lateralAcceleration_mps2)), lateralAcceleration_mps2);
maximumAcceleration_mps2 = findMaximumMagnitude(...
    vehicle, lateralAcceleration_mps2, 'acceleration');
maximumDeceleration_mps2 = findMaximumMagnitude(...
    vehicle, lateralAcceleration_mps2, 'braking');
maximumAcceleration_mps2(~feasible) = 0;
maximumDeceleration_mps2(~feasible) = 0;
limits.maximumAcceleration_mps2 = maximumAcceleration_mps2;
limits.maximumDeceleration_mps2 = maximumDeceleration_mps2;
limits.feasible = feasible;
end

function maximumMagnitude_mps2 = findMaximumMagnitude(vehicle, fixedValue, direction)
gravity_mps2 = 9.81;
lower = zeros(size(fixedValue));
upper = 5 * gravity_mps2 * ones(size(fixedValue));
for iteration = 1:35
    candidate = (lower + upper) / 2;
    switch direction
        case 'lateral'
            feasible = lts.model.singleTrack.feasibleMask(vehicle, ...
                zeros(size(candidate)), candidate);
        case 'acceleration'
            feasible = lts.model.singleTrack.feasibleMask(vehicle, candidate, fixedValue);
        case 'braking'
            feasible = lts.model.singleTrack.feasibleMask(vehicle, -candidate, fixedValue);
        otherwise
            error('lts:model:Direction', 'Unknown search direction: %s.', direction);
    end
    lower(feasible) = candidate(feasible);
    upper(~feasible) = candidate(~feasible);
end
maximumMagnitude_mps2 = lower;
end
