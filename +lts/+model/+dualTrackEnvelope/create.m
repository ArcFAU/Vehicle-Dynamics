function model = create(vehicle)
%CREATE Build Level 2.0 four-wheel force-envelope model.

lts.validation.validateVehicle(vehicle);
peakEnvelope = lts.tire.createPeakEnvelope(vehicle);
model.name = 'Level 2.0 Dual Track Force Envelope';
model.level = '2.0';
model.vehicle = vehicle;
model.peakEnvelope = peakEnvelope;
model.maximumSpeedForCurvature = ...
    @(curvature_1pm) maximumSpeedForCurvature(vehicle, peakEnvelope, ...
    curvature_1pm);
model.accelerationLimits = ...
    @(speed_mps, lateralAcceleration_mps2) ...
    accelerationLimits(vehicle, peakEnvelope, speed_mps, ...
    lateralAcceleration_mps2);
model.operatingPoint = @(speed_mps, longitudinalAcceleration_mps2, ...
    lateralAcceleration_mps2) lts.model.dualTrackEnvelope.operatingPoint(...
    vehicle, speed_mps, longitudinalAcceleration_mps2, ...
    lateralAcceleration_mps2, peakEnvelope);
end

function maximumSpeed_mps = maximumSpeedForCurvature(vehicle, peakEnvelope, curvature_1pm)
query = @(speed, ay) lts.model.dualTrackEnvelope.feasibleMask( ...
    vehicle, speed, zeros(size(speed)), ay, peakEnvelope);
maximumSpeed_mps = lts.model.maximumSteadySpeed( ...
    vehicle, curvature_1pm, query, 45);
end

function limits = accelerationLimits(vehicle, peakEnvelope, speed_mps, ...
        lateralAcceleration_mps2)
validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'real', 'finite', 'size', size(speed_mps)});
baseFeasible = lts.model.dualTrackEnvelope.feasibleMask(vehicle, speed_mps, ...
    zeros(size(speed_mps)), lateralAcceleration_mps2, peakEnvelope);
limits.maximumAcceleration_mps2 = findLimit(vehicle, speed_mps, ...
    lateralAcceleration_mps2, 1, peakEnvelope);
limits.maximumDeceleration_mps2 = findLimit(vehicle, speed_mps, ...
    lateralAcceleration_mps2, -1, peakEnvelope);
limits.maximumAcceleration_mps2(~baseFeasible) = 0;
limits.maximumDeceleration_mps2(~baseFeasible) = 0;
limits.feasible = baseFeasible;
end

function magnitude_mps2 = findLimit(vehicle, speed_mps, ...
        lateralAcceleration_mps2, signAx, peakEnvelope)
lower = zeros(size(speed_mps));
upper = 5 * 9.81 * ones(size(speed_mps));
for iteration = 1:35
    candidate = (lower + upper) / 2;
    feasible = lts.model.dualTrackEnvelope.feasibleMask(vehicle, speed_mps, ...
        signAx * candidate, lateralAcceleration_mps2, peakEnvelope);
    lower(feasible) = candidate(feasible);
    upper(~feasible) = candidate(~feasible);
end
magnitude_mps2 = lower;
end
