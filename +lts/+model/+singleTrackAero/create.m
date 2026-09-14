function model = create(vehicle)
%CREATE Build Level 1.2 single track with aero and powertrain limits.

lts.validation.validateVehicle(vehicle);
peakEnvelope = lts.tire.createPeakEnvelope(vehicle);
model.name = 'Level 1.2 Single Track with Aero and Power';
model.level = '1.2';
model.vehicle = vehicle;
model.peakEnvelope = peakEnvelope;
model.maximumSpeedForCurvature = ...
    @(curvature_1pm) maximumSpeedForCurvature(vehicle, peakEnvelope, curvature_1pm);
model.accelerationLimits = ...
    @(speed_mps, lateralAcceleration_mps2) ...
    accelerationLimits(vehicle, peakEnvelope, speed_mps, lateralAcceleration_mps2);
model.operatingPoint = @(speed_mps, longitudinalAcceleration_mps2, ...
    lateralAcceleration_mps2) lts.model.singleTrackAero.operatingPoint(...
    vehicle, speed_mps, longitudinalAcceleration_mps2, ...
    lateralAcceleration_mps2);
end

function maximumSpeed_mps = maximumSpeedForCurvature(vehicle, peakEnvelope, curvature_1pm)
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 0);
lowerSpeed_mps = zeros(size(curvature_1pm));
upperSpeed_mps = drive.maximumVehicleSpeed_mps * ones(size(curvature_1pm));
for iteration = 1:28
    candidateSpeed_mps = (lowerSpeed_mps + upperSpeed_mps) / 2;
    lateralAcceleration_mps2 = candidateSpeed_mps .^ 2 .* curvature_1pm;
    feasible = lts.model.singleTrackAero.feasibleMask(vehicle, ...
        candidateSpeed_mps, zeros(size(candidateSpeed_mps)), ...
        lateralAcceleration_mps2, peakEnvelope);
    lowerSpeed_mps(feasible) = candidateSpeed_mps(feasible);
    upperSpeed_mps(~feasible) = candidateSpeed_mps(~feasible);
end
maximumSpeed_mps = lowerSpeed_mps;
end

function limits = accelerationLimits(vehicle, peakEnvelope, speed_mps, lateralAcceleration_mps2)
validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'real', 'finite', 'size', size(speed_mps)});
baseFeasible = lts.model.singleTrackAero.feasibleMask(vehicle, speed_mps, ...
    zeros(size(speed_mps)), lateralAcceleration_mps2, peakEnvelope);
limits.maximumAcceleration_mps2 = findLimit(vehicle, speed_mps, ...
    lateralAcceleration_mps2, 1, peakEnvelope);
limits.maximumDeceleration_mps2 = findLimit(vehicle, speed_mps, ...
    lateralAcceleration_mps2, -1, peakEnvelope);
limits.maximumAcceleration_mps2(~baseFeasible) = 0;
limits.maximumDeceleration_mps2(~baseFeasible) = 0;
limits.feasible = baseFeasible;
end

function magnitude_mps2 = findLimit(vehicle, speed_mps, lateralAcceleration_mps2, signAx, peakEnvelope)
lower = zeros(size(speed_mps));
upper = 5 * 9.81 * ones(size(speed_mps));
for iteration = 1:12
    candidate = (lower + upper) / 2;
    feasible = lts.model.singleTrackAero.feasibleMask(vehicle, speed_mps, ...
        signAx * candidate, lateralAcceleration_mps2, peakEnvelope);
    lower(feasible) = candidate(feasible);
    upper(~feasible) = candidate(~feasible);
end
magnitude_mps2 = lower;
end
