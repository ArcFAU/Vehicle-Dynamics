function classification = limitClassification(lap, model)
%LIMITCLASSIFICATION Classify the active local performance limitation.

segmentCount = numel(lap.segmentAcceleration_mps2);
pointIndex = (1:segmentCount)';
braking = lap.segmentAcceleration_mps2 < 0;
pointIndex(braking) = pointIndex(braking) + 1;
speed_mps = lap.speed_mps(pointIndex);
ay_mps2 = lap.lateralAcceleration_mps2(pointIndex);
ax_mps2 = lap.segmentAcceleration_mps2(:);
limits = model.accelerationLimits(speed_mps, ay_mps2);
accelerationUsage = max(ax_mps2, 0) ...
    ./ max(limits.maximumAcceleration_mps2(:), eps);
brakingUsage = max(-ax_mps2, 0) ...
    ./ max(limits.maximumDeceleration_mps2(:), eps);
curveLimit_mps = model.maximumSpeedForCurvature( ...
    lap.curvature_1pm(pointIndex));
lateralUsage = speed_mps ./ max(curveLimit_mps(:), eps);

primary = repmat("reserve", segmentCount, 1);
primary(braking & brakingUsage >= 0.95) = "braking";
primary(~braking & lateralUsage >= 0.995) = "tire-lateral";
primary(~braking & accelerationUsage >= 0.95) = "traction";

if isfield(model, 'vehicle')
    vehicle = model.vehicle;
    drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, speed_mps);
    aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
    requiredDriveForce_N = max(0, vehicle.mass.total_kg .* ax_mps2 ...
        + aero.drag_N);
    driveLimited = ~braking & accelerationUsage >= 0.95 ...
        & requiredDriveForce_N >= 0.97 .* drive.maximumForce_N(:);
    torqueLimited = driveLimited ...
        & drive.torqueLimitedForce_N(:) <= drive.powerLimitedForce_N(:);
    powerLimited = driveLimited & ~torqueLimited;
    primary(torqueLimited) = "torque";
    primary(powerLimited) = "power";

    if strcmp(string(model.level), "1.2")
        peakEnvelope = model.peakEnvelope;
        state = lts.model.singleTrackAero.calculateState(vehicle, ...
            speed_mps, ax_mps2, ay_mps2, peakEnvelope);
        tireLimited = max(state.frontUtilization(:), ...
            state.rearUtilization(:)) >= 0.95;
        front = tireLimited ...
            & state.frontUtilization(:) >= state.rearUtilization(:);
        rear = tireLimited & ~front;
        primary(front) = "tire-front";
        primary(rear) = "tire-rear";
    end
end

classification.segmentMidpoint_m = ...
    (lap.distance_m(1:end-1) + lap.distance_m(2:end)) / 2;
classification.primary = primary;
classification.accelerationUsage = accelerationUsage;
classification.brakingUsage = brakingUsage;
classification.lateralUsage = lateralUsage;
classification.categories = unique(primary, 'stable');
end
