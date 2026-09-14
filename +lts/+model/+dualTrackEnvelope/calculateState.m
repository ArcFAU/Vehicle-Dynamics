function state = calculateState(vehicle, speed_mps, longitudinalAcceleration_mps2, lateralAcceleration_mps2)
%CALCULATESTATE Four-wheel normal loads and force-envelope allocation.
% Wheel order is FL, FR, RL, RR. Positive ay is a left turn, therefore the
% right wheels are outside and gain normal load.

mass_kg = vehicle.mass.total_kg;
frontFraction = vehicle.geometry.frontMassFraction;
rearFraction = 1 - frontFraction;
wheelbase_m = vehicle.geometry.wheelbase_m;
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);

longitudinalTransfer_N = (mass_kg .* longitudinalAcceleration_mps2 ...
    .* vehicle.geometry.cgHeight_m + aero.drag_N ...
    .* vehicle.aerodynamics.dragApplicationHeight_m) / wheelbase_m;
frontAxleLoad_N = mass_kg * 9.81 * frontFraction ...
    + aero.frontDownforce_N - longitudinalTransfer_N;
rearAxleLoad_N = mass_kg * 9.81 * rearFraction ...
    + aero.rearDownforce_N + longitudinalTransfer_N;

totalRollMoment_Nm = mass_kg .* lateralAcceleration_mps2 ...
    .* vehicle.geometry.cgHeight_m;
if isfield(vehicle, 'modelOptions') ...
        && isfield(vehicle.modelOptions, 'lateralLoadTransferScale')
    totalRollMoment_Nm = totalRollMoment_Nm ...
        .* vehicle.modelOptions.lateralLoadTransferScale;
end
frontTransferPerSide_N = totalRollMoment_Nm ...
    .* vehicle.suspension.frontRollMomentFraction ...
    / vehicle.geometry.frontTrack_m;
rearTransferPerSide_N = totalRollMoment_Nm ...
    .* (1 - vehicle.suspension.frontRollMomentFraction) ...
    / vehicle.geometry.rearTrack_m;
state.normalLoads_N = [frontAxleLoad_N / 2 - frontTransferPerSide_N, ...
    frontAxleLoad_N / 2 + frontTransferPerSide_N, ...
    rearAxleLoad_N / 2 - rearTransferPerSide_N, ...
    rearAxleLoad_N / 2 + rearTransferPerSide_N];

frontLateralForce_N = mass_kg .* lateralAcceleration_mps2 * frontFraction;
rearLateralForce_N = mass_kg .* lateralAcceleration_mps2 * rearFraction;
pureForceCapacity_N = lts.tire.maximumForce(vehicle.tireEnvelope, ...
    max(state.normalLoads_N, 0));
frontCapacitySum_N = pureForceCapacity_N(:, 1) + pureForceCapacity_N(:, 2);
rearCapacitySum_N = pureForceCapacity_N(:, 3) + pureForceCapacity_N(:, 4);
frontWeights = safeWeights(pureForceCapacity_N(:, 1:2), frontCapacitySum_N);
rearWeights = safeWeights(pureForceCapacity_N(:, 3:4), rearCapacitySum_N);
state.lateralForces_N = [frontWeights .* frontLateralForce_N, ...
    rearWeights .* rearLateralForce_N];

requiredTireForce_N = mass_kg .* longitudinalAcceleration_mps2 + aero.drag_N;
useIdealDrive = strcmp(vehicle.controls.driveAllocation.mode, 'ideal');
useIdealBrake = strcmp(vehicle.controls.brakeAllocation.mode, 'ideal');
if useIdealDrive || useIdealBrake
    longitudinalCapacities_N = lts.tire.combinedLongitudinalCapacity( ...
        vehicle.tireEnvelope, max(state.normalLoads_N, 0), ...
        state.lateralForces_N);
    state.longitudinalForces_N = zeros(size(longitudinalCapacities_N));
    allocationFeasible = false(size(requiredTireForce_N));
    allocations = cell(numel(requiredTireForce_N), 1);
    for row = 1:numel(requiredTireForce_N)
        operation = "drive";
        if requiredTireForce_N(row) < 0
            operation = "brake";
        end
        allocation = lts.control.allocateLongitudinalForces(vehicle, ...
            speed_mps(row), longitudinalCapacities_N(row, :), ...
            state.lateralForces_N(row, :), requiredTireForce_N(row), ...
            operation);
        state.longitudinalForces_N(row, :) = allocation.forces_N;
        allocationFeasible(row) = allocation.feasible;
        allocations{row} = allocation;
    end
    state.allocation = allocations;
else
    frontFractionFx = vehicle.powertrain.frontDriveForceFraction ...
        * ones(size(requiredTireForce_N));
    braking = longitudinalAcceleration_mps2 < 0;
    frontFractionFx(braking) = vehicle.brakes.frontForceFraction;
    frontLongitudinalForce_N = requiredTireForce_N .* frontFractionFx;
    rearLongitudinalForce_N = requiredTireForce_N ...
        .* (1 - frontFractionFx);
    state.longitudinalForces_N = [frontLongitudinalForce_N / 2, ...
        frontLongitudinalForce_N / 2, rearLongitudinalForce_N / 2, ...
        rearLongitudinalForce_N / 2];
    allocationFeasible = true(size(requiredTireForce_N));
end

state.tireUtilization = lts.tire.utilization(vehicle.tireEnvelope, ...
    max(state.normalLoads_N, 0), state.longitudinalForces_N, ...
    state.lateralForces_N);
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, speed_mps);
state.aero = aero;
state.driveForceRequired_N = max(requiredTireForce_N, 0);
state.driveForceAvailable_N = drive.maximumForce_N;
state.feasible = all(state.normalLoads_N > 0, 2) ...
    & all(state.tireUtilization <= 1 + 1e-12, 2) ...
    & allocationFeasible ...
    & state.driveForceRequired_N <= state.driveForceAvailable_N + 1e-9;
end

function weights = safeWeights(capacities_N, capacitySum_N)
weights = zeros(size(capacities_N));
loaded = capacitySum_N > 0;
weights(loaded, :) = capacities_N(loaded, :) ./ capacitySum_N(loaded);
end
