function model = create(vehicle, options)
%CREATE Build Level 0.3 point mass with tire, aero and power limits.

arguments
    vehicle (1, 1) struct
    options.UseAxleLoads (1, 1) logical = false
end

lts.validation.validateVehicle(vehicle);
peakEnvelope = lts.tire.createPeakEnvelope(vehicle);
model.name = 'Level 0.3 Point Mass with Aero and Power';
model.level = '0.3';
model.vehicle = vehicle;
model.peakEnvelope = peakEnvelope;
model.usesAxleLoads = options.UseAxleLoads;
model.maximumSpeedForCurvature = ...
    @(curvature_1pm) maximumSpeedForCurvature(vehicle, peakEnvelope, curvature_1pm);
model.accelerationLimits = ...
    @(speed_mps, lateralAcceleration_mps2) ...
    accelerationLimits(vehicle, peakEnvelope, speed_mps, ...
    lateralAcceleration_mps2, options.UseAxleLoads);
end

function maximumSpeed_mps = maximumSpeedForCurvature(vehicle, peakEnvelope, curvature_1pm)
curvatureMagnitude_1pm = abs(curvature_1pm);
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 0);
upperSpeed_mps = drive.maximumVehicleSpeed_mps * ones(size(curvature_1pm));
lowerSpeed_mps = zeros(size(curvature_1pm));
for iteration = 1:28
    candidateSpeed_mps = (lowerSpeed_mps + upperSpeed_mps) / 2;
    lateralAcceleration_mps2 = candidateSpeed_mps .^ 2 ...
        .* curvatureMagnitude_1pm;
    feasible = steadySpeedFeasible(vehicle, peakEnvelope, candidateSpeed_mps, ...
        lateralAcceleration_mps2);
    lowerSpeed_mps(feasible) = candidateSpeed_mps(feasible);
    upperSpeed_mps(~feasible) = candidateSpeed_mps(~feasible);
end
maximumSpeed_mps = lowerSpeed_mps;
end

function limits = accelerationLimits(vehicle, peakEnvelope, speed_mps, ...
    lateralAcceleration_mps2, useAxleLoads)
validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'real', 'finite', 'size', size(speed_mps)});
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
normalLoadPerTire_N = (vehicle.mass.total_kg * 9.81 ...
    + aero.totalDownforce_N) / 4;
lateralForcePerTire_N = vehicle.mass.total_kg ...
    .* lateralAcceleration_mps2 / 4;
if useAxleLoads
    driveTireCapacity_N = findAxleDriveCapacity(vehicle, peakEnvelope, ...
        speed_mps, lateralAcceleration_mps2);
    brakeTireCapacity_N = findAxleBrakeCapacity(vehicle, peakEnvelope, ...
        speed_mps, lateralAcceleration_mps2);
else
    longitudinalCapacityPerTire_N = ...
        lts.tire.directionalLongitudinalCapacity(peakEnvelope, ...
        normalLoadPerTire_N, lateralForcePerTire_N);
    driveTireCapacity_N = min(4, vehicle.powertrain.drivenWheelCount) ...
        .* longitudinalCapacityPerTire_N;
    brakeTireCapacity_N = 4 .* longitudinalCapacityPerTire_N;
end
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, speed_mps);
availableDriveForce_N = min(driveTireCapacity_N, ...
    drive.maximumForce_N);

limits.maximumAcceleration_mps2 = max(0, ...
    (availableDriveForce_N - aero.drag_N) / vehicle.mass.total_kg);
limits.maximumDeceleration_mps2 = ...
    (brakeTireCapacity_N + aero.drag_N) / vehicle.mass.total_kg;
limits.feasible = steadySpeedFeasible(vehicle, peakEnvelope, speed_mps, ...
    lateralAcceleration_mps2);
end

function capacity_N = findAxleDriveCapacity(vehicle, peakEnvelope, ...
    speed_mps, lateralAcceleration_mps2)
lower = zeros(size(speed_mps));
upper = 5 * 9.81 * ones(size(speed_mps));
for iteration = 1:12
    candidate = (lower + upper) / 2;
    capacity_N = axleLongitudinalCapacity(vehicle, peakEnvelope, ...
        speed_mps, candidate, lateralAcceleration_mps2, true);
    aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
    required_N = vehicle.mass.total_kg .* candidate + aero.drag_N;
    feasible = required_N <= capacity_N;
    lower(feasible) = candidate(feasible);
    upper(~feasible) = candidate(~feasible);
end
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
capacity_N = vehicle.mass.total_kg .* lower + aero.drag_N;
end

function capacity_N = findAxleBrakeCapacity(vehicle, peakEnvelope, ...
    speed_mps, lateralAcceleration_mps2)
lower = zeros(size(speed_mps));
upper = 5 * 9.81 * ones(size(speed_mps));
for iteration = 1:12
    candidate = (lower + upper) / 2;
    capacity_N = axleLongitudinalCapacity(vehicle, peakEnvelope, ...
        speed_mps, -candidate, lateralAcceleration_mps2, false);
    aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
    required_N = max(0, vehicle.mass.total_kg .* candidate - aero.drag_N);
    feasible = required_N <= capacity_N;
    lower(feasible) = candidate(feasible);
    upper(~feasible) = candidate(~feasible);
end
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
capacity_N = max(0, vehicle.mass.total_kg .* lower - aero.drag_N);
end

function capacity_N = axleLongitudinalCapacity(vehicle, peakEnvelope, ...
    speed_mps, ax_mps2, ay_mps2, driving)
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
mass_kg = vehicle.mass.total_kg;
wheelbase_m = vehicle.geometry.wheelbase_m;
frontFraction = vehicle.geometry.frontMassFraction;
transfer_N = (mass_kg .* ax_mps2 .* vehicle.geometry.cgHeight_m ...
    + aero.drag_N .* vehicle.aerodynamics.dragApplicationHeight_m) ...
    / wheelbase_m;
frontLoad_N = mass_kg * 9.81 * frontFraction ...
    + aero.frontDownforce_N - transfer_N;
rearLoad_N = mass_kg * 9.81 * (1 - frontFraction) ...
    + aero.rearDownforce_N + transfer_N;
lateralPerTire_N = mass_kg .* ay_mps2 / 4;
frontCapacityPerTire_N = lts.tire.directionalLongitudinalCapacity( ...
    peakEnvelope, max(0, frontLoad_N / 2), lateralPerTire_N, 0);
rearCapacityPerTire_N = lts.tire.directionalLongitudinalCapacity( ...
    peakEnvelope, max(0, rearLoad_N / 2), lateralPerTire_N, 0);
if driving
    frontShare = vehicle.powertrain.frontDriveForceFraction;
    if frontShare <= 0
        capacity_N = 2 .* rearCapacityPerTire_N;
    elseif frontShare >= 1
        capacity_N = 2 .* frontCapacityPerTire_N;
    else
        capacity_N = min(2 .* frontCapacityPerTire_N ./ frontShare, ...
            2 .* rearCapacityPerTire_N ./ (1 - frontShare));
    end
else
    frontShare = vehicle.brakes.frontForceFraction;
    capacity_N = min(2 .* frontCapacityPerTire_N ./ max(frontShare, eps), ...
        2 .* rearCapacityPerTire_N ./ max(1 - frontShare, eps));
end
end

function feasible = steadySpeedFeasible(vehicle, peakEnvelope, speed_mps, lateralAcceleration_mps2)
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
normalLoadPerTire_N = (vehicle.mass.total_kg * 9.81 ...
    + aero.totalDownforce_N) / 4;
lateralForcePerTire_N = vehicle.mass.total_kg ...
    .* lateralAcceleration_mps2 / 4;
peaks = lts.tire.peakForces(peakEnvelope, normalLoadPerTire_N);
longitudinalCapacityPerTire_N = ...
    lts.tire.directionalLongitudinalCapacity(peakEnvelope, ...
    normalLoadPerTire_N, lateralForcePerTire_N);
driveLongitudinalCapacity_N = min(4, vehicle.powertrain.drivenWheelCount) ...
    .* longitudinalCapacityPerTire_N;
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, speed_mps);
feasible = abs(lateralForcePerTire_N) <= peaks.lateral_N + 1e-9 ...
    & aero.drag_N <= driveLongitudinalCapacity_N + 1e-9 ...
    & aero.drag_N <= drive.maximumForce_N + 1e-9;
end
