function allocation = allocateLongitudinalForces(vehicle, speed_mps, ...
        capacities_N, lateralForces_N, requiredForce_N, operation)
%ALLOCATELONGITUDINALFORCES Fixed or ideal four-wheel force allocation.
% Wheel order is FL, FR, RL, RR. Ideal mode satisfies total longitudinal
% force and yaw moment while minimizing normalized tire-force usage.

arguments
    vehicle (1, 1) struct
    speed_mps (1, 1) double {mustBeNonnegative}
    capacities_N (1, 4) double {mustBeNonnegative}
    lateralForces_N (1, 4) double
    requiredForce_N (1, 1) double
    operation (1, 1) string {mustBeMember(operation, ["drive", "brake"])}
end

if operation == "drive"
    mode = vehicle.controls.driveAllocation.mode;
    fixedFrontFraction = vehicle.powertrain.frontDriveForceFraction;
else
    mode = vehicle.controls.brakeAllocation.mode;
    fixedFrontFraction = vehicle.brakes.frontForceFraction;
end

positions = wheelPositions(vehicle);
lateralYawMoment_Nm = sum(positions.x_m .* lateralForces_N);
requiredLongitudinalYawMoment_Nm = ...
    vehicle.controls.targetYawMoment_Nm - lateralYawMoment_Nm;

switch mode
    case 'fixed'
        frontForce_N = requiredForce_N * fixedFrontFraction;
        rearForce_N = requiredForce_N * (1 - fixedFrontFraction);
        forces_N = [frontForce_N / 2, frontForce_N / 2, ...
            rearForce_N / 2, rearForce_N / 2];
        exitFlag = double(all(abs(forces_N) <= capacities_N + 1e-9));
    case 'ideal'
        [forces_N, exitFlag] = solveIdeal(vehicle, speed_mps, ...
            capacities_N, requiredForce_N, ...
            requiredLongitudinalYawMoment_Nm, positions.y_m, operation);
    otherwise
        error('lts:control:AllocationMode', ...
            'Unknown %s allocation mode: %s.', operation, mode);
end

allocation.mode = mode;
allocation.operation = char(operation);
allocation.forces_N = forces_N;
allocation.feasible = exitFlag > 0 ...
    && all(abs(forces_N) <= capacities_N + 1e-7);
allocation.residuals.longitudinalForce_N = sum(forces_N) ...
    - requiredForce_N;
allocation.residuals.yawMoment_Nm = lateralYawMoment_Nm ...
    - sum(positions.y_m .* forces_N) ...
    - vehicle.controls.targetYawMoment_Nm;
allocation.feasible = allocation.feasible ...
    && abs(allocation.residuals.longitudinalForce_N) <= 1e-6 ...
    && abs(allocation.residuals.yawMoment_Nm) <= 1e-6;
allocation.maximumNormalizedUsage = max(abs(forces_N) ...
    ./ max(capacities_N, 1e-12));
end

function [forces_N, exitFlag] = solveIdeal(vehicle, speed_mps, ...
        capacities_N, requiredForce_N, requiredYaw_Nm, y_m, operation)
assert(exist('quadprog', 'file') == 2, 'lts:control:OptimizationToolbox', ...
    'Ideal torque allocation requires Optimization Toolbox quadprog.');
lowerBounds_N = -capacities_N(:);
upperBounds_N = capacities_N(:);
A = zeros(0, 4);
b = zeros(0, 1);
Aeq = [ones(1, 4); -y_m];
beq = [requiredForce_N; requiredYaw_Nm];

if operation == "drive"
    lowerBounds_N(:) = 0;
    groups = vehicle.powertrain.wheelActuatorGroups;
    upperBounds_N(groups == 0) = 0;
    activeGroups = unique(groups(groups > 0));
    motorTorqueForce_N = vehicle.powertrain.maximumMotorTorque_Nm ...
        * vehicle.powertrain.gearRatio ...
        * vehicle.powertrain.gearboxEfficiency ...
        / vehicle.powertrain.effectiveTireRadius_m;
    motorPowerForce_N = Inf;
    if speed_mps > 0
        motorPowerForce_N = vehicle.powertrain.maximumMotorPower_W ...
            * vehicle.powertrain.gearboxEfficiency / speed_mps;
    end
    motorForceLimit_N = min(motorTorqueForce_N, motorPowerForce_N);
    for groupIndex = 1:numel(activeGroups)
        group = activeGroups(groupIndex);
        members = find(groups == group);
        row = zeros(1, 4);
        row(members) = 1;
        A(end + 1, :) = row; %#ok<AGROW>
        b(end + 1, 1) = motorForceLimit_N; %#ok<AGROW>
        for memberIndex = 2:numel(members)
            equalityRow = zeros(1, 4);
            equalityRow(members(1)) = 1;
            equalityRow(members(memberIndex)) = -1;
            Aeq(end + 1, :) = equalityRow; %#ok<AGROW>
            beq(end + 1, 1) = 0; %#ok<AGROW>
        end
    end
else
    upperBounds_N(:) = 0;
end

scale_N = max(capacities_N(:), 1);
H = diag(1 ./ scale_N .^ 2);
solverOptions = optimoptions('quadprog', 'Display', 'off', ...
    'OptimalityTolerance', 1e-10, 'ConstraintTolerance', 1e-9);
[forces_N, ~, exitFlag] = quadprog(H, zeros(4, 1), A, b, ...
    Aeq, beq, lowerBounds_N, upperBounds_N, [], solverOptions);
if isempty(forces_N)
    forces_N = zeros(4, 1);
end
forces_N = forces_N.';
end

function positions = wheelPositions(vehicle)
frontFraction = vehicle.geometry.frontMassFraction;
positions.x_m = [(1 - frontFraction) * vehicle.geometry.wheelbase_m ...
    * ones(1, 2), -frontFraction * vehicle.geometry.wheelbase_m ...
    * ones(1, 2)];
positions.y_m = [vehicle.geometry.frontTrack_m / 2, ...
    -vehicle.geometry.frontTrack_m / 2, ...
    vehicle.geometry.rearTrack_m / 2, ...
    -vehicle.geometry.rearTrack_m / 2];
end
