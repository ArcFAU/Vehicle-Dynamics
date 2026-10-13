function point = operatingPoint(vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, options)
%OPERATINGPOINT Solve a slip-consistent steady-state four-wheel condition.
% This diagnostic Level 2.1b solver is not yet used for lap boundaries.

arguments
    vehicle (1, 1) struct
    speed_mps (1, 1) double
    longitudinalAcceleration_mps2 (1, 1) double
    lateralAcceleration_mps2 (1, 1) double
    options.enableRetries (1, 1) logical = true
    options.targetYawMoment_Nm (1, 1) double = 0
    options.initialUnknowns double = []
end

validateattributes(speed_mps, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(longitudinalAcceleration_mps2, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
tireModel = lts.tire.create(vehicle);
vehicle.controls.targetYawMoment_Nm = options.targetYawMoment_Nm;
initializationEnvelope = createInitializationEnvelope(vehicle);
envelopeState = lts.model.dualTrackEnvelope.calculateState(vehicle, ...
    speed_mps, longitudinalAcceleration_mps2, lateralAcceleration_mps2, ...
    initializationEnvelope);
normalLoads_N = envelopeState.normalLoads_N;
targetLongitudinalForces_N = envelopeState.longitudinalForces_N;
if any(normalLoads_N <= 0)
    error('lts:model:WheelLift', ...
        'A slip operating point requires four positive normal loads.');
end

if isempty(options.initialUnknowns)
    initial = initialGuess(vehicle, tireModel, speed_mps, ...
        lateralAcceleration_mps2, envelopeState);
else
    assert(numel(options.initialUnknowns) == 6, ...
        'lts:model:SlipInitialGuess', ...
        'A slip-solver initial guess must contain six values.');
    initial = options.initialUnknowns(:);
end
steeringLimit_rad = vehicle.validity.maximumRoadWheelSteeringAngle_rad;
minimumSlipRatio = vehicle.validity.minimumSlipRatio;
maximumSlipRatio = vehicle.validity.maximumSlipRatio;
lowerBounds = [-0.5; -steeringLimit_rad; ...
    minimumSlipRatio * ones(4, 1)];
upperBounds = [0.5; steeringLimit_rad; ...
    maximumSlipRatio * ones(4, 1)];
initial = max(min(initial, upperBounds), lowerBounds);
objective = @(unknowns) lts.model.dualTrackSlip.residualVector(...
    vehicle, tireModel, speed_mps, longitudinalAcceleration_mps2, ...
    lateralAcceleration_mps2, unknowns, normalLoads_N, ...
    targetLongitudinalForces_N);
solverOptions = optimoptions('lsqnonlin', 'Display', 'off', ...
    'FunctionTolerance', 1e-11, 'StepTolerance', 1e-11, ...
    'OptimalityTolerance', 1e-11, 'MaxIterations', 150);
[solution, residualNorm, normalizedResidual, exitFlag, output] = ...
    lsqnonlin(objective, initial, lowerBounds, upperBounds, solverOptions);
attemptCount = 1;
if options.enableRetries && norm(normalizedResidual, Inf) >= 1e-6
    retryStarts = [[solution(1:2); zeros(4, 1)], zeros(6, 1)];
    for retry = 1:size(retryStarts, 2)
        [candidateSolution, candidateNorm, candidateResidual, ...
            candidateExitFlag, candidateOutput] = lsqnonlin(objective, ...
            retryStarts(:, retry), lowerBounds, upperBounds, solverOptions);
        attemptCount = attemptCount + 1;
        if norm(candidateResidual, Inf) < norm(normalizedResidual, Inf)
            solution = candidateSolution;
            residualNorm = candidateNorm;
            normalizedResidual = candidateResidual;
            exitFlag = candidateExitFlag;
            output = candidateOutput;
        end
        if norm(normalizedResidual, Inf) < 1e-6
            break
        end
    end
end
[~, point] = objective(solution);
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, speed_mps);
positiveWheelForce_N = sum(max(point.wheelLongitudinalForces_N, 0));
point.solver.exitFlag = exitFlag;
point.solver.iterations = output.iterations;
point.solver.attemptCount = attemptCount;
point.solver.residualNorm = residualNorm;
point.solver.normalizedResidual = normalizedResidual;
point.solver.solution = solution;
point.solver.initializationEnvelope = initializationEnvelope.classification;
point.driveForceAvailable_N = drive.maximumForce_N;
point.driveForceUsed_N = positiveWheelForce_N;
point.tireModel = tireModel.type;
point.tireParameterSet = tireModel.parameterSet;
point.targetYawMoment_Nm = options.targetYawMoment_Nm;
point.feasible = exitFlag > 0 && norm(normalizedResidual, Inf) < 1e-6 ...
    && positiveWheelForce_N <= drive.maximumForce_N + 1e-6;
point.validity = physicalValidity(point, vehicle, lowerBounds, upperBounds);
point.physicallyValid = point.feasible ...
    && point.validity.withinSteeringLimits ...
    && point.validity.withinSlipAngleLimits ...
    && point.validity.withinSlipRatioLimits ...
    && point.validity.withinNormalLoadLimits ...
    && point.validity.awayFromSolverBounds;
end

function validity = physicalValidity(point, vehicle, lowerBounds, upperBounds)
limits = vehicle.validity;
validity.withinSteeringLimits = ...
    abs(point.steeringAngle_rad) ...
    <= limits.maximumRoadWheelSteeringAngle_rad + 1e-12;
validity.withinSlipAngleLimits = ...
    all(point.slipAngles_rad >= limits.minimumSlipAngle_rad) ...
    && all(point.slipAngles_rad <= limits.maximumSlipAngle_rad);
validity.withinSlipRatioLimits = ...
    all(point.slipRatios >= limits.minimumSlipRatio) ...
    && all(point.slipRatios <= limits.maximumSlipRatio);
validity.withinNormalLoadLimits = ...
    all(point.normalLoads_N >= limits.minimumValidatedNormalLoad_N) ...
    && all(point.normalLoads_N <= limits.maximumValidatedNormalLoad_N);

margin = limits.solverBoundMarginFraction;
span = upperBounds - lowerBounds;
marginLower = lowerBounds + margin * span;
marginUpper = upperBounds - margin * span;
solution = point.solver.solution;
validity.awayFromSolverBounds = all(solution >= marginLower - 1e-12) ...
    && all(solution <= marginUpper + 1e-12);
validity.minimumSolverBoundMargin = min([...
    solution - lowerBounds; upperBounds - solution]);
end

function envelope = createInitializationEnvelope(vehicle)
%CREATEINITIALIZATIONENVELOPE Numerical seed, not the solved tire physics.
% The nonlinear residual always evaluates the explicitly selected tire model.
% TMeasy has no validated fast peak adapter, so its solver seed uses the
% declared load-sensitive envelope without enabling a physical approximation.

if strcmp(vehicle.tireModel.active, 'fittedCombinedSlip')
    envelope = lts.tire.createPeakEnvelope(vehicle);
else
    envelope.type = 'loadSensitiveSymmetric';
    envelope.name = 'Numerical initialization envelope';
    envelope.parameters = struct();
    envelope.fallback = vehicle.tireEnvelope;
    envelope.combinedForceExponent = ...
        vehicle.tireEnvelope.combinedForceExponent;
    envelope.classification = 'numericalInitializationOnly';
end
end

function initial = initialGuess(vehicle, tireModel, speed_mps, ...
    lateralAcceleration_mps2, envelopeState)
targetFx_N = envelopeState.longitudinalForces_N;
targetFy_N = envelopeState.lateralForces_N;
slipRatios = zeros(4, 1);
slipAngles_rad = zeros(4, 1);
camberAngles_rad = [vehicle.suspension.frontCamber_rad, ...
    -vehicle.suspension.frontCamber_rad, ...
    vehicle.suspension.rearCamber_rad, ...
    -vehicle.suspension.rearCamber_rad];
for wheel = 1:4
    estimate = lts.tire.estimateSlipForForce(tireModel, ...
        envelopeState.normalLoads_N(wheel), camberAngles_rad(wheel), ...
        targetFx_N(wheel), targetFy_N(wheel), speed_mps);
    slipRatios(wheel) = estimate.slipRatio;
    slipAngles_rad(wheel) = estimate.slipAngle_rad;
end
slipRatios = max(min(slipRatios, 0.3), -0.3);
frontFraction = vehicle.geometry.frontMassFraction;
xRear_m = -frontFraction * vehicle.geometry.wheelbase_m;
yawRate_radps = lateralAcceleration_mps2 / speed_mps;
beta_rad = -mean(slipAngles_rad(3:4)) ...
    - yawRate_radps * xRear_m / speed_mps;
xFront_m = (1 - frontFraction) * vehicle.geometry.wheelbase_m;
steeringAngle_rad = mean(slipAngles_rad(1:2)) + beta_rad ...
    + yawRate_radps * xFront_m / speed_mps;
initial = [beta_rad; steeringAngle_rad; slipRatios];
end
