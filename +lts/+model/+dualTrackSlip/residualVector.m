function [residual, state] = residualVector(vehicle, tireModel, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, unknowns, ...
    normalLoads_N, targetLongitudinalForces_N)
%RESIDUALVECTOR Six steady-state equations for the Level 2.1b solver.
% Unknown order: body slip beta, steering delta, then kappa FL/FR/RL/RR.

beta_rad = unknowns(1);
steeringAngle_rad = unknowns(2);
slipRatios = unknowns(3:6).';
mass_kg = vehicle.mass.total_kg;
wheelbase_m = vehicle.geometry.wheelbase_m;
frontFraction = vehicle.geometry.frontMassFraction;
distanceCgToFront_m = (1 - frontFraction) * wheelbase_m;
distanceCgToRear_m = frontFraction * wheelbase_m;
x_m = [distanceCgToFront_m, distanceCgToFront_m, ...
    -distanceCgToRear_m, -distanceCgToRear_m];
y_m = [vehicle.geometry.frontTrack_m / 2, ...
    -vehicle.geometry.frontTrack_m / 2, ...
    vehicle.geometry.rearTrack_m / 2, ...
    -vehicle.geometry.rearTrack_m / 2];
yawRate_radps = lateralAcceleration_mps2 / speed_mps;
velocityX_mps = speed_mps * cos(beta_rad) - yawRate_radps .* y_m;
velocityY_mps = speed_mps * sin(beta_rad) + yawRate_radps .* x_m;
wheelHeading_rad = [steeringAngle_rad, steeringAngle_rad, 0, 0];
slipAngles_rad = wheelHeading_rad - atan2(velocityY_mps, velocityX_mps);
camberAngles_rad = [vehicle.suspension.frontCamber_rad, ...
    -vehicle.suspension.frontCamber_rad, ...
    vehicle.suspension.rearCamber_rad, ...
    -vehicle.suspension.rearCamber_rad];

wheelLongitudinalForces_N = zeros(1, 4);
wheelLateralForces_N = zeros(1, 4);
tireUtilization = zeros(1, 4);
for wheel = 1:4
    tireForces = lts.tire.evaluate(tireModel, normalLoads_N(wheel), ...
        camberAngles_rad(wheel), slipRatios(wheel), ...
        slipAngles_rad(wheel), speed_mps);
    wheelLongitudinalForces_N(wheel) = tireForces.longitudinal_N;
    wheelLateralForces_N(wheel) = tireForces.lateral_N;
    tireUtilization(wheel) = tireForces.utilization;
end

cosHeading = cos(wheelHeading_rad);
sinHeading = sin(wheelHeading_rad);
bodyLongitudinalForces_N = wheelLongitudinalForces_N .* cosHeading ...
    - wheelLateralForces_N .* sinHeading;
bodyLateralForces_N = wheelLongitudinalForces_N .* sinHeading ...
    + wheelLateralForces_N .* cosHeading;
aero = lts.aerodynamics.forces(vehicle.aerodynamics, speed_mps);
longitudinalBalance_N = sum(bodyLongitudinalForces_N) - aero.drag_N ...
    - mass_kg * longitudinalAcceleration_mps2;
lateralBalance_N = sum(bodyLateralForces_N) ...
    - mass_kg * lateralAcceleration_mps2;
generatedYawMoment_Nm = sum(x_m .* bodyLateralForces_N ...
    - y_m .* bodyLongitudinalForces_N);
targetYawMoment_Nm = vehicle.controls.targetYawMoment_Nm;
yawBalance_Nm = generatedYawMoment_Nm - targetYawMoment_Nm;

frontForce_N = sum(wheelLongitudinalForces_N(1:2));
rearForce_N = sum(wheelLongitudinalForces_N(3:4));
if longitudinalAcceleration_mps2 < 0
    frontForceFraction = vehicle.brakes.frontForceFraction;
else
    frontForceFraction = vehicle.powertrain.frontDriveForceFraction;
end
leftRightFrontBalance_N = wheelLongitudinalForces_N(1) ...
    - wheelLongitudinalForces_N(2);
leftRightRearBalance_N = wheelLongitudinalForces_N(3) ...
    - wheelLongitudinalForces_N(4);
axleDistributionBalance_N = (1 - frontForceFraction) * frontForce_N ...
    - frontForceFraction * rearForce_N;
allocationDistributionBalance_N = wheelLongitudinalForces_N(1:3).' ...
    - targetLongitudinalForces_N(1:3).';
if sum(targetLongitudinalForces_N) < 0
    allocationMode = vehicle.controls.brakeAllocation.mode;
else
    allocationMode = vehicle.controls.driveAllocation.mode;
end

forceScale_N = mass_kg * 9.81;
if strcmp(allocationMode, 'ideal')
    distributionResidual_N = allocationDistributionBalance_N;
else
    distributionResidual_N = [leftRightFrontBalance_N; ...
        leftRightRearBalance_N; axleDistributionBalance_N];
end
residual = [longitudinalBalance_N; lateralBalance_N; ...
    yawBalance_Nm / wheelbase_m; distributionResidual_N] / forceScale_N;

state.bodySlipAngle_rad = beta_rad;
state.steeringAngle_rad = steeringAngle_rad;
state.yawRate_radps = yawRate_radps;
state.slipRatios = slipRatios;
state.slipAngles_rad = slipAngles_rad;
state.normalLoads_N = normalLoads_N;
state.wheelLongitudinalForces_N = wheelLongitudinalForces_N;
state.wheelLateralForces_N = wheelLateralForces_N;
state.bodyLongitudinalForces_N = bodyLongitudinalForces_N;
state.bodyLateralForces_N = bodyLateralForces_N;
state.tireUtilization = tireUtilization;
state.targetLongitudinalForces_N = targetLongitudinalForces_N;
state.aero = aero;
state.residuals.longitudinalForce_N = longitudinalBalance_N;
state.residuals.lateralForce_N = lateralBalance_N;
state.residuals.yawMoment_Nm = yawBalance_Nm;
state.generatedYawMoment_Nm = generatedYawMoment_Nm;
state.targetYawMoment_Nm = targetYawMoment_Nm;
state.residuals.frontLeftRightForce_N = leftRightFrontBalance_N;
state.residuals.rearLeftRightForce_N = leftRightRearBalance_N;
state.residuals.axleDistributionForce_N = axleDistributionBalance_N;
state.residuals.allocationDistributionForce_N = ...
    allocationDistributionBalance_N.';
end
