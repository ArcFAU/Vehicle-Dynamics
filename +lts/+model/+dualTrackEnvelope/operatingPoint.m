function point = operatingPoint(vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, peakEnvelope)
%OPERATINGPOINT Named four-wheel state and equilibrium residuals.

if nargin < 5
    peakEnvelope = lts.tire.createPeakEnvelope(vehicle);
end

validateattributes(speed_mps, {'numeric'}, {'scalar', 'real', 'finite', 'nonnegative'});
validateattributes(longitudinalAcceleration_mps2, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
state = lts.model.dualTrackEnvelope.calculateState(vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, peakEnvelope);
mass_kg = vehicle.mass.total_kg;
frontFraction = vehicle.geometry.frontMassFraction;
distanceCgToFront_m = vehicle.geometry.wheelbase_m * (1 - frontFraction);
distanceCgToRear_m = vehicle.geometry.wheelbase_m * frontFraction;
frontLateralForce_N = sum(state.lateralForces_N(1:2));
rearLateralForce_N = sum(state.lateralForces_N(3:4));

point = state;
point.wheelOrder = {'FL', 'FR', 'RL', 'RR'};
point.speed_mps = speed_mps;
point.longitudinalAcceleration_mps2 = longitudinalAcceleration_mps2;
point.lateralAcceleration_mps2 = lateralAcceleration_mps2;
point.residuals.longitudinalForce_N = sum(state.longitudinalForces_N) ...
    - state.aero.drag_N - mass_kg * longitudinalAcceleration_mps2;
point.residuals.lateralForce_N = sum(state.lateralForces_N) ...
    - mass_kg * lateralAcceleration_mps2;
point.residuals.yawMoment_Nm = distanceCgToFront_m * frontLateralForce_N ...
    - distanceCgToRear_m * rearLateralForce_N ...
    - sum([vehicle.geometry.frontTrack_m / 2, ...
    -vehicle.geometry.frontTrack_m / 2, vehicle.geometry.rearTrack_m / 2, ...
    -vehicle.geometry.rearTrack_m / 2] .* state.longitudinalForces_N) ...
    - vehicle.controls.targetYawMoment_Nm;
end
