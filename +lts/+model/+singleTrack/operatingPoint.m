function point = operatingPoint(vehicle, longitudinalAcceleration_mps2, lateralAcceleration_mps2)
%OPERATINGPOINT Evaluate axle loads and forces for a stationary state.
% Sign convention: x forward, y left, positive ax accelerates the vehicle,
% positive ay is a left turn. Tire forces are expressed on the vehicle.

validateattributes(longitudinalAcceleration_mps2, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
validateattributes(lateralAcceleration_mps2, {'numeric'}, ...
    {'scalar', 'real', 'finite'});

state = lts.model.singleTrack.calculateState(vehicle, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2);
mass_kg = vehicle.mass.total_kg;
wheelbase_m = vehicle.geometry.wheelbase_m;
frontMassFraction = vehicle.geometry.frontMassFraction;
distanceCgToFront_m = wheelbase_m * (1 - frontMassFraction);
distanceCgToRear_m = wheelbase_m * frontMassFraction;
axleNormalLoads_N = [state.frontNormalLoad_N, state.rearNormalLoad_N];
axleLongitudinalForces_N = [state.frontLongitudinalForce_N, ...
    state.rearLongitudinalForce_N];
axleLateralForces_N = [state.frontLateralForce_N, state.rearLateralForce_N];
axleUtilization = [state.frontUtilization, state.rearUtilization];

point.feasible = state.feasible;
point.longitudinalAcceleration_mps2 = longitudinalAcceleration_mps2;
point.lateralAcceleration_mps2 = lateralAcceleration_mps2;
point.axleOrder = {'front', 'rear'};
point.axleNormalLoads_N = axleNormalLoads_N;
point.axleLongitudinalForces_N = axleLongitudinalForces_N;
point.axleLateralForces_N = axleLateralForces_N;
point.axleUtilization = axleUtilization;
point.residuals.longitudinalForce_N = ...
    sum(axleLongitudinalForces_N) ...
    - mass_kg * longitudinalAcceleration_mps2;
point.residuals.lateralForce_N = ...
    sum(axleLateralForces_N) - mass_kg * lateralAcceleration_mps2;
point.residuals.yawMoment_Nm = distanceCgToFront_m ...
    * state.frontLateralForce_N - distanceCgToRear_m ...
    * state.rearLateralForce_N;
end
