function [configuredVehicle, coupling] = applyTorsionalStiffness(vehicle, stiffness_NmPerRad)
%APPLYTORSIONALSTIFFNESS Return a vehicle copy with chassis roll coupling.

validateattributes(stiffness_NmPerRad, {'numeric'}, ...
    {'scalar', 'real', 'nonnegative'});
assert(~isnan(stiffness_NmPerRad), 'lts:chassis:Stiffness', ...
    'Chassis torsional stiffness must not be NaN.');
[configuredVehicle, ~] = lts.suspension.applyRollStiffness(vehicle);
configuredVehicle.chassis.torsionalStiffness_NmPerRad = stiffness_NmPerRad;
coupling = lts.chassis.rollCoupling(configuredVehicle);
configuredVehicle.suspension.frontRollMomentFraction = ...
    coupling.frontRollMomentFraction;
end
