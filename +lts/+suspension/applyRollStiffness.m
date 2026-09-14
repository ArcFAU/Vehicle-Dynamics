function [configuredVehicle, properties] = applyRollStiffness(vehicle)
%APPLYROLLSTIFFNESS Return a copy using the derived elastic distribution.

properties = lts.suspension.rollProperties(vehicle);
configuredVehicle = vehicle;
configuredVehicle.suspension.frontRollMomentFraction = ...
    properties.frontRollMomentFraction;
configuredVehicle.suspension.frontRollStiffness_NmPerRad = ...
    properties.frontAxleRollStiffness_NmPerRad;
configuredVehicle.suspension.rearRollStiffness_NmPerRad = ...
    properties.rearAxleRollStiffness_NmPerRad;
end
