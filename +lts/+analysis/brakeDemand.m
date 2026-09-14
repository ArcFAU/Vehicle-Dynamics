function demand = brakeDemand(lapResult, vehicle)
%BRAKEDEMAND Derive required friction-plus-regen brake force and wheel torque.
% This is an output diagnostic. It does not impose a mechanical brake limit.

arguments
    lapResult (1, 1) struct
    vehicle (1, 1) struct
end
lts.validation.validateVehicle(vehicle);
distance_m = lapResult.distance_m(:);
segmentLength_m = diff(distance_m);
segmentSpeed_mps = segmentLength_m ./ diff(lapResult.time_s(:));
acceleration_mps2 = lapResult.segmentAcceleration_mps2(:);
aero = lts.aerodynamics.forces(vehicle.aerodynamics, segmentSpeed_mps);
rollingForce_N = vehicle.energy.rollingResistanceCoefficient ...
    .* (vehicle.mass.total_kg * 9.81 + aero.totalDownforce_N);
tireForce_N = vehicle.mass.total_kg .* acceleration_mps2 ...
    + aero.drag_N + rollingForce_N;
totalBrakeForce_N = max(0, -tireForce_N);
frontForce_N = totalBrakeForce_N * vehicle.brakes.frontForceFraction;
rearForce_N = totalBrakeForce_N ...
    * (1 - vehicle.brakes.frontForceFraction);
radius_m = vehicle.powertrain.effectiveTireRadius_m;

demand.segmentMidpoint_m = ...
    (distance_m(1:end-1) + distance_m(2:end)) / 2;
demand.segmentSpeed_mps = segmentSpeed_mps;
demand.totalForce_N = totalBrakeForce_N;
demand.frontAxleForce_N = frontForce_N;
demand.rearAxleForce_N = rearForce_N;
demand.frontAxleMoment_Nm = frontForce_N * radius_m;
demand.rearAxleMoment_Nm = rearForce_N * radius_m;
demand.frontWheelMoment_Nm = demand.frontAxleMoment_Nm / 2;
demand.rearWheelMoment_Nm = demand.rearAxleMoment_Nm / 2;
demand.maximumTotalForce_N = max(totalBrakeForce_N);
demand.maximumFrontAxleMoment_Nm = max(demand.frontAxleMoment_Nm);
demand.maximumRearAxleMoment_Nm = max(demand.rearAxleMoment_Nm);
demand.maximumFrontWheelMoment_Nm = max(demand.frontWheelMoment_Nm);
demand.maximumRearWheelMoment_Nm = max(demand.rearWheelMoment_Nm);
demand.note = ['Required tire-contact-patch braking demand; no hydraulic ', ...
    'ratio, disc radius or caliper efficiency is assumed.'];
end
