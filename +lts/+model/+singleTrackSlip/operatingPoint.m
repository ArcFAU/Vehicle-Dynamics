function point = operatingPoint(vehicle, speed_mps, ax_mps2, ay_mps2, options)
%OPERATINGPOINT Level 1.3 slip-consistent front/rear axle state.

arguments
    vehicle (1, 1) struct
    speed_mps (1, 1) double
    ax_mps2 (1, 1) double
    ay_mps2 (1, 1) double
    options.targetYawMoment_Nm (1, 1) double = 0
    options.enableRetries (1, 1) logical = true
    options.initialUnknowns double = []
end
equivalent = lts.model.singleTrackSlip.equivalentVehicle(vehicle);
equivalent.controls.targetYawMoment_Nm = options.targetYawMoment_Nm;
wheel = lts.model.dualTrackSlip.operatingPoint(equivalent, speed_mps, ...
    ax_mps2, ay_mps2, enableRetries=options.enableRetries, ...
    targetYawMoment_Nm=options.targetYawMoment_Nm, ...
    initialUnknowns=options.initialUnknowns);

point = wheel;
point.axleOrder = {'front', 'rear'};
point.axleNormalLoads_N = [sum(wheel.normalLoads_N(1:2)), ...
    sum(wheel.normalLoads_N(3:4))];
point.axleLongitudinalForces_N = ...
    [sum(wheel.bodyLongitudinalForces_N(1:2)), ...
    sum(wheel.bodyLongitudinalForces_N(3:4))];
point.axleLateralForces_N = [sum(wheel.bodyLateralForces_N(1:2)), ...
    sum(wheel.bodyLateralForces_N(3:4))];
point.axleSlipRatios = [mean(wheel.slipRatios(1:2)), ...
    mean(wheel.slipRatios(3:4))];
point.axleSlipAngles_rad = [mean(wheel.slipAngles_rad(1:2)), ...
    mean(wheel.slipAngles_rad(3:4))];
point.axleUtilization = [max(wheel.tireUtilization(1:2)), ...
    max(wheel.tireUtilization(3:4))];
point.targetYawMoment_Nm = options.targetYawMoment_Nm;
end
