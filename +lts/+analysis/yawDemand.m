function demand = yawDemand(lapResult, vehicle, options)
%YAWDEMAND Derive yaw rate, yaw acceleration and inertial yaw moment.

arguments
    lapResult (1, 1) struct
    vehicle (1, 1) struct
    options.CouplingMode (1, 1) string = "diagnosticOnly"
end

distance_m = lapResult.distance_m(:);
time_s = lapResult.time_s(:);
speed_mps = lapResult.speed_mps(:);
curvature_1pm = lapResult.curvature_1pm(:);
yawRate_radps = speed_mps .* curvature_1pm;
yawAcceleration_radps2 = zeros(size(yawRate_radps));
if numel(time_s) > 2
    yawAcceleration_radps2(2:end-1) = ...
        (yawRate_radps(3:end) - yawRate_radps(1:end-2)) ...
        ./ (time_s(3:end) - time_s(1:end-2));
end
demand.distance_m = distance_m;
demand.yawRate_radps = yawRate_radps;
demand.yawAcceleration_radps2 = yawAcceleration_radps2;
demand.inertialMoment_Nm = vehicle.mass.yawInertia_kgm2 ...
    .* yawAcceleration_radps2;
demand.maximumAbsoluteMoment_Nm = max(abs(demand.inertialMoment_Nm));
demand.couplingMode = char(options.CouplingMode);
if options.CouplingMode == "criticalPointAudit"
    demand.note = ['Inertial yaw demand is enforced in directly audited ', ...
        'critical operating points; the interpolated base envelope remains ', ...
        'steady-state.'];
else
    demand.note = ['Diagnostic demand from the converged path profile; ', ...
        'not enforced in the lap boundaries.'];
end
end
