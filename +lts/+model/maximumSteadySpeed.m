function speed_mps = maximumSteadySpeed(vehicle, curvature_1pm, isFeasible, iterations)
%MAXIMUMSTEADYSPEED Shared finite search for stationary cornering limits.
% With RPM limiting disabled, drag and available shaft power provide an
% upper bound. Without drag, corners are bracketed by finite speed queries;
% straight sections have no stationary speed cap and return Inf.
validateattributes(curvature_1pm, {'numeric'}, {'real','finite'});
drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 0);
ceiling_mps = drive.maximumVehicleSpeed_mps;
if ~isfinite(ceiling_mps)
    a = vehicle.aerodynamics;
    dragFactor = 0.5*a.airDensity_kgpm3*a.referenceArea_m2*a.dragCoefficient;
    if dragFactor > 0
        wheelPower_W = drive.availableElectricalPower_W ...
            * vehicle.powertrain.gearboxEfficiency;
        ceiling_mps = (wheelPower_W/dragFactor)^(1/3);
    end
end
upper = ceiling_mps*ones(size(curvature_1pm));
lower = zeros(size(curvature_1pm));
unbounded = false(size(curvature_1pm));
if ~isfinite(ceiling_mps)
    unbounded = curvature_1pm == 0;
    upper(:) = 10;
    pending = ~unbounded;
    for attempt = 1:20
        if ~any(pending(:)), break; end
        feasible = isFeasible(upper, upper.^2.*curvature_1pm);
        pending = pending & feasible;
        upper(pending) = 2*upper(pending);
    end
    assert(~any(pending(:)), 'lts:model:SpeedSearchBracket', ...
        'No finite cornering boundary was found; review tire/aero inputs.');
end
for attempt = 1:iterations
    candidate = (lower+upper)/2;
    feasible = isFeasible(candidate, candidate.^2.*curvature_1pm);
    lower(feasible) = candidate(feasible);
    upper(~feasible) = candidate(~feasible);
end
speed_mps = lower;
speed_mps(unbounded) = Inf;
end
