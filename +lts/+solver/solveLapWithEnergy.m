function result = solveLapWithEnergy(track,model,options)
%SOLVELAPWITHENERGY SoC-coupled speed with one battery-to-wheel limit.
% Friction brakes provide any braking force unavailable from regeneration.

arguments
    track (1,1) struct
    model (1,1) struct
    options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 1
    options.InitialSoc (1,1) double = NaN
    options.InitialSpeed_mps (1,1) double = NaN
    options.FinalSpeed_mps (1,1) double {mustBeNonnegative} = 100
    options.MaximumSpeed_mps (1,1) double {mustBePositive} = 100
    options.MaximumIterations (1,1) double {mustBeInteger,mustBePositive} = 30
    options.SpeedTolerance_mps (1,1) double {mustBePositive} = 1e-5
    options.BaseLap (1,1) struct = struct()
    options.ReusePeriodicLaps (1,1) logical = true
end

lts.validation.validateTrack(track);
assert(isfield(model,'vehicle') && isfield(model.vehicle,'energy'), ...
    'requires a vehicle with an energy model.');
vehicle = model.vehicle;
if isnan(options.InitialSoc), options.InitialSoc=vehicle.energy.initialSoc; end
assert(options.InitialSoc>=0 && options.InitialSoc<=1);
vehicle.energy.initialSoc=options.InitialSoc;
flat=isfield(vehicle.energy,'useSocLimits') && ~vehicle.energy.useSocLimits;
if options.ReusePeriodicLaps && flat && track.isClosed && options.Repetitions>1 ...
        && options.FinalSpeed_mps >= options.MaximumSpeed_mps
    steady=lts.solver.solveLapWithEnergy(track,model, ...
        InitialSoc=options.InitialSoc,InitialSpeed_mps=NaN, ...
        BaseLap=options.BaseLap,MaximumIterations=options.MaximumIterations, ...
        MaximumSpeed_mps=options.MaximumSpeed_mps,FinalSpeed_mps=options.FinalSpeed_mps, ...
        SpeedTolerance_mps=options.SpeedTolerance_mps,ReusePeriodicLaps=false);
    first=lts.solver.solveLapWithEnergy(track,model, ...
        InitialSoc=options.InitialSoc,InitialSpeed_mps=options.InitialSpeed_mps, ...
        BaseLap=steady,MaximumIterations=options.MaximumIterations, ...
        MaximumSpeed_mps=options.MaximumSpeed_mps,FinalSpeed_mps=options.FinalSpeed_mps, ...
        SpeedTolerance_mps=options.SpeedTolerance_mps,ReusePeriodicLaps=false);
    if abs(first.speed_mps(end)-steady.speed_mps(1))<=options.SpeedTolerance_mps
        ds=repmat(diff(track.distance_m(:)),options.Repetitions,1);
        result=first;
        result.distance_m=[0;cumsum(ds)];
        result.speed_mps=[first.speed_mps; ...
            repmat(steady.speed_mps(2:end),options.Repetitions-1,1)];
        result.curvature_1pm=[repmat(track.curvature_1pm(1:end-1), ...
            options.Repetitions,1);track.curvature_1pm(end)];
        dt=2*ds./(result.speed_mps(1:end-1)+result.speed_mps(2:end));
        result.time_s=[0;cumsum(dt)]; result.lapTime_s=result.time_s(end);
        result.segmentAcceleration_mps2=diff(result.speed_mps.^2)./(2*ds);
        result.lateralAcceleration_mps2=result.speed_mps.^2.*result.curvature_1pm;
        result.energy=lts.energy.analyzeLap(result,vehicle);
        if result.energy.energyBeyondInstalledCapacity_kWh==0
            result.socBoundary=result.energy.profile.socBoundary;
            result.meta.repetitions=options.Repetitions;
            result.meta.periodicReuse=true;
            result.meta.iterations=first.meta.iterations+steady.meta.iterations;
            result.meta.powerLimitViolationEnergy_kWh=result.energy.powerLimitViolationEnergy_kWh;
            result.meta.capacityShortfall_kWh=0;
            assert(result.energy.powerLimitViolationEnergy_kWh<1e-8);
            return
        end
    end
end
if options.Repetitions>1
    assert(track.isClosed,'Repeated tracks must be closed.');
end

if isnan(options.InitialSpeed_mps)
    if track.isClosed, initialSpeed=options.MaximumSpeed_mps; else, initialSpeed=0; end
else
    initialSpeed=options.InitialSpeed_mps;
end
if isempty(fieldnames(options.BaseLap))
    baseInitialSpeed=initialSpeed;
    if track.isClosed, baseInitialSpeed=options.MaximumSpeed_mps; end
    base=lts.solver.solveLap(track,model, ...
        initialSpeed_mps=baseInitialSpeed, ...
        finalSpeed_mps=options.FinalSpeed_mps, ...
        maximumSpeed_mps=options.MaximumSpeed_mps);
else
    base=options.BaseLap;
    assert(isequal(base.distance_m(:),track.distance_m(:)) ...
        && numel(base.speed_mps)==numel(track.distance_m), ...
        'BaseLap must match the selected track.');
end
distance=[0;cumsum(repmat(diff(track.distance_m(:)), ...
    options.Repetitions,1))];
curvature=[repmat(track.curvature_1pm(1:end-1), ...
    options.Repetitions,1);track.curvature_1pm(end)];
speed=[repmat(base.speed_mps(1:end-1), ...
    options.Repetitions,1);base.speed_mps(end)];
validateattributes(speed, {'double'}, {'column','real','finite','nonnegative'});
% A supplied profile is an upper estimate, not permission to ignore inputs.
speed=min(speed,options.MaximumSpeed_mps);
speed(end)=min(speed(end),options.FinalSpeed_mps);
ds=diff(distance);
converged=false;
for iteration=1:options.MaximumIterations
    previous=speed;
    for i=numel(speed)-1:-1:1
        ay=speed(i+1)^2*curvature(i+1);
        envelope=model.accelerationLimits(speed(i+1),ay);
        reachable=sqrt(max(0,speed(i+1)^2 ...
            +2*envelope.maximumDeceleration_mps2*ds(i)));
        speed(i)=min(speed(i),reachable);
    end
    speed(1)=min(speed(1),initialSpeed);
    soc=zeros(numel(speed),1);
    soc(1)=options.InitialSoc;
    for i=2:numel(speed)
        v0=speed(i-1);
        ay=v0^2*curvature(i-1);
        envelope=model.accelerationLimits(v0,ay);
        envelopeSpeed=sqrt(max(0,v0^2 ...
            +2*envelope.maximumAcceleration_mps2*ds(i-1)));
        speed(i)=min(speed(i),envelopeSpeed);
        speed(i)=limitSegmentAtMeanSpeed(vehicle,v0,speed(i), ...
            ds(i-1),soc(i-1));
        vMean=(v0+speed(i))/2;
        dt=2*ds(i-1)/(v0+speed(i));
        actualAx=(speed(i)^2-v0^2)/(2*ds(i-1));
        wheelForce=requiredWheelForce(vehicle,vMean,actualAx);
        step=lts.energy.propagateSegment(vehicle,soc(i-1),vMean, ...
            wheelForce,ds(i-1),dt);
        soc(i)=step.nextSoc;
    end
    if track.isClosed && options.Repetitions==1 && isnan(options.InitialSpeed_mps)
        seam=min(speed(1),speed(end)); speed(1)=seam; speed(end)=seam;
    end
    if max(abs(speed-previous))<=options.SpeedTolerance_mps
        converged=true;
        break
    end
end
assert(converged,'speed/SoC iteration did not converge.');
dt=2*ds./(speed(1:end-1)+speed(2:end));
assert(all(isfinite(dt)) && all(dt>0),'Invalid segment times.');
result.meta.modelName=model.name;
result.meta.modelLevel='2.4c';
if isfield(model,'level'), result.meta.baseModelLevel=model.level; end
result.meta.iterations=iteration;
result.meta.converged=converged;
result.meta.repetitions=options.Repetitions;
result.distance_m=distance;
result.time_s=[0;cumsum(dt)];
result.lapTime_s=result.time_s(end);
result.speed_mps=speed;
result.curvature_1pm=curvature;
result.lateralAcceleration_mps2=speed.^2.*curvature;
result.segmentAcceleration_mps2=diff(speed.^2)./(2*ds);
result.socBoundary=soc;
result.energy=lts.energy.analyzeLap(result,vehicle);
result.meta.powerLimitViolationEnergy_kWh= ...
    result.energy.powerLimitViolationEnergy_kWh;
assert(result.meta.powerLimitViolationEnergy_kWh<1e-8, ...
    'coupled speed still exceeds the battery discharge limit.');
result.meta.capacityShortfall_kWh= ...
    result.energy.energyBeyondInstalledCapacity_kWh;
end

function v1=limitSegmentAtMeanSpeed(vehicle,v0,v1,ds,soc)
% The energy integrator evaluates wheel work at segment-mean speed.
% Enforce the same motor/battery force bound at that point, not at v0.
if segmentWithinDriveLimit(vehicle,v0,v1,ds,soc)
    return
end
upper=v1;
lower=0;
assert(segmentWithinDriveLimit(vehicle,v0,lower,ds,soc), ...
    'No feasible segment speed under the battery/motor limits.');
for k=1:36
    candidate=(lower+upper)/2;
    if segmentWithinDriveLimit(vehicle,v0,candidate,ds,soc)
        lower=candidate;
    else
        upper=candidate;
    end
end
v1=lower;
end

function feasible=segmentWithinDriveLimit(vehicle,v0,v1,ds,soc)
vMean=(v0+v1)/2;
ax=(v1^2-v0^2)/(2*ds);
needed=requiredWheelForce(vehicle,vMean,ax);
if needed<=0, feasible=true; return; end
feasible=lts.powertrain.driveDemandFeasible(vehicle,vMean,needed,soc);
end

function force=requiredWheelForce(vehicle,speed,acceleration)
aero=lts.aerodynamics.forces(vehicle.aerodynamics,speed);
rolling=vehicle.energy.rollingResistanceCoefficient ...
    *(vehicle.mass.total_kg*9.81+aero.totalDownforce_N);
force=vehicle.mass.total_kg*acceleration+aero.drag_N+rolling;
end
