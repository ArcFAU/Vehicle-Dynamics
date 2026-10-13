function eff = conversionEfficiencies(vehicle,speed,force,soc,direction)
%CONVERSIONEFFICIENCIES Shared conversion with explicit motor modules.
% Torque rows, speed columns; terminal and stored energy are separate.
arguments
    vehicle (1,1) struct
    speed double {mustBeNonnegative,mustBeFinite}
    force double {mustBeFinite}
    soc double {mustBeInRange(soc,0,1)}
    direction (1,1) string {mustBeMember(direction,["drive","regen"])}
end
p=vehicle.powertrain; e=vehicle.energy; model=e.efficiencyModel;
eff.gearbox=p.gearboxEfficiency*ones(size(speed));
eff.inverter=e.inverterEfficiency*ones(size(speed));
eff.motorSpeed_rpm=speed/p.effectiveTireRadius_m*p.gearRatio*60/(2*pi);
if direction=="drive"
    eff.motorTorque_Nm=abs(force)*p.effectiveTireRadius_m/(p.gearRatio*p.motorCount*p.gearboxEfficiency);
    eff.motor=e.motorEfficiency*ones(size(speed));
    eff.battery=e.batteryDischargeEfficiency*ones(size(speed));
    shaftPower=abs(force.*speed)/p.gearboxEfficiency;
else
    eff.motorTorque_Nm=abs(force)*p.effectiveTireRadius_m*p.gearboxEfficiency/(p.gearRatio*p.motorCount);
    eff.motor=e.regenMotorEfficiency*ones(size(speed));
    eff.battery=e.batteryChargeEfficiency*ones(size(speed));
    shaftPower=abs(force.*speed)*p.gearboxEfficiency;
end
eff.mapClamped=false(size(speed)); eff.motorLoss_W=zeros(size(speed));
if strcmp(model.mode,'map')
    if direction=="drive"
        mv=model.motor.driveValues; iv=model.inverter.driveValues; bv=model.battery.dischargeValues;
    else
        mv=model.motor.regenValues; iv=model.inverter.regenValues; bv=model.battery.chargeValues;
    end
    [eff.motor,eff.mapClamped]=interpolateMap(model.motor.speed_rpm,model.motor.torque_Nm,mv,eff.motorSpeed_rpm,eff.motorTorque_Nm);
    eff.inverter=interpolateMap(model.inverter.power_W,model.inverter.soc,iv,shaftPower,soc);
    eff.battery=interpolateMap(model.battery.power_W,model.battery.soc,bv,shaftPower,soc);
end
module=e.motorModel;
if ~strcmp(module.mode,'constant')
    map=module.map;
    if strcmp(module.mode,'efficiencyMap')
        if direction=="drive", values=map.driveEfficiency; else, values=map.regenEfficiency; end
        [eff.motor,eff.mapClamped]=interpolateMap(map.speed_rpm,map.torque_Nm,values,eff.motorSpeed_rpm,eff.motorTorque_Nm);
    else
        [loss,eff.mapClamped]=interpolateMap(map.speed_rpm,map.torque_Nm,map.loss_W,eff.motorSpeed_rpm,eff.motorTorque_Nm,false);
        power=eff.motorTorque_Nm.*eff.motorSpeed_rpm*2*pi/60;
        eff.motorLoss_W=loss; eff.motor=ones(size(power)); active=power>0;
        if direction=="drive"
            eff.motor(active)=power(active)./(power(active)+loss(active));
        else
            eff.motor(active)=max(0,1-loss(active)./power(active));
        end
    end
end
assert(all(isfinite(eff.inverter(:)) & eff.inverter(:)>0 & eff.inverter(:)<=1) ...
    && all(isfinite(eff.battery(:)) & eff.battery(:)>0 & eff.battery(:)<=1), ...
    'lts:energy:EfficiencyMap','Component efficiencies must be in (0,1].');
eff.terminalTotal=eff.gearbox.*eff.motor.*eff.inverter;
eff.total=eff.terminalTotal;
if strcmp(e.measurementBoundary,'storedEnergy'), eff.total=eff.total.*eff.battery; end
assert(all(isfinite(eff.total(:)) & eff.total(:)>=0 & eff.total(:)<=1) ...
    && (direction=="regen" || all(eff.total(:)>0)), ...
    'lts:energy:EfficiencyMap','Invalid conversion efficiency.');
end
function [values,clamped] = interpolateMap(x,y,map,xq,yq,isEfficiency)
if nargin<6, isEfficiency=true; end
assert(numel(x)>=2 && numel(y)>=2 && all(diff(x)>0) && all(diff(y)>0) ...
    && isequal(size(map),[numel(y),numel(x)]) && all(isfinite(map(:))), ...
    'lts:energy:EfficiencyMap','Invalid map dimensions, axes or values.');
if isEfficiency
    assert(all(map(:)>0 & map(:)<=1),'lts:energy:EfficiencyMap','Efficiencies must be in (0,1].');
else
    assert(all(map(:)>=0),'lts:energy:EfficiencyMap','Losses must be nonnegative.');
end
xc=min(max(xq,x(1)),x(end)); yc=min(max(yq,y(1)),y(end));
clamped=abs(xc-xq)>1e-6 | abs(yc-yq)>1e-7;
values=interp2(x,y,map,xc,yc,'linear');
end
