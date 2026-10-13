function step = propagateSegment(vehicle,soc,speed,force,ds,dt)
%PROPAGATESEGMENT Signed electrical energy at the accumulator terminals.
arguments
    vehicle (1,1) struct
    soc (1,1) double {mustBeInRange(soc,0,1)}
    speed (1,1) double {mustBeNonnegative,mustBeFinite}
    force (1,1) double {mustBeFinite}
    ds (1,1) double {mustBePositive,mustBeFinite}
    dt (1,1) double {mustBePositive,mustBeFinite}
end
limits=lts.energy.limitsAtSoc(vehicle,soc);
e=vehicle.energy; p=vehicle.powertrain;
aux=e.auxiliaryPower_W*dt;
traction=max(0,force*ds); braking=max(0,-force*ds);
drive=lts.energy.conversionEfficiencies(vehicle,speed,force,soc,"drive");
tractionBattery=traction/drive.total;
allocation=1;
if strcmp(vehicle.controls.regenAllocation.mode,'fixed')
    allocation=vehicle.controls.regenAllocation.fixedUtilizationFactor;
end
% In regeneration gearbox losses reduce power reaching the motor shaft.
mappedTorque=lts.powertrain.motorTorqueLimit(vehicle,speed);
torqueForce=min(e.maximumRegenMotorTorque_Nm,mappedTorque)*p.motorCount*p.gearRatio ...
    /(p.gearboxEfficiency*p.effectiveTireRadius_m) ...
    *limits.regenTorqueScale*allocation;
shaftPower=min(e.maximumMechanicalRegenPower_W, ...
    e.maximumRegenMotorPower_W*p.motorCount)*limits.regenPowerScale*allocation;
regen=min([braking,torqueForce*ds, ...
    shaftPower/p.gearboxEfficiency*dt, ...
    e.maximumRegenDeceleration_mps2*vehicle.mass.total_kg*ds*allocation]);
rpm=speed/p.effectiveTireRadius_m*p.gearRatio*60/(2*pi);
if speed<e.minimumRegenSpeed_mps || (p.limitOptions.motorSpeedEnabled && rpm>p.maximumMotorSpeed_rpm), regen=0; end
capacity=e.installedCapacity_kWh*3.6e6;
chargeLimit=limits.maximumBatteryChargePower_W*dt;
headroom=max(0,(1-soc)*capacity+aux);
% Evaluate the motor map at the actual regenerative torque, not total brake.
regenEff=lts.energy.conversionEfficiencies(vehicle,speed, ...
    -regen/ds,soc,"regen");
if chargeLimit==0 || headroom==0
    regen=0;
    regenEff=lts.energy.conversionEfficiencies(vehicle,speed,0,soc,"regen");
end
if regen*regenEff.terminalTotal>chargeLimit || regen*regenEff.total>headroom
    low=0; high=regen;
    for k=1:36
        mid=(low+high)/2;
        candidate=lts.energy.conversionEfficiencies( ...
            vehicle,speed,-mid/ds,soc,"regen");
        if mid*candidate.terminalTotal<=chargeLimit && mid*candidate.total<=headroom, low=mid; else, high=mid; end
    end
    regen=low;
    regenEff=lts.energy.conversionEfficiencies( ...
        vehicle,speed,-regen/ds,soc,"regen");
end
% Below the generator loss threshold, use friction braking. Active motor
% braking that consumes electrical power is outside this recuperation model.
if regenEff.total==0
    regen=0;
    regenEff=lts.energy.conversionEfficiencies(vehicle,speed,0,soc,"regen");
end
recovered=regen*regenEff.total;
net=tractionBattery+aux-recovered;
potential=lts.energy.conversionEfficiencies(vehicle,speed,-braking/ds,soc,"regen");
step.potentialRecovered_J=braking*potential.total;
step.terminalTraction_J=traction/drive.terminalTotal;
step.terminalRecovered_J=regen*regenEff.terminalTotal;
step.terminalBatteryPower_W=(step.terminalTraction_J+aux-step.terminalRecovered_J)/dt;
step.tractionMechanical_J=traction;
step.brakingMechanical_J=braking;
step.tractionBattery_J=tractionBattery;
step.auxiliary_J=aux;
step.recoveredBattery_J=recovered;
step.regenMechanical_J=regen;
step.frictionBrake_J=braking-regen;
step.netBattery_J=net;
step.batteryPower_W=net/dt;
step.powerLimitViolation_J=max(0,(traction/drive.terminalTotal+aux)/dt ...
    -limits.maximumBatteryDischargePower_W)*dt;
step.capacityShortfall_J=max(0,net-soc*capacity);
step.dischargePowerLimit_W=limits.maximumBatteryDischargePower_W;
step.chargePowerLimit_W=limits.maximumBatteryChargePower_W;
step.driveEfficiency=drive.total;
step.regenEfficiency=regenEff.total;
step.motorSpeed_rpm=drive.motorSpeed_rpm;
step.motorTorque_Nm=drive.motorTorque_Nm*(traction>0) ...
    -regenEff.motorTorque_Nm*(regen>0);
step.motorShaftPower_W=(traction/p.gearboxEfficiency ...
    -regen*p.gearboxEfficiency)/dt;
step.motorMapClamped=(traction>0 && drive.mapClamped) ...
    || (regen>0 && regenEff.mapClamped);
step.regenAllocationMode=vehicle.controls.regenAllocation.mode;
step.regenAllocationFactor=allocation;
step.nextSoc=min(1,max(0,soc-net/capacity));
end
