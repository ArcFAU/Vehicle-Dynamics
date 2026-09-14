function step = propagateSegment(vehicle, soc, speed_mps, ...
        wheelForce_N, segmentLength_m, segmentTime_s)
%PROPAGATESEGMENT Advance battery energy and SoC over one track segment.

arguments
    vehicle (1, 1) struct
    soc (1, 1) double {mustBeInRange(soc, 0, 1)}
    speed_mps (1, 1) double {mustBeNonnegative}
    wheelForce_N (1, 1) double {mustBeFinite}
    segmentLength_m (1, 1) double {mustBePositive}
    segmentTime_s (1, 1) double {mustBePositive}
end

limits = lts.energy.limitsAtSoc(vehicle, soc);
regenAllocationFactor = 1;
if strcmp(vehicle.controls.regenAllocation.mode, 'fixed')
    regenAllocationFactor = ...
        vehicle.controls.regenAllocation.fixedUtilizationFactor;
end
driveEfficiency = lts.energy.conversionEfficiencies(vehicle, speed_mps, ...
    wheelForce_N, soc, "drive");
regenEfficiency = lts.energy.conversionEfficiencies(vehicle, speed_mps, ...
    wheelForce_N, soc, "regen");
wheelWork_J = wheelForce_N * segmentLength_m;
tractionMechanical_J = max(0, wheelWork_J);
brakingMechanical_J = max(0, -wheelWork_J);
tractionBattery_J = tractionMechanical_J / driveEfficiency.total;
auxiliary_J = vehicle.energy.auxiliaryPower_W * segmentTime_s;

torqueLimitedForce_N = vehicle.energy.maximumRegenMotorTorque_Nm ...
    * vehicle.powertrain.gearRatio ...
    * vehicle.powertrain.gearboxEfficiency ...
    * vehicle.powertrain.motorCount ...
    / vehicle.powertrain.effectiveTireRadius_m ...
    * limits.regenTorqueScale * regenAllocationFactor;
motorRegenPower_W = min(vehicle.energy.maximumMechanicalRegenPower_W, ...
    vehicle.energy.maximumRegenMotorPower_W ...
    * vehicle.powertrain.motorCount) * limits.regenPowerScale ...
    * regenAllocationFactor;
maximumRegenMechanical_J = min( ...
    vehicle.energy.maximumRegenDeceleration_mps2 ...
        * vehicle.mass.total_kg * segmentLength_m ...
        * regenAllocationFactor, ...
    torqueLimitedForce_N * segmentLength_m);
maximumRegenMechanical_J = min(maximumRegenMechanical_J, ...
    motorRegenPower_W * segmentTime_s);
maximumRegenMechanical_J = min(maximumRegenMechanical_J, ...
    limits.maximumBatteryChargePower_W / regenEfficiency.total ...
    * segmentTime_s);
if speed_mps < vehicle.energy.minimumRegenSpeed_mps
    maximumRegenMechanical_J = 0;
end
regenMechanical_J = min(brakingMechanical_J, maximumRegenMechanical_J);
recoveredBattery_J = regenMechanical_J * regenEfficiency.total;
capacity_J = vehicle.energy.installedCapacity_kWh * 3.6e6;
availableChargeHeadroom_J = (1 - soc) * capacity_J + auxiliary_J;
recoveredBattery_J = min(recoveredBattery_J, ...
    max(0, availableChargeHeadroom_J));
regenMechanical_J = recoveredBattery_J / regenEfficiency.total;

netBattery_J = tractionBattery_J + auxiliary_J - recoveredBattery_J;
requestedDischargePower_W = max(0, ...
    (tractionBattery_J + auxiliary_J) / segmentTime_s);
unconstrainedNextSoc = soc - netBattery_J / capacity_J;

step.tractionMechanical_J = tractionMechanical_J;
step.brakingMechanical_J = brakingMechanical_J;
step.tractionBattery_J = tractionBattery_J;
step.auxiliary_J = auxiliary_J;
step.recoveredBattery_J = recoveredBattery_J;
step.potentialRecovered_J = brakingMechanical_J * regenEfficiency.total;
step.frictionBrake_J = brakingMechanical_J - regenMechanical_J;
step.netBattery_J = netBattery_J;
step.powerLimitViolation_J = max(0, requestedDischargePower_W ...
    - limits.maximumBatteryDischargePower_W) * segmentTime_s;
step.capacityShortfall_J = max(0, -unconstrainedNextSoc * capacity_J);
step.batteryPower_W = netBattery_J / segmentTime_s;
step.dischargePowerLimit_W = limits.maximumBatteryDischargePower_W;
step.chargePowerLimit_W = limits.maximumBatteryChargePower_W;
step.regenMechanical_J = regenMechanical_J;
step.driveEfficiency = driveEfficiency.total;
step.regenEfficiency = regenEfficiency.total;
step.regenAllocationMode = vehicle.controls.regenAllocation.mode;
step.regenAllocationFactor = regenAllocationFactor;
step.nextSoc = min(1, max(0, unconstrainedNextSoc));
end
