function energy = analyzeLap(lapResult, vehicle, options)
%ANALYZELAP Calculate EV energy flow, SoC, recuperation and pack targets.
% Positive wheel work propels the car. Negative wheel work is braking work.
% The model is quasi-static and assumes a flat track without wind.

arguments
    lapResult (1, 1) struct
    vehicle (1, 1) struct
    options.DistanceWindows_m (:, 2) double = zeros(0, 2)
    options.Aggregation (1, 1) string = "fullTrack"
    options.Repetitions (1, 1) double {mustBePositive} = 1
end

lts.validation.validateVehicle(vehicle);
validateLapResult(lapResult);
validateTiming(options.DistanceWindows_m, options.Aggregation, lapResult);

distance_m = lapResult.distance_m(:);
segmentLength_m = diff(distance_m);
segmentTime_s = diff(lapResult.time_s(:));
segmentSpeed_mps = segmentLength_m ./ segmentTime_s;
segmentAcceleration_mps2 = lapResult.segmentAcceleration_mps2(:);
aero = lts.aerodynamics.forces(vehicle.aerodynamics, segmentSpeed_mps);
rollingForce_N = vehicle.energy.rollingResistanceCoefficient ...
    .* (vehicle.mass.total_kg * 9.81 + aero.totalDownforce_N);
wheelForce_N = vehicle.mass.total_kg .* segmentAcceleration_mps2 ...
    + aero.drag_N + rollingForce_N;
wheelWork_J = wheelForce_N .* segmentLength_m;

selectionWeight = timingWeights(distance_m, ...
    options.DistanceWindows_m, options.Aggregation);
sequence = expandSequence(segmentLength_m, segmentTime_s, ...
    segmentSpeed_mps, wheelForce_N, wheelWork_J, selectionWeight, ...
    options.Repetitions);
flow = simulateElectricalFlow(sequence, vehicle);

energy.status = 'calculated';
energy.meta.modelLevel = '2.4b';
energy.meta.efficiencyModel = vehicle.energy.efficiencyModel.mode;
energy.meta.motorModel = vehicle.energy.motorModel.mode;
energy.meta.measurementBoundary = vehicle.energy.measurementBoundary;
energy.meta.repetitions = options.Repetitions;
energy.meta.aggregation = char(options.Aggregation);
energy.meta.distanceWindows_m = options.DistanceWindows_m;
energy.totalDistance_m = sum(sequence.length_m);
energy.totalTime_s = sum(sequence.time_s);
energy.tractionMechanicalEnergy_kWh = ...
    sum(flow.tractionMechanical_J) / 3.6e6;
energy.brakingMechanicalEnergy_kWh = ...
    sum(flow.brakingMechanical_J) / 3.6e6;
energy.tractionBatteryEnergy_kWh = ...
    sum(flow.tractionBattery_J) / 3.6e6;
energy.auxiliaryEnergy_kWh = sum(flow.auxiliary_J) / 3.6e6;
energy.recoveredEnergy_kWh = sum(flow.recoveredBattery_J) / 3.6e6;
energy.theoreticalRecoverableEnergy_kWh = ...
    sum(flow.potentialRecovered_J) / 3.6e6;
energy.frictionBrakeEnergy_kWh = ...
    sum(flow.frictionBrake_J) / 3.6e6;
energy.netBatteryEnergy_kWh = sum(flow.netBattery_J) / 3.6e6;
energy.powerLimitViolationEnergy_kWh = ...
    sum(flow.powerLimitViolation_J) / 3.6e6;
energy.energyBeyondInstalledCapacity_kWh = ...
    sum(flow.capacityShortfall_J) / 3.6e6;
energy.powerLimitedSegmentCount = nnz(flow.powerLimitViolation_J > 0);
energy.initialSoc = vehicle.energy.initialSoc;
energy.finalSoc = flow.socBoundary(end);
energy.minimumSoc = min(flow.socBoundary);
energy.maximumSoc = max(flow.socBoundary);
energy.installedCapacity_kWh = vehicle.energy.installedCapacity_kWh;
energy.requiredUsableEnergy_kWh = max([0; cumsum(flow.netBattery_J)]) / 3.6e6;
energy.motorMapClampedSegmentCount = nnz(flow.motorMapClamped);
energy.netTerminalEnergy_kWh = sum(flow.terminalBatteryPower_W .* sequence.time_s) / 3.6e6;
energy.tractionTerminalEnergy_kWh = sum(flow.terminalTraction_J) / 3.6e6;
energy.recoveredTerminalEnergy_kWh = sum(flow.terminalRecovered_J) / 3.6e6;

if energy.theoreticalRecoverableEnergy_kWh > 0
    energy.regenPotentialUtilization = energy.recoveredEnergy_kWh ...
        / energy.theoreticalRecoverableEnergy_kWh;
else
    energy.regenPotentialUtilization = NaN;
end
grossConsumption_kWh = energy.tractionBatteryEnergy_kWh ...
    + energy.auxiliaryEnergy_kWh;
if grossConsumption_kWh > 0
    energy.recoveredShareOfGrossConsumption = ...
        energy.recoveredEnergy_kWh / grossConsumption_kWh;
else
    energy.recoveredShareOfGrossConsumption = NaN;
end

energy.peakBatteryDischargePower_kW = max(0,max(flow.terminalBatteryPower_W)) / 1e3;
energy.peakBatteryChargePower_kW = max(0, -min(flow.terminalBatteryPower_W)) / 1e3;
energy.peakBatteryDischargeCurrent_A = ...
    energy.peakBatteryDischargePower_kW * 1e3 ...
    / vehicle.energy.nominalAccumulatorVoltage_V;
energy.peakBatteryChargeCurrent_A = ...
    energy.peakBatteryChargePower_kW * 1e3 ...
    / vehicle.energy.nominalAccumulatorVoltage_V;
energy.batteryDischargePowerLimit_kW = ...
    vehicle.energy.maximumBatteryDischargePower_W / 1e3;
energy.batteryDischargePowerUtilization = max( ...
    flow.terminalBatteryPower_W ./ max(flow.dischargePowerLimit_W,eps));
energy.exceedsBatteryDischargePowerLimit = ...
    energy.powerLimitedSegmentCount > 0;

availableCapacityFraction = vehicle.energy.usableSocWindow ...
    * (1 - vehicle.energy.finishEnergyReserveFraction) ...
    * vehicle.energy.endOfLifeCapacityRetention;
energy.requiredInstalledCapacity_kWh = energy.requiredUsableEnergy_kWh ...
    / availableCapacityFraction;
energy.requiredNominalCapacity_Ah = energy.requiredInstalledCapacity_kWh ...
    * 1e3 / vehicle.energy.nominalAccumulatorVoltage_V;
energy.estimatedAccumulatorMass_kg = energy.requiredInstalledCapacity_kWh ...
    * 1e3 / vehicle.energy.packSpecificEnergy_Whpkg;

energy.profile = flow;
energy.profile.segmentLength_m = sequence.length_m;
energy.profile.segmentTime_s = sequence.time_s;
energy.profile.eventDistance_m = cumsum(sequence.length_m) ...
    - sequence.length_m / 2;
energy.profile.sourceSegmentIndex = sequence.sourceIndex;
energy.profile.segmentSpeed_mps = sequence.speed_mps;
energy.profile.wheelForce_N = sequence.wheelForce_N;
energy.profile.wheelPower_W = ...
    sequence.wheelForce_N .* sequence.speed_mps;
energy.profile.batteryPower_W = flow.batteryPower_W;
energy.profile.dischargePowerLimit_W = flow.dischargePowerLimit_W;
energy.profile.chargePowerLimit_W = flow.chargePowerLimit_W;
energy.profile.regenMechanicalPower_W = ...
    flow.regenMechanical_J ./ sequence.time_s;
energy.profile.driveEfficiency = flow.driveEfficiency;
energy.profile.regenEfficiency = flow.regenEfficiency;
energy.profile.soc = flow.socBoundary(1:end-1);
energy.profile.socBoundary = flow.socBoundary;
assert(abs(sum(flow.batteryPower_W.*sequence.time_s)/3.6e6-energy.netBatteryEnergy_kWh)<1e-9, ...
    'lts:energy:EnergyBalance','Integrated power and net energy disagree.');
end

function flow = simulateElectricalFlow(sequence, vehicle)
count = numel(sequence.length_m);
socBoundary = zeros(count + 1, 1);
socBoundary(1) = vehicle.energy.initialSoc;
fields = {'tractionMechanical_J', 'brakingMechanical_J', ...
    'tractionBattery_J', 'auxiliary_J', 'recoveredBattery_J', ...
    'potentialRecovered_J', 'frictionBrake_J', 'netBattery_J', ...
    'powerLimitViolation_J', 'capacityShortfall_J', ...
    'batteryPower_W', 'dischargePowerLimit_W', 'chargePowerLimit_W', ...
    'regenMechanical_J', 'driveEfficiency', 'regenEfficiency', ...
    'motorSpeed_rpm','motorTorque_Nm','motorShaftPower_W','motorMapClamped', ...
    'terminalBatteryPower_W','terminalTraction_J','terminalRecovered_J'};
for fieldIndex = 1:numel(fields)
    flow.(fields{fieldIndex}) = zeros(count, 1);
end

for index = 1:count
    soc = socBoundary(index);
    step = lts.energy.propagateSegment(vehicle, soc, ...
        sequence.speed_mps(index), sequence.wheelForce_N(index), ...
        sequence.length_m(index), sequence.time_s(index));
    socBoundary(index + 1) = step.nextSoc;
    for fieldIndex = 1:numel(fields)
        flow.(fields{fieldIndex})(index) = step.(fields{fieldIndex});
    end
end
flow.socBoundary = socBoundary;
end

function sequence = expandSequence(length_m, time_s, speed_mps, ...
        wheelForce_N, wheelWork_J, selectionWeight, repetitions)
fullRepetitions = floor(repetitions);
fractions = ones(1, fullRepetitions);
remainder = repetitions - fullRepetitions;
if remainder > 1e-12
    fractions(end + 1) = remainder;
end
sourceIndex = zeros(0, 1);
weights = zeros(0, 1);
for repetitionIndex = 1:numel(fractions)
    active = find(selectionWeight > 0);
    sourceIndex = [sourceIndex; active]; %#ok<AGROW>
    weights = [weights; selectionWeight(active) ...
        * fractions(repetitionIndex)]; %#ok<AGROW>
end
sequence.sourceIndex = sourceIndex;
sequence.length_m = length_m(sourceIndex) .* weights;
sequence.time_s = time_s(sourceIndex) .* weights;
sequence.speed_mps = speed_mps(sourceIndex);
sequence.wheelForce_N = wheelForce_N(sourceIndex);
sequence.wheelWork_J = wheelWork_J(sourceIndex) .* weights;
end

function weights = timingWeights(distance_m, windows_m, aggregation)
segmentStart_m = distance_m(1:end-1);
segmentEnd_m = distance_m(2:end);
segmentLength_m = segmentEnd_m - segmentStart_m;
if aggregation == "fullTrack"
    weights = ones(size(segmentLength_m));
    return
end
weights = zeros(size(segmentLength_m));
for windowIndex = 1:size(windows_m, 1)
    overlap_m = max(0, min(segmentEnd_m, windows_m(windowIndex, 2)) ...
        - max(segmentStart_m, windows_m(windowIndex, 1)));
    weights = weights + overlap_m ./ segmentLength_m;
end
if aggregation == "meanWindows"
    weights = weights / size(windows_m, 1);
end
end

function validateTiming(windows_m, aggregation, lapResult)
validAggregations = ["fullTrack", "sumWindows", "meanWindows"];
assert(any(aggregation == validAggregations), 'lts:energy:Aggregation', ...
    'Unknown energy aggregation: %s.', aggregation);
assert(all(windows_m(:, 2) > windows_m(:, 1)), ...
    'lts:energy:TimingWindows', ...
    'Every distance window must have increasing boundaries.');
if aggregation ~= "fullTrack"
    assert(~isempty(windows_m), 'lts:energy:TimingWindows', ...
        'Window aggregation requires distance windows.');
    assert(all(windows_m(:) >= lapResult.distance_m(1)) ...
        && all(windows_m(:) <= lapResult.distance_m(end)), ...
        'lts:energy:TimingWindows', ...
        'A distance window lies outside the lap result.');
end
end

function validateLapResult(lapResult)
required = {'distance_m', 'time_s', 'speed_mps', ...
    'segmentAcceleration_mps2'};
for index = 1:numel(required)
    assert(isfield(lapResult, required{index}), 'lts:energy:LapResult', ...
        'Missing lap-result field: %s.', required{index});
end
assert(numel(lapResult.distance_m) >= 2 ...
    && numel(lapResult.time_s) == numel(lapResult.distance_m) ...
    && numel(lapResult.speed_mps) == numel(lapResult.distance_m) ...
    && numel(lapResult.segmentAcceleration_mps2) ...
        == numel(lapResult.distance_m) - 1, ...
    'lts:energy:LapResult', 'Lap-result vectors have incompatible sizes.');
assert(all(diff(lapResult.distance_m) > 0) ...
    && all(diff(lapResult.time_s) > 0), ...
    'lts:energy:LapResult', ...
    'Lap-result distance and time must increase strictly.');
end
