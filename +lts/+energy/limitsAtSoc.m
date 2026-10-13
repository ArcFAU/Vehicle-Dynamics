function limits = limitsAtSoc(vehicle, soc)
%LIMITSATSOC Evaluate independent drive, charge and regen SoC curves.

validateattributes(soc, {'numeric'}, {'real', 'finite', '>=', 0, '<=', 1});
if ~vehicle.energy.useSocLimits
    limits=struct('drivePowerScale',ones(size(soc)),'chargePowerScale',ones(size(soc)), ...
        'regenTorqueScale',ones(size(soc)),'regenPowerScale',ones(size(soc)), ...
        'maximumBatteryDischargePower_W',vehicle.energy.maximumBatteryDischargePower_W*ones(size(soc)), ...
        'maximumBatteryChargePower_W',vehicle.energy.maximumBatteryChargePower_W*ones(size(soc)));
    return
end
driveModel = vehicle.powertrain.driveLimitModel;
energyModel = vehicle.energy.socLimitModel;
limits.drivePowerScale = interpolateCurve(driveModel.socBreakpoints, ...
    driveModel.powerScale, soc);
limits.chargePowerScale = interpolateCurve(energyModel.socBreakpoints, ...
    energyModel.chargePowerScale, soc);
limits.regenTorqueScale = interpolateCurve(energyModel.socBreakpoints, ...
    energyModel.regenTorqueScale, soc);
limits.regenPowerScale = interpolateCurve(energyModel.socBreakpoints, ...
    energyModel.regenPowerScale, soc);
limits.maximumBatteryDischargePower_W = ...
    vehicle.energy.maximumBatteryDischargePower_W .* limits.drivePowerScale;
limits.maximumBatteryChargePower_W = ...
    vehicle.energy.maximumBatteryChargePower_W .* limits.chargePowerScale;
limits.maximumBatteryChargePower_W(limits.chargePowerScale==0)=0; % Inf times zero closes charging.
end

function value = interpolateCurve(breakpoints, values, query)
clamped = min(max(query, breakpoints(1)), breakpoints(end));
value = interp1(breakpoints, values, clamped, 'linear');
end
