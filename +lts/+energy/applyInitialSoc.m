function vehicle = applyInitialSoc(vehicle, initialSoc)
%APPLYINITIALSOC Set energy start SoC and the laptime derating design point.

validateattributes(initialSoc, {'numeric'}, ...
    {'scalar', 'real', 'finite', '>=', 0, '<=', 1}, ...
    mfilename, 'initialSoc');
vehicle.energy.initialSoc = initialSoc;
vehicle.powertrain.driveLimitModel.evaluationSoc = initialSoc;
end
