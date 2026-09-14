function results = runSet(events, model, options)
%RUNSET Simulate a selected set of manual and driverless dynamic events.

arguments
    events (1, :) struct
    model (1, 1) struct
    options.IncludeDriverless (1, 1) logical
    options.CoupleEnergy (1, 1) logical
end

results = struct('id', {}, 'name', {}, 'discipline', {}, 'mode', {}, ...
    'time_s', {}, 'energy', {}, 'energyCoupled', {}, ...
    'lapResult', {}, 'track', {});
for eventIndex = 1:numel(events)
    event = events(eventIndex);
    if ~event.enabled || (~options.IncludeDriverless ...
            && strcmp(event.mode, 'driverless'))
        continue
    end
    validateEvent(event);
    track = lts.io.loadTrackMatrix(string(event.track.file), ...
        event.track.isClosed);
    canCouple = options.CoupleEnergy && isfield(model, 'vehicle') ...
        && isfield(model.vehicle, 'energy') ...
        && strcmp(event.timing.aggregation, 'fullTrack') ...
        && event.timing.multiplier == round(event.timing.multiplier);
    if options.CoupleEnergy && ~canCouple
        error('lts:events:EnergyCouplingUnavailable', ...
            ['Energy coupling was explicitly requested for event "%s", ' ...
            'but requires model.vehicle.energy, fullTrack timing and an ' ...
            'integer repetition multiplier. No uncoupled fallback was run.'], ...
            event.id);
    end
    if canCouple
        coupledInitialSpeed_mps = event.solver.initialSpeed_mps;
        if track.isClosed
            coupledInitialSpeed_mps = NaN;
        end
        lapResult = lts.solver.solveLapWithEnergy(track, model, ...
            Repetitions=round(event.timing.multiplier), ...
            InitialSoc=model.vehicle.energy.initialSoc, ...
            InitialSpeed_mps=coupledInitialSpeed_mps, ...
            FinalSpeed_mps=event.solver.finalSpeed_mps, ...
            MaximumSpeed_mps=event.solver.maximumSpeed_mps);
        eventTime_s = lapResult.lapTime_s;
        energy = lapResult.energy;
    else
        lapResult = lts.solver.solveLap(track, model, ...
            initialSpeed_mps=event.solver.initialSpeed_mps, ...
            finalSpeed_mps=event.solver.finalSpeed_mps, ...
            maximumSpeed_mps=event.solver.maximumSpeed_mps);
        eventTime_s = aggregateTime(lapResult, event.timing);
        energy = unavailableEnergy();
    end
    if ~canCouple && isfield(model, 'vehicle') ...
            && isfield(model.vehicle, 'energy')
        energy = lts.energy.analyzeLap(lapResult, model.vehicle, ...
            DistanceWindows_m=event.timing.windows_m, ...
            Aggregation=string(event.timing.aggregation), ...
            Repetitions=event.timing.multiplier);
    end

    outputIndex = numel(results) + 1;
    results(outputIndex).id = event.id;
    results(outputIndex).name = event.name;
    results(outputIndex).discipline = event.discipline;
    results(outputIndex).mode = event.mode;
    results(outputIndex).time_s = eventTime_s;
    results(outputIndex).energy = energy;
    results(outputIndex).energyCoupled = canCouple;
    results(outputIndex).lapResult = lapResult;
    results(outputIndex).track = track.meta;
end

function energy = unavailableEnergy()
energy.status = 'not available for this vehicle model';
energy.netBatteryEnergy_kWh = NaN;
energy.requiredInstalledCapacity_kWh = NaN;
end
end

function eventTime_s = aggregateTime(lapResult, timing)
switch timing.aggregation
    case 'fullTrack'
        baseTime_s = lapResult.lapTime_s;
    case {'sumWindows', 'meanWindows'}
        assert(all(timing.windows_m(:) >= lapResult.distance_m(1)) ...
            && all(timing.windows_m(:) <= lapResult.distance_m(end)), ...
            'lts:events:TimingWindows', ...
            'A timing window lies outside the track distance.');
        boundaryTime_s = interp1(lapResult.distance_m, lapResult.time_s, ...
            timing.windows_m(:), 'linear');
        windowTime_s = reshape(boundaryTime_s, size(timing.windows_m));
        windowTime_s = windowTime_s(:, 2) - windowTime_s(:, 1);
        if strcmp(timing.aggregation, 'sumWindows')
            baseTime_s = sum(windowTime_s);
        else
            baseTime_s = mean(windowTime_s);
        end
    otherwise
        error('lts:events:TimingAggregation', ...
            'Unknown timing aggregation: %s.', timing.aggregation);
end
eventTime_s = baseTime_s * timing.multiplier;
end

function validateEvent(event)
requiredPaths = {'id', 'name', 'discipline', 'mode', 'enabled', ...
    'track.file', 'track.isClosed', 'timing.windows_m', ...
    'timing.aggregation', 'timing.multiplier', ...
    'solver.initialSpeed_mps', 'solver.finalSpeed_mps', ...
    'solver.maximumSpeed_mps'};
for index = 1:numel(requiredPaths)
    assert(hasFieldPath(event, requiredPaths{index}), ...
        'lts:events:Definition', 'Missing event field: %s.', ...
        requiredPaths{index});
end
assert(isfile(event.track.file), 'lts:events:TrackFile', ...
    'Track file does not exist: %s.', event.track.file);
end

function present = hasFieldPath(value, fieldPath)
parts = split(fieldPath, '.');
present = true;
for index = 1:numel(parts)
    if ~isstruct(value) || ~isfield(value, parts{index})
        present = false;
        return
    end
    value = value.(parts{index});
end
end
