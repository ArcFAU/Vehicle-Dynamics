function event = createDefinition(options)
%CREATEDEFINITION Create one readable, validated dynamic-event definition.

arguments
    options.Id (1, 1) string
    options.Name (1, 1) string
    options.Discipline (1, 1) string
    options.Mode (1, 1) string = "manual"
    options.TrackFile (1, 1) string
    options.IsClosed (1, 1) logical
    options.TimingWindows_m (:, 2) double = zeros(0, 2)
    options.TimingAggregation (1, 1) string = "fullTrack"
    options.TimeMultiplier (1, 1) double {mustBePositive} = 1
    options.InitialSpeed_mps (1, 1) double {mustBeNonnegative} = 0
    options.FinalSpeed_mps (1, 1) double {mustBeNonnegative} = 100
    options.MaximumSpeed_mps (1, 1) double {mustBePositive} = 100
    options.Enabled (1, 1) logical = true
end

validModes = ["manual", "driverless"];
validAggregations = ["fullTrack", "sumWindows", "meanWindows"];
assert(any(options.Mode == validModes), 'lts:events:Mode', ...
    'Mode must be manual or driverless.');
assert(any(options.TimingAggregation == validAggregations), ...
    'lts:events:TimingAggregation', ...
    'Unknown timing aggregation: %s.', options.TimingAggregation);
assert(all(options.TimingWindows_m(:, 2) > options.TimingWindows_m(:, 1)), ...
    'lts:events:TimingWindows', ...
    'Every timing window must have increasing start and end distances.');
if options.TimingAggregation ~= "fullTrack"
    assert(~isempty(options.TimingWindows_m), 'lts:events:TimingWindows', ...
        'Window aggregation requires at least one timing window.');
end

event.id = char(options.Id);
event.name = char(options.Name);
event.discipline = char(options.Discipline);
event.mode = char(options.Mode);
event.enabled = options.Enabled;
event.track.file = char(options.TrackFile);
event.track.format = 'distanceCurvatureMatrix';
event.track.isClosed = options.IsClosed;
event.timing.windows_m = options.TimingWindows_m;
event.timing.aggregation = char(options.TimingAggregation);
event.timing.multiplier = options.TimeMultiplier;
event.solver.initialSpeed_mps = options.InitialSpeed_mps;
event.solver.finalSpeed_mps = options.FinalSpeed_mps;
event.solver.maximumSpeed_mps = options.MaximumSpeed_mps;
end
