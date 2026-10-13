function ruleSet = fsg2026(references)
%FSG2026 Versioned FSG 2026 dynamic scoring configuration.
% Update only references when new event results become available. Manual
% events use D9.1 time curves. Autonomous acceleration and skidpad use the
% D9.2 ranking formula and therefore require competitor times or rank data.

arguments
    references (1, 1) struct = struct()
end

ruleSet.meta.organizer = 'Formula Student Germany';
ruleSet.meta.year = 2026;
ruleSet.meta.rulesVersion = 'FS Rules 2026 v1.1';
ruleSet.meta.source = ...
    'https://www.formulastudent.de/fileadmin/user_upload/all/2026/rules/FS-Rules_2026_v1.1.pdf';
ruleSet.meta.referenceNote = ...
    'Annual best times and ranking data are external, replaceable inputs.';

ruleSet.events(1) = timeRule('acceleration', 'Acceleration', 'manual', ...
    50, 1.70, 0.05, readBestTime(references, 'acceleration'));
ruleSet.events(2) = timeRule('skidpad', 'Skidpad', 'manual', ...
    50, 1.35, 0.05, readBestTime(references, 'skidpad'));
ruleSet.events(3) = timeRule('autocross', 'Autocross', 'manual', ...
    100, 1.40, 0.10, readBestTime(references, 'autocross'));
ruleSet.events(4) = timeRule('endurance', 'Endurance', 'manual', ...
    250, 1.50, 0.10, readBestTime(references, 'endurance'));
ruleSet.events(5) = rankRule('dvAcceleration', ...
    'Driverless Acceleration', 75, ...
    readRankingReference(references, 'dvAcceleration'));
ruleSet.events(6) = rankRule('dvSkidpad', ...
    'Driverless Skidpad', 75, ...
    readRankingReference(references, 'dvSkidpad'));
ruleSet.events(7) = efficiencyRule( ...
    readEfficiencyReference(references));
end

function rule = efficiencyRule(reference)
rule.eventId = 'efficiency';
rule.name = 'Efficiency';
rule.mode = 'manual';
rule.method = 'efficiencyFactor';
rule.maximumPoints = 75;
rule.cutoffFactor = 2;
rule.minimumPointsFraction = 0;
rule.exponent = 2;
rule.reference = reference;
end

function rule = timeRule(eventId, name, mode, maximumPoints, ...
        cutoffFactor, minimumPointsFraction, bestTime_s)
rule.eventId = eventId;
rule.name = name;
rule.mode = mode;
rule.method = 'timeCurve';
rule.maximumPoints = maximumPoints;
rule.cutoffFactor = cutoffFactor;
rule.minimumPointsFraction = minimumPointsFraction;
rule.exponent = 2;
rule.reference = struct('bestTime_s', bestTime_s);
end

function reference = readEfficiencyReference(references)
reference = struct('minimumFactor_s2kWh', NaN, 'regenEnergyCredit', 0.9);
if isfield(references,'efficiency') && isfield(references.efficiency,'regenEnergyCredit')
    reference.regenEnergyCredit=references.efficiency.regenEnergyCredit;
end
if isfield(references, 'efficiency') ...
        && isfield(references.efficiency, 'minimumFactor_s2kWh')
    reference.minimumFactor_s2kWh = ...
        references.efficiency.minimumFactor_s2kWh;
end
end

function rule = rankRule(eventId, name, maximumPoints, reference)
rule.eventId = eventId;
rule.name = name;
rule.mode = 'driverless';
rule.method = 'rankLinear';
rule.maximumPoints = maximumPoints;
rule.cutoffFactor = NaN;
rule.minimumPointsFraction = NaN;
rule.exponent = NaN;
rule.reference = reference;
end

function bestTime_s = readBestTime(references, field)
bestTime_s = NaN;
if isfield(references, field) ...
        && isfield(references.(field), 'bestTime_s')
    bestTime_s = references.(field).bestTime_s;
end
end

function reference = readRankingReference(references, field)
reference = struct('competitorTimes_s', [], 'rank', NaN, ...
    'participantCount', NaN);
if isfield(references, field)
    supplied = references.(field);
    names = fieldnames(supplied);
    for index = 1:numel(names)
        reference.(names{index}) = supplied.(names{index});
    end
end
end
