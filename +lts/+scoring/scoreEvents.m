function summary = scoreEvents(eventResults, ruleSet, options)
%SCOREEVENTS Apply a versioned scoring basis to simulated event results.

arguments
    eventResults (1, :) struct
    ruleSet (1, 1) struct
    options.IncludeDriverless (1, 1) logical
end

scores = struct('eventId', {}, 'name', {}, 'mode', {}, 'time_s', {}, ...
    'efficiencyFactor_s2kWh', {}, 'points', {}, 'maximumPoints', {}, ...
    'method', {}, 'status', {});
for resultIndex = 1:numel(eventResults)
    result = eventResults(resultIndex);
    if ~options.IncludeDriverless && strcmp(result.mode, 'driverless')
        continue
    end
    ruleIndices = find(strcmp({ruleSet.events.eventId}, result.id));
    assert(~isempty(ruleIndices), 'lts:scoring:MissingRule', ...
        'No scoring rule exists for simulated event "%s".', result.id);
    assert(isscalar(ruleIndices), 'lts:scoring:DuplicateRule', ...
        'Multiple scoring rules exist for simulated event "%s".', result.id);
    ruleIndex = ruleIndices(1);
    rule = ruleSet.events(ruleIndex);
    [points, status] = scoreOne(result.time_s, rule);
    outputIndex = numel(scores) + 1;
    scores(outputIndex).eventId = result.id;
    scores(outputIndex).name = rule.name;
    scores(outputIndex).mode = rule.mode;
    scores(outputIndex).time_s = result.time_s;
    scores(outputIndex).efficiencyFactor_s2kWh = NaN;
    scores(outputIndex).points = points;
    scores(outputIndex).maximumPoints = rule.maximumPoints;
    scores(outputIndex).method = rule.method;
    scores(outputIndex).status = status;
end

efficiencyRuleIndex = find(strcmp({ruleSet.events.method}, ...
    'efficiencyFactor'), 1);
enduranceResultIndex = find(strcmp({eventResults.id}, 'endurance'), 1);
if ~isempty(efficiencyRuleIndex) && ~isempty(enduranceResultIndex)
    rule = ruleSet.events(efficiencyRuleIndex);
    result = eventResults(enduranceResultIndex);
    [points, factor_s2kWh, status] = scoreEfficiency(result, rule);
    outputIndex = numel(scores) + 1;
    scores(outputIndex).eventId = rule.eventId;
    scores(outputIndex).name = rule.name;
    scores(outputIndex).mode = rule.mode;
    scores(outputIndex).time_s = result.time_s;
    scores(outputIndex).efficiencyFactor_s2kWh = factor_s2kWh;
    scores(outputIndex).points = points;
    scores(outputIndex).maximumPoints = rule.maximumPoints;
    scores(outputIndex).method = rule.method;
    scores(outputIndex).status = status;
end

pointValues = [scores.points];
assert(~isempty(scores), 'lts:scoring:NoScoredEvents', ...
    'No enabled event result was available for scoring.');
summary.meta = ruleSet.meta;
summary.events = scores;
summary.scoredPointsSubtotal = sum(pointValues(isfinite(pointValues)));
summary.maximumAvailablePoints = sum([scores.maximumPoints]);
summary.isComplete = all(isfinite(pointValues));
if summary.isComplete
    summary.totalPoints = summary.scoredPointsSubtotal;
else
    summary.totalPoints = NaN;
end

function [points, factor_s2kWh, status] = scoreEfficiency(result, rule)
points = NaN;
factor_s2kWh = NaN;
if ~isfield(result, 'energy') ...
        || ~isfield(result.energy, 'netBatteryEnergy_kWh') ...
        || ~isfinite(result.energy.netBatteryEnergy_kWh)
    status = 'missing energy result';
    return
end
factor_s2kWh = result.time_s ^ 2 * result.energy.netBatteryEnergy_kWh;
minimumFactor = rule.reference.minimumFactor_s2kWh;
if ~isfinite(minimumFactor) || minimumFactor <= 0
    status = 'missing annual minimum efficiency factor';
    return
end
maximumFactor = rule.cutoffFactor * minimumFactor;
cappedFactor = min(factor_s2kWh, maximumFactor);
points = rule.maximumPoints ...
    * ((maximumFactor - cappedFactor) ...
    / (maximumFactor - minimumFactor)) ^ rule.exponent;
points = min(rule.maximumPoints, max(0, points));
status = 'scored';
end
end

function [points, status] = scoreOne(teamTime_s, rule)
validateattributes(teamTime_s, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'}, mfilename, 'teamTime_s');
switch rule.method
    case 'timeCurve'
        bestTime_s = rule.reference.bestTime_s;
        if ~isfinite(bestTime_s) || bestTime_s <= 0
            points = NaN;
            status = 'missing annual best time';
            return
        end
        maximumTime_s = rule.cutoffFactor * bestTime_s;
        cappedTime_s = min(teamTime_s, maximumTime_s);
        minimumPoints = rule.minimumPointsFraction * rule.maximumPoints;
        normalizedMargin = (maximumTime_s - cappedTime_s) ...
            / (maximumTime_s - bestTime_s);
        points = (rule.maximumPoints - minimumPoints) ...
            * normalizedMargin ^ rule.exponent + minimumPoints;
        points = min(rule.maximumPoints, max(minimumPoints, points));
        status = 'scored';
    case 'rankLinear'
        [rank, participantCount] = deriveRank(teamTime_s, rule.reference);
        if ~isfinite(rank) || ~isfinite(participantCount)
            points = NaN;
            status = 'missing annual ranking data';
            return
        end
        points = rule.maximumPoints ...
            * (participantCount + 1 - rank) / participantCount;
        points = min(rule.maximumPoints, max(0, points));
        status = 'scored';
    otherwise
        error('lts:scoring:Method', 'Unknown scoring method: %s.', ...
            rule.method);
end
end

function [rank, participantCount] = deriveRank(teamTime_s, reference)
rank = NaN;
participantCount = NaN;
if isfield(reference, 'competitorTimes_s') ...
        && ~isempty(reference.competitorTimes_s)
    competitorTimes_s = reference.competitorTimes_s;
    competitorTimes_s = competitorTimes_s(isfinite(competitorTimes_s) ...
        & competitorTimes_s > 0);
    rank = 1 + sum(competitorTimes_s < teamTime_s);
    participantCount = numel(competitorTimes_s) + 1;
elseif isfield(reference, 'rank') ...
        && isfield(reference, 'participantCount')
    rank = reference.rank;
    participantCount = reference.participantCount;
end
end
