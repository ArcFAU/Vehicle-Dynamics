function coverage = operatingPointCoverage(results, options)
%OPERATINGPOINTCOVERAGE Aggregate actually driven states across cases.

arguments
    results (1, :) cell
    options.SpeedEdges_mps (1, :) double = 0:2:50
    options.LongitudinalAccelerationEdges_mps2 (1, :) double = -20:1:20
    options.LateralAccelerationEdges_mps2 (1, :) double = -20:1:20
end
speed = [];
ax = [];
ay = [];
sourceCase = [];
for index = 1:numel(results)
    lap = results{index}.lap;
    midpointSpeed = (lap.speed_mps(1:end-1) + lap.speed_mps(2:end)) / 2;
    midpointAy = (lap.lateralAcceleration_mps2(1:end-1) ...
        + lap.lateralAcceleration_mps2(2:end)) / 2;
    count = numel(lap.segmentAcceleration_mps2);
    speed = [speed; midpointSpeed(:)]; %#ok<AGROW>
    ax = [ax; lap.segmentAcceleration_mps2(:)]; %#ok<AGROW>
    ay = [ay; midpointAy(:)]; %#ok<AGROW>
    sourceCase = [sourceCase; index * ones(count, 1)]; %#ok<AGROW>
end
coverage.speed_mps = speed;
coverage.longitudinalAcceleration_mps2 = ax;
coverage.lateralAcceleration_mps2 = ay;
coverage.sourceCase = sourceCase;
coverage.histogramAxAy = histcounts2(ax, ay, ...
    options.LongitudinalAccelerationEdges_mps2, ...
    options.LateralAccelerationEdges_mps2);
coverage.histogramSpeedAy = histcounts2(speed, ay, ...
    options.SpeedEdges_mps, options.LateralAccelerationEdges_mps2);
coverage.meta.caseCount = numel(results);
coverage.meta.sampleCount = numel(speed);
coverage.meta.purpose = ...
    'Driven-state coverage; an optional performance envelope can be overlaid later.';
end
