function parameters = magicFormulaParameters(fit, normalLoad_N, camberAngle_rad)
%MAGICFORMULAPARAMETERS Evaluate named pure-slip parameters at one wheel.

validateattributes(normalLoad_N, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(camberAngle_rad, {'numeric'}, ...
    {'scalar', 'real', 'finite'});
requiredFields = {'coefficients', 'lateralScale', 'longitudinalScale'};
for index = 1:numel(requiredFields)
    if ~isfield(fit, requiredFields{index})
        error('lts:tire:InvalidFit', 'Missing fit field %s.', requiredFields{index});
    end
end

sourceCamber_deg = -rad2deg(camberAngle_rad);
sourceNormalLoad_N = -normalLoad_N;
parameters.lateral = evaluateDirection(fit.coefficients, 2, ...
    sourceCamber_deg, sourceNormalLoad_N, fit.lateralScale, true);
parameters.longitudinal = evaluateDirection(fit.coefficients, 10, ...
    sourceCamber_deg, sourceNormalLoad_N, fit.longitudinalScale, false);
end

function direction = evaluateDirection(T, firstRow, camber_deg, normalLoad_N, scale, isAngle)
peakForce_N = evaluateRow(T(firstRow, :), camber_deg, normalLoad_N);
peakSlipSource = evaluateRow(T(firstRow + 1, :), camber_deg, normalLoad_N);
forceAtPeak_N = evaluateRow(T(firstRow + 2, :), camber_deg, normalLoad_N);
slopeAngle_rad = evaluateRow(T(firstRow + 3, :), camber_deg, normalLoad_N);

ratio = max(min(forceAtPeak_N / peakForce_N, 0.999), -0.999);
shapeFactor = 2 - 2 / pi * asin(ratio);
stiffnessFactor = tan(slopeAngle_rad) / (shapeFactor * peakForce_N);
scaledSlip = stiffnessFactor * peakSlipSource;
denominator = scaledSlip - atan(scaledSlip);
curvatureFactor = (scaledSlip - tan(pi / (2 * shapeFactor))) / denominator;

direction.B = stiffnessFactor;
direction.C = shapeFactor;
direction.D_N = scale * peakForce_N;
direction.E = curvatureFactor;
if isAngle
    direction.peakSlip = deg2rad(peakSlipSource);
    direction.inputUnit = 'rad';
else
    direction.peakSlip = peakSlipSource;
    direction.inputUnit = 'ratio';
end
end

function value = evaluateRow(row, camber, normalLoad)
value = row(1) + row(2) * camber + row(3) * normalLoad ...
    + row(4) * camber ^ 2 + row(5) * camber * normalLoad ...
    + row(6) * normalLoad ^ 2 + row(7) * exp(row(8) * camber) ...
    + row(9) * camber + row(10) * exp(row(11) * normalLoad) ...
    + row(12) * normalLoad + row(13);
end
