function peaks = peakForces(envelope, normalLoad_N, camberAngle_rad)
%PEAKFORCES Fast pure longitudinal and lateral peak forces.

arguments
    envelope (1, 1) struct
    normalLoad_N double {mustBeReal, mustBeFinite, mustBeNonnegative}
    camberAngle_rad double {mustBeReal, mustBeFinite} = 0
end

if isscalar(camberAngle_rad)
    camberAngle_rad = camberAngle_rad + zeros(size(normalLoad_N));
end
assert(isequal(size(camberAngle_rad), size(normalLoad_N)), ...
    'lts:tire:PeakShape', 'Camber and normal load must have equal size.');

switch envelope.type
    case 'quadraticDirectional'
        peaks = lts.tire.evaluateDirectionalEnvelope(envelope,normalLoad_N);
    case 'fittedDirectionalPeak'
        fit = envelope.parameters;
        sourceCamber_deg = -rad2deg(camberAngle_rad);
        sourceNormalLoad_N = -normalLoad_N;
        lateral_N = fit.lateralScale .* evaluateRow( ...
            fit.coefficients(2, :), sourceCamber_deg, sourceNormalLoad_N);
        longitudinal_N = fit.longitudinalScale .* evaluateRow( ...
            fit.coefficients(10, :), sourceCamber_deg, sourceNormalLoad_N);
        peaks.lateral_N = abs(lateral_N);
        peaks.longitudinal_N = abs(longitudinal_N);
        peaks.drive_N = peaks.longitudinal_N;
        peaks.brake_N = peaks.longitudinal_N;
    case 'loadSensitiveSymmetric'
        force_N = lts.tire.maximumForce(envelope.fallback, normalLoad_N);
        peaks.lateral_N = force_N;
        peaks.longitudinal_N = force_N;
        peaks.drive_N = force_N;
        peaks.brake_N = force_N;
    otherwise
        error('lts:tire:PeakEnvelope', ...
            'Unknown peak-envelope type: %s.', envelope.type);
end
peaks.lateral_N(normalLoad_N == 0) = 0;
peaks.longitudinal_N(normalLoad_N == 0) = 0;
peaks.drive_N(normalLoad_N == 0) = 0;
peaks.brake_N(normalLoad_N == 0) = 0;
end

function value = evaluateRow(row, camber, normalLoad)
value = row(1) + row(2) .* camber + row(3) .* normalLoad ...
    + row(4) .* camber.^2 + row(5) .* camber .* normalLoad ...
    + row(6) .* normalLoad.^2 + row(7) .* exp(row(8) .* camber) ...
    + row(9) .* camber + row(10) .* exp(row(11) .* normalLoad) ...
    + row(12) .* normalLoad + row(13);
end
