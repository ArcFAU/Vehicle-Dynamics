function usage = directionalUtilization(envelope, normalLoad_N, ...
    longitudinalForce_N, lateralForce_N, camberAngle_rad)
%DIRECTIONALUTILIZATION Combined usage with separate Fx/Fy peak forces.

arguments
    envelope (1, 1) struct
    normalLoad_N double
    longitudinalForce_N double
    lateralForce_N double
    camberAngle_rad double = 0
end

peaks = lts.tire.peakForces(envelope, normalLoad_N, camberAngle_rad);
assert(isequal(size(normalLoad_N), size(longitudinalForce_N), ...
    size(lateralForce_N)), 'lts:tire:UtilizationShape', ...
    'Normal load and force arrays must have equal size.');
usage = Inf(size(normalLoad_N));
loaded = peaks.longitudinal_N > 0 & peaks.lateral_N > 0;
p = envelope.combinedForceExponent;
usage(loaded) = ((abs(longitudinalForce_N(loaded)) ...
    ./ peaks.longitudinal_N(loaded)).^p ...
    + (abs(lateralForce_N(loaded)) ./ peaks.lateral_N(loaded)).^p) ...
    .^ (1 / p);
zero = ~loaded & longitudinalForce_N == 0 & lateralForce_N == 0;
usage(zero) = 0;
end
