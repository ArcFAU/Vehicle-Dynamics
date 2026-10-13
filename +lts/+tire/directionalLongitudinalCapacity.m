function capacity_N = directionalLongitudinalCapacity(envelope, ...
    normalLoad_N, lateralForce_N, camberAngle_rad, operation)
%DIRECTIONALLONGITUDINALCAPACITY Remaining Fx with directional peaks.

arguments
    envelope (1, 1) struct
    normalLoad_N double
    lateralForce_N double
    camberAngle_rad double = 0
    operation (1,1) string {mustBeMember(operation,["drive","brake"])} = "drive"
end

peaks = lts.tire.peakForces(envelope, normalLoad_N, camberAngle_rad);
p = envelope.combinedForceExponent;
lateralFraction = Inf(size(normalLoad_N));
loaded = peaks.lateral_N > 0 & peaks.longitudinal_N > 0;
lateralFraction(loaded) = abs(lateralForce_N(loaded)) ...
    ./ peaks.lateral_N(loaded);
remaining = max(0, 1 - lateralFraction.^p);
longitudinalPeak_N = peaks.drive_N;
if operation == "brake", longitudinalPeak_N = peaks.brake_N; end
capacity_N = longitudinalPeak_N .* remaining.^(1 / p);
capacity_N(~loaded | lateralFraction > 1) = 0;
end
