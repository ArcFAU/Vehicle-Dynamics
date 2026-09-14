function capacity_N = combinedLongitudinalCapacity(tire, normalLoad_N, lateralForce_N)
%COMBINEDLONGITUDINALCAPACITY Remaining Fx inside a symmetric p-envelope.

peakForce_N = lts.tire.maximumForce(tire, normalLoad_N);
exponent = tire.combinedForceExponent;
lateralFraction = zeros(size(peakForce_N));
loaded = peakForce_N > 0;
lateralFraction(loaded) = abs(lateralForce_N(loaded)) ./ peakForce_N(loaded);
remaining = max(0, 1 - lateralFraction .^ exponent);
capacity_N = peakForce_N .* remaining .^ (1 / exponent);
capacity_N(~loaded | lateralFraction > 1) = 0;
end
