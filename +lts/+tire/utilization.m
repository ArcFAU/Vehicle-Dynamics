function utilization = utilization(tire, normalLoad_N, longitudinalForce_N, lateralForce_N)
%UTILIZATION Combined-force utilization; values <= 1 are feasible.

peakForce_N = lts.tire.maximumForce(tire, normalLoad_N);
exponent = tire.combinedForceExponent;
utilization = Inf(size(peakForce_N));
loaded = peakForce_N > 0;
utilization(loaded) = (...
    (abs(longitudinalForce_N(loaded)) ./ peakForce_N(loaded)) .^ exponent ...
    + (abs(lateralForce_N(loaded)) ./ peakForce_N(loaded)) .^ exponent) ...
    .^ (1 / exponent);
zeroLoadAndForce = ~loaded & longitudinalForce_N == 0 & lateralForce_N == 0;
utilization(zeroLoadAndForce) = 0;
end
