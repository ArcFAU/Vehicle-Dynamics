function maximumForce_N = maximumForce(tire, normalLoad_N)
%MAXIMUMFORCE Peak pure-slip force of a load-sensitive tire envelope.
%   mu(Fz) = muRef * (Fz/FzRef)^loadSensitivityExponent
%   Fmax   = mu(Fz) * Fz

validateattributes(normalLoad_N, {'numeric'}, ...
    {'real', 'finite', 'nonnegative'});
validateattributes(tire.referenceNormalLoad_N, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(tire.frictionCoefficientAtReferenceLoad, {'numeric'}, ...
    {'scalar', 'real', 'finite', 'positive'});
validateattributes(tire.loadSensitivityExponent, {'numeric'}, ...
    {'scalar', 'real', 'finite', '<=', 0});

maximumForce_N = zeros(size(normalLoad_N));
loaded = normalLoad_N > 0;
loadRatio = normalLoad_N(loaded) ./ tire.referenceNormalLoad_N;
frictionCoefficient = tire.frictionCoefficientAtReferenceLoad ...
    .* loadRatio .^ tire.loadSensitivityExponent;
maximumForce_N(loaded) = frictionCoefficient .* normalLoad_N(loaded);
end
