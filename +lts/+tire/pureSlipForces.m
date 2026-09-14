function forces = pureSlipForces(fit, normalLoad_N, camberAngle_rad, slipRatio, slipAngle_rad)
%PURESLIPFORCES Pure longitudinal and lateral forces for one wheel.
% Positive slip ratio creates positive Fx. Positive slip angle creates
% positive Fy. Normal load is positive in compression; angles use radians.

validateattributes(slipRatio, {'numeric'}, {'scalar', 'real', 'finite'});
validateattributes(slipAngle_rad, {'numeric'}, {'scalar', 'real', 'finite'});
parameters = lts.tire.magicFormulaParameters(fit, normalLoad_N, camberAngle_rad);
forces.longitudinal_N = evaluateFormula(parameters.longitudinal, slipRatio);
forces.lateral_N = evaluateFormula(parameters.lateral, rad2deg(slipAngle_rad));
forces.parameters = parameters;
end

function force_N = evaluateFormula(p, slip)
argument = p.B * slip;
force_N = p.D_N * sin(p.C * atan(argument ...
    - p.E * (argument - atan(argument))));
end
