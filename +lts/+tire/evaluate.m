function forces = evaluate(model, normalLoad_N, camberAngle_rad, ...
        slipRatio, slipAngle_rad, speed_mps)
%EVALUATE Evaluate one stationary tire operating point in SI units.

arguments
    model (1, 1) struct
    normalLoad_N (1, 1) double {mustBePositive, mustBeFinite}
    camberAngle_rad (1, 1) double {mustBeFinite}
    slipRatio (1, 1) double {mustBeFinite}
    slipAngle_rad (1, 1) double {mustBeFinite}
    speed_mps (1, 1) double {mustBePositive, mustBeFinite}
end

switch model.type
    case 'fittedCombinedSlip'
        forces = lts.tire.fittedCombinedSlip.evaluate(model.parameters, ...
            normalLoad_N, camberAngle_rad, slipRatio, slipAngle_rad);
    case 'tmeasyAdtm'
        forces = lts.tire.tmeasyAdtm.evaluate(model.parameters, ...
            normalLoad_N, camberAngle_rad, slipRatio, slipAngle_rad, ...
            speed_mps, model.muScale);
    otherwise
        error('lts:tire:UnknownModel', 'Unknown tire model: %s.', model.type);
end
end
