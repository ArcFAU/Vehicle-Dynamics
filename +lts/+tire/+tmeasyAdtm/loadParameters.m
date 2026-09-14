function tir = loadParameters(parameterSet)
%LOADPARAMETERS Load and validate a named, local ADTM TMeasy parameter set.

arguments
    parameterSet (1, 1) string
end

switch parameterSet
    case "Hoosier_16x7_5_10x8_R20_82kPa"
        tir = lts.tire.tmeasyAdtm.Hoosier_16x7_5_10x8_R20_82kPa();
    otherwise
        error('lts:tire:UnknownParameterSet', ...
            'Unknown TMeasy parameter set: %s.', parameterSet);
end
lts.tire.tmeasyAdtm.validateParameters(tir);
end
