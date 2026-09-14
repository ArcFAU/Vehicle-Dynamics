function [feasible, point] = isFeasible(vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, options)
%ISFEASIBLE Robust scalar feasibility query for boundary searches.

arguments
    vehicle (1, 1) struct
    speed_mps (1, 1) double
    longitudinalAcceleration_mps2 (1, 1) double
    lateralAcceleration_mps2 (1, 1) double
    options.enableRetries (1, 1) logical = false
    options.targetYawMoment_Nm (1, 1) double = 0
    options.initialUnknowns double = []
end

try
    point = lts.model.dualTrackSlip.operatingPoint(vehicle, speed_mps, ...
        longitudinalAcceleration_mps2, lateralAcceleration_mps2, ...
        enableRetries=options.enableRetries, ...
        targetYawMoment_Nm=options.targetYawMoment_Nm, ...
        initialUnknowns=options.initialUnknowns);
    feasible = point.feasible;
catch exception
    expectedIdentifiers = ["lts:model:WheelLift", ...
        "optimlib:lsqncommon:UsrObjUndefAtX0"];
    if any(string(exception.identifier) == expectedIdentifiers)
        feasible = false;
        point = struct.empty;
    else
        rethrow(exception);
    end
end
end
