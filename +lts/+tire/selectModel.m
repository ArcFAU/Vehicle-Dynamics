function vehicle = selectModel(vehicle, modelName)
%SELECTMODEL Return a vehicle copy using the requested tire-force model.

arguments
    vehicle (1, 1) struct
    modelName (1, 1) string {mustBeMember(modelName, ...
        ["fittedCombinedSlip", "tmeasyAdtm"])}
end

vehicle.tireModel.active = char(modelName);
lts.validation.validateVehicle(vehicle);
end
