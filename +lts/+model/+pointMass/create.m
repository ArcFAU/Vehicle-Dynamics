function model = create(vehicle)
%CREATE Build the common vehicle-model interface for Level 0.1.

lts.validation.validateVehicle(vehicle);
model.name = 'Level 0.1 Point Mass';
model.level = '0.1';
model.vehicle = vehicle;
model.maximumSpeedForCurvature = ...
    @(curvature_1pm) maximumSpeedForCurvature(vehicle, curvature_1pm);
model.accelerationLimits = ...
    @(speed_mps, lateralAcceleration_mps2) ...
    lts.model.pointMass.accelerationLimits(vehicle, speed_mps, ...
    lateralAcceleration_mps2);
end

function maximumSpeed_mps = maximumSpeedForCurvature(vehicle, curvature_1pm)
gravity_mps2 = 9.81;
maximumLateralAcceleration_mps2 = ...
    vehicle.tireApproximation.frictionCoefficient * gravity_mps2;
maximumSpeed_mps = Inf(size(curvature_1pm));
corner = abs(curvature_1pm) > eps;
maximumSpeed_mps(corner) = sqrt(maximumLateralAcceleration_mps2 ...
    ./ abs(curvature_1pm(corner)));
end
