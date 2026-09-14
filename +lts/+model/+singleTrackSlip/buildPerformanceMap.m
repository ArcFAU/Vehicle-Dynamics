function map = buildPerformanceMap(vehicle, options)
%BUILDPERFORMANCEMAP Reuse the common slip map with equal axle-side loads.

arguments
    vehicle (1, 1) struct
    options.speedGrid_mps (1, :) double = []
    options.lateralFractions (1, :) double = linspace(0, 1, 5)
    options.boundaryIterations (1, 1) double = 10
    options.maximumAccelerationSearch_mps2 (1, 1) double = 5 * 9.81
    options.interpolationSafetyFactor (1, 1) double = 0.90
    options.showProgress (1, 1) logical = true
end
equivalent = lts.model.singleTrackSlip.equivalentVehicle(vehicle);
if isempty(options.speedGrid_mps)
    drive = lts.powertrain.maximumDriveForce(vehicle.powertrain, 0);
    options.speedGrid_mps = linspace(3, drive.maximumVehicleSpeed_mps, 7);
end
map = lts.model.dualTrackSlip.buildPerformanceMap(equivalent, ...
    speedGrid_mps=options.speedGrid_mps, ...
    lateralFractions=options.lateralFractions, ...
    boundaryIterations=options.boundaryIterations, ...
    maximumAccelerationSearch_mps2=options.maximumAccelerationSearch_mps2, ...
    interpolationSafetyFactor=options.interpolationSafetyFactor, ...
    showProgress=options.showProgress, progressLabel="Level 1.3 map", ...
    enableSolverRetries=true);
map.meta.modelLevel = '1.3';
map.meta.modelFamily = 'singleTrackSlip';
map.meta.leftRightLoadTransfer = false;
map.meta.sourceVehicleFrontTrack_m = vehicle.geometry.frontTrack_m;
map.meta.sourceVehicleRearTrack_m = vehicle.geometry.rearTrack_m;
map.meta.sourceVehicleMass_kg = vehicle.mass.total_kg;
map.meta.sourceVehicleWheelbase_m = vehicle.geometry.wheelbase_m;
map.meta.sourceVehicleFrontMassFraction = ...
    vehicle.geometry.frontMassFraction;
map.meta.sourceVehicleCgHeight_m = vehicle.geometry.cgHeight_m;
map.meta.sourceVehicleYawInertia_kgm2 = vehicle.mass.yawInertia_kgm2;
end
