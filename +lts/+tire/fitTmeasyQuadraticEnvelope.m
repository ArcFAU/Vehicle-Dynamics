function [envelope,diagnostics] = fitTmeasyQuadraticEnvelope(vehicle,options)
%FITTMEASYQUADRATICENVELOPE Fit fast directional peaks from TMeasy.

arguments
    vehicle (1,1) struct
    options.MuScale (1,1) double {mustBePositive} = vehicle.tireModel.tmeasyAdtm.muScale
    options.NormalLoads_N (:,1) double {mustBePositive} = (200:50:1600).'
    options.CamberAngle_rad (1,1) double = 0
    options.Speed_mps (1,1) double {mustBePositive} = 15
    options.CombinedForceExponent (1,1) double {mustBeGreaterThanOrEqual(options.CombinedForceExponent,1)} = 2
end

tir = lts.tire.tmeasyAdtm.loadParameters(vehicle.tireModel.tmeasyAdtm.parameterSet);
loads = options.NormalLoads_N(:);
drive = zeros(size(loads)); brake = drive; lateral = drive;
for index = 1:numel(loads)
    positive = lts.tire.tmeasyAdtm.evaluate(tir,loads(index), ...
        options.CamberAngle_rad,0.01,0,options.Speed_mps,options.MuScale);
    negative = lts.tire.tmeasyAdtm.evaluate(tir,loads(index), ...
        options.CamberAngle_rad,-0.01,0,options.Speed_mps,options.MuScale);
    pureLateral = lts.tire.tmeasyAdtm.evaluate(tir,loads(index), ...
        options.CamberAngle_rad,0,0,options.Speed_mps,options.MuScale);
    drive(index) = abs(positive.FxM_N);
    brake(index) = abs(negative.FxM_N);
    lateral(index) = abs(pureLateral.FyM_N);
end

[muDrive,kDrive,fitDrive] = fitQuadratic(loads,drive);
[muBrake,kBrake,fitBrake] = fitQuadratic(loads,brake);
[muLateral,kLateral,fitLateral] = fitQuadratic(loads,lateral);
envelope = lts.tire.createQuadraticPeakEnvelope( ...
    DriveMu0=muDrive,DriveDegression_Ninv=kDrive, ...
    BrakeMu0=muBrake,BrakeDegression_Ninv=kBrake, ...
    LateralMu0=muLateral,LateralDegression_Ninv=kLateral, ...
    CombinedForceExponent=options.CombinedForceExponent, ...
    Name="TMeasy quadratic fit, muScale="+options.MuScale);
envelope.classification = 'tmeasyDerivedApproximation';
envelope.source = struct('parameterSet',vehicle.tireModel.tmeasyAdtm.parameterSet, ...
    'muScale',options.MuScale,'camberAngle_rad',options.CamberAngle_rad, ...
    'speed_mps',options.Speed_mps,'normalLoads_N',loads);
diagnostics = table(loads,drive,fitDrive,brake,fitBrake,lateral,fitLateral, ...
    'VariableNames',{'NormalLoad_N','TMeasyDrive_N','FitDrive_N', ...
    'TMeasyBrake_N','FitBrake_N','TMeasyLateral_N','FitLateral_N'});
end

function [mu0,degression,fitted] = fitQuadratic(loads,forces)
design = [loads,-loads.^2];
coefficient = design\forces;
mu0 = max(coefficient(1),eps);
degression = max(coefficient(2),0);
fitted = max(0,mu0*loads-degression*loads.^2);
end
