function [vehicle, mapping] = mapLegacyVehicle(legacyVehicle)
%MAPLEGACYVEHICLE Map the original Tau-style C struct to a V2 vehicle.
% This is a data adapter only. It neither calls nor depends on a legacy
% simulation function and verifies that the local fitted dataset matches C.T.

arguments
    legacyVehicle (1, 1) struct
end

required = {'m','lv','lh','h','wv','wh','P','vmax','quer','laengs', ...
    'T','Ca','Cw','av','aeroh'};
for index = 1:numel(required)
    assert(isfield(legacyVehicle, required{index}), ...
        'lts:compatibility:LegacyField', ...
        'Missing legacy vehicle field: %s.', required{index});
end

vehicle = FAUmaxTauV2("singleRearMotor");
vehicle.meta.name = 'Legacy Tau mapped to nextGen V2';
vehicle.meta.status = 'Compatibility dataset; verify before design release';
vehicle.mass.driver_kg = 0;
vehicle.mass.carWithoutDriver_kg = legacyVehicle.m;
vehicle.mass.total_kg = legacyVehicle.m;
if isfield(legacyVehicle, 'Thetaz') && isfinite(legacyVehicle.Thetaz) ...
        && legacyVehicle.Thetaz > 0
    vehicle.mass.yawInertia_kgm2 = legacyVehicle.Thetaz;
end
vehicle.geometry.wheelbase_m = legacyVehicle.lv + legacyVehicle.lh;
vehicle.geometry.frontMassFraction = legacyVehicle.lh ...
    / vehicle.geometry.wheelbase_m;
vehicle.geometry.cgHeight_m = legacyVehicle.h;
vehicle.geometry.frontTrack_m = legacyVehicle.wv;
vehicle.geometry.rearTrack_m = legacyVehicle.wh;
% The historical MP/ES paths call pacejkavec with zero camber.
vehicle.suspension.frontCamber_rad = 0;
vehicle.suspension.rearCamber_rad = 0;

vehicle.powertrain.maximumPower_W = legacyVehicle.P;
vehicle.powertrain.maximumMotorPower_W = legacyVehicle.P;
vehicle.powertrain.maximumMotorTorque_Nm = 1e6;
vehicle.powertrain.gearboxEfficiency = 1;
vehicle.powertrain.driveLimitModel.mode = 'constant';
vehicle.powertrain.gearRatio = vehicle.powertrain.maximumMotorSpeed_rpm ...
    * 2 * pi / 60 * vehicle.powertrain.effectiveTireRadius_m ...
    / legacyVehicle.vmax;
if isfield(legacyVehicle, 'bal')
    vehicle.brakes.frontForceFraction = legacyVehicle.bal;
end

referenceSpeed_mps = 60 / 3.6;
dynamicPressureArea = 0.5 * vehicle.aerodynamics.airDensity_kgpm3 ...
    * referenceSpeed_mps^2 * vehicle.aerodynamics.referenceArea_m2;
vehicle.aerodynamics.dragCoefficient = max(0, legacyVehicle.Cw) ...
    / dynamicPressureArea;
frontAeroFraction = legacyVehicle.av / 100;
vehicle.aerodynamics.liftCoefficientFront = ...
    -legacyVehicle.Ca * frontAeroFraction / dynamicPressureArea;
vehicle.aerodynamics.liftCoefficientRear = ...
    -legacyVehicle.Ca * (1 - frontAeroFraction) / dynamicPressureArea;
vehicle.aerodynamics.dragApplicationHeight_m = legacyVehicle.aeroh;
if isfield(legacyVehicle, 'cr')
    vehicle.energy.rollingResistanceCoefficient = legacyVehicle.cr;
end

localFit = load(vehicle.tireModel.fittedCombinedSlip.file, 'T');
assert(isequal(size(localFit.T), size(legacyVehicle.T)) ...
    && max(abs(localFit.T - legacyVehicle.T), [], 'all') <= 1e-12, ...
    'lts:compatibility:TireDataset', ...
    'The V2 fitted tire dataset does not match legacy C.T.');
vehicle.tireModel.fittedCombinedSlip.lateralScale = legacyVehicle.quer;
vehicle.tireModel.fittedCombinedSlip.longitudinalScale = legacyVehicle.laengs;

referenceLoad_N = legacyVehicle.m * 9.81 / 4;
peakEnvelope = lts.tire.createPeakEnvelope(vehicle);
peaks = lts.tire.peakForces(peakEnvelope, referenceLoad_N, 0);
vehicle.tireApproximation.frictionCoefficient = min( ...
    [peaks.longitudinal_N, peaks.lateral_N]) / referenceLoad_N;
vehicle.tireEnvelope.referenceNormalLoad_N = referenceLoad_N;
vehicle.tireEnvelope.frictionCoefficientAtReferenceLoad = ...
    vehicle.tireApproximation.frictionCoefficient;

mapping.source = 'legacy C struct';
mapping.target = 'nextGen V2 vehicle';
mapping.powertrainLayout = vehicle.powertrain.layout;
mapping.tireDatasetVerified = true;
mapping.directionalPeakEnvelope = true;
mapping.maximumVehicleSpeed_mps = legacyVehicle.vmax;
mapping.note = ['The adapter maps physical parameters only; legacy state ', ...
    'filtering and solver iteration remain intentionally outside V2.'];
end
