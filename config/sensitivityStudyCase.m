function caseConfig = sensitivityStudyCase(options)
%SENSITIVITYSTUDYCASE Readable event and scoring setup for concept studies.
%   Selects a predefined FAUmax Tau parameter study and returns the complete
%   internal configuration adapted by sensitivityStudyCaseV2 for V2 runners.
%
%   Example:
%       caseConfig = sensitivityStudyCase(Study="mass", ...
%           Model="pointMassAero", PowertrainLayout="fourMotorAWD", ...
%           TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
%           CoupleEnergy=false, CreatePlots=true);
%   Use sensitivityStudyCaseV2 and runSensitivityStudyV2 for public runs.
%
%   The available study definitions are listed in the study-selection
%   section below. Copy the returned struct before making case-specific
%   changes, so the predefined setup remains reproducible.

arguments
    options.Study (1, 1) string
    options.Model (1, 1) string {mustBeMember(options.Model, ...
        ["pointMassAero", "singleTrackAero", "dualTrackEnvelope"])}
    options.PowertrainLayout (1, 1) string
    options.TireModel (1, 1) string {mustBeMember(options.TireModel, ...
        ["fittedCombinedSlip", "tmeasyAdtm"])}
    options.IncludeDriverless (1, 1) logical
    options.CoupleEnergy (1, 1) logical
    options.CreatePlots (1, 1) logical
end

trackFiles = lts.io.trackCatalog();

%% Study identity and model
caseConfig.name = 'FSG_2026_concept_sensitivity';
caseConfig.model = char(options.Model);
caseConfig.includeDriverless = options.IncludeDriverless;
caseConfig.coupleEnergy = options.CoupleEnergy;
caseConfig.baseVehicle = lts.tire.selectModel( ...
    FAUmaxTauV2(options.PowertrainLayout), options.TireModel);
caseConfig.createPlots = options.CreatePlots;

% Optional detail limits. maximumPower_W remains the primary system-power
% input; disable individual secondary limits for an idealized power sweep.
caseConfig.baseVehicle.powertrain.limitOptions.motorTorqueEnabled = true;
caseConfig.baseVehicle.powertrain.limitOptions.motorPowerEnabled = true;
caseConfig.baseVehicle.powertrain.limitOptions.motorSpeedEnabled = true;
caseConfig.baseVehicle.powertrain.limitOptions.socPowerScaleEnabled = true;

%% Event and track selection
% Any field may point to another distance/curvature MAT file.
% For a different event, build a new array with lts.events.createDefinition.
trackFiles.dvAcceleration = trackFiles.acceleration;
trackFiles.dvSkidpad = trackFiles.skidpad;
caseConfig.events = lts.config.events.fsg2026(TrackFiles=trackFiles);

%% Rule year and annual result basis
% Enter the fastest official manual times from the chosen event and year.
% For driverless ranking points, enter the published valid competitor times.
references = lts.config.scoring.referenceTemplate();
references.acceleration.bestTime_s = 3.369;
references.skidpad.bestTime_s = 4.935;
references.autocross.bestTime_s = 72.925;
references.endurance.bestTime_s = 1319.67;
references.efficiency.minimumFactor_s2kWh = 8725793.17;
references.dvAcceleration.competitorTimes_s = [];
references.dvSkidpad.competitorTimes_s = [];
caseConfig.scoring = lts.config.scoring.fsg2026(references);

%% Vehicle parameters to vary
% Available studies: mass, massYaw, massDistribution, downforce,
% aeroBalance, aeroMap, power and brakeBias. aeroMap is a two-dimensional
% full-factorial sweep; all others are one-dimensional.
caseConfig.study = char(options.Study);
base = caseConfig.baseVehicle;
switch options.Study
    case "mass"
        caseConfig.parameters = lts.study.parameter( ...
            Name="Car mass without driver", ...
            Path="mass.carWithoutDriver_kg", Unit="kg", ...
            Values=150:5:190, Baseline=base.mass.carWithoutDriver_kg);
    case "massYaw"
        caseConfig.parameters = lts.study.parameter( ...
            Name="Total mass with proportional yaw inertia", ...
            Path="mass.total_kg", Unit="kg", Values=230:5:270, ...
            Baseline=base.mass.total_kg, ...
            Apply=@(vehicle, value) applyMassYaw(vehicle, value, ...
                base.mass.total_kg, base.mass.yawInertia_kgm2));
    case "massDistribution"
        caseConfig.parameters = lts.study.parameter( ...
            Name="Front mass fraction", ...
            Path="geometry.frontMassFraction", Unit="-", ...
            Values=0.42:0.01:0.52, ...
            Baseline=base.geometry.frontMassFraction);
    case "downforce"
        caseConfig.parameters = downforceParameter(base);
    case "aeroBalance"
        caseConfig.parameters = aeroBalanceParameter(base);
    case "aeroMap"
        caseConfig.parameters(1) = downforceParameter(base);
        caseConfig.parameters(2) = aeroBalanceParameter(base);
    case "power"
        caseConfig.parameters = lts.study.parameter( ...
            Name="System power", Path="powertrain.maximumPower_W", ...
            Unit="W", Values=(50:5:110) * 1e3, ...
            Baseline=base.powertrain.maximumPower_W);
    case "brakeBias"
        caseConfig.parameters = lts.study.parameter( ...
            Name="Front brake force fraction", ...
            Path="brakes.frontForceFraction", Unit="-", ...
            Values=0.55:0.025:0.80, ...
            Baseline=base.brakes.frontForceFraction);
    otherwise
        error('lts:study:UnknownPreset', ...
            'Unknown sensitivity study preset: %s.', options.Study);
end
end

function definition = downforceParameter(base)
definition = lts.study.parameter( ...
    Name="Total lift coefficient", ...
    Path="aerodynamics.totalLiftCoefficient", Unit="-", ...
    Values=-6.6:0.3:-4.2, ...
    Baseline=base.aerodynamics.totalLiftCoefficient, ...
    Apply=@applyTotalLiftCoefficient);
end

function definition = aeroBalanceParameter(base)
definition = lts.study.parameter( ...
    Name="Front aero balance", ...
    Path="aerodynamics.frontDownforceFraction", Unit="-", ...
    Values=0.40:0.025:0.60, ...
    Baseline=base.aerodynamics.frontDownforceFraction, ...
    Apply=@applyAeroBalance);
end

function vehicle = applyTotalLiftCoefficient(vehicle, coefficient)
vehicle.aerodynamics.totalLiftCoefficient = coefficient;
vehicle = updateAxleLiftCoefficients(vehicle);
end

function vehicle = applyAeroBalance(vehicle, frontFraction)
vehicle.aerodynamics.frontDownforceFraction = frontFraction;
vehicle = updateAxleLiftCoefficients(vehicle);
end

function vehicle = updateAxleLiftCoefficients(vehicle)
total = vehicle.aerodynamics.totalLiftCoefficient;
frontFraction = vehicle.aerodynamics.frontDownforceFraction;
vehicle.aerodynamics.liftCoefficientFront = total * frontFraction;
vehicle.aerodynamics.liftCoefficientRear = total * (1 - frontFraction);
end

function vehicle = applyMassYaw(vehicle, totalMass_kg, ...
        referenceMass_kg, referenceYawInertia_kgm2)
vehicle.mass.total_kg = totalMass_kg;
vehicle.mass.carWithoutDriver_kg = totalMass_kg - vehicle.mass.driver_kg;
vehicle.mass.yawInertia_kgm2 = referenceYawInertia_kgm2 ...
    * totalMass_kg / referenceMass_kg;
end
