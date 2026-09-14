function vehicle = FAUmaxTauV2(powertrainLayoutName)
%FAUMAXTAUV2 Readable vehicle parameters for nextGen V2.
% All physical values use SI units unless the field name states otherwise.
% The structure follows the section-based style of FAUmaxTau.m while
% avoiding loose variables in the MATLAB workspace.

arguments
    powertrainLayoutName (1, 1) string = "fourMotorAWD"
end

%% Metadata
vehicle.meta.name = 'FAUmaxTau nextGen V2';
vehicle.meta.status = [ ...
    'Concept dataset; LC0 tire fit retained as the initially validated ', ...
    'legacy tire basis, remaining vehicle data require validation'];
vehicle.meta.modelLevel = 'Supports model levels 0.1 through 2.5c in V2';

%% Tire selection
vehicle.tires.frontLeft  = 'Hoosier_16x7_5_10x8_R20_82kPa';
vehicle.tires.frontRight = 'Hoosier_16x7_5_10x8_R20_82kPa';
vehicle.tires.rearLeft   = 'Hoosier_16x7_5_10x8_R20_82kPa';
vehicle.tires.rearRight  = 'Hoosier_16x7_5_10x8_R20_82kPa';

%% Mass and chassis geometry
vehicle.mass.driver_kg = 80;                 % Driver mass
vehicle.mass.carWithoutDriver_kg = 170;      % Complete car without driver
vehicle.mass.total_kg = ...
    vehicle.mass.carWithoutDriver_kg + vehicle.mass.driver_kg;

vehicle.geometry.wheelbase_m = 1.530;        % Distance between axles
vehicle.geometry.frontTrack_m = 1.240;       % Front wheel-center distance
vehicle.geometry.rearTrack_m = 1.200;        % Rear wheel-center distance
vehicle.geometry.frontMassFraction = 0.470;  % Static mass fraction at front axle
vehicle.geometry.cgHeight_m = 0.290;         % CoG height above ground plane
vehicle.mass.yawInertia_kgm2 = 90.5;           % Yaw inertia about vehicle CoG

%% Level 0.1 tire approximation
% Constant friction coefficient for the analytic point-mass grip circle.
% Load sensitivity, combined-slip tire fits and camber start at later levels.
vehicle.tireApproximation.frictionCoefficient = 1.20;

%% Load-sensitive tire envelope - Level 0.2 and later
% The coefficients below define a transparent engineering approximation.
% They are placeholders until a selected tire dataset is validated.
vehicle.tireEnvelope.referenceNormalLoad_N = 600;
vehicle.tireEnvelope.frictionCoefficientAtReferenceLoad = 1.40;
vehicle.tireEnvelope.loadSensitivityExponent = -0.10;
vehicle.tireEnvelope.combinedForceExponent = 2.0;

%% Slip-based tire development dataset - Level 2.1 foundation
% This identified LC0 fit is retained as a sourced development dataset. It
% does not represent the R20 tire names selected above and is therefore not
% yet used by the lap models.
projectRoot = fileparts(fileparts(mfilename('fullpath')));
vehicle.tireSlipModel.developmentFit.name = 'Hoosier_10_16_8_LC0_83';
vehicle.tireSlipModel.developmentFit.file = fullfile(projectRoot, 'data', ...
    'tires', 'Hoosier_10_16_8_LC0_83.mat');
vehicle.tireSlipModel.developmentFit.lateralScale = 0.48;
vehicle.tireSlipModel.developmentFit.longitudinalScale = 0.50;
vehicle.tireSlipModel.developmentFit.status = ...
    'Development fit only; not the selected R20 tire';

%% Selectable stationary tire-force model - Level 2.5
% fittedCombinedSlip preserves the imported fitted-curve dataset exactly.
% tmeasyAdtm uses the local, versioned copy of the ADTM TMeasy V2.5 model.
vehicle.tireModel.active = 'fittedCombinedSlip';
% A fast envelope may not silently replace the selected tire model.
vehicle.tireModel.allowFastEnvelopeApproximation = false;
vehicle.tireModel.fittedCombinedSlip.name = ...
    'Fitted curves with friction-ellipse coupling';
vehicle.tireModel.fittedCombinedSlip.file = ...
    vehicle.tireSlipModel.developmentFit.file;
vehicle.tireModel.fittedCombinedSlip.lateralScale = ...
    vehicle.tireSlipModel.developmentFit.lateralScale;
vehicle.tireModel.fittedCombinedSlip.longitudinalScale = ...
    vehicle.tireSlipModel.developmentFit.longitudinalScale;
vehicle.tireModel.fittedCombinedSlip.version = 'fitted LC0 V1';
vehicle.tireModel.tmeasyAdtm.name = 'ADTM TMeasy V2.5 stationary';
vehicle.tireModel.tmeasyAdtm.parameterSet = ...
    'Hoosier_16x7_5_10x8_R20_82kPa';
vehicle.tireModel.tmeasyAdtm.muScale = 1.0;
vehicle.tireModel.tmeasyAdtm.version = 'ADTM 1.4 TMeasy V2.5 2025-06-09';
vehicle.tireModel.tmeasyAdtm.parameterVersion = 'V1.0 2024-09-14';
vehicle.tireModel.tmeasyAdtm.status = [ ...
    'Lateral and longitudinal parameters originate from different ' ...
    'Hoosier tire sizes; development reference'];

%% Aerodynamics - reserved for Level 0.3
vehicle.aerodynamics.referenceArea_m2 = 1.0;
vehicle.aerodynamics.dragCoefficient = 1.7;
% Negative lift coefficients create downforce. Total coefficient and front
% balance are the readable design inputs; the axle coefficients consumed by
% the models are derived from them.
vehicle.aerodynamics.totalLiftCoefficient = -5.4;
vehicle.aerodynamics.frontDownforceFraction = 0.50;
vehicle.aerodynamics.liftCoefficientFront = ...
    vehicle.aerodynamics.totalLiftCoefficient ...
    * vehicle.aerodynamics.frontDownforceFraction;
vehicle.aerodynamics.liftCoefficientRear = ...
    vehicle.aerodynamics.totalLiftCoefficient ...
    * (1 - vehicle.aerodynamics.frontDownforceFraction);
vehicle.aerodynamics.airDensity_kgpm3 = 1.225;
vehicle.aerodynamics.dragApplicationHeight_m = 0;

%% Brakes - reserved for later model levels
vehicle.brakes.maximumDeceleration_mps2 = Inf;
vehicle.brakes.frontForceFraction = 0.70;

%% Longitudinal control allocation
% "fixed" reproduces the prescribed axle split used by the lower levels.
% "ideal" minimizes normalized tire usage while satisfying force and yaw
% equilibrium within the actuator groups of the selected powertrain layout.
vehicle.controls.driveAllocation.mode = 'fixed';
vehicle.controls.brakeAllocation.mode = 'fixed';
vehicle.controls.regenAllocation.mode = 'fixed';
vehicle.controls.regenAllocation.fixedUtilizationFactor = 1.0;
vehicle.controls.targetYawMoment_Nm = 0;

%% Powertrain - reserved for Level 0.3
vehicle.powertrain.maximumPower_W = 80e3;
vehicle.powertrain.maximumMotorPower_W = 35e3;
vehicle.powertrain.maximumMotorTorque_Nm = 21;
vehicle.powertrain.maximumMotorSpeed_rpm = 20e3;
vehicle.powertrain.gearRatio = 12.421;
vehicle.powertrain.gearboxEfficiency = 0.97;
vehicle.powertrain.effectiveTireRadius_m = 0.202;
vehicle.powertrain.driveLimitModel.mode = 'socCurve';
vehicle.powertrain.driveLimitModel.evaluationSoc = 0.50;
vehicle.powertrain.driveLimitModel.socBreakpoints = ...
    [0.00, 0.10, 0.20, 0.80, 0.95, 1.00];
vehicle.powertrain.driveLimitModel.powerScale = ...
    [0.00, 0.60, 1.00, 1.00, 1.00, 0.90];
% The system-power limit is the primary sensitivity input. Detail limits can
% be enabled independently to expose motor torque, motor power, speed and
% SoC effects without changing the corresponding physical values.
vehicle.powertrain.limitOptions.motorTorqueEnabled = true;
vehicle.powertrain.limitOptions.motorPowerEnabled = true;
vehicle.powertrain.limitOptions.motorSpeedEnabled = true;
vehicle.powertrain.limitOptions.socPowerScaleEnabled = true;
vehicle.powertrain = powertrainLayout(vehicle.powertrain, powertrainLayoutName);

%% EV energy and accumulator targets - Level 2.4a
% Constant efficiencies are transparent concept assumptions. Replace them
% with speed/torque maps in a later level when measured component data exist.
vehicle.energy.rollingResistanceCoefficient = 0.015;
vehicle.energy.motorEfficiency = 0.92;
vehicle.energy.regenMotorEfficiency = 0.85;
vehicle.energy.inverterEfficiency = 0.97;
vehicle.energy.batteryDischargeEfficiency = 0.98;
vehicle.energy.batteryChargeEfficiency = 0.98;
vehicle.energy.auxiliaryPower_W = 500;
vehicle.energy.maximumRegenMotorTorque_Nm = 18;
vehicle.energy.maximumRegenMotorPower_W = 30e3;
vehicle.energy.maximumMechanicalRegenPower_W = 60e3;
vehicle.energy.maximumBatteryChargePower_W = 60e3;
vehicle.energy.maximumBatteryDischargePower_W = 80e3;
vehicle.energy.maximumRegenDeceleration_mps2 = 4.0;
vehicle.energy.minimumRegenSpeed_mps = 1.0;

% SoC state and independent charge/regen limits. Curves are linearly
% interpolated and clamped at their first and last breakpoints.
vehicle.energy.installedCapacity_kWh = 8.0;
vehicle.energy.initialSoc = 0.95;
vehicle.energy.socLimitModel.socBreakpoints = ...
    [0.00, 0.10, 0.20, 0.80, 0.95, 1.00];
vehicle.energy.socLimitModel.chargePowerScale = ...
    [1.00, 1.00, 1.00, 1.00, 0.60, 0.00];
vehicle.energy.socLimitModel.regenTorqueScale = ...
    [1.00, 1.00, 1.00, 1.00, 0.70, 0.00];
vehicle.energy.socLimitModel.regenPowerScale = ...
    [1.00, 1.00, 1.00, 1.00, 0.70, 0.00];

% The constant mode is active initially. In map mode, each component uses
% a readable 2-D table. Empty templates make missing map data explicit.
vehicle.energy.efficiencyModel.mode = 'constant';
vehicle.energy.efficiencyModel.motor.speed_rpm = [];
vehicle.energy.efficiencyModel.motor.torque_Nm = [];
vehicle.energy.efficiencyModel.motor.driveValues = [];
vehicle.energy.efficiencyModel.motor.regenValues = [];
vehicle.energy.efficiencyModel.inverter.power_W = [];
vehicle.energy.efficiencyModel.inverter.soc = [];
vehicle.energy.efficiencyModel.inverter.driveValues = [];
vehicle.energy.efficiencyModel.inverter.regenValues = [];
vehicle.energy.efficiencyModel.battery.power_W = [];
vehicle.energy.efficiencyModel.battery.soc = [];
vehicle.energy.efficiencyModel.battery.dischargeValues = [];
vehicle.energy.efficiencyModel.battery.chargeValues = [];

% Capacity targets reserve energy at the finish and keep operation inside a
% selected SoC window. Pack specific energy is for concept mass estimates.
vehicle.energy.nominalAccumulatorVoltage_V = 600;
vehicle.energy.usableSocWindow = 0.80;
vehicle.energy.finishEnergyReserveFraction = 0.10;
vehicle.energy.endOfLifeCapacityRetention = 0.90;
vehicle.energy.packSpecificEnergy_Whpkg = 160;

%% Suspension and chassis - reserved for Level 2.2 and later
vehicle.suspension.frontRollMomentFraction = 0.50;
vehicle.suspension.frontCamber_rad = deg2rad(-1.0);
vehicle.suspension.rearCamber_rad = deg2rad(-1.0);
vehicle.suspension.frontRollStiffness_NmPerRad = NaN;
vehicle.suspension.rearRollStiffness_NmPerRad = NaN;

% Elastic roll-stiffness inputs. Motion ratio is wheel travel divided by
% damper travel. Additional ARB stiffness is expressed directly at the axle.
vehicle.suspension.front.springRate_Npm = 158e3;
vehicle.suspension.front.motionRatioWheelPerDamper = 1.219;
vehicle.suspension.front.antiRollStiffness_NmPerRad = 0;
vehicle.suspension.rear.springRate_Npm = 88e3;
vehicle.suspension.rear.motionRatioWheelPerDamper = 1.185;
vehicle.suspension.rear.antiRollStiffness_NmPerRad = 0;
vehicle.chassis.torsionalStiffness_NmPerRad = Inf;
end
