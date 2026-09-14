function drive = maximumDriveForce(powertrain, speed_mps)
%MAXIMUMDRIVEFORCE Positive tire force available from torque and power.
% maximumPower_W is treated as system power before gearbox losses.

validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
wheelAngularSpeed_radps = speed_mps ./ powertrain.effectiveTireRadius_m;
motorAngularSpeed_radps = wheelAngularSpeed_radps .* powertrain.gearRatio;
drive.motorSpeed_rpm = motorAngularSpeed_radps * 60 / (2 * pi);

if powertrain.limitOptions.motorTorqueEnabled
    torqueLimitedForce_N = powertrain.maximumMotorTorque_Nm ...
        * powertrain.gearRatio * powertrain.gearboxEfficiency ...
        * powertrain.motorCount / powertrain.effectiveTireRadius_m;
else
    torqueLimitedForce_N = Inf;
end
availableElectricalPower_W = powertrain.maximumPower_W;
if powertrain.limitOptions.motorPowerEnabled
    availableElectricalPower_W = min(availableElectricalPower_W, ...
        powertrain.motorCount * powertrain.maximumMotorPower_W);
end
powerScale = drivePowerScale(powertrain);
availableElectricalPower_W = availableElectricalPower_W * powerScale;
mechanicalPower_W = availableElectricalPower_W * powertrain.gearboxEfficiency;
powerLimitedForce_N = Inf(size(speed_mps));
moving = speed_mps > 0;
powerLimitedForce_N(moving) = mechanicalPower_W ./ speed_mps(moving);

drive.maximumForce_N = min(torqueLimitedForce_N, powerLimitedForce_N);
overSpeed = powertrain.limitOptions.motorSpeedEnabled ...
    & drive.motorSpeed_rpm > powertrain.maximumMotorSpeed_rpm;
drive.maximumForce_N(overSpeed) = 0;
drive.torqueLimitedForce_N = torqueLimitedForce_N * ones(size(speed_mps));
drive.powerLimitedForce_N = powerLimitedForce_N;
drive.withinSpeedLimit = ~overSpeed;
if powertrain.limitOptions.motorSpeedEnabled
    drive.maximumVehicleSpeed_mps = powertrain.maximumMotorSpeed_rpm ...
        * 2 * pi / 60 / powertrain.gearRatio ...
        * powertrain.effectiveTireRadius_m;
else
    drive.maximumVehicleSpeed_mps = Inf;
end
drive.availableElectricalPower_W = availableElectricalPower_W;
drive.limitEvaluationSoc = powertrain.driveLimitModel.evaluationSoc;
drive.socPowerScale = powerScale;
end

function scale = drivePowerScale(powertrain)
if ~powertrain.limitOptions.socPowerScaleEnabled
    scale = 1;
    return
end
model = powertrain.driveLimitModel;
switch model.mode
    case 'constant'
        scale = 1;
    case 'socCurve'
        soc = min(max(model.evaluationSoc, ...
            model.socBreakpoints(1)), model.socBreakpoints(end));
        scale = interp1(model.socBreakpoints, model.powerScale, soc, ...
            'linear');
    otherwise
        error('lts:powertrain:DriveLimitModel', ...
            'Unknown drive-limit model: %s.', model.mode);
end
end
