function powertrain = powertrainLayout(powertrain, layoutName)
%POWERTRAINLAYOUT Configure motor count and fixed force distribution.
% Central motors drive both wheels of their axle through a differential.
% Multiple-motor layouts use equal commands; torque vectoring is disabled.

arguments
    powertrain (1, 1) struct
    layoutName (1, 1) string
end

switch layoutName
    case "singleFrontMotor"
        powertrain.motorCount = 1;
        powertrain.drivenWheelCount = 2;
        powertrain.frontDriveForceFraction = 1.0;
        powertrain.wheelActuatorGroups = [1, 1, 0, 0];
    case "singleRearMotor"
        powertrain.motorCount = 1;
        powertrain.drivenWheelCount = 2;
        powertrain.frontDriveForceFraction = 0.0;
        powertrain.wheelActuatorGroups = [0, 0, 1, 1];
    case "dualFrontMotors"
        powertrain.motorCount = 2;
        powertrain.drivenWheelCount = 2;
        powertrain.frontDriveForceFraction = 1.0;
        powertrain.wheelActuatorGroups = [1, 2, 0, 0];
    case "dualRearMotors"
        powertrain.motorCount = 2;
        powertrain.drivenWheelCount = 2;
        powertrain.frontDriveForceFraction = 0.0;
        powertrain.wheelActuatorGroups = [0, 0, 1, 2];
    case "dualMotorAWD"
        powertrain.motorCount = 2;
        powertrain.drivenWheelCount = 4;
        powertrain.frontDriveForceFraction = 0.5;
        powertrain.wheelActuatorGroups = [1, 1, 2, 2];
    case "fourMotorAWD"
        powertrain.motorCount = 4;
        powertrain.drivenWheelCount = 4;
        powertrain.frontDriveForceFraction = 0.5;
        powertrain.wheelActuatorGroups = [1, 2, 3, 4];
    otherwise
        error('lts:config:PowertrainLayout', ...
            'Unknown powertrain layout: %s.', layoutName);
end
powertrain.layout = char(layoutName);
powertrain.leftRightTorqueAllocation = 'equal';
if powertrain.frontDriveForceFraction == 1
    powertrain.drivenAxle = 'front';
elseif powertrain.frontDriveForceFraction == 0
    powertrain.drivenAxle = 'rear';
else
    powertrain.drivenAxle = 'all';
end
end
