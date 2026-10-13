function settings = trackLocalSettings(options)
%TRACKLOCALSETTINGS Explicit numerical settings shared by V2 runners.

arguments
    options.SpeedRows (1, 1) double
    options.LateralFractions (1, :) double
    options.BoundaryIterations (1, 1) double
    options.InterpolationSafetyFactor (1, 1) double
    options.ShowProgress (1, 1) logical
    options.AuditPointCount (1, 1) double
end
settings.speedRows = options.SpeedRows;
settings.lateralFractions = options.LateralFractions;
settings.boundaryIterations = options.BoundaryIterations;
settings.interpolationSafetyFactor = options.InterpolationSafetyFactor;
settings.showProgress = options.ShowProgress;
settings.auditPointCount = options.AuditPointCount;
end
