function envelope = createQuadraticPeakEnvelope(options)
%CREATEQUADRATICPEAKENVELOPE Directional quadratic force envelope.
%   Fmax = mu0*Fz - degression*Fz^2. Coefficients may differ between
%   drive, braking and lateral force. The default is the team reference.

arguments
    options.DriveMu0 (1,1) double {mustBePositive} = 1.39
    options.DriveDegression_Ninv (1,1) double {mustBeNonnegative} = 0.00019425
    options.BrakeMu0 (1,1) double {mustBePositive} = 1.39
    options.BrakeDegression_Ninv (1,1) double {mustBeNonnegative} = 0.00019425
    options.LateralMu0 (1,1) double {mustBePositive} = 1.39
    options.LateralDegression_Ninv (1,1) double {mustBeNonnegative} = 0.00019425
    options.CombinedForceExponent (1,1) double {mustBeGreaterThanOrEqual(options.CombinedForceExponent,1)} = 2
    options.Name (1,1) string = "Team quadratic tire envelope"
end

envelope.type = 'quadraticDirectional';
envelope.name = char(options.Name);
envelope.classification = 'engineeringReference';
envelope.combinedForceExponent = options.CombinedForceExponent;
envelope.quadratic.drive = coefficients(options.DriveMu0,options.DriveDegression_Ninv);
envelope.quadratic.brake = coefficients(options.BrakeMu0,options.BrakeDegression_Ninv);
envelope.quadratic.lateral = coefficients(options.LateralMu0,options.LateralDegression_Ninv);
end

function value = coefficients(mu0,degression)
value = struct('mu0',mu0,'degression_Ninv',degression);
end
