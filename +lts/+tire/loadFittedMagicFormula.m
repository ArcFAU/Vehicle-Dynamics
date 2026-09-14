function fit = loadFittedMagicFormula(filePath, lateralScale, longitudinalScale)
%LOADFITTEDMAGICFORMULA Load one identified 13-row fitted tire dataset.
% The adapter contains the source matrix convention. Downstream tire functions
% use positive normal load, radians for angles and named result fields.

arguments
    filePath (1, 1) string
    lateralScale (1, 1) double {mustBePositive, mustBeFinite} = 1
    longitudinalScale (1, 1) double {mustBePositive, mustBeFinite} = 1
end

if ~isfile(filePath)
    error('lts:tire:FitFileNotFound', 'Tire fit file not found: %s', filePath);
end
data = load(filePath, 'T');
if ~isfield(data, 'T') || ~isnumeric(data.T) ...
        || size(data.T, 1) < 13 || size(data.T, 2) < 13 ...
        || any(~isfinite(data.T(:)))
    error('lts:tire:InvalidFittedFit', ...
        'Expected a finite numeric tire-fit matrix T of at least 13 by 13.');
end

[~, name, extension] = fileparts(filePath);
fit.name = char(name + extension);
fit.coefficients = data.T;
fit.lateralScale = lateralScale;
fit.longitudinalScale = longitudinalScale;
fit.sourceFile = char(filePath);
fit.sourceConvention = ['negative Fz for a loaded tire, camber and slip ' ...
    'angle in degrees, slip ratio dimensionless'];
end
