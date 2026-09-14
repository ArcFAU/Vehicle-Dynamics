function vehicle = applyParameter(vehicle, parameter, value)
%APPLYPARAMETER Apply one sweep value without mutating the base vehicle.

if ~isempty(parameter.apply)
    vehicle = parameter.apply(vehicle, value);
    return
end

parts = split(string(parameter.path), '.');
assert(~isempty(parts), 'lts:study:ParameterPath', ...
    'Parameter path must not be empty.');
vehicle = setNestedField(vehicle, cellstr(parts), value);
if any(strcmp(parameter.path, ...
        {'mass.carWithoutDriver_kg', 'mass.driver_kg'}))
    vehicle.mass.total_kg = vehicle.mass.carWithoutDriver_kg ...
        + vehicle.mass.driver_kg;
end
end

function output = setNestedField(input, parts, value)
field = parts{1};
assert(isfield(input, field), 'lts:study:ParameterPath', ...
    'Unknown parameter path component: %s.', field);
if isscalar(parts)
    input.(field) = value;
else
    input.(field) = setNestedField(input.(field), parts(2:end), value);
end
output = input;
end
