function definition = parameter(options)
%PARAMETER Define one readable vehicle parameter sweep axis.

arguments
    options.Name (1, 1) string
    options.Path (1, 1) string
    options.Unit (1, 1) string = ""
    options.Values (1, :) double
    options.Baseline (1, 1) double = NaN
    options.Apply = []
end

assert(~isempty(options.Values) && all(isfinite(options.Values)), ...
    'lts:study:Values', 'Parameter values must be finite and nonempty.');
assert(isempty(options.Apply) || isa(options.Apply, 'function_handle'), ...
    'lts:study:Apply', 'Apply must be empty or a function handle.');
definition.name = char(options.Name);
definition.path = char(options.Path);
definition.unit = char(options.Unit);
definition.values = options.Values;
definition.baseline = options.Baseline;
definition.apply = options.Apply;
end
