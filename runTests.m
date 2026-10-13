function results = runTests(options)
%RUNTESTS Run the nextGen MATLAB test suite.
%   Discovers all tests below tests/, runs them, and raises an error if at
%   least one test fails.
%
%   Usage:
%       results = runTests;
%
%   Inspect RESULTS for individual durations and diagnostics.

arguments
    options.IncludeLegacyRegression (1,1) logical = false
end
% The optional external Tau regression is deliberately opt-in. All selected
% tests must pass; skipped tests in the selected suite remain an error.
%% Discover and run complete suite
projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot,'vehicles'),fullfile(projectRoot,'config'), ...
    fullfile(projectRoot,'tools'),fullfile(projectRoot,'tools','comparisons'), ...
    fullfile(projectRoot,'tools','diagnostics'),fullfile(projectRoot,'tools','legacy'));
suite = testsuite(fullfile(projectRoot, 'tests'), 'IncludeSubfolders', true);
if ~options.IncludeLegacyRegression
    suite=suite(~startsWith(string({suite.Name}),'TestLegacyRegressionV2/'));
    fprintf('External Tau regression excluded; enable IncludeLegacyRegression=true to run it.\n');
end
results = run(suite);

%% Require a fully successful suite
assertSuccess(results);
end
