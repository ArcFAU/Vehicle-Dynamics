function results = runTests()
%RUNTESTS Run the nextGen MATLAB test suite.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));
suite = testsuite(fullfile(projectRoot, 'tests'), 'IncludeSubfolders', true);
results = run(suite);
assertSuccess(results);
end
