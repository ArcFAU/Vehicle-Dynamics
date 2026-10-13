function report = loadR20Reference()
%LOADR20REFERENCE Load the frozen numerical release as an input artifact.
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
file = fullfile(root,'data','reference','r20SlipMapReleaseValidationV6.mat');
assert(isfile(file),'lts:io:ReferenceFile','Missing R20 reference input: %s',file);
stored = load(file,'report');
assert(isfield(stored,'report') && stored.report.releasePassed, ...
    'lts:io:ReferenceRelease','R20 numerical reference is not released.');
report = stored.report;
report.vehicle = lts.compatibility.normalizeEnergyModel(report.vehicle);
% The frozen source vehicle stored machine-specific tire-fit file paths.
% Rebase only those file locations; retain all physical input values.
tireFile = fullfile(root,'data','tires','Hoosier_10_16_8_LC0_83.mat');
report.vehicle.tireModel.fittedCombinedSlip.file = tireFile;
report.vehicle.tireSlipModel.developmentFit.file = tireFile;
end
