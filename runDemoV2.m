function result = runDemoV2()
%RUNDEMOV2 One complete, explicit input script for every V2 model family.

projectRoot = fileparts(mfilename('fullpath'));
addpath(projectRoot, fullfile(projectRoot, 'vehicles'));

input.vehicle = FAUmaxTauV2("fourMotorAWD");
tracks = lts.io.trackCatalog();
input.track = lts.io.loadTrackMatrix(tracks.endurance, true);

% Exactly one of: pointMass, singleTrack, dualTrack.
input.model = "singleTrack";

% Every physical feature is explicit; there are no hidden feature defaults.
input.features.aero = true;
input.features.loadTransfer = true;
input.features.tireSlip = false;
input.features.energy = true;
input.features.yawInertia = false;
input.features.chassisTorsion = false;

% Numerical choices are separate from physical model selection.
input.solver.strategy = "direct";
input.solver.maximumSpeed_mps = 120/3.6;
input.solver.initialSpeed_mps = 0;
input.solver.finalSpeed_mps = Inf;
input.solver.performanceMap = [];
input.solver.trackLocal.speedRows = 3;
input.solver.trackLocal.lateralFractions = [];
input.solver.trackLocal.boundaryIterations = 6;
input.solver.trackLocal.interpolationSafetyFactor = NaN;
input.solver.trackLocal.showProgress = false;
input.solver.trackLocal.auditPointCount = 8;

config = lts.v2.createCase(input);
result = lts.v2.runCase(config);
end
