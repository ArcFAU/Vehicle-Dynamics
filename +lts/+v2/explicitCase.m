function config = explicitCase(options)
%EXPLICITCASE Compact constructor with no implicit physical selections.

arguments
    options.Vehicle (1, 1) struct
    options.Track (1, 1) struct
    options.Model (1, 1) string
    options.Aero (1, 1) logical
    options.LoadTransfer (1, 1) logical
    options.TireSlip (1, 1) logical
    options.Energy (1, 1) logical
    options.YawInertia (1, 1) logical
    options.ChassisTorsion (1, 1) logical
    options.Strategy (1, 1) string
    options.MaximumSpeed_mps (1, 1) double
    options.InitialSpeed_mps (1, 1) double
    options.FinalSpeed_mps (1, 1) double
    options.PerformanceMap
    options.TrackLocal (1, 1) struct
end

input.vehicle = options.Vehicle;
input.track = options.Track;
input.model = options.Model;
input.features = struct('aero', options.Aero, ...
    'loadTransfer', options.LoadTransfer, 'tireSlip', options.TireSlip, ...
    'energy', options.Energy, 'yawInertia', options.YawInertia, ...
    'chassisTorsion', options.ChassisTorsion);
input.solver.strategy = options.Strategy;
input.solver.maximumSpeed_mps = options.MaximumSpeed_mps;
input.solver.initialSpeed_mps = options.InitialSpeed_mps;
input.solver.finalSpeed_mps = options.FinalSpeed_mps;
input.solver.performanceMap = options.PerformanceMap;
input.solver.trackLocal = options.TrackLocal;
config = lts.v2.createCase(input);
end
