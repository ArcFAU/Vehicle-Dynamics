function [model, effective] = buildDirectModel(options)
%BUILDDIRECTMODEL Build a transparent direct model for event workflows.
%   [MODEL, EFFECTIVE] = lts.v2.buildDirectModel(...) resolves one of the
%   three public V2 model families without using hidden feature defaults.
%   This entry point is intended for event and sensitivity workflows that
%   require a model object instead of a single-track case result.
arguments
    options.Vehicle (1,1) struct
    options.Model (1,1) string
    options.Aero (1,1) logical
    options.LoadTransfer (1,1) logical
    options.Energy (1,1) logical
    options.YawInertia (1,1) logical
    options.ChassisTorsion (1,1) logical
end
assert(~options.YawInertia,'lts:v2:UnsupportedFeature', ...
    'Direct event models do not support yawInertia.');
assert(~options.ChassisTorsion,'lts:v2:UnsupportedFeature', ...
    'Direct event models do not support chassisTorsion.');
vehicle=options.Vehicle;
if ~options.Aero
    vehicle.aerodynamics.dragCoefficient=0;
    vehicle.aerodynamics.totalLiftCoefficient=0;
    vehicle.aerodynamics.liftCoefficientFront=0;
    vehicle.aerodynamics.liftCoefficientRear=0;
end
vehicle.chassis.torsionalStiffness_NmPerRad=Inf;
switch options.Model
    case "pointMass"
        model=lts.model.pointMassAero.create(vehicle,UseAxleLoads=options.LoadTransfer);
    case "singleTrack"
        assert(options.LoadTransfer,'lts:v2:UnsupportedFeature', ...
            'singleTrack requires loadTransfer=true.');
        model=lts.model.singleTrackAero.create(vehicle);
    case "dualTrack"
        assert(options.LoadTransfer,'lts:v2:UnsupportedFeature', ...
            'dualTrack requires loadTransfer=true.');
        model=lts.model.dualTrackEnvelope.create(vehicle);
    otherwise
        error('lts:v2:Model','Unknown model family: %s.',options.Model);
end
if ~options.Energy
    model.vehicle = rmfield(model.vehicle, 'energy');
    vehicle = rmfield(vehicle, 'energy');
end
effective.vehicle=vehicle;
effective.model=char(options.Model);
effective.features=struct('aero',options.Aero,'loadTransfer',options.LoadTransfer, ...
    'tireSlip',false,'energy',options.Energy,'yawInertia',options.YawInertia, ...
    'chassisTorsion',options.ChassisTorsion);
fprintf('V2 event model: %s | aero=%d loadTransfer=%d energy=%d\n', ...
    options.Model,options.Aero,options.LoadTransfer,options.Energy);
end
