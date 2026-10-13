function caseConfig = sensitivityStudyCaseV2(options)
%SENSITIVITYSTUDYCASEV2 Create a readable V2 sensitivity-study definition.
%   CASECONFIG = sensitivityStudyCaseV2(...) defines the model family,
%   physical features, vehicle, events, scoring rules and sweep parameters.
%   Every physical selection is mandatory; no feature is enabled implicitly.
%
%   Example:
%       config = sensitivityStudyCaseV2(Study="mass", ...
%           Model="pointMass", Aero=true, LoadTransfer=false, Energy=true, ...
%           PowertrainLayout="fourMotorAWD", ...
%           TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
%           CoupleEnergy=false, CreatePlots=true);
%       study = runSensitivityStudyV2(config);
arguments
    options.Study (1,1) string
    options.Model (1,1) string {mustBeMember(options.Model,["pointMass","singleTrack","dualTrack"])}
    options.Aero (1,1) logical
    options.LoadTransfer (1,1) logical
    options.Energy (1,1) logical
    options.PowertrainLayout (1,1) string
    options.TireModel (1,1) string
    options.IncludeDriverless (1,1) logical
    options.CoupleEnergy (1,1) logical
    options.CreatePlots (1,1) logical
end

assert(~options.CoupleEnergy || options.Energy, 'lts:v2:EnergyCoupling', ...
    'CoupleEnergy=true requires Energy=true.');
if options.Model ~= "pointMass"
    assert(options.LoadTransfer, 'lts:v2:UnsupportedFeature', ...
        '%s requires LoadTransfer=true.', options.Model);
end

%% Reuse the established parameter, event and scoring definitions
legacyIds=struct('pointMass',"pointMassAero",'singleTrack',"singleTrackAero", ...
    'dualTrack',"dualTrackEnvelope");
caseConfig=sensitivityStudyCase(Study=options.Study, ...
    Model=legacyIds.(char(options.Model)),PowertrainLayout=options.PowertrainLayout, ...
    TireModel=options.TireModel,IncludeDriverless=options.IncludeDriverless, ...
    CoupleEnergy=options.CoupleEnergy,CreatePlots=options.CreatePlots);
caseConfig.model=char(options.Model);
caseConfig.features=struct('aero',options.Aero,'loadTransfer',options.LoadTransfer, ...
    'tireSlip',false,'energy',options.Energy,'yawInertia',false,'chassisTorsion',false);
caseConfig.name=[caseConfig.name,'_V2'];
end
