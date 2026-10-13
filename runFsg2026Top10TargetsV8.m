function report = runFsg2026Top10TargetsV8(options)
%RUNFSG2026TOP10TARGETSV8 Corrected-envelope concept targets for FSG 2026.
% Only new V8 checkpoints are used. V7 outputs remain unchanged for audit.

arguments
    options.ShowProgress (1,1) logical = true
    options.TireBasis (1,1) string ...
        {mustBeMember(options.TireBasis,["teamQuadratic","tmeasy050","tmeasy060"])} ...
        = "teamQuadratic"
end

root = fileparts(mfilename('fullpath'));
stored.report = lts.io.loadR20Reference();
assert(stored.report.releasePassed,'Numerical reference is not released.');
base = lts.control.applyIdealAllocation(stored.report.vehicle);
base.controls.targetYawMoment_Nm = 0;
switch options.TireBasis
    case "teamQuadratic"
        envelope = lts.tire.createQuadraticPeakEnvelope();
    case "tmeasy050"
        envelope = lts.tire.fitTmeasyQuadraticEnvelope(base,MuScale=0.50);
    case "tmeasy060"
        envelope = lts.tire.fitTmeasyQuadraticEnvelope(base,MuScale=0.60);
end
base.tireEnvelope.peakEnvelope = envelope;
settings = studySettings();
events = manualFsgEvents();
rules = lts.config.scoring.fsg2026(referenceTimes());
cache = fullfile(root,'results','fsg2026Top10TargetsV8_cache_modular_energy_v1', ...
    char(options.TireBasis));
if ~isfolder(cache), mkdir(cache); end

aeroRows = struct([]);
for ci = 1:numel(settings.ClA_m2)
    for di = 1:numel(settings.CdA_m2)
        vehicle = withAero(base,settings.ClA_m2(ci), ...
            settings.CdA_m2(di),base.aerodynamics.frontDownforceFraction);
        key = sprintf('aero_cl%02d_cd%02d.mat',ci,di);
        sim = cachedCase(fullfile(cache,key),vehicle,events,rules);
        aeroRows = appendRows(aeroRows,caseRow("aero",key,sim,base,settings));
        progress(options.ShowProgress,"aero",numel(aeroRows), ...
            numel(settings.ClA_m2)*numel(settings.CdA_m2));
    end
end
aero = struct2table(aeroRows);

coupledRows = struct([]);
bestVehicle = struct();
bestPoints = -Inf;
for fi = 1:numel(settings.FrontSpringScales)
    for ri = 1:numel(settings.RearSpringScales)
        for ai = 1:numel(settings.AeroBalances)
            for mi = 1:numel(settings.FrontMassFractions)
                vehicle = base;
                vehicle.suspension.front.springRate_Npm = ...
                    base.suspension.front.springRate_Npm ...
                    *settings.FrontSpringScales(fi);
                vehicle.suspension.rear.springRate_Npm = ...
                    base.suspension.rear.springRate_Npm ...
                    *settings.RearSpringScales(ri);
                vehicle.geometry.frontMassFraction = ...
                    settings.FrontMassFractions(mi);
                [vehicle,~] = lts.suspension.applyRollStiffness(vehicle);
                vehicle = withAero(vehicle,settings.CoupledClA_m2, ...
                    settings.CoupledCdA_m2,settings.AeroBalances(ai));
                key = sprintf('coupled_f%02d_r%02d_a%02d_m%02d.mat', ...
                    fi,ri,ai,mi);
                sim = cachedCase(fullfile(cache,key),vehicle,events,rules);
                coupledRows = appendRows(coupledRows, ...
                    caseRow("coupled",key,sim,base,settings));
                if sim.dynamicPoints > bestPoints
                    bestPoints = sim.dynamicPoints;
                    bestVehicle = vehicle;
                end
                progress(options.ShowProgress,"coupled",numel(coupledRows), ...
                    numel(settings.FrontSpringScales) ...
                    *numel(settings.RearSpringScales) ...
                    *numel(settings.AeroBalances) ...
                    *numel(settings.FrontMassFractions));
            end
        end
    end
end
coupled = struct2table(coupledRows);

baseKey = 'baseline.mat';
baselineSim = cachedCase(fullfile(cache,baseKey),base,events,rules);
bestKey = 'best_coupled.mat';
bestSim = cachedCase(fullfile(cache,bestKey),bestVehicle,events,rules);
yawAudit = yawInertiaAudit(baselineSim,bestSim,settings);
physicsAudit = validationAudit(base,bestVehicle,envelope);
assert(all(physicsAudit.Passed), ...
    'V8 fast feasibility disagrees with the diagnostic state.');

report.meta.created = datetime('now','TimeZone','local');
report.meta.tireBasis = options.TireBasis;
report.meta.model = 'dualTrackEnvelope independent lateral guard';
report.meta.energyKernel = 'modular-energy-v1';
report.meta.overallCutoffSource = ...
    'FSG26_Overall_Results_v1_rp00042.pdf, place 10, 628.83 points';
report.meta.caveats = { ...
    'Historical 10th-place score is a threshold comparison, not a rank guarantee.'; ...
    'Static-event points, penalties, reliability and driverless are not simulated.'; ...
    'Yaw inertia is audited as moment demand, not coupled to lap time.'; ...
    'No aero-platform, camber-gain, transient or stiffness-mass model.'; ...
    'Event tracks are proxies and tire scale requires experimental validation.'};
report.settings = settings;
report.aero = aero;
report.coupled = coupled;
report.bestCoupled = coupled(coupled.DynamicPoints==max(coupled.DynamicPoints),:);
report.bestCoupled = report.bestCoupled(1,:);
report.yawAudit = yawAudit;
report.physicsAudit = physicsAudit;

matFile = lts.v2.nextResultFile(string(fullfile(root,'results')), ...
    "fsg2026Top10TargetsV8_modular_energy_v1");
[folder,stem] = fileparts(matFile);
report.files.mat = matFile;
report.files.aeroCsv = fullfile(folder,stem+"_aero.csv");
report.files.coupledCsv = fullfile(folder,stem+"_coupled.csv");
report.files.yawCsv = fullfile(folder,stem+"_yawInertiaAudit.csv");
report.files.auditCsv = fullfile(folder,stem+"_physicsAudit.csv");
save(matFile,'report','-v7.3');
writetable(aero,report.files.aeroCsv);
writetable(coupled,report.files.coupledCsv);
writetable(yawAudit,report.files.yawCsv);
writetable(physicsAudit,report.files.auditCsv);
fprintf('Saved corrected V8 results: %s\n',matFile);
end

function s = studySettings()
s.ClA_m2 = [0,1.5,3,4.5,5.4,6.3,7.2];
s.CdA_m2 = [0.6,0.9,1.2,1.5,1.7,2];
s.CoupledClA_m2 = 5.4;
s.CoupledCdA_m2 = 1.7;
s.FrontSpringScales = [0.75,1,1.25];
s.RearSpringScales = [0.75,1,1.25];
s.AeroBalances = [0.44,0.50,0.56];
s.FrontMassFractions = [0.44,0.47,0.50];
s.YawInertia_kgm2 = [60,75,90.5,110,130];
s.OverallTop10Cutoff_points = 628.83;
s.StaticPointsScenarios = [150,175,200,225,250,275];
s.Reserve_points = 20;
end

function vehicle = withAero(vehicle,clA,cdA,frontBalance)
area = vehicle.aerodynamics.referenceArea_m2;
vehicle.aerodynamics.totalLiftCoefficient = -clA/area;
vehicle.aerodynamics.dragCoefficient = cdA/area;
vehicle.aerodynamics.frontDownforceFraction = frontBalance;
vehicle.aerodynamics.liftCoefficientFront = ...
    vehicle.aerodynamics.totalLiftCoefficient*frontBalance;
vehicle.aerodynamics.liftCoefficientRear = ...
    vehicle.aerodynamics.totalLiftCoefficient*(1-frontBalance);
end

function sim = cachedCase(file,vehicle,events,rules)
version = 'modular-energy-v1-independent-lateral-limit';
if isfile(file)
    stored = load(file,'simulation');
    assert(strcmp(stored.simulation.modelVersion,version) ...
        && isequaln(stored.simulation.vehicle,vehicle), ...
        'V8 checkpoint does not match current vehicle/model: %s',file);
    sim = stored.simulation;
    return
end
model = lts.model.dualTrackEnvelope.create(vehicle);
eventResults = lts.events.runSet(events,model, ...
    IncludeDriverless=false,CoupleEnergy=false);
scoring = lts.scoring.scoreEvents(eventResults,rules, ...
    IncludeDriverless=false);
assert(scoring.isComplete,'V8 scoring is incomplete for %s.',file);
sim = struct('vehicle',vehicle,'eventResults',eventResults, ...
    'scoring',scoring,'dynamicPoints',scoring.totalPoints, ...
    'modelVersion',version);
simulation = sim;
temporary = string(tempname(fileparts(file)))+".mat";
cleanup = onCleanup(@()deleteIfPresent(temporary));
save(temporary,'simulation','-v7.3');
[ok,message] = movefile(temporary,file,'f');
assert(ok,'%s',message);
clear cleanup
end

function deleteIfPresent(file)
if isfile(file), delete(file); end
end

function rows = appendRows(rows,newRows)
if isempty(rows), rows = newRows;
else, rows = [rows;newRows]; end
end

function row = caseRow(phase,key,sim,base,settings)
v = sim.vehicle;
row.Phase = phase;
row.CaseId = string(key);
row.ClA_m2 = -v.aerodynamics.totalLiftCoefficient ...
    *v.aerodynamics.referenceArea_m2;
row.CdA_m2 = v.aerodynamics.dragCoefficient ...
    *v.aerodynamics.referenceArea_m2;
row.FrontAeroBalance = v.aerodynamics.frontDownforceFraction;
row.FrontMassFraction = v.geometry.frontMassFraction;
row.CgFromRearAxle_m = v.geometry.frontMassFraction ...
    *v.geometry.wheelbase_m;
row.FrontSpringScale = v.suspension.front.springRate_Npm ...
    /base.suspension.front.springRate_Npm;
row.RearSpringScale = v.suspension.rear.springRate_Npm ...
    /base.suspension.rear.springRate_Npm;
row.FrontRollMomentFraction = v.suspension.frontRollMomentFraction;
row.Acceleration_s = eventTime(sim,'acceleration');
row.Skidpad_s = eventTime(sim,'skidpad');
row.Autocross_s = eventTime(sim,'autocross');
row.Endurance_s = eventTime(sim,'endurance');
row.EnduranceEnergy_kWh = eventById(sim,'endurance').energy.netBatteryEnergy_kWh;
row.Acceleration_points = eventScore(sim,'acceleration');
row.Skidpad_points = eventScore(sim,'skidpad');
row.Autocross_points = eventScore(sim,'autocross');
row.Endurance_points = eventScore(sim,'endurance');
row.Efficiency_points = eventScore(sim,'efficiency');
row.DynamicPoints = sim.dynamicPoints;
row.RequiredStaticForTop10_points = ...
    settings.OverallTop10Cutoff_points-sim.dynamicPoints;
row.RequiredStaticWithReserve_points = ...
    row.RequiredStaticForTop10_points+settings.Reserve_points;
row.ProjectedTotalAt200Static_points = sim.dynamicPoints+200;
row.MeetsHistoricalCutoffAt200Static = ...
    row.ProjectedTotalAt200Static_points ...
    > settings.OverallTop10Cutoff_points;
end

function result = eventById(sim,id)
index = find(strcmp({sim.eventResults.id},id),1);
assert(~isempty(index),'Missing event: %s.',id);
result = sim.eventResults(index);
end

function time_s = eventTime(sim,id)
event = eventById(sim,id);
time_s = event.time_s;
end

function points = eventScore(sim,id)
scores = sim.scoring.events;
index = find(strcmp({scores.eventId},id),1);
assert(~isempty(index),'Missing event score: %s.',id);
points = scores(index).points;
end

function tableOut = yawInertiaAudit(baseline,best,settings)
sims = {baseline,best};
names = ["Baseline","Best coupled"];
rows = struct([]);
for si = 1:numel(sims)
    event = eventById(sims{si},'endurance');
    for ii = 1:numel(settings.YawInertia_kgm2)
        vehicle = sims{si}.vehicle;
        vehicle.mass.yawInertia_kgm2 = settings.YawInertia_kgm2(ii);
        demand = lts.analysis.yawDemand(event.lapResult,vehicle);
        moments = sort(abs(demand.inertialMoment_Nm(:)));
        index95 = max(1,ceil(0.95*numel(moments)));
        row.Configuration = names(si);
        row.YawInertia_kgm2 = vehicle.mass.yawInertia_kgm2;
        row.PeakInertialYawMoment_Nm = moments(end);
        row.P95InertialYawMoment_Nm = moments(index95);
        row.LapTime_s = event.time_s;
        row.CoupledToLapTime = false;
        rows = appendRows(rows,row);
    end
end
tableOut = struct2table(rows);
end

function tableOut = validationAudit(base,best,envelope)
vehicles = {base,best};
names = ["Baseline","Best coupled"];
speed = [8;11;15;20];
ax = [0;0;2;-4];
ay = [5;14;9;11];
rows = struct([]);
for vi = 1:numel(vehicles)
    vehicle = vehicles{vi};
    fast = lts.model.dualTrackEnvelope.feasibleMask( ...
        vehicle,speed,ax,ay,envelope);
    diagnostic = lts.model.dualTrackEnvelope.calculateState( ...
        vehicle,speed,ax,ay,envelope);
    for index = 1:numel(speed)
        row.Configuration = names(vi);
        row.Speed_mps = speed(index);
        row.Ax_mps2 = ax(index);
        row.Ay_mps2 = ay(index);
        row.FastFeasible = fast(index);
        row.DiagnosticFeasible = diagnostic.feasible(index);
        row.MaximumTireUtilization = max(diagnostic.tireUtilization(index,:));
        row.Passed = fast(index)==diagnostic.feasible(index);
        rows = appendRows(rows,row);
    end
end
tableOut = struct2table(rows);
end

function progress(enabled,phase,index,total)
if enabled, fprintf('V8 %s %d/%d\n',phase,index,total); end
end

function references = referenceTimes()
references.acceleration.bestTime_s = 3.369;
references.skidpad.bestTime_s = 4.678;
references.autocross.bestTime_s = 72.925;
references.endurance.bestTime_s = 1325.67;
references.efficiency.minimumFactor_s2kWh = 8725793.17;
end

function events = manualFsgEvents()
files = lts.io.trackCatalog();
events(1) = lts.events.createDefinition(Id="acceleration", ...
    Name="Acceleration",Discipline="acceleration", ...
    TrackFile=files.acceleration,IsClosed=false, ...
    InitialSpeed_mps=0,FinalSpeed_mps=100);
events(2) = lts.events.createDefinition(Id="skidpad", ...
    Name="Skidpad",Discipline="skidpad",TrackFile=files.skidpad, ...
    IsClosed=true,TimingWindows_m=[52,105;157,210], ...
    TimingAggregation="meanWindows");
events(3) = lts.events.createDefinition(Id="autocross", ...
    Name="Autocross",Discipline="autocross",TrackFile=files.autocross, ...
    IsClosed=false,InitialSpeed_mps=0,FinalSpeed_mps=100);
events(4) = lts.events.createDefinition(Id="endurance", ...
    Name="Endurance",Discipline="endurance",TrackFile=files.endurance, ...
    IsClosed=true,TimeMultiplier=18);
end
