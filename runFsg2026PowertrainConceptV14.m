function report = runFsg2026PowertrainConceptV14(options)
%RUNFSG2026POWERTRAINCONCEPTV14 Prepare or run the DC-terminal concept study.
% Default runs AMK 100 C loss maps with a conservative 500-V voltage cap.
% Generator losses and feasible torque envelope mirror the positive side.
% Tire midpoint and wheelwise regeneration limitations remain documented.
arguments
    options.ShowProgress (1,1) logical = true
    options.PrepareOnly (1,1) logical = false
    options.MotorModel (1,1) string {mustBeMember(options.MotorModel,["constant","efficiencyMap","lossMap"])} = "lossMap"
    options.MotorEfficiencyMap (1,1) struct = struct()
    options.Repetitions (1,1) double {mustBeInteger,mustBePositive} = 18
    options.ClA_m2 (1,:) double = [0,5,6.6,7]
    options.BatteryDischargeLimits_kW (1,:) double = 50:5:80
    options.DriverMass_kg (1,:) double = [60,80]
    options.OtherEventDriverMass_kg (1,:) double = [60,80]
    options.DriverMassByDiscipline (1,1) struct = struct()
    options.BatteryChargeLimits_kW (1,:) double = [0,40,60,80,Inf]
    options.OtherEventChargeLimit_kW (1,1) double {mustBeNonnegative} = Inf
    options.OtherEventDischargeLimit_kW (1,1) double {mustBePositive} = 80
    options.ReferenceCapacity_kWh (1,1) double {mustBePositive} = 100
    options.InitialSoc (1,1) double {mustBeInRange(options.InitialSoc,0,1)} = 0.5
    options.UseSocLimits (1,1) logical = false
    options.RunOtherEvents (1,1) logical = true
    options.BestEnduranceTime_s (1,1) double {mustBePositive} = 1325.67
    options.MinimumEfficiencyFactor_s2kWh (1,1) double {mustBePositive} = 8725793.17
    options.PointLossTarget (1,1) double {mustBePositive} = 7
    options.ExportClA_m2 (1,1) double = 6.6
    options.ExportDriverMass_kg (1,1) double = 80
    options.ExportChargeLimit_kW (1,1) double = Inf
    options.OutputTag (1,1) string = "modular_amk100C_500V"
end
% Optional independent grids override the common defaults by discipline.
if isfield(options.DriverMassByDiscipline,'endurance')
    options.DriverMass_kg=options.DriverMassByDiscipline.endurance;
end
assert(all(isfinite(options.ClA_m2)) && all(options.ClA_m2>=0));
assert(all(isfinite(options.BatteryDischargeLimits_kW)) ...
    && all(options.BatteryDischargeLimits_kW>=50) ...
    && any(options.BatteryDischargeLimits_kW==80), ...
    'V14 power sweep starts at 50 kW and must contain the 80 kW reference.');
assert(all(isfinite(options.DriverMass_kg)) && all(options.DriverMass_kg>0));
assert(~isempty(options.BatteryChargeLimits_kW) ...
    && all(~isnan(options.BatteryChargeLimits_kW)) ...
    && all(options.BatteryChargeLimits_kW>=0));
assert(all(isfinite(options.OtherEventDriverMass_kg)) ...
    && all(options.OtherEventDriverMass_kg>0));
assert(~isempty(regexp(char(options.OutputTag),'^[a-zA-Z0-9_-]+$','once')));
if options.MotorModel=="lossMap" && isempty(fieldnames(options.MotorEfficiencyMap))
    options.MotorEfficiencyMap=lts.powertrain.importAmkMotorMap();
end
lts.validation.validateMotorModel(struct('mode',char(options.MotorModel),'map',options.MotorEfficiencyMap));
root=fileparts(mfilename('fullpath'));
resultsDir=fullfile(root,'results');
out=fullfile(resultsDir,'fsg2026PowertrainConceptV14',char(options.OutputTag));
if ~isfolder(out), mkdir(out); end
cache=fullfile(out,'cache');
version='modular-energy-v1';
cdA=2*ones(size(options.ClA_m2)); cdA(options.ClA_m2==0)=0.8;
[ci,pi,di,ai]=ndgrid(1:numel(options.BatteryChargeLimits_kW), ...
    1:numel(options.BatteryDischargeLimits_kW),1:numel(options.DriverMass_kg), ...
    1:numel(options.ClA_m2));
manifest=table(ai(:),di(:),pi(:),ci(:),reshape(options.ClA_m2(ai),[],1), ...
    reshape(cdA(ai),[],1),reshape(options.DriverMass_kg(di),[],1), ...
    reshape(options.BatteryDischargeLimits_kW(pi),[],1), ...
    reshape(options.BatteryChargeLimits_kW(ci),[],1), ...
    'VariableNames',{'AeroIndex','DriverIndex','PowerIndex','ChargeIndex','ClA_m2','CdA_m2', ...
    'DriverMass_kg','BatteryDischargeLimit_kW','BatteryChargeLimit_kW'});
report.meta=struct('version',version,'created',datetime('now'), ...
    'measurementBoundary','accumulator DC terminals', ...
    'TMeasyMuScale',0.5,'motorModel',options.MotorModel,'motorMapProvided',~isempty(fieldnames(options.MotorEfficiencyMap)), ...
    'referenceCapacityIsSizingTarget',false, ...
    'notes',{{'No auxiliary loads; motor limits 21 Nm and 35 kW per motor.', ...
    'Scoring energy = discharge energy - 0.9 * regenerated energy.', ...
    'Reference capacity prevents arbitrary pack-size/SoC taper from setting the concept optimum.', ...
    'Capacity target is maximum cumulative depletion, without reserve scaling.', ...
    'Each Endurance case uses one constant driver mass; driver-change dynamics are not included.', ...
    'Driver mass changes total mass; driver-specific CoG and inertia changes need measured inputs.', ...
    'Current export uses nominal pack voltage, not a cell/voltage model.', ...
    'Annual scoring references are fixed across all compared cases.'}});
report.options=options;
report.manifest=manifest;
report.expectedEnduranceCases=height(manifest);
report.expectedOtherEventCases=3*numel(options.ClA_m2) ...
    *numel(options.OtherEventDriverMass_kg)*options.RunOtherEvents;
report.otherEventManifest=table();
for id=["acceleration","skidpad","autocross"]
    masses=driverGrid(options,id);
    [a,d,ch]=ndgrid(1:numel(options.ClA_m2),1:numel(masses), ...
        1:numel(options.BatteryChargeLimits_kW));
    part=table(repmat(id,numel(a),1),reshape(options.ClA_m2(a),[],1), ...
        reshape(cdA(a),[],1),reshape(masses(d),[],1), ...
        repmat(options.OtherEventDischargeLimit_kW,numel(a),1), ...
        reshape(options.BatteryChargeLimits_kW(ch),[],1), ...
        'VariableNames',{'Discipline','ClA_m2','CdA_m2','DriverMass_kg', ...
        'BatteryDischargeLimit_kW','BatteryChargeLimit_kW'});
    report.otherEventManifest=[report.otherEventManifest;part];
end
report.expectedOtherEventCases=height(report.otherEventManifest)*options.RunOtherEvents;
report.files.outputDirectory=string(out);
if options.PrepareOnly
    report.status='prepared; shared modular energy model';
    writetable(manifest,fullfile(out,'prepared_cases.csv'));
    writetable(report.otherEventManifest,fullfile(out,'prepared_other_events.csv'));
    save(fullfile(out,'prepared.mat'),'report');
    if options.ShowProgress
        fprintf('V14 prepared: %d Endurance cases, %d other-event cases.\n', ...
            report.expectedEnduranceCases,report.expectedOtherEventCases);
    end
    return
end
release.report = lts.io.loadR20Reference();
assert(release.report.releasePassed,'Numerical reference is not released.');
base=lts.control.applyIdealAllocation(release.report.vehicle);
base.controls.targetYawMoment_Nm=0;
[envelope,fit]=lts.tire.fitTmeasyQuadraticEnvelope(base,MuScale=0.5);
base.tireEnvelope.peakEnvelope=envelope;
base.tireModel.tmeasyAdtm.muScale=0.5;
report.fitDiagnostics=fit;
report.referenceVehicle=makeVehicle(base,6.6,2,80,80,options,Inf);
report.motorMap=options.MotorEfficiencyMap;
save(fullfile(out,'motor_map.mat'),'report','-v7.3');
report.constantEfficiencies=struct('motorDrive',base.energy.motorEfficiency, ...
    'motorRegen',base.energy.regenMotorEfficiency,'inverter',base.energy.inverterEfficiency, ...
    'gearbox',base.powertrain.gearboxEfficiency);
if ~isfolder(cache), mkdir(cache); end
files=lts.io.trackCatalog();
track=lts.io.loadTrackMatrix(string(files.endurance),true);
rows=struct([]);
for i=1:height(manifest)
    c=manifest(i,:);
    vehicle=makeVehicle(base,c.ClA_m2,c.CdA_m2,c.DriverMass_kg, ...
        c.BatteryDischargeLimit_kW,options,c.BatteryChargeLimit_kW);
    speedKey=sprintf('speed_a%02d_d%02d_p%02d',c.AeroIndex,c.DriverIndex,c.PowerIndex);
    key=sprintf('endu_a%02d_d%02d_p%02d_c%02d', ...
        c.AeroIndex,c.DriverIndex,c.PowerIndex,c.ChargeIndex);
    caseFile=fullfile(cache,[key,'.mat']);
    if isfile(caseFile)
        saved=load(caseFile,'identity','row');
        assert(isequaln(unscaledCapacityMetadata(saved.identity.vehicle),vehicle) ...
            && strcmp(saved.identity.version,version) ...
            && saved.identity.repetitions==options.Repetitions ...
            && isequaln(saved.identity.track,track),'lts:V14:CacheIdentity', ...
            'V14 energy checkpoint mismatch; use a different OutputTag.');
        obsolete=intersect(fieldnames(saved.row), ...
            {'NominalCapacityAt80pctWindow_kWh','NominalCapacity80pctWindow10pctReserve_kWh'});
        if ~isempty(obsolete), saved.row=rmfield(saved.row,obsolete); end
        if isempty(rows), rows=saved.row; else, rows(end+1)=saved.row; end
        continue
    end
    if ~options.UseSocLimits
        speedVehicle=makeVehicle(base,c.ClA_m2,c.CdA_m2,c.DriverMass_kg, ...
            c.BatteryDischargeLimit_kW,options,Inf);
        lap=cachedLap(fullfile(cache,[speedKey,'.mat']),track,speedVehicle, ...
            options.Repetitions,version);
        lap.energy=lts.energy.analyzeLap(lap,vehicle);
        lap.socBoundary=lap.energy.profile.socBoundary;
    else
        speedKey=[key,'_speed'];
        lap=cachedLap(fullfile(cache,[speedKey,'.mat']),track,vehicle, ...
            options.Repetitions,version);
    end
    r=struct('CaseKey',string(key),'ClA_m2',c.ClA_m2,'CdA_m2',c.CdA_m2, ...
        'DriverMass_kg',c.DriverMass_kg, ...
        'BatteryDischargeLimit_kW',c.BatteryDischargeLimit_kW, ...
        'BatteryChargeLimit_kW',c.BatteryChargeLimit_kW);
    e=lap.energy;
    r.Endurance_s=lap.lapTime_s;
    r.NetBatteryEnergy_kWh=e.netBatteryEnergy_kWh;
    r.DischargeEnergy_kWh=e.tractionBatteryEnergy_kWh+e.auxiliaryEnergy_kWh;
    r.RecoveredEnergy_kWh=e.recoveredEnergy_kWh;
    scoringRules=lts.config.scoring.fsg2026();
    scoringReference=scoringRules.events(strcmp({scoringRules.events.method},'efficiencyFactor')).reference;
    r.ScoringEnergy_kWh=lts.scoring.energyForEfficiency(e,scoringReference);
    r.RequiredUsableEnergy_kWh=e.requiredUsableEnergy_kWh;
    r.NominalVoltage_V=vehicle.energy.nominalAccumulatorVoltage_V;
    r.RequiredUsableCapacity_Ah=e.requiredUsableEnergy_kWh*1000/r.NominalVoltage_V;
    r.PeakBatteryDischarge_kW=e.peakBatteryDischargePower_kW;
    r.PeakBatteryCharge_kW=e.peakBatteryChargePower_kW;
    r.PowerViolationEnergy_kWh=e.powerLimitViolationEnergy_kWh;
    r.MapClampedSegments=e.motorMapClampedSegmentCount;
    rule=lts.config.scoring.fsg2026(struct('endurance', ...
        struct('bestTime_s',options.BestEnduranceTime_s),'efficiency', ...
        struct('minimumFactor_s2kWh',options.MinimumEfficiencyFactor_s2kWh)));
    ev=struct('id','endurance','mode','manual','time_s',lap.lapTime_s, ...
        'energy',e);
    scored=lts.scoring.scoreEvents(ev,rule,IncludeDriverless=false);
    r.Endurance_points=scored.events(1).points;
    r.Efficiency_points=scored.events(2).points;
    r.EnduranceEfficiency_points=r.Endurance_points+r.Efficiency_points;
    identity=struct('version',version,'vehicle',vehicle, ...
        'track',track,'repetitions',options.Repetitions);
    row=r;
    temporary=[tempname(cache),'.mat'];
    save(temporary,'identity','speedKey','row','-v7.3');
    [ok,message]=movefile(temporary,caseFile); assert(ok,'%s',message);
    if isempty(rows), rows=r; else, rows(end+1)=r; end %#ok<AGROW>
    if mod(i,numel(options.BatteryChargeLimits_kW))==0
        writetable(struct2table(rows),fullfile(out,'partial_summary.csv'));
    end
    if options.ShowProgress
        fprintf('V14 Endurance %d/%d: ClA %.1f, driver %.0f kg, DC %.0f kW, charge %g kW; %.2f s, %.3f kWh\n', ...
            i,height(manifest),c.ClA_m2,c.DriverMass_kg,c.BatteryDischargeLimit_kW, ...
            c.BatteryChargeLimit_kW,r.Endurance_s,r.NetBatteryEnergy_kWh);
    end
end
report.summary=struct2table(rows);
report.selectedProfiles=selectProfiles(report.summary,cache,out,options);
report.otherEvents=table();
if options.RunOtherEvents
    report.otherEvents=otherEvents(base,files,cdA,cache,version,options);
end
report.status='completed';
writetable(report.summary,fullfile(out,'summary.csv'));
writetable(report.selectedProfiles,fullfile(out,'selected_profiles.csv'));
if ~isempty(report.otherEvents), writetable(report.otherEvents,fullfile(out,'other_events.csv')); end
save(fullfile(out,'report.mat'),'report','-v7.3');
if any(report.summary.DriverMass_kg==80)
    if height(report.summary)==280, analyzeFsg2026PowertrainConceptV14(OutputTag=options.OutputTag); end
end
end

function vehicle=makeVehicle(base,clA,cdA,driver,limit,o,charge)
if nargin<7, charge=o.OtherEventChargeLimit_kW; end
vehicle=lts.powertrain.configureVehicle(base,ClA_m2=clA,CdA_m2=cdA, ...
    DriverMass_kg=driver,BatteryDischargeLimit_kW=limit, ...
    BatteryChargeLimit_kW=charge, ...
    ReferenceCapacity_kWh=o.ReferenceCapacity_kWh,InitialSoc=o.InitialSoc, ...
    UseSocLimits=o.UseSocLimits,MotorModel=o.MotorModel,MotorEfficiencyMap=o.MotorEfficiencyMap);
end

function lap=cachedLap(file,track,vehicle,repetitions,version)
identity=struct('version',version,'vehicle',vehicle, ...
    'track',track,'repetitions',repetitions);
if isfile(file)
    saved=load(file,'identity','lap');
    saved.identity.vehicle=unscaledCapacityMetadata(saved.identity.vehicle);
    assert(isequaln(identity,saved.identity),'lts:V14:CacheIdentity', ...
        'Checkpoint differs; select a different OutputTag: %s',file);
    lap=saved.lap; return
end
model=cachedModel(vehicle);
baseIdentity=struct('version',version,'vehicle',rmfield(vehicle,'energy'),'track',track);
baseFile=fullfile(fileparts(file),sprintf('base_ClA_%g_CdA_%g_driver_%g_%s.mat', ...
    -vehicle.aerodynamics.totalLiftCoefficient*vehicle.aerodynamics.referenceArea_m2, ...
    vehicle.aerodynamics.dragCoefficient*vehicle.aerodynamics.referenceArea_m2, ...
    vehicle.mass.driver_kg,track.meta.name));
% Track names can contain characters unsuitable for Windows file names.
[folder,name,extension]=fileparts(baseFile);
name=regexprep(name,'[^a-zA-Z0-9_.-]','_'); baseFile=fullfile(folder,[name,extension]);
if isfile(baseFile)
    stored=load(baseFile,'baseIdentity','baseLap');
    assert(isequaln(stored.baseIdentity,baseIdentity),'lts:V14:CacheIdentity');
    baseLap=stored.baseLap;
else
    initial=0; if track.isClosed, initial=100; end
    baseLap=lts.solver.solveLap(track,model,initialSpeed_mps=initial,finalSpeed_mps=100);
    save(baseFile,'baseIdentity','baseLap','-v7.3');
end
lap=lts.solver.solveLapWithEnergy(track,model,Repetitions=repetitions, ...
    InitialSpeed_mps=0,BaseLap=baseLap);
temporary=[tempname(fileparts(file)),'.mat'];
save(temporary,'identity','lap','-v7.3');
[ok,message]=movefile(temporary,file); assert(ok,'%s',message);
end

function selected=selectProfiles(summary,cache,out,o)
rows=struct([]);
for clA=unique(summary.ClA_m2).'
    for driver=unique(summary.DriverMass_kg).'
      for charge=unique(summary.BatteryChargeLimit_kW).'
        group=summary(summary.ClA_m2==clA & summary.DriverMass_kg==driver ...
            & summary.BatteryChargeLimit_kW==charge,:);
        [bestPoints,best]=max(group.EnduranceEfficiency_points);
        reference=find(group.BatteryDischargeLimit_kW==80,1);
        lower=find(group.BatteryDischargeLimit_kW<group.BatteryDischargeLimit_kW(best));
        indices=[reference,best]; roles=["reference80","optimum"];
        if ~isempty(lower)
            [~,k]=min(abs(bestPoints-group.EnduranceEfficiency_points(lower)-o.PointLossTarget));
            indices(end+1)=lower(k); roles(end+1)="near7pointsBelow";
        end
        for j=1:numel(indices)
            c=group(indices(j),:);
            name=sprintf('ClA_%g_driver_%g_charge_%g_%s_%gkW.txt',clA,driver, ...
                charge,roles(j),c.BatteryDischargeLimit_kW);
            export=clA==o.ExportClA_m2 && driver==o.ExportDriverMass_kg ...
                && charge==o.ExportChargeLimit_kW;
            if export
                saved=load(fullfile(cache,char(c.CaseKey)+".mat"),'identity','speedKey');
                speedSaved=load(fullfile(cache,[saved.speedKey,'.mat']),'lap');
                lap=speedSaved.lap;
                lap.energy=lts.energy.analyzeLap(lap,saved.identity.vehicle);
                profile=lts.energy.profileTable(lap,saved.identity.vehicle);
                scoringRules=lts.config.scoring.fsg2026();
                reference=scoringRules.events(strcmp({scoringRules.events.method},'efficiencyFactor')).reference;
                [~,profile.CumulativeScoringEnergy_kWh]=lts.scoring.energyForEfficiency(lap.energy,reference);
                writetable(profile,fullfile(out,name),'Delimiter','\t','FileType','text');
            else
                name='';
            end
            r=struct('ClA_m2',clA,'DriverMass_kg',driver, ...
                'BatteryChargeLimit_kW',charge,'Role',roles(j), ...
                'BatteryDischargeLimit_kW',c.BatteryDischargeLimit_kW, ...
                'Points',c.EnduranceEfficiency_points,'LossToOptimum_points', ...
                bestPoints-c.EnduranceEfficiency_points, ...
                'Endurance_s',c.Endurance_s,'RequiredUsableEnergy_kWh',c.RequiredUsableEnergy_kWh, ...
                'File',string(name));
            if isempty(rows), rows=r; else, rows(end+1)=r; end %#ok<AGROW>
        end
      end
    end
end
selected=struct2table(rows);
% A 7-point target outside the grid is not silently represented as exact.
end

function result=otherEvents(base,files,cdA,cache,version,o)
rows=struct([]);
for ai=1:numel(o.ClA_m2)
    for id=["acceleration","skidpad","autocross"]
        masses=driverGrid(o,id);
        for di=1:numel(masses)
            v=makeVehicle(base,o.ClA_m2(ai),cdA(ai),masses(di), ...
                o.OtherEventDischargeLimit_kW,o);
            closed=id=="skidpad";
            track=lts.io.loadTrackMatrix(string(files.(id)),closed);
            key=sprintf('%s_a%02d_d%02d',id,ai,di);
            if ~o.UseSocLimits
                sharedLap=cachedLap(fullfile(cache,[key,'.mat']),track,v,1,version);
            end
          for ci=1:numel(o.BatteryChargeLimits_kW)
            charge=o.BatteryChargeLimits_kW(ci);
            energyVehicle=makeVehicle(base,o.ClA_m2(ai),cdA(ai),masses(di), ...
                o.OtherEventDischargeLimit_kW,o,charge);
            if o.UseSocLimits
                % Charging changes subsequent SoC and hence drive limits.
                chargeKey=sprintf('%s_c%02d.mat',key,ci);
                lap=cachedLap(fullfile(cache,chargeKey),track,energyVehicle,1,version);
                e=lap.energy;
            else
                lap=sharedLap;
                e=lts.energy.analyzeLap(lap,energyVehicle);
            end
            time=lap.lapTime_s;
            if closed
                windows=[52,105;157,210];
                times=interp1(lap.distance_m,lap.time_s,windows,'linear');
                time=mean(times(:,2)-times(:,1));
            end
            r=struct('Discipline',id,'ClA_m2',o.ClA_m2(ai),'CdA_m2',cdA(ai), ...
                'DriverMass_kg',masses(di), ...
                'BatteryDischargeLimit_kW',o.OtherEventDischargeLimit_kW, ...
                'BatteryChargeLimit_kW',charge,'Time_s',time, ...
                'PeakBatteryDischarge_kW',e.peakBatteryDischargePower_kW, ...
                'NetBatteryEnergy_kWh',e.netBatteryEnergy_kWh, ...
                'RequiredUsableEnergy_kWh',e.requiredUsableEnergy_kWh);
            if isempty(rows), rows=r; else, rows(end+1)=r; end %#ok<AGROW>
          end
        end
    end
end

result=struct2table(rows);
end

function model=cachedModel(vehicle)
% Exact memoization; no interpolation or change of the numerical grid.
persistent lastIdentity lastModel lastMemo
identity=rmfield(vehicle,'energy');
if isempty(lastIdentity) || ~isequaln(identity,lastIdentity)
    if ~isempty(lastMemo), clearCache(lastMemo); end
    model=lts.model.dualTrackEnvelope.create(vehicle);
    lastMemo=memoize(model.accelerationLimits);
    lastMemo.CacheSize=50000;
    callback=lastMemo;
    model.accelerationLimits=@(speed,ay) callback(speed,ay);
    lastIdentity=identity;
    lastModel=model;
end
model=lastModel;
model.vehicle=vehicle;
end
function masses=driverGrid(o,id)
masses=o.OtherEventDriverMass_kg;
if isfield(o.DriverMassByDiscipline,id), masses=o.DriverMassByDiscipline.(id); end
assert(isrow(masses) && ~isempty(masses) && all(isfinite(masses)) ...
    && all(masses>0),'lts:V14:DriverGrid','Invalid driver grid for %s.',id);
end

function vehicle=unscaledCapacityMetadata(vehicle)
% These legacy sizing factors do not enter any V14 physics calculation.
vehicle.energy.usableSocWindow=1;
vehicle.energy.finishEnergyReserveFraction=0;
vehicle.energy.endOfLifeCapacityRetention=1;
end

