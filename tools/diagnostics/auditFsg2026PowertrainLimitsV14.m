function audit = auditFsg2026PowertrainLimitsV14(options)
%AUDITFSG2026POWERTRAINLIMITSV14 Audit four saved reference event profiles.
% Existing study outputs and checkpoints remain unchanged.
arguments
    options.OutputTag (1,1) string = "modular_amk100C_500V"
    options.ClA_m2 (1,1) double = 6.6
    options.DriverMass_kg (1,1) double = 80
    options.BatteryDischargeLimit_kW (1,1) double = 80
    options.BatteryChargeLimit_kW (1,1) double {mustBeNonnegative} = Inf
    options.OutputDirectory (1,1) string = ""
end
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
folder=fullfile(root,'results','fsg2026PowertrainConceptV14',options.OutputTag);
saved=load(fullfile(folder,'report.mat'),'report'); settings=saved.report.options;
ai=find(settings.ClA_m2==options.ClA_m2); di=find(settings.DriverMass_kg==options.DriverMass_kg);
pi=find(settings.BatteryDischargeLimits_kW==options.BatteryDischargeLimit_kW);
assert(isscalar(ai) && isscalar(di) && isscalar(pi),'Requested reference case is not in the saved grid.');
ci=find(settings.BatteryChargeLimits_kW==options.BatteryChargeLimit_kW);
assert(isscalar(ci),'Requested charge limit is not in the saved grid.');
rows=struct([]); audit.events=struct();
for id=["acceleration","skidpad","autocross","endurance"]
    if id=="endurance"
        key=sprintf('speed_a%02d_d%02d_p%02d.mat',ai,di,pi);
        if settings.UseSocLimits
            key=sprintf('endu_a%02d_d%02d_p%02d_c%02d_speed.mat',ai,di,pi,ci);
        end
    else
        masses=settings.OtherEventDriverMass_kg;
        if isfield(settings.DriverMassByDiscipline,id), masses=settings.DriverMassByDiscipline.(id); end
        eventDi=find(masses==options.DriverMass_kg);
        assert(isscalar(eventDi),'Requested driver is not in the event grid.');
        key=sprintf('%s_a%02d_d%02d.mat',id,ai,eventDi);
        if settings.UseSocLimits, key=sprintf('%s_a%02d_d%02d_c%02d.mat',id,ai,eventDi,ci); end
    end
    source=fullfile(folder,'cache',key);
    stored=load(source,'lap','identity'); vehicle=lts.compatibility.normalizeEnergyModel(stored.identity.vehicle);
    vehicle.energy.maximumBatteryChargePower_W=options.BatteryChargeLimit_kW*1000;
    vehicle.tireModel.fittedCombinedSlip.file=fullfile(root,'data','tires','Hoosier_10_16_8_LC0_83.mat');
    vehicle.tireSlipModel.developmentFit.file=vehicle.tireModel.fittedCombinedSlip.file;
    result=lts.analysis.auditLap(stored.lap,vehicle);
    result.meta.sourceFile=source; result.meta.sourceVersion=stored.identity.version;
    audit.events.(id)=result;
    row=result.summary; row.Discipline=id; row.Passed=result.passed;
    if isempty(rows),rows=row;else,rows(end+1)=row;end %#ok<AGROW>
end
audit.summary=struct2table(rows); audit.passed=all(audit.summary.Passed);
audit.meta=struct('created',datetime('now'),'sourceOutputTag',options.OutputTag, ...
    'note','Audit of saved profiles; no simulation rerun or numerical certification.');
if options.OutputDirectory==""
    options.OutputDirectory=fullfile(folder,"audit_"+string(datetime('now','Format','yyyyMMdd_HHmmss')));
end
assert(~isfolder(options.OutputDirectory),'Audit folder exists; select a new OutputDirectory.');
mkdir(options.OutputDirectory); audit.outputDirectory=options.OutputDirectory;
writetable(audit.summary,fullfile(options.OutputDirectory,'limits_audit.csv'));
save(fullfile(options.OutputDirectory,'limits_audit.mat'),'audit');
fid=fopen(fullfile(options.OutputDirectory,'Grenzaudit.md'),'w','n','UTF-8');
assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# V14 Grenzaudit\n\nQuelle: `%s`. Gespeicherte Profile wurden erneut geprüft.\n\n',options.OutputTag);
fprintf(fid,'Gesamtprüfung bestanden: %d. Die CSV enthält zeitgewichtete Überschreitungsanteile.\n\n',audit.passed);
fprintf(fid,'Die Reifenprüfung verwendet eine ideale Null-Giermoment-Obergrenze mit Rollwiderstand. Eine Verletzung beweist einen unzulässigen Segmentmittelpunkt; ein bestandener Check bestätigt kein vollständiges Schlupfmodell. Alte Profile werden nicht korrigiert.\n');
disp(audit.summary);
end
