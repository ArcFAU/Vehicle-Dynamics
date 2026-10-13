function files = plotFsg2026SensitivityV9(options)
%PLOTFSG2026SENSITIVITYV9 Read-only V9 analysis and reproducible figures.
% Does not run simulations or modify existing V9 result files.

arguments
    options.SourceFile (1,1) string = ""
    options.OutputDirectory (1,1) string = ""
end
root = fileparts(mfilename('fullpath'));
if options.SourceFile==""
    candidates=dir(fullfile(root,'results','fsg2026SensitivityV9_modular_energy_v1*.mat'));
    assert(~isempty(candidates),'lts:analysis:StudySource', ...
        'No modular V9 report found. Run the study or select SourceFile explicitly.');
    [~,latest]=max([candidates.datenum]);
    options.SourceFile=string(fullfile(candidates(latest).folder,candidates(latest).name));
end
stored=load(options.SourceFile,'report'); report=stored.report;
aero=report.aero; coupled=report.coupled; one=report.oneFactor;
gate=report.yawGateSummary; audit=report.yawAudit;
assert(height(aero)==30 && height(coupled)==72 && height(one)==39);
assert(height(gate)==8 && height(audit)==16);
[source,stem]=fileparts(options.SourceFile);
if options.OutputDirectory==""
    options.OutputDirectory=fullfile(source,stem+"_plots_"+string(datetime('now','Format','yyyyMMdd_HHmmss')));
end
out=options.OutputDirectory;
assert(~isfolder(out),'lts:analysis:PlotDirectory','Select a new output directory.');
mkdir(out);
files = strings(6,1);

% Aero map: full factorial CL*A and CD*A, no interpolation.
cl = unique(aero.ClA_m2);
cd = unique(aero.CdA_m2);
z = nan(numel(cl),numel(cd));
for i=1:height(aero)
    z(cl==aero.ClA_m2(i),cd==aero.CdA_m2(i)) = aero.DynamicPoints(i);
end
f = newFigure();
imagesc(cd,cl,z); axis xy; colorbar;
xlabel('C_D A (m^2)'); ylabel('C_L A (m^2)');
title('Aero: dynamische FSG-Punkte');
xticks(cd); yticks(cl);
for i=1:numel(cl)
    for j=1:numel(cd)
        text(cd(j),cl(i),sprintf('%.0f',z(i,j)), ...
            'HorizontalAlignment','center','FontWeight','bold', ...
            'Color',contrastColor(z(i,j),z));
    end
end
files(1) = writePlot(f,out,'01_aero_punkte.png');

% Top-10 threshold is overall 628.83 minus simulated dynamic points.
f = newFigure(); hold on;
for j=1:numel(cd)
    rows = aero(aero.CdA_m2==cd(j),:);
    rows = sortrows(rows,'ClA_m2');
    plot(rows.ClA_m2,rows.RequiredStaticForTop10_points,'-o', ...
        'LineWidth',1.8,'DisplayName',sprintf('C_D A = %.1f m^2',cd(j)));
end
yline(200,'--','200 Statikpunkte','LabelHorizontalAlignment','left', ...
    'HandleVisibility','off');
yline(225,':','225 Statikpunkte','LabelHorizontalAlignment','left', ...
    'HandleVisibility','off');
xlabel('C_L A (m^2)'); ylabel('Benötigte Statikpunkte');
title('Top-10-Schwelle: 628,83 Gesamtpunkte');
legend('Location','northeastoutside'); grid on;
files(2) = writePlot(f,out,'02_aero_top10_schwelle.png');

% Independent one-factor sweeps; never imply simultaneous optimization.
f = newFigure();
factors = ["ReferenceTotalMass_kg","CgHeight_m", ...
    "SystemPower_kW","MotorTorque_Nm"];
labels = ["Referenz-Gesamtmasse (kg)","Schwerpunkthöhe (m)", ...
    "Systemleistung (kW)","Motormoment (Nm)"];
for k=1:4
    subplot(2,2,k);
    rows = one(string(one.Factor)==factors(k),:);
    rows = sortrows(rows,'FactorValue');
    plot(rows.FactorValue,rows.DynamicPoints,'-o','LineWidth',1.8);
    xlabel(labels(k)); ylabel('Dynamische Punkte'); grid on;
end
sgtitle('Einzelfaktoren bei sonst unverändertem Basisfahrzeug');
files(3) = writePlot(f,out,'03_masse_cog_leistung_moment.png');

% Coupled 4 x 6 grid for each aero-balance. Each cell is an actual case.
f = newFigure();
ab = unique(coupled.FrontAeroBalance);
mf = unique(coupled.FrontMassFraction);
rf = unique(coupled.FrontRollMomentFraction);
limits = [min(coupled.DynamicPoints),max(coupled.DynamicPoints)];
for k=1:numel(ab)
    subplot(1,3,k);
    m = nan(numel(rf),numel(mf));
    rows = coupled(coupled.FrontAeroBalance==ab(k),:);
    for i=1:height(rows)
        m(rf==rows.FrontRollMomentFraction(i), ...
          mf==rows.FrontMassFraction(i)) = rows.DynamicPoints(i);
    end
    imagesc(mf*100,rf*100,m,limits); axis xy;
    title(sprintf('Aerobalance vorne %.0f %%',ab(k)*100));
    xlabel('Vorderachslast (%)');
    if k==1, ylabel('Rollmomentanteil vorne (%)'); end
    xticks(mf*100); yticks(rf*100);
end
cb = colorbar; cb.Position = [0.92 0.14 0.02 0.72];
sgtitle('Gekoppelte Balance: dynamische Punkte');
files(4) = writePlot(f,out,'04_gekoppelte_balance.png');

% Regen study: points and energy; no claimed lap-time effect.
rows = one(string(one.Factor)=="RegenPowerLimit_kW",:);
rows = sortrows(rows,'FactorValue');
f = newFigure();
subplot(1,2,1);
plot(-rows.FactorValue,rows.Efficiency_points,'-o','LineWidth',1.8);
xlabel('Betrag Reku-Leistungslimit (kW)');
ylabel('Efficiency-Punkte'); grid on;
subplot(1,2,2); hold on;
plot(-rows.FactorValue,rows.EnduranceRecoveredEnergy_kWh,'-o', ...
    'LineWidth',1.8,'DisplayName','Reku-Energie');
plot(-rows.FactorValue,rows.EnduranceFrictionBrakeEnergy_kWh,'-s', ...
    'LineWidth',1.8,'DisplayName','Reibbremse');
xlabel('Betrag Reku-Leistungslimit (kW)');
ylabel('Endurance-Bremsenergie (kWh)'); legend('Location','best'); grid on;
sgtitle('Reku ohne künstliche 4-m/s^2-Grenze: Energie-Modellobergrenze');
files(5) = writePlot(f,out,'05_reku_energie_punkte.png');

% Yaw moment gate is diagnostic only. No lap-time or points effect.
f = newFigure(); hold on;
ev = ["autocross","endurance"];
for k=1:numel(ev)
    rows = gate(string(gate.Event)==ev(k),:);
    rows = sortrows(rows,'YawInertia_kgm2');
    plot(rows.YawInertia_kgm2,rows.TargetMomentFeasibleCount, ...
        '-o','LineWidth',1.8,'DisplayName',char(ev(k)));
end
yline(20,'--','20 geprüfte Punkte','HandleVisibility','off');
xlabel('Gierträgheit I_z (kg m^2)');
ylabel('Zustände mit machbarem Giermoment / 20');
title('Gierträgheit: Validierungsgate, keine Rundenzeitwirkung');
legend('Location','southwest'); grid on;
files(6) = writePlot(f,out,'06_giertraegheit_gate.png');

disp(files);
end

function f = newFigure()
f = figure('Visible','off','Color','w','Position',[100 100 1250 720]);
end

function file = writePlot(f,folder,name)
file = string(fullfile(folder,name));
exportgraphics(f,file,'Resolution',170);
close(f);
end

function c = contrastColor(value,matrix)
fraction = (value-min(matrix(:)))/(max(matrix(:))-min(matrix(:)));
if fraction>0.62, c=[0 0 0]; else, c=[1 1 1]; end
end
