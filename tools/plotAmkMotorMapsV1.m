function report = plotAmkMotorMapsV1(options)
%PLOTAMKMOTORMAPSV1 Positive AMK motor maps with explicit voltage filtering.
% Run from the project root: addpath('tools'); plotAmkMotorMapsV1
% The 500 V map is a subset of existing operating points, not a new FOC solve.
arguments
    options.SourceFolder (1,1) string = "C:/Octanes/SVN/VehicleDynamics/05_Sonstiges/AMK_Motordata/A2370DD_Matlab"
    options.OutputFolder (1,1) string = ""
    options.MaximumTorque_Nm (1,1) double {mustBePositive} = 21
    options.MaximumShaftPower_kW (1,1) double {mustBePositive} = 35
end
root = fileparts(fileparts(mfilename('fullpath')));
if options.OutputFolder == ""
    options.OutputFolder = fullfile(root,'results','amkMotorMapsV1');
end
assert(~isfolder(options.OutputFolder), ...
    'Output folder already exists. Select a new OutputFolder to preserve it.');
temperature = [80 100 120];
voltage = [500 600];
torque = (0:0.1:options.MaximumTorque_Nm).';
maps = struct([]);
audits = struct([]);
for ti = 1:numel(temperature)
    source = fullfile(options.SourceFolder, ...
        sprintf('A2370DD_T%dC.mat',temperature(ti)));
    s = load(source);
    required = {'Speed','Shaft_Torque','Electromagnetic_Torque', ...
        'Total_Loss','Stator_Copper_Loss','Iron_Loss','Magnet_Loss', ...
        'Mechanical_Loss','Voltage_Line_RMS','Voltage_Phase_RMS', ...
        'Stator_Current_Line_RMS','Stator_Current_Phase_RMS','Power_Factor'};
    assert(all(isfield(s,required)),'Incomplete source: %s',source);
    n = s.Speed(:,1).';
    assert(isequal(size(s.Speed),[201 21]) && all(diff(n)>0));
    residual = s.Total_Loss-(s.Stator_Copper_Loss+s.Iron_Loss ...
        +s.Magnet_Loss+s.Mechanical_Loss);
    assert(max(abs(residual(:)))<1e-7,'Loss sum mismatch.');
    assert(all(s.Electromagnetic_Torque(:)>=-1e-8), ...
        'Source contains generator data: reassess positive-only handling.');
    assert(max(abs(s.Voltage_Line_RMS(:)-s.Voltage_Phase_RMS(:)))<1e-8);
    currentResidual = s.Stator_Current_Line_RMS ...
        -sqrt(3)*s.Stator_Current_Phase_RMS;
    assert(max(abs(currentResidual(:)))<1e-7,'Delta current convention mismatch.');
    shaft = s.Shaft_Torque.*s.Speed*2*pi/60;
    inputFromVI = sqrt(3)*s.Voltage_Line_RMS ...
        .*s.Stator_Current_Line_RMS.*s.Power_Factor;
    electricalResidual = inputFromVI-shaft-s.Total_Loss;
    moving = s.Speed>0;
    assert(max(abs(electricalResidual(moving)))<1e-6, ...
        'Electrical power mismatch outside the zero-speed row.');
    audit = struct('temperature_C',temperature(ti),'source',source, ...
        'maximumLossResidual_W',max(abs(residual(:))), ...
        'maximumMovingPowerResidual_W',max(abs(electricalResidual(moving))), ...
        'rawMaximumShaftPower_kW',max(shaft(:))/1000, ...
        'rawMaximumTorque_Nm',max(s.Shaft_Torque(:)), ...
        'negativeShaftPoints',nnz(s.Shaft_Torque<0));
    audits = [audits; audit]; %#ok<AGROW>
    for vi = 1:numel(voltage)
        map = buildMap(s,n,torque,temperature(ti),voltage(vi),options);
        maps = [maps; map]; %#ok<AGROW>
    end
end
% Verify the 500 V feasible domain is contained in the 600 V domain.
for ti = 1:numel(temperature)
    low = maps((ti-1)*2+1); high = maps((ti-1)*2+2);
    assert(~any(low.valid(:) & ~high.valid(:)));
end
mkdir(options.OutputFolder);
report = struct('version','AMK motor maps V1','created',datetime('now'), ...
    'options',options,'maps',maps,'audits',audits, ...
    'connection','delta','inverterIncluded',false, ...
    'generatorMapPresent',false,'sourceVoltageAssumption_V',600, ...
    'voltageConvention','SVPWM linear range: line RMS <= Udc/sqrt(2)', ...
    'voltage500Method','Voltage-filtered existing 600 V operating points; no new field-weakening optimization');
save(fullfile(options.OutputFolder,'amkMotorMapsV1.mat'),'report','-v7.3');
reference = maps([maps.temperature_C]==100);
for vi = 1:2
    m = reference(vi);
    for quantity = ["efficiency","loss","power"]
        f = newFigure([1400 950]);
        ax = axes(f);
        drawMap(ax,m,quantity,true);
        annotation(f,'textbox',[.08 .015 .84 .065], ...
            'String',plotNote(m),'EdgeColor','none', ...
            'HorizontalAlignment','center','FontSize',11,'Interpreter','none');
        savePlot(f,options.OutputFolder,sprintf('%s_%dV_100C',quantity,m.dcVoltage_V));
    end
end
f = newFigure([1550 850]);
layout = tiledlayout(f,1,2,'TileSpacing','compact','Padding','compact');
colors = lines(3);
for metric = 1:2
    ax = nexttile(layout); hold(ax,'on');
    for mi = 1:numel(maps)
        m = maps(mi); ti = find(temperature==m.temperature_C);
        if m.dcVoltage_V==500, style='--'; else, style='-'; end
        if metric==1, y=m.maximumAvailableTorque_Nm;
        else, y=m.maximumAvailableTorque_Nm.*m.speed_rpm*2*pi/60/1000; end
        plot(ax,m.speed_rpm,y,style,'Color',colors(ti,:), ...
            'LineWidth',1.8,'DisplayName',sprintf('%d V / %d degC',m.dcVoltage_V,m.temperature_C));
    end
    xlabel(ax,'Drehzahl n [rpm]'); grid(ax,'on'); xlim(ax,[0 20000]);
    if metric==1, ylabel(ax,'Wellenmoment M [Nm]'); ylim(ax,[0 23]);
        title(ax,'Verfuegbares positives Wellenmoment');
    else, ylabel(ax,'Wellenleistung P [kW]'); ylim(ax,[0 38]);
        title(ax,'Verfuegbare positive Wellenleistung'); end
    legend(ax,'Location','southwest','FontSize',10);
end
title(layout,{'AMK A2370DD: Grenzen je Motor (Spitzenbetrieb)', ...
    '500 V: Spannungsfilter; keine neu optimierte Feldschwaechung'},'FontSize',15);
savePlot(f,options.OutputFolder,'torque_power_limits');
f = newFigure([1900 1100]);
layout = tiledlayout(f,2,3,'TileSpacing','compact','Padding','compact');
for vi=1:2
    for ti=1:3
        ax=nexttile(layout);
        drawMap(ax,maps((ti-1)*2+vi),"efficiency",false);
    end
end
title(layout,{'Motorwirkungsgrad: Temperatur- und Spannungsvergleich', ...
    '500 V = Filter vorhandener Betriebspunkte; 600 V = angenommene Quelldatenspannung'},'FontSize',15);
savePlot(f,options.OutputFolder,'temperature_efficiency_comparison');
f = newFigure([1550 1100]);
layout = tiledlayout(f,2,2,'TileSpacing','compact','Padding','compact');
components = {'Stator_Copper_Loss','Iron_Loss','Magnet_Loss','Mechanical_Loss'};
labels = {'Kupferverluste','Eisenverluste','Magnetverluste','Mechanische Verluste'};
m = reference(2);
for ci=1:4
    ax=nexttile(layout);
    values=m.components_W.(components{ci}); values(~m.valid)=NaN;
    contourf(ax,m.speed_rpm,m.torque_Nm,values,20,'LineColor','none');
    colormap(ax,turbo(256)); cb=colorbar(ax); cb.Label.String='Verlustleistung [W]';
    hold(ax,'on'); plot(ax,m.speed_rpm,m.maximumAvailableTorque_Nm,'k','LineWidth',1.3);
    xlabel(ax,'Drehzahl n [rpm]'); ylabel(ax,'Wellenmoment M [Nm]');
    title(ax,labels{ci}); grid(ax,'on'); xlim(ax,[0 20000]); ylim(ax,[0 23]);
end
title(layout,'AMK A2370DD: Motorverlustanteile bei 100 degC / 600 V, je Motor','FontSize',15);
savePlot(f,options.OutputFolder,'loss_components_600V_100C');
writeReport(report,options.OutputFolder);
fprintf('AMK maps V1 saved: %s\n',options.OutputFolder);
for mi=1:numel(maps)
    m=maps(mi); eta=m.efficiency_percent;
    fprintf('%d V / %d C: feasible=%d maxEta=%.3f%% maxPower=%.3f kW\n', ...
        m.dcVoltage_V,m.temperature_C,nnz(m.valid),max(eta(:),[],'omitnan'), ...
        max(m.shaftPower_W(m.valid))/1000);
end
end

function m = buildMap(s,n,torque,temperature,voltage,options)
fields = {'Total_Loss','Voltage_Line_RMS','Stator_Current_Line_RMS', ...
    'Stator_Copper_Loss','Iron_Loss','Magnet_Loss','Mechanical_Loss'};
sz=[numel(torque),numel(n)];
values=struct(); for f=fields, values.(f{1})=NaN(sz); end
% Invert each adjacent segment of the original current grid. If multiple
% stored branches reach a torque, choose the feasible minimum-loss branch.
% Never extrapolate, triangulate across gaps, or solve a new FOC strategy.
vmax=voltage/sqrt(2);
for ni=1:numel(n)
    for ci=1:size(s.Shaft_Torque,2)-1
        t0=s.Shaft_Torque(ni,ci); t1=s.Shaft_Torque(ni,ci+1);
        if abs(t1-t0)<1e-10, continue; end
        q=find(torque>=min(t0,t1)-1e-9 & torque<=max(t0,t1)+1e-9);
        if isempty(q), continue; end
        a=(torque(q)-t0)/(t1-t0);
        v=s.Voltage_Line_RMS(ni,ci)+a*(s.Voltage_Line_RMS(ni,ci+1)-s.Voltage_Line_RMS(ni,ci));
        current=s.Stator_Current_Line_RMS(ni,ci)+a*(s.Stator_Current_Line_RMS(ni,ci+1)-s.Stator_Current_Line_RMS(ni,ci));
        loss=s.Total_Loss(ni,ci)+a*(s.Total_Loss(ni,ci+1)-s.Total_Loss(ni,ci));
        feasible=v<=vmax+1e-6 & current<=105+1e-6 ...
            & torque(q)*n(ni)*2*pi/60<=options.MaximumShaftPower_kW*1000+1e-6;
        old=values.Total_Loss(q,ni);
        take=feasible & (isnan(old) | loss<old);
        q=q(take); a=a(take);
        for fi=1:numel(fields)
            x=s.(fields{fi});
            values.(fields{fi})(q,ni)=x(ni,ci)+a*(x(ni,ci+1)-x(ni,ci));
        end
    end
end
valid=isfinite(values.Total_Loss);
[N,M]=meshgrid(n,torque);
power=M.*N*2*pi/60;
eta=100*power./(power+values.Total_Loss);
eta(~valid | N==0 | M==0)=NaN;
limit=NaN(1,numel(n));
for ni=1:numel(n)
    feasible=find(valid(:,ni));
    if ~isempty(feasible), limit(ni)=max(torque(feasible)); end
end
assert(all(values.Total_Loss(valid)>=0));
assert(all(eta(isfinite(eta))>=0 & eta(isfinite(eta))<=100));
assert(all(values.Voltage_Line_RMS(valid)<=vmax+1e-6));
assert(all(power(valid)<=options.MaximumShaftPower_kW*1000+1e-6));
components=struct();
for f={'Stator_Copper_Loss','Iron_Loss','Magnet_Loss','Mechanical_Loss'}
    components.(f{1})=values.(f{1});
end
residual=values.Total_Loss-(components.Stator_Copper_Loss+components.Iron_Loss ...
    +components.Magnet_Loss+components.Mechanical_Loss);
assert(max(abs(residual(valid)))<1e-7);
m=struct('temperature_C',temperature,'dcVoltage_V',voltage, ...
    'speed_rpm',n,'torque_Nm',torque,'valid',valid, ...
    'shaftPower_W',power,'motorInputPower_W',power+values.Total_Loss, ...
    'totalLoss_W',values.Total_Loss,'efficiency_percent',eta, ...
    'lineVoltage_RMS_V',values.Voltage_Line_RMS, ...
    'lineCurrent_RMS_A',values.Stator_Current_Line_RMS, ...
    'components_W',components,'maximumAvailableTorque_Nm',limit);
end

function f=newFigure(dimensions)
f=figure('Visible','off','Color','w', ...
    'Position',[50 50 dimensions],'DefaultAxesFontSize',12);
end

function drawMap(ax,m,quantity,labelContours)
switch quantity
    case "efficiency"
        z=m.efficiency_percent; levels=[0 50 60 70 75 80 82:1:94 96 98 100];
        scale=[75 94]; ticks=76:2:94;
        name='Motorwirkungsgrad'; unit='Motorwirkungsgrad [%]';
        lineLevels=80:2:94;
    case "loss"
        z=m.totalLoss_W/1000; levels=0:.25:7;
        scale=[0 6.5]; ticks=0:1:6;
        name='Motorverluste'; unit='Motorverlustleistung [kW]';
        lineLevels=.5:.5:6;
    case "power"
        z=m.shaftPower_W/1000; levels=0:2:36;
        scale=[0 35]; ticks=0:5:35;
        name='Wellenleistung'; unit='Mechanische Wellenleistung [kW]';
        lineLevels=5:5:35;
end
z(~m.valid)=NaN;
contourf(ax,m.speed_rpm,m.torque_Nm,z,levels,'LineColor','none');
hold(ax,'on');
[c,h]=contour(ax,m.speed_rpm,m.torque_Nm,z,lineLevels,'k','LineWidth',.65);
if labelContours, clabel(c,h,'FontSize',9,'LabelSpacing',450); end
plot(ax,m.speed_rpm,m.maximumAvailableTorque_Nm,'k','LineWidth',1.6);
colormap(ax,turbo(256)); clim(ax,scale);
cb=colorbar(ax); cb.Label.String=unit; cb.Ticks=ticks;
xlim(ax,[0 20000]); ylim(ax,[0 23]); xticks(ax,0:2500:20000);
yticks(ax,0:3:21); xlabel(ax,'Drehzahl n [rpm]'); ylabel(ax,'Wellenmoment M [Nm]');
grid(ax,'on'); ax.Layer='top'; ax.GridAlpha=.18;
if labelContours
    title(ax,{sprintf('AMK A2370DD: %s je Motor',name), ...
        sprintf('%d V DC / %d degC / Dreieckschaltung',m.dcVoltage_V,m.temperature_C)});
    ax.Position=[.08 .17 .74 .70];
else
    title(ax,sprintf('%d V / %d degC',m.dcVoltage_V,m.temperature_C));
end
end

function note=plotNote(m)
if m.dcVoltage_V==500
    basis='500 V: Spannungsfilter der vorhandenen 600-V-Betriebspunkte; keine neue Feldschwaechung.';
else
    basis='600 V: aus der Spannungsgrenze der Quelldaten abgeleitete Basis.';
end
note={basis,'Positiver Spitzenbetrieb, begrenzt auf 21 Nm / 35 kW; weiss = nicht abgedeckt.', ...
    'Nur Motor; ohne Umrichter, Getriebe und Akku. Kein negatives Kennfeld.'};
end

function savePlot(f,folder,name)
cleanup=onCleanup(@()close(f));
exportgraphics(f,fullfile(folder,string(name)+'.png'),'Resolution',180);
savefig(f,fullfile(folder,string(name)+'.fig'));
end

function writeReport(report,folder)
path=fullfile(folder,'Auswertung.md');
fid=fopen(path,'w','n','UTF-8'); assert(fid>=0); cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# AMK A2370DD - Motor-Kennfelder V1\n\n');
fprintf(fid,'Separate Auswertung der vorhandenen AMK-MAT-Dateien; keine Aenderung der Laptime-Modelle.\n\n');
fprintf(fid,'## Basis und Methode\n\n');
fprintf(fid,'- Dreieckschaltung (D). Leiter-/Phasenspannung gleich, Leiterstrom = sqrt(3) * Phasenstrom.\n');
fprintf(fid,'- 80/100/120 degC laut Dateinamen; genaue Temperaturdefinition nicht dokumentiert. Hauptbilder: 100 degC.\n');
fprintf(fid,'- Originalraster: 0 bis 20000 rpm, 100-rpm-Schritte; 21 Stromstufen von 0 bis 105 A RMS.\n');
fprintf(fid,'- Wellenmoment als Achse. Positive Betriebspunkte, %.1f Nm und %.1f kW je Motor als Obergrenzen.\n', ...
    report.options.MaximumTorque_Nm,report.options.MaximumShaftPower_kW);
fprintf(fid,'- Lineare Interpolation innerhalb benachbarter Stromstufen; bei Mehrdeutigkeit geringste Verluste unter den zulaessigen gespeicherten Zweigen. Keine Extrapolation.\n');
fprintf(fid,'- 600 V DC als Basis aus maximal 424.264 V Leiter-RMS / 600 V Leiter-Peak abgeleitet, kein explizites Udc-Metadatum.\n');
fprintf(fid,'- Annahme: lineare SVPWM-Spannungsgrenze U_line_RMS <= Udc/sqrt(2). Umrichter-Spannungsreserve nicht modelliert.\n');
fprintf(fid,'- 500 V: Filter derselben gespeicherten Betriebspunkte bei 353.553 V Leiter-RMS. KEIN neu optimiertes 500-V-Feldschwaechungskennfeld. Hohe Drehzahlen koennen daher zu konservativ abgedeckt sein.\n');
fprintf(fid,'- Weisse Bereiche sind nicht abgedeckt; sie duerfen nicht mit Nullverlusten oder freier Extrapolation gefuellt werden.\n\n');
fprintf(fid,'## Leistungsdefinitionen\n\n');
fprintf(fid,'P_Welle = M_Welle * n * 2*pi/60; P_Motor_el = P_Welle + P_Motorverlust; eta = P_Welle/P_Motor_el.\n\n');
fprintf(fid,'Total_Loss = Kupfer + Eisen + Magnet + Mechanik. Diese Anteile nicht nochmals addieren.\n\n');
fprintf(fid,'Die Wirkungsgrade enthalten keine Umrichter-, Getriebe- oder Akkuverluste. Bei Stillstand kann positive Verlustleistung bestehen, obwohl die Wellenleistung null ist; Wirkungsgrad wird dort nicht angezeigt.\n\n');
fprintf(fid,'## Datenpruefung\n\n');
fprintf(fid,'| Temperatur | Rohdaten-Pmax je Motor | maximaler Verlustsummenfehler | maximaler elektrischer Bilanzfehler bei n > 0 |\n');
fprintf(fid,'|---|---:|---:|---:|\n');
for a=report.audits.'
    fprintf(fid,'| %d degC | %.3f kW | %.3g W | %.3g W |\n', ...
        a.temperature_C,a.rawMaximumShaftPower_kW,a.maximumLossResidual_W,a.maximumMovingPowerResidual_W);
end
fprintf(fid,'\nPower_Factor ist in der Stillstandszeile null und ergibt dort eine falsche elektrische Leistungsberechnung aus U/I/cos(phi). Deshalb erfolgt die Kennfeldberechnung konsequent aus Wellenleistung plus Verlusten. Bei n > 0 stimmen beide Wege ueberein.\n\n');
fprintf(fid,'## Feldschwaechung und Reku\n\n');
fprintf(fid,'Mit zunehmender Drehzahl steigt die Gegen-EMK. Wenn die verfuegbare Spannung nicht mehr ausreicht, wird mit negativem d-Achsenstrom das wirksame Feld reduziert. Das erlaubt hoehere Drehzahlen, kostet Stromreserve und kann Verluste erhoehen. Eine frei spannungsabhaengige Modellierung berechnet fuer beliebiges Udc die zulaessigen Id/Iq-Betriebspunkte neu. Diese V1 tut das nicht.\n\n');
fprintf(fid,'Kein Generator-Kennfeld vorhanden. Kleine negative Wellenmomente in den Rohdaten bei nichtnegativem elektromagnetischem Moment sind keine Reku-Daten. Daher nur positive Plots.\n\n');
fprintf(fid,'Fuer eine spaetere spiegelnde Konzeptannahme koennen Verluste bei gleichem abs(M) und n verwendet werden; das ist keine Generatorvalidierung. Vorzeichenkonvention Wellenleistung positiv zum Antrieb: P_Motor_el = P_Welle + P_Verlust. Im Generatorbetrieb sind beide Leistungen negativ, sofern abs(P_Welle) > P_Verlust. Nicht einfach denselben Wirkungsgrad blind fuer beide Richtungen einsetzen.\n\n');
fprintf(fid,'## Dauer- und Spitzenbetrieb\n\n');
fprintf(fid,'105 A RMS sind laut Datenblatt auf 1.24 s begrenzt. Die gezeigten Obergrenzen sind Spitzenbetrieb, keine Dauerkennlinie. Das Datenblatt nennt 12.3 kW / 9.8 Nm bei 12000 rpm und 13.8 Nm Dauerstillstandsmoment; ausreichende Wasserkuehlung ist erforderlich.\n\n');
fprintf(fid,'Rotortraegheit laut Datenblatt: 2.74 kg cm^2 = 0.000274 kg m^2 je Motor. Noch nicht in die Simulation integriert.\n\n');
fprintf(fid,'Keine VA-/HA-Betriebspunktwolken: Es wurden keine Achs-Betriebspunkte oder Messdaten importiert.\n\n');
fprintf(fid,'## Dateien\n\n');
fprintf(fid,'efficiency/loss/power_500V_100C und _600V_100C: PNG plus editierbare FIG.\n');
fprintf(fid,'torque_power_limits, temperature_efficiency_comparison, loss_components_600V_100C: PNG plus FIG.\n');
fprintf(fid,'amkMotorMapsV1.mat: sechs Kennfelder mit Gueltigkeitsmasken, Grenzen und Herkunft.\n\n');
fprintf(fid,'Ausfuehrung vom Projektverzeichnis: addpath(''tools''); plotAmkMotorMapsV1. Bestehende Ausgabeordner werden nicht ueberschrieben.\n');
end
