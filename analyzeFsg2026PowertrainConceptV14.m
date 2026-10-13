function analysis=analyzeFsg2026PowertrainConceptV14(options)
%ANALYZEFSG2026POWERTRAINCONCEPTV14 Capacity and points from completed V14 CSV.
arguments
    options.OutputTag (1,1) string = "modular_amk100C_500V"
    options.DriverMass_kg (1,1) double = 80
    options.EnergyOnly (1,1) logical = false
    options.UncappedOnly (1,1) logical = false
    options.CompareV13 (1,1) logical = false
end
root=fileparts(mfilename('fullpath'));
out=fullfile(root,'results','fsg2026PowertrainConceptV14',options.OutputTag);
s=readtable(fullfile(out,'summary.csv'),'TextType','string');
assert(height(s)==280,'V14 Endurance grid is incomplete.');
assert(all(isfinite(s.Endurance_s)) && all(s.RequiredUsableEnergy_kWh>0));
assert(all(abs(s.NetBatteryEnergy_kWh ...
    -(s.DischargeEnergy_kWh-s.RecoveredEnergy_kWh))<1e-8));
assert(all(abs(s.ScoringEnergy_kWh ...
    -(s.DischargeEnergy_kWh-0.9*s.RecoveredEnergy_kWh))<1e-8));
assert(all(s.PowerViolationEnergy_kWh<1e-8));
assert(all(s.RecoveredEnergy_kWh(s.BatteryChargeLimit_kW==0)==0));
if options.UncappedOnly
    saved=load(fullfile(out,'analysis.mat'),'analysis'); analysis=saved.analysis;
    reportText=fileread(fullfile(out,'Auswertung.md'));
    marker=strfind(reportText,'## Ungedeckelte Konzeptpunkte');
    if ~isempty(marker), reportText=reportText(1:marker(1)-1); end
    fid=fopen(fullfile(out,'Auswertung.md'),'w','n','UTF-8'); assert(fid>=0);
    cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',reportText);
    [names,uncapped]=uncappedConcept(s,out,fid);
    analysis.plotFiles=unique([analysis.plotFiles,names],'stable');
    analysis.uncappedOptima=uncapped;
    save(fullfile(out,'analysis.mat'),'analysis');
    return
end
if options.EnergyOnly
    names=energyBreakdownPlots(s,out);
    saved=load(fullfile(out,'analysis.mat'),'analysis'); analysis=saved.analysis;
    analysis.plotFiles=unique([analysis.plotFiles,names],'stable');
    save(fullfile(out,'analysis.mat'),'analysis');
    report=fileread(fullfile(out,'Auswertung.md'));
    heading='## Akkuenergie getrennt ohne Punktedeckel';
    marker=strfind(report,heading);
    if ~isempty(marker), report=report(1:marker(1)-1); end
    fid=fopen(fullfile(out,'Auswertung.md'),'w','n','UTF-8'); assert(fid>=0);
    cleanup=onCleanup(@()fclose(fid)); fprintf(fid,'%s',report);
    writeEnergySection(fid,s,names,options.DriverMass_kg);
    return
end
groups=unique(s(:,{'ClA_m2','DriverMass_kg','BatteryChargeLimit_kW'}),'rows');
best=s([],:);
for i=1:height(groups)
    g=s(s.ClA_m2==groups.ClA_m2(i) & s.DriverMass_kg==groups.DriverMass_kg(i) ...
        & s.BatteryChargeLimit_kW==groups.BatteryChargeLimit_kW(i),:);
    [~,k]=max(g.EnduranceEfficiency_points); best=[best;g(k,:)]; %#ok<AGROW>
end
writetable(best,fullfile(out,'optima.csv'));
driver=options.DriverMass_kg;
assert(any(s.DriverMass_kg==driver),'Requested plot driver is missing.');
charges=unique(s.BatteryChargeLimit_kW).'; aeros=unique(s.ClA_m2).';
labels=string(compose('%g kW',charges)); labels(isinf(charges))="unbegrenzt";
fields={'RequiredUsableEnergy_kWh','EnduranceEfficiency_points','Endurance_s', ...
    'Endurance_points','Efficiency_points'};
titles={'Nutzbare Energie am Akku-Ausgang [kWh]', ...
    'Endurance + Efficiency [Punkte]','Endurance-Zeit [s]', ...
    'Endurance [Punkte]','Efficiency [Punkte]'};
names={'capacity_vs_discharge.png','points_vs_discharge.png','time_vs_discharge.png', ...
    'endurance_points_vs_discharge.png','efficiency_points_vs_discharge.png'};
for fi=1:numel(fields)
    f=figure('Visible','off','Position',[50,50,1400,850]);
    cleanup=onCleanup(@()close(f));
    layout=tiledlayout(2,3,'TileSpacing','compact');
    for clA=aeros
        ax=nexttile(layout); hold(ax,'on');
        for charge=charges
            g=s(s.ClA_m2==clA & s.DriverMass_kg==driver ...
                & s.BatteryChargeLimit_kW==charge,:);
            g=sortrows(g,'BatteryDischargeLimit_kW');
            plot(ax,g.BatteryDischargeLimit_kW,g.(fields{fi}),'-o','LineWidth',1.3);
        end
        title(ax,sprintf('ClA = %g m²',clA)); xlabel(ax,'Entladelimit am Akku [kW]');
        ylabel(ax,titles{fi}); grid(ax,'on');
    end
    legend(ax,cellstr(labels),'Location','best');
    title(layout,sprintf('V14: Fahrer %g kg; AMK-Motorkennfeld 100 C / 500 V',driver));
    exportgraphics(f,fullfile(out,names{fi}),'Resolution',180);
    clear cleanup
end
extraNames=additionalPlots(s,out,driver,charges,labels,aeros);
names=[names,extraNames];
other=readtable(fullfile(out,'other_events.csv'),'TextType','string');
assert(height(other)==120,'V14 other-event grid is incomplete.');
capacity=s;
capacity.PeakDischargeC_rate=capacity.PeakBatteryDischarge_kW./capacity.RequiredUsableEnergy_kWh;
capacity.PeakChargeC_rate=capacity.PeakBatteryCharge_kW./capacity.RequiredUsableEnergy_kWh;
writetable(capacity,fullfile(out,'capacity_sizing.csv'));
references=struct('acceleration',struct('bestTime_s',3.369), ...
    'skidpad',struct('bestTime_s',4.678),'autocross',struct('bestTime_s',72.925));
rules=lts.config.scoring.fsg2026(references);
other.Points=zeros(height(other),1);
for i=1:height(other)
    event=struct('id',char(other.Discipline(i)),'mode','manual','time_s',other.Time_s(i));
    scored=lts.scoring.scoreEvents(event,rules,IncludeDriverless=false);
    other.Points(i)=scored.totalPoints;
end
writetable(other,fullfile(out,'other_events_scored.csv'));
names=[names,otherPlots(other,out,driver,charges,labels,aeros)];
names=[names,timeComparisons(s,other,out,driver,aeros,charges,labels)];
names=[names,regenComparisons(s,other,out,aeros,charges,labels)];
energyNames=energyBreakdownPlots(s,out); names=[names,energyNames];
analysis=struct('caseCount',height(s),'optima',best, ...
    'minimumRequiredUsableEnergy_kWh',min(s.RequiredUsableEnergy_kWh), ...
    'maximumRequiredUsableEnergy_kWh',max(s.RequiredUsableEnergy_kWh), ...
    'plotFiles',{names},'otherEventReferences',references);
save(fullfile(out,'analysis.mat'),'analysis');
file=fopen(fullfile(out,'Auswertung.md'),'w','n','UTF-8');
assert(file>=0); cleanup=onCleanup(@()fclose(file));
fprintf(file,'# V14: AMK-Motorkennfeld 100 C / 500 V, Akku-Leistung und Kapazität\n\n');
fprintf(file,'Ausgewertet: %d Endurance-Fälle. Benötigte nutzbare Energie über das gesamte Raster: %.3f bis %.3f kWh.\n\n', ...
    height(s),analysis.minimumRequiredUsableEnergy_kWh,analysis.maximumRequiredUsableEnergy_kWh);
fprintf(file,'## AMK-Kennfeld und fortbestehende Modellgrenzen\n\nMotorverluste bei 100 C; positive Verlustseite für Reku gespiegelt. Konstante Akkuspannung 500 V, Leiter-Leiter-Spannungsgrenze 500/sqrt(2) = 353.55 V RMS. Kein neu optimiertes Feldschwächungskennfeld und kein thermisches Modell. Umrichter/Getriebe wie in V13 separat. Leerlaufverluste bei null angefordertem Wellenmoment sind nicht bilanziert.\n\nDie bekannten V13-Reifen-/Segmentmittelpunkt- und radweisen Reku-Verteilungsprobleme sind NICHT behoben. Diese Auswertung ist keine erneute Freigabe der Fahrphysik. Zeiten, Punkte und Kapazität bleiben Konzeptwerte. Unterschiede zu V13 bündeln Motorkennfeld und Spannungslimit; sie sind kein isolierter Wirkungsgradeffekt.\n\n');
fprintf(file,'Der Kapazitätsrohwert ist die maximale kumulierte Entnahme am Akku-Ausgang. Keine pauschale SoC- oder Reserve-Skalierung.\n\n');
if isfile(fullfile(out,'limits_audit.csv'))
    limitsAudit=readtable(fullfile(out,'limits_audit.csv'),'TextType','string');
    fprintf(file,'## Numerischer Grenzaudit: Konzeptwerte noch nicht freigegeben\n\n');
    fprintf(file,'Die gespeicherten Kurvenprofile verletzen teilweise die momentane Hüllkurve am Segmentmittelpunkt. Das betrifft die Strecken-/Profil-Konsistenz; korrekte Energieintegration allein ist keine vollständige physikalische Validierung.\n\n');
    fprintf(file,'| Disziplin, ClA=6.6 / 80 kg / 80 kW | Modell am Mittelpunkt unzulässig [Zeit-%%] | Kombinierte Reifenlast >102 %% [Zeit-%%] | Höchste kombinierte Auslastung |\n|---|---:|---:|---:|\n');
    g=limitsAudit(limitsAudit.ClA_m2==6.6 & limitsAudit.BatteryDischargeLimit_kW==80,:);
    for i=1:height(g)
        fprintf(file,'| %s | %.2f | %.2f | %.3f |\n',g.Discipline(i), ...
            g.SolverModelMidpointInfeasible_pct(i),g.TireMidpointOver102pct_pct(i), ...
            g.MaxCombinedTireUtilization(i));
    end
    fprintf(file,'\nDie Audit-Reifenprüfung enthält Rollwiderstand und eine interpolierte Krümmung am Segmentmittelpunkt. Der Originalmodell-Vergleich benutzt denselben Mittelpunkt, aber dessen originale Kraftdefinition. Die Überschreitungen sind ein Prüfbedarf für Segmentinterpolation und Grenzdurchsetzung, kein experimenteller Validierungsnachweis. Zeiten, Punkte und Kapazitäten gelten bis zur Klärung als Konzeptwerte.\n\n');
    if isfile(fullfile(out,'regen_limits_audit.csv'))
        regenAudit=readtable(fullfile(out,'regen_limits_audit.csv'));
        g=regenAudit(regenAudit.ClA_m2==6.6 & regenAudit.DriverMass_kg==driver ...
            & regenAudit.BatteryDischargeLimit_kW==80,:);
        fprintf(file,'### Radweise Reku-Grenzen\n\nDie gespeicherte Energieauswertung begrenzt das gesamte Rekumoment. Der Audit begrenzt dagegen jeden Motor zusätzlich anhand der jeweiligen Reifenreserve und des Null-Giermoments. Bei unbegrenzt Akku-Ladelimit liegen relevante Überschreitungen vor; der Energiebedarf ist deshalb noch kein belastbares Akkuziel.\n\n');
        fprintf(file,'| Ladelimit [kW] | Radweise Rekugrenze >102 %% [Bremszeit-%%] | Davon bei Reifen-Gesamtauslastung <=102 %% [Bremszeit-%%] |\n|---:|---:|---:|\n');
        for i=1:height(g)
            fprintf(file,'| %g | %.2f | %.2f |\n',g.BatteryChargeLimit_kW(i), ...
                g.WheelRegenOver102pct_pctOfBraking(i),g.WheelRegenOver102pctOnTireFeasible_pctOfBraking(i));
        end
    end
end
fprintf(file,'## Endurance getrennt\n\nReku beeinflusst bei deaktivierter SoC-Leistungsreduktion die Bremszeit nicht: die Reibbremse ergänzt das fehlende Rekumoment. Die Endurance-Zeiten sind deshalb über die fünf Ladelimits identisch. Das Zeitoptimum im berechneten Raster liegt bei 80 kW Entladung.\n\n');
fprintf(file,'| ClA [m²] | Zeit bei 80 kW [s] | Endurance-Punkte |\n|---:|---:|---:|\n');
base=s(s.DriverMass_kg==driver & s.BatteryDischargeLimit_kW==80 & s.BatteryChargeLimit_kW==Inf,:);
for i=1:height(base)
    fprintf(file,'| %g | %.2f | %.2f |\n',base.ClA_m2(i),base.Endurance_s(i),base.Endurance_points(i));
end
fprintf(file,'\n[Endurance-Heatmaps](Endurance_points_heatmaps.png) · [Endurance-Punkte über Leistung](endurance_points_vs_discharge.png)\n\n');
fprintf(file,'## Efficiency getrennt\n\nDie Wertung verwendet T² mal (entnommene Energie minus 0.9 mal Rekuenergie). Maximal 75 Punkte. Ob der Punktedeckel erreicht wird, hängt vom jeweiligen Fall ab. Die kWh- und Zeitdiagramme bleiben unabhängig davon aussagefähig.\n\n');
fprintf(file,'| Entladung [kW] | Reku-Limit [kW] | Efficiency-Punkte bei ClA=6.6 | Wertungsenergie [kWh] |\n|---:|---:|---:|---:|\n');
eff=s(s.DriverMass_kg==driver & s.ClA_m2==6.6 ...
    & ismember(s.BatteryDischargeLimit_kW,[55,80]) & ~isinf(s.BatteryChargeLimit_kW),:);
eff=sortrows(eff,{'BatteryDischargeLimit_kW','BatteryChargeLimit_kW'});
for i=1:height(eff)
    fprintf(file,'| %g | %g | %.2f | %.3f |\n',eff.BatteryDischargeLimit_kW(i), ...
        eff.BatteryChargeLimit_kW(i),eff.Efficiency_points(i),eff.ScoringEnergy_kWh(i));
end
fprintf(file,'\n[Efficiency-Heatmaps](Efficiency_points_heatmaps.png) · [Efficiency-Punkte über Leistung](efficiency_points_vs_discharge.png)\n\n');
fprintf(file,'## Benötigte Akkugröße\n\nEnergieminimum bei 80 kW Entladung und %g kg Fahrer; ohne pauschale Reserve und ohne begrenztes SoC-Fenster:\n\n',driver);
fprintf(file,'| ClA | Ohne Reku [kWh] | 40 kW Reku [kWh] | Unbegrenzt Reku [kWh] |\n|---:|---:|---:|---:|\n');
for clA=aeros
    g=s(s.DriverMass_kg==driver & s.ClA_m2==clA & s.BatteryDischargeLimit_kW==80,:);
    fprintf(file,'| %g | %.3f | %.3f | %.3f |\n',clA, ...
        g.RequiredUsableEnergy_kWh(g.BatteryChargeLimit_kW==0), ...
        g.RequiredUsableEnergy_kWh(g.BatteryChargeLimit_kW==40), ...
        g.RequiredUsableEnergy_kWh(g.BatteryChargeLimit_kW==Inf));
end
fprintf(file,'\nEnergieminimum ist nicht gleich vollständig ausgelegtes Akkupaket: Spitzen-C-Raten relativ zu diesem Minimum sind P[kW]/E[kWh]. Zellspannungen, Innenwiderstand, thermische Grenzen und zulässige Lade-/Entladeströme können ein größeres Paket erfordern. capacity_sizing.csv enthält alle 280 Fälle und die Spitzen-P/E-Verhältnisse. Die bekannte V13-Grenzprüfung bleibt ein Warnhinweis, kein neuer V14-Audit: [V13-Grenzaudit](../../fsg2026PowertrainConceptV13/constant_charge_sweep/Grenzen_und_Akkugroesse.md).\n\n');
fprintf(file,'## Punkteoptima für %g kg Fahrergewicht\n\n',driver);
fprintf(file,'| ClA [m²] | Ladelimit [kW] | Entladelimit [kW] | Endu [s] | Endu-Punkte | Eff.-Punkte | Benötigte Energie am Akku-Ausgang [kWh] |\n');
fprintf(file,'|---:|---:|---:|---:|---:|---:|---:|\n');
b=best(best.DriverMass_kg==driver,:);
for i=1:height(b)
    label=sprintf('%g',b.BatteryChargeLimit_kW(i));
    if isinf(b.BatteryChargeLimit_kW(i)), label='unbegrenzt'; end
    fprintf(file,'| %g | %s | %g | %.2f | %.2f | %.2f | %.3f |\n', ...
        b.ClA_m2(i),label,b.BatteryDischargeLimit_kW(i),b.Endurance_s(i), ...
        b.Endurance_points(i),b.Efficiency_points(i),b.RequiredUsableEnergy_kWh(i));
end
fprintf(file,'\nGleiche Punktesummen können durch die Punktedeckel entstehen. Das gewählte Optimum ist dann ein Rastermaximum, kein eindeutig bestimmtes physikalisches Optimum.\n\n');
fprintf(file,'Der Kapazitätsbedarf enthält weder ein begrenztes SoC-Fenster noch pauschale Reserve- oder Alterungsfaktoren. Zellverluste, Temperatur und Strombelastbarkeit folgen aus der Zellsimulation. Der virtuelle 100-kWh-Akku legt weder Fahrzeugmasse noch Kapazitätsziel fest.\n\n');
fprintf(file,'18 Runden je Fall, ein konstantes Fahrergewicht; Fahrerwechsel und Akkumasse-Kopplung sind nicht modelliert. Motor-, Reifen- und Drehzahlgrenzen gelten auch bei unbegrenzter Ladeleistung.\n\n');
fprintf(file,'## Andere Disziplinen\n\nAcceleration, Skidpad und Autocross: 80 kW Akku-Entladelimit. Aero und Fahrergewicht werden variiert. Reku beeinflusst in diesem Modell die Energie, aber nicht die Fahrzeit; eine Leistungssensitivität wurde für diese Disziplinen in V14 nicht simuliert.\n\n');
fprintf(file,'Punktereferenzen aus der bisherigen Projektwertung: Acceleration 3.369 s, Skidpad 4.678 s, Autocross 72.925 s.\n\n');
g=other(other.DriverMass_kg==driver & other.Discipline=="autocross" & other.BatteryChargeLimit_kW==Inf,:);
no=g(g.ClA_m2==0,:); aeroCase=g(g.ClA_m2==6.6,:);
fprintf(file,'Autocross wird durch Aero schneller: ohne Abtrieb %.2f s, mit ClA=6.6 %.2f s. Der Punkteplot erreicht mit Aero den 100-Punkte-Deckel; deshalb unbedingt den Zeitplot mitlesen.\n\n',no.Time_s,aeroCase.Time_s);
fprintf(file,'Acceleration: Die V13-Grenzdiagnose wurde mit konstanten Wirkungsgraden gerechnet und darf nicht als V14-Grenzaufteilung interpretiert werden. V14 verwendet zusätzliche kennfeld- und spannungsabhängige Grenzen.\n\n');
fprintf(file,'## Diagramme\n\nDie Hauptdiagramme nutzen %g kg Fahrergewicht. Bei ClA=0 gilt CdA=0.8; bei allen Aero-Fällen CdA=2. Das ist beim Vergleich der Aero-Kurven zu beachten.\n\n',driver);
fprintf(file,'## Reku-Vergleich für alle Fahrergewichte\n\nFür beide Fahrergewichte zeigen die regen_*_driver_XXkg.png-Diagramme den Einfluss der Akku-Ladelimits 0, 40, 60, 80 kW und unbegrenzt. Je Aero-Setup werden sämtliche Entladelimits gegenübergestellt. EnergySaving_kWh in regen_comparison.csv ist die Reduktion der benötigten nutzbaren Energie gegenüber 0 kW Reku bei identischem Aero, Fahrergewicht und Entladelimit; sie ist nicht zwingend gleich der gesamten zurückgewonnenen Energie.\n\n');
fprintf(file,'Die Efficiency-Diagramme bleiben von den Endurance-Diagrammen getrennt. regen_efficiency_vs_discharge_driver_XXkg.png und regen_combined_vs_discharge_driver_XXkg.png zeigen alle Ladelimits über Entladeleistung. Die anderen Disziplinen werden in regen_other_energy_driver_XXkg.png bei den tatsächlich simulierten 80 kW Entladung dargestellt. Reku verändert in diesem Modell keine Fahrzeit. Null-Reku und identische Zeiten über Ladelimits werden numerisch geprüft. Der 120-kW-Ladefall ist in V14 nicht enthalten. Die Energiegewinne bei hoher Reku sind wegen der bekannten radweisen Grenzprobleme vorläufig.\n\n');
fprintf(file,'## Zeitbasierte Vergleiche ohne Punktedeckel\n\nDie Datei time_energy_comparison.csv enthält alle 400 Fälle mit Zeiten, Energie und Punkten. DeltaTime_s und DeltaTime_pct beziehen sich je Disziplin und Fahrergewicht auf ClA=6.6, 80 kW Entladung und unbegrenzt Ladung. Negative Werte bedeuten schneller. Die Referenz hält das Fahrergewicht konstant; damit wird der Aero-/Leistungseinfluss nicht mit Fahrergewicht vermischt.\n\n');
fprintf(file,'Die neuen Zeitdiagramme zeigen absolute Zeiten und Änderungen gegenüber dieser Referenz. Efficiency hat keine separate Fahrzeit: energy_vs_endurance_time.png zeigt den Energie-Zeit-Kompromiss, getrennt nach Ladelimit. Alle Ladelimits bleiben in der CSV enthalten; identische Zeitkurven werden im Aero-/Leistungsdiagramm nur einmal dargestellt. Andere Disziplinen wurden nur mit 80 kW Entladung gerechnet. Keine zusätzlichen Simulationen und keine neuen Leistungssensitivitäten werden daraus abgeleitet. Die bekannten physikalischen Audit-Einschränkungen gelten unverändert.\n\n');
fprintf(file,'## Beide Fahrergewichte\n\nDie Zeit- und Energie-Zeit-Diagramme sind zusätzlich für 60 und 80 kg gespeichert (Suffix driver_XXkg). Jeder Lauf verwendet ein konstantes Fahrergewicht. Verschiedene Fahrer je Disziplin lassen sich aus den Zeilen separat wählen; Fahrerwechsel innerhalb Endurance ist nicht modelliert. driver_weight_times.png und driver_weight_energy.png vergleichen die Gewichte direkt bei 80 kW Entladung und unbegrenzt Ladung für alle Aero-Setups.\n\n');
fprintf(file,'Referenz-Setup ClA=6.6, CdA=2, Entladung 80 kW, Ladung unbegrenzt:\n\n| Fahrer [kg] | Acceleration [s] | Skidpad [s] | Autocross [s] | Endurance [s] | Endurance-Akkuenergie [kWh] |\n|---:|---:|---:|---:|---:|---:|\n');
for mass=unique(s.DriverMass_kg).'
    row=s(s.DriverMass_kg==mass & s.ClA_m2==6.6 & s.BatteryDischargeLimit_kW==80 & s.BatteryChargeLimit_kW==Inf,:);
    times=zeros(1,3); ids=["acceleration","skidpad","autocross"];
    for ei=1:3
        r=other(other.DriverMass_kg==mass & other.ClA_m2==6.6 & other.BatteryChargeLimit_kW==Inf & other.Discipline==ids(ei),:);
        assert(height(r)==1); times(ei)=r.Time_s;
    end
    fprintf(file,'| %g | %.3f | %.3f | %.3f | %.2f | %.3f |\n',mass,times(1),times(2),times(3),row.Endurance_s,row.RequiredUsableEnergy_kWh);
end
fprintf(file,'\nMehr Gewicht verändert nicht nur die Trägheit, sondern auch Radlasten und Reifen-Lastsensitivität. Effekte sind daher nicht zwingend linear. Die bekannten Grenzaudit-Probleme gelten weiterhin für diese Konzeptvergleiche.\n\n');
for i=1:numel(names)
    fprintf(file,'- [%s](%s)\n',strrep(names{i},'_',' '),names{i});
end
writeEnergySection(file,s,energyNames,driver);
if options.CompareV13
    referenceCsv=fullfile(root,'results','fsg2026PowertrainConceptV13', ...
        'constant_charge_sweep','time_energy_comparison.csv');
    assert(isfile(referenceCsv),'lts:V14:HistoricalComparison', ...
        'CompareV13=true requires the historical V13 CSV: %s',referenceCsv);
    analysis.comparisonV13=compareV13(s,other,out,root,file);
else
    analysis.comparisonV13=table();
end
[uncappedNames,uncapped]=uncappedConcept(s,out,file);
analysis.plotFiles=unique([analysis.plotFiles,uncappedNames],'stable');
analysis.uncappedOptima=uncapped;
save(fullfile(out,'analysis.mat'),'analysis');

function names=timeComparisons(s,other,out,driver,aeros,charges,labels)
common={'ClA_m2','CdA_m2','DriverMass_kg','BatteryDischargeLimit_kW', ...
    'BatteryChargeLimit_kW','NetBatteryEnergy_kWh','RequiredUsableEnergy_kWh'};
t=s(:,common); t.Discipline=repmat("endurance",height(s),1);
t.Time_s=s.Endurance_s; t.Points=s.Endurance_points; t.Efficiency_points=s.Efficiency_points;
u=other(:,common); u.Discipline=other.Discipline; u.Time_s=other.Time_s;
u.Points=other.Points; u.Efficiency_points=nan(height(other),1);
t=[t;u]; t.ReferenceTime_s=nan(height(t),1);
ids=["acceleration","skidpad","autocross","endurance"];
for id=ids
    for mass=unique(t.DriverMass_kg).'
        mask=t.Discipline==id & t.DriverMass_kg==mass;
        ref=t(mask & t.ClA_m2==6.6 & t.BatteryDischargeLimit_kW==80 ...
            & t.BatteryChargeLimit_kW==Inf,:);
        assert(height(ref)==1); t.ReferenceTime_s(mask)=ref.Time_s;
    end
end
t.DeltaTime_s=t.Time_s-t.ReferenceTime_s;
t.DeltaTime_pct=100*t.DeltaTime_s./t.ReferenceTime_s;
t=sortrows(t,{'Discipline','DriverMass_kg','ClA_m2','BatteryDischargeLimit_kW','BatteryChargeLimit_kW'});
assert(height(t)==height(s)+height(other) && all(isfinite(t.DeltaTime_s)));
writetable(t,fullfile(out,'time_energy_comparison.csv'));
names={};
selectedDriver=driver;
for driver=unique(t.DriverMass_kg).'
for metric=["Time_s","DeltaTime_s","DeltaTime_pct"]
    f=figure('Visible','off','Position',[50,50,1450,950]);
    c=onCleanup(@()close(f)); layout=tiledlayout(2,2,'TileSpacing','compact');
    for id=ids
        ax=nexttile(layout); hold(ax,'on');
        g=t(t.Discipline==id & t.DriverMass_kg==driver & t.BatteryChargeLimit_kW==Inf,:);
        powers=unique(g.BatteryDischargeLimit_kW).';
        for power=powers
            h=sortrows(g(g.BatteryDischargeLimit_kW==power,:),'ClA_m2');
            plot(ax,h.ClA_m2,h.(metric),'-o','LineWidth',1.4, ...
                'DisplayName',sprintf('%g kW DC Entladung',power));
        end
        title(ax,id); xlabel(ax,'ClA [m²]'); grid(ax,'on');
        if metric=="Time_s", ylabel(ax,'Zeit [s]');
        elseif metric=="DeltaTime_s", ylabel(ax,'Zeitänderung zur Referenz [s]'); yline(ax,0,':','HandleVisibility','off');
        else, ylabel(ax,'Zeitänderung zur Referenz [%]'); yline(ax,0,':','HandleVisibility','off'); end
        legend(ax,'Location','best');
    end
    title(layout,sprintf('Zeitvergleich | Fahrer %g kg | Referenz ClA 6.6 / 80 kW | vorläufige Konzeptwerte',driver));
    name=char("discipline_"+lower(metric)+"_comparison.png");
    if driver==selectedDriver
        exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name;
    end
    name=strrep(name,'.png',sprintf('_driver_%gkg.png',driver));
    exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name;
    clear c
end
f=figure('Visible','off','Position',[50,50,1550,950]);
c=onCleanup(@()close(f)); layout=tiledlayout(2,3,'TileSpacing','compact');
for ci=1:numel(charges)
    ax=nexttile(layout); hold(ax,'on');
    for clA=aeros
        g=sortrows(s(s.DriverMass_kg==driver & s.ClA_m2==clA ...
            & s.BatteryChargeLimit_kW==charges(ci),:),'BatteryDischargeLimit_kW');
        plot(ax,g.Endurance_s,g.RequiredUsableEnergy_kWh,'-o','LineWidth',1.4, ...
            'DisplayName',sprintf('ClA %g',clA));
        for j=[1,height(g)]
            text(ax,g.Endurance_s(j),g.RequiredUsableEnergy_kWh(j), ...
                sprintf(' %g kW',g.BatteryDischargeLimit_kW(j)),'FontSize',8);
        end
    end
    title(ax,"Ladelimit "+labels(ci)); xlabel(ax,'Endurance-Zeit [s]');
    ylabel(ax,'Benötigte nutzbare Akkuenergie [kWh]'); grid(ax,'on');
end
legend(ax,'Location','best');
title(layout,sprintf('Energie-Zeit-Kompromiss | Fahrer %g kg | Entladung 50–80 kW | vorläufig',driver));
name='energy_vs_endurance_time.png';
if driver==selectedDriver
    exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name;
end
name=sprintf('energy_vs_endurance_time_driver_%gkg.png',driver);
exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name; clear c
end
for metric=["Time_s","RequiredUsableEnergy_kWh"]
    f=figure('Visible','off','Position',[50,50,1450,950]);
    c=onCleanup(@()close(f)); layout=tiledlayout(2,2,'TileSpacing','compact');
    for id=ids
        ax=nexttile(layout); hold(ax,'on');
        for clA=aeros
            g=sortrows(t(t.Discipline==id & t.ClA_m2==clA ...
                & t.BatteryDischargeLimit_kW==80 & t.BatteryChargeLimit_kW==Inf,:),'DriverMass_kg');
            assert(height(g)==numel(unique(t.DriverMass_kg)));
            plot(ax,g.DriverMass_kg,g.(metric),'-o','LineWidth',1.4,'DisplayName',sprintf('ClA %g',clA));
        end
        title(ax,id); xlabel(ax,'Fahrergewicht [kg]'); grid(ax,'on');
        if metric=="Time_s", ylabel(ax,'Zeit [s]'); else, ylabel(ax,'Benötigte nutzbare Akkuenergie [kWh]'); end
        legend(ax,'Location','best');
    end
    title(layout,'Direkter Fahrergewichtsvergleich | 80 kW Entladung / unbegrenzt Ladung | vorläufig');
    if metric=="Time_s", name='driver_weight_times.png'; else, name='driver_weight_energy.png'; end
    exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name; clear c
end
end
end

function names=energyBreakdownPlots(s,out)
e=s(:,{'ClA_m2','CdA_m2','DriverMass_kg','BatteryDischargeLimit_kW', ...
    'BatteryChargeLimit_kW','Endurance_s','DischargeEnergy_kWh','RecoveredEnergy_kWh', ...
    'NetBatteryEnergy_kWh','ScoringEnergy_kWh','RequiredUsableEnergy_kWh'});
e.RecoveredFractionOfDischarge_pct=100*e.RecoveredEnergy_kWh./e.DischargeEnergy_kWh;
assert(height(e)==height(s) && all(e.RecoveredEnergy_kWh>=0));
assert(all(abs(e.NetBatteryEnergy_kWh-e.DischargeEnergy_kWh+e.RecoveredEnergy_kWh)<1e-8));
writetable(e,fullfile(out,'energy_breakdown.csv'));
names={}; aeros=unique(e.ClA_m2).'; powers=unique(e.BatteryDischargeLimit_kW).';
charges=unique(e.BatteryChargeLimit_kW).'; labels=string(charges); labels(isinf(charges))="∞";
metrics=["DischargeEnergy_kWh","RecoveredEnergy_kWh","NetBatteryEnergy_kWh"];
captions=["Entnommene Akkuenergie [kWh]","Zurückgewonnene Akkuenergie [kWh]","Netto-Akkuenergie [kWh]"];
stems=["energy_discharge","energy_recovered","energy_net"];
for mass=unique(e.DriverMass_kg).'
    for mi=1:3
        f=figure('Visible','off','Position',[50,50,1500,950]); c=onCleanup(@()close(f));
        layout=tiledlayout(2,3,'TileSpacing','compact');
        for clA=aeros
            ax=nexttile(layout); hold(ax,'on');
            for power=powers
                g=sortrows(e(e.DriverMass_kg==mass & e.ClA_m2==clA ...
                    & e.BatteryDischargeLimit_kW==power,:),'BatteryChargeLimit_kW');
                plot(ax,1:numel(charges),g.(metrics(mi)),'-o','LineWidth',1.3, ...
                    'DisplayName',sprintf('%g kW Entladung',power));
            end
            ax.XTick=1:numel(charges); ax.XTickLabel=labels;
            title(ax,sprintf('ClA %g / CdA %g',clA,g.CdA_m2(1))); grid(ax,'on');
            xlabel(ax,'Akku-Ladelimit [kW]; ∞ = unbegrenzt'); ylabel(ax,captions(mi));
        end
        legend(ax,'Location','best');
        title(layout,sprintf('Akkuenergie ohne Punktedeckel | Fahrer %g kg | vorläufig',mass));
        name=char(stems(mi)+sprintf('_driver_%gkg.png',mass));
        exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
    end
    f=figure('Visible','off','Position',[50,50,1500,950]); c=onCleanup(@()close(f));
    layout=tiledlayout(2,3,'TileSpacing','compact');
    for clA=aeros
        ax=nexttile(layout); hold(ax,'on');
        for ci=1:numel(charges)
            g=sortrows(e(e.DriverMass_kg==mass & e.ClA_m2==clA ...
                & e.BatteryChargeLimit_kW==charges(ci),:),'BatteryDischargeLimit_kW');
            plot(ax,g.BatteryDischargeLimit_kW,g.RecoveredEnergy_kWh,'-o','LineWidth',1.3, ...
                'DisplayName',"Ladung "+labels(ci)+" kW");
        end
        title(ax,sprintf('ClA %g',clA)); xlabel(ax,'Akku-Entladelimit [kW]');
        ylabel(ax,'Zurückgewonnene Akkuenergie [kWh]'); grid(ax,'on');
    end
    legend(ax,'Location','best');
    title(layout,sprintf('Rekuenergie über Antriebsleistung | Fahrer %g kg | vorläufig',mass));
    name=sprintf('energy_recovered_vs_discharge_driver_%gkg.png',mass);
    exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
    f=figure('Visible','off','Position',[50,50,1500,950]); c=onCleanup(@()close(f));
    layout=tiledlayout(2,3,'TileSpacing','compact');
    for ci=1:numel(charges)
        ax=nexttile(layout); hold(ax,'on');
        for clA=aeros
            g=sortrows(e(e.DriverMass_kg==mass & e.ClA_m2==clA ...
                & e.BatteryChargeLimit_kW==charges(ci),:),'BatteryDischargeLimit_kW');
            plot(ax,g.Endurance_s,g.RecoveredEnergy_kWh,'-o','LineWidth',1.3, ...
                'DisplayName',sprintf('ClA %g',clA));
        end
        title(ax,"Ladelimit "+labels(ci)+" kW"); xlabel(ax,'Endurance-Zeit [s]');
        ylabel(ax,'Zurückgewonnene Akkuenergie [kWh]'); grid(ax,'on');
    end
    legend(ax,'Location','best');
    title(layout,sprintf('Rekuenergie gegen Fahrzeit | Fahrer %g kg | Entladung 50–80 kW | vorläufig',mass));
    name=sprintf('energy_recovered_vs_time_driver_%gkg.png',mass);
    exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
end
end

function writeEnergySection(fid,s,names,driver)
fprintf(fid,'\n## Akkuenergie getrennt ohne Punktedeckel\n\nAlle 280 Endurance-Fälle sind in energy_breakdown.csv enthalten. Zurückgewonnene Energie ist die tatsächliche elektrische Rekuenergie am Akku-Anschluss, ohne den 0.9-Wertungsfaktor. Nettoenergie = Entnahme minus Reku; Wertungsenergie = Entnahme minus 0.9 mal Reku. Benötigte nutzbare Kapazität ist dagegen die maximale kumulierte Entnahme und kann größer als die Nettoenergie am Ziel sein. Es gibt keine Punktebegrenzung in den kWh-Diagrammen.\n\n');
fprintf(fid,'Je Fahrergewicht werden Entnahme, Reku und Nettoenergie über Ladelimit getrennt gezeigt, mit allen Aero-Setups und Entladeleistungen. Zusätzlich Rekuenergie über Entladeleistung sowie über Endurance-Zeit. Die Daten enthalten AMK-Motorverluste bei 100 C und eine konservative 500-V-Spannungsgrenze; hohe Rekugewinne bleiben wegen der bekannten radweisen Grenzprobleme vorläufig.\n\n');
fprintf(fid,'Beispiel ClA=6.6, CdA=2, Fahrer %g kg, 80 kW Akku-Entladung:\n\n| Ladelimit [kW] | Entnahme [kWh] | Reku [kWh] | Netto [kWh] | Benötigte nutzbare Energie [kWh] |\n|---:|---:|---:|---:|---:|\n',driver);
g=sortrows(s(s.ClA_m2==6.6 & s.DriverMass_kg==driver & s.BatteryDischargeLimit_kW==80,:),'BatteryChargeLimit_kW');
for i=1:height(g)
    fprintf(fid,'| %g | %.4f | %.4f | %.4f | %.4f |\n',g.BatteryChargeLimit_kW(i), ...
        g.DischargeEnergy_kWh(i),g.RecoveredEnergy_kWh(i),g.NetBatteryEnergy_kWh(i),g.RequiredUsableEnergy_kWh(i));
end
fprintf(fid,'\n');
for name=names, fprintf(fid,'- [%s](%s)\n',name{1},name{1}); end
end

function names=regenComparisons(s,other,out,aeros,charges,labels)
r=s; r.EnergySaving_kWh=zeros(height(r),1); r.EnergySaving_pct=zeros(height(r),1);
groups=unique(r(:,{'ClA_m2','DriverMass_kg','BatteryDischargeLimit_kW'}),'rows');
for i=1:height(groups)
    mask=r.ClA_m2==groups.ClA_m2(i) & r.DriverMass_kg==groups.DriverMass_kg(i) ...
        & r.BatteryDischargeLimit_kW==groups.BatteryDischargeLimit_kW(i);
    g=r(mask,:); assert(height(g)==numel(charges));
    base=g.RequiredUsableEnergy_kWh(g.BatteryChargeLimit_kW==0);
    assert(isscalar(base) && max(g.Endurance_s)-min(g.Endurance_s)<1e-8);
    r.EnergySaving_kWh(mask)=base-r.RequiredUsableEnergy_kWh(mask);
    r.EnergySaving_pct(mask)=100*r.EnergySaving_kWh(mask)/base;
end
assert(all(r.EnergySaving_kWh>=-1e-8));
writetable(r,fullfile(out,'regen_comparison.csv'));
names={}; powers=unique(r.BatteryDischargeLimit_kW).';
metrics=["RequiredUsableEnergy_kWh","EnergySaving_kWh","Efficiency_points"];
captions=["Benötigte nutzbare Akkuenergie [kWh]","Energieeinsparung gegenüber 0 kW Reku [kWh]","Efficiency [Punkte]"];
stems=["regen_capacity","regen_energy_saving","regen_efficiency"];
chargeNumbers=string(charges); chargeNumbers(isinf(charges))="∞";
for mass=unique(r.DriverMass_kg).'
    for mi=1:numel(metrics)
        f=figure('Visible','off','Position',[50,50,1500,950]); c=onCleanup(@()close(f));
        layout=tiledlayout(2,3,'TileSpacing','compact');
        for clA=aeros
            ax=nexttile(layout); hold(ax,'on');
            for power=powers
                g=sortrows(r(r.DriverMass_kg==mass & r.ClA_m2==clA ...
                    & r.BatteryDischargeLimit_kW==power,:),'BatteryChargeLimit_kW');
                plot(ax,1:numel(charges),g.(metrics(mi)),'-o','LineWidth',1.3, ...
                    'DisplayName',sprintf('%g kW Entladung',power));
            end
            ax.XTick=1:numel(charges); ax.XTickLabel=chargeNumbers;
            xlabel(ax,'Akku-Ladelimit [kW]; ∞ = unbegrenzt'); ylabel(ax,captions(mi));
            title(ax,sprintf('ClA %g / CdA %g',clA,g.CdA_m2(1))); grid(ax,'on');
            if metrics(mi)=="Efficiency_points", ylim(ax,[0,75]); end
        end
        legend(ax,'Location','best');
        title(layout,sprintf('Reku-Vergleich | Fahrer %g kg | vorläufige Konzeptwerte',mass));
        name=char(stems(mi)+sprintf('_driver_%gkg.png',mass));
        exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
    end
    for metric=["Efficiency_points","EnduranceEfficiency_points"]
        f=figure('Visible','off','Position',[50,50,1500,950]); c=onCleanup(@()close(f));
        layout=tiledlayout(2,3,'TileSpacing','compact');
        for clA=aeros
            ax=nexttile(layout); hold(ax,'on');
            for ci=1:numel(charges)
                g=sortrows(r(r.DriverMass_kg==mass & r.ClA_m2==clA ...
                    & r.BatteryChargeLimit_kW==charges(ci),:),'BatteryDischargeLimit_kW');
                plot(ax,g.BatteryDischargeLimit_kW,g.(metric),'-o','LineWidth',1.3, ...
                    'DisplayName',"Ladung "+labels(ci));
            end
            xlabel(ax,'Akku-Entladelimit [kW]'); grid(ax,'on'); title(ax,sprintf('ClA %g',clA));
            if metric=="Efficiency_points", ylabel(ax,'Efficiency [Punkte]'); ylim(ax,[0,75]);
            else, ylabel(ax,'Endurance + Efficiency [Punkte]'); end
        end
        legend(ax,'Location','best'); title(layout,sprintf('Reku und Entladung | Fahrer %g kg | vorläufig',mass));
        if metric=="Efficiency_points", stem='regen_efficiency_vs_discharge'; else, stem='regen_combined_vs_discharge'; end
        name=sprintf('%s_driver_%gkg.png',stem,mass);
        exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
    end
    f=figure('Visible','off','Position',[50,50,1500,550]); c=onCleanup(@()close(f));
    layout=tiledlayout(1,3,'TileSpacing','compact');
    for id=["acceleration","skidpad","autocross"]
        ax=nexttile(layout); hold(ax,'on');
        for clA=aeros
            g=sortrows(other(other.DriverMass_kg==mass & other.ClA_m2==clA ...
                & other.Discipline==id,:),'BatteryChargeLimit_kW');
            assert(height(g)==numel(charges) && max(g.Time_s)-min(g.Time_s)<1e-8);
            plot(ax,1:numel(charges),g.RequiredUsableEnergy_kWh,'-o','LineWidth',1.3, ...
                'DisplayName',sprintf('ClA %g',clA));
        end
        ax.XTick=1:numel(charges); ax.XTickLabel=chargeNumbers;
        title(ax,id); xlabel(ax,'Akku-Ladelimit [kW]; ∞ = unbegrenzt');
        ylabel(ax,'Benötigte nutzbare Akkuenergie [kWh]'); grid(ax,'on');
    end
    legend(ax,'Location','best'); title(layout,sprintf('Andere Disziplinen | Fahrer %g kg | Entladung 80 kW | vorläufig',mass));
    name=sprintf('regen_other_energy_driver_%gkg.png',mass);
    exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
end
end

function names=additionalPlots(s,out,driver,charges,labels,aeros)
names={}; powers=unique(s.BatteryDischargeLimit_kW).';
metrics={'Endurance_points','Efficiency_points','EnduranceEfficiency_points'};
captions={'Endurance [Punkte]','Efficiency [Punkte]','Endurance + Efficiency [Punkte]'};
limits={[0,250],[0,75],[0,325]};
for m=1:3
    f=figure('Visible','off','Position',[50,50,1700,1000]);
    cleanup=onCleanup(@()close(f)); layout=tiledlayout(2,3,'TileSpacing','compact');
    for clA=aeros
        matrix=nan(numel(charges),numel(powers));
        for ci=1:numel(charges)
            for pi=1:numel(powers)
                row=s(s.ClA_m2==clA & s.DriverMass_kg==driver ...
                    & s.BatteryChargeLimit_kW==charges(ci) ...
                    & s.BatteryDischargeLimit_kW==powers(pi),:);
                assert(height(row)==1); matrix(ci,pi)=row.(metrics{m});
            end
        end
        ax=nexttile(layout); imagesc(ax,matrix); ax.YDir='normal';
        ax.XTick=1:numel(powers); ax.XTickLabel=string(powers);
        ax.YTick=1:numel(charges); ax.YTickLabel=labels;
        clim(ax,limits{m}); colorbar(ax); colormap(ax,parula(256));
        xlabel(ax,'Entladelimit am Akku [kW]'); ylabel(ax,'Ladelimit am Akku');
        title(ax,sprintf('ClA = %g m²',clA));
        for ci=1:numel(charges)
            for pi=1:numel(powers)
                color='k';
                if matrix(ci,pi)<limits{m}(2)*0.5, color='w'; end
                text(ax,pi,ci,sprintf('%.1f',matrix(ci,pi)), ...
                    'HorizontalAlignment','center','Color',color,'FontSize',9);
            end
        end
    end
    title(layout,sprintf('%s | Fahrer %g kg | AMK-Motorkennfeld 100 C / 500 V',captions{m},driver));
    name=sprintf('%s_heatmaps.png',metrics{m});
    exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name;
    clear cleanup
end
f=figure('Visible','off','Position',[50,50,1500,540]);
cleanup=onCleanup(@()close(f)); layout=tiledlayout(1,3,'TileSpacing','compact');
for m=1:3
    ax=nexttile(layout); hold(ax,'on');
    for power=powers
        g=s(s.DriverMass_kg==driver & s.BatteryChargeLimit_kW==Inf ...
            & s.BatteryDischargeLimit_kW==power,:); g=sortrows(g,'ClA_m2');
        plot(ax,g.ClA_m2,g.(metrics{m}),'-o','LineWidth',1.3);
    end
    xlabel(ax,'ClA [m²]'); ylabel(ax,captions{m}); grid(ax,'on');
end
legend(ax,cellstr(string(compose('%g kW Entladung',powers))),'Location','best');
title(layout,sprintf('Aero-Vergleich | Fahrer %g kg | Ladelimit unbegrenzt | CdA 2, ohne Abtrieb 0.8',driver));
name='aero_points_by_discharge.png'; exportgraphics(f,fullfile(out,name),'Resolution',180);
names{end+1}=name; clear cleanup
f=figure('Visible','off','Position',[50,50,1500,540]);
cleanup=onCleanup(@()close(f)); layout=tiledlayout(1,3,'TileSpacing','compact');
driverFields={'Endurance_s','EnduranceEfficiency_points','RequiredUsableEnergy_kWh'};
driverLabels={'Endurance-Zeit [s]','Endurance + Efficiency [Punkte]','Benötigte Akkuenergie [kWh]'};
for m=1:3
    ax=nexttile(layout); hold(ax,'on');
    for clA=aeros
        g=s(s.ClA_m2==clA & s.BatteryDischargeLimit_kW==80 ...
            & s.BatteryChargeLimit_kW==Inf,:); g=sortrows(g,'DriverMass_kg');
        plot(ax,g.DriverMass_kg,g.(driverFields{m}),'-o','LineWidth',1.3);
    end
    xlabel(ax,'Fahrergewicht [kg]'); ylabel(ax,driverLabels{m}); grid(ax,'on');
end
legend(ax,cellstr(string(compose('ClA %g m²',aeros))),'Location','best');
title(layout,'Fahrergewicht und Aero | Entladung 80 kW | Ladung unbegrenzt');
name='driver_mass_endurance.png'; exportgraphics(f,fullfile(out,name),'Resolution',180);
names{end+1}=name; clear cleanup
end

function names=otherPlots(s,out,driver,charges,labels,aeros)
names={}; ids=["acceleration","skidpad","autocross"];
titles={'Acceleration','Skidpad','Autocross'};
masses=unique(s.DriverMass_kg).';
for metric=["Time_s","Points"]
    f=figure('Visible','off','Position',[50,50,1500,540]);
    cleanup=onCleanup(@()close(f)); layout=tiledlayout(1,3,'TileSpacing','compact');
    for ei=1:3
        ax=nexttile(layout); hold(ax,'on');
        for mass=masses
            g=s(s.Discipline==ids(ei) & s.DriverMass_kg==mass ...
                & s.BatteryChargeLimit_kW==Inf,:); g=sortrows(g,'ClA_m2');
            plot(ax,g.ClA_m2,g.(metric),'-o','LineWidth',1.3);
        end
        title(ax,titles{ei}); xlabel(ax,'ClA [m²]'); grid(ax,'on');
        if metric=="Time_s", ylabel(ax,'Zeit [s]'); else, ylabel(ax,'Punkte'); end
    end
    legend(ax,cellstr(string(compose('%g kg Fahrer',masses))),'Location','best');
    title(layout,'Andere Disziplinen | Entladelimit 80 kW | CdA 2, ohne Abtrieb 0.8');
    name=char("other_disciplines_"+lower(metric)+".png");
    exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name;
    clear cleanup
end
f=figure('Visible','off','Position',[50,50,1500,540]);
cleanup=onCleanup(@()close(f)); layout=tiledlayout(1,3,'TileSpacing','compact');
for ei=1:3
    ax=nexttile(layout); hold(ax,'on');
    for clA=aeros
        y=zeros(size(charges));
        for ci=1:numel(charges)
            g=s(s.Discipline==ids(ei) & s.DriverMass_kg==driver ...
                & s.ClA_m2==clA & s.BatteryChargeLimit_kW==charges(ci),:);
            assert(height(g)==1); y(ci)=g.RequiredUsableEnergy_kWh;
        end
        plot(ax,1:numel(charges),y,'-o','LineWidth',1.3);
    end
    chargeNumbers=string(charges); chargeNumbers(isinf(charges))="∞";
    ax.XTick=1:numel(charges); ax.XTickLabel=chargeNumbers; ax.XTickLabelRotation=0;
    title(ax,titles{ei}); xlabel(ax,'Akku-Ladelimit [kW]; ∞ = unbegrenzt'); ylabel(ax,'Benötigte Akkuenergie [kWh]'); grid(ax,'on');
end
legend(ax,cellstr(string(compose('ClA %g m²',aeros))),'Location','best');
title(layout,sprintf('Reku und Energie der anderen Disziplinen | Fahrer %g kg | Entladung 80 kW',driver));
name='other_disciplines_energy_vs_regen.png';
exportgraphics(f,fullfile(out,name),'Resolution',180); names{end+1}=name;
clear cleanup
end

function [names,optima]=uncappedConcept(s,out,fid)
% Remove only the upper point ceiling; preserve slow-time/poor-energy cutoff.
source=load(fullfile(out,'report.mat'),'report'); o=source.report.options;
bestTime=o.BestEnduranceTime_s; minimumFactor=o.MinimumEfficiencyFactor_s2kWh;
e=s;
e.Endurance_uncapped_points=25+225*((1.5*bestTime-min(e.Endurance_s,1.5*bestTime))/(0.5*bestTime)).^2;
e.EfficiencyFactor_s2kWh=e.Endurance_s.^2.*e.ScoringEnergy_kWh;
e.Efficiency_uncapped_points=75*((2*minimumFactor-min(e.EfficiencyFactor_s2kWh,2*minimumFactor))/minimumFactor).^2;
e.Total_uncapped_points=e.Endurance_uncapped_points+e.Efficiency_uncapped_points;
assert(all(abs(min(250,e.Endurance_uncapped_points)-e.Endurance_points)<1e-8));
assert(all(abs(min(75,e.Efficiency_uncapped_points)-e.Efficiency_points)<1e-8));
writetable(e,fullfile(out,'concept_points_uncapped.csv'));
other=readtable(fullfile(out,'other_events_scored.csv'),'TextType','string');
other.UncappedConcept_points=nan(height(other),1);
ids=["acceleration","skidpad","autocross"]; times=[3.369,4.678,72.925];
cutoffs=[1.7,1.35,1.4]; maxima=[50,50,100]; floors=[2.5,2.5,10];
for i=1:3
    mask=other.Discipline==ids(i); t=other.Time_s(mask); maximumTime=cutoffs(i)*times(i);
    other.UncappedConcept_points(mask)=floors(i)+(maxima(i)-floors(i))* ...
        ((maximumTime-min(t,maximumTime))/(maximumTime-times(i))).^2;
    assert(all(abs(min(maxima(i),other.UncappedConcept_points(mask))-other.Points(mask))<1e-8));
end
writetable(other,fullfile(out,'other_events_uncapped.csv'));
groups=unique(e(:,{'ClA_m2','DriverMass_kg','BatteryChargeLimit_kW'}),'rows'); rows=struct([]);
for i=1:height(groups)
    g=e(e.ClA_m2==groups.ClA_m2(i) & e.DriverMass_kg==groups.DriverMass_kg(i) ...
        & e.BatteryChargeLimit_kW==groups.BatteryChargeLimit_kW(i),:);
    [~,official]=max(g.EnduranceEfficiency_points); [~,raw]=max(g.Total_uncapped_points);
    b=g(raw,:); a=g(official,:);
    r=struct('ClA_m2',b.ClA_m2,'DriverMass_kg',b.DriverMass_kg, ...
        'BatteryChargeLimit_kW',b.BatteryChargeLimit_kW, ...
        'OfficialOptimumDischarge_kW',a.BatteryDischargeLimit_kW, ...
        'UncappedOptimumDischarge_kW',b.BatteryDischargeLimit_kW, ...
        'Endurance_uncapped_points',b.Endurance_uncapped_points, ...
        'Efficiency_uncapped_points',b.Efficiency_uncapped_points, ...
        'Total_uncapped_points',b.Total_uncapped_points, ...
        'OfficialPointsAtUncappedOptimum',b.EnduranceEfficiency_points, ...
        'OfficialPointLoss',a.EnduranceEfficiency_points-b.EnduranceEfficiency_points, ...
        'Endurance_s',b.Endurance_s,'RequiredUsableEnergy_kWh',b.RequiredUsableEnergy_kWh);
    if isempty(rows), rows=r; else, rows(end+1)=r; end %#ok<AGROW>
end
optima=struct2table(rows); assert(height(optima)==40);
writetable(optima,fullfile(out,'optima_uncapped.csv'));
names={}; aeros=unique(e.ClA_m2).'; charges=unique(e.BatteryChargeLimit_kW).';
labels=string(charges); labels(isinf(charges))="∞";
rawFields=["Endurance_uncapped_points","Efficiency_uncapped_points","Total_uncapped_points"];
officialFields=["Endurance_points","Efficiency_points","EnduranceEfficiency_points"];
captions=["Endurance","Efficiency","Endurance + Efficiency"];
for mass=unique(e.DriverMass_kg).'
    for mi=1:3
        f=figure('Visible','off','Position',[50,50,1400,900]); c=onCleanup(@()close(f));
        layout=tiledlayout(2,2,'TileSpacing','compact'); colors=lines(numel(charges));
        for clA=aeros
            ax=nexttile(layout); hold(ax,'on');
            for ci=1:numel(charges)
                g=sortrows(e(e.ClA_m2==clA & e.DriverMass_kg==mass ...
                    & e.BatteryChargeLimit_kW==charges(ci),:),'BatteryDischargeLimit_kW');
                plot(ax,g.BatteryDischargeLimit_kW,g.(rawFields(mi)),'-o','Color',colors(ci,:), ...
                    'DisplayName',"Ladung "+labels(ci)+" kW; Konzept");
                plot(ax,g.BatteryDischargeLimit_kW,g.(officialFields(mi)),'--','Color',colors(ci,:), ...
                    'DisplayName',"Ladung "+labels(ci)+" kW; offiziell");
            end
            title(ax,sprintf('ClA %g',clA)); xlabel(ax,'Akku-Entladelimit [kW]');
            ylabel(ax,captions(mi)+" [Punkte / Konzeptpunkte]"); grid(ax,'on');
        end
        legend(ax,'Location','best','FontSize',8);
        title(layout,sprintf('Ungedeckelt vs. offiziell | %g kg | keine realen Zusatzpunkte | vorläufig',mass));
        name=char("uncapped_"+lower(replace(captions(mi)," + ","_"))+sprintf('_driver_%gkg.png',mass));
        exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
    end
    f=figure('Visible','off','Position',[50,50,1500,550]); c=onCleanup(@()close(f));
    layout=tiledlayout(1,3,'TileSpacing','compact');
    for i=1:3
        ax=nexttile(layout); hold(ax,'on');
        g=sortrows(other(other.DriverMass_kg==mass & other.Discipline==ids(i) ...
            & isinf(other.BatteryChargeLimit_kW),:),'ClA_m2');
        plot(ax,g.ClA_m2,g.UncappedConcept_points,'-o','DisplayName','Konzept, ungedeckelt');
        plot(ax,g.ClA_m2,g.Points,'--o','DisplayName','Offiziell, gedeckelt');
        title(ax,ids(i)); xlabel(ax,'ClA [m²]'); ylabel(ax,'Punkte / Konzeptpunkte');
        grid(ax,'on'); legend(ax,'Location','best');
    end
    title(layout,sprintf('Andere Disziplinen | Fahrer %g kg | 80 kW Entladung | vorläufig',mass));
    name=sprintf('uncapped_other_events_driver_%gkg.png',mass);
    exportgraphics(f,fullfile(out,name),'Resolution',160); names{end+1}=name; clear c
end
fprintf(fid,'\n## Ungedeckelte Konzeptpunkte\n\nNur die obere Punktebegrenzung wird entfernt. Die bisherigen Zeit-/Energie-Cutoffs und unteren Punktewerte bleiben erhalten. Endurance: 25 + 225*((1.5*Tbest-min(T,1.5*Tbest))/(0.5*Tbest))^2. Efficiency: 75*((2*EFmin-min(EF,2*EFmin))/EFmin)^2, EF=T^2*(Entnahme-0.9*Reku). Tbest=%.5f s, EFmin=%.8f s² kWh.\n\n',bestTime,minimumFactor);
fprintf(fid,'Werte oberhalb von 250 bzw. 75 sind rein rechnerische Konzeptpunkte, keine erreichbaren Wettbewerbspunkte. Sie extrapolieren die bestehende Bewertungsgewichtung außerhalb des offiziellen Bereichs. Deshalb bleiben Zeiten und Energie primäre physikalische Größen; diese Erweiterung kalibriert oder validiert das Fahrzeugmodell nicht. Die offiziellen Werte werden nicht verändert. Rückbegrenzung der Konzeptpunkte reproduziert sämtliche vorhandenen offiziellen Punkte numerisch.\n\n');
fprintf(fid,'| ClA | Fahrer kg | Ladung kW | Offizielles Optimum kW | Ungedeckeltes Optimum kW | Konzept-Endu | Konzept-Eff | Konzeptsumme | Verlust offizieller Punkte | Nutzbare kWh |\n|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
for i=1:height(optima)
    b=optima(i,:); fprintf(fid,'| %g | %g | %g | %g | %g | %.2f | %.2f | %.2f | %.2f | %.3f |\n', ...
        b.ClA_m2,b.DriverMass_kg,b.BatteryChargeLimit_kW,b.OfficialOptimumDischarge_kW, ...
        b.UncappedOptimumDischarge_kW,b.Endurance_uncapped_points,b.Efficiency_uncapped_points, ...
        b.Total_uncapped_points,b.OfficialPointLoss,b.RequiredUsableEnergy_kWh);
end
fprintf(fid,'\nTabellen: concept_points_uncapped.csv (280 Fälle), other_events_uncapped.csv (120 Fälle), optima_uncapped.csv (40 Gruppen). Gleiche Rastermaxima sind nicht zwingend eindeutige physikalische Optima.\n\n');
for name=names, fprintf(fid,'- [%s](%s)\n',name{1},name{1}); end
end

function comparison=compareV13(s,other,out,root,fid)
keys={'Discipline','ClA_m2','CdA_m2','DriverMass_kg','BatteryDischargeLimit_kW','BatteryChargeLimit_kW'};
a=readtable(fullfile(out,'time_energy_comparison.csv'),'TextType','string');
old=readtable(fullfile(root,'results','fsg2026PowertrainConceptV13', ...
    'constant_charge_sweep','time_energy_comparison.csv'),'TextType','string');
b=old(:,[keys,{'Time_s','RequiredUsableEnergy_kWh','Points','Efficiency_points'}]);
a=renamevars(a,{'Time_s','RequiredUsableEnergy_kWh','Points','Efficiency_points'}, ...
    {'TimeV14_s','EnergyV14_kWh','PointsV14','EfficiencyV14_points'});
b=renamevars(b,{'Time_s','RequiredUsableEnergy_kWh','Points','Efficiency_points'}, ...
    {'TimeV13_s','EnergyV13_kWh','PointsV13','EfficiencyV13_points'});
comparison=innerjoin(a,b,'Keys',keys);
assert(height(comparison)==height(s)+height(other),'Incomplete V13 comparison.');
comparison.DeltaTime_s=comparison.TimeV14_s-comparison.TimeV13_s;
comparison.DeltaEnergy_kWh=comparison.EnergyV14_kWh-comparison.EnergyV13_kWh;
comparison.DeltaPoints=comparison.PointsV14-comparison.PointsV13;
comparison.DeltaEfficiency_points=comparison.EfficiencyV14_points-comparison.EfficiencyV13_points;
writetable(comparison,fullfile(out,'comparison_V13.csv'));
fprintf(fid,'\n## Vergleich mit V13 bei identischen Rasterpunkten\n\n400 passend zugeordnete Fälle. Delta = V14 minus V13; negative Zeitänderung ist schneller. Energie ist die benötigte nutzbare Energie, ohne Reserve. Unbegrenzt wird mit unbegrenzt verglichen, nicht mit dem ausgeschlossenen 120-kW-Fall.\n\n');
fprintf(fid,'Referenz ClA=6.6, 80 kW Entladung, unbegrenzte Ladung:\n\n| Disziplin | Fahrer kg | V13 s | V14 s | Delta s | V13 kWh | V14 kWh |\n|---|---:|---:|---:|---:|---:|---:|\n');
g=comparison(comparison.ClA_m2==6.6 & comparison.BatteryDischargeLimit_kW==80 ...
    & isinf(comparison.BatteryChargeLimit_kW),:);
for i=1:height(g)
    fprintf(fid,'| %s | %g | %.3f | %.3f | %+.3f | %.4f | %.4f |\n', ...
        g.Discipline(i),g.DriverMass_kg(i),g.TimeV13_s(i),g.TimeV14_s(i), ...
        g.DeltaTime_s(i),g.EnergyV13_kWh(i),g.EnergyV14_kWh(i));
end
fprintf(fid,'\n## Punkteoptima mit AMK-Kennfeld\n\n| ClA | Fahrer kg | Ladelimit kW | Optimale Entladung kW | Endu s | Endu-Punkte | Eff-Punkte | Summe | Nutzbare kWh |\n|---:|---:|---:|---:|---:|---:|---:|---:|---:|\n');
best=readtable(fullfile(out,'optima.csv'));
for i=1:height(best)
    b=best(i,:);
    fprintf(fid,'| %g | %g | %g | %g | %.2f | %.2f | %.2f | %.2f | %.3f |\n', ...
        b.ClA_m2,b.DriverMass_kg,b.BatteryChargeLimit_kW,b.BatteryDischargeLimit_kW, ...
        b.Endurance_s,b.Endurance_points,b.Efficiency_points,b.EnduranceEfficiency_points,b.RequiredUsableEnergy_kWh);
end
end
