# Laptime Simulation – transparente V2

Die Simulation unterstützt Fahrzeug-Designentscheidungen mit drei öffentlichen
Modellfamilien: `pointMass`, `singleTrack` und `dualTrack`. Physikalische Features
und numerische Strategie werden ausdrücklich gewählt. Die historischen Level
bleiben als interne Rechenkerne und Kompatibilitätsschnittstellen verfügbar.

## Start

Öffne `LaptimeSimulation_nextGen.prj` in MATLAB R2024b oder neuer. Alternativ:

```matlab
run('project/startup.m');
results = runTests;
result = runDemoV2;
```

`runDemoV2.m` enthält einen vollständigen Einzelfall mit Fahrzeug, Strecke,
Modell, Feature-Flags und Solvergrenzen. Vor dem Lauf zeigt V2 die effektive
Konfiguration; dieselbe Information wird im Ergebnis gespeichert. Einzelne
Felder als `used`/`ignored` zu klassifizieren bleibt eine Modellbeschreibung,
kein experimenteller Nachweis ihres Einflusses.

## Haupteinstiege

| Datei | Zweck |
|---|---|
| `runTests.m` | projektlokale Tests, einschließlich Fehlerregressionen |
| `runDemoV2.m` | allgemeiner Simulationsfall |
| `runModelComparisonV2.m` | Vergleich der drei Modellfamilien |
| `runSensitivityStudyV2.m` | explizite Parameter- und Eventstudie |
| `runEnergyReportV2.m` | Energiebericht einer Baseline |
| `runFsg2026SensitivityV9.m` | FSG-Sensitivitäten und disziplinspezifische Fahrermassen |
| `runFsg2026Top10TargetsV8.m` | FSG-Konzeptziele mit korrigierter Reifenhülle |
| `runFsg2026PowertrainConceptV14.m` | aktuelle Antriebs-/Aero-/Reku-Studie mit AMK-Kennfeld |

Spezielle Antriebs-, Reifen-, Fahrwerks- und Regelungsvergleiche liegen unter
`tools/comparisons`. CFD-Zustände, Reifenvergleich, V14-Grenzaudit und historische
R20-Freigabe liegen unter `tools/diagnostics`. Der externe Tau-Vergleich bleibt unter `tools/legacy` erhalten. Die alten
V13-Rechenpfade wurden durch auswählbare Module im gemeinsamen Kern ersetzt. `project/startup.m` und `runTests` fügen
nur diese aktiven Verzeichnisse hinzu; verwende kein `genpath` über das gesamte
Projekt, da Sicherungen und Archive alte Funktionen enthalten können.

## Aktuelle FSG-Antriebsstudie

```matlab
report = runFsg2026PowertrainConceptV14(PrepareOnly=true); % zunächst Fallraster
report = runFsg2026PowertrainConceptV14;                  % vollständige Studie
analysis = analyzeFsg2026PowertrainConceptV14;
audit = auditFsg2026PowertrainLimitsV14;
```

Allgemeine Simulation und FSG-Studie nutzen dieselben Funktionen:
`lts.powertrain.configureVehicle`, `lts.solver.solveLapWithEnergy`,
`lts.energy.analyzeLap` und `lts.energy.profileTable`. Motorvarianten werden
über `MotorModel` gewählt, ohne Kopien der Rechenfunktionen:

```matlab
report = runFsg2026PowertrainConceptV14(MotorModel="constant", ...
    OutputTag="modular_constant", PrepareOnly=true);
vehicle = lts.powertrain.configureVehicle(FAUmaxTauV2(), ...
    MotorModel="efficiencyMap", MotorEfficiencyMap=measuredMap);
```

Standard ist `lossMap` mit dem projektlokalen AMK-Kennfeld. `efficiencyMap`
benötigt explizite Messdaten. Versionsnamen von Studien und historischen
Ergebnisordnern bezeichnen weiterhin deren Herkunft.

Die vollständige Studie ist umfangreich. Die aktuelle Auswertung erwartet das
Standardraster mit 280 Endurance- und 120 weiteren Eventfällen. Neue V14-Läufe verwenden standardmäßig
`results/fsg2026PowertrainConceptV14/modular_amk100C_500V` mit Checkpoint-Version `modular-energy-v1`. Alte Ergebnisse werden dadurch nicht überschrieben. Für
weitere Konfigurationen einen eigenen `OutputTag` wählen. Der V13-Vergleich
ist optional: `analyzeFsg2026PowertrainConceptV14(CompareV13=true)` benötigt die
historische V13-Ergebnis-CSV. Der neue Grenzaudit schreibt einen eigenen Ordner
und kennzeichnet Verletzungen in gespeicherten Profilen; er repariert sie nicht
und ersetzt keine vollständige nichtlineare Schlupfvalidierung.

V8/V9 verwenden ebenfalls neue Cache-Verzeichnisse mit dem Zusatz
`modular_energy_v1`. `plotFsg2026SensitivityV9` liest den neuesten modularen
Bericht; alte Berichte können über `SourceFile` ausdrücklich ausgewählt werden.

Die projektlokalen Eingaben sind unter `data/tracks`, `data/tires`,
`data/powertrain` und `data/reference` abgelegt. Das AMK-Kennfeld und die
numerische R20-Freigabe benötigen keinen lokalen SVN-Pfad. Sie sind feste
Entwicklungs-/Referenzdaten; Herkunft und Grenzen stehen in `data/README.md`.
Die historische V5-Quelle zur erneuten R20-Freigabe ist nicht enthalten;
`runR20SlipMapReleaseValidationV6(SourceFile=...)` bleibt ein optionales Werkzeug.

## Tests und Grenzen

`runTests` verlangt Erfolg für alle ausgewählten Tests. Drei Regressionen gegen
externe `Tau.mat`-Daten sind ausdrücklich optional:

```matlab
results = runTests(IncludeLegacyRegression=true);
```

Für detaillierte Schlupfmodelle und ideale Allokation ist die Optimization
Toolbox erforderlich. Historische Parallelstudien können zusätzliche Toolboxes
benötigen. Simulation und Energie sind quasistatisch; thermische Dynamik,
Zellspannung, gemessene Generator-Kennfelder und vollständige Plattformdynamik
sind nicht abgedeckt. Zeitkonvergenz allein beweist keine Reifenfeasibilität an
allen Segmentmittelpunkten. Für Auslegungsentscheidungen Grenzaudit und
numerische Verfeinerung prüfen.

Erzeugte Ergebnisse, Caches, lokale Archive und `_backup_*`-Ordner sind von Git
ausgeschlossen. Feste Eingaben unter `data` bleiben versionierbar. Die erste Bereinigung steht in `docs/cleanup-2026-10-13.md`; die gemeinsame
Energiearchitektur und Wiederherstellung in `docs/cleanup-2026-10-14.md`.
Weitere Details: `docs/transparent-v2.md`, `docs/architecture.md`,
`docs/script-execution-flow.md` und `docs/energy-model.md`.
