# Skriptablauf und Datenfluss

## Einstieg und Projektpfade

`project/startup.m` aktiviert Projektstamm, `vehicles`, `config`, `tools` sowie
`tools/comparisons`, `tools/diagnostics` und `tools/legacy`. Acht Run-Funktionen
bleiben im Projektstamm. Archive und Sicherungen sind keine MATLAB-Suchpfade.

| Fragestellung | Einstieg |
|---|---|
| Projekt prüfen | `runTests` |
| Ein Simulationsfall | `runDemoV2` |
| Modellfamilien vergleichen | `runModelComparisonV2` |
| Parameter, Events und Punkte | `runSensitivityStudyV2` |
| Baseline-Energie | `runEnergyReportV2` |
| FSG-Sensitivitäten | `runFsg2026SensitivityV9` |
| FSG-Konzeptziele | `runFsg2026Top10TargetsV8` |
| Aktuelles AMK-Antriebskonzept | `runFsg2026PowertrainConceptV14` |

## Allgemeiner V2-Fall

```text
runDemoV2
  -> vehicles/FAUmaxTauV2 + lts.io.trackCatalog/loadTrackMatrix
  -> lts.v2.createCase / validateCase
  -> lts.v2.runCase
  -> lts.config.createCase / lts.design.runCase
  -> Rechenkern aus lts.model.registry
  -> lts.solver.solveLap
  -> Bremsbedarf, Limitklassifikation, Gierbedarf, optionale Energieanalyse
```

Die öffentlichen Modelle sind `pointMass`, `singleTrack` und `dualTrack`.
Features `aero`, `loadTransfer`, `tireSlip`, `energy`, `yawInertia` und
`chassisTorsion` werden explizit gesetzt. Ohne Schlupf wird `direct` verwendet;
mit Schlupf `trackLocal` oder eine ausdrücklich übergebene `mapped`-Map.

`trackLocal` baut eine fahrzeug- und streckenspezifische Map und prüft kritische
Punkte direkt mit dem nichtlinearen Solver. Eine globale Map ist an Fahrzeug,
Reifen, Allokation und Numerik gebunden. Sie darf nach physikalischen
Fahrzeugänderungen nicht ungeprüft wiederverwendet werden.

## Event- und Sensitivitätsworkflow

```matlab
config = sensitivityStudyCaseV2(Study="mass",Model="dualTrack", ...
    Aero=true,LoadTransfer=true,Energy=true,PowertrainLayout="fourMotorAWD", ...
    TireModel="fittedCombinedSlip",IncludeDriverless=false, ...
    CoupleEnergy=false,CreatePlots=false);
study = runSensitivityStudyV2(config);
```

`lts.study.runParameterSweep` erstellt eine eigene Fahrzeugkopie für jede
Parameterkombination. `lts.v2.buildDirectModel` liefert das Modell,
`lts.events.runSet` die Eventzeiten/Energien und `lts.scoring.scoreEvents` die
Punkte. Bei `coupleEnergy=true` wird der allgemeine energiegekoppelte Solver
verwendet. Event-Timingfenster unterstützen diese Kopplung nicht; eine explizite
Anforderung führt zu einem Fehler statt zu einem stillen Ersatzlauf.

## Aktueller FSG-V14-Pfad

```text
runFsg2026PowertrainConceptV14
  -> data/reference/r20SlipMapReleaseValidationV6.mat
  -> TMeasy-abgeleitete quadratische Reifenhülle
  -> data/powertrain/A2370DD_T100C.mat
  -> lts.powertrain.configureVehicle
  -> lts.model.dualTrackEnvelope
  -> lts.solver.solveLapWithEnergy
  -> lts.energy.analyzeLap / profileTable
  -> neue Checkpoints, Tabellen und ausgewählte Profile
  -> analyzeFsg2026PowertrainConceptV14
  -> auditFsg2026PowertrainLimitsV14
```

Der gemeinsame Energie-Solver begrenzt ein übergebenes Basisprofil ebenfalls durch Maximal- und
Zielgeschwindigkeit. Wiederverwendete stationäre Runden erhalten dieselben
Solveroptionen. Ein bindender Zielwert am Ende eines wiederholten Events
verwendet die vollständige Rechnung statt periodischer Wiederverwendung.

Der Standard-OutputTag `modular_amk100C_500V` und die Checkpoint-Version wurden
bei der Bereinigung geändert. Alte Resultate bleiben als historische Daten
verfügbar. Der optionale V13-Vergleich ist keine Voraussetzung für V14.

## Spezialläufe

| Ordner | Funktionen |
|---|---|
| `tools/comparisons` | `runPowertrainComparisonV2`, `runPowertrainEnergyComparisonV2`, `runTorqueAllocationComparisonV2`, `runSingleTrackSlipComparisonV2`, `runSlipModelComparisonV2`, `runTireModelComparisonV2`, `runRollStiffnessComparisonV2`, `runRollStiffnessSlipComparisonV2`, `runChassisStiffnessComparisonV2`, `runChassisStiffnessSlipComparisonV2` |
| `tools/diagnostics` | `runAeroStateEnvelopeV2`, `runFsg2026SlipEnvelopeTireComparison`, `runR20SlipMapReleaseValidationV6`, `auditFsg2026PowertrainLimitsV14` |
| `tools/legacy` | `runTau2026LegacyComparison` |

Die historischen Top10-Runner V2–V6 und die Studienrunner V10–V12 wurden nach
vollständiger Sicherung entfernt. Aktuelle V8-/V9-/V14-Einstiege ersetzen ihre
Rolle im aktiven Projekt. Die ersetzten V12/V13-Helfer wurden ebenfalls entfernt. Der aktuelle
Auswerter kann weiterhin historische V13-Ergebnis-CSV zum Vergleich lesen.

## Rechenkerne und Qualität

- `lts.model.maximumSteadySpeed` bracketing und Suche verwenden nur endliche
  Geschwindigkeiten. Bei deaktivierter Drehzahlgrenze liefert Drag plus
  verfügbare Leistung eine Obergrenze. Ohne Drag werden Kurven endlich
  eingeklammert; gerade Abschnitte haben keine stationäre Geschwindigkeitsgrenze.
- Der gemeinsame Zweispurkern prüft die Reifenquerlast unabhängig von der
  idealen Längskraftallokation. Die frühere V8-Kompatibilitätshülle wurde entfernt.
- Das Massepunktmodell verwendet eigene Antriebs- und Bremspeaks, auch bei
  optionaler Achslastauflösung.
- `auditLap` prüft Segmentmittelpunkte samt Rollwiderstand und berichtet
  zeitgewichtete Reifen-/Motor-/Batterieüberschreitungen. Seine ideale Reifen-
  Obergrenze kann Unzulässigkeit nachweisen, aber keine Schlupflösung bestätigen.
- `runTests` führt alle projektlokalen Tests aus und erzwingt Erfolg. Die
  externen Tau-Regressionen sind mit `IncludeLegacyRegression=true` zuschaltbar.
