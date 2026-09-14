# Skriptablauf und Datenfluss

## Zweck und Lesart

Diese Dokumentation beschreibt, wie die ausfuehrbaren `run*.m`-Dateien das
nextGen-V2-Modell aufrufen, welche Daten dabei entstehen und an welcher Stelle
welche Berechnung stattfindet. Die Einstiegsskripte liegen im Projektstamm; die
eigentliche Implementierung ist nach Aufgaben in den MATLAB-Paketen unter
`+lts` gegliedert.

Die wichtigsten Begriffe sind:

- **Fahrzeug**: Struktur aus `vehicles/FAUmaxTauV2.m`, einschliesslich Masse,
  Geometrie, Reifen, Aero, Antrieb, Bremsen, Fahrwerk, Regelung und Energie.
- **Strecke**: Struktur mit Weg `distance_m`, Kruemmung `curvature_1pm`,
  Metadaten und Kennzeichen `isClosed`.
- **Modell**: Einheitliche Schnittstelle mit mindestens
  `maximumSpeedForCurvature` und `accelerationLimits`.
- **Lap-Result**: Konvergiertes Geschwindigkeits-, Zeit- und
  Beschleunigungsprofil aus `lts.solver.solveLap`.
- **Performance-Map**: Vorberechnete, interpolierbare Fahrgrenze eines
  detaillierten Slip-Modells.

## Schnellauswahl: Was soll untersucht werden?

Diese Tabelle ist der empfohlene Einstieg. Sie trennt die fachliche
Fragestellung von der technischen Implementierung.

| Fragestellung | Empfohlener Einstieg | Modell/Strategie | Warum diese Wahl? |
|---|---|---|---|
| Funktioniert das Projekt grundsaetzlich? | `runTests`, danach `runDemo` | `pointMassAero` / `direct` | Schnellster Integritaets- und End-to-End-Check |
| Welchen Einfluss hat die Modellkomplexitaet? | `runModelComparison` | explizite Modell-ID-Liste | Rechnet nur die angeforderten Modelle auf derselben Strecke |
| Schneller allgemeiner Konzeptvergleich | `lts.design.runCase` | `pointMassAero` / `direct` | Beruecksichtigt Aero und Antrieb bei geringer Laufzeit |
| Schneller Vergleich mit Achslast- und Bremsbalance | `lts.design.runCase` | `singleTrackAero` / `direct` | Vorder-/Hinterachse statt reiner Punktmasse |
| Radlasttransfer oder radindividuelle Regelung | `lts.design.runCase` | `dualTrackEnvelope` / `direct` | Vier Radaufstandskraefte ohne teuren Slip-Map-Aufbau |
| Vollstaendige Einspur-Slip-Zustaende | `lts.design.runCase` | `singleTrackSlip` / `trackLocal` | Lenkwinkel, Schwimmwinkel und Achsschlupf fuer eine Strecke |
| Vollstaendige Zweispur-/Torque-Vectoring-Analyse | `lts.design.runCase` | `dualTrackSlipFitted` / `trackLocal` | Radindividuelle Kraefte und Schlupf mit direktem Audit |
| Alternatives TMeasy-Reifenmodell | `runTireModelComparison` oder `runCase` | `dualTrackSlipTMeasy` / `trackLocal` | Vergleicht beziehungsweise nutzt den ausgewaehlten Reifenkern |
| Viele Streckenlaeufe mit exakt gleichem Fahrzeug | eigener Aufruf von `runCase` | 1.3, 2.1 oder 2.5 / `mapped` | Einmalige globale Map kann wiederverwendet werden |
| Antriebslayouts vergleichen | `runPowertrainComparison` | 2.0-Hülle | Direkter Vergleich von sechs Layouts |
| Antriebslayout inklusive Energie/Reku | `runPowertrainEnergyComparison` | 2.0-Hülle + Event/Energie | Liefert Zeit-, Energie- und Akkumulatorwerte |
| Feste gegen ideale Kraftverteilung | `runTorqueAllocationComparison` | 2.0-Hülle | Isoliert das Potenzial der Allokation |
| Reifenmodelle vergleichen | `runTireModelComparison` | 2.1/2.5-Maps | Baut fuer jedes Reifenmodell eine passende Map |
| Rollsteifigkeitsverteilung, schnelle Vorauswahl | `runRollStiffnessComparison` | 2.0-Hülle | Geeignet fuer schnelle Fahrwerksbewertung |
| Rollsteifigkeit mit detailliertem Slip | `runRollStiffnessSlipComparison` | 2.1-Maps | Erst nach der schnellen Vorauswahl verwenden |
| Chassis-Torsionssteifigkeit, schneller Sweep | `runChassisStiffnessComparison` | 2.0-Hülle | Mehrere Steifigkeiten mit vertretbarer Laufzeit |
| Ausgewaehlte Chassissteifigkeit mit Slip | `runChassisStiffnessSlipComparison` | 2.1-Maps | Detaillierter Nachweis fuer einen ausgewaehlten Wert |
| Parameter-, Event- und Punktesensitivitaet | `runSensitivityStudy` | explizit gewaehltes Direktmodell | Vollfaktorieller Sweep ohne Map-Aufbau pro Fall |
| Energie- und Akkumulatorauslegung | `runEnergyReport` | Studienmodell + Energie | Baseline ueber alle konfigurierten Events |
| Rueckwirkung von SoC auf Rundenzeit | `runSensitivityStudy` mit `coupleEnergy=true` | Level 2.4c-Pfad | Leistung wird segmentweise durch den SoC begrenzt |

### Entscheidungsbaum

```text
Welche Aussage wird benoetigt?
  |
  +-- Nur schneller Funktionscheck ---------------------- runDemo
  |
  +-- Einzelner allgemeiner Designfall
  |     +-- Punktmasse ausreichend --------------------- pointMassAero / direct
  |     +-- Achslasten relevant ------------------------ singleTrackAero / direct
  |     +-- einzelne Raeder/Lasttransfer relevant ------ dualTrackEnvelope / direct
  |     +-- Schlupfzustände benoetigt
  |            +-- Einspur ausreichend ---------------- singleTrackSlip / trackLocal
  |            +-- radindividuell/TV ------------------ dualTrackSlipFitted / trackLocal
  |            +-- TMeasy ----------------------------- dualTrackSlipTMeasy / trackLocal
  |
  +-- Viele Varianten oder Parameterpunkte
  |     +-- Fahrzeug aendert sich je Fall -------------- direct/Hüllenmodell
  |     +-- Fahrzeug bleibt exakt gleich --------------- globale mapped-Map
  |
  +-- Vordefinierter Fachvergleich --------------------- passendes run*Comparison
  |
  +-- Events, Punkte oder Sensitivitaet ---------------- runSensitivityStudy
        +-- SoC nur auswerten -------------------------- coupleEnergy=false
        +-- SoC begrenzt Fahrleistung ------------------ coupleEnergy=true
```

Die Standardempfehlung lautet: erst mit Level 0.3, 1.2 oder 2.0 eine schnelle
Vorauswahl treffen und nur die verbleibenden Kandidaten mit Level 1.3, 2.1 oder
2.5 detailliert nachrechnen.

## Erlaubte Modell-/Strategiekombinationen

`lts.config.createCase()` akzeptiert nur die folgenden Kombinationen und
`lts.design.runCase()` führt anschließend ausschließlich diese geprüfte
Konfiguration aus:

Die Tabelle muss nicht aus der Dokumentation abgeschrieben werden. Der aktuelle
Stand der Registry ist jederzeit read-only abrufbar:

```matlab
lts.model.availableConfigurations()
lts.model.availableConfigurations(Strategy="mapped")
```

| Modell-ID | Level | `direct` | `legacyQss` | `trackLocal` | `mapped` | Typischer Aufwand |
|---|:---:|:---:|:---:|:---:|:---:|---|
| `pointMass` | 0.1 | ja | nein | nein | nein | sehr gering |
| `pointMassLoadSensitive` | 0.2 | ja | nein | nein | nein | sehr gering |
| `pointMassAero` | 0.3 | ja | ja | nein | nein | gering |
| `singleTrack` | 1.1 | ja | nein | nein | nein | gering |
| `singleTrackAero` | 1.2 | ja | ja | nein | nein | gering |
| `singleTrackSlip` | 1.3 | nein | nein | ja | ja | hoch beim Map-Aufbau |
| `dualTrackEnvelope` | 2.0 | ja | nein | nein | nein | gering bis mittel |
| `dualTrackSlipFitted` | 2.1 | nein | nein | ja | ja | hoch beim Map-Aufbau |
| `dualTrackSlipTMeasy` | 2.5 | nein | nein | ja | ja | hoch beim Map-Aufbau |

Dabei bedeutet:

- `direct`: Modellgrenzen werden direkt und ohne vorbereitete Slip-Map
  ausgewertet. Das ist die Standardwahl fuer Variantenstudien.
- `legacyQss`: V2-interne schnelle Vergleichsbasis fuer historische
  QSS-Vergleiche. Sie ruft keinen Code aus dem alten Projekt auf.
- `trackLocal`: `runCase` erstellt automatisch eine kompakte Map fuer genau
  dieses Fahrzeug und diese Strecke. Der Nutzer muss keine Map verwalten.
- `mapped`: Der Aufrufer muss eine zuvor erzeugte, zum Fahrzeug passende Map
  ueber `PerformanceMap=` uebergeben.

### Technisch moeglich ist nicht automatisch fachlich geeignet

| Aussage | Mindestwahl | Ungeeignete Vereinfachung |
|---|---|---|---|
| reine analytische Referenz | 0.1 | keine |
| lastabhaengige Reifenwirkung | 0.2 | 0.1 |
| Leistung, Uebersetzung, Topspeed oder Aero | 0.3 | 0.1/0.2 |
| Masselage oder Bremsbalance | 1.1, mit Aero/Antrieb 1.2 | Punktmasse 0.x |
| Lenkwinkel, Schwimmwinkel oder Achsschlupf | 1.3 | Hüllenmodelle bis 1.2 |
| Links-/Rechts-Radlasttransfer | 2.0 | Einspurmodelle |
| radindividueller Schlupf oder Torque Vectoring | 2.1 | 2.0 oder niedriger |
| TMeasy-spezifische Aussage | 2.5 | fitted-Combined-Slip-Modell |
| SoC-abhaengige Rundenzeit | energiegekoppelter 2.4c-Pfad | reine Nachanalyse |

Ein niedrigeres Modell darf fuer schnelle Vorauswahl und Plausibilisierung
verwendet werden, aber nicht als Nachweis fuer Physik, die es nicht enthaelt.

## Runner-Kompatibilitaet und Voraussetzungen

| Runner | Eingabeparameter | Internes Modell | Map | Energie | Voraussetzung |
|---|---|---|---|---|---|
| `runDemo()` | keine | 0.3 `direct` via `runCase` | nein | Nachanalyse | keine |
| `runModelComparison(modelIds, map)` | explizite Modell-ID-Liste, Map nur fuer Slip-Modelle | genau angeforderte Modelle | fuer Slip-Modelle erforderlich | nein | Map muss zum Fahrzeug und Modell passen |
| `runPowertrainComparison()` | keine | 2.0 | nein | nein | keine |
| `runPowertrainEnergyComparison()` | keine | 2.0 | nein | Eventanalyse | keine |
| `runTorqueAllocationComparison(layout)` | verpflichtender Layoutname | 2.0 | nein | Nachanalyse | gueltiger Layoutname |
| `runTireModelComparison(options)` | Raster-/Map-Optionen | 2.1/2.5 Slip | pro Reifen neu | Nachanalyse | lange Laufzeit moeglich |
| `runSlipModelComparison()` | keine | 2.0 gegen 2.1 | wird neu gebaut | nein | erzeugt Baseline-Datei |
| `runSingleTrackSlipComparison(map, BuildMap=...)` | Map plus explizite Build-Entscheidung | 1.2 gegen 1.3 | Eingabe oder bewusst neu gebaut | nein | genau eine Map-Quelle waehlen |
| `runRollStiffnessComparison()` | keine | 2.0 | nein | nein | keine |
| `runRollStiffnessSlipComparison()` | keine | 2.1 Slip | Baseline laden, zweite Map bauen | nein | zuerst `runSlipModelComparison` |
| `runChassisStiffnessComparison()` | keine | 2.0 | nein | nein | keine |
| `runChassisStiffnessSlipComparison(k)` | verpflichtende endliche Steifigkeit | 2.1 Slip | starre Map laden, endliche bauen | nein | zuerst `runRollStiffnessSlipComparison` |
| `runSensitivityStudy(config)` | explizite Konfiguration | 0.3, 1.2 oder 2.0 | nein | explizit gekoppelt oder ungekoppelt | alle Konfigurationsfelder erforderlich |
| `runEnergyReport(config)` | explizite Konfiguration | wie Sensitivitaetsfall | nein | ja | vollstaendige Konfiguration erforderlich |
| `runTests()` | keine | alle Testmodelle | testabhaengig | testabhaengig | MATLAB Test Framework |

### Wichtige Map-Regeln

- Eine Single-Track-Map kann nicht als Dual-Track-Map verwendet werden und
  umgekehrt.
- Eine Map ist an die relevante Fahrzeug-, Reifen-, Fahrwerks- und
  Regelungskonfiguration gebunden.
- Aendert ein Sweep das Fahrzeug, darf nicht dieselbe Map ungeprueft fuer alle
  Varianten verwendet werden.
- `trackLocal` ist die sicherste bequeme Wahl fuer einen einzelnen
  Detailfall; `mapped` lohnt sich bei vielen Strecken mit unveraendertem
  Fahrzeug.
- Die `*SlipComparison`-Runner bauen benoetigte Varianten-Maps bewusst neu
  oder laden eine dokumentierte Baseline aus `results/`.

## Konkrete Aufrufrezepte

### Schneller Designfall

```matlab
vehicle = FAUmaxTauV2();
tracks = lts.io.trackCatalog();
track = lts.io.loadTrackMatrix(tracks.endurance, true);

config = lts.config.createCase(Vehicle=vehicle, Model="dualTrackEnvelope", ...
    Strategy="direct", IncludeEnergy=false, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);
```

### Detaillierter Einspur-Slip-Fall ohne manuelle Map

```matlab
config = lts.config.createCase(Vehicle=vehicle, Model="singleTrackSlip", ...
    Strategy="trackLocal", IncludeEnergy=false, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);
```

### Detaillierter radindividueller Fall mit Giertraegheits-Audit

```matlab
config = lts.config.createCase(Vehicle=vehicle, Model="dualTrackSlipFitted", ...
    Strategy="trackLocal", IncludeEnergy=false, IncludeYawInertia=true, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);
```

### Globale Map fuer mehrere Strecken wiederverwenden

```matlab
performanceMap = lts.model.dualTrackSlip.buildPerformanceMap(vehicle);

config = lts.config.createCase(Vehicle=vehicle, Model="dualTrackSlipFitted", ...
    Strategy="mapped", PerformanceMap=performanceMap, ...
    IncludeEnergy=false, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
endurance = lts.design.runCase(config, enduranceTrack);
autocross = lts.design.runCase(config, autocrossTrack);
```

Zwischen beiden Laeufen darf die relevante Fahrzeugkonfiguration nicht
veraendert werden.

### Schnelle Sensitivitaetsstudie

```matlab
config = sensitivityStudyCase(Study="mass", ...
    Model="dualTrackEnvelope", PowertrainLayout="fourMotorAWD", ...
    TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
    CoupleEnergy=false, CreatePlots=true);
study = runSensitivityStudy(config);
```

### SoC-gekoppelte Eventstudie

```matlab
config = sensitivityStudyCase(Study="power", ...
    Model="dualTrackEnvelope", PowertrainLayout="fourMotorAWD", ...
    TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
    CoupleEnergy=true, CreatePlots=true);
study = runSensitivityStudy(config);
```

Die gekoppelte Variante ist deutlich teurer, weil das Geschwindigkeitsprofil
und der SoC gemeinsam iteriert werden.

## Gesamtuebersicht

```text
run*.m
  |
  +-- Projektpfade setzen
  +-- Fahrzeug laden/variieren ---------- vehicles/FAUmaxTauV2.m
  +-- Strecke/Event laden --------------- +lts/+io, +lts/+config/+events
  +-- Modell erzeugen -------------------- +lts/+model
  |      +-- Reifen ---------------------- +lts/+tire
  |      +-- Aero ------------------------ +lts/+aerodynamics
  |      +-- Antrieb --------------------- +lts/+powertrain
  |      +-- Kraftverteilung ------------- +lts/+control
  |
  +-- optional Performance-Map aufbauen -- singleTrackSlip/dualTrackSlip
  +-- Runde/Event loesen ----------------- +lts/+solver, +lts/+events
  +-- Ergebnisse auswerten --------------- +lts/+analysis, +lts/+energy
  +-- optional Punkte/Sensitivitaet ------- +lts/+scoring, +lts/+study
  +-- Ausgabe
         +-- Rueckgabestruktur im Workspace
         +-- Konsolenausgabe/Plots
         +-- teilweise results/*.mat
```

## Projektstart und Eingabedaten

Beim Oeffnen von `LaptimeSimulation_nextGen.prj` verwendet MATLAB die
Projektkonfiguration. Jedes Einstiegsskript ermittelt zusaetzlich seinen
Projektordner ueber `mfilename('fullpath')` und fuegt den Projektstamm sowie,
wo erforderlich, `vehicles` und `config` zum Suchpfad hinzu. Dadurch sind die
Paketnamen `lts.*`, `FAUmaxTauV2` und die Studienkonfiguration erreichbar.

`lts.io.trackCatalog()` bildet die logischen Streckennamen auf versionierte
MAT-Dateien unter `data/tracks` ab. `lts.io.loadTrackMatrix()` liest daraus
ausschliesslich die Variable `A`:

1. Spalte 1 wird als Wegkoordinate uebernommen und auf null normiert.
2. Spalte 2 wird als Kruemmung verwendet.
3. Dateiname, Quellpfad und Importer werden in `track.meta` dokumentiert.
4. `lts.validation.validateTrack()` prueft die entstandene Struktur.

Die Quelldatei wird dabei nicht veraendert.

## Zentraler Designfall: `runDemo` und `lts.design.runCase`

`runDemo()` ist der kuerzeste vollstaendige Standardlauf:

```text
runDemo
  -> FAUmaxTauV2
  -> lts.io.trackCatalog
  -> lts.io.loadTrackMatrix(endurance, closed=true)
  -> lts.config.createCase(Model="pointMassAero", Strategy="direct", ...)
  -> lts.design.runCase(config, track)
  -> Textausgabe und Geschwindigkeitsplot
```

`lts.config.createCase()` ist der zentrale Einstieg fuer alle fachlichen
Entscheidungen; `lts.design.runCase()` ist der zentrale Rechenlauf fuer einen
einzelnen Designfall. Sein Ablauf ist:

1. `lts.config.validateCase()` prueft Schema, Eingabestrukturen, Strecke,
   Registry-Zuordnung, Reifenanforderung und gegebenenfalls die Map. Vor dem
   Lauf wird standardmaessig eine kompakte Zusammenfassung angezeigt.
2. Bei `Strategy="trackLocal"` wird zuerst eine streckenspezifische
   Performance-Map erzeugt.
3. `lts.model.registry()` ordnet die explizite Modell-ID genau einer
   Modellfabrik, einem Level und erlaubten Strategien zu.
4. `lts.solver.solveLap()` berechnet das Rundenprofil.
5. Eine lokale Slip-Map wird an kritischen gefahrenen Punkten direkt mit dem
   nichtlinearen Betriebspunktsolver auditiert.
6. Bremsbedarf, lokale Limitklasse und Gierbedarf werden nachgerechnet.
7. Sofern `IncludeEnergy=true`, folgt die Energieanalyse.
8. Metadaten und Capability-Matrix werden gemeinsam mit allen Ergebnissen in
   `result` zurueckgegeben.

### Zuordnung von Modell-ID und Strategie

| Modell-ID | Level | Strategie | erzeugtes Modell |
|---|---|---|
| `pointMass` | 0.1 | `direct` | `lts.model.pointMass.create` |
| `pointMassLoadSensitive` | 0.2 | `direct` | `lts.model.pointMassLoadSensitive.create` |
| `pointMassAero` | 0.3 | `direct`, `legacyQss` | `lts.model.pointMassAero.create` |
| `singleTrack` | 1.1 | `direct` | `lts.model.singleTrack.create` |
| `singleTrackAero` | 1.2 | `direct`, `legacyQss` | Aero- oder Legacy-QSS-Einspurmodell |
| `singleTrackSlip` | 1.3 | `mapped`, `trackLocal` | `lts.model.singleTrackSlip.create` |
| `dualTrackEnvelope` | 2.0 | `direct` | `lts.model.dualTrackEnvelope.create` |
| `dualTrackSlipFitted` | 2.1 | `mapped`, `trackLocal` | Dual-Track-Slip mit fitted Combined Slip |
| `dualTrackSlipTMeasy` | 2.5 | `mapped`, `trackLocal` | Dual-Track-Slip mit TMeasy |

Unzulaessige Kombinationen werden vor der Rundenrechnung mit einer eindeutigen
Fehlermeldung abgebrochen.

### Vorabpruefung ohne Simulation

```matlab
summary = lts.config.validateCase(config, track);
```

Dieser Aufruf veraendert weder Fahrzeug noch Strecke, baut keine Map und startet
keinen Lap-Solver. `summary.workload` unterscheidet direkte Auswertung,
Wiederverwendung einer validierten Map und den angeforderten Track-Local-Aufwand.
Mit `ShowSummary=false` in `createCase` kann lediglich die automatische Anzeige
in Batchlaeufen unterdrueckt werden; die Validierung selbst bleibt aktiv.

### Ergebnisstruktur von `runCase`

```text
result
  meta                 Level, Strategie, Modellname, Status, ggf. Map-Infos
  lap                  Weg, Zeit, Geschwindigkeit, Beschleunigungen, Solvermeta
  brakeDemand          Bremskraft- und Radbremsmomentbedarf
  limitClassification  lokale Ursache der Fahrgrenze
  yawDemand            Gierdynamik- und Momentenbedarf
  energy               Energie, Rekuperation, SoC und Akkumulatorziele (optional)
  capabilities         gueltige Designaussagen je Modelllevel
```

## Kernsolver `lts.solver.solveLap`

Der Solver kennt keine konkrete Fahrzeuggleichung. Er verwendet nur die zwei
Methoden der Modellschnittstelle. Dadurch durchlaufen alle Hüllenmodelle und
interpolierten Slip-Modelle denselben Streckensolver.

```text
Strecke + Modell
  -> Eingaben validieren
  -> kurvenbegrenzte Startgeschwindigkeit an jedem Stuetzpunkt
  -> Start-/Endrandbedingung setzen
  -> wiederholen bis Konvergenz:
       Vorwaertspass: erreichbare Geschwindigkeit aus Beschleunigungsgrenze
       Rueckwaertspass: erreichbare Geschwindigkeit aus Bremsgrenze
       geschlossene Strecke: ersten und letzten Punkt angleichen
  -> Segmentzeiten integrieren
  -> Quer- und Laengsbeschleunigungen berechnen
  -> Lap-Result erzeugen
```

Im Vorwaertspass gilt segmentweise die kinematische Beziehung
`v2^2 = v1^2 + 2*a*ds`. Die maximal zulaessige positive Beschleunigung kommt
aus `model.accelerationLimits()`. Der Rueckwaertspass verwendet dieselbe
Beziehung mit der maximalen Verzoegerung. Sobald sich das Profil um hoechstens
`speedTolerance_mps` aendert, gilt es als konvergiert. Nichtkonvergenz und ein
Segment mit verschwindender mittlerer Geschwindigkeit fuehren zu einem Fehler
statt zu einem scheinbar gueltigen Ergebnis.

## Track-lokale und globale Slip-Maps

Eine globale Map wird in den Vergleichsskripten explizit durch
`buildPerformanceMap()` aufgebaut und kann fuer mehrere Laeufe desselben
Fahrzeugs wiederverwendet werden. `trackLocal` wird dagegen innerhalb von
`runCase()` nur fuer die aktuelle Fahrzeug-Strecken-Kombination erzeugt:

1. Ein schnelles Hüllenmodell berechnet eine Seed-Runde.
2. Aus deren Geschwindigkeiten und Querbeschleunigungen werden repraesentative
   Stuetzstellen ausgewaehlt.
3. Der nichtlineare Single- oder Dual-Track-Solver sucht an diesen Stellen die
   positiven und negativen Fahrgrenzen.
4. Die Map erhaelt Fahrzeug-, Reifen-, Raster-, Sicherheits- und
   Streckenmetadaten.
5. Der normale Lap-Solver interpoliert anschliessend diese Grenzen.
6. Stark ausgelastete, besonders schnelle und querbeschleunigungsreiche Punkte
   werden nochmals direkt geprueft. Optional wird dabei `I_z*dr/dt` in der
   Giermomentenbilanz beruecksichtigt.

Kann der direkte Audit einen Punkt nicht bestaetigen, verwirft `runCase()` den
Lauf. Eine numerische Map ist damit ein kontrolliertes Rechenartefakt und nicht
selbst die physikalische Wahrheit.

## Energiepfade

### Nachgelagerte Energieanalyse

`lts.energy.analyzeLap(lap, vehicle)` wertet ein bereits konvergiertes Profil
aus. Sie bilanziert unter anderem kinetische Energie, Luft- und Rollwiderstand,
Nebenverbraucher, Antriebswirkungsgrade und begrenzte Rekuperation. Dieser Weg
veraendert die zuvor berechnete Rundenzeit nicht.

### Energiegekoppelter Solver

`lts.solver.solveLapWithEnergy()` koppelt den SoC und die verfuegbare
Entladeleistung in das Geschwindigkeitsprofil zurueck:

```text
Basismodell und Strecke
  -> normale Seed-Runde
  -> Strecke bei Event-Wiederholungen expandieren
  -> iterativ:
       Rueckwaertspass mit voller Bremsgrenze
       Vorwaertspass mit SoC-abhaengiger Antriebsleistung
       SoC segmentweise mit propagateSegment fortschreiben
  -> Zeitprofil und finale Energieanalyse
```

Nicht rekuperierbare Bremsleistung wird weiterhin durch Reibbremse abgedeckt.
Dieser Pfad entspricht der optionalen Kopplung auf Level 2.4c.

## Events, Scoring und Sensitivitaet

`lts.events.runSet()` verarbeitet eine Liste von Eventdefinitionen:

1. Deaktivierte und optional ausgeschlossene Driverless-Events werden
   uebersprungen.
2. Pflichtfelder und Streckendatei werden validiert.
3. Die Eventstrecke wird geladen.
4. Bei geeigneter Energie-Kopplung wird `solveLapWithEnergy()` verwendet,
   andernfalls `solveLap()`.
5. Eventzeiten entstehen je nach Definition aus der ganzen Strecke, der Summe
   oder dem Mittel festgelegter Zeitfenster und einem Multiplikator.
6. Die Energie wird gekoppelt berechnet oder nachgelagert analysiert.

`lts.study.runParameterSweep()` baut darauf einen vollfaktoriellen Sweep auf:

```text
Basisfahrzeug + Parameterachsen
  -> jede Parameterkombination erzeugen
  -> Parameter auf eine Fahrzeugkopie anwenden
  -> Modell ueber modelFactory erzeugen
  -> alle Events mit lts.events.runSet berechnen
  -> Punkte mit lts.scoring.scoreEvents bestimmen
  -> absolute, relative und lokale Sensitivitaeten aggregieren
```

Das Basisfahrzeug bleibt unveraendert; jeder Fall arbeitet mit seiner eigenen
Strukturkopie. Als lokale Sensitivitaet wird um den Baseline-Fall eine
zentrale beziehungsweise randseitige Differenz aus benachbarten Stuetzstellen
gebildet.

## Einstiegsskripte im Projektstamm

| Skript | Zweck und wesentliche Aufrufkette | Persistente Ausgabe |
|---|---|---|
| `runDemo` | Level 0.3 auf Endurance ueber `runCase`; zeigt Speedplot | keine |
| `runModelComparison` | rechnet ausschliesslich die explizit angeforderten Modell-IDs; Slip-Modelle verlangen eine passende Map | keine |
| `runPowertrainComparison` | sechs Antriebslayouts, Dual-Track-Hülle, je ein Lap-Lauf | `results/powertrainComparison.mat` |
| `runPowertrainEnergyComparison` | sechs Layouts, Endurance-Event, Zeit- und Energievergleich | `results/powertrainEnergyComparison.mat` |
| `runTorqueAllocationComparison` | feste gegen ideale Radkraftverteilung; Lap plus Energie | `results/torqueAllocationComparison.mat` |
| `runTireModelComparison` | fitted Combined Slip gegen TMeasy; Map-Aufbau, Lap und Energie | `results/tireModelComparison.mat` |
| `runSlipModelComparison` | schnelle Level-2.0-Hülle gegen Level-2.1-Slip-Map | `results/slipModelComparison.mat` |
| `runSingleTrackSlipComparison` | Level 1.2 gegen 1.3; vorhandene Map oder `BuildMap=true` muss explizit gewaehlt werden | keine |
| `runRollStiffnessComparison` | feste gegen aus Federn/ARB abgeleitete Rollverteilung | `results/rollStiffnessComparison.mat` |
| `runRollStiffnessSlipComparison` | wiederholt den Rollvergleich mit detaillierten Slip-Maps | `results/rollStiffnessSlipComparison.mat` |
| `runChassisStiffnessComparison` | Sweep der Torsionssteifigkeit mit schneller Level-2.0-Hülle | `results/chassisStiffnessComparison.mat` |
| `runChassisStiffnessSlipComparison` | starres gegen ein endliches Chassis mit Slip-Maps | `results/chassisStiffnessSlipComparison_<Wert>NmPerDeg.mat` |
| `runSensitivityStudy` | Konfiguration laden, vollfaktoriellen Event-/Punktesweep rechnen und plotten | keine |
| `runEnergyReport` | Baseline aus der Sensitivitaetskonfiguration rechnen und Energie-/Akkuziele ausgeben | keine |
| `runTests` | alle Tests unter `tests` sammeln, ausfuehren und Erfolg erzwingen | MATLAB-Testresultat |

Alle Skripte geben ihre Ergebnisstruktur auch direkt an den Aufrufer zurueck.
Die Spalte „Persistente Ausgabe“ bezeichnet nur zusaetzlich geschriebene
MAT-Dateien.

## Abhaengigkeiten zwischen Vergleichsskripten

Die meisten Runner sind unabhaengig. Zwei detaillierte Fahrwerksvergleiche
verwenden jedoch bewusst bereits erzeugte Maps:

```text
runSlipModelComparison
  -> results/slipModelComparison.mat (performanceMap)
  -> runRollStiffnessSlipComparison
       -> results/rollStiffnessSlipComparison.mat (derivedMap)
       -> runChassisStiffnessSlipComparison
```

Fehlt die benoetigte Vorstufe, brechen die Skripte mit
`lts:runner:MissingBaselineMap` beziehungsweise
`lts:runner:MissingRigidSlipMap` ab. Die schnelle
`runChassisStiffnessComparison` besitzt diese Abhaengigkeit nicht.

## Detailablauf der Vergleiche

### Antrieb und Kraftverteilung

- `runPowertrainComparison` erzeugt fuer jedes Layout ein neues Fahrzeug,
  verwendet immer `dualTrackEnvelope` und vergleicht nur die Rundenzeiten.
- `runPowertrainEnergyComparison` nutzt die FSG-Eventdefinition fuer
  Endurance. Neben der Eventzeit werden Nettoenergie, Rekuperation,
  Akku-Kapazitaet sowie Spitzenleistungen gespeichert.
- `runTorqueAllocationComparison` kopiert das Basisfahrzeug durch
  `lts.control.applyIdealAllocation`, rechnet feste und ideale Allokation mit
  identischer Strecke und wertet Zeitpotenzial sowie Energie aus.

### Reifen, Roll- und Chassissteifigkeit

- `runTireModelComparison` waehlt je Reifenmodell den Parameterdatensatz,
  baut eine eigene Dual-Track-Slip-Map und verwendet diese fuer Lap und Energie.
- `runSlipModelComparison` erzeugt die Baseline-Slip-Map, die auch als
  Ausgangsraster fuer den spaeteren Rollsteifigkeitsvergleich dient.
- `runRollStiffnessComparison` verwendet die schnelle Hülle. Die Slip-Variante
  baut fuer das geaenderte Fahrzeug eine neue Map mit identischem Raster und
  identischen Numerikoptionen.
- `runChassisStiffnessComparison` veraendert nacheinander die
  Torsionssteifigkeit. Die Slip-Variante vergleicht nur einen ausgewaehlten
  endlichen Wert mit dem starren Referenzfall und baut dafuer eine neue Map.

## Sensitivitaetskonfiguration

`runSensitivityStudy()` akzeptiert nur eine explizite, vollstaendige
Konfigurationsstruktur. `config/sensitivityStudyCase.m` erzeugt sie erst, wenn
Studie, Modell, Antriebslayout, Reifenmodell, Driverless-Umfang,
Energie-Kopplung und Plot-Ausgabe angegeben wurden. Verfuegbare Studien sind
Masse, Masse mit proportionaler Giertraegheit, Massenverteilung, Abtrieb,
Aerobalance, zweidimensionale Aeromap, Leistung und Bremsbalance.

`runEnergyReport()` reduziert die erste Parameterachse auf ihren Baseline-Wert
und nutzt danach denselben Studienpfad. Auch hier ist die Konfiguration ein
Pflichtargument. Der Report ist daher kein separater Physikpfad, sondern eine
kompakte Auswertung eines einzelnen Sweep-Falls.

## Fallback- und Annahmenpolitik

Fachliche Entscheidungen duerfen nicht durch fehlende Angaben ausgeloest
werden. Die oeffentlichen Simulationspfade folgen deshalb diesen Klassen:

| Klasse | Verhalten |
|---|---|
| `error` | fehlende Pflichtauswahl; die Simulation startet nicht |
| `notAvailable` | eine Auswertung ist mangels Daten nicht berechenbar und liefert Status plus `NaN` |
| `approximation` | vereinfachte Physik; nur nach explizitem Opt-in |
| `compatibilityDefault` | Altbestand ohne vollstaendige Metadaten; nicht fuer neue Ergebnisse zulaessig |
| `numericalRetry` | gleiche Physik mit anderem Startwert; kein Modellwechsel |

Konkret sind `Model`, `Strategy`, `IncludeEnergy`, `IncludeYawInertia` und die
Geschwindigkeitsgrenzen beim Erzeugen eines Falls Pflicht. Das Level wird nur
noch als abgeleitete Metainformation aus der Registry gespeichert und kann
nicht separat gewaehlt oder widerspruechlich gesetzt werden.
Auch Track-Local-Audit und Map-Aufbau unterscheiden Modelle ueber Registry-
Eigenschaften wie `mapType` und `supportsYawAudit`, nicht ueber Levelnummern.
Sensitivitaetsstudien verlangen alle fachlichen Auswahlfelder. Neue
Dual-Track-Maps muessen vollstaendige Reifen- und Regelungsmetadaten besitzen.
Die symmetrische schnelle TMeasy-Hülle ist nur nach explizitem Setzen von
`vehicle.tireModel.allowFastEnvelopeApproximation=true` zulaessig; fuer eine
TMeasy-Aussage ist Level 2.5 mit Slip-Map vorzuziehen.

## Was im Dateisystem veraendert wird

- Strecken- und Fahrzeugquelldaten werden nur gelesen.
- Die meisten Runner erzeugen lediglich MATLAB-Workspace-Daten, Textausgaben
  und Figuren.
- Vergleichsskripte mit einer Ausgabe in der obigen Tabelle legen bei Bedarf
  `results/` an. Existiert die fest benannte MAT-Datei bereits, wird der Lauf
  vor dem Speichern mit `lts:io:ResultExists` abgebrochen; bestehende
  Ergebnisse werden niemals still ueberschrieben.
- Slip-Maps koennen gross und rechenintensiv sein; die Map-Metadaten muessen zum
  Fahrzeug passen und werden beim Erzeugen des Modells geprueft.
- `runTests()` veraendert keinen Produktionscode, kann aber normale temporaere
  MATLAB-Testartefakte erzeugen.

## Empfohlene Reihenfolge

1. `runTests` fuer den Integritaetsnachweis ausfuehren.
2. `runDemo` fuer einen schnellen End-to-End-Test verwenden.
3. `runModelComparison` fuer die Wirkung der Modellstufen verwenden.
4. Direkte/Hüllenvergleiche fuer schnelle Konzeptstudien ausfuehren.
5. Slip-Map-Vergleiche nur fuer ausgewaehlte Konzepte rechnen.
6. Fuer die aufeinander aufbauenden Fahrwerks-Slip-Studien die dokumentierte
   Reihenfolge einhalten.
7. `runSensitivityStudy` fuer systematische Parameter- und Punkteauswertung
   verwenden; Energie-Kopplung nur aktivieren, wenn die Rueckwirkung des SoC
   auf die Rundenzeit untersucht werden soll.

## Fehler- und Qualitaetssicherung

Die Berechnung bricht unter anderem ab, wenn:

- Fahrzeug, Strecke, Event oder Modellschnittstelle unvollstaendig sind,
- Modelllevel und Strategie nicht zusammenpassen,
- eine Map fehlt oder nicht zur Fahrzeugkonfiguration passt,
- eine benoetigte Ergebnisdatei einer Vorstufe nicht existiert,
- der normale oder energiegekoppelte Lap-Solver nicht konvergiert,
- ein Segment wegen null mittlerer Geschwindigkeit unendliche Zeit haette,
- ein kritischer Track-local-Betriebspunkt nicht direkt bestaetigt wird.

Eine explizit angeforderte Energiekopplung wird ebenfalls abgebrochen, wenn
Modell oder Event die Kopplung nicht unterstuetzen. Es findet dann keine stille
Rueckkehr zur nachgelagerten Energieanalyse statt. Scoring bricht ab, wenn fuer
ein simuliertes Event keine oder mehr als eine Regel existiert oder ueberhaupt
kein Event bewertet werden kann. Dual-Track-Performance-Maps speichern die
vollstaendige Quellfahrzeugdefinition und werden bei jeder Abweichung vom
aktuellen Fahrzeug abgelehnt.

Die Tests unter `tests/` decken Modellstufen, Reifenmodelle, Energie, Events,
Scoring, Sensitivitaet, Regelung, Fahrwerk, offene Strecken und
Legacy-Vergleiche ab. `runTests()` ruft `assertSuccess` auf, sodass ein Lauf mit
mindestens einem fehlgeschlagenen Test nicht stillschweigend als erfolgreich
endet.

Letzter Gesamtlauf am 14.09.2026: 118 von 119 Tests bestanden. Der verbleibende
rote Test `conditionalRestartEscapesLocalMinimum` verlangt mehr als einen
Solverstart, obwohl der geprüfte Betriebspunkt reproduzierbar im ersten Versuch
mit `residualInf = 4.636e-16` konvergiert. Dies ist eine veraltete Testannahme;
der Rechenwert selbst ist gültig.
