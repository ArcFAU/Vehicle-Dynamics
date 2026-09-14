# Laptime Simulation nextGen V2

V2 ist eine übersichtliche, erweiterbare QSS-Rundenzeitsimulation für
Formula-Student-Konzeptentscheidungen. Alle Modellstufen verwenden gemeinsame
Fahrzeug-, Reifen-, Antriebs-, Energie- und Auswertungskomponenten.

## Modell-IDs und Physikstufen

Die Modell-ID ist die ausführbare Auswahl. Das zugehörige Level beschreibt nur
den physikalischen Umfang und wird von der Registry abgeleitet.

| Level | Modell | Typische Designaussagen |
|---|---|---|
| 0.1 | Massepunkt, konstanter Reibwert | analytische Referenz |
| 0.2 | Massepunkt, lastabhängige Reifenhülle | Masse, Reifenlastsensitivität |
| 0.3 | Massepunkt mit Aero und Antrieb | Masse, Leistung, Übersetzung, Topspeed, Aero, Energie- und Bremsbedarf |
| 1.1 | Einspur mit Achslasten | Masselage und Bremsbalance |
| 1.2 | Einspur mit Aero und Antrieb | schnelle Achskonzepte |
| 1.3 | Einspur mit vollständigem Reifenmodell | Lenkwinkel, Schwimmwinkel, Schlupf und Reifenausnutzung |
| 2.0 | Zweispur-Reifenhülle | Radlasttransfer und radindividuelle Regelung |
| 2.1+ | Zweispur mit vollständigem Reifenmodell | Torque Vectoring, Reifen- und Fahrwerkskonzepte |

Die vollständige Zuordnung liefert `lts.design.capabilityMatrix()`.
Ein niedrigeres Level darf nur für Aussagen verwendet werden, deren Physik es
enthält. Insbesondere kann ein Massepunkt keine Masselage oder Torque-Vectoring-
Wirkung abbilden.

## Zentraler Arbeitsablauf

Die Modellwahl erfolgt ausschließlich über eine der stabilen Modell-IDs aus
`lts.model.registry()`. Das numerische Level wird daraus abgeleitet und ist
keine zweite, unabhängig setzbare Auswahl. Fehlen Modell oder Strategie, wird
kein Lauf gestartet.

Alle erlaubten Kombinationen können direkt aus derselben Registry angezeigt
oder gefiltert werden:

```matlab
lts.model.availableConfigurations()
lts.model.availableConfigurations(Strategy="trackLocal")
lts.model.availableConfigurations(Model="dualTrackSlipFitted")
```

```matlab
vehicle = FAUmaxTauV2();
tracks = lts.io.trackCatalog();
track = lts.io.loadTrackMatrix(tracks.endurance, true);

config = lts.config.createCase(Vehicle=vehicle, Model="pointMassAero", ...
    Strategy="direct", IncludeEnergy=true, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);
```

`createCase` aktiviert standardmäßig eine kompakte Fallzusammenfassung vor dem
Rechenlauf. Für automatisierte Läufe kann sie mit `ShowSummary=false`
unterdrückt werden. Eine reine Vorabprüfung ohne Simulation ist ebenfalls
möglich:

```matlab
summary = lts.config.validateCase(config, track);
```

Die Prüfung kontrolliert Modell, Strategie, Reifenanforderung, Strecke,
Geschwindigkeitsgrenzen und gegebenenfalls die übergebene Performance-Map. Bei
`trackLocal` zeigt `summary.workload` außerdem die angeforderte Raster- und
Iterationsgröße an.

Sensitivity studies require every physics and scope selection explicitly:

```matlab
config = sensitivityStudyCase(Study="mass", ...
    Model="dualTrackEnvelope", PowertrainLayout="fourMotorAWD", ...
    TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
    CoupleEnergy=false, CreatePlots=true);
mass = runSensitivityStudy(config);
```

Für ein vollständiges Reifenmodell ohne vorbereitete globale Map:

```matlab
config = lts.config.createCase(Vehicle=vehicle, Model="singleTrackSlip", ...
    Strategy="trackLocal", IncludeEnergy=true, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);
```

Für schnelle Vergleiche mit der ursprünglichen quasistatischen LaptimeSim:

```matlab
referenceConfig = lts.config.createCase(Vehicle=vehicle, Model="singleTrackAero", ...
    Strategy="legacyQss", IncludeEnergy=false, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
detailedConfig = lts.config.createCase(Vehicle=vehicle, Model="singleTrackSlip", ...
    Strategy="trackLocal", IncludeEnergy=false, IncludeYawInertia=false, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
reference = lts.design.runCase(referenceConfig, track);
detailed = lts.design.runCase(detailedConfig, track);
comparison = lts.analysis.compareResults(reference, detailed);
```

Ein historischer `C`-Datensatz wird einmalig in die normale V2-Struktur
überführt; danach läuft ausschließlich V2-Code:

```matlab
legacy = load('C:/Octanes/SVN/LaptimeSim/RundenzeitSimulation/Fahrzeuge/Tau.mat', 'C');
[vehicle, mapping] = lts.compatibility.mapLegacyVehicle(legacy.C);
```

`legacyQss` ist vollständig in V2 implementiert und führt keinen Legacy-Code
aus. Die Strategie bezeichnet die schnelle Vergleichsbasis; sie behauptet
keine bitgenaue Gleichheit mit der historischen Implementierung. Exportierte
Legacy-Profile können mit `lts.analysis.adaptLegacyResult` in dieselbe
Ergebnisform gebracht und anschließend mit `compareResults` ausgewertet werden.

Die Modelle `singleTrackSlip`, `dualTrackSlipFitted` und
`dualTrackSlipTMeasy` arbeiten quasistatisch. Damit bleibt der
stationäre theoretische Maximalwert reproduzierbar und der Map-Aufbau kompakt.
Für eine strengere Prüfung kann die Gierträgheit in den direkt nachgerechneten
kritischen Betriebspunkten zugeschaltet werden:

```matlab
config = lts.config.createCase(Vehicle=vehicle, Model="singleTrackSlip", ...
    Strategy="trackLocal", IncludeEnergy=false, IncludeYawInertia=true, ...
    MaximumSpeed_mps=100, InitialSpeed_mps=0, FinalSpeed_mps=Inf);
result = lts.design.runCase(config, track);
```

Scheitert diese Prüfung, wird kein unbemerkt gültiges Ergebnis ausgegeben.

`result` enthält Rundenzeit und Geschwindigkeitsprofil sowie Bremsbedarf,
Gierbedarf, Energie und die Capability-Matrix. Für Punkte- und
Sensitivitätsstudien stehen die bestehenden `lts.events`, `lts.scoring` und
`lts.study` Pakete zur Verfügung.

Alle Design-Runner speichern unter `comparison(i).result` dieselbe
`lts.design.runCase`-Struktur. Die eigentliche Runde liegt immer unter
`result.lap`; Modell-ID, Strategie und vollständige Fallkonfiguration stehen
unter `result.meta`.

## Ausführungsstrategien

- `direct`: regulärer schneller Rechenkern der Hüllenmodelle.
- `legacyQss`: explizit benannte schnelle V2-Vergleichsbasis für
  `pointMassAero` und `singleTrackAero`. Sie verwendet die gemeinsamen
  quasistatischen Hüllengleichungen und
  enthält keine Abhängigkeit vom alten Projekt.
- `mapped`: vorberechnete Slip-Performance-Map für viele Streckenläufe mit
  unverändertem Fahrzeug.
- `trackLocal`: automatisch erzeugte, nur für die aktuelle Strecke gültige
  Slip-Hülle für die registrierten Slip-Modelle. Sie verwendet denselben nichtlinearen
  Betriebspunktsolver wie `mapped`, beschränkt dessen Berechnungsbereich aber
  auf Geschwindigkeits- und Querbeschleunigungsquantile einer schnellen
  Vorrechnung.

`trackLocal` benötigt keine vom Anwender aufgebaute oder gespeicherte Map. Die
interne lokale Hülle wird bei jeder geänderten Fahrzeug-Strecken-Kombination neu
berechnet. Aufbauzeit, Rasterbereich und Anzahl der berechneten Grenzen stehen
anschließend unter `result.meta.trackLocal`.
Standardmäßig werden zusätzlich die acht am stärksten beanspruchten sowie die
Punkte mit maximaler Querbeschleunigung und Geschwindigkeit direkt mit dem
nichtlinearen Solver geprüft. Mit `IncludeYawInertia=true` kommt der Punkt mit
maximalem Gierträgheitsbedarf hinzu und
`targetYawMoment_Nm = I_z * yawAcceleration_radps2` geht in die Momentenbilanz
ein. Kann der Solver einen Punkt nicht bestätigen, wird der Lauf nicht als
gültiges Ergebnis ausgegeben.

Die Laufzeit wird bei Level 1.3/2.x weiterhin vom nichtlinearen
Betriebspunktsolver bestimmt. Sie ist deshalb deutlich höher als bei Level
0.3/1.2. Eine globale Map lohnt sich vor allem bei vielen Streckenläufen mit
unverändertem Fahrzeug; bei häufig veränderten Fahrzeugen sind `direct` oder
`legacyQss` der Standard und `trackLocal` die gezielte Detailprüfung.

Eine Map ist eine numerische Approximation. Raster, Grenzsuche,
Sicherheitsfaktor und Fahrzeugidentität werden deshalb mit ihr gespeichert.
Dual-Track-Maps enthalten die vollständige Quellfahrzeugdefinition und werden
nach jeder Abweichung der aktuellen Fahrzeugstruktur abgelehnt.
Für schnelle Variantenstudien ist `legacyQss` beziehungsweise `direct` die
erste Wahl; `trackLocal` dient der detaillierteren Reifen-, Schlupf- und
Fahrdynamikaussage.

## Designausgaben

- Rundenzeit, Geschwindigkeit und Beschleunigungen
- mechanischer Bremskraft- und Radbremsmomentbedarf
- Energiebedarf, Rekuperation, SoC und Akkumulatorziele
- Lenkwinkel, Schwimmwinkel, Schlupf und Reifenausnutzung in den Slip-Modellen
- Giermomentbedarf und Regelungspotenzial
- lokale Limitklassifikation (Reifen, Bremsen, Drehmoment, Leistung)
- Eventpunkte, allgemeine Parametersensitivitäten und Zeit-/Punkteverlust
  gegenüber dem besten untersuchten Konzept
- tatsächlich befahrene Betriebspunkte über
  `lts.analysis.operatingPointCoverage`

Eine optionale vollständige Performance-Hülle kann später mit dieser Abdeckung
überlagert werden, ohne Bestandteil des Solverkerns zu werden.

## Daten und Herkunft

Die Projektstrecken liegen versioniert unter `data/tracks`; V2 benötigt zur
Laufzeit keinen älteren Simulationsordner. Der fitted Combined-Slip-Datensatz
ist ein übernommener Entwicklungsdatensatz mit dokumentierter Herkunft. Das
alternative stationäre TMeasy-Modell bleibt auswählbar. Vor Designfreigaben
müssen Reifendatensatz, Aero- und Antriebsdaten zum betrachteten Fahrzeug passen.

## Qualität

```matlab
results = runTests;
```

Letzter vollständiger Lauf am 14.09.2026: 118 von 119 Tests bestanden. Der
einzige rote Test erwartet an einem bereits im ersten Versuch vollständig
konvergierten Betriebspunkt zwingend einen numerischen Retry. Der Solverwert
ist machbar und reproduzierbar (`residualInf = 4.636e-16`); betroffen ist die
Testannahme, nicht das Rechenergebnis.

Modellgleichungen und Grenzen sind in `docs/model-levels.md` beschrieben, die
Paketstruktur in `docs/architecture.md`, der detaillierte Ablauf aller
Einstiegsskripte in `docs/script-execution-flow.md` und Event-/Punkteberechnungen
in `docs/events-scoring-sensitivity.md`. Der Abgleich mit der Anforderungsliste
steht in `docs/requirements-status.md`.
