# Architektur nextGen V2

> Die öffentliche, vereinfachte Schnittstelle ist in
> `transparent-v2.md` beschrieben. Sie stellt nur `pointMass`, `singleTrack`
> und `dualTrack` bereit. Die nachfolgend dokumentierten Level und Modell-IDs
> bleiben interne Rechenkerne und Kompatibilitätsschnittstellen.

## Ziel

V2 trennt physikalische Modelle, Ausführungsstrategien und Designauswertungen.
Eine Gleichung wird genau einem Paket zugeordnet und von allen Modellstufen
wiederverwendet.

## Pakete

- `+lts/+model`: Fahrzeugmodelle und lokale Betriebspunkte
- `+lts/+solver`: streckenbezogene Vorwärts-/Rückwärtslösung
- `+lts/+tire`: gemeinsame stationäre Reifenmodelle
- `+lts/+powertrain`: Drehmoment-, Leistungs- und Drehzahlgrenzen
- `+lts/+aerodynamics`: Luftwiderstand und Abtrieb
- `+lts/+control`: feste und optimierte Radkraftverteilung
- `+lts/+energy`: Energiefluss, Rekuperation und SoC
- `+lts/+analysis`: Brems-, Gier- und Betriebspunktanalysen
- `+lts/+design`: zentraler Designfall und Capability-Matrix
- `+lts/+events`, `+lts/+scoring`, `+lts/+study`: Events, Punkte und Sensitivitäten
- `+lts/+io`: projektlokale Eingabedaten

## Modell und Strategie

Die explizite Modell-ID legt die Physik fest. Das zugehörige Level ist nur eine
abgeleitete Klassifikation. Die Strategie legt fest, wie dieselbe Physik
ausgewertet wird. Erlaubte Kombinationen stammen ausschließlich aus
`lts.model.registry`; `lts.model.availableConfigurations` stellt sie dar.

- `direct`: direkte schnelle Hüllengleichungen
- `legacyQss`: V2-native, schnelle Referenzausführung der quasistatischen
  Hüllenmodelle in Level 0.3 und 1.2 für den Vergleich mit historischen
  Ergebnissen
- `trackLocal`: intern und temporär erzeugte Slip-Grenzen für den durch eine
  schnelle Vorrechnung bestimmten Streckengeschwindigkeitsbereich; Stützstellen
  werden aus den gefahrenen Geschwindigkeits- und Querbeschleunigungsquantilen
  gewählt
- `mapped`: globale vorberechnete Slip-Grenzen

`trackLocal` und `mapped` müssen denselben Betriebspunktsolver verwenden.
Aktuell ist `trackLocal` für Level 1.3 sowie Level 2.1/2.5 implementiert. Die
lokale Hülle wird nicht als Fahrzeugartefakt gespeichert und kann daher nicht
versehentlich für eine andere Fahrzeugkonfiguration wiederverwendet werden.
Nach der Rundenrechnung werden kritische gefahrene Betriebspunkte direkt mit
dem nichtlinearen Solver auditiert. Bei aktivierter Gierträgheitskopplung geht
`I_z * yawAcceleration_radps2` als Sollmoment in dessen Momentenbilanz ein.
Ein nicht bestätigter Punkt bricht den Lauf ab, statt unbemerkt als gültiges
Ergebnis weiterverarbeitet zu werden. Die interpolierte Grundhülle selbst ist
weiterhin stationär; die Kopplung ist daher als kritischer Hybrid-Audit und
nicht als zusätzliche globale Map-Dimension umgesetzt.
Eine numerische Nichtkonvergenz ist der Status `unknown` und keine physikalische
Unzulässigkeit.

`legacyQss` ist kein zweiter Physikkern: Level 0.3 verwendet die gemeinsame
Massepunkt-Aero-Hülle, Level 1.2 die gemeinsame Einspur-Aero-Hülle. Der eigene
Strategiename macht Vergleichsläufe und Ergebnisdateien eindeutig. Historische
Profile werden ausschließlich als Daten über `lts.analysis.adaptLegacyResult`
eingelesen; V2 führt keine Dateien aus dem alten Simulationsordner aus.
`lts.analysis.compareResults` vergleicht Rundenzeit und Geschwindigkeitsprofil
auf einem gemeinsamen Wegkoordinatenraster.

Die schnelle Reifenhülle wird einmal aus demselben ausgewählten Reifenmodell
abgeleitet, das auch die detaillierten Betriebspunkte verwendet. Sie speichert
getrennte Längs- und Querkraftmaxima als Funktion der Radlast und kombiniert
beide Richtungen mit dem gemeinsamen Combined-Slip-Exponenten. Dadurch sind
Level 0.3/1.2 schnell, ohne das Reifenmodell auf einen einzelnen konstanten
Reibwert zu reduzieren. Die Level 1.3/2.x verwenden weiterhin die vollständige
Schlupflösung. Es existiert damit ein Reifendatensatz mit zwei klaren
Auswertetiefen statt zweier unabhängig parametrierter Reifenmodelle.

Der Legacy-Adapter unter `+lts/+compatibility` ist eine reine Datenbrücke. Er
prüft die Identität des lokal versionierten Reifenfits und erzeugt eine normale
V2-Fahrzeugstruktur. Produktionsmodelle und Solver besitzen keine Abhängigkeit
vom alten Projekt.

## Datenfluss

```text
vehicle + track + modelId + strategy
                |
                v
        lts.config.createCase
                |
                v
       lts.config.validateCase
                |
                v
        lts.design.runCase
                |
                v
      strategy preparation
      (trackLocal if selected)
                |
                v
       common lap solver
                |
       +--------+---------+
       |        |         |
     energy   brake      yaw
       |      demand    demand
       +--------+---------+
                |
        design result
```

## Erweiterungsregeln

1. Neue Physik erhält eine klar benannte Komponente, keine Kopie eines Modells.
2. Ein niedrigeres Level bleibt als analytische oder schnelle Referenz erhalten.
3. Jede Designaussage wird in `lts.design.capabilityMatrix` einem Mindestlevel
   zugeordnet.
4. Performance-Daten speichern Modell-, Reifen-, Fahrzeug- und Numerikidentität.
5. Ergebnisse enthalten Einheiten im Feldnamen und einen Qualitätsstatus.
6. Projektcode hängt zur Laufzeit nicht von älteren Simulationsordnern ab.
7. Offene Strecken verwenden nur explizit gesetzte Start-/Endgeschwindigkeiten;
   ohne Endwert wird am Ziel keine künstliche Bremsung erzwungen.
8. Modell-ID, Strategie, Energie- und Gieroptionen sind explizit; fehlende oder
   widersprüchliche Angaben brechen vor der Simulation ab.

## Bereinigung am 2026-10-13

Die allgemeine Simulation und die FSG-Studien verwenden unmittelbar
`lts.model.dualTrackEnvelope`; die frühere V8-Hülle wurde entfernt. Die Querlastprüfung bleibt unabhängig von idealer
Längskraftallokation wirksam. `lts.model.maximumSteadySpeed` löst die
Geschwindigkeitssuche auch ohne aktive Motordrehzahlgrenze. Antriebs- und
Bremspeaks im Massepunktmodell werden getrennt ausgewertet.

Aktive Runner, Referenzdaten und Qualitätsprüfungen sind in
`script-execution-flow.md` dokumentiert. Die aktuellen Referenzdaten sind
projektlokale Eingaben unter `data`; erzeugte Ergebnisse und Sicherungen sind
keine Laufzeitquellen der allgemeinen Simulation.

## Gemeinsame Energiearchitektur

`lts.energy.propagateSegment` ist der einzige Energieintegrator.
`lts.energy.analyzeLap` verwendet ihn für vollständige Strecken, Zeitfenster und
Wiederholungen. Der gekoppelte Solver ruft denselben Integrator auf. Gemeinsame
Antriebsgrenzen liegen in `lts.powertrain.driveLimits` und
`driveDemandFeasible`; ein optionaler drehzahlabhängiger Drehmomentverlauf liegt
im Motor-Modul. `constant`, `efficiencyMap` und `lossMap` wählen die Motorkonversion.
Die FSG-Bewertung wendet ihren Rekuperationsfaktor separat an.

Nur `lts.compatibility.normalizeEnergyModel` kennt die alten V13/V14-Datenfelder.
Der Adapter übersetzt historische Eingaben beim Laden; neue Ergebnisse und
der aktive Kern verwenden ein gemeinsames Schema.
