# Transparente V2-Konfiguration

## Grundsatz

Ein Input beschreibt vollständig und sichtbar genau einen Simulationsfall.
Physikalische Features werden ausschließlich durch explizite logische Flags
aktiviert. Numerische Einstellungen sind davon getrennt. Fehlende Eingaben,
unzulässige Kombinationen und nicht unterstützte aktive Features stoppen den
Lauf mit einer eindeutigen Fehlermeldung.

## Öffentliche Modelle und Features

Die öffentliche Modellwahl ist auf `pointMass`, `singleTrack` und `dualTrack`
begrenzt. Jeder Fall setzt die Flags `aero`, `loadTransfer`, `tireSlip`,
`energy`, `yawInertia` und `chassisTorsion` ausdrücklich auf `true` oder
`false`.

`singleTrack` und `dualTrack` benötigen aktuell `loadTransfer=true`, weil die
vorhandenen validierten Kerne keine Variante ohne Lasttransfer besitzen.
`chassisTorsion` ist nur mit `dualTrack` und `tireSlip=true` darstellbar.
`yawInertia` ist nur als Audit der lokalen Schlupfberechnung verfügbar.

## Keine versteckten Einstellungen

`lts.v2.validateCase` erzeugt eine Kopie des effektiven Fahrzeugs. Bei
`aero=false` werden Widerstands- und Auftriebskoeffizienten in dieser Kopie auf
null gesetzt. Bei deaktivierter Chassistorsion wird eine unendlich steife
Struktur verwendet. Das Eingabefahrzeug selbst wird nicht verändert.

Vor der Rechnung zeigt `lts.v2.runCase` Modellfamilie, Feature-Flags,
gewünschte und tatsächlich verwendete Rechenstrategie, Rechenkern, jeden Input
mit Status `used`, `ignored` oder `notSupported` und die vollständige effektive Struktur.
Dieselbe Information wird mit dem Ergebnis gespeichert.

## Numerische Strategie

Fälle ohne Schlupf verwenden `direct`. Schlupfmodelle verwenden `trackLocal`
oder eine explizite `mapped` Performance-Map. Die axellastabhängige
Point-Mass-Auswertung trägt intern den historischen Namen `legacyQss`; diese
Abbildung wird vor dem Lauf ausdrücklich ausgegeben und gespeichert.

## Rückwärtskompatibilität

Die bestehenden Level-IDs bleiben als interne und historische Schnittstelle
erhalten. V2 kapselt sie, statt geprüfte Modellgleichungen zu duplizieren.

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
