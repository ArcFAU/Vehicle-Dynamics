# Anforderungsabgleich nextGen V2

Diese Übersicht trennt umgesetzte Funktionen von bewusst späteren
Erweiterungen. Maßgeblich bleibt die projektlokale DOCX-Anforderungsliste.

| Bereich | Status in V2 | Nachweis / Grenze |
|---|---|---|
| Gemeinsamer QSS-Rundensolver | umgesetzt | Alle Level liefern dieselbe Schnittstelle an `lts.solver.solveLap`. |
| Level 0 Massepunkt | umgesetzt | 0.1 konstant, 0.2 lastabhängig, 0.3 mit Reifenhülle, Aero und Antrieb. |
| Level 1 Einspur | umgesetzt | Achslasten, Lasttransfer, Aero-/Antriebsgrenzen; 1.3 zusätzlich vollständige stationäre Schlupfzustände. |
| Level 2 Zweispur | umgesetzt | Vier Radlasten, Lasttransfer, radindividuelle Kräfte und Torque-Vectoring-Schnittstellen. |
| Reifen | umgesetzt | Austauschbare stationäre Modelle; fitted Combined Slip und TMeasy; last-, richtungs- und combined-slip-abhängige schnelle Hülle. |
| Performance-Map | umgesetzt | Globale oder tracklokale Erzeugung, Fahrzeugidentität, Numerikmetadaten und gespeicherte Grenzzustände. |
| Häufig wechselnde Setups | umgesetzt | Level 0.3/1.2 direkt ohne Map; `trackLocal` nur für gezielte Detailrechnungen. |
| Bis zu zehn Strecken / Events | umgesetzt | Projektlokaler Track-Katalog und Event-Set; globale Map optional, nicht Voraussetzung. |
| Aero | teilweise | Drag, Abtrieb, Balance und Angriffshöhe umgesetzt; mehrdimensionale Aero-Kennfelder/DRS-Zustände sind noch nicht allgemein implementiert. |
| Powertrain / Übersetzung | umgesetzt | Drehmoment-, Leistungs-, Drehzahl-, Übersetzungs- und Effizienzgrenzen. |
| Akku, Energie, Reku | umgesetzt | Energiefluss, SoC, Leistungsgrenzen, Reku und Akkumulatorziele; thermische/elektrische Dynamik bleibt geplant. |
| Bremsbedarf | umgesetzt | Erforderliche Gesamtbremskraft und Radbremsmomente als Ergebnis; keine künstliche mechanische Standardgrenze. |
| Fahrwerk / Chassis | teilweise | Rollsteifigkeitsverteilung und Chassistorsion in Level 2; allgemeine Kinematikkennfelder bleiben geplant. |
| Regelungen / Torque Vectoring | umgesetzt | Feste und ideale Radkraftverteilung sowie Soll-Giermoment im Zweispur-Betriebspunkt. |
| Design-Sensitivitäten | umgesetzt | Vollfaktorielle 1D/2D-Studien, lokale Ableitungen, Eventpunkte, Energiegrößen und Verlust zum besten Konzept. |
| Ergebnisdiagnose | umgesetzt | Geschwindigkeits-/Beschleunigungsprofile, Segmentzeitvergleich, Limitklassifikation, Brems-/Gierbedarf und Betriebspunktabdeckung. |
| Legacy-Vergleich | umgesetzt | Reine Datenabbildung über `lts.compatibility.mapLegacyVehicle`; feste Regressionen für Tau Acceleration und Endurance. Keine Laufzeitabhängigkeit vom Altprojekt. |

## Bewusste Modellgrenzen

- V2 ist eine quasistatische Designsimulation. Transiente Reifenrelaxation,
  vollständige Vertikaldynamik und Fahrerregler gehören nicht in den schnellen
  Standardkern.
- `IncludeYawInertia=true` prüft kritische gefahrene Punkte mit zusätzlichem
  Gierträgheitsmoment. Der Standard bleibt die stationäre theoretische Grenze.
- Eine Performance-Map ist eine kontrollierte numerische Approximation. Für
  wenige Strecken und häufige Fahrzeugänderungen wird sie nicht automatisch
  global aufgebaut.
- Ein identisches Ergebnis zu Legacy ist nur bei identischen Eingangsdaten und
  vergleichbarer Physik zu erwarten. Die Regression überwacht definierte
  Abweichungsbänder; zusätzliche Aussagekraft der V2 darf nicht durch
  willkürliches Nachstellen einzelner Ergebniszahlen verloren gehen.
