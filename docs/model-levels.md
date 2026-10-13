# Model Levels

Levels classify implemented physics; they are not executable defaults. Users
select an explicit model ID and strategy, while `lts.model.registry` derives
the associated level. Every level must pass its analytic and regression tests.

| Level | Physics | Status |
|---|---|---|
| 0.1 | Point mass and constant-friction grip circle | Implemented |
| 0.2 | Load-sensitive tire force envelope | Implemented |
| 0.3 | Power, torque, drag and downforce | Implemented |
| 1.0 | Front and rear axle balance | Implemented in 1.1 |
| 1.1 | Longitudinal load transfer | Implemented |
| 1.2 | Single track with aero and powertrain limits | Implemented |
| 1.3 | Fast mapped single track with slip states | Implemented |
| 2.0 | Four-wheel force envelope and lateral load transfer | Implemented |
| 2.1a | Named slip-force law and identified fit adapter | Implemented |
| 2.1b | Slip-consistent dual-track operating point | Implemented |
| 2.1c | Slip-consistent mapped lap-performance boundaries | Implemented |
| 2.2a | Elastic front/rear roll-stiffness distribution | Implemented |
| 2.2b | Roll-stiffness slip-map comparison | Implemented |
| 2.3a | Finite chassis torsional stiffness | Implemented |
| 2.3b | Selected chassis-stiffness slip-map comparison | Implemented |
| 2.4a | EV energy flow, recuperation and accumulator targets | Implemented |
| 2.4b | SoC limits and component-efficiency map interfaces | Implemented |
| 2.4c | Optional SoC-coupled laptime and repeated events | Implemented |
| 2.4d | Fixed versus ideal drive, brake and regen allocation | Implemented |
| 2.5a | Common selectable stationary tire-model interface | Implemented |
| 2.5b | ADTM TMeasy V2.5 stationary operating points | Implemented |
| 2.5c | TMeasy performance maps and lap interface | Implemented |
| 2.6+ | Thermal limits, voltage dynamics, platform and kinematics | Planned |

Level 0.1 intentionally ignores vehicle mass in acceleration limits because a
constant tire coefficient produces force proportional to mass. Mass becomes a
meaningful laptime design variable when power, drag, downforce and/or tire load
sensitivity enter later levels.

Level 0.2 treats four tires as equally loaded. Level 1.1 separates front and
rear axles, satisfies stationary lateral force and yaw-moment balance, applies
the configured driven axle and brake-force distribution, and updates axle
normal loads with longitudinal acceleration. Lateral left/right load transfer
begins at Level 2.0.

Levels 0.3 and 1.2 use a cached directional peak envelope derived from the
selected stationary tire model. Longitudinal and lateral peaks retain their
own fitted load dependency; a common combined-slip exponent couples them.
Traction respects the number of driven wheels, while braking uses all four
contact patches. The `legacyQss` Level 0.3 option additionally resolves axle
loads and longitudinal load transfer. This is more informative than a single
constant friction coefficient while remaining much faster than solving slip
variables at every boundary query.

Level 1.3 reuses the common stationary tire model, nonlinear slip solver and
performance-map implementation with equal left/right axle loads. It exposes
front/rear slip ratio, slip angle, body slip, steering angle, tire forces and
utilization without duplicating the Level 2 tire equations. Map generation is
an offline approximation step; subsequent lap and parameter-study runs use
fast interpolation. Grid density, interpolation reserve and direct checks of
critical operating points must be recorded with results. `lts.analysis.yawDemand`
reports the path-imposed `Iz*dr/dt` demand; feedback of that varying moment into
the map boundary is not yet implemented and remains a documented difference
from the historical ES iteration.

A performance map is an approximation, not merely an algebraic rewrite. Its
accuracy depends on speed and lateral-fraction resolution, boundary-search
iterations and interpolation reserve. Every Level 1.3 map therefore stores
these settings plus the relevant vehicle identity and is rejected for a
different vehicle. Coarse maps are suitable for integration checks only;
decision studies require grid-convergence evidence and direct nonlinear checks
at critical profile points. Boundary states (normal and tire forces, slip
variables, body slip and steering angle where available) are stored alongside
the numerical envelope, so the approximation remains inspectable.

For historical equivalence studies, Level 0.3 and Level 1.2 are parameterized
from the same source dataset and used instead of the intentionally reduced
Level 0.1/1.1 baselines. This activates power, motor-speed, drag and downforce
limits while retaining the normal nextGen model hierarchy. Mechanical brake
capacity is not imposed as a laptime constraint; the required contact-patch
brake force and per-wheel moment are derived from the converged profile by
`lts.analysis.brakeDemand`. The comparison bridge is for regression evidence
only and is not a permanent dependency of the production models.

Level 2.1a preserves the identified LC0 source fit through a narrow adapter and
exposes pure- and combined-slip tire forces with positive normal load and SI
angle inputs. The fit is a development dataset and does not match the selected
R20 tire names. Level 2.1b solves body slip, steering and four wheel-slip
ratios against longitudinal force, lateral force, yaw moment and drivetrain
distribution. Level 2.1c samples these nonlinear operating points into signed
left/right performance maps. The unchanged lap solver interpolates those maps,
while Level 2.0 remains available as the fast force-envelope reference.
The current Level 2.1c map applies a configurable 0.80 interpolation safety
factor and bilinear interpolation. Denser-map validation can later support a
smaller reserve.

Level 2.2a converts spring rates through the stated wheel/damper motion ratios,
adds axle ARB stiffness and derives the elastic roll-moment distribution. The
application function returns a modified vehicle copy. Geometric load transfer,
unsprung-mass transfer and compliant-chassis coupling remain later steps.
Level 2.2b rebuilds the signed slip-performance map with the derived roll
distribution. Each map records that distribution and is rejected when paired
with a different vehicle configuration.

Level 2.3a represents front and rear roll as two elastic nodes coupled by the
chassis torsional stiffness. A rigid chassis recovers the axle-stiffness ratio;
zero coupling recovers the locally applied roll moments. The initial laptime
sweep uses Level 2.0 so several chassis concepts remain quick to compare.
Level 2.3b builds the more expensive slip map for one explicitly selected
stiffness and compares it with the rigid 2.2b map. The map records chassis
stiffness and is rejected when paired with another chassis configuration.
The nonlinear operating-point solver retries from neutral wheel-slip states
when its fast initial solution does not close the normalized equations.
Bulk map searches use the single-start mode and retain the explicit map safety
factor; final profile validation uses the robust retry mode.

Level 2.4a post-processes the converged speed profile. It balances kinetic
energy, aerodynamic drag, rolling resistance and auxiliaries, then applies
separate traction and recuperation efficiencies. Regeneration is bounded by
per-motor torque and power, total system power, battery charge power, minimum
speed and a configurable maximum regenerative deceleration. Event repetition
and timing windows use the same configuration as the event time.

Accumulator targets include the usable SoC window, finish reserve, end-of-life
capacity retention, nominal voltage and pack specific energy. The model reports
when its calculated discharge power exceeds the configured battery limit. That
flag is diagnostic in 2.4a; closing the power limit back into the laptime model
belongs to the next energy level.

Level 2.4b advances SoC segment by segment using the configured installed
capacity and starting SoC. Independent curves scale drive power, battery charge
power, regenerative motor torque and regenerative motor power. The laptime
model evaluates its drive derating at a selected SoC design point; the energy
post-processor uses the changing SoC along the complete event. Component
efficiencies use either the default constants or interpolated motor, inverter
and battery maps.

Level 2.4c expands repeated closed events into a continuous distance axis. Its
forward pass advances SoC and limits acceleration with locally available
battery power; its backward pass retains the model's full braking envelope,
with friction braking supplying the part that cannot be recuperated. The two
passes repeat until the speed profile converges. The solver consumes the same
vehicle-model interface and therefore works with the aero point mass, single
track, dual-track envelope and mapped dual-track slip models.

Level 2.4d keeps the previous fixed axle splits as the default and adds an
optional ideal wheel-force allocator. It respects driven wheels and shared or
independent motor groups, motor torque and power, tire capacity, total force
and yaw demand. Regeneration can likewise use the ideal upper bound or a fixed
utilization factor. Slip maps record the allocation configuration and are
rejected when paired with a different controller setup.

Level 2.5 keeps the original fitted-curve/friction-ellipse model as the
`fittedCombinedSlip` reference and adds `tmeasyAdtm`. The latter is a local,
versioned copy of ADTM TMeasy V2.5 evaluated without transient tire-deflection
states. A common SI interface supplies normal load, camber, slip ratio, slip
angle and speed. Performance maps store both model and parameter-set identity.
The initial Hoosier parameter set combines lateral and longitudinal data from
different tire sizes and is therefore marked as a development reference.
# Hinweis zur öffentlichen V2-Schnittstelle

Die Level in diesem Dokument beschreiben weiterhin die vorhandenen validierten
Rechenkerne. Für neue Studien werden sie nicht mehr direkt ausgewählt. Die
transparente V2 bietet ausschließlich `pointMass`, `singleTrack` und
`dualTrack` plus explizite Feature-Flags an und legt die tatsächlich verwendete
Level-Implementierung offen. Siehe `transparent-v2.md`.

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
