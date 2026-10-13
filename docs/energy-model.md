# Level 2.4 EV energy model

Level 2.4a evaluates the converged speed profile on a flat track. For every
segment it calculates the wheel force from longitudinal acceleration,
aerodynamic drag and rolling resistance. Positive wheel work passes through
gearbox, motor, inverter and battery-discharge efficiencies. Negative wheel
work is the theoretical braking-energy source.

Actual recuperation is the minimum allowed by:

- braking energy available in the segment;
- motor-count-dependent regenerative torque and power;
- total regenerative system power;
- accumulator charge power;
- maximum regenerative vehicle deceleration;
- minimum regenerative speed.

The difference between braking work and accepted mechanical regeneration is
reported as friction-brake energy. Auxiliary power is integrated over event
time and recovered electrical energy is subtracted from battery consumption.

Accumulator capacity is a concept target:

```text
installed capacity = maximum cumulative energy depletion
                     / usable SoC window
                     / (1 - finish reserve)
                     / end-of-life capacity retention
```

Nominal voltage converts this value to Ah. Pack specific energy gives a first
mass estimate. These inputs are grouped in `vehicle.energy` in
`vehicles/FAUmaxTauV2.m` and are explicitly initial assumptions.

## SoC-dependent limits

Level 2.4b advances SoC after every selected track segment and every repeated
lap. The following inputs are independent:

- `powertrain.driveLimitModel`: drive-power scale and its laptime evaluation
  SoC;
- `energy.socLimitModel.chargePowerScale`: accumulator charge acceptance;
- `energy.socLimitModel.regenTorqueScale`: regenerative motor torque;
- `energy.socLimitModel.regenPowerScale`: regenerative motor power.

All curves have explicit SoC breakpoints and use clamped linear interpolation.
`energy.initialSoc` and `energy.installedCapacity_kWh` determine the SoC trace.
Use `lts.energy.applyInitialSoc` in a parameter sweep when the initial SoC and
the laptime drive-limit evaluation point should change together.

## Constant factors and component maps

`energy.efficiencyModel.mode` selects `constant` or `map`. Constant mode uses
the readable motor, regenerative motor, inverter and battery factors directly
above it in the vehicle file. Map mode expects:

- motor efficiency over motor speed and absolute motor torque;
- inverter efficiency over power and SoC;
- battery charge/discharge efficiency over power and SoC.

Each map stores its axes and separate drive/regen value matrices. Inputs are
clamped to the available map range. This lets measured maps replace the initial
constants without changing the event solver or energy-result schema.

The supplied general vehicle selects constant efficiencies. The FSG study
selects the local AMK 100 C loss map explicitly. Cell voltage sag, current-dependent cell losses,
temperature, cooling power, track gradient and wind remain outside the model.
The fast Level 2.4b mode uses a chosen SoC derating point and reports violations
along the changing SoC trace. Optional Level 2.4c recomputes the speed profile
with those local discharge-power limits.

## Optional coupled solver

Set `caseConfig.coupleEnergy = true` before `runSensitivityStudyV2` or
`runEnergyReportV2`. Full-track events with an integer repetition count then use
`lts.solver.solveLapWithEnergy`. Timing-window events do not support coupled energy. An explicit coupling
request is rejected rather than silently replaced with post-processing.

The coupled solver uses the same `maximumSpeedForCurvature` and
`accelerationLimits` functions as all existing models. It has been tested with
the Level 1.2 single-track model and is equally available to the point-mass and
dual-track adapters. Total deceleration remains available from regenerative and
friction brakes; the energy result records their split.

Run `runEnergyReportV2` for the selected baseline. Run
`runPowertrainEnergyComparisonV2` to compare the six configured drivetrain
layouts. Regenerative torque and power scale with motor count and remain capped
by the common system and accumulator limits.


## Shared motor modules and energy boundaries

All scenarios use `lts.energy.conversionEfficiencies`, `propagateSegment`,
`analyzeLap` and `profileTable`, plus `lts.solver.solveLapWithEnergy` when coupled.
There are no independent V13/V14 kernels. The study runner name and historical
result folders retain their provenance labels.

`energy.motorModel.mode` selects `constant`, `efficiencyMap` or `lossMap`.
`energy.motorModel.map` contains the selected module's input. An efficiency map
requires `speed_rpm`, `torque_Nm`, `driveEfficiency`, `regenEfficiency`. A loss map
requires the same axes and nonnegative `loss_W`; optional `maximumTorque_Nm`
defines the speed-dependent feasible shaft envelope. The AMK importer supplies
that envelope using the fixed temperature, voltage and current assumptions.
Do not enable a component motor map and an independent motor map simultaneously.

`lts.powertrain.configureVehicle` configures the FSG concept assumptions. Its
default module is `lossMap`; use `MotorModel="constant"` for a constant comparison
or `MotorModel="efficiencyMap", MotorEfficiencyMap=map` for explicit efficiencies.
General simulations can set the same module fields directly in their vehicle.

`energy.useSocLimits` enables SoC derating without changing the integrator.
SoC and charge headroom remain tracked when derating is disabled.
`maximumBatteryChargePower_W` accepts zero (no charge) and Inf (no power cap).
An Inf limit multiplied by a zero SoC gate yields zero, not NaN.

`energy.measurementBoundary` is `storedEnergy` for the general vehicle or
`dcTerminals` for the FSG concept. The latter excludes internal battery losses,
as in the historical logger studies, and uses terminal energy as a capacity
reference. SoC follows the selected boundary and its configured capacity;
this remains a quasi-static approximation. Terminal traction, regeneration
and power are additionally reported in both modes. Battery power limits and
nominal-voltage current always refer to DC terminals.

Motor-shaft torque is wheel torque divided by gearbox efficiency when driving
and multiplied by efficiency when regenerating. Regeneration efficiencies are
evaluated at actual limited regenerative torque, including charge headroom;
friction brakes absorb the remainder. The capacity demand is the largest
cumulative energy depletion, followed by explicit window, reserve and aging
factors. A later recovery does not remove earlier required capacity.

When generator losses exceed shaft input and electrical recovery is zero,
friction braking handles the demand. Active motor braking that consumes
electrical power is outside this recuperation model.

FSG credit is configured at `rules.events(...).reference.regenEnergyCredit`.
`lts.scoring.energyForEfficiency` applies it to terminal discharge plus auxiliary
energy and recovered energy. Physics results contain no scoring energy.
`profileTable` exports physical powers; the study appends its scoring column.

Historical frozen inputs are translated by
`lts.compatibility.normalizeEnergyModel` at the load boundary. They remain
unchanged on disk. Old checkpoints cannot be resumed as new simulations;
the modular study uses a new output tag and checkpoint identity.

`lts.analysis.auditLap` and `auditFsg2026PowertrainLimitsV14` check saved profiles.
Midpoint tire-envelope violations are diagnostic failures. Energy balance tests
do not replace tire-envelope or nonlinear slip validation. Regeneration uses
mirrored positive-shaft AMK losses, not measured generator data.
