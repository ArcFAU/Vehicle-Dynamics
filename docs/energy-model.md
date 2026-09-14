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
installed capacity = positive net event energy
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

The supplied vehicle currently selects constant efficiencies. No measured map
has been added yet. Cell voltage sag, current-dependent cell losses,
temperature, cooling power, track gradient and wind remain outside the model.
The fast Level 2.4b mode uses a chosen SoC derating point and reports violations
along the changing SoC trace. Optional Level 2.4c recomputes the speed profile
with those local discharge-power limits.

## Optional coupled solver

Set `caseConfig.coupleEnergy = true` before `runSensitivityStudy` or
`runEnergyReport`. Full-track events with an integer repetition count then use
`lts.solver.solveLapWithEnergy`. Timing-window events continue to use the fast
post-processor because their surrounding untimed geometry is required for a
well-defined coupled speed state.

The coupled solver uses the same `maximumSpeedForCurvature` and
`accelerationLimits` functions as all existing models. It has been tested with
the Level 1.2 single-track model and is equally available to the point-mass and
dual-track adapters. Total deceleration remains available from regenerative and
friction brakes; the energy result records their split.

Run `runEnergyReport` for the selected baseline. Run
`runPowertrainEnergyComparison` to compare the six configured drivetrain
layouts. Regenerative torque and power scale with motor count and remain capped
by the common system and accumulator limits.
