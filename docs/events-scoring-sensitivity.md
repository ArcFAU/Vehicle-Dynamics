# Events, scoring and sensitivity studies

The study layer is separate from vehicle physics and the lap solver. A study
selects vehicle parameters, event tracks, a model and a versioned scoring
basis. This keeps a change in an event result or rule year out of the vehicle
model.

## Starting a study

`config/sensitivityStudyCase.m` is the readable user-facing setup. Copy it to a
new filename for a new event or season, adjust its function name and pass the
returned struct to `runSensitivityStudy`.

```matlab
caseConfig = sensitivityStudyCase(Study="mass", ...
    Model="dualTrackEnvelope", PowertrainLayout="fourMotorAWD", ...
    TireModel="fittedCombinedSlip", IncludeDriverless=true, ...
    CoupleEnergy=true, CreatePlots=true);
study = runSensitivityStudy(caseConfig);
```

There is deliberately no no-argument or short-name execution path. Study,
model, powertrain layout, tire model, event scope, energy coupling and plot
creation must be selected explicitly before a simulation starts.

Available names are `mass`, `massYaw`, `massDistribution`, `downforce`,
`aeroBalance`, `aeroMap`, `power` and `brakeBias`. The two axes of `aeroMap`
are total lift coefficient and front aero balance; front and rear coefficients
remain coupled. One-dimensional studies create event-time, event-points,
total-points and energy plots. Two-dimensional studies create a labelled
contour plot. Set `caseConfig.createPlots = false` for batch runs.

For an idealized power study, motor torque, motor power, speed and SoC scaling
can be disabled independently through
`caseConfig.baseVehicle.powertrain.limitOptions`. The battery discharge limit
belongs to the optional coupled-energy calculation and is active only when
`caseConfig.coupleEnergy` is true.

Create a complete configuration for each requested study; do not reuse it
after changing its vehicle or physics fields without reviewing every option.

For an idealized system-power study, secondary limits can be disabled:

```matlab
caseConfig = sensitivityStudyCase(Study="power", ...
    Model="dualTrackEnvelope", PowertrainLayout="fourMotorAWD", ...
    TireModel="fittedCombinedSlip", IncludeDriverless=false, ...
    CoupleEnergy=false, CreatePlots=true);
caseConfig.baseVehicle.powertrain.limitOptions.motorTorqueEnabled = false;
caseConfig.baseVehicle.powertrain.limitOptions.motorPowerEnabled = false;
caseConfig.baseVehicle.powertrain.limitOptions.motorSpeedEnabled = false;
caseConfig.baseVehicle.powertrain.limitOptions.socPowerScaleEnabled = false;
study = runSensitivityStudy(caseConfig);
```

The battery discharge limit belongs to the optional coupled-energy calculation
and affects the lap only when `caseConfig.coupleEnergy` is true.

When an annual reference is `NaN` or an autonomous ranking list is empty, lap
times are still simulated. The total points value remains `NaN` and
`scoring.isComplete` is false, so an incomplete points basis cannot look like
a valid total.

## Selecting another event or track

An event is a named track plus timing rules. It does not contain vehicle or
scoring data. Use `lts.events.createDefinition` for every discipline. A track
MAT track files use the project importer and must contain `A` with
distance in column 1 and curvature in column 2.

```matlab
events(1) = lts.events.createDefinition( ...
    Id="acceleration", Name="Acceleration FS Austria", ...
    Discipline="acceleration", Mode="manual", ...
    TrackFile="D:/tracks/FSA_2027_acceleration.mat", ...
    IsClosed=false, InitialSpeed_mps=0, FinalSpeed_mps=100);
```

`TimingWindows_m` can select measured sections of a longer geometry.
`TimingAggregation` accepts `fullTrack`, `sumWindows` or `meanWindows`.
`TimeMultiplier` represents repeated identical laps, such as the current
project Endurance approximation.

## Updating a rule year

The scoring formula is stored in `+lts/+config/+scoring/fsg2026.m`; annual
event results are separate in the `references` struct. Create a new named
configuration when formulas or maximum points change. If only the event
results change, update the fastest manual times and autonomous competitor-time
arrays in the copied study configuration.

FSG 2026 manual dynamic events use the D9.1 time curve. Autonomous Acceleration
and Skidpad use the D9.2 ranking formula. The latter produces stepwise point
sensitivities because a simulated time changes points when it passes a
competitor time. Driverless events are present in the catalog and are included
only when `includeDriverless` is true.

Efficiency uses the FSG 2026 factor `time_s^2 * netBatteryEnergy_kWh`. Its
annual reference is `references.efficiency.minimumFactor_s2kWh`. The units of
the team and reference factor must match; kWh is used throughout nextGen.

## Parameter axes and outputs

Each `lts.study.parameter` has a readable name, nested vehicle-field path,
unit, values and baseline. Multiple axes form a full factorial study. A custom
`Apply` function can update coupled parameters. Changes to `driver_kg` or
`carWithoutDriver_kg` automatically update `total_kg`.

The active baseline tire is the retained LC0 fitted-combined-slip dataset. Its
legacy validation does not imply that it represents the R20 tire labels in the
vehicle file; that distinction remains explicit in the vehicle metadata.

The result contains every vehicle variant, every event result, event points,
the complete parameter matrix and a local points-per-unit derivative at the
selected baseline. `study.penalty.eventTime_s` reports the time loss of each
concept relative to the fastest simulated concept for every event;
`study.penalty.totalPointsLoss` provides the corresponding total-points loss
when the scoring basis is complete. This replaces the hard-coded mass and aero
loops in the old `sensitivitaet.m` and `FSG_sensitiv.m` scripts.

Energy outputs include net and gross battery energy, theoretical and actual
recuperation, friction-brake energy, charge/discharge peaks, required installed
capacity, Ah target and estimated pack mass. The study also returns local
sensitivities for event time, net energy, recovered energy and installed
capacity. `runEnergyReport` prints the baseline values for all selected events.
