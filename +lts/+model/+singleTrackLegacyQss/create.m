function model = create(vehicle)
%CREATE Fast V2-native quasistatic reference model.
% Uses the shared single-track aero equations with explicit compatibility
% metadata; it does not depend on or execute the legacy simulator.
model = lts.model.singleTrackAero.create(vehicle);
model.name = 'Level 1.2 Legacy-compatible QSS';
model.level = '1.2';
model.executionStrategy = 'legacyQss';
model.description = ['Fast quasistatic reference path for comparison with ', ...
    'the original LaptimeSim.'];
end
