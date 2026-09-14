function capabilities = capabilityMatrix()
%CAPABILITYMATRIX Document which model level supports which design claim.

topic = ["mass"; "massDistribution"; "power"; "gearRatio"; ...
    "energy"; "brakeDemand"; "regeneration"; "tireSelection"; ...
    "aerodynamics"; "torqueVectoring"; "rollStiffness"; ...
    "chassisTorsion"; "steeringAndBodySlip"];
minimumLevel = ["0.2"; "1.1"; "0.3"; "0.3"; "0.3+2.4"; ...
    "0.3"; "0.3+2.4"; "0.2"; "0.3"; "2.0"; "2.2"; ...
    "2.3"; "1.3"];
recommendedLevel = ["0.3"; "1.2"; "0.3"; "0.3"; "1.2+2.4"; ...
    "1.2"; "1.2+2.4"; "1.3"; "1.2"; "2.1"; "2.2"; ...
    "2.3"; "1.3"];
notes = [
    "Mass needs load sensitivity, power or aero to affect lap time.";
    "Point mass cannot represent front/rear mass distribution.";
    "Includes system power, motor torque and speed limits.";
    "Uses motor speed, effective radius and gearbox efficiency.";
    "Post-process converged mechanical work; coupled SoC is optional.";
    "Demand output, not an imposed mechanical brake limit.";
    "Includes motor, inverter, battery and SoC-dependent limits.";
    "Level 1.3/2.1 use the complete selected stationary tire model.";
    "Includes drag, axle downforce distribution and application height.";
    "Requires individual wheel forces and yaw-moment balance.";
    "Requires lateral load transfer and axle roll distribution.";
    "Requires coupled front/rear roll nodes.";
    "Requires the slip-consistent single-track operating point."];
capabilities = table(topic, minimumLevel, recommendedLevel, notes);
end
