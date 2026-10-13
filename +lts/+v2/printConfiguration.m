function printConfiguration(config, report)
%PRINTCONFIGURATION Print the complete selected and effective V2 state.

fprintf('\n=== Effective Laptime Simulation V2 configuration ===\n');
fprintf('Model: %s\n', config.model);
fprintf('Solver strategy: %s\n', config.solver.strategy);
disp(report.features);
fprintf('Legacy calculation kernel: %s\n', config.legacyModelId);
fprintf('Calculation strategy: %s\n', config.legacyStrategy);
fprintf('Used: %d | Ignored: %d | Not supported: %d\n', ...
    nnz(report.inputs.status == "used"), ...
    nnz(report.inputs.status == "ignored"), ...
    nnz(report.inputs.status == "notSupported"));
disp(report.inputs);
fprintf('Effective vehicle structure:\n');
disp(report.effectiveVehicle);
end
