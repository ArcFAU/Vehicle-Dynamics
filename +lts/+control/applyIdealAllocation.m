function vehicle = applyIdealAllocation(vehicle)
%APPLYIDEALALLOCATION Enable ideal drive and brake torque allocation.

vehicle.controls.driveAllocation.mode = 'ideal';
vehicle.controls.brakeAllocation.mode = 'ideal';
vehicle.controls.regenAllocation.mode = 'ideal';
vehicle.powertrain.leftRightTorqueAllocation = 'optimized';
end
