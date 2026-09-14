function forces = combinedSlipForces(fit, normalLoad_N, camberAngle_rad, slipRatio, slipAngle_rad)
%COMBINEDSLIPFORCES Pure-slip curves coupled by a friction ellipse.

forces = lts.tire.pureSlipForces(fit, normalLoad_N, camberAngle_rad, ...
    slipRatio, slipAngle_rad);
longitudinalPeak_N = abs(forces.parameters.longitudinal.D_N);
lateralPeak_N = abs(forces.parameters.lateral.D_N);
rawUtilization = hypot(forces.longitudinal_N / longitudinalPeak_N, ...
    forces.lateral_N / lateralPeak_N);
scale = max(1, rawUtilization);
forces.longitudinal_N = forces.longitudinal_N / scale;
forces.lateral_N = forces.lateral_N / scale;
forces.utilization = min(rawUtilization, 1);
forces.unconstrainedUtilization = rawUtilization;
end
