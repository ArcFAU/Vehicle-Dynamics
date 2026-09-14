function forces = evaluate(fit, normalLoad_N, camberAngle_rad, ...
        slipRatio, slipAngle_rad)
%EVALUATE Evaluate the sourced fitted-curve combined-slip model.

forces = lts.tire.combinedSlipForces(fit, normalLoad_N, ...
    camberAngle_rad, slipRatio, slipAngle_rad);
forces.aligningMoment_Nm = 0;
forces.overturningMoment_Nm = 0;
forces.rollingResistanceMoment_Nm = 0;
forces.dynamicRadius_m = NaN;
forces.slipPower_W = NaN;
end
