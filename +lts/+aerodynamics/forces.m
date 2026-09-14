function aero = forces(aerodynamics, speed_mps)
%FORCES Aerodynamic forces at zero yaw and nominal ride height.
% Drag and downforce outputs are positive magnitudes. Input lift coefficients
% follow the common convention: negative values create downforce.

validateattributes(speed_mps, {'numeric'}, {'real', 'finite', 'nonnegative'});
dynamicPressure_Pa = 0.5 * aerodynamics.airDensity_kgpm3 .* speed_mps .^ 2;
aero.drag_N = dynamicPressure_Pa .* aerodynamics.referenceArea_m2 ...
    .* aerodynamics.dragCoefficient;
aero.frontDownforce_N = -dynamicPressure_Pa .* aerodynamics.referenceArea_m2 ...
    .* aerodynamics.liftCoefficientFront;
aero.rearDownforce_N = -dynamicPressure_Pa .* aerodynamics.referenceArea_m2 ...
    .* aerodynamics.liftCoefficientRear;
aero.totalDownforce_N = aero.frontDownforce_N + aero.rearDownforce_N;
end
