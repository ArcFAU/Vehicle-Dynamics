function coupling = rollCoupling(vehicle)
%ROLLCOUPLING Solve front/rear roll nodes coupled by chassis torsion.
% A unit total roll moment is applied according to static axle mass fraction.

lts.validation.validateVehicle(vehicle);
roll = lts.suspension.rollProperties(vehicle);
frontAppliedFraction = vehicle.geometry.frontMassFraction;
rearAppliedFraction = 1 - frontAppliedFraction;
frontStiffness = roll.frontAxleRollStiffness_NmPerRad;
rearStiffness = roll.rearAxleRollStiffness_NmPerRad;
chassisStiffness = vehicle.chassis.torsionalStiffness_NmPerRad;

if isinf(chassisStiffness)
    frontReactionFraction = roll.frontRollMomentFraction;
    rearReactionFraction = roll.rearRollMomentFraction;
    frontRotationPerMoment_radPerNm = 1 / roll.totalRollStiffness_NmPerRad;
    rearRotationPerMoment_radPerNm = frontRotationPerMoment_radPerNm;
else
    stiffnessMatrix = [frontStiffness + chassisStiffness, -chassisStiffness; ...
        -chassisStiffness, rearStiffness + chassisStiffness];
    rotationPerMoment = stiffnessMatrix ...
        \ [frontAppliedFraction; rearAppliedFraction];
    frontRotationPerMoment_radPerNm = rotationPerMoment(1);
    rearRotationPerMoment_radPerNm = rotationPerMoment(2);
    frontReactionFraction = frontStiffness ...
        * frontRotationPerMoment_radPerNm;
    rearReactionFraction = rearStiffness ...
        * rearRotationPerMoment_radPerNm;
end

coupling.frontAxleRollStiffness_NmPerRad = frontStiffness;
coupling.rearAxleRollStiffness_NmPerRad = rearStiffness;
coupling.chassisTorsionalStiffness_NmPerRad = chassisStiffness;
coupling.frontAppliedMomentFraction = frontAppliedFraction;
coupling.rearAppliedMomentFraction = rearAppliedFraction;
coupling.frontRollMomentFraction = frontReactionFraction;
coupling.rearRollMomentFraction = rearReactionFraction;
coupling.frontRotationPerMoment_radPerNm = ...
    frontRotationPerMoment_radPerNm;
coupling.rearRotationPerMoment_radPerNm = ...
    rearRotationPerMoment_radPerNm;
coupling.chassisTwistPerMoment_radPerNm = ...
    frontRotationPerMoment_radPerNm - rearRotationPerMoment_radPerNm;
coupling.chassisTorquePerMoment = ...
    frontAppliedFraction - frontReactionFraction;
coupling.equilibriumResidual = ...
    frontReactionFraction + rearReactionFraction - 1;
end
