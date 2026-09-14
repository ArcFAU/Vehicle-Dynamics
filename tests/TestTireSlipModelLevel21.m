classdef TestTireSlipModelLevel21 < matlab.unittest.TestCase
    methods (Test)
        function identifiedFitLoadsWithNamedMetadata(testCase)
            fit = developmentFit();
            testCase.verifySize(fit.coefficients, [13, 18]);
            testCase.verifyEqual(fit.lateralScale, 0.48);
            testCase.verifyEqual(fit.longitudinalScale, 0.50);
        end

        function zeroSlipProducesZeroForce(testCase)
            forces = lts.tire.pureSlipForces(developmentFit(), 600, 0, 0, 0);
            testCase.verifyEqual(forces.longitudinal_N, 0, 'AbsTol', 1e-12);
            testCase.verifyEqual(forces.lateral_N, 0, 'AbsTol', 1e-12);
        end

        function statedPeakSlipsProducePeakForces(testCase)
            fit = developmentFit();
            parameters = lts.tire.magicFormulaParameters(fit, 600, 0);
            forces = lts.tire.pureSlipForces(fit, 600, 0, ...
                parameters.longitudinal.peakSlip, parameters.lateral.peakSlip);
            testCase.verifyEqual(forces.longitudinal_N, ...
                parameters.longitudinal.D_N, 'RelTol', 1e-10);
            testCase.verifyEqual(forces.lateral_N, ...
                parameters.lateral.D_N, 'RelTol', 1e-10);
        end

        function forceSignsFollowSlipSigns(testCase)
            fit = developmentFit();
            positive = lts.tire.pureSlipForces(fit, 600, 0, 0.04, deg2rad(3));
            negative = lts.tire.pureSlipForces(fit, 600, 0, -0.04, deg2rad(-3));
            testCase.verifyGreaterThan(positive.longitudinal_N, 0);
            testCase.verifyGreaterThan(positive.lateral_N, 0);
            testCase.verifyEqual(negative.longitudinal_N, ...
                -positive.longitudinal_N, 'RelTol', 1e-12);
            testCase.verifyEqual(negative.lateral_N, ...
                -positive.lateral_N, 'RelTol', 1e-12);
        end

        function combinedSlipStaysInsideEllipse(testCase)
            fit = developmentFit();
            parameters = lts.tire.magicFormulaParameters(fit, 600, 0);
            forces = lts.tire.combinedSlipForces(fit, 600, 0, ...
                parameters.longitudinal.peakSlip, parameters.lateral.peakSlip);
            utilization = hypot(forces.longitudinal_N ...
                / abs(parameters.longitudinal.D_N), forces.lateral_N ...
                / abs(parameters.lateral.D_N));
            testCase.verifyEqual(utilization, 1, 'AbsTol', 1e-12);
            testCase.verifyGreaterThan(forces.unconstrainedUtilization, 1);
        end
    end
end

function fit = developmentFit()
vehicle = FAUmaxTauV2();
configuration = vehicle.tireSlipModel.developmentFit;
fit = lts.tire.loadFittedMagicFormula(string(configuration.file), ...
    configuration.lateralScale, configuration.longitudinalScale);
end
