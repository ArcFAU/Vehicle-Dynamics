classdef TestSingleTrackAeroLevel12 < matlab.unittest.TestCase
    methods (Test)
        function operatingPointClosesEquilibria(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrackAero.create(vehicle);
            point = model.operatingPoint(15, 1.0, 5.0);

            testCase.verifyEqual(struct2array(point.residuals), zeros(1, 3), ...
                'AbsTol', 1e-10);
        end

        function downforceRaisesTotalNormalLoad(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrackAero.create(vehicle);
            point = model.operatingPoint(20, 0, 0);
            expected_N = vehicle.mass.total_kg * 9.81 ...
                + point.aero.totalDownforce_N;

            actual_N = point.frontNormalLoad_N + point.rearNormalLoad_N;
            testCase.verifyEqual(actual_N, expected_N, 'AbsTol', 1e-10);
        end

        function dragCompensationUsesDrivenAxle(testCase)
            vehicle = FAUmaxTauV2("dualRearMotors");
            model = lts.model.singleTrackAero.create(vehicle);
            point = model.operatingPoint(20, 0, 0);

            testCase.verifyEqual(point.frontLongitudinalForce_N, 0);
            testCase.verifyEqual(point.rearLongitudinalForce_N, ...
                point.aero.drag_N, 'AbsTol', 1e-12);
        end

        function leftAndRightLimitsAreSymmetric(testCase)
            vehicle = FAUmaxTauV2();
            model = lts.model.singleTrackAero.create(vehicle);
            left = model.accelerationLimits(18, 5);
            right = model.accelerationLimits(18, -5);

            testCase.verifyEqual(left.maximumAcceleration_mps2, ...
                right.maximumAcceleration_mps2, 'AbsTol', 1e-10);
            testCase.verifyEqual(left.maximumDeceleration_mps2, ...
                right.maximumDeceleration_mps2, 'AbsTol', 1e-10);
        end

        function unloadedAxleIsInfeasibleWithoutTireModelError(testCase)
            vehicle = FAUmaxTauV2();
            peakEnvelope = lts.tire.createPeakEnvelope(vehicle);

            accelerating = lts.model.singleTrackAero.calculateState( ...
                vehicle, 20, 5 * 9.81, 0, peakEnvelope);
            braking = lts.model.singleTrackAero.calculateState( ...
                vehicle, 20, -5 * 9.81, 0, peakEnvelope);

            testCase.verifyFalse(accelerating.feasible);
            testCase.verifyFalse(braking.feasible);
            testCase.verifyTrue(accelerating.frontNormalLoad_N < 0);
            testCase.verifyTrue(braking.rearNormalLoad_N < 0);
        end

        function operatingPointTraceUsesNamedLevel12States(testCase)
            vehicle = FAUmaxTauV2();
            track.meta.name = 'Level 1.2 trace';
            track.distance_m = (0:5:30).';
            track.curvature_1pm = [0; 0.01; 0.02; 0.02; 0.01; 0; 0];
            track.isClosed = true;
            config = lts.config.createCase(Vehicle=vehicle, ...
                Model="singleTrackAero", ...
                Strategy="direct", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            result = lts.design.runCase(config, track);
            trace = lts.analysis.operatingPointTrace(result, vehicle, ...
                PointCount=3);

            testCase.verifyEqual(numel(trace.points), 3);
            testCase.verifyTrue(trace.allConfirmedFeasible);
            testCase.verifyTrue(isfield(trace.points{1}, ...
                'frontUtilization'));
        end
    end
end
