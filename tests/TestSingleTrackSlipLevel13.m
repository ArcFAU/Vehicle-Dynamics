classdef TestSingleTrackSlipLevel13 < matlab.unittest.TestCase
    methods (Test)
        function equivalentVehicleSuppressesLateralTransfer(testCase)
            vehicle = FAUmaxTauV2();
            equivalent = lts.model.singleTrackSlip.equivalentVehicle(vehicle);
            testCase.verifyEqual(equivalent.geometry.frontTrack_m, ...
                vehicle.geometry.frontTrack_m);
            state = lts.model.dualTrackEnvelope.calculateState(...
                equivalent, 15, 0, 8);
            testCase.verifyEqual(state.normalLoads_N(1), ...
                state.normalLoads_N(2), 'AbsTol', 1e-6);
            testCase.verifyEqual(state.normalLoads_N(3), ...
                state.normalLoads_N(4), 'AbsTol', 1e-6);
        end

        function operatingPointExposesAxleSlipStates(testCase)
            point = lts.model.singleTrackSlip.operatingPoint(...
                FAUmaxTauV2(), 15, 0.5, 4);
            testCase.verifyTrue(point.feasible);
            testCase.verifySize(point.axleSlipAngles_rad, [1, 2]);
            testCase.verifySize(point.axleSlipRatios, [1, 2]);
            testCase.verifyGreaterThan(abs(point.steeringAngle_rad), 0);
        end

        function yawDemandUsesConfiguredInertia(testCase)
            vehicle = FAUmaxTauV2();
            lap.distance_m = [0; 10; 20; 30];
            lap.time_s = [0; 1; 2; 3];
            lap.speed_mps = [10; 10; 10; 10];
            lap.curvature_1pm = [0; 0.1; 0.1; 0];
            demand = lts.analysis.yawDemand(lap, vehicle);
            testCase.verifyEqual(demand.inertialMoment_Nm(2), ...
                0.5 * vehicle.mass.yawInertia_kgm2);
            testCase.verifyEqual(demand.inertialMoment_Nm(3), ...
                -0.5 * vehicle.mass.yawInertia_kgm2);
        end
    end
end
