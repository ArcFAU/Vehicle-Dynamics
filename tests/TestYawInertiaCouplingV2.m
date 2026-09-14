classdef TestYawInertiaCouplingV2 < matlab.unittest.TestCase
    methods (Test)
        function nonlinearPointEnforcesTargetYawMoment(testCase)
            vehicle = FAUmaxTauV2();
            targetYawMoment_Nm = 40;
            point = lts.model.dualTrackSlip.operatingPoint(vehicle, ...
                15, 0, 4, targetYawMoment_Nm=targetYawMoment_Nm);

            testCase.verifyTrue(point.feasible);
            testCase.verifyEqual(point.targetYawMoment_Nm, ...
                targetYawMoment_Nm);
            testCase.verifyEqual(point.generatedYawMoment_Nm, ...
                targetYawMoment_Nm, 'AbsTol', 1e-3);
            testCase.verifyLessThan(abs(point.residuals.yawMoment_Nm), 1e-3);
        end

        function previousSolutionCanBeUsedAsWarmStart(testCase)
            vehicle = FAUmaxTauV2();
            first = lts.model.dualTrackSlip.operatingPoint(vehicle, ...
                15, 0, 4, targetYawMoment_Nm=30);
            second = lts.model.dualTrackSlip.operatingPoint(vehicle, ...
                15.1, 0, 4.05, targetYawMoment_Nm=31, ...
                initialUnknowns=first.solver.solution);

            testCase.verifyTrue(second.feasible);
            testCase.verifySize(second.solver.solution, [6, 1]);
        end

        function yawDemandRecordsCouplingMode(testCase)
            vehicle = FAUmaxTauV2();
            lap.distance_m = [0; 10; 20; 30];
            lap.time_s = [0; 1; 2; 3];
            lap.speed_mps = [10; 10; 10; 10];
            lap.curvature_1pm = [0; 0.1; 0.1; 0];
            demand = lts.analysis.yawDemand(lap, vehicle, ...
                CouplingMode="criticalPointAudit");

            testCase.verifyEqual(demand.couplingMode, ...
                'criticalPointAudit');
            testCase.verifyGreaterThan(demand.maximumAbsoluteMoment_Nm, 0);
        end
    end
end
