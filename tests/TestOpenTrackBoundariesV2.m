classdef TestOpenTrackBoundariesV2 < matlab.unittest.TestCase
    methods (Test)
        function openTrackDoesNotForceStopAtFinish(testCase)
            vehicle = FAUmaxTauV2();
            track.distance_m = (0:1:75)';
            track.curvature_1pm = zeros(size(track.distance_m));
            track.isClosed = false;
            config = lts.config.createCase(Vehicle=vehicle, ...
                Model="pointMassAero", ...
                Strategy="direct", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=Inf);
            result = lts.design.runCase(config, track);
            testCase.verifyEqual(result.lap.speed_mps(1), 0, AbsTol=1e-12);
            testCase.verifyGreaterThan(result.lap.speed_mps(end), 1);
        end

        function explicitFinishSpeedIsPropagated(testCase)
            vehicle = FAUmaxTauV2();
            track.distance_m = (0:2:40)';
            track.curvature_1pm = zeros(size(track.distance_m));
            track.isClosed = false;
            config = lts.config.createCase(Vehicle=vehicle, ...
                Model="pointMassAero", ...
                Strategy="direct", IncludeEnergy=false, ...
                IncludeYawInertia=false, MaximumSpeed_mps=100, ...
                InitialSpeed_mps=0, FinalSpeed_mps=5);
            result = lts.design.runCase(config, track);
            testCase.verifyLessThanOrEqual(result.lap.speed_mps(end), 5 + 1e-12);
        end
    end
end
