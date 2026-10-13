classdef TestCleanupRegression < matlab.unittest.TestCase
    methods (Test)
        function disabledRpmLimitWorksInEveryEnvelopeFamily(test)
            vehicle=FAUmaxTauV2();
            vehicle.powertrain.limitOptions.motorSpeedEnabled=false;
            factories={@lts.model.pointMassAero.create, ...
                @lts.model.singleTrackAero.create, ...
                @lts.model.dualTrackEnvelope.create};
            for k=1:numel(factories)
                model=factories{k}(vehicle);
                speed=model.maximumSpeedForCurvature([0;.1;-.1]);
                test.verifyTrue(all(isfinite(speed) & speed>0));
                test.verifySize(speed,[3,1]);
            end
        end
        function noDragStraightIsUnboundedButCornerIsFinite(test)
            vehicle=zeroAeroVehicle();
            vehicle.powertrain.limitOptions.motorSpeedEnabled=false;
            vehicle.tireEnvelope.peakEnvelope=lts.tire.createQuadraticPeakEnvelope( ...
                DriveDegression_Ninv=0,BrakeDegression_Ninv=0,LateralDegression_Ninv=0);
            model=lts.model.pointMassAero.create(vehicle);
            speed=model.maximumSpeedForCurvature([0;.1]);
            test.verifyEqual(speed(1),Inf);
            test.verifyEqual(speed(2),sqrt(1.39*9.81/.1),'AbsTol',1e-6);
        end
        function separateBrakePeakIsUsedWithAndWithoutAxleLoads(test)
            vehicle=zeroAeroVehicle();
            vehicle.geometry.cgHeight_m=eps;
            vehicle.geometry.frontMassFraction=.5;
            vehicle.brakes.frontForceFraction=.5;
            vehicle.tireEnvelope.peakEnvelope=lts.tire.createQuadraticPeakEnvelope( ...
                DriveMu0=1,BrakeMu0=2,DriveDegression_Ninv=0, ...
                BrakeDegression_Ninv=0,LateralDegression_Ninv=0);
            for useAxleLoads=[false,true]
                model=lts.model.pointMassAero.create(vehicle,UseAxleLoads=useAxleLoads);
                limits=model.accelerationLimits(5,0);
                test.verifyEqual(limits.maximumDeceleration_mps2,2*9.81,'AbsTol',.02);
                test.verifyLessThan(limits.maximumAcceleration_mps2,limits.maximumDeceleration_mps2);
            end
        end
        function baseLapHonorsMaximumAndFinalSpeed(test)
            [track,base]=straightTrack();
            for module=["constant","lossMap"]
                [vehicle,solver]=energyModule(module);
                model=syntheticModel(vehicle);
                lap=solver(track,model,InitialSpeed_mps=0, ...
                    MaximumSpeed_mps=5,FinalSpeed_mps=3,BaseLap=base);
                test.verifyLessThanOrEqual(max(lap.speed_mps),5);
                test.verifyLessThanOrEqual(lap.speed_mps(end),3);
                test.verifyLessThan(lap.energy.powerLimitViolationEnergy_kWh,1e-8);
            end
        end
        function periodicReusePreservesMaximumSpeedAndMatchesFullSolve(test)
            track=struct('distance_m',(0:10:400).', ...
                'curvature_1pm',.02*ones(41,1),'isClosed',true);
            for module=["constant","lossMap"]
                [vehicle,solver]=energyModule(module);
                model=lts.model.singleTrackAero.create(vehicle);
                fast=solver(track,model,Repetitions=2,InitialSpeed_mps=0, ...
                    MaximumSpeed_mps=8,FinalSpeed_mps=8);
                full=solver(track,model,Repetitions=2,InitialSpeed_mps=0, ...
                    MaximumSpeed_mps=8,FinalSpeed_mps=8,ReusePeriodicLaps=false);
                test.verifyLessThanOrEqual(max(fast.speed_mps),8);
                test.verifyEqual(fast.speed_mps,full.speed_mps,'AbsTol',1e-4);
            end
        end
        function repeatedEventFinalLimitAppliesOnlyAtEventFinish(test)
            track=struct('distance_m',(0:10:400).', ...
                'curvature_1pm',.02*ones(41,1),'isClosed',true);
            for module=["constant","lossMap"]
                [vehicle,solver]=energyModule(module);
                model=lts.model.singleTrackAero.create(vehicle);
                lap=solver(track,model,Repetitions=2,InitialSpeed_mps=0, ...
                    MaximumSpeed_mps=8,FinalSpeed_mps=3);
                test.verifyLessThanOrEqual(lap.speed_mps(end),3);
                test.verifyGreaterThan(lap.speed_mps(41),3);
                test.verifyLessThanOrEqual(max(lap.speed_mps),8);
            end
        end
        function sharedAuditAcceptsFeasibleConstantSpeedProfile(test)
            vehicle=lts.powertrain.configureVehicle(FAUmaxTauV2());
            vehicle.aerodynamics.dragCoefficient=0;
            vehicle.aerodynamics.totalLiftCoefficient=0;
            vehicle.aerodynamics.liftCoefficientFront=0;
            vehicle.aerodynamics.liftCoefficientRear=0;
            vehicle=lts.control.applyIdealAllocation(vehicle);
            lap=struct('distance_m',[0;10;20],'time_s',[0;1;2], ...
                'speed_mps',[10;10;10],'curvature_1pm',[0;0;0], ...
                'segmentAcceleration_mps2',[0;0]);
            audit=lts.analysis.auditLap(lap,vehicle);
            test.verifyTrue(audit.passed);
            test.verifyEqual(audit.summary.TireUpperBoundViolated_pct,0);
            test.verifyEqual(audit.summary.BatteryDischargeViolated_pct,0);
        end
        function sharedAuditDetectsLateralOverload(test)
            vehicle=lts.powertrain.configureVehicle(zeroAeroVehicle());
            vehicle=lts.control.applyIdealAllocation(vehicle);
            vehicle.tireEnvelope.peakEnvelope=lts.tire.createQuadraticPeakEnvelope( ...
                DriveMu0=1,BrakeMu0=1,LateralMu0=1,DriveDegression_Ninv=0, ...
                BrakeDegression_Ninv=0,LateralDegression_Ninv=0);
            lap=struct('distance_m',[0;10;20],'time_s',[0;1;2], ...
                'speed_mps',[10;10;10],'curvature_1pm',[.2;.2;.2], ...
                'segmentAcceleration_mps2',[0;0]);
            audit=lts.analysis.auditLap(lap,vehicle);
            test.verifyFalse(audit.passed);
            test.verifyEqual(audit.summary.TireUpperBoundViolated_pct,100);
        end
        function referenceInputsResolveInsideThisCheckout(test)
            root=fileparts(fileparts(mfilename('fullpath')));
            reference=lts.io.loadR20Reference();
            test.verifyTrue(reference.releasePassed);
            test.verifyEqual(reference.vehicle.tireModel.fittedCombinedSlip.file, ...
                fullfile(root,'data','tires','Hoosier_10_16_8_LC0_83.mat'));
            map=lts.powertrain.importAmkMotorMap();
            test.verifyEqual(map.meta.source,string(fullfile(root,'data','powertrain','A2370DD_T100C.mat')));
        end
    end
end
function vehicle=zeroAeroVehicle()
vehicle=FAUmaxTauV2();
vehicle.aerodynamics.dragCoefficient=0;
vehicle.aerodynamics.totalLiftCoefficient=0;
vehicle.aerodynamics.liftCoefficientFront=0;
vehicle.aerodynamics.liftCoefficientRear=0;
end
function [track,base]=straightTrack()
track=struct('distance_m',(0:5:75).','curvature_1pm',zeros(16,1),'isClosed',false);
base=struct('distance_m',track.distance_m,'speed_mps',min(30,sqrt(40*track.distance_m)));
end
function model=syntheticModel(vehicle)
model=struct('vehicle',vehicle,'name','boundary regression','level','test', ...
    'accelerationLimits',@(speed,ay)struct('maximumAcceleration_mps2',20, ...
    'maximumDeceleration_mps2',20));
end
function [vehicle,solver]=energyModule(module)
vehicle=lts.powertrain.configureVehicle(FAUmaxTauV2(),MotorModel=module);
solver=@lts.solver.solveLapWithEnergy;
end
