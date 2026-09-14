function feasible = feasibleMask(vehicle, speed_mps, longitudinalAcceleration_mps2, lateralAcceleration_mps2, peakEnvelope)
%FEASIBLEMASK Fast Level 1.2 feasibility check for limit searches.

if nargin < 5
    peakEnvelope = lts.tire.createPeakEnvelope(vehicle);
end

state = lts.model.singleTrackAero.calculateState(vehicle, speed_mps, ...
    longitudinalAcceleration_mps2, lateralAcceleration_mps2, peakEnvelope);
feasible = state.feasible;
end
