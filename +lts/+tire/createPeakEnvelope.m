function envelope = createPeakEnvelope(vehicle)
%CREATEPEAKENVELOPE Build a fast directional peak-force tire envelope.
% The fitted model preserves the load- and camber-dependent longitudinal
% and lateral peak forces. TMeasy currently uses the transparent symmetric
% load-sensitive fallback until a dedicated peak adapter is validated.

arguments
    vehicle (1, 1) struct
end

envelope.combinedForceExponent = vehicle.tireEnvelope.combinedForceExponent;
envelope.fallback = vehicle.tireEnvelope;
if isfield(vehicle.tireEnvelope,'peakEnvelope')
    envelope = vehicle.tireEnvelope.peakEnvelope;
    return
end
switch vehicle.tireModel.active
    case 'fittedCombinedSlip'
        configuration = vehicle.tireModel.fittedCombinedSlip;
        envelope.type = 'fittedDirectionalPeak';
        envelope.name = configuration.name;
        envelope.parameters = lts.tire.loadFittedMagicFormula( ...
            string(configuration.file), configuration.lateralScale, ...
            configuration.longitudinalScale);
        envelope.classification = 'selectedModel';
    case 'tmeasyAdtm'
        assert(isfield(vehicle.tireModel, ...
            'allowFastEnvelopeApproximation') ...
            && vehicle.tireModel.allowFastEnvelopeApproximation, ...
            'lts:tire:ExplicitApproximationRequired', ...
            ['The selected TMeasy model has no validated fast peak ' ...
            'adapter. Explicitly set ' ...
            'vehicle.tireModel.allowFastEnvelopeApproximation=true to ' ...
            'use the load-sensitive symmetric approximation, or use a ' ...
            'Level 2.5 slip-map model.']);
        envelope.type = 'loadSensitiveSymmetric';
        envelope.name = 'Load-sensitive fallback for TMeasy';
        envelope.parameters = struct();
        envelope.classification = 'approximation';
    otherwise
        error('lts:tire:UnknownModel', 'Unknown tire model: %s.', ...
            vehicle.tireModel.active);
end
end
