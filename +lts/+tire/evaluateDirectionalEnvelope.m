function peaks = evaluateDirectionalEnvelope(envelope,normalLoad_N)
%EVALUATEDIRECTIONALENVELOPE Evaluate drive, brake and lateral peaks.

arguments
    envelope (1,1) struct
    normalLoad_N double {mustBeNonnegative}
end
assert(strcmp(envelope.type,'quadraticDirectional'), ...
    'lts:tire:EnvelopeType','Expected a quadratic directional envelope.');
peaks.drive_N = evaluateOne(envelope.quadratic.drive,normalLoad_N);
peaks.brake_N = evaluateOne(envelope.quadratic.brake,normalLoad_N);
peaks.lateral_N = evaluateOne(envelope.quadratic.lateral,normalLoad_N);
peaks.longitudinal_N = min(peaks.drive_N,peaks.brake_N);
end

function force_N = evaluateOne(coefficients,normalLoad_N)
force_N = max(0,coefficients.mu0.*normalLoad_N ...
    - coefficients.degression_Ninv.*normalLoad_N.^2);
force_N(normalLoad_N==0)=0;
end
