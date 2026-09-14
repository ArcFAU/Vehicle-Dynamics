function track = loadTrackMatrix(filename, isClosed)
%LOADTRACKMATRIX Import distance and curvature from a numeric track matrix.
% The original MAT file remains unchanged and is recorded as source data.

arguments
    filename (1, 1) string
    isClosed (1, 1) logical = true
end

loaded = load(filename, 'A');
assert(isfield(loaded, 'A'), 'lts:io:TrackMatrixSchema', ...
    'The track file must contain a variable named A.');
assert(isnumeric(loaded.A) && size(loaded.A, 2) >= 2, ...
    'lts:io:TrackMatrixSchema', ...
    'A must be a numeric matrix with distance and curvature columns.');

track.meta.name = char(erase(string(filename), fileparts(filename) + filesep));
track.meta.source = char(filename);
track.meta.importer = 'lts.io.loadTrackMatrix';
track.distance_m = loaded.A(:, 1) - loaded.A(1, 1);
track.curvature_1pm = loaded.A(:, 2);
track.isClosed = isClosed;
lts.validation.validateTrack(track);
end
