function tracks = trackCatalog()
%TRACKCATALOG Versioned project-local event-track inputs.

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
trackRoot = fullfile(projectRoot, 'data', 'tracks');
tracks.acceleration = fullfile(trackRoot, 'A_Accel.mat');
tracks.skidpad = fullfile(trackRoot, 'SkidPad_1m.mat');
tracks.autocross = fullfile(trackRoot, 'A_FSGautox.mat');
tracks.endurance = fullfile(trackRoot, 'A_FSGend_3.mat');
end
