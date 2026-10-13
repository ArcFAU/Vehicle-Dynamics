function file = nextResultFile(folder, baseName)
%NEXTRESULTFILE Return a new versioned MAT-file path without overwriting.
%   The first run uses BASENAME.mat; later runs append _002, _003, and so on.
arguments
    folder (1,1) string
    baseName (1,1) string
end
if ~isfolder(folder), mkdir(folder); end
file=fullfile(folder,baseName+".mat");
version=2;
while isfile(file)
    file=fullfile(folder,baseName+"_"+compose('%03d',version)+".mat");
    version=version+1;
end
fprintf('Result file: %s\n',file);
end
