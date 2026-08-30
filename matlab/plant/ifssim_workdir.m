function d = ifssim_workdir()
%IFSSIM_WORKDIR  Send Simulink's build output somewhere that is not the repo.
%
%   Simulink writes its cache (*.slxc) and generated code (slprj/) into the
%   CURRENT FOLDER, whatever that happens to be. Run anything from the repo
%   root -- which is the natural place to start -- and the root fills up with
%   eight .slxc files and an slprj tree that have nothing to do with the
%   project. They are gitignored, so they never showed up in a diff; they just
%   sat there making the repo look like a build directory.
%
%   Called by ifssim_setup and by the build entry points, so it does not
%   matter where you started MATLAB.

d = fullfile(fileparts(fileparts(mfilename('fullpath'))), 'build');
if ~isfolder(d), mkdir(d); end
try
    Simulink.fileGenControl('set', 'CacheFolder', d, 'CodeGenFolder', d, ...
                            'createDir', true);
catch e
    warning('ifssim:workdir', ...
            'could not redirect the Simulink build folder (%s); artefacts will land in %s', ...
            e.message, pwd);
end
end
