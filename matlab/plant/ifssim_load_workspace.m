function P = ifssim_load_workspace()
%IFSSIM_LOAD_WORKSPACE  Put parameters and buses where Simulink can see them.
%
%   Call before building, compiling or simulating any plant model.
%
%   Simulink resolves data types and block parameters from the BASE workspace,
%   so everything a model needs has to be there first. Two forms are provided,
%   for two different consumers:
%
%     IFSSIM_P            the full struct, with provenance and derived values.
%                         Use it in mask expressions and block parameters.
%
%     IFSSIM_Mass, ...    flat scalars. MATLAB Function blocks resolve workspace
%                         variables as tunable PARAMETERS, and that resolution
%                         does not reach into nested structs — 'IFSSIM_P.Assumed.Izz'
%                         inside a MATLAB Function block fails to determine its
%                         type at compile time. The flat names are the workaround.
%
%   These are NOT a second copy of the numbers. They are assigned here from
%   ifssim_params(), which reads settings.json, so there is still exactly one
%   source and it is still the file. Do not hand-edit them.

P = ifssim_params();
assignin('base','IFSSIM_P', P);
ifssim_plant_buses();

flat = struct( ...
    'IFSSIM_Ts',    1/960, ...            % s, the plant's fixed step
    'IFSSIM_Mass',  P.Mass, ...           % kg
    'IFSSIM_Ixx',   P.Assumed.Ixx, ...    % kg*m^2  ASSUMPTION
    'IFSSIM_Iyy',   P.Assumed.Iyy, ...
    'IFSSIM_Izz',   P.Assumed.Izz, ...
    'IFSSIM_CoGH',  P.Assumed.CoGHeightUsed);   % m

f = fieldnames(flat);
for i = 1:numel(f), assignin('base', f{i}, flat.(f{i})); end

% Zeroed bus structs, referenced by name from the placeholder Constant blocks.
for b = {'IFSSIM_PoseBus','IFSSIM_WheelsBus','IFSSIM_PowertrainBus','IFSSIM_StatusBus'}
    assignin('base',[b{1} '_zero'], Simulink.Bus.createMATLABStruct(b{1}));
end
end
