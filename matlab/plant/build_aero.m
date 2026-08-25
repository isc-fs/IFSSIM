function build_aero(outdir)
%BUILD_AERO  Fill in IFSSIM_Aero: drag and downforce, in the BODY frame.
%
%   Two things the UE plant gets wrong and this gets right:
%
%   1. DOWNFORCE ACTS NORMAL TO THE FLOOR, not along world -Z. A wing makes its
%      force perpendicular to the car, so when the car rolls or pitches the
%      force rolls and pitches with it. Applying it along world -Z means a car
%      at 5 degrees of roll gets its downforce in the wrong direction by
%      5 degrees, which shows up as a lateral load-transfer error exactly when
%      the car is cornering hard and downforce matters most.
%
%   2. THE MOMENT ARM COMES FROM THE WHEELBASE. In the UE plant it was a literal
%      813.f labelled "cm" against a comment saying 813 mm — a 10x error in the
%      aero pitch couple that survived because total downforce was unaffected.
%      Here front and rear application points are aF and -bR from IFSSIM_P.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
P = ifssim_load_workspace();

name = 'IFSSIM_Aero';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','inf');

add_block('simulink/Sources/In1',[name '/Pose'],'Position',[30 60 60 80], ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus','BusOutputAsStruct','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/Pose Select'], ...
          'Position',[130 50 140 100],'OutputSignals','vel_body');
add_line(name,'Pose/1','Pose Select/1','autorouting','on');

fcn = [name '/Drag and Downforce'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
          'Position',[240 40 440 140]);
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',fcn);
chart.Script = aero_code();

sizes = struct('velb',3,'aero_force',3,'aero_torque',3);
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes,d.Name), d.Props.Array.Size = num2str(sizes.(d.Name)); end
end
params = {'IFSSIM_rho','IFSSIM_CdA','IFSSIM_ClA','IFSSIM_abal', ...
          'IFSSIM_aF','IFSSIM_bR','IFSSIM_hcop'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing,params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name = params{k}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end

add_block('simulink/Sinks/Out1',[name '/aero_force'],'Position',[520 50 550 70], ...
          'PortDimensions','3');
add_block('simulink/Sinks/Out1',[name '/aero_torque'],'Position',[520 110 550 130], ...
          'PortDimensions','3');
FB = 'Drag and Downforce';
add_line(name,'Pose Select/1',sprintf('%s/1',FB),'autorouting','on');
add_line(name,sprintf('%s/1',FB),'aero_force/1','autorouting','on');
add_line(name,sprintf('%s/2',FB),'aero_torque/1','autorouting','on');

add_block('built-in/Note',[name '/Notes'],'Position',[40 200], ...
    'Text', sprintf([ ...
     'IFSSIM_AERO                              OWNER: aero\\n\\n' ...
     'CdA %.2f m^2, ClA %.2f m^2, balance %.0f%%%% front, rho %.3f kg/m^3.\\n' ...
     'At 20 m/s: %.0f N of downforce, %.0f N of drag.\\n\\n' ...
     'Downforce acts along BODY -Z, normal to the floor — not world -Z. A wing\\n' ...
     'makes its force perpendicular to the car, so it rolls and pitches with it.\\n' ...
     'The UE plant applies it in world Z, which is wrong by the roll angle\\n' ...
     'exactly when the car is cornering hard and downforce matters most.\\n\\n' ...
     'Front/rear application points come from the wheelbase (aF %.3f m,\\n' ...
     'bR %.3f m). In the UE plant this was a literal 813 labelled cm against a\\n' ...
     'comment saying mm — a 10x error in the pitch couple, invisible because\\n' ...
     'total downforce was unaffected.\\n\\n' ...
     'NOT MODELLED: ride-height and yaw sensitivity, and the drag pitch couple\\n' ...
     '(CoP height above CoG is set to 0 — measure it and set IFSSIM_hcop).\\n\\n' ...
     'Edit build_aero.m, not this model — it is regenerated.'], ...
     P.CdA, P.ClA, P.AeroBalanceFront*100, P.Assumed.AirDensity, ...
     P.Derived.DownforceAt20ms, P.Derived.DragAt20ms, ...
     P.Derived.aFront, P.Derived.bRear), 'HorizontalAlignment','left');

save_system(name,f);
fprintf('wrote %s\n',f);
fprintf('  CdA %.2f, ClA %.2f, balance %.0f%% front -> %.0f N down / %.0f N drag at 20 m/s\n', ...
        P.CdA, P.ClA, P.AeroBalanceFront*100, P.Derived.DownforceAt20ms, P.Derived.DragAt20ms);
close_system(name,0);
end

%% =======================================================================
function c = aero_code()
L = {
"function [aero_force, aero_torque] = aero(velb)"
"%#codegen"
"% Drag and downforce in the BODY frame."
""
"v = norm(velb);"
"q = 0.5 * IFSSIM_rho * v * v;          % dynamic pressure, Pa"
""
"% Drag opposes the airflow, so it acts along -velocity. Computing it in the"
"% body frame means a sideways-sliding car is dragged sideways too, which is"
"% what actually happens and is missed by any model that only drags along x."
"if v > 0.1"
"    drag = -q * IFSSIM_CdA * (velb / v);"
"else"
"    drag = [0; 0; 0];"
"end"
""
"% Downforce along BODY -Z: normal to the floor, so it rolls and pitches with"
"% the car rather than staying vertical in the world."
"Fz_total = q * IFSSIM_ClA;"
"Ff = Fz_total * IFSSIM_abal;           % front"
"Fr = Fz_total * (1 - IFSSIM_abal);     % rear"
""
"aero_force = drag + [0; 0; -Fz_total];"
""
"% Pitch couple from the front/rear split. Application points are the axle"
"% positions from the CoG, so the arm tracks the wheelbase in settings.json"
"% rather than being a hardcoded number in the wrong unit."
"Mf = cross([ IFSSIM_aF; 0; 0], [0; 0; -Ff]);"
"Mr = cross([-IFSSIM_bR; 0; 0], [0; 0; -Fr]);"
""
"% Drag acting above the CoG adds its own pitch couple. hcop defaults to 0 —"
"% no measurement, so no invented moment."
"Md = cross([0; 0; IFSSIM_hcop], drag);"
""
"aero_torque = Mf + Mr + Md;"
"end"
};
c = char(strjoin(string(L), newline));
end
