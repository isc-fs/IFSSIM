function T = sm_corner_sweep(axle, angles_deg)
%SM_CORNER_SWEEP  Camber, toe and track change against wheel travel.
%
%   T = SM_CORNER_SWEEP('front') sweeps the lower wishbone through its range
%   and reports what the WHEEL does. This is the first number this port
%   produces that the plant currently invents: Susp.CamberGainFront is 0.80
%   because somebody chose 0.80, and here it falls out of the hardpoints.
%
%   METHOD. The corner is assembled at a sequence of lower-arm angles and the
%   wheel frame is read at each one. Assembly, not simulation: a kinematic
%   curve is a statement about geometry, and driving the joint with prescribed
%   motion would need the input's first two derivatives and would mix the
%   mechanism's dynamics into a measurement that has none. StopTime is zero;
%   what is being read is where the linkage CAN be, not where it goes.

if nargin < 1 || isempty(axle), axle = 'front'; end
if nargin < 2 || isempty(angles_deg), angles_deg = -10:1:10; end

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','plant'), fullfile(here,'..','spec'));
P = ifssim_load_workspace();
H = sm_hardpoints(axle);
mdl = build_sm_corner(H);
load_system(mdl);

% Sense the wheel frame against the world. The upright node IS the wheel
% centre, which is why the body was built around that frame.
load_system('sm_lib'); load_system('nesl_utility');
TS = ['sm_lib/Frames and' newline 'Transforms/Transform Sensor'];
add_block(TS,[mdl '/WheelSensor'],'Position',[760 420 820 500]);
set_param([mdl '/WheelSensor'],'MeasurementFrame','World', ...
    'SenseRotationSequence','on','RotationSequence','ZYX', ...
    'SenseXYZ','on');      % the cartesian triple; not 'SenseTranslationCartesian'
add_line(mdl,'World/RConn1','WheelSensor/LConn1','autorouting','on');
add_line(mdl,'Upright/RConn1','WheelSensor/RConn1','autorouting','on');

PS2SL = ['nesl_utility/PS-Simulink' newline 'Converter'];
outs = {'rot','WheelSensor/RConn2'; 'pos','WheelSensor/RConn3'};
for k = 1:size(outs,1)
    add_block(PS2SL,[mdl '/c_' outs{k,1}],'Position',[880 400+70*k 940 430+70*k]);
    add_block('simulink/Sinks/To Workspace',[mdl '/w_' outs{k,1}], ...
        'VariableName',['LOG_' outs{k,1}],'SaveFormat','Timeseries', ...
        'Position',[1000 400+70*k 1060 430+70*k]);
    add_line(mdl,outs{k,2},['c_' outs{k,1} '/LConn1'],'autorouting','on');
    add_line(mdl,['c_' outs{k,1} '/1'],['w_' outs{k,1} '/1'],'autorouting','on');
end

set_param(mdl,'StopTime','0');
piv = [mdl '/pivot_lca'];
set_param(piv,'PositionTargetSpecify','on','PositionTargetPriority','High', ...
              'PositionTargetValueUnits','deg');

n = numel(angles_deg);
raw = NaN(n,6);
T = table('Size',[n 5],'VariableTypes',repmat({'double'},1,5), ...
          'VariableNames',{'arm_deg','travel_mm','camber_deg','toe_deg','track_mm'});
ref = [];
for i = 1:n
    set_param(piv,'PositionTargetValue', sprintf('%.10g', angles_deg(i)));
    try
        r = sim(mdl);
    catch ME
        fprintf('  angle %+5.1f deg: did not assemble (%s)\n', angles_deg(i), ...
                strrep(ME.message(1:min(60,end)),newline,' '));
        T{i,:} = NaN;  raw(i,:) = NaN(1,6);  continue
    end
    rot = squeeze(r.get('LOG_rot').Data);   rot = rot(:)';    % [z y x] radians
    pos = squeeze(r.get('LOG_pos').Data);   pos = pos(:)';    % [x y z] metres
    raw(i,:) = [pos, rot]; %#ok<AGROW>
    T.arm_deg(i) = angles_deg(i);
end
close_system(mdl,0);

% Reference the MIDDLE of the sweep, not its first point. Measuring travel
% from one extreme makes the whole curve one-sided -- it reads as 106 mm of
% pure droop and never crosses zero, which hides whether the car gains or
% loses camber in BUMP, the half that matters.
mid = ceil(n/2);
if all(isnan(raw(mid,:))), mid = find(~isnan(raw(:,1)),1); end
ref = raw(mid,:);
for i = 1:n
    if isnan(raw(i,1)), T{i,:} = NaN; T.arm_deg(i) = angles_deg(i); continue; end
    T.travel_mm(i)  = (raw(i,3) - ref(3)) * 1000;
    T.camber_deg(i) = raw(i,6) * 180/pi;    % x rotation: wheel plane lean
    T.toe_deg(i)    = raw(i,4) * 180/pi;    % z rotation: steer
    T.track_mm(i)   = (raw(i,2) - ref(2)) * 1000;
end

ok = ~isnan(T.travel_mm);
fprintf('\n=== %s corner, %d of %d positions assembled ===\n', axle, sum(ok), n);
fprintf('  %8s %10s %11s %9s %10s\n','arm deg','travel mm','camber deg','toe deg','track mm');
for i = 1:n
    if ~ok(i), continue; end
    fprintf('  %8.1f %10.2f %11.3f %9.3f %10.2f\n', ...
            T.arm_deg(i), T.travel_mm(i), T.camber_deg(i), T.toe_deg(i), T.track_mm(i));
end

% The number the plant currently guesses.
v = T(ok,:);
if height(v) > 2
    p = polyfit(v.travel_mm/1000, v.camber_deg, 1);      % deg per metre
    fprintf('\n  camber slope        %+.2f deg/m of travel\n', p(1));
    halfTrack = H.track/2;
    % deg of camber per deg of body roll: a roll phi lifts one wheel by
    % (track/2)*phi, so the chain is slope [deg/m] * halfTrack [m] * rad2deg.
    gain = abs(p(1)) * halfTrack * pi/180;
    fprintf('  implied camber gain  %.3f  (car_spec assumes %.3f)\n', ...
            gain, P.Susp.CamberGainFront);
    fprintf('  bump steer          %+.2f deg/m   (car_spec targets 0)\n', ...
            polyval(polyfit(v.travel_mm/1000, v.toe_deg, 1), 0) * 0 + ...
            [1 0] * polyfit(v.travel_mm/1000, v.toe_deg, 1)');
    fprintf('  scrub               %+.1f mm over the swept travel\n', ...
            max(v.track_mm) - min(v.track_mm));
    fprintf('\n  MAGNITUDE ONLY on the gain. Whether it is signed + or - depends\n');
    fprintf('  on the mirroring convention this port has not yet pinned against\n');
    fprintf('  the plant''s, and asserting a sign that has not been checked is\n');
    fprintf('  exactly the mistake the camber work already made once.\n');
    fprintf('\n  That comparison is the point of this whole exercise: the gain\n');
    fprintf('  is an OUTPUT of the hardpoints here, and an assumed input there.\n');
    fprintf('  The hardpoints are themselves placeholders, so treat the number\n');
    fprintf('  as a demonstration of the mechanism, not as the IFS-08''s curve.\n');
end
end
