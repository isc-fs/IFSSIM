function T = sm_corner_sweep(axle, angles_deg, quiet)
%SM_CORNER_SWEEP  Camber, toe and track change against wheel travel.
%
%   T = SM_CORNER_SWEEP('front') sweeps the lower wishbone through its range
%   and reports what the WHEEL does. This is the first number this port
%   produces that the plant currently invents: Susp.CamberGainFront is 0.80
%   because somebody chose 0.80, and here it falls out of the hardpoints.
%
%   METHOD. The corner is assembled at a sequence of lower-arm angles and the
%   wheel frame is read at each one. The hardpoints are the ACTIVE car's
%   (sm_hardpoints), so ifssim_car('IFS-09') sweeps the IFS-09. Assembly, not simulation: a kinematic
%   curve is a statement about geometry, and driving the joint with prescribed
%   motion would need the input's first two derivatives and would mix the
%   mechanism's dynamics into a measurement that has none. StopTime is zero;
%   what is being read is where the linkage CAN be, not where it goes.

if nargin < 1 || isempty(axle), axle = 'front'; end
if nargin < 2 || isempty(angles_deg), angles_deg = -10:1:10; end
if nargin < 3, quiet = false; end

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

% A second sensor on the UPPER BALL JOINT, reading position only. This is
% what makes the sign check independent: it never passes through an angle
% decomposition, so it cannot share an error with the camber extraction.
add_block(TS,[mdl '/UcaSensor'],'Position',[760 560 820 640]);
set_param([mdl '/UcaSensor'],'MeasurementFrame','World','SenseXYZ','on');
add_line(mdl,'World/RConn1','UcaSensor/LConn1','autorouting','on');
add_line(mdl,'up_to_uca/RConn1','UcaSensor/RConn1','autorouting','on');

% Damper length: the distance between the damper's chassis mount and the
% rocker's damper end, read in the mount's frame. The damper is not modelled
% as a joint chain -- that would be a third closed loop to measure one length.
hasRocker = isfield(H,'pr_lca');
if hasRocker
    add_block(TS,[mdl '/DamperSensor'],'Position',[760 700 820 780]);
    set_param([mdl '/DamperSensor'],'MeasurementFrame','Base','SenseXYZ','on');
    add_line(mdl,'mount_dmp/RConn1','DamperSensor/LConn1','autorouting','on');
    add_line(mdl,'arm_rk_dmp/RConn1','DamperSensor/RConn1','autorouting','on');
end

PS2SL = ['nesl_utility/PS-Simulink' newline 'Converter'];
outs = {'rot','WheelSensor/RConn2'; 'pos','WheelSensor/RConn3'; 'uca','UcaSensor/RConn2'};
if hasRocker, outs(end+1,:) = {'dmp','DamperSensor/RConn2'}; end
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
raw = NaN(n,9);
Ld  = NaN(n,1);
T = table('Size',[n 7],'VariableTypes',repmat({'double'},1,7), ...
          'VariableNames',{'arm_deg','travel_mm','camber_deg','toe_deg','track_mm','top_y','uca_dy'});
ref = [];
for i = 1:n
    set_param(piv,'PositionTargetValue', sprintf('%.10g', angles_deg(i)));
    try
        r = sim(mdl);
    catch ME
        fprintf('  angle %+5.1f deg: did not assemble (%s)\n', angles_deg(i), ...
                strrep(ME.message(1:min(60,end)),newline,' '));
        T{i,:} = NaN;  raw(i,:) = NaN(1,9);  continue
    end
    rot = squeeze(r.get('LOG_rot').Data);   rot = rot(:)';    % [z y x] radians
    pos = squeeze(r.get('LOG_pos').Data);   pos = pos(:)';    % [x y z] metres
    uca = squeeze(r.get('LOG_uca').Data);   uca = uca(:)';
    if hasRocker
        dv = squeeze(r.get('LOG_dmp').Data);  Ld(i) = norm(dv(:)); %#ok<AGROW>
    end
    raw(i,:) = [pos, rot, uca]; %#ok<AGROW>
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
% THE MODEL IS THE CAR AT STATIC. The middle of the sweep is the lower arm at
% its hardpoint angle, so the assembled wheel centre and upper ball joint must
% sit EXACTLY on the hardpoints. If they do not, the linkage built is not the
% one described, and every curve below is of something else.
asmErr = max([norm(ref(1:3) - H.wheel_centre), norm(ref(7:9) - H.uca_outer)]);
T.Properties.UserData.assemblyError_m = asmErr;
for i = 1:n
    if isnan(raw(i,1)), T{i,:} = NaN; T.arm_deg(i) = angles_deg(i); continue; end
    T.travel_mm(i)  = (raw(i,3) - ref(3)) * 1000;
    % SAE camber: POSITIVE when the top of the wheel leans OUTWARD. For this
    % LEFT wheel, outward is +y. A positive rotation about +x carries the
    % wheel's vertical axis to [0, -sin, cos], i.e. the top toward -y, i.e.
    % INWARD -- so SAE camber is the NEGATIVE of the x angle. That derivation
    % is not trusted on its own: top_y below reads the same fact off the
    % full rotation matrix, and the two are checked against each other.
    T.camber_deg(i) = -raw(i,6) * 180/pi;
    T.toe_deg(i)    = raw(i,4) * 180/pi;    % z rotation: steer
    T.track_mm(i)   = (raw(i,2) - ref(2)) * 1000;
    Rw = rotz_(raw(i,4)) * roty_(raw(i,5)) * rotx_(raw(i,6));   % ZYX
    T.top_y(i) = Rw(2,3);   % y-component of the wheel vertical axis
    % Lateral offset of the upper ball joint from the wheel centre, from
    % POSITIONS alone. When the top of the upright tips inward this shrinks.
    T.uca_dy(i) = raw(i,8) - raw(i,2);
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
    % SIGNED, now that the convention is pinned. car_spec's CamberGain is the
    % fraction of body roll the geometry RECOVERS. In roll the outer wheel
    % goes into bump and the body tilts it top-OUT (positive SAE camber); the
    % geometry recovers that only if bump drives camber NEGATIVE (top in).
    % So a negative camber-vs-bump slope is positive gain.
    gain = -p(1) * halfTrack * pi/180;
    sfx = tern(strcmp(axle,'front'), 'Front', 'Rear');
    fprintf('  camber gain         %+.3f  (car_spec says %+.3f)\n', ...
            gain, P.Susp.(['CamberGain' sfx]));
    T.Properties.UserData.camberSlope_degpm = p(1);

    % THE CONVENTION CHECK. Whatever the angle extraction says, the wheel's
    % top either leans inward or it doesn't, and the rotation matrix knows
    % which. For a left wheel, top-in means the vertical axis has a NEGATIVE
    % y-component and SAE camber must be NEGATIVE. Every assembled position
    % has to agree, or the sign above is wrong.
    agree = ((v.top_y < 0) == (v.camber_deg < 0)) | abs(v.camber_deg) < 1e-6;
    fprintf('  algebra check       %s at %d of %d positions (same angles, so this only checks the sign derivation)\n', ...
            ternary(all(agree),'CONSISTENT','INCONSISTENT'), nnz(agree), numel(agree));
    % INDEPENDENT CHECK, from positions only. Camber going more negative
    % (top in) must coincide with the upper ball joint moving TOWARD the car
    % centre relative to the wheel centre, i.e. uca_dy decreasing. So the two
    % must rise and fall together across the sweep. This cannot share an
    % error with the angle extraction, because it never uses an angle.
    c = corrcoef(v.camber_deg, v.uca_dy);  c = c(1,2);
    indep = c > 0.99;
    fprintf('  independent check   %s  (camber vs upper-ball-joint offset, r = %+.4f)\n', ...
            ternary(indep,'CONFIRMS','CONTRADICTS'), c);
    T.Properties.UserData.signConsistent = all(agree) && indep;
    T.Properties.UserData.gain = gain;
    pt = polyfit(v.travel_mm/1000, v.toe_deg, 1);
    T.Properties.UserData.bumpSteer_degpm = pt(1);
    fprintf('  bump steer          %+.2f deg/m   (car_spec says %+.2f)\n', ...
            pt(1), P.Susp.(['BumpSteer' sfx]));
    if hasRocker && any(isfinite(Ld))
        % MOTION RATIO = damper travel per unit wheel travel. Positive by
        % convention: the damper SHORTENS as the wheel goes into bump.
        tr = T.travel_mm(ok)/1000;  L = Ld(ok);
        [tr, ix] = sort(tr);  L = L(ix);
        mr = -gradient(L, tr);                     % local, along the travel
        mr0 = interp1(tr, mr, 0, 'linear');        % at static ride height
        T.Properties.UserData.motionRatio = mr0;
        T.Properties.UserData.mrCurve = [tr*1000, mr];
        % TOGGLE CHECK. If the motion ratio changes sign anywhere in the
        % swept travel, an arm has lined up with its link and the damper
        % reverses direction. That is a broken linkage, not a characteristic,
        % and every number derived from it is meaningless -- so say so rather
        % than report a static value that looks reasonable.
        toggles = any(mr <= 0) || any(diff(sign(mr)) ~= 0);
        T.Properties.UserData.toggles = toggles;
        if toggles
            fprintf('  motion ratio        LINKAGE TOGGLES within the travel -- geometry is invalid\n');
            fprintf('                      (ranges %.3f to %.3f; a damper must not reverse)\n', min(mr), max(mr));
            fprintf('  scrub               %+.1f mm over the swept travel\n', max(v.track_mm)-min(v.track_mm));
            return
        end
        fprintf('  motion ratio        %.3f at static  (car_spec says %.3f)\n', ...
                mr0, P.Susp.(['MotionRatio' sfx]));
        fprintf('                      %.3f at full droop -> %.3f at full bump  (%s)\n', ...
                mr(1), mr(end), ternary(mr(end) > mr(1), 'PROGRESSIVE', 'REGRESSIVE'));
        kw0 = P.Susp.(['SpringRate' sfx]) * mr0^2;
        fprintf('  wheel rate          %.0f N/m from the same spring  (plant uses %.0f)\n', ...
                kw0, P.Derived.(['WheelRate' sfx]));
    end
    fprintf('  scrub               %+.1f mm over the swept travel\n', ...
            max(v.track_mm) - min(v.track_mm));
    fprintf('  assembled at static %.2g m from the hardpoints  (%s)\n', asmErr, ...
            tern(asmErr < 1e-6, 'the model IS the described linkage', 'NOT the described linkage'));
    fprintf('\n  Camber is SAE: positive = top of the wheel OUTWARD. Hardpoints: %s.\n', H.car);
end
end

% -------------------------------------------------------------------------
function R = rotx_(a), R = [1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)]; end
function R = roty_(a), R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)]; end
function R = rotz_(a), R = [cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1]; end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
function s = tern(c,a,b), if c, s=a; else, s=b; end, end
