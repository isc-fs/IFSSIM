function [cyc, T] = fs_track_cycle(verbose)
%FS_TRACK_CYCLE  A speed trace the car could actually drive on an FS track.
%
%   Builds a lap from the geometry the rules ALLOW rather than from a generic
%   drive cycle, then solves the fastest way round it. Returns [t v] at 100 Hz.
%
%   The cycle shipped in MODEL_IFS_08/SIMSCAPE is a MathWorks demo trace and
%   does not describe this sport: it peaks at 95 km/h, holds above 70 km/h for
%   99 m where the rules cap a straight at 80, and contains accelerations of
%   6.5 g. A car cannot do 6.5 g, so that trace was never driven by anything.
%
%   The rules give the track its shape (FS Rules, dynamic event layout):
%     straights      no longer than 80 m
%     constant turns 30 to 54 m diameter
%     hairpins       at least 9 m outside diameter
%     slaloms        cones 7.5 to 12 m apart
%     average speed  around 48 to 57 km/h for endurance
%
%   Corner speeds come from grip, not from a guess: v = sqrt(mu*(m*g+aero)*R/m).
%   Between corners the car accelerates as hard as the rear tyres and the
%   accumulator allow, then brakes as hard as four tyres allow into the next
%   one. That is a quasi-steady-state lap sim -- the standard way to get a
%   speed trace when you have a car and a track and no driver.

if nargin < 1, verbose = true; end
addpath(fileparts(mfilename('fullpath')));
C = car_spec(); PK = pack_from_cells(C);
v = @(n) C.Fields.(strrep(n,'.','_')).value;

m   = v('Mass');      g = 9.81;     mu = v('TireMu');
ClA = v('ClA');       CdA = v('CdA');   rho = 1.225;
Rw  = v('WheelRadius');
wdr = 1 - v('WeightDistFront');            % rear share, for traction
Tmot= v('MotorMaxTorque')*v('GearRatio')*v('DrivetrainEfficiency');
Ppack = PK.POperating * v('DrivetrainEfficiency');
Crr = v('RollingResistance');

% ---- the track, as a list of segments: [radius_m length_m] ----------------
% Radius Inf is a straight. Lengths and radii chosen inside the rule limits,
% in a mix that resembles an endurance layout rather than a single loop.
% A slalom is not one corner, it is a rapid alternation, so it is written out
% as a run of tight radii rather than smoothed into a single arc. Cones at
% 7.5-12 m give an effective radius of roughly 5-8 m through the weave.
slalom = repmat([6 8; 6 8], 4, 1);          % ~64 m of weaving

% Straights kept SHORT. The rules permit 80 m, but a rules maximum is not a
% typical layout: on a real endurance track the straights connect tight
% features and the car is rarely pointing straight for long. Left at the
% permitted maximum this lap spent more time above 80 km/h than a generic
% demo cycle did, which is the wrong way round for an event whose average is
% around 50 km/h. Lengthen these and the peaks come back.
seg = [ Inf 45;  15 25;  Inf 30;   4.5 14
        slalom
        Inf 55;  20 30;  Inf 22;   9 20
        Inf 35;  27 40;  Inf 25;   6 16
        slalom
        Inf 60;  15 25;  Inf 22;  12 22
        Inf 38;  22 35;  Inf 25;   4.5 14
        slalom
        Inf 30;  18 28;  Inf 22;  15 25 ];

ds = 0.25;                                   % path step, m
s = []; r = [];
for i = 1:size(seg,1)
    n = max(1, round(seg(i,2)/ds));
    s = [s; (numel(s)*ds) + (1:n)'*ds];      %#ok<AGROW>
    r = [r; repmat(seg(i,1), n, 1)];         %#ok<AGROW>
end
N = numel(s); L = s(end);

% ---- corner-limited speed, with downforce helping ------------------------
vcorner = zeros(N,1);
for i = 1:N
    if isinf(r(i)), vcorner(i) = 60; continue; end          % straights: unbound here
    % mu*(m*g + 0.5*rho*ClA*v^2) = m*v^2/R  ->  solve for v
    a = m/r(i) - 0.5*rho*ClA*mu;
    if a <= 0, vcorner(i) = 60; else, vcorner(i) = sqrt(mu*m*g/a); end
end

% ---- forward and backward passes, WRAPPED ---------------------------------
% A lap is a loop, so the passes have to wrap round it. Run open, and the
% first segment starts at whatever the corner limit says -- which for a
% straight is "unbounded", so the car begins the lap at 60 m/s and never
% comes down. That produced a 205 km/h peak and a mean well above what the
% rules describe. Two laps of each pass is enough to converge.
vf = vcorner;
for pass = 1:2
    for i = 1:N
        j = 1 + mod(i-2, N);                       % previous point, wrapping
        vprev = vf(j);
        Fdrag = 0.5*rho*CdA*vprev^2 + Crr*m*g;
        Fz_r  = wdr*(m*g + 0.5*rho*ClA*vprev^2);
        Ftr   = min([mu*Fz_r, Tmot/Rw, Ppack/max(vprev,1)]);
        acc   = (Ftr - Fdrag)/m;
        vf(i) = min([vcorner(i), vf(i), sqrt(max(vprev^2 + 2*acc*ds, 0))]);
    end
end
vb = vf;
for pass = 1:2
    for i = N:-1:1
        j = 1 + mod(i, N);                         % next point, wrapping
        vnext = vb(j);
        Fz    = m*g + 0.5*rho*ClA*vnext^2;
        dec   = (mu*Fz + 0.5*rho*CdA*vnext^2)/m;
        vb(i) = min(vb(i), sqrt(vnext^2 + 2*dec*ds));
    end
end
vprof = vb;

% ---- distance profile to a time trace ------------------------------------
dt = ds ./ max(vprof, 0.5);
tt = [0; cumsum(dt(1:end-1))];
tq = (0:0.01:tt(end))';
vq = interp1(tt, vprof, tq, 'linear');
cyc = [tq vq];

T = struct('LapLength',L,'LapTime',tt(end),'MeanKmh',mean(vq)*3.6, ...
           'PeakKmh',max(vq)*3.6,'PeakAccel',max(diff(vq)./diff(tq)));
if verbose
    fprintf('\n=== a lap the rules allow, driven as fast as the car can ===\n');
    fprintf('  length            %.0f m\n', T.LapLength);
    fprintf('  lap time          %.1f s\n', T.LapTime);
    fprintf('  mean speed        %.1f km/h   (rules put endurance near 48-57)\n', T.MeanKmh);
    fprintf('  peak speed        %.1f km/h\n', T.PeakKmh);
    fprintf('  peak acceleration %.2f g      (the demo cycle claimed 6.5)\n', T.PeakAccel/9.81);
    fprintf('  slowest corner    %.1f km/h\n', min(vq)*3.6);
end
end
