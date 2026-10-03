function L = lap_sim(G, seg, opts)
%LAP_SIM  Quasi-steady lap time on a g-g-V envelope.
%
%   L = LAP_SIM()               the car as specified, on lap_track
%   L = LAP_SIM(G)              a lap_ggv envelope (e.g. from overrides)
%   L = LAP_SIM(G, seg, opts)   a different track; opts.closed (default
%                               true), opts.v0 (open runs, default 0),
%                               opts.ds (path step, default 0.25 m)
%
%   The standard method. Every point is capped by the corner speed the
%   envelope allows at that curvature; a forward pass accelerates out of
%   every corner as hard as the grip left over allows, a backward pass brakes
%   into every corner the same way; the lap is the lower of the three. A
%   closed lap is wrapped and run twice so the start speed is the car's own.
%
%   L.time      lap time                                    [s]
%   L.s, L.v    the speed profile                           [m], [m/s]
%   L.limit     per point: corner | accel | brake
%   L.E_pack    pack energy for the lap, net of regen        [kJ]
%   L.E_drag    energy lost to aero drag                     [kJ]
%
%   What it is not: a driver. No yaw transients, no line optimisation (the
%   path is the track centreline), no tyre temperature. Use it to RANK
%   designs; the absolute time is an upper bound on what is possible.

here = fileparts(mfilename('fullpath'));
addpath(here);
if nargin < 1 || isempty(G), G = lap_ggv(); end
if nargin < 2 || isempty(seg), seg = lap_track(); end
if nargin < 3, opts = struct(); end
if ~isfield(opts,'closed'), opts.closed = true; end
if ~isfield(opts,'v0'),     opts.v0 = 0;        end
if ~isfield(opts,'ds'),     opts.ds = 0.25;     end
ds = opts.ds;

% ---- path ---------------------------------------------------------------
k = [];
for i = 1:size(seg,1)
    n = max(1, round(seg(i,2)/ds));
    k = [k; repmat(1/seg(i,1), n, 1)]; %#ok<AGROW>
end
N = numel(k);
s = (0:N-1)' * ds;

% ---- corner-limited speed ------------------------------------------------
vmax = G.v(end);
vcorner = vmax * ones(N,1);
[ku, ~, iu] = unique(k);
vc_u = vmax * ones(size(ku));
for j = 1:numel(ku)
    if ku(j) == 0, continue; end
    f = G.ay0 - G.v.^2 * ku(j);             % grip left at each speed
    last = find(f >= 0, 1, 'last');
    if isempty(last), vc_u(j) = G.v(1);
    elseif last == numel(G.v), vc_u(j) = vmax;
    else
        vc_u(j) = interp1(f(last:last+1), G.v(last:last+1), 0);
    end
end
vcorner = vc_u(iu);

% ---- how hard it can accelerate / brake, given the lateral it must hold ----
% Tabulated once over (v, ay) and looked up bilinearly, rather than searched
% at every path step: the search was 7.5 s a lap, which a study that tests
% each override alone cannot afford.
T = accel_tables(G);

% ---- passes --------------------------------------------------------------
reps = 1 + opts.closed;                      % wrap twice when closed
vf = vcorner;
if ~opts.closed, vf(1) = min(vf(1), opts.v0); end
for r = 1:reps
    for i = 1:N
        if i == 1
            if ~opts.closed, continue; end
            vp = vf(N);
        else
            vp = vf(i-1);
        end
        a = lookup(T, T.Aplus,  vp, vp^2*k(max(i-1,1)));
        vf(i) = min([vcorner(i), vf(i), sqrt(max(vp^2 + 2*a*ds, 0))]);
    end
end
vb = vf;
for r = 1:reps
    for i = N:-1:1
        if i == N
            if ~opts.closed, continue; end
            vn = vb(1);
        else
            vn = vb(i+1);
        end
        a = lookup(T, T.Aminus, vn, vn^2*k(min(i+1,N)));   % most negative ax
        vb(i) = min(vb(i), sqrt(max(vn^2 - 2*a*ds, 0)));
    end
end
v = vb;

limit = repmat("accel", N, 1);
limit(v >= vcorner - 1e-6) = "corner";
limit(v < vf - 1e-6) = "brake";

% ---- time and energy -----------------------------------------------------
if opts.closed, vnext = [v(2:end); v(1)]; else, vnext = [v(2:end); v(end)]; end
vm = max(0.5*(v + vnext), 0.05);
dt = ds ./ vm;
P = G.P;  E = G.E;  g = 9.81;
rho = P.Assumed.AirDensity;
axs  = (vnext.^2 - v.^2) / (2*ds);
Fl   = 0.5*rho*P.ClA*vm.^2;
Fdrg = 0.5*rho*P.CdA*vm.^2;
Fneed = G.m_eff*axs + Fdrg + P.RollingResistance*(P.Mass*g + Fl);
eta = P.DrivetrainEfficiency;
Freg = interp1(E.v, E.F_regen, vm, 'linear', 'extrap');
Ewheel_drive = sum(max(Fneed,0) .* ds);
Eregen       = sum(min(max(-Fneed,0), Freg) .* ds);      % friction does the rest
L.time   = sum(dt);
L.s = s;  L.v = v;  L.vcorner = vcorner;  L.limit = limit;  L.k = k;
L.E_pack = (Ewheel_drive/eta - Eregen*eta) / 1000;          % eta once, each way
L.E_drag = sum(Fdrg .* ds) / 1000;
L.length = N*ds;
L.v_mean = L.length / L.time;
L.v_peak = max(v);
L.v_min  = min(v);
end

% -------------------------------------------------------------------------
function T = accel_tables(G)
%ACCEL_TABLES  Largest ax (Aplus) and most negative ax (Aminus) that still
%   leave ay of lateral grip, on a (v, ay) grid.
T.v  = G.v;
T.ay = linspace(0, max(G.ay(:), [], 'omitnan'), 300);
nv = numel(G.v);  na = numel(T.ay);
T.Aplus = zeros(nv, na);  T.Aminus = zeros(nv, na);
pos = G.ax >= 0;  neg = G.ax <= 0;
axp = G.ax(pos);  axn = G.ax(neg);
for i = 1:nv
    rp = G.ay(i, pos);  rn = G.ay(i, neg);
    for j = 1:na
        jp = find(rp >= T.ay(j) - 1e-9, 1, 'last');
        jn = find(rn >= T.ay(j) - 1e-9, 1, 'first');
        if ~isempty(jp), T.Aplus(i,j)  = axp(jp); end
        if ~isempty(jn), T.Aminus(i,j) = axn(jn); end
    end
end
T.dv = T.v(2) - T.v(1);  T.da = T.ay(2) - T.ay(1);
end

function a = lookup(T, A, v, ay)
%LOOKUP  Bilinear, on the uniform (v, ay) grid, clamped to it.
x = (min(max(v, T.v(1)), T.v(end)) - T.v(1)) / T.dv + 1;
y = min(max(ay, 0), T.ay(end)) / T.da + 1;
i = min(floor(x), numel(T.v)-1);  fx = x - i;
j = min(floor(y), numel(T.ay)-1); fy = y - j;
a = (1-fx)*(1-fy)*A(i,j) + fx*(1-fy)*A(i+1,j) + (1-fx)*fy*A(i,j+1) + fx*fy*A(i+1,j+1);
end
