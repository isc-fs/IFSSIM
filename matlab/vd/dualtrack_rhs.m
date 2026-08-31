function [xdot, d] = dualtrack_rhs(x, u, M)
%DUALTRACK_RHS  One evaluation of the dual-track model.
%
%   x = [vy; r]        lateral velocity and yaw rate, body frame
%   u = [delta; vx; ax] road-wheel steer (rad), forward speed, forward accel
%   M = dualtrack_build(...)
%
%   Speed is an INPUT, not a state. That is not a shortcut, it is what makes
%   this model useful: to validate it you replay the steering and the speed a
%   vehicle actually had and compare the yaw rate it predicts against the yaw
%   rate that was measured (Escofet eq. 12). Integrating vx would mean also
%   modelling the powertrain, and every error there would be charged to the
%   lateral model.
%
%   Returns diagnostics in d: per-wheel Fz, slip angle and lateral force.

vy = x(1);  r = x(2);
delta = u(1);  vx = u(2);  ax = u(3);
vx = max(vx, 0.5);            % below walking pace slip angles are meaningless

% ---- steering: Ackermann ---------------------------------------------
% A single road-wheel angle in, two front-wheel angles out. Full Ackermann
% puts both front wheels on a common centre; parallel steering gives them the
% same angle. Real cars sit somewhere between, hence the fraction.
dl = delta;  dr = delta;
if abs(delta) > 1e-6 && M.ackermann > 0
    R  = M.L / tan(abs(delta));
    di = atan(M.L / max(R - M.tF/2, 0.1));     % inner wheel, tighter
    do = atan(M.L / (R + M.tF/2));             % outer wheel
    s  = sign(delta);
    if s > 0            % turning left: left wheel is inner
        dl = s*(M.ackermann*di + (1-M.ackermann)*abs(delta));
        dr = s*(M.ackermann*do + (1-M.ackermann)*abs(delta));
    else                % turning right: right wheel is inner
        dl = s*(M.ackermann*do + (1-M.ackermann)*abs(delta));
        dr = s*(M.ackermann*di + (1-M.ackermann)*abs(delta));
    end
end
dw = [dl; dr; 0; 0];

% ---- vertical load ----------------------------------------------------
% Algebraic, from the accelerations -- the defining simplification of this
% model. The plant gets Fz by integrating four spring/damper states; here it
% is a closed form evaluated every step, which is why this runs in
% milliseconds and can be inverted.
%
% Lateral transfer follows Escofet eq. (3): a GEOMETRIC part carried through
% the suspension links at the roll centre, which needs no roll at all, and an
% ELASTIC part carried through the springs and split by roll stiffness. The
% total is m*ay*h/t either way; the split is what decides the balance.
Fl = 0.5*M.rho*M.ClA*vx^2;                   % downforce, positive down
mf = M.m*M.wdF;   mr = M.m*(1-M.wdF);

dWlon = M.m*ax*M.h / M.L;                    % + moves load rearward
FzF   = mf*M.g - dWlon + Fl*M.aeroF;         % per AXLE
FzR   = mr*M.g + dWlon + Fl*(1-M.aeroF);

% ay is needed to size the lateral transfer, and the transfer changes Fz,
% which changes Fy, which changes ay. Three fixed-point passes settle it to
% well under a newton; the alternative is carrying ay as a delayed state,
% which puts a one-step lag into the very quantity the model exists to get
% right.
ay = r*vx;                                   % first guess: steady state
Fz = zeros(4,1);  alpha = zeros(4,1);  Fy = zeros(4,1);
Ksum = M.KrF + M.KrR;
elastic = mf*(M.h - M.hrcF) + mr*(M.h - M.hrcR);
for it = 1:3
    dWf = ay*( mf*M.hrcF/M.tF + (M.KrF/Ksum)*elastic/M.tF );
    dWr = ay*( mr*M.hrcR/M.tR + (M.KrR/Ksum)*elastic/M.tR );
    % +ay is leftward, so load moves RIGHT. FL/RL are the left wheels.
    Fz = [FzF/2 - dWf; FzF/2 + dWf; FzR/2 - dWr; FzR/2 + dWr];
    Fz = max(Fz, 0);                         % a lifted wheel carries nothing

    % ---- suspension kinematics ---------------------------------------
    % Body roll, from the moment about the roll axis over the total roll
    % stiffness. This model has no roll DOF and does not need one -- load
    % transfer is algebraic -- but the ANGLE is what drives camber.
    roll = M.m*ay*((mf*(M.h-M.hrcF) + mr*(M.h-M.hrcR))/M.m) / (M.KrF + M.KrR);

    % Inclination each tyre actually sees. Without camber gain a wheel leans
    % with the body, so the outer one loses its static negative camber; gain
    % takes back that fraction. sgn is +1 on the wheel that is OUTSIDE the
    % corner, which for a left turn (+ay) is the right-hand pair.
    sgn   = [-1; 1; -1; 1] * sign(ay + eps);
    camS  = [M.camF; M.camF; M.camR; M.camR];
    gam   = camS + sgn .* (1 - [M.cgainF; M.cgainF; M.cgainR; M.cgainR]) * abs(roll);

    % Toe change with travel -- ROLL STEER, which needs a per-side sign. Toe is
    % measured inward-positive on each side, so the same toe number points the
    % two wheels of an axle in OPPOSITE directions in the ground frame. Without
    % the sign the two contributions cancel exactly (measured sum 8e-19 rad),
    % the axle gets no net steer, and what is left is pigeon-toe scrub.
    %
    % Zero on both axles by design, so nothing printed today depends on it --
    % but this is the one parameter here whose whole purpose is to be replaced
    % by a string-pot measurement, and it would have handed that measurement
    % back inverted.
    travel = [Fz(1)-FzF/2; Fz(2)-FzF/2; Fz(3)-FzR/2; Fz(4)-FzR/2] ./ ...
             [M.kwF; M.kwF; M.kwR; M.kwR];
    toeSign = [1; -1; 1; -1];        % +y is LEFT, so toe-in is -steer on the left
    dtoe  = toeSign .* [M.bumpF; M.bumpF; M.bumpR; M.bumpR] .* travel;

    for i = 1:4
        vxi = vx - r*M.wy(i);
        vyi = vy + r*M.wx(i);
        alpha(i) = dw(i) + dtoe(i) - atan2(vyi, max(vxi, 0.5));
        Fy(i)    = mf_lateral(alpha(i), Fz(i), M, gam(i), camS(i));
    end
    Fyb = Fy .* cos(dw);                     % Fx is zero here, see below
    ay  = sum(Fyb)/M.m;
end

% ---- equations of motion ---------------------------------------------
% Fx is deliberately absent. This model exists to predict YAW RATE from
% steering and speed, which is the quantity the literature validates against
% and the quantity a yaw controller needs. Driving and braking forces enter
% through ax (load transfer) and through vx being prescribed; giving each
% wheel its own Fx is what turns this into a torque-vectoring model, and the
% hooks for it are the wheel positions and dw above.
xdot    = zeros(2,1);
xdot(1) = sum(Fyb)/M.m - r*vx;               % vy_dot
xdot(2) = sum(M.wx .* Fyb)/M.Izz;            % r_dot

d = struct('Fz',Fz,'alpha',alpha,'Fy',Fy,'ay',ay,'delta_wheel',dw, ...
           'camber',gam,'roll',roll,'toe',dtoe, ...
           'dWlat',[dWf; dWr],'dWlon',dWlon,'downforce',Fl);
end

function Fy = mf_lateral(alpha, Fz, M, gam, camStatic)
%MF_LATERAL  Pure-slip Magic Formula, same coefficients as the plant's block.
%
%   gam is the inclination angle the tyre sees. Its cost is charged against
%   the DEPARTURE FROM STATIC CAMBER, not against upright -- because the
%   static setting was presumably chosen near the tyre's optimum, and we have
%   no data saying where that optimum is. So this answers "what does the
%   geometry cost you as the wheel moves away from where you set it", which is
%   a statement about the geometry, and declines to answer "what is the right
%   static camber", which needs a tyre on a rig.
if Fz <= 0, Fy = 0; return; end
dfz = (Fz - M.Fz0)/M.Fz0;
mu  = M.PDY1 + M.PDY2*dfz;                   % load sensitivity
if nargin >= 5
    % Charged against THIS axle's static camber. It used to use the front's for
    % all four wheels, and the comment claiming they were the same magnitude was
    % simply false -- front is -1.5 deg, rear -1.0. That charged the rear pair
    % 0.5 deg of penalty with the car standing still, charged the loaded rear
    % outer 2.8x too much at the limit, destroyed left/right symmetry on the
    % rear axle, and inverted the camber-gain sensitivity on the rear inner.
    mu = mu * (1 - M.camSens*abs(gam - camStatic)*180/pi);
end
D   = mu*Fz;
if D <= 0, Fy = 0; return; end
C   = M.PCY1;
E   = M.PEY1;
K   = abs(M.PKY1)*M.Fz0*sin(M.PKY4*atan(Fz/(M.PKY2*M.Fz0)));   % cornering stiffness
B   = K/(C*D);
Fy  = D*sin(C*atan(B*alpha - E*(B*alpha - atan(B*alpha))));
end
