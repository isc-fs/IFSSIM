function build_tiresuspension(outdir)
%BUILD_TIRESUSPENSION  Fill in IFSSIM_TireSuspension.
%
%   The block that turns the chassis from a projectile into a car. Every force
%   that steers, accelerates or stops the vehicle is generated here.
%
%   MODEL, and what is deliberately simplified:
%
%     * Suspension is QUASI-STATIC — spring and damper acting between the body
%       corner and the road, with no separate unsprung mass. A proper unsprung
%       DOF adds four states and a much stiffer numerical problem, and it buys
%       wheel-hop fidelity that matters on kerbs and not much on flat tarmac.
%       Say so out loud rather than let someone assume it is in there.
%
%     * Tyre is Pacejka Magic Formula '96, coefficients from settings.json,
%       with a friction-ellipse combined-slip limit. No relaxation length, no
%       camber thrust, no load sensitivity of mu, no thermal model.
%
%     * Wheel spin IS a real state, integrated from torque. That is the whole
%       point: Chaos snaps wheel speed to ground speed, so longitudinal slip is
%       structurally unrepresentable there and /motor_rpm is chassis speed
%       round-tripped. Here a wheel can actually spin up or lock.

if nargin < 1 || isempty(outdir)
    outdir = fullfile(fileparts(mfilename('fullpath')), 'models');
end
addpath(fileparts(mfilename('fullpath'))); addpath(outdir);
P = ifssim_load_workspace();

name = 'IFSSIM_TireSuspension';
if bdIsLoaded(name), close_system(name,0); end
f = fullfile(outdir,[name '.slx']);
if isfile(f), delete(f); end

new_system(name,'Model');
set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
               'FixedStep','1/960','StartTime','0','StopTime','inf');

%% ---- inputs -----------------------------------------------------------
add_block('simulink/Sources/In1',[name '/Road'],'Position',[30 40 60 60], ...
          'OutDataTypeStr','Bus: IFSSIM_RoadBus','BusOutputAsStruct','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/Road Select'], ...
          'Position',[130 20 140 120],'OutputSignals','valid,height,mu');
add_line(name,'Road/1','Road Select/1','autorouting','on');

add_block('simulink/Sources/In1',[name '/Pose'],'Position',[30 160 60 180], ...
          'OutDataTypeStr','Bus: IFSSIM_PoseBus','BusOutputAsStruct','on');
add_block('simulink/Signal Routing/Bus Selector',[name '/Pose Select'], ...
          'Position',[130 140 140 240],'OutputSignals','position,quat,vel_body,omega_body');
add_line(name,'Pose/1','Pose Select/1','autorouting','on');

vecIn = {'steer','drive_torque','brake_torque'};
y = 280;
for i = 1:numel(vecIn)
    add_block('simulink/Sources/In1',[name '/' vecIn{i}],'Position',[30 y 60 y+20], ...
              'PortDimensions','4');
    y = y + 60;
end

%% ---- the model --------------------------------------------------------
fcn = [name '/Tyre and Suspension'];
add_block('simulink/User-Defined Functions/MATLAB Function', fcn, ...
          'Position',[280 40 520 460]);
S = sfroot;
chart = S.find('-isa','Stateflow.EMChart','Path',fcn);
chart.Script = tiresusp_code();

% Explicit sizes: the wheel-speed state feeds back through a delay, so nothing
% can be inferred. Same circular-inference trap as the chassis.
sizes = struct( ...
  'road_valid',4,'road_h',4,'road_mu',4, ...
  'pos',3,'quat',4,'velb',3,'omegab',3, ...
  'steer_in',4,'drive_t',4,'brake_t',4,'w_i',4, ...
  'w_n',4, ...
  'omega',4,'steer',4,'fz',4,'fx',4,'fy',4, ...
  'slip_ratio',4,'slip_angle',4,'susp_travel',4,'in_contact',4, ...
  'tyre_force',3,'tyre_torque',3);
data = chart.find('-isa','Stateflow.Data');
for k = 1:numel(data)
    d = data(k);
    if isfield(sizes,d.Name), d.Props.Array.Size = num2str(sizes.(d.Name)); end
end

params = {'IFSSIM_Ts','IFSSIM_aF','IFSSIM_bR','IFSSIM_tF','IFSSIM_tR','IFSSIM_Rw', ...
          'IFSSIM_kw','IFSSIM_cw','IFSSIM_L0','IFSSIM_FzF','IFSSIM_FzR', ...
          'IFSSIM_mu','IFSSIM_LatB','IFSSIM_LatC','IFSSIM_LatE', ...
          'IFSSIM_LonB','IFSSIM_LonC','IFSSIM_LonE','IFSSIM_Iw','IFSSIM_vreg'};
existing = {data.Name};
for k = 1:numel(params)
    if any(strcmp(existing,params{k})), continue; end
    d = Stateflow.Data(chart);
    d.Name = params{k}; d.Scope = 'Parameter'; d.Props.Array.Size = '1';
end

%% ---- wheel-speed state ------------------------------------------------
add_block('simulink/Discrete/Unit Delay',[name '/wheel omega (state)'], ...
          'Position',[330 520 400 550],'InitialCondition','[0;0;0;0]','SampleTime','-1');

%% ---- outputs ----------------------------------------------------------
add_block('simulink/Signal Routing/Bus Creator',[name '/Wheels Bus'], ...
          'Position',[600 40 610 320],'Inputs','9', ...
          'OutDataTypeStr','Bus: IFSSIM_WheelsBus','NonVirtualBus','on');
add_block('simulink/Sinks/Out1',[name '/Wheels'],'Position',[680 170 710 190], ...
          'OutDataTypeStr','Bus: IFSSIM_WheelsBus');
add_line(name,'Wheels Bus/1','Wheels/1','autorouting','on');
add_block('simulink/Sinks/Out1',[name '/tyre_force'],'Position',[680 360 710 380], ...
          'PortDimensions','3');
add_block('simulink/Sinks/Out1',[name '/tyre_torque'],'Position',[680 420 710 440], ...
          'PortDimensions','3');

%% ---- wiring -----------------------------------------------------------
FB = 'Tyre and Suspension';
for i = 1:3, add_line(name,sprintf('Road Select/%d',i),sprintf('%s/%d',FB,i),'autorouting','on'); end
for i = 1:4, add_line(name,sprintf('Pose Select/%d',i),sprintf('%s/%d',FB,3+i),'autorouting','on'); end
for i = 1:3, add_line(name,[vecIn{i} '/1'],sprintf('%s/%d',FB,7+i),'autorouting','on'); end
add_line(name,'wheel omega (state)/1',sprintf('%s/11',FB),'autorouting','on');

add_line(name,sprintf('%s/1',FB),'wheel omega (state)/1','autorouting','on');
for i = 1:9, add_line(name,sprintf('%s/%d',FB,1+i),sprintf('Wheels Bus/%d',i),'autorouting','on'); end
add_line(name,sprintf('%s/11',FB),'tyre_force/1','autorouting','on');
add_line(name,sprintf('%s/12',FB),'tyre_torque/1','autorouting','on');

add_block('built-in/Note',[name '/Notes'],'Position',[40 620], ...
    'Text', tiresusp_notes(P),'HorizontalAlignment','left');

save_system(name,f);
fprintf('wrote %s\n',f);
fprintf('  wheel rate %.0f N/m, damping %.0f N.s/m, mu %.2f, Pacejka LatB %.1f\n', ...
        P.Derived.WheelRateEach, P.Derived.SuspensionDampingCoeff, P.TireMu, P.Pacejka.LatB);
close_system(name,0);
end

%% =======================================================================
function c = tiresusp_code()
L = {
"function [w_n, omega, steer, fz, fx, fy, slip_ratio, slip_angle, susp_travel, in_contact, tyre_force, tyre_torque] = ..."
"         tiresusp(road_valid, road_h, road_mu, pos, quat, velb, omegab, steer_in, drive_t, brake_t, w_i)"
"%#codegen"
"% Per-wheel suspension and tyre forces, summed into a body-frame wrench."
"%"
"% Wheel order FL, FR, RL, RR. Body frame ISO 8855: x forward, y LEFT, z up."
""
"Ts = IFSSIM_Ts;"
"Rw = IFSSIM_Rw;"
""
"% Wheel positions in the body frame, at CoG height. y is POSITIVE LEFT, so the"
"% left wheels take +track/2. Getting this backwards mirrors the car and is"
"% almost impossible to see in a lap time."
"rx = [ IFSSIM_aF;  IFSSIM_aF; -IFSSIM_bR; -IFSSIM_bR];"
"ry = [ IFSSIM_tF/2; -IFSSIM_tF/2;  IFSSIM_tR/2; -IFSSIM_tR/2];"
"Fz_static = [IFSSIM_FzF; IFSSIM_FzF; IFSSIM_FzR; IFSSIM_FzR];"
""
"q = quat / max(norm(quat), eps);"
"R = q2r(q);                      % body -> world"
"vel_world = R * velb;"
"omega_world = R * omegab;"
""
"omega      = zeros(4,1);  steer      = zeros(4,1);"
"fz         = zeros(4,1);  fx         = zeros(4,1);  fy = zeros(4,1);"
"slip_ratio = zeros(4,1);  slip_angle = zeros(4,1);"
"susp_travel= zeros(4,1);  in_contact = zeros(4,1);"
"w_n        = zeros(4,1);"
"F_sum = zeros(3,1);  M_sum = zeros(3,1);"
""
"for i = 1:4"
"    r_b = [rx(i); ry(i); 0];"
"    r_w = R * r_b;"
"    p_w = pos + r_w;                      % corner position, world"
""
"    % ---- suspension ------------------------------------------------"
"    % delta > 0 means compressed relative to static ride height."
"    gap   = p_w(3) - (road_h(i) + Rw);    % attachment above wheel centre"
"    delta = IFSSIM_L0 - gap;"
""
"    % Compression RATE is the downward velocity of this corner. Using the"
"    % corner velocity rather than the CoG velocity is what makes roll and"
"    % pitch damping appear at all."
"    v_corner = vel_world + cross(omega_world, r_w);"
"    ddelta   = -v_corner(3);"
""
"    Fz_i = Fz_static(i) + IFSSIM_kw * delta + IFSSIM_cw * ddelta;"
""
"    % A tyre cannot pull the road. Clamping here is what lets a wheel lift"
"    % in a corner instead of generating negative grip."
"    if Fz_i < 0, Fz_i = 0; end"
"    contact = (road_valid(i) > 0.5) && (Fz_i > 0);"
"    if ~contact, Fz_i = 0; end"
""
"    % ---- contact-patch velocity ------------------------------------"
"    v_b = velb + cross(omegab, r_b);"
"    d   = steer_in(i);"
"    cd  = cos(d); sd = sin(d);"
"    vx =  v_b(1)*cd + v_b(2)*sd;          % along the wheel"
"    vy = -v_b(1)*sd + v_b(2)*cd;          % across it"
"    vref = max(abs(vx), IFSSIM_vreg);     % see note on regularisation"
""
"    % ---- slip ------------------------------------------------------"
"    kappa = (w_i(i)*Rw - vx) / vref;"
"    alpha = atan2(vy, vref);"
""
"    % ---- Pacejka ---------------------------------------------------"
"    muw = road_mu(i);"
"    if muw <= 0, muw = IFSSIM_mu; end"
"    Fmax = muw * Fz_i;"
""
"    Fx0 =  Fmax * mf(kappa, IFSSIM_LonB, IFSSIM_LonC, IFSSIM_LonE);"
"    % Lateral force OPPOSES lateral slip, hence the minus."
"    Fy0 = -Fmax * mf(alpha, IFSSIM_LatB, IFSSIM_LatC, IFSSIM_LatE);"
""
"    % Friction ellipse: the tyre has one budget, spent on both axes."
"    if Fmax > 0"
"        s = sqrt((Fx0/Fmax)^2 + (Fy0/Fmax)^2);"
"        if s > 1, Fx0 = Fx0/s; Fy0 = Fy0/s; end"
"    end"
""
"    % ---- wheel spin ------------------------------------------------"
"    % Iw*dw = drive - brake*sign(w) - Fx*Rw."
"    % tanh instead of sign: sign() chatters at zero crossing and, at a fixed"
"    % 1/960 step, that chatter becomes a limit cycle in the wheel speed."
"    T_brake = brake_t(i) * tanh(w_i(i) * 10);"
"    dw = (drive_t(i) - T_brake - Fx0*Rw) / IFSSIM_Iw;"
"    w_n(i) = w_i(i) + Ts*dw;"
""
"    % ---- to the body frame -----------------------------------------"
"    Fb = [Fx0*cd - Fy0*sd; Fx0*sd + Fy0*cd; Fz_i];"
"    F_sum = F_sum + Fb;"
"    M_sum = M_sum + cross(r_b, Fb);"
""
"    omega(i)=w_i(i); steer(i)=d; fz(i)=Fz_i; fx(i)=Fx0; fy(i)=Fy0;"
"    slip_ratio(i)=kappa; slip_angle(i)=alpha;"
"    susp_travel(i)=delta; in_contact(i)=double(contact);"
"end"
""
"tyre_force  = F_sum;"
"tyre_torque = M_sum;"
"end"
""
"function y = mf(x, B, C, E)"
"%#codegen"
"% Pacejka Magic Formula '96 shape, peak normalised to 1 so the caller scales"
"% by mu*Fz. y = sin(C*atan(B*x - E*(B*x - atan(B*x))))"
"Bx = B*x;"
"y  = sin(C * atan(Bx - E*(Bx - atan(Bx))));"
"end"
""
"function R = q2r(q)"
"%#codegen"
"w=q(1); x=q(2); y=q(3); z=q(4);"
"R = [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w); ..."
"       2*(x*y+z*w), 1-2*(x*x+z*z),   2*(y*z-x*w); ..."
"       2*(x*z-y*w),   2*(y*z+x*w), 1-2*(x*x+y*y)];"
"end"
};
% L holds string scalars (double-quoted), so it is a cell of strings —
% neither a string array nor a cell of char vectors. Convert explicitly.
c = char(strjoin(string(L), newline));
end

%% =======================================================================
function t = tiresusp_notes(P)
t = sprintf([ ...
 'IFSSIM_TIRESUSPENSION\\n\\nOWNER: dynamics\\n\\n' ...
 'Every force that steers, accelerates or stops this car is generated here.\\n\\n' ...
 'PARAMETERS from settings.json: wheel rate %.0f N/m per corner, damping\\n' ...
 '%.0f N.s/m (from the declared ratio %.1f), mu %.2f, Pacejka lateral\\n' ...
 'B=%.1f C=%.2f E=%.2f.\\n\\n' ...
 'WHAT IS SIMPLIFIED — read this before trusting a result:\\n\\n' ...
 '  * Suspension is QUASI-STATIC. Spring and damper act between the body\\n' ...
 '    corner and the road; there is no unsprung mass DOF. That costs wheel-hop\\n' ...
 '    fidelity over kerbs and saves four stiff states. Fine on flat tarmac.\\n\\n' ...
 '  * Tyre is Magic Formula with a friction-ellipse combined-slip limit. No\\n' ...
 '    relaxation length, no camber thrust, no load sensitivity of mu, no\\n' ...
 '    thermal model. Transient lateral response will be slightly quick.\\n\\n' ...
 '  * Slip divides by max(|vx|, %.1f m/s). Below that speed the tyre model is\\n' ...
 '    not to be trusted, so a launch-from-rest study must say so rather than\\n' ...
 '    quietly believing the number.\\n\\n' ...
 'WHEEL SPIN IS A REAL STATE, integrated from torque. That is the point: Chaos\\n' ...
 'snaps wheel speed to ground speed, which makes longitudinal slip structurally\\n' ...
 'unrepresentable and leaves /motor_rpm as chassis speed round-tripped. Here a\\n' ...
 'wheel can genuinely spin up or lock.\\n\\n' ...
 'Fz is clamped at zero: a tyre cannot pull on the road. That clamp is what\\n' ...
 'lets an inside wheel lift in a corner instead of inventing negative grip.'], ...
 P.Derived.WheelRateEach, P.Derived.SuspensionDampingCoeff, P.SuspensionDamping, ...
 P.TireMu, P.Pacejka.LatB, P.Pacejka.LatC, P.Pacejka.LatE, ...
 P.Assumed.SlipRegularisationSpeed);
end
