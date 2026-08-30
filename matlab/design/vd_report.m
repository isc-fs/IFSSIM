function R = vd_report(M)
%VD_REPORT  Characterise the car. One command, no ROS, no engine, no Docker.
%
%   This is the vehicle-dynamics workbench: the classical manoeuvres, run on
%   the design model, printed as the numbers a vehicle dynamicist actually
%   asks for. It is meant to be run by somebody who does not know how any of
%   the simulator works.
%
%       >> cd matlab/design
%       >> vd_report
%
%   To try a setup change, edit the number in matlab/car/car_spec.m, run
%   build_car, and run this again. The parameters come from settings.json
%   through ifssim_params, so the car described here is the same car the
%   simulator drives -- there is no second copy to keep in step.
%
%   THE LEVERS WORTH TURNING FIRST, and what they do here:
%     RollStiffnessFront / RollStiffnessRear  -> understeer gradient, balance
%     WeightDistFront                         -> balance, and everything else
%     CoGHeight                               -> load transfer, so grip at the limit
%     Tyre.LoadSensitivity (PDY2)             -> how much grip transfer costs
%     TireMu                                  -> the whole grip level
%     Assumed.Izz (ifssim_params)             -> transient response only

if nargin < 1 || isempty(M)
    here = fileparts(mfilename('fullpath'));
    addpath(here); addpath(fullfile(here,'..','plant'));
    M = dualtrack_build();
end
R = struct();

fprintf('\n');
fprintf('==================== VEHICLE CHARACTERISATION ====================\n');
fprintf('  mass %.0f kg, wheelbase %.3f m, track %.3f/%.3f m, CoG %.3f m up\n', ...
        M.m, M.L, M.tF, M.tR, M.h);
fprintf('  weight distribution %.1f%% front, yaw inertia %.0f kg.m^2\n', ...
        100*M.wdF, M.Izz);
fprintf('  roll stiffness %.0f/%.0f N.m/rad (%.1f%% front), roll centres %.0f/%.0f mm\n', ...
        M.KrF, M.KrR, 100*M.KrF/(M.KrF+M.KrR), 1000*M.hrcF, 1000*M.hrcR);
fprintf('  tyre mu %.2f, load sensitivity %.2f\n', M.mu, M.PDY2);

%% ---- steady state: the understeer gradient --------------------------
fprintf('\n---- CONSTANT RADIUS ---------------------------------------------\n');
fprintf('  Steering needed to hold a circle, against lateral acceleration.\n');
fprintf('  delta = L/R + K*ay/g;  K > 0 is understeer.\n\n');
fprintf('    R        Ackermann    K (deg/g)     v_max      ay_max   balance\n');
R.radius = struct('R',{},'K',{},'v_max',{},'ay_max',{});
for Rr = [9.125 15 25]
    C = vd_constant_radius(Rr, M, 4:0.5:22);
    if isempty(C.v), continue; end
    bal = 'understeer';
    if C.K < 0, bal = 'oversteer'; end
    fprintf('  %5.1f m   %6.2f deg   %+7.2f     %5.1f m/s   %5.2f g   %s\n', ...
            Rr, C.ackermann*180/pi, C.K, C.v(end), C.ay(end)/9.81, bal);
    R.radius(end+1) = struct('R',Rr,'K',C.K,'v_max',C.v(end),'ay_max',C.ay(end)); %#ok<AGROW>
end

%% ---- the skid pad, as a lap time ------------------------------------
fprintf('\n---- SKID PAD (FS event, 9.125 m path radius) ---------------------\n');
S = vd_skidpad(M);
R.skidpad = S;
if isfinite(S.v_max)
    fprintf('  max speed   %5.2f m/s (%.1f km/h)\n', S.v_max, S.v_max*3.6);
    fprintf('  lateral     %5.2f g\n', S.ay_max/9.81);
    fprintf('  LAP TIME    %5.2f s        <- compare this against a stopwatch\n', S.lap_time);
    fprintf('  balance     %s at the limit\n', S.balance);
else
    fprintf('  the car cannot hold the circle at any speed tested\n');
end

%% ---- limit grip -----------------------------------------------------
fprintf('\n---- RAMP STEER (limit grip) --------------------------------------\n');
fprintf('  Steering opened gradually at constant speed until the car lets go.\n\n');
fprintf('  NOTE: no balance verdict here, deliberately. A ramp keeps adding lock\n');
fprintf('  past the limit, so at peak ay the front is being steered into\n');
fprintf('  saturation and its slip angle runs away for reasons that have\n');
fprintf('  nothing to do with balance. Balance is a STEADY-STATE property and\n');
fprintf('  the constant-radius test above is the one that measures it.\n\n');
fprintf('    v        peak ay     at steer\n');
R.ramp = struct('v',{},'ay',{},'delta',{});
tr = (0:0.002:12)';
for v = [6 9 12 15]
    Y = dualtrack_sim(tr, M.maxSteer*tr/12, v, M);
    [aymax,i] = max(abs(Y.ay));
    fprintf('  %4.1f m/s   %5.2f g     %5.2f deg\n', ...
            v, aymax/9.81, M.maxSteer*tr(i)/12*180/pi);
    R.ramp(end+1) = struct('v',v,'ay',aymax,'delta',M.maxSteer*tr(i)/12); %#ok<AGROW>
end

%% ---- transient ------------------------------------------------------
fprintf('\n---- STEP STEER (transient response) ------------------------------\n');
fprintf('  How quickly the car answers a sudden input. Only this section is\n');
fprintf('  sensitive to yaw inertia, which is an ASSUMPTION (%.0f kg.m^2).\n\n', M.Izz);
fprintf('    v         steer      yaw rate    time to 90%%   overshoot   sideslip\n');
R.step = struct('v',{},'t90',{},'overshoot',{},'r_ss',{});
for v = [8 12 16]
    S2 = vd_step_steer(v, 4*pi/180, M);
    fprintf('  %4.1f m/s   %4.1f deg   %6.3f r/s   %6.3f s      %5.1f%%     %+5.2f deg\n', ...
            v, 4, S2.r_ss, S2.t90, S2.overshoot, S2.beta_ss*180/pi);
    R.step(end+1) = struct('v',v,'t90',S2.t90,'overshoot',S2.overshoot,'r_ss',S2.r_ss); %#ok<AGROW>
end

%% ---- the one lever ---------------------------------------------------
fprintf('\n---- SETUP LEVER: roll stiffness distribution ---------------------\n');
fprintf('  Total roll stiffness held, front share swept. This is what moving\n');
fprintf('  an anti-roll bar does.\n\n');
fprintf('    front share   K (deg/g)   balance      skidpad lap\n');
B = vd_balance_sweep(M);
R.balance = B;
for i = 1:numel(B.frac)
    bal = 'understeer';  if B.K(i) < 0, bal = 'oversteer '; end
    fprintf('      %4.0f%%       %+7.3f    %s   %5.2f s%s\n', 100*B.frac(i), B.K(i), bal, ...
            B.skidpad(i), repmat('   <- as built', 1, abs(B.frac(i)-M.KrF/(M.KrF+M.KrR))<1e-6));
end

fprintf('\n---- WHAT THIS IS AND IS NOT --------------------------------------\n');
fprintf('  This is the DESIGN model: two states, algebraic load transfer, no\n');
fprintf('  suspension dynamics. It agrees with the full Simulink plant to\n');
fprintf('  within 0.6-8.6%% of yaw rate on a ramp steer (validate_dualtrack).\n');
fprintf('  NOTHING here has been checked against the real car. The tyre is a\n');
fprintf('  fit to no data at all, and CoGHeight, Izz and the roll stiffnesses\n');
fprintf('  are unmeasured. Treat these as what the CAR AS DESCRIBED would do,\n');
fprintf('  not as what the car does.\n');
fprintf('===================================================================\n\n');
end
