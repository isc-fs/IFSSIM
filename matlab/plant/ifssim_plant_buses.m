function B = ifssim_plant_buses()
%IFSSIM_PLANT_BUSES  Define the platform<->plant signal contract as Simulink buses.
%
%   This is the contract from docs/fmu_plant_migration.md, expressed as
%   something Simulink ENFORCES rather than something a comment asserts. If a
%   subsystem emits the wrong signal, the wrong width, or the wrong type, the
%   model fails to compile — which is the whole point. A boundary that is only
%   documented is a boundary that drifts.
%
%   Units are SI throughout and stated per element: metres, radians, newtons,
%   newton-metres, seconds. Body frame is ISO 8855 / REP-103 — x forward,
%   y LEFT, z up. World frame is ENU.
%
%   Wheel order is FL=1, FR=2, RL=3, RR=4 everywhere. That ordering is already
%   load-bearing on the C++ side; changing it here would be a silent, symmetric,
%   almost undetectable bug.
%
%   Buses are assigned into the base workspace because that is where Simulink
%   resolves data types from.

    function e = el(name, width, unit, desc)
        e = Simulink.BusElement;
        e.Name = name;
        e.Dimensions = width;
        e.DataType = 'double';
        e.Unit = unit;
        e.Description = desc;
    end

    function b = bus(desc, elems)
        b = Simulink.Bus;
        b.Description = desc;
        b.Elements = elems;
    end

%% ---- Platform -> Plant ----------------------------------------------

Cmd = bus('Autonomy commands. Normalised, exactly as they arrive on the wire.', [
    el('throttle',   1, '1',   'Drive demand, 0..1')
    el('regen',      1, '1',   'Regen demand, 0..1. The ONLY service braking this car has.')
    el('steer_norm', 1, '1',   'Steering demand, -1..1. NORMALISED on purpose: converting to radians at the boundary and back is not the identity, and it breaks bit-identity comparisons.')
    el('ebs_latch',  1, '1',   'Emergency brake latched (0/1). The latch is the platform''s; the torque it commands is the plant''s.')
    el('handbrake',  1, '1',   'Handbrake demand (0/1), distinct from EBS.')
    ]);

Road = bus(['Road under each wheel, supplied by the platform because only it owns ' ...
            'the terrain. A PATCH, not a point: a plane fit gives the internal ' ...
            'solver a continuous ground reference between communication steps ' ...
            'instead of a 60 Hz staircase.'], [
    el('valid',     4, '1',   'Contact query succeeded (0/1). Distinguishes airborne from "road at z=0".')
    el('height',    4, 'm',   'Plane height under each wheel, world Z')
    el('normal_x',  4, '1',   'Surface normal X, unit')
    el('normal_y',  4, '1',   'Surface normal Y, unit')
    el('normal_z',  4, '1',   'Surface normal Z, unit')
    el('residual',  4, 'm',   'RMS distance of the probe rays from the fitted plane. NEGATIVE (-1) means NO PLANE WAS FITTED - fewer than three rays hit - and must not be read as a good fit. Zero means measured-and-flat, which is why the not-fitted case is not zero')
    el('mu',        4, '1',   'Surface friction coefficient at the contact patch')
    ]);

Env = bus('Environment and external forces the platform resolves.', [
    el('gravity_z',        1, 'm/s^2', 'Signed gravity along world Z. Told, not assumed.')
    el('ext_force',        3, 'N',     'External force (cone/barrier contact), world frame')
    el('ext_torque',       3, 'N*m',   'External torque, world frame')
    el('ext_point',        3, 'm',     'Application point, world frame')
    el('chassis_grounded', 1, '1',     'Chassis body touching terrain (nose-down / rollover)')
    ]);

Sync = bus(['State injection. The platform WRITES the plant''s state, rather ' ...
            'than the plant reporting it.'], [
    el('enable',     1, '1',     'Nonzero = overwrite the integrator state THIS step with the fields below')
    el('pos',        3, 'm',     'World ENU position of the body origin')
    el('quat',       4, '1',     'Body->world orientation [w x y z]')
    el('vel_body',   3, 'm/s',   'Body-frame velocity')
    el('omega_body', 3, 'rad/s', 'Body-frame angular velocity')
    ]);
% WHY THIS EXISTS, since a plant that lets you overwrite its state looks wrong.
%
% Two things are impossible without it, and both are load-bearing.
%
% RESET. FMI gives no way to write pose into an FMU: it is internal state, and
% the only handle is a whole-state snapshot. So an FMU can be returned to where
% it started and nowhere else, which is not enough for a platform that spawns
% the car on an arbitrary start gate.
%
% PARITY. Comparing this plant against a reference by running both and watching
% them drift measures the CONTROLLER as much as the plant: an open-loop shadow
% diverges without bound whatever its quality, so trajectory divergence says
% nothing. The question worth asking is "given the SAME state and the SAME
% inputs, does this plant respond the same way?" — which needs the states to be
% forced equal every step.
%
% enable is a per-step flag, not a mode. Normal running leaves it zero and the
% integrator is untouched, so nothing about the plant's dynamics changes.

%% ---- Plant -> Platform ----------------------------------------------

Pose = bus('Rigid-body pose and motion. The most-read signals in the simulator.', [
    el('position',      3, 'm',       'World ENU position of the body origin')
    el('quat',          4, '1',       'Body->world orientation [w x y z]')
    el('vel_world',     3, 'm/s',     'Velocity, world frame')
    el('vel_body',      3, 'm/s',     'Velocity, body frame. Published so the platform never has to rotate it itself.')
    el('omega_body',    3, 'rad/s',   'Angular velocity, body frame')
    el('alpha_body',    3, 'rad/s^2', 'Angular acceleration, body frame. REQUIRED for lever-arm translation to offset sensors without re-differencing.')
    el('accel_proper',  3, 'm/s^2',   'Proper acceleration at the BODY ORIGIN, body frame. Not at a sensor mount: sensor placement is the platform''s business.')
    el('attitude',      3, '1',       '[roll rad, pitch rad, heave m] — real state, not caller-supplied as it is today')
    ]);

Wheels = bus('Per-wheel state. Order FL, FR, RL, RR.', [
    el('omega',       4, 'rad/s', 'Wheel angular velocity. Real, unlike today''s /motor_rpm which is chassis speed round-tripped.')
    el('steer',       4, 'rad',   'ACTUAL road-wheel angle, not the command echoed back')
    el('fz',          4, 'N',     'Vertical load at the contact patch')
    el('fx',          4, 'N',     'Longitudinal tyre force. No Chaos analogue exists.')
    el('fy',          4, 'N',     'Lateral tyre force')
    el('slip_ratio',  4, '1',     'Longitudinal slip. STRUCTURALLY unrepresentable in Chaos, which snaps wheel speed to ground speed.')
    el('slip_angle',  4, 'rad',   'Tyre slip angle')
    el('susp_travel', 4, 'm',     'Suspension deflection from static')
    el('in_contact',  4, '1',     'Tyre touching the road (0/1)')
    ]);

Powertrain = bus('Motor and driveline state.', [
    el('motor_rpm',    1, 'rpm', 'Signed motor speed')
    el('motor_torque', 1, 'N*m', 'Signed shaft torque. Negative = regen. Never exported today.')
    el('motor_power',  1, 'W',   'Signed electrical power. Negative = recuperating.')
    el('batt_soc',     1, '1',   'Battery state of charge, 0..1')
    el('batt_voltage', 1, 'V',   'Pack terminal voltage under load')
    ]);

Status = bus('Plant health. Replaces IsSimulatingPhysics() guards.', [
    el('plant_ok',     1, '1', 'Plant integrated this step successfully. The failure branch must LOG, never publish a well-formed zero.')
    el('plant_status', 1, '1', 'Status enum: 0 OK, 1 warning, 2 diverged, 3 failed')
    ]);

% NAMES END IN 'Bus' deliberately. A bus object and a model may not share a
% name — Simulink resolves both through the same path/workspace lookup, and
% 'IFSSIM_Powertrain' as both a bus and a referenced model shadows one with the
% other. The suffix also makes it obvious at a port what is a bus and what is a
% plain vector.
B = struct('IFSSIM_CmdBus',Cmd, 'IFSSIM_RoadBus',Road, 'IFSSIM_EnvBus',Env, ...
           'IFSSIM_SyncBus',Sync, ...
           'IFSSIM_PoseBus',Pose, 'IFSSIM_WheelsBus',Wheels, ...
           'IFSSIM_PowertrainBus',Powertrain, 'IFSSIM_StatusBus',Status);

f = fieldnames(B);
for i = 1:numel(f)
    assignin('base', f{i}, B.(f{i}));
end
end
