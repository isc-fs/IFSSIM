function out = plant_direct_sim(P, stopTime, cmdStruct, syncStruct, tvec)
%PLANT_DIRECT_SIM  Run IFSSIM_Plant without wrapping it in a Model block.
%
%   OUT = PLANT_DIRECT_SIM(P, STOPTIME, CMD) drives the plant on flat road at
%   standard gravity and returns the simulation output. OUT.yout carries the
%   plant's outputs in port order: Pose, Wheels, Powertrain.
%
%   PLANT_DIRECT_SIM(P, STOPTIME, CMD, SYNC, TVEC) drives a TIME-VARYING
%   command: every field of CMD is then either a scalar, held for the run, or
%   a vector sampled at the times in TVEC. That is what lets one run
%   accelerate and then brake, which a constant cannot do and which is the
%   only way to measure a stopping distance.
%
%   WHY NOT A HARNESS WITH A MODEL BLOCK. Because that cannot be made to work
%   once the chassis is continuous. A referencing parent and the plant end up
%   with fixed steps 2 ulp apart -- 0.0010416666666666671 against ...667 --
%   and Simulink requires them to match to the bit when the referenced model
%   mixes discrete and continuous components. The gap is not something the
%   parent can influence; it was measured against
%
%     - seven declared literals, 1/960 and both explicit decimals and 'auto':
%       every one gave the parent ...671;
%     - variable-step parents (ode45, ode23t, VariableStepDiscrete): rejected
%       outright as a solver-type mismatch;
%     - SimulationMode Normal and Accelerator: both clash.
%
%   Nothing written on the parent changes the number Simulink compares. So the
%   fix is to stop having a parent. Driving the model directly through
%   Simulink.SimulationInput removes the negotiation entirely rather than
%   tuning a literal against it, and it does so for BOTH variants -- this is
%   not VDB-specific scaffolding.
%
%   One consequence worth knowing: the plant has FOUR root inports and the old
%   Model-block harness only ever wired three. Simulink grounded Sync
%   silently. Driving directly requires all four, so the sync bus is now
%   explicit rather than defaulted.

if nargin < 4 || isempty(syncStruct)
    syncStruct = Simulink.Bus.createMATLABStruct('IFSSIM_SyncBus');
    syncStruct.enable = 0;
    syncStruct.pos = [0;0;P.CoGHeight];  syncStruct.quat = [1;0;0;0];
    syncStruct.vel_body = [0;0;0];       syncStruct.omega_body = [0;0;0];
end

if nargin < 5 || isempty(tvec)
    t = [0; stopTime];
else
    t = tvec(:);
end

road = Simulink.Bus.createMATLABStruct('IFSSIM_RoadBus');
road.valid = ones(4,1);  road.height = zeros(4,1);  road.mu = P.TireMu*ones(4,1);
road.normal_x = zeros(4,1); road.normal_y = zeros(4,1); road.normal_z = ones(4,1);
road.residual = zeros(4,1);

env = Simulink.Bus.createMATLABStruct('IFSSIM_EnvBus');
env.gravity_z = -9.81;  env.ext_force = [0;0;0];  env.ext_torque = [0;0;0];

ds = Simulink.SimulationData.Dataset;
% Only the COMMAND bus may carry trajectories. Road, env and sync are built
% here and are constant by construction, and letting them be interpreted as
% trajectories is not a hypothetical: road.valid is a 4-WHEEL vector, and with
% four time samples a size test cannot tell four wheels from four timesteps.
% Every IFSSIM_CmdBus leaf is width 1, so within the command bus there is no
% such ambiguity to resolve.
ds = ds.addElement(bus_ts(cmdStruct,  t, true),  'cmd');
ds = ds.addElement(bus_ts(road,       t, false), 'road');
ds = ds.addElement(bus_ts(env,        t, false), 'env');
ds = ds.addElement(bus_ts(syncStruct, t, false), 'sync');
assignin('base','PLANT_DIRECT_IN', ds);

in = Simulink.SimulationInput('IFSSIM_Plant');
in = in.setModelParameter('StopTime', num2str(stopTime), ...
                          'LoadExternalInput','on','ExternalInput','PLANT_DIRECT_IN', ...
                          'SaveOutput','on','SaveFormat','Dataset');
out = sim(in);
end

% -------------------------------------------------------------------------
function s = bus_ts(v, t, allowTraj)
%BUS_TS  A bus value as the struct-of-timeseries root inports want.
%
%   With ALLOWTRAJ false every leaf is HELD across the run. With it true, a
%   leaf whose length matches the time vector is taken as a trajectory --
%   which is safe only where leaves are scalars, i.e. the command bus.
if isstruct(v)
    f = fieldnames(v);  s = struct();
    for k = 1:numel(f), s.(f{k}) = bus_ts(v.(f{k}), t, allowTraj); end
else
    v = double(v);
    if allowTraj && isvector(v) && numel(v) == numel(t) && numel(t) > 1
        s = timeseries(v(:), t);
    else
        s = timeseries(repmat(v(:)', numel(t), 1), t);
    end
end
end
