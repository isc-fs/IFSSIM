function info = fork_vehicle_body(model, blk, pos)
%FORK_VEHICLE_BODY  Vehicle Body 6DOF with externally resettable state.
%
%   INFO = FORK_VEHICLE_BODY(MODEL, BLK, POS) drops MathWorks' "Vehicle Body
%   6DOF" into MODEL at BLK, breaks its library links, and converts its five
%   integrators to take an external initial condition and a rising-edge reset.
%   It returns the Goto tags the caller must drive.
%
%   WHY THIS EXISTS. The block cannot accept an arbitrary state injection as
%   shipped: every integrator is InitialConditionSource='internal',
%   ExternalReset='none', and Xe_o/eul_o/xbdot_o/p_o are COMPILE-TIME initial
%   conditions, not a runtime reset. Our chassis owns its integrator in a
%   MATLAB Function block and overwrites its own state when sync.enable is
%   set -- that is how the platform puts the car on the start gate, and it is
%   not negotiable. Losing it would mean losing the ability to position the
%   car at all. See docs/vdb_plant_migration.md section 3b.
%
%   WHY A FORK IS ACCEPTABLE HERE. This plant is GENERATED: every .slx is
%   written by a build_*.m, so this is not a modified block checked into the
%   repo but a scripted transformation re-applied on every build. If a MATLAB
%   upgrade moves the internals, the assertion below fails and the BUILD
%   breaks -- loudly, at build time -- rather than the car silently losing its
%   reset. That failure mode is the argument for doing it this way.
%
%   WHAT IT COSTS. The transformation is coupled to MathWorks' internal block
%   structure, which is not a supported interface and can change without
%   notice. Budget for it breaking at some upgrade.
%
%   ROUTING. The reset trigger and the initial conditions reach integrators
%   nested 2 to 3 levels down. Rather than adding ports to every intervening
%   subsystem -- which means editing MathWorks' subsystems, not just their
%   integrators -- this uses global Goto/From. The From sits beside the
%   integrator; the Goto sits at the model's top level. Nothing in between is
%   touched, which is both less work and less to re-do at an upgrade.

if nargin < 3, pos = [200 100 400 300]; end
load_system('vehdynlibeom');
add_block('vehdynlibeom/Vehicle Body 6DOF', blk, 'Position', pos);

% ---- break the library links ------------------------------------------
% The outer link, then any nested one. Without this every set_param below
% either fails or silently edits the LIBRARY, which would corrupt the
% installation for every other project on this machine.
set_param(blk,'LinkStatus','none');
nested = find_system(blk,'LookUnderMasks','all','FollowLinks','on', ...
                     'RegExp','on','LinkStatus','resolved|implicit');
for k = 1:numel(nested)
    try  set_param(nested{k},'LinkStatus','none'); catch; end
end

% ---- the five integrators ---------------------------------------------
% name fragment -> Goto tag, and what the state MEANS
spec = { 'phi',              'IFSSIM_SYNC_EULER', 'body Euler angles [rad]'
         'p,q,r',            'IFSSIM_SYNC_PQR',   'body angular rates [rad/s]'
         'ub,vb,wb',         'IFSSIM_SYNC_VB',    'body-frame velocity [m/s]'
         'xe,ye,ze',         'IFSSIM_SYNC_XE',    'earth-frame position [m]'
         'SignalCollection', 'IFSSIM_SYNC_ACC',   'SignalCollection accumulator' };

ints = find_system(blk,'LookUnderMasks','all','FollowLinks','on','BlockType','Integrator');
assert(numel(ints) == 5, 'fork_vehicle_body:structure', ...
   ['Expected 5 integrators in Vehicle Body 6DOF, found %d. MathWorks has ' ...
    'changed the block internals; the state-injection fork must be redone ' ...
    'against the new structure. See docs/vdb_plant_migration.md 3b.'], numel(ints));

info = struct('tag',{},'meaning',{},'path',{},'width',{});
matched = false(size(spec,1),1);
for k = 1:numel(ints)
    p   = ints{k};
    rel = strrep(p, [blk '/'], '');
    row = 0;
    for r = 1:size(spec,1)
        if contains(rel, spec{r,1}), row = r; break; end
    end
    assert(row > 0, 'fork_vehicle_body:unknownIntegrator', ...
        'Integrator "%s" matches none of the five expected states.', rel);
    assert(~matched(row), 'fork_vehicle_body:ambiguous', ...
        'Two integrators matched "%s"; the name match is not unique.', spec{row,1});
    matched(row) = true;

    % External IC and a LEVEL reset. Port order then becomes
    % [1] derivative in, [2] reset, [3] initial condition.
    %
    % LEVEL, not 'rising', and the difference is the contract. IFSSIM_Chassis
    % overwrites its state on EVERY step for as long as sync_en > 0.5 -- the
    % platform holds enable high while it positions the car, and expects the
    % pose to stay put, not to be nudged once and then drift off under
    % whatever forces happen to be applied. A rising-edge reset injects the
    % pose and immediately lets go.
    %
    % This was forked as 'rising' first, and step 4a's gate passed it: that
    % test asserted the state jumps and then keeps integrating, which is
    % exactly what edge semantics do. It was testing the wrong contract. The
    % A/B against the incumbent chassis in vdb_step4b_check is what caught
    % it -- the teleported car fell 4.6 m while the real chassis held station.
    set_param(p,'InitialConditionSource','external','ExternalReset','level');

    parent = get_param(p,'Parent');
    ip     = get_param(p,'Position');
    nm     = get_param(p,'Name');
    safe   = matlab.lang.makeValidName(spec{row,2});

    add_block('simulink/Signal Routing/From',[parent '/' safe '_rst'], ...
              'GotoTag','IFSSIM_SYNC_TRIG','Position',[ip(1)-140 ip(2)-40 ip(1)-70 ip(2)-24]);
    add_block('simulink/Signal Routing/From',[parent '/' safe '_ic'], ...
              'GotoTag',spec{row,2},'Position',[ip(1)-140 ip(2)+30 ip(1)-70 ip(2)+46]);
    add_line(parent,[safe '_rst/1'],[nm '/2'],'autorouting','on');
    add_line(parent,[safe '_ic/1'], [nm '/3'],'autorouting','on');

    info(end+1) = struct('tag',spec{row,2},'meaning',spec{row,3}, ...
                         'path',rel,'width',NaN); %#ok<AGROW>
end
assert(all(matched), 'fork_vehicle_body:missing', ...
    'Not every expected state was found among the integrators.');

% ---- the Goto side, at the model's top level --------------------------
% TagVisibility 'global' is what lets a From nested inside MathWorks'
% subsystems see these without any port being added to those subsystems.
% The caller wires a signal into each of these; nothing is connected here,
% because what drives them is the caller's business.
allTags = [{'IFSSIM_SYNC_TRIG'}, {info.tag}];
for k = 1:numel(allTags)
    g = [model '/' matlab.lang.makeValidName(allTags{k}) '_goto'];
    add_block('simulink/Signal Routing/Goto', g, 'GotoTag', allTags{k}, ...
              'TagVisibility','global','Position',[60 60+40*k 130 76+40*k]);
end
info(1).gotos = allTags;
end
