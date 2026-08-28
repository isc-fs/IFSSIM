function nset = configure_tyre_block(blk, TP, wheel)
%CONFIGURE_TYRE_BLOCK  Apply our tyre to a Combined Slip Wheel 2DOF block.
%
%   nset = CONFIGURE_TYRE_BLOCK(blk, TP, wheel) configures the block at path
%   blk from the Magic Formula parameter struct TP (see BUILD_TYRE_PARAMSET)
%   and the wheel struct, and returns how many coefficients were written.
%
%   wheel.Radius, wheel.Inertia, wheel.Omega0 may each be a number or the NAME
%   of a base-workspace variable. The plant passes names, so settings.json
%   stays the single source of truth; the test rig in TYRE_REPORT passes
%   numbers, so it can freeze the wheel and sweep slip directly.
%
%   THIS EXISTS SO THERE IS ONE RECIPE, NOT TWO. The plant and the report that
%   validates it must configure the block identically, or the report is
%   measuring something the car does not use. Every ordering constraint below
%   was established by experiment: the block's implementation is P-coded and
%   none of this is documented.

if nargin < 3, wheel = struct(); end
if ~isfield(wheel,'Radius'),  wheel.Radius  = 'IFSSIM_Rw'; end
if ~isfield(wheel,'Inertia'), wheel.Inertia = 'IFSSIM_Iw'; end
if ~isfield(wheel,'Omega0'),  wheel.Omega0  = 0;           end

% (1) THE FILE, THEN THE TYPE. Both have to be ours, because the block reads
% from both. Coefficients that appear as dialog fields are taken from there,
% in step (2); ones that do NOT appear -- Q_RE0 and the rest of the vertical
% set are hidden once vertType is None -- are still read from this file.
% Pointed at the shipped passenger-car set, Q_RE0 = 1.267 silently multiplied
% our rolling radius and a free wheel turned at 39.2 rad/s instead of 49.5.
%
% Bare filename, not an absolute path, so the saved model stays portable.
set_param(blk,'tireParamSet','ifssim_tyre.mat');
set_param(blk,'tireType','External file');

% (2) THE COEFFICIENTS, ONE BY ONE. While tireType names a built-in tyre the
% dialog fields are ignored entirely; switching to 'External file' is what
% makes them authoritative. They cannot be loaded FROM the file here: that
% path runs through a mask callback which opens a file dialog, and under
% matlab -batch it fails outright. Writing only some of them yields a tyre
% that generates no force at all, so it is all of them or none.
mn   = get_param(blk,'MaskNames');
nset = 0;
for k = 1:numel(mn)
    fld = mn{k};
    if ~isfield(TP,fld), continue; end
    v = TP.(fld);
    if ~isnumeric(v) || ~isscalar(v), continue; end
    try, set_param(blk, fld, num2str(v,16)); nset = nset + 1; catch, end %#ok<CTCH>
end

% (3) THE WHEEL, as opposed to the tyre. The file describes the rubber; these
% describe the thing it is wrapped around. br = 0 because bearing drag is
% already accounted for in our rolling resistance.
set_param(blk, ...
    'UNLOADED_RADIUS', as_str(wheel.Radius), ...
    'IYY',             as_str(wheel.Inertia), ...
    'br',              '0', ...
    'omegao',          as_str(wheel.Omega0));

% (4) Q_RE0 scales the free rolling radius and is HIDDEN from the dialog while
% vertType is None, so it cannot be written in the loop above. Expose it,
% write it, hide it again.
set_param(blk,'vertType','Magic Formula');
set_param(blk,'Q_RE0','1','Q_V1','0','Q_V2','0');

% (5) THE POPUPS GO LAST. Every dialog write re-runs the mask initialisation,
% which puts vertType back to 'Magic Formula' -- and that computes Fz from
% ground penetration instead of taking it from Fext, so the tyre reports zero
% vertical load and the car has no grip whatsoever. Set these first and they
% are silently undone by step (2).
set_param(blk,'BrakeType','None','vertType','None','turnslip','off','plySteer','off');
end

function s = as_str(v)
if ischar(v) || isstring(v), s = char(v); else, s = num2str(v,16); end
end
