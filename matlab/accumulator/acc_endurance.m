function R = acc_endurance(P, opts)
%ACC_ENDURANCE  Does the pack finish the endurance, and in what state?
%
%   R = ACC_ENDURANCE(P)         P from ifssim_params(overrides)
%   R = ACC_ENDURANCE(P, opts)   opts.distance  event length, m (default 22000)
%
%   Drives lap_track flat out, lap after lap, with the pack discharging under
%   it. Lap by lap:
%     - the lap is driven at the pack's CURRENT state of charge, so a tired
%       pack gives less power and a slower lap (lap_ggv(P, soc));
%     - each point's electrical power becomes a current by solving
%       V*I = P with V = Voc(soc) - I*R -- the sag the current itself causes;
%     - charge, cell heat and the lowest cell voltage are accumulated.
%
%   Laps are run at four states of charge and interpolated between, so the
%   whole event costs four laps, not twenty-five.
%
%   R.finishes      true if it covers the distance above the cell voltage floor
%   R.soc_end       state of charge at the flag (or where it stopped)
%   R.laps          laps completed
%   R.time          event time                                     [s]
%   R.E_used_kWh    energy drawn from the pack, net of regen
%   R.V_cell_min    lowest cell terminal voltage seen               [V]
%   R.I_cell_peak   highest cell current                           [A]
%   R.I_cell_rms    rms cell current over the event                [A]
%   R.dT_cell       cell temperature rise, NO COOLING              [K]
%
%   UPPER BOUNDS, all of them. Every lap is the car's quasi-steady limit, with
%   no lift-and-coast, no driver change, no cooling. A real endurance is
%   driven slower and costs less. Read these as "the pack copes with the worst
%   case" or "it does not, by this much".

if nargin < 2, opts = struct(); end
if ~isfield(opts, 'distance'), opts.distance = 22000; end
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here,'..','lap'), fullfile(here,'..','spec'));

PK = pack_from_cells(P);
Ns = PK.Ns;  Np = PK.Np;
Rcell = P.Cell.Rint;
Vmin_cell = P.Cell.VMin;

% ---- four laps, across the discharge ---------------------------------
socs = [PK.SoC0, 0.6, 0.35, 0.12];
lap = struct('soc',{},'time',{},'Q_Ah',{},'heat_J',{},'Vmin',{},'Ipk',{},'I2t',{},'E_kWh',{},'ok',{});
for k = 1:numel(socs)
    s = socs(k);
    L = lap_sim(lap_ggv(P, s));
    Voc = PK.OCV(s);
    Pe  = L.P_elec;
    disc = Voc^2 - 4*PK.Rint*Pe;
    ok = all(disc >= 0);                         % can the pack deliver it at all?
    I = (Voc - sqrt(max(disc, 0))) / (2*PK.Rint);
    Vt = Voc - I*PK.Rint;
    Ic = I / Np;
    lap(k) = struct('soc', s, 'time', L.time, 'Q_Ah', sum(I.*L.dt)/3600, ...
        'heat_J', sum(Ic.^2*Rcell.*L.dt), 'Vmin', min(Vt)/Ns, 'Ipk', max(Ic), ...
        'I2t', sum(Ic.^2.*L.dt), 'E_kWh', sum(Pe.*L.dt)/3.6e6, 'ok', ok);
    if k == 1, R.lap_length = L.length; end
end
f = @(fld, s) interp1([lap.soc], [lap.(fld)], s, 'linear', 'extrap');

% ---- the event, lap by lap ------------------------------------------
nLaps = opts.distance / R.lap_length;
soc = PK.SoC0;  t = 0;  E = 0;  heat = 0;  I2t = 0;  Vmin = Inf;  Ipk = 0;  done = 0;
finishes = true;
while done < nLaps - 1e-9
    frac = min(1, nLaps - done);
    vmin_here = f('Vmin', soc);
    if vmin_here < Vmin_cell
        finishes = false;  break                 % the floor: this lap would breach it
    end
    q = f('Q_Ah', soc)/PK.CapacityAh;            % charge a whole lap takes
    if soc - frac*q < 0
        % Runs out PART-WAY round. Count the distance it does cover: that
        % fraction is what a small pack change buys, and stopping at the
        % last whole lap hid it (laps moved only in steps of one).
        frac = soc / q;
        finishes = false;
    end
    t    = t    + frac*f('time', soc);
    E    = E    + frac*f('E_kWh', soc);
    heat = heat + frac*f('heat_J', soc);
    I2t  = I2t  + frac*f('I2t', soc);
    Vmin = min(Vmin, vmin_here);
    Ipk  = max(Ipk, f('Ipk', soc));
    soc  = soc - frac*q;
    done = done + frac;
    if ~finishes, break; end
end

R.finishes    = finishes;
R.laps        = done;
R.laps_needed = nLaps;
R.soc_end     = soc;
R.time        = t;
R.E_used_kWh  = E;
R.E_pack_kWh  = PK.EnergyWh/1000;
R.V_cell_min  = Vmin;
R.I_cell_peak = Ipk;
R.I_cell_rms  = sqrt(I2t / max(t, eps));
R.dT_cell     = heat / (P.Cell.Mass * P.Cell.SpecificHeat);
R.lap_first   = lap(1).time;
R.lap_tired   = f('time', max(soc, 0));
R.per_lap     = lap;
end
