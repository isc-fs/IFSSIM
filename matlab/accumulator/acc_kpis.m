function K = acc_kpis(P, opts)
%ACC_KPIS  The numbers the accumulator department is judged on, for one car.
%
%   K = ACC_KPIS()         the car as specified
%   K = ACC_KPIS(P)        P from ifssim_params(overrides)
%   K = ACC_KPIS(P, opts)  opts.massFollowsPack (default true)
%                          opts.distance        endurance length, m
%
%   THE CAR'S MASS FOLLOWS THE PACK. car_spec declares the car's total mass
%   and, separately, the cell mass and count, and nothing links them: add a
%   module and the pack gets bigger while the car stays 275 kg. A study that
%   adds cells for free will always say "add cells". So here the change in
%   cell mass is added to the car's mass before anything is run. Cells only:
%   the module housings, busbars and cooling that come with them are not in
%   car_spec, so even this under-charges a bigger pack.
%
%   About 5 s: four laps for the endurance, plus pt_model.

here = fileparts(mfilename('fullpath'));
addpath(here, fullfile(here,'..','lap'), fullfile(here,'..','pt'), ...
        fullfile(here,'..','plant'), fullfile(here,'..','spec'));
if nargin < 1 || isempty(P), P = ifssim_params(); end
if nargin < 2, opts = struct(); end
if ~isfield(opts, 'massFollowsPack'), opts.massFollowsPack = true; end

PK  = pack_from_cells(P);
PK0 = pack_from_cells(ifssim_params());
K.dMass = PK.Mass - PK0.Mass;
if opts.massFollowsPack && abs(K.dMass) > 1e-9
    ov = P.Overrides;
    ov.Mass = P.Mass + K.dMass;
    P = ifssim_params(ov);
end

K.Ns = PK.Ns;  K.Np = PK.Np;
K.V_max   = PK.VMax;
K.V_nom   = PK.VNom;
K.E_kWh   = PK.EnergyWh / 1000;
K.cap_Ah  = PK.CapacityAh;
K.R_pack  = PK.Rint;
K.m_cells = PK.Mass;
K.m_car   = P.Mass;
K.I_limit = PK.IOperating;
K.I_cell_at_limit = PK.CellAmpsAtOperating;
K.I_cell_sustained = P.Cell.ISustained;

E  = pt_model(P);
El = pt_model(P, struct('soc', 0.25));
K.kW_fresh = E.shaft_kW;
K.kW_tired = El.shaft_kW;
K.t75      = E.t_accel;

R = acc_endurance(P, opts);
K.finishes   = double(R.finishes);
K.soc_end    = R.soc_end;
K.laps       = R.laps;
K.endurance  = R.time;
K.E_used     = R.E_used_kWh;
K.V_cell_min = R.V_cell_min;
K.I_cell_pk  = R.I_cell_peak;
K.I_cell_rms = R.I_cell_rms;
K.dT_cell    = R.dT_cell;
K.lap_first  = R.lap_first;
K.lap_tired  = R.lap_tired;
K.R = R;  K.P = P;  K.PK = PK;
end
