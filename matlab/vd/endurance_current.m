function R = endurance_current(verbose)
%ENDURANCE_CURRENT  Accumulator current over an endurance lap, from the lap.
%
%   Works out the current directly from the speed trace and the car, WITHOUT
%   asking the plant to drive the lap -- because the plant cannot. It has no
%   modulated service brake: build_brakes says so in its own first line, "EBS,
%   and the service brake this car lacks", because the driverless car brakes
%   on regen and the EBS is a separate latched system. Regen and drag give it
%   0.18 to 0.23 g. A lap needs 1.4 g into the corners.
%
%   So a plant-in-the-loop endurance run measures a car overshooting every
%   corner at full throttle, which is what the first version of this did: it
%   tracked the trace with an RMS error of 5.4 m/s and sat pinned at the
%   current limit for a quarter of the lap. The number it produced was a
%   property of that failure, not of the track.
%
%   Here the lap says what force is needed at every point and the car says
%   what that costs. Under braking the motor recovers what the regen limit
%   allows and FRICTION does the rest, drawing nothing -- which is what
%   happens on the real car, where a driver has a brake pedal.

if nargin < 1, verbose = true; end
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')),'..','spec'));
C = car_spec(); PK = pack_from_cells(C);
v_ = @(n) C.Fields.(strrep(n,'.','_')).value;
m = v_('Mass'); g = 9.81; rho = 1.225;
CdA = v_('CdA'); Crr = v_('RollingResistance'); eta = v_('DrivetrainEfficiency');
Preg = v_('MaxRegenPower'); Ilim = v_('Pack.CurrentLimit');
Vpack = PK.VNom;

[cyc, T] = fs_track_cycle(false);
t = cyc(:,1); v = cyc(:,2);
a = [0; diff(v)./max(diff(t),1e-6)];

Fres = 0.5*rho*CdA*v.^2 + Crr*m*g;      % always opposing
Freq = m*a + Fres;                       % what the powertrain must supply

Pmech = Freq .* v;
I = zeros(size(v));
drv = Pmech > 0;
I(drv) = Pmech(drv) ./ eta ./ Vpack;                    % drawing
I(~drv) = -min(-Pmech(~drv)*eta, Preg) ./ Vpack;        % recovering, capped
I = min(I, Ilim);                                        % the pack's own limit

R.I_rms  = sqrt(mean(I.^2));
R.I_mean = mean(I(I>0));
R.I_peak = max(I);
R.cell_rms = R.I_rms/PK.Np;
R.watts_cell = R.cell_rms^2 * v_('Cell.Rint');
R.pack_watts = R.watts_cell * PK.NCells;
R.KperSec = R.watts_cell / (v_('Cell.Mass')*v_('Cell.SpecificHeat'));
R.frac_limited = mean(I >= Ilim-0.5);
R.frac_regen = mean(I < 0);
R.lap = T; R.t = t; R.I = I; R.v = v;

if verbose
    nl = ceil(22000/T.LapLength);
    fprintf('\n=== accumulator over an endurance lap ===\n');
    fprintf('  lap: %.0f m, %.0f s, mean %.1f km/h, peak %.1f km/h\n', ...
            T.LapLength, T.LapTime, T.MeanKmh, T.PeakKmh);
    fprintf('\n  RMS current        %6.1f A  = %5.2f A per cell\n', R.I_rms, R.cell_rms);
    fprintf('  mean while driving %6.1f A\n', R.I_mean);
    fprintf('  peak               %6.1f A  = %5.2f A per cell\n', R.I_peak, R.I_peak/PK.Np);
    fprintf('  at the pack limit  %5.1f%% of the lap\n', 100*R.frac_limited);
    fprintf('  recovering         %5.1f%% of the lap\n', 100*R.frac_regen);
    fprintf('\n  heat: %.1f W per cell, %.2f kW into the pack\n', R.watts_cell, R.pack_watts/1000);
    fprintf('  adiabatic rise (NO cooling): %.1f K per lap, %.0f K over %d laps (%.0f min)\n', ...
            R.KperSec*T.LapTime, R.KperSec*T.LapTime*nl, nl, T.LapTime*nl/60);
    % ENERGY IS THE CROSS-CHECK. Current can be wrong in ways that look
    % plausible; energy over a full endurance cannot, because the pack either
    % has enough or the car stops on track.
    Elap = trapz(t, I*Vpack)/3.6e6;               % kWh per lap
    fprintf('\n  energy: %.3f kWh per lap, %.2f kWh over %d laps\n', Elap, Elap*nl, nl);
    fprintf('          pack holds %.2f kWh -> %.0f%% depth of discharge\n', ...
            PK.EnergyWh/1000, 100*Elap*nl/(PK.EnergyWh/1000));
    fprintf('          (%.0f Wh/km, and FS EVs sit around 200-300)\n', Elap*1000/(T.LapLength/1000));

    fprintf('\n  cell sustained figure is about 15 A: %s\n', ...
        ternary(R.cell_rms<15,'this is inside it','THIS IS ABOVE IT'));
end
end
function s = ternary(c,a,b), if c, s=a; else, s=b; end, end
