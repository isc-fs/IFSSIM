function ifssim_params_report(P)
%IFSSIM_PARAMS_REPORT  Print the parameter set and where each value came from.
%
%   Prints provenance per field, because "the model uses 275 kg" and "the model
%   fell back to a 290 kg default because settings.json was silent" look
%   identical in a block diagram and are completely different claims.
%
%   Also runs the sanity checks that would have caught the divergences this
%   project has already paid for.

if nargin < 1, P = ifssim_params(); end

fprintf('\n=== IFSSIM vehicle parameters ===\n');
fprintf('source file : %s\n', P.SettingsPath);
fprintf('vehicle     : %s\n\n', P.VehicleName);

f = fieldnames(P);
nFromFile = 0; nDefault = 0;
for i = 1:numel(f)
    k = f{i};
    if any(strcmp(k, {'Source','Pacejka','Derived','SettingsPath','VehicleName'})), continue; end
    src = P.Source.(k);
    if strcmp(src,'settings.json'), nFromFile = nFromFile + 1; tag = '   ';
    else,                           nDefault  = nDefault  + 1; tag = '(D)';
    end
    fprintf('  %s %-22s %12.6g   %s\n', tag, k, P.(k), src);
end

fprintf('\n  Pacejka:\n');
pf = fieldnames(P.Pacejka);
for i = 1:numel(pf)
    k = pf{i};
    src = P.Source.(['Pacejka_' k]);
    tag = '   '; if ~strcmp(src,'settings.json'), tag = '(D)'; end
    fprintf('  %s   %-20s %12.6g   %s\n', tag, k, P.Pacejka.(k), src);
end

fprintf('\n  derived:\n');
d = fieldnames(P.Derived);
for i = 1:numel(d)
    fprintf('      %-22s %12.6g\n', d{i}, P.Derived.(d{i}));
end

fprintf('\n%d parameter(s) from settings.json, %d fell back to C++ defaults\n', ...
        nFromFile, nDefault);
if nDefault > 0
    fprintf(['NOTE: a "(D)" value is NOT configured — it mirrors the default in\n' ...
             '      FSDSSettings.h. The plant is only as authoritative as the file.\n']);
end

% ---------------------------------------------------------------------
% Sanity checks. Each exists because something in this project's history
% failed it, and each is cheap enough that there is no excuse not to.
% ---------------------------------------------------------------------
fprintf('\n--- sanity ---\n');
chk = @(ok,msg) fprintf('  [%s] %s\n', ternary(ok,'ok  ','WARN'), msg);

% A 16x7.5-10 Hoosier is ~0.203 m radius. IFS_Sim carried 0.30 m, which would
% put every speed and energy figure it produced ~48% out.
chk(P.WheelRadius > 0.15 && P.WheelRadius < 0.28, ...
    sprintf('wheel radius %.3f m plausible for a 10in FS wheel', P.WheelRadius));

% FS cars with aero run roughly 3-5 Hz. Chaos ran at 3.3 Hz on an unset
% spring rate until the declared HeaveStiffness was actually applied.
chk(P.Derived.RideFreqHz > 2.5 && P.Derived.RideFreqHz < 6.0, ...
    sprintf('ride frequency %.2f Hz in the 3-5 Hz band for an aero FS car', P.Derived.RideFreqHz));

chk(P.WeightDistFront > 0.35 && P.WeightDistFront < 0.60, ...
    sprintf('front weight distribution %.1f%%', P.WeightDistFront*100));

chk(P.Mass > 150 && P.Mass < 400, sprintf('mass %.0f kg', P.Mass));

% TireMu feeds the Pacejka peak. Above ~2 is not a slick, it is a typo.
chk(P.TireMu > 0.8 && P.TireMu < 2.0, sprintf('tyre mu %.2f', P.TireMu));

% The one that cost this project a season: two models, two stiffnesses.
chk(abs(P.Derived.WheelRateEach*4 - P.HeaveStiffness) < 1, ...
    sprintf('wheel rate x4 (%.0f N/m) equals declared HeaveStiffness', ...
            P.Derived.WheelRateEach*4));

if strcmp(P.Source.Mass,'default')
    fprintf(['  [WARN] Mass is a DEFAULT (290 kg, documented as "car 210 + driver 80")\n' ...
             '         on a DRIVERLESS car. settings.json declares 275. If this model\n' ...
             '         reports 290 the file was not read.\n']);
end
end

function s = ternary(c,a,b)
if c, s = a; else, s = b; end
end
