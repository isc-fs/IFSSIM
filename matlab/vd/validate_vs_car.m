function R = validate_vs_car(D, M, opts)
%VALIDATE_VS_CAR  Score a vehicle model against a logged run.
%
%   R = VALIDATE_VS_CAR(D) drives the dual-track design model with the speed
%   and road-wheel steer a run recorded, and scores what it produced against
%   what was measured. D comes from vd_car_data.
%
%   The method is the one the reference thesis validates with (Diwakar, TU
%   Delft 2018, ch. 6), so the numbers are directly comparable to its table:
%
%                       RMSE ax     RMSE ay     RMSE yaw rate
%     skid pad          0.65 m/s2   1.26 m/s2   0.162 rad/s
%     full lap          1.43 m/s2   3.19 m/s2   0.158 rad/s
%
%   plus Escofet's eq. (12) relative yaw-rate fit, which the rest of this
%   project already reports (yaw_rate_fit).
%
%   ONE DEPARTURE FROM THE THESIS, AND IT MATTERS. The thesis drives its model
%   with measured wheel TORQUE and compares the resulting ax. Our logs carry no
%   torque, so the model is driven by the SPEED the car had. That makes ax a
%   consequence of the input rather than a prediction: ax RMSE here measures
%   how well the speed trace was differentiated, not how good the powertrain
%   model is. It is reported for completeness and marked as such. The lateral
%   numbers -- ay and yaw rate -- are genuine predictions either way, and they
%   are what this validates.
%
%   OPTS fields, all optional:
%     window   [t0 t1] seconds to score; default: wherever the car moves
%     vmin     ignore samples slower than this, m/s (default 1.0)

if nargin < 2 || isempty(M), M = dualtrack_build(); end
if nargin < 3, opts = struct(); end
if ~D.has_steer
    error('validate_vs_car:nosteer', ...
          '%s has no steering, so there is nothing to drive the model with.', D.file);
end

% Model driven by what the car did: same clock, same steer, same speed.
Y = dualtrack_sim(D.t, D.delta, D.vx_f, M);

vmin = getf(opts, 'vmin', 1.0);
use  = D.vx_f >= vmin;
if isfield(opts,'window')
    use = use & D.t >= opts.window(1) & D.t <= opts.window(2);
end

R.file  = D.file;
R.n     = nnz(use);
R.speed = [min(D.vx_f(use)) max(D.vx_f(use))];

% Measured side is the FILTERED signal, as in the thesis: the RMSE is meant
% to measure model error, not sensor noise.
R.rmse.ax = rms_err(D.ax_f(use), model_ax(Y, D, use));
R.rmse.ay = rms_err(D.ay_f(use), Y.ay(use));
R.rmse.r  = rms_err(D.r_f(use),  Y.r(use));
[R.fit_pct, R.eps_rel] = yaw_rate_fit(D.r_f(use), Y.r(use));

R.thesis = struct('skidpad',[0.65 1.26 0.162], 'fulllap',[1.425 3.19 0.158]);
R.ax_is_prediction = false;   % see the note above: it is not
R.t = D.t;  R.use = use;  R.Y = Y;
end

% =========================================================================
function a = model_ax(Y, D, use) %#ok<INUSD>
% The model was GIVEN the speed trace, so its ax is the derivative of that
% trace. Taken from the model output if it publishes one, else differentiated
% here -- either way it is an input restated, not a prediction.
if isfield(Y,'ax')
    a = Y.ax(use);
else
    a = gradient(D.vx_f, D.t);  a = a(use);
end
end

function e = rms_err(meas, model)
d = meas(:) - model(:);
d = d(isfinite(d));
e = sqrt(mean(d.^2));
end

function v = getf(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end
