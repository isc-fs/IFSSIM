function [pct, eps_rel] = yaw_rate_fit(r_ref, r_model)
%YAW_RATE_FIT  The vehicle-dynamics validation metric, Escofet eq. (12).
%
%   eps_rel = 100 * sum|r_ref - r_model| / sum|r_ref|
%   pct     = 100 - eps_rel, the "Fit = 96.74%" the thesis puts on its plots.
%
%   Yaw rate, not position and not lateral acceleration. Position integrates
%   every error and flatters nothing; lateral acceleration is dominated by
%   speed. Yaw rate is what a vehicle model is FOR -- it is the state a yaw
%   controller closes on -- and it is measured directly by the IMU, so it
%   needs no estimator between the vehicle and the number.
%
%   For reference, the thesis reports 96.74% on a ramp steer, 98.77% on a skid
%   pad and 82.52% on an autocross lap, and calls that satisfactory.

r_ref = r_ref(:);  r_model = r_model(:);
if numel(r_ref) ~= numel(r_model)
    error('yaw_rate_fit:size', 'reference has %d points, model has %d', ...
          numel(r_ref), numel(r_model));
end
den = sum(abs(r_ref));
if den <= 0
    error('yaw_rate_fit:straight', ...
          ['the reference never turns, so a relative yaw-rate error is ' ...
           'undefined. Validate on a manoeuvre that steers.']);
end
eps_rel = 100 * sum(abs(r_ref - r_model)) / den;
pct = 100 - eps_rel;
end
