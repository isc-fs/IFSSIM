function seg = lap_track()
%LAP_TRACK  An endurance-style FS lap, as [radius_m length_m] segments.
%
%   Radius Inf is a straight. Built from the geometry the rules ALLOW
%   (FS Rules, dynamic event layout): straights no longer than 80 m, constant
%   turns 30 to 54 m diameter, hairpins at least 9 m outside diameter,
%   slaloms with cones 7.5 to 12 m apart.
%
%   ONE COPY. fs_track_cycle (the speed trace endurance_current is built on)
%   and lap_sim (the department lap time) both drive this table, so a change
%   to the track cannot leave the two disagreeing about what a lap is.

% A slalom is not one corner, it is a rapid alternation, so it is written out
% as a run of tight radii rather than smoothed into a single arc. Cones at
% 7.5-12 m give an effective radius of roughly 5-8 m through the weave.
slalom = repmat([6 8; 6 8], 4, 1);          % ~64 m of weaving

% Straights kept SHORT. The rules permit 80 m, but a rules maximum is not a
% typical layout: on a real endurance track the straights connect tight
% features and the car is rarely pointing straight for long. Left at the
% permitted maximum this lap spent more time above 80 km/h than a generic
% demo cycle did, which is the wrong way round for an event whose average is
% around 50 km/h. Lengthen these and the peaks come back.
seg = [ Inf 45;  15 25;  Inf 30;   4.5 14
        slalom
        Inf 55;  20 30;  Inf 22;   9 20
        Inf 35;  27 40;  Inf 25;   6 16
        slalom
        Inf 60;  15 25;  Inf 22;  12 22
        Inf 38;  22 35;  Inf 25;   4.5 14
        slalom
        Inf 30;  18 28;  Inf 22;  15 25 ];
end
