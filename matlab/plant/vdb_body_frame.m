function out = vdb_body_frame(what, v)
%VDB_BODY_FRAME  Convert between our ENU world and the VDB body block's frame.
%
%   The block is x-forward, y-RIGHT, z-DOWN, measured (see
%   docs/vdb_step4b_body_semantics.md): released with no force it falls to
%   Xe(3) = +4.905 m in one second. Our IFSSIM_PoseBus is World ENU, z UP.
%
%   The two differ by a 180 degree rotation about x, so the conversion is its
%   own inverse: negate y and z. It is written out here, once, with a name,
%   rather than being three minus signs scattered through the wiring -- a
%   frame error of this kind does not throw, it just drives the car
%   underground, and a reader needs to be able to find every place it happens.
%
%   For the QUATERNION the same similarity transform reduces to negating the
%   y and z components of the vector part, leaving w and x alone.

switch what
    case {'vec','force','torque'}       % self-inverse either direction
        out = [v(1); -v(2); -v(3)];
    case 'quat'                         % [w x y z], body->world
        out = [v(1); v(2); -v(3); -v(4)];
    case 'euler'                        % [roll pitch yaw]
        out = [v(1); -v(2); -v(3)];
    otherwise
        error('vdb_body_frame:what','Unknown conversion "%s".', what);
end
end
