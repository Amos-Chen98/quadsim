function [ s ] = init_state( start, roll, pitch, yaw ,xdot, ydot, zdot)
%INIT_STATE Initialize 13 x 1 state vector

s     = zeros(13,1);
phi0   = roll;
theta0 = pitch;
psi0   = yaw;
Rot0   = RPYtoRot_ZXY(phi0, theta0, psi0);
Quat0  = RotToQuat(Rot0);
s(1)  = start(1); %x
s(2)  = start(2); %y
s(3)  = start(3); %z
s(4)  = xdot;        %xdot
s(5)  = ydot;        %ydot
s(6)  = zdot;        %zdot
s(7)  = Quat0(1); %qw
s(8)  = Quat0(2); %qx
s(9)  = Quat0(3); %qy
s(10) = Quat0(4); %qz
s(11) = 0;        %p
s(12) = 0;        %q
s(13) = 0;        %r

end