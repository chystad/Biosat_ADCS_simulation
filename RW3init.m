clear

% RW3sim is trying to accomplish a realistic multi-axis reaction wheel 
% control. The situation I will try to simulate is when a reaction wheel is
% attached to the satallite's z- and x-axis.

% Definitions:
% The earth-centered-inertial frame is self-explanatory
% The orbit frame is the frame defined by the z-axis pointing towards the
% earth's center, and y-axis defined by the cross product of the satellite
% speed and earth-distance vector. 
% The body frame is the frame that is rigidly attached to the cube-sat
% The desired frame is the frame symoling the wanted satellite attitude
% The reaction wheel frame is the frame showing the RW orientations ins
% respect to the body frame

% Physical constants are extracted from the paper: 
% Spacecraft Attitude and Angular Rate Tracking using Reaction Wheels and
% Magnetorquers


%% Reaction wheels
% Note to self: RW mass is not necessary for a true spacecraft attitude 
% physics simulation. This is because a torque applied to the RWs will also
% apply the oppsite torque on the spacecraft. However, the RW masses are 
% necessary for determening the RW rotational speed and angular momentum.

% RW1 spin axis in the body frame
a1 = [0;
      0;
      1] * 1/norm([0;1;0]);
% RW2 spin axis in the body frame
a2 = [1;
      0;
      0] * 1/norm([0;1;0]);
% RW3 spin axis in the body frame
a3 = [0;
      1;
      0] * 1/norm([0;1;0]);
% RW2 spin axis in the body frame
a4 = [-1;
      -1;
      2] * 1/norm([-1;-1;3]);

% Transformation from RW frame to body frame
A = [a1 a2 a3 a4];
num_RW = size(A, 2);     % Number of reaction wheels

% Psudo inverse of A (transformation from body frame to RW frame)
A_psudoinv = pinv(A);       % A^+

% Diagonal matrix of axial reaction wheel inertia 
% Assuming that all the reaction wheels are identical
Js_const = 2.2984e-4;     
Js = eye(num_RW)*Js_const;
Js_inv = inv(Js);

% The following specs are from a RW with a similar Js (4.67 x 10-4 kgm²)
% https://www.tamagawa-seiki.com/products/space/ta6494.html

% Upper and lower limit for applied RW torque (based on the highest value
% seen in the paper)
tau_max = 2e-2;
tau_min = -2e-2;        % 

% Resulting limits for RW angular acceleration [rad/s^2]
alpha_max = tau_max/Js_const;
alpha_min = tau_min/Js_const;

% Upper and lower limit for angular momentum [Nms]
L_max = 0.2;
L_min = -0.2;

% Upper and lower limit for rotational speed [rad/s]
w_max = 4000*2*pi/60; % 4000 rpm
w_min = -4000*2*pi/60;


% RW DC motor specs:
d = 0.1;            % Motor viscous friction constant [Nms]
Ke = 0.1;       % Electromotive force constant [V/rad/s]
Kt = 0.1;       % Motor torque constant [Nm/Amp]
L_motor = 0.5;        % Electric inductance [H]
R_motor = 0.1;    % Electric resistance [ohm]
gear_ratio = 4000*2*pi/(60*25);
J_load = Js_const * gear_ratio^2;

% Motor transfer function w(s)/v(s)
motor_tf = tf([gear_ratio*Ke], [L_motor*J_load, L_motor*d + R_motor*J_load, R_motor*d + Kt^2]);

%step(5*motor_tf)
%grid






%% Satellite (6U cube sat (2 i x-retn) x (1 i y-retn) x (3 i z-retn))
% Inertia matrix
Jxx = 7.75e-2;      % [kgm^2]
Jxy = 2e-4;         Jyx = Jxy;
Jyy = 1.067e-1;
Jyz = 5e-4;         Jzy = Jyz;
Jzz = 3.89e-2;
Jzx = -2e-4;        Jxz = Jzx;
Jb = [Jxx    Jxy     Jxz;
     Jyx    Jyy     Jyz;
     Jzx    Jzy     Jzz];

Jb_inv = inv(Jb);


%% Simulation
% Initial orientation (quaternions)
r_init = transpose([1 0 0 0]);      % The satellite is alligned with the orbit frame

% Satellite initial rotational speed 
wb_init = transpose([0 -2*pi 0]);     % [w_x; w_y; w_z] [rad/s]

% Reference signal (desired) satellite rotational speed 
% in relation to the orbit frame
wb_od = transpose([0 0 0]);      % [rad/s]

% RW initial rotational speed 
ws_init = zeros(num_RW, 1);     % [w_RW1; w_RW2; w_RW3; w_RW4] [rad/s]
