%% TODO
% Don't bother simulating the rotation of the reaction wheel. 
% From simulink, dont send rotation angle, only angular velocity, and 
% show this simply as an angular velocity vector emenating from the RW
% component in the visualization. This (I think) will also boost the
% visualization performance. 

%% Simulation data
% The Simulink 'RWsim3_paper.slx' must first be run to get simulation data
% The data is stored in the workspace as out.Component_rot and have the
% following structure:
%
%   out.Component_rot.Data(:,1) - Satellite rot about x-axis [rad]
%   out.Component_rot.Data(:,2) - Satellite rot about y-axis [rad]
%   out.Component_rot.Data(:,3) - Satellite rot about z-axis [rad] 
%   out.Component_rot.Data(:,4) - 1. RW (z-axis RW) angular velocity [rad/s]
%   out.Component_rot.Data(:,5) - 2. RW (x-axis RW) angular velocity [rad/s]
% 
% Running this sctipt will interpolate the dataset at run-time
% The variable 'animate' will contain the interpolated values at this time
% and will has the same structure as the aforementioned dataset. 


%% Simulation constants and scaling factors
% TODO: Vis rotasjonshastigheter til de ulike komponentene i sim-vinuet
clf


%[out.Sat_rot.time, out.Sat_rot.signals.values];

%[out.Component_rot.time, out.Component_rot.signals.values(:,1),
%out.Component_rot.signals.values(:,2)]; % Used in the older matlab version (think it was 2022b)

[out.Component_rot.Time, out.Component_rot.Data]; % Must use for matlab 2024a


% Visualisation constants
ScaleFrame_RW = 2; % Scaling factor to ajust the RW frame size
ScaleFrame = 5; % Scaling factor to ajust the satellite frame size
FS = 15; % font size for test
SW = 0.035; % arrow size
%AM = 160; % Angular momentum vector scaling factor
AM = 1e-2; % Angular momentum vector scaling factor
origo = zeros(3,1);
RW_trans = 5; % How mutch the RWs are offset along the satellite x, y or z axis


% Sim time :)
% 1. Run RWinit.m
% 2. Run RWsim.simulink
% 3. Profit?


%% Visialize the sattelite attitude

% Plotting and scheeming (Real-time)
tic; % Reset matlab clock
time_display = 0;
% max(out.Sat_rot.time)
while time_display < max(out.Component_rot.Time)
    time_animate = toc; % like millis()
    % Interpolate the simulation at the current time
    % R = reshape(state_animate(4:12), 3, 3); % orientation
    % omega = R*state_animate(16:18);
    
    %animate = interp1(out.Component_rot.time, out.Component_rot.signals.values, time_animate);
    animate = interp1(out.Component_rot.Time, out.Component_rot.Data, time_animate)
    
    
    % Satellite rotational matrix
    r = [animate(1,3) animate(1,2) animate(1,1)]; % ZYX euler angles
    R = eul2rotm(r); % Rotational matrix describing the satellite's orientation
    % R = expm([  0,      -r(3)   +r(2);
    %         +r(3),  0,      -r(1);
    %         -r(2),  +r(1),   0]); % Rotation matrix describing the satellite orientation'

    
    % X RW rotational matrix and origo
    % xRW_r = [animate(:,5) 0 0]; % Orientation of the reaction wheel (spinning around the x-axis)
    % xRW_R = expm([ 0,     -xRW_r(3),    +xRW_r(2);
    %        +xRW_r(3),            0,    -xRW_r(1);
    %        -xRW_r(2),     +xRW_r(1),    0]); % Rotation matrix describing the reaction wheel orientation
    % yBasisRot = [0 0 1;
    %              0 1 0;
    %              -1 0 0];
    % xRW_orientation = R * xRW_R* yBasisRot;
    yBasisRot = [0 0 1;
                 0 1 0;
                 -1 0 0];
    xRW_orientation = R * yBasisRot;
    xRW_CM = RW_trans*R(:,1);



    % Y RW rotational matrix and origo
    % yRW_r = [0 0 0]; % Orientation of the reaction wheel (spinning around the y-axis)
    % yRW_R = expm([ 0,     -yRW_r(3),    +yRW_r(2);
    %        +yRW_r(3),            0,    -yRW_r(1);
    %        -yRW_r(2),     +yRW_r(1),    0]); % Rotation matrix describing the reaction wheel orientation
    % xBasisRot = [1 0 0;
    %              0 0 1;
    %              0 -1 0];
    % yRW_orientation = R * yRW_R* xBasisRot;
    % yRW_CM = RW_trans*R(:,2);
    xBasisRot = [1 0 0;
                 0 0 1;
                 0 -1 0];
    yRW_orientation = R * xBasisRot;
    yRW_CM = RW_trans*R(:,2);
    


    % Z RW rotational matrix and origo
    %zRW_r = [0 0 (animate(1,2)-animate(1,3))]; % Orientation of the reaction wheel (spinning around the z-axis)
    % zRW_r = [0 0 animate(1,4)];
    % zRW_R = expm([ 0,     -zRW_r(3),    +zRW_r(2);
    %        +zRW_r(3),            0,    -zRW_r(1);
    %        -zRW_r(2),     +zRW_r(1),    0]); % Rotation matrix describing the reaction wheel orientation
    % zRW_orientation = R * zRW_R;
    zRW_CM = RW_trans*R(:,3);

    
    figure(1);
    clf;
    hold on;

    text(12, 2, 0, sprintf('Time: %.2f s', time_animate), 'FontSize', 12, 'Color', 'k');

    % Draw sattelite, body frame and angular momentum vector
    DrawSat(origo, R, 'color', [0.5 0.5 0.5], 'facealpha', 0.5)
    MakeFrame(origo, R, ScaleFrame, FS, SW, 'b', 'color', 'r')
    %MakeArrow(origo, AM * animate(1,4) * R(:,3), FS, 0.05, SW, 'color', [0 0.5 0])

    % Draw all n reaction wheels and angular momentum vectors
    for i = 1:num_RW
        RWi_CM = RW_trans * R * A(:,i);
        DrawRectionWheel( ...
            RWi_CM, ...
            R*eul2rotm([0 atan2(A(1,i), A(3,i)) -atan2(A(2,i), A(3,i))]), ...
            'color', [0.5 0.5 0.5] ...
        )

       MakeArrow(RWi_CM, AM * animate(1,3+i) * R * A(:,i), FS, 0.05, SW, 'color', [0 0.5 0])
    end

   
    % % Draw X reaction wheel and corresponding frame
    % DrawRectionWheel(xRW_CM, xRW_orientation, 'color', [0.5 0.5 0.5])
    % %MakeFrame(xRW_CM, xRW_orientation, ScaleFrame_RW, FS, SW, 'b', 'color', 'r')
    % MakeArrow(xRW_CM, AM * animate(1,5) * R(:,1), FS, 0.05, SW, 'color', [0 0.5 0])
  
    % mArrow3(1*R(:,2),5*R(:,2), 'stemWidth', 4*SW) % Arrow showing the pointing vector of the satellite
    
    FormatPicture([0;0;2], 0.5*[73.8380 21.0967 30.1493]) % Create a visualisation of all drawn elements

%     if time_display == 0
%         display('Hit a key to start animation')
%         pause
%         tic
%     end
    time_display = toc; % Get current time

    
end
