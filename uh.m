

%% Euler angles [xrot, yrot, zrot] <--> Quaternions

eulerang = [4/9*pi 5/18*pi 17/18*pi];
n = eul2rotm(eulerang, 'XYZ')
qn = rotm2quat(n)

% Denne metoden fungerte ikke!! Jeg skjønner ikke hvorfor denne gir riktige
% rotasjonsmatriser i visualiseringskoden, men ikke her
xRW_R = expm([ 0,     -eulerang(3),    +eulerang(2);
   +eulerang(3),            0,    -eulerang(1);
   -eulerang(2),     +eulerang(1),    0]);

%% Quaternions --> rotmat

quat = [ 0.462 0 0  0.887];
rotmateq = quat2rotm(quat)
eulang = quat2eul(quat)
deg = eulang * 180/pi


%% 
for i = 1:num_RW
    A(:,i)
    aa3 = a3
end
