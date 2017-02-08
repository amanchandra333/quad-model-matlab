
sp= 0.01; % [s] sampling period only used to compute YiA term in dinamica.m
g= 9.806;    % [m/s^2]
pi=22/7;
P=4;
%***** Arm length *****/
L= 0.232; 			%// [m]

m = 0.53;	%// [kg]
b= 3.13E-5;       % [N.s2] thrust factor in hover
N=2; % number of blades
R=0.15; % [m] propeller radius
A=pi*(R^2); % [m^2] prop disk area
c=0.0394; % [m]   chord
rho= 1.293;     % [kg/m^3]   air density
w=(m/P)*g;   % [N] weigth of the helicopter/number of propellers


theta0=0.2618;   % [rad] pitch of incidence
thetatw=0.045;   % [rad] twist pitch
sigma_=(N*c)/(pi*R); % solidity ratio (rotor fill ratio) [rad^-1]
a=5.7;  % Lift slope (given in literature)
OmegaH=sqrt(w/b); % [rad/s] prop spd at hover
Cd=0.052;   % Airfoil drag coefficient -- found by tests
Ac=0.005;  % [m^2] helicopter center hub area 


%***** Rotor inertia *****/
r=4;    % reduction ratio
jm= 4e-7;	%// [kg.m2];  
jp= 6e-5;	%// [kg.m2];  
jr = jp+jm/r;	%// [kg.m2];

%***** CoG position *****/
% h= 0.058; 			%// [m] VERTICAL DISTANCE between CoG and propellers plan
h= 0; 			%// [m] more stable with h=0 !!!

% ********* Longitudinal Drag Coefficients *********
Cx=1.32;    % estimation taken from literature [less]
Cy=1.32;    % estimation taken from literature [less]
Cz=1.32;    % estimation taken from literature [less]

%***** Inertia components *****/
Ixx = 6.228E-3;	%// [kg.m2]    
Iyy = 6.228E-3;	%// [kg.m2]
Izz = 1.121E-2;	%// [kg.m2]

% ***** OS4 volume *****
Vol=3.04E-4; 	% [m3]
PArchim = rho*g*Vol;  % [N]

