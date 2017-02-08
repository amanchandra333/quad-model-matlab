function [ output ] = output( input )

Model.constants;

global Wr_old;

W=input(1:4);
rolld=input(5);
pitchd=input(6);
yawd=input(7);
xd=input(8);
yd=input(9);
zd=input(10);
roll=input(11);
pitch=input(12);
yaw=input(13);

V=sqrt(xd^2+yd^2);  % horizontal speed [m/s]
v1=sqrt(-0.5*V^2+sqrt((0.5*V^2)^2+(w/(2*rho*A))^2)); % Inflow velocity [m/s]
lambda=(v1+zd)/(OmegaH*R); % Inflow ratio [less]
mu=V/(OmegaH*R); % advance ratio [less]
muX=xd/(OmegaH*R); % advance ratio in x axis [less]
muY=yd/(OmegaH*R); % advance ratio in y axis [less]

Ct=sigma_*a*(((1/6)+(mu^2)/4)*theta0-((1+mu^2)*thetatw/8)-lambda/4); % Lift coeff [less]
ChX=sigma_*a*((muX*Cd/(4*a))+(0.25*lambda*muX*(theta0-0.5*thetatw))); % [less]
ChY=sigma_*a*((muY*Cd/(4*a))+(0.25*lambda*muY*(theta0-0.5*thetatw))); % [less]
Cq=sigma_*a*((1/(8*a))*(1+mu^2)*Cd+lambda*((theta0/6)-(thetatw/8)-(lambda/4))); % [less]

CrX= - sigma_*a*(muX*(theta0/6-thetatw/8-lambda/8)); % NEGATIVE ! % [less]
CrY= - sigma_*a*(muY*(theta0/6-thetatw/8-lambda/8)); % [less]

T = zeros(P,1);
HX = zeros(P,1);
HY = zeros(P,1);
Q = zeros(P,1);
RRX = zeros(P,1);
RRY = zeros(P,1);

for i=1:P
    % ********* Thrust force *********
T(i,1)=Ct*rho*A*((W(i)*R)^2); % Thrust [N]

% ********* Hub force *********
HX(i,1)=ChX*rho*A*((W(i)*R)^2); % Hub force in X [N]
HY(i,1)=ChY*rho*A*((W(i)*R)^2); % Hub force in Y [N]

% ********* Torque *********
Q(i)=Cq*rho*A*(W(i)^2)*(R^3); % [Nm]

% ********* Rolling moment *********
RRX(i,1)=CrX*rho*A*(W(i)^2)*(R^3); % [Nm]
RRY(i,1)=CrY*rho*A*(W(i)^2)*(R^3); % [Nm]


end

Wr=+W(1)-W(2)+W(3)-W(4); % Om residual propellers rot. speed [rad/s]

% *************** Rotations (in body fixed frame) *************** 
% Roll moments
RgB = pitchd*yawd*(Iyy-Izz);                % Body gyro effect [Nm]
RgP = jr*pitchd*Wr;                           % Propeller gyro effect [Nm] 
RaA = L*(-T(2)+T(4));                           % Roll actuator action [Nm]
RhF = (HY(1)+HY(2)+HY(3)+HY(4))*h;              % Hub force in y axis causes positive roll [Nm]
RrM = +RRX(1)-RRX(2)+RRX(3)-RRX(4);                 % Rolling moment due to forward flight in X [Nm]
RfM = 0.5*Cz*A*rho*rolld*abs(rolld)*L*(P/2)*L;   % Roll friction moment VOIR L'IMPORTANCE [Nm]

% Pitch moments
PgB = rolld*yawd*(Izz-Ixx); % [Nm]
PgP = jr*rolld*Wr; % [Nm]
PaA = L*(-T(1)+T(3)); % [Nm]
PhF = (HX(1)+HX(2)+HX(3)+HX(4))*h; % [Nm]
PrM = +RRY(1)-RRY(2)+RRY(3)-RRY(4);             % Pitching moment due to sideward flight % [Nm]
PfM = 0.5*Cz*A*rho*pitchd*abs(pitchd)*L*(P/2)*L; % Pitch friction moment VOIR L'IMPORTANCE % [Nm]

% Yaw moments
YgB = pitchd*rolld*(Ixx-Iyy); % [Nm]
YiA = jr*(Wr-Wr_old)/sp;            % Inertial acceleration/deceleration produces oposit yawing moment % [Nm]
YawA = +Q(1)-Q(2)+Q(3)-Q(4);               % counter torques difference produces yawing % [Nm]
YhFx = (-HX(2)+HX(4))*L;                    % Hub force unbalance produces a yawing moment % [Nm]
YhFy = (-HY(1)+HY(3))*L; % [Nm]

Wr_old=Wr; % [rad/s]

% *************** Translations (in earth fixed frame) *************** 

% Z forces
ZaA = (cos(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4));          % actuators action [N]
ZaR = PArchim;      % Archimedes force [N]
ZaF = 0.5*Cz*A*rho*zd*abs(zd)*P + 0.5*Cz*Ac*rho*zd*abs(zd);  % friction force estimation (propellers friction+OS4 center friction) [N]

% X forces
XaA = (sin(yaw)*sin(roll)+cos(yaw)*sin(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); % [N] 
XdF = 0.5*Cx*Ac*rho*xd*abs(xd); % [N]
XhF = HX(1)+HX(2)+HX(3)+HX(4); % [N]

% Y forces
YaA = (-cos(yaw)*sin(roll)+sin(yaw)*sin(pitch)*cos(roll))*(T(1)+T(2)+T(3)+T(4)); % [N]
YdF = 0.5*Cy*Ac*rho*yd*abs(yd); % [N]
YhF = HY(1)+HY(2)+HY(3)+HY(4); % [N]

output(1)=(XaA - XdF - XhF)/m;  % x accel [m/s^2]
output(2)=(YaA - YdF - YhF)/m;  % y accel [m/s^2]
output(3)=-g + (ZaR + ZaA - ZaF)/m;  % z accel [m/s^2]
output(4)=(RgB + RgP + RaA + RhF + RrM - RfM) /Ixx; % roll accel [rad/s^2]
output(5)=(PgB - PgP + PaA - PhF + PrM - PfM) /Iyy; % pitch accel [rad/s^2]
output(6)=(YgB + YiA +YawA + YhFx + YhFy) /Izz;  % yaw accel [rad/s^2]


end

