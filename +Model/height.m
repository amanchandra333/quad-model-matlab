function [ output ] = height( input )
roll=input(1);
pitch=input(2);
thrust=input(3);
g=input(4);
m=input(5);
yaw=input(6);

xdd = ((sin(yaw)*sin(roll)+cos(yaw)*cos(roll)*sin(pitch))*thrust)/m;
ydd = ((sin(yaw)*sin(pitch)+cos(yaw)*cos(pitch)*sin(roll))*thrust)/m;
zdd = g - cos(roll)*cos(pitch)*thrust/m;
output(1)=xdd;
output(2)=ydd;
output(3)=zdd;
end

