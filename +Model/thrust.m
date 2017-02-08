function [ output ] = thrust( input )

w1 = input(1);
w2 = input(2);
w3 = input(3);
w4 = input(4);
b  = input(5);
l  = input(6);
d  = input(7);

%global t1 t2 t3 t4;%
t1 = b*w1*w1;
t2 = b*w2*w2;
t3 = b*w3*w3;
t4 = b*w4*w4;

thrusttotal = t1 + t2 + t3 + t4;
rollmoment = l*(t3-t4);
pitchmoment = l*(t1-t2);
yawmoment = d*(-t1+t2-t3+t4)/b;
Wr = -w1+w2-w3+w4;
output(1)=thrusttotal;
output(2)=rollmoment;
output(3)=pitchmoment;
output(4)=yawmoment;
output(5)=Wr;

end

