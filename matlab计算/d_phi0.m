
syms phi_0 phi_1 phi_2 phi_4 l1 l2 l3 l4 l5 L0 d_phi1 d_phi4

XD=l5+l4*cos(phi_4);
YD=l4*sin(phi_4);
XB=l1*cos(phi_1);
YB=l1*sin(phi_1);
A0=2*l2*(XD-XB);
B0=2*l2*(YD-YB);
lBD=sqrt((XD-XB)^2+(YD-YB)^2);
C0=l2^2+lBD^2-l3^2;
phi_2=2*atan((B0+sqrt(A0^2+B0^2-C0^2))/(A0+C0));
XC=XB+l2*cos(phi_2);
YC=YB+l2*sin(phi_2);
L0=sqrt((XC-l5/2)^2+YC^2);
phi_0=atan(YC/(XC-l5/2));

J=jacobian([L0;phi_0],[phi_1;phi_4]);
phi0_dot = J(2,1)*d_phi1 +J(2,2)*d_phi4

