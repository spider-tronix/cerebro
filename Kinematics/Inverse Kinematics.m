%Inverse Kinematic equations for a 3dof arm

l1=0.2
l2=0.2  %IDEAL CASE

L=sqrt(l1^2)+(l2^2))

x=0.02;
y=0.02;
z=0.02;

%Planar component
t=sqrt(x^2+y^2);

%Revolute angle sum and diff
theta12diff= acos((z^2+t^2-L^2)/(2*l1*l2))
theta12summ= acos((z^2-t^2-L^2)/(2*l1*l2))

%Individual angle obtained using arccosine

theta1= (theta12diff+theta12summ)/2
theta2= (theta12diff-theta12summ)/2
theta3= acos(y/(l1*cos(theta1)+l2*cos(theta2)))
