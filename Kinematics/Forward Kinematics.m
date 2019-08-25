%Forward Kinematics for simple 3 dof freedom arm.

l1=0.2; %Theoretically zero 
l2=0.3;
theta1=pi/8;
theta2=pi/8;
theta3=pi/8;

%Basic inverse Forward equations with 2 dof base 

%Projections across the xy-plane
t=l1*cos(theta1);
m=l2*cos(theta2);

%X,Y co-ordinates of the end effector
x=(t+m)*cos(theta3);
y=(t+m)*sin(theta3);

%Z co-ordinate of the end effector
z=l1*sin(theta1)+l2*sin(theta2)
