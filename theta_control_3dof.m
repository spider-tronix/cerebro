%Inverse Kinematic equations for a 3dof arm

l1=0.2;
l2=0.2;  %IDEAL CASE

L=sqrt((l1^2)+(l2^2));

x=0;
y=0;
z=0.4;

%Planar component
t=sqrt(x^2+y^2);

%Revolute angle sum and diff
theta12diff= acos((z^2+t^2-L^2)/(2*l1*l2));
theta12summ= acos((z^2-t^2-L^2)/(2*l1*l2));

%Individual angle obtained using arccosine

theta1_req= (theta12diff+theta12summ)/2
theta2_req= (theta12diff-theta12summ)/2
theta3_req= acos(y/(l1*cos(theta1_req)+l2*cos(theta2_req)))

kp = 1.2;
i = 1; 
N = 14;

error_theta = zeros(3,N);
delta_theta = zeros(3,N);
instant_theta = zeros(3,N);

while i < N
    
    error_theta(1,i+1)   = theta1_req-instant_theta(1,i) ;
    delta_theta(1,i+1)   = kp*error_theta(1,i+1);
    instant_theta(1,i+1) = instant_theta(1,i) + delta_theta(1,i+1);  
    
    
    error_theta(2,i+1)   = theta2_req-instant_theta(2,i) ;
    delta_theta(2,i+1)   = kp*error_theta(2,i+1);
    instant_theta(2,i+1) = instant_theta(2,i) + delta_theta(2,i+1); 
    
    
    error_theta(3,i+1)   = theta3_req-instant_theta(3,i) ;
    delta_theta(3,i+1)   = kp*error_theta(3,i+1);
    instant_theta(3,i+1) = instant_theta(3,i) + delta_theta(3,i+1); 
    
    plot(instant_theta(3,:))
    i = i+1;
end   
