rosinit;
%Publisher  publishing  to the control theta 
%subscriber subscribing to simulation theta values
%custom message file simulation/theta
pub_name=rospublisher("/controltheta","simulation/theta");
sub_name=rossubscriber("/thetafeedback",@callbacktheta)
pause(2)
%Creating a ROS msg holder 
theta=rosmessage(pub_name);

%Callback function for theta feedback from Gazebo
function callbacktheta(msg)
    xyz=msg.position
end

while true:
         % set theta1
        error_theta(1,i+1)   = theta1_req-instant_theta(1,i) ;
        sum_error_theta(1,1) = error_theta(1,1);
        for v=1:+1:i
            sum_error_theta(1,v+1) = sum_error_theta(1,v) + error_theta(1,v+1);
            dif_error_theta(1,v+1) = error_theta(1,v+1) - error_theta(1,v);
        end 
        delta_theta(1,i+1)   = kp*error_theta(1,i+1) + ki*sum_error_theta(1,i+1) + kd*dif_error_theta(1,i+1);    
        instant_theta(1,i+1) = instant_theta(1,i) + delta_theta(1,i+1);  
        
         % set theta2
        error_theta(2,i+1)   = theta2_req-instant_theta(2,i) ;sum_error_theta(1,1) = error_theta(1,1);    
        sum_error_theta(2,1) = error_theta(2,1);
        for v=1:+1:i
            sum_error_theta(2,v+1) = sum_error_theta(2,v) + error_theta(2,v+1);
            dif_error_theta(2,v+1) = error_theta(2,v+1) - error_theta(2,v);
        end 
        delta_theta(2,i+1)   = kp*error_theta(2,i+1) + ki*sum_error_theta(2,i+1) + kd*dif_error_theta(2,i+1);    
        
        instant_theta(2,i+1) = instant_theta(2,i) + delta_theta(2,i+1); 
        
         % set theta3
        error_theta(3,i+1)   = theta3_req-instant_theta(3,i) ;
        sum_error_theta(3,1) = error_theta(3,1);
        for v=1:+1:i
            sum_error_theta(3,v+1) = sum_error_theta(3,v) + error_theta(3,v+1);
            dif_error_theta(3,v+1) = error_theta(3,v+1) - error_theta(3,v);
        end 
        delta_theta(3,i+1)   = kp*error_theta(3,i+1)+ ki*sum_error_theta(3,i+1) + kd*dif_error_theta(3,i+1);
        instant_theta(3,i+1) = instant_theta(3,i) + delta_theta(3,i+1);

        theta.theta1=         %place for input for theta1
        theta.theta2=         %place for input for theta2
        theta.theta3=         %place for input for theta3
        send(pub_name,theta
end 
        
