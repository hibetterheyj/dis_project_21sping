function [x, y, heading] = odo_enc(x, y, heading, T, Aenc_r, Aenc_l)

% Constants : You need to change these two values to improve your odometry

WHEEL_AXIS      = 0.057;

WHEEL_RADIUS    = 0.020;

% Rad to meter : Convert the wheel encoders units into meters

Aenc_r = Aenc_r*WHEEL_RADIUS ;

Aenc_l = Aenc_l*WHEEL_RADIUS ;

% Comupute speeds : Compute the forward and the rotational speed

v = (Aenc_r+Aenc_l)/(2*T) ; 

omega = (Aenc_r-Aenc_l)/(T*WHEEL_AXIS) ;

% Compute the speed into the world frame (A) 

omega_w = omega ;

vx_w = v*cos(heading) ;

vy_w = v*sin(heading) ;


% Compute the odometry

x = x+vx_w*T ; 

y = y+vy_w*T ;

heading = heading+omega_w*T ;
end