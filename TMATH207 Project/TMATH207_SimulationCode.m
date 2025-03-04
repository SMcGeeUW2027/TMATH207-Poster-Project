% Shea McGee  3/1/25
%ALL CREDIT GOES TO NICK HAUGER AND TAN NGUYEN OF UW TACOMA
%Github Repository: https://github.com/Neeezha/AutoFerry/tree/nzh-branch
%sin and cos when using degrees
%Indexing Matrices: A(#,:) gives entire row
                   %A(:,#) Gives entire column
                   %A(2:3,#) Gives 2nd-3rd rows, #th column
clear variables; clc;close all 

dt = 0.1;
ttotal = 0:dt:10000;
N = length(ttotal);
%Position vectors
x=zeros(N,1); y=zeros(N,1); theta_motor=zeros(N,1); theta_f = zeros(N,1);
%error vectors 
x_error=zeros(N,1); y_error=zeros(N,1); theta_error = zeros(N,1);
%wanted position vectors
x_desired=zeros(N,1); y_desired=zeros(N,1); %theta_desired = zeros(N,1);
%rate of change vectors
x_derivative = zeros(N,1); y_derivative = zeros(N,1); theta_derivative = zeros(N,1);
%heading control vector?
u = zeros(N,1);

%constants
V_motor = 20;
b = 1.5;
control_gain = 1;

%Vel magnitudes/direction
V_current = 5; theta_current = 60; %water current/direction

%Waypoints on the picture
waypoints = [100 330; 125 300; 150 300; 200 465; 300 700; 325 685; 400 400; 600 600;750 775; 945 800; 750 775; 600 600;400 400;325 685;300 700;200 465;150 300;125 300;100 330];
%initial position/angle
theta_motor(1) = 0;
x(1) = waypoints(1,1); y(1) = waypoints(1,2);
x_desired(1) = waypoints(2,1); y_desired(1) = waypoints(2,2);

%counter for waypoints
waypoints_reached = 1;
waypoints_size = length(waypoints);

for k = 2:(N)
    V_motor_x = V_motor*cosd(theta_motor(k-1));
    V_motor_y = V_motor*sind(theta_motor(k-1));
    V_current_x = V_current*cosd(theta_current);
    V_current_y = V_current*sind(theta_current);
    
    %Finding total Velocity (magnitude)
    V_fx = V_motor_x + V_current_x;
    V_fy = V_motor_y + V_current_y;

    V_f = sqrt((V_fx^2) + (V_fy^2));
    theta_f(k) = atan2d(V_fy,V_fx);
     
    %Finding the derivative to find distance over time with the final
    %velocity times the angle the boat needs to be pointed at.
    x_derivative(k) = V_f.*cosd(theta_f(k)); 
    y_derivative(k) = V_f.*sind(theta_f(k));
    theta_derivative(k) = b*(u(k-1));

    %calculating next point
    theta_motor(k) = theta_motor(k-1) + theta_derivative(k)*dt;
    x(k) = x(k-1) + x_derivative(k)*dt; %EULERS EQUATIONS   
    y(k) = y(k-1) + y_derivative(k)*dt; %EULERS EQUATIONS

    %finding error
    x_error(k) = x_desired(k-1) - x(k);
    y_error(k) = y_desired(k-1) - y(k);

    %Figures out where the next waypoint will be
    distance = sqrt(x_error(k)^2 + y_error(k)^2);

    if distance < 4.8 && waypoints_reached < waypoints_size
        waypoints_reached = waypoints_reached + 1;
        x_desired(k) = waypoints(waypoints_reached, 1);
        y_desired(k) = waypoints(waypoints_reached, 2);

    elseif waypoints_reached == waypoints_size
        waypoints_reached = 1;
        x_desired(k) = x_desired(k-1);
        y_desired(k) = y_desired(k-1);
    else
        x_desired(k) = x_desired(k-1);
        y_desired(k) = y_desired(k-1);
    end



    theta_desired = atan2d(y_error(k),x_error(k));

    theta_motor(k) = degree_bounder(theta_motor(k),theta_motor(k));
    theta_desired = degree_bounder(theta_desired, theta_desired - theta_f(k));



    theta_error(k) = theta_desired - theta_f(k);

    u(k) = control_gain * theta_error(k);
end
set(groot, 'defaultFigureWindowState', 'maximized');
figure(1);
hold on;

axis([0 1000 0 1000])
picture = imread('Ferry_outline_TMATH207.png');
background = image(xlim, flip(ylim),picture);
uistack(background,'bottom')

%making the ships body
shipbody_x = [1,0,-1,-1,0,1];
shipbody_y = [0,.5,.5,-.5,-.5,0];
s = hgtransform;
patch('XData',shipbody_x*7.5,'YData',shipbody_y*7.5,'Parent',s)

ship_trail = plot([x(1) x(2)], [y(1) y(2)],'--');
current_waypoint = plot(x_desired(1),y_desired(1),'X',MarkerSize=20, LineWidth=5);

for i = 1:N-1
    s.Matrix = makehgtform('translate', [x(i+1) y(i+1) 0], 'zrotate', theta_motor(i)*pi/180);

    set(ship_trail,'XData',x(1:i));
    set(ship_trail,'YData',y(1:i));

    set(current_waypoint,'XData',x_desired(i));
    set(current_waypoint,'YData',y_desired(i));

    drawnow

end

%%%%%%CREDIT: https://github.com/Neeezha/AutoFerry/tree/nzh-branch %%%%%%%