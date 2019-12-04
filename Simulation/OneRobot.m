%% Create Map & Boundary
map = binaryOccupancyMap(20,20,10);

% Create Object
numObject = 1;
sizeObject = 0.8;
% object = 6*rand(numObject, 2)+2;
object = [5 4];
setOccupancy(map, object, ones(numObject,1));
inflate(map, sizeObject)

% Create Boundary
sizeBoundary = 10;
MapBoundary_x = linspace(0,sizeBoundary);
MapBoundary_x = transpose(MapBoundary_x);
MapBoundary_y = zeros(size(MapBoundary_x));

setOccupancy(map,[MapBoundary_x MapBoundary_y], 1);
setOccupancy(map,[MapBoundary_y MapBoundary_x], 1);

MapBoundary_y = zeros(size(MapBoundary_x))+sizeBoundary;
setOccupancy(map,[MapBoundary_x MapBoundary_y], 1);
setOccupancy(map,[MapBoundary_y MapBoundary_x], 1);

axis([0 10 0 10]);
% figure
% show(map)

%% Multi-Robot Env
numRobots = 1;
sizeRobot = 0.3;
env = MultiRobotEnv(numRobots);
env.robotRadius = sizeRobot*ones(numRobots,1);
env.showTrajectory = ones(numRobots,true);
env.mapName = 'map';
env.hasWaypoints = true;

%% Sensor Define
lidar = MultiRobotLidarSensor;
lidar.robotIdx = 1;
lidar.sensorOffset = [0,0];
lidar.scanAngles = linspace(-26.6*pi/180,26.6*pi/180,25);
lidar.maxRange = 2.5;
attachLidarSensor(env,lidar);

%% Obstacle Detection


%% Define Vehicle, initial position, Destination
% waypoints = [8 8];
objects = [2, 8, 1;
           8, 11, 2;
           8, 4, 3];
env.objectColors = [1 0 0;0 1 0;0 0 1];
env.objectMarkers = 'so^';

% Sample time [s]
sampleTime = 0.1;               
tVec = 0:sampleTime:25; 

% Create a DifferentialDrive object
R = 0.1; % Wheel radius [m]
L = 0.5; % Wheelbase [m]
vehicle1 = DifferentialDrive(R,L);

global pose1;
pose1 = [1;1;pi/4];
pose2 = [2;8;-pi];
pose3 = [10;10;0];
env.Poses = [pose1 pose2 pose3];

des_point = [8 8;];


%% Potential Define
% Obstacle Potential
forward_K_obs= 9000;
K_obs = 0;
obstacle = [0 0];
mu = 0 ; % One variable = distance bet robot and object
sigma = 0.1 ;
gm = gmdistribution(mu,sigma);
% 무한대에서 1까지 확률
% obstacle_dist = dist_xy([pose1(1,1) pose1(2,1)], obstacle);
% obstacle_dist = sum(cdf(gm,obstacle_dist)*obstacle_dist);
fun_obs = @(x) K_obs*sum(cdf(gm, dist_xy([x(1) x(2)], obstacle)));

% Obstacle 없는 위치로 dir 가중치
forward_K_dir = -9000;
K_dir = -0.1;
nanAngle1 = [];
fun_dir = @(x) K_dir* sum((nanAngle1-atan((x(2)-pose1(2,1))/(x(1)-pose1(1,1)))).^2));

% Potential to Destination
K_des = 0.0001;
fun_des = @(x) K_des*((des_point(1,1)-x(1))^2+(des_point(1,2)-x(2))^2);

% W Limitation -> function
% Lidar Detection -> function
K_speed = -0.1;
fun_speed = @(x) K_speed*((x(1)-pose1(1,1))^2+(x(2)-pose1(2,1))^2);

% Total Potential Derivative 
fun_total = @(x) (K_des*((x(1)-des_point(1,1)).^2+(x(2)-des_point(1,2)).^2) + K_obs*sum(1-cdf(gm, ((x(1)-obstacle(:,1)).^2 + (x(2)-obstacle(:,2)).^2).^(1/2))) + K_dir* sum((nanAngle1-atan((x(2)-pose1(2,1))/(x(1)-pose1(1,1)))).^2));

% Velocity Limitation - Condition
v_limit = 1;

% find min
% x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
% x = fmincon(fun_total,[0,0],[],[],[],[],[0,0],[10, 10]);

%% Visualize
% poses = 4*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
% env.Poses = poses;
dTheta = pi/64;
ranges = cell(1,numRobots);
r = rateControl(1/sampleTime);
for idx = 2:40
    % Get the current time step's ranges
    ranges1 = lidar();
    scanAngle1 = lidar.scanAngles;
    scanAngle1 = transpose(scanAngle1);
    
    % find NaN scanAngle
    nanAngle1 = ismissing(ranges1).*scanAngle1;
    nanAngle1 = pose1(3,1) - nanAngle1;
    
    % Check No Obstacle forward
    if(sum(ismissing(ranges1))==size(ranges1,1))
        K_dir = 0
    else
        K_dir = forward_K_dir
        K_obs = forward_K_obs;
    end
    if any(~isnan(ranges1)),
        disp(ranges1.');
    end
    % Update the environment and poses
    env(1, pose1, des_point, ranges1)
    
    % Add Obstacle Position
    new_obstacle = [pose1(1,1) + ranges1.*cos(pose1(3,1) + scanAngle1), pose1(2,1) + ranges1.*sin(pose1(3,1) + scanAngle1)];
    if(isempty(obstacle))
        new_obstacle = [];
    else
        new_obstacle =   remove_overlap(obstacle, new_obstacle);
        obstacle = [obstacle; new_obstacle];
    end
    obstacle = rmmissing(obstacle);
%     obstacle = unique(obstacle);
    
    % Compute next x, y for minimizing Potential function
    [pose1_next,fval] = fmincon(fun_total,[pose1(1,1) pose1(2,1)],[1 0;0 1],[10;10],[],[],[0 0],[10 10], @con_v_limit)
    pose1_next = transpose(pose1_next)
    % Compute v, w
    dx1 = pose1_next(1,1)-pose1(1,1);
    dy1 = pose1_next(2,1)-pose1(2,1);
    v = ((dx1)^2+(dy1)^2)/sampleTime;
    if (dx1 == 0 )
        dtheta1 = pose1(3,1);
    elseif (atan(dy1/dx1)<0)
        dtheta1 = atan(dy1/dx1) + pi;
    else 
        dtheta1 = atan(dy1/dx1);
    end
%     w = (dtheta1-pose1(3,1))/sampleTime;
    pose1_next(3,1) = dtheta1;
%     [wL,wR] = inverseKinematics(dd,v,w); % for actual navigation
    
    % Compute the velocities from measurement
%     velB = [v;0;w]; % Body velocities [vx;vy;w]
%     vel = bodyToWorld(velB,pose1(:,1));  % Convert from body to world
    
    % Perform forward discrete integration step from measurement [x y theta]
%     pose1(:,1) = pose1(:,1) + vel*sampleTime; 
    pose1(:,1) = pose1_next(:,1);
    
%     poses(3,3) = poses(3,3) + dTheta;   
    if(((pose1(1,1)-des_point(1,1))^2 + (pose1(2,1)-des_point(1,2))^2)^(1/2)) < 0.3
        break
    end
    waitfor(r);
end

%% Def function

% x1, x2 (mx2 matrix 
function y = dist_xy(x1, x2)
    y = ((x1(1,1)-x2(:,1)).^2 + (x1(1,2)-x2(:,2)).^2).^(1/2);
end

function rem_overlap = remove_overlap(old, new);
    overlap_limit = 0.3;
    rem_index = [];
    for x = 1:size(new(:,1))
        if min(sum(abs(old - new(x,:)),2)) < overlap_limit
            rem_index = [rem_index, x]; 
        end
    end
    new(rem_index,:) = [];
    rem_overlap = new;
end

function [c,ceq] = con_v_limit(x)
global pose1;
% ((x(1)-pose1(1,1))^2+(x(2)-pose1(2,1))^2)^(1/2)

%     abs(pose1(3,1)-atan((x(2)-pose1(2,1))/(x(1)-pose1(1,1))))-pi/2
ceq = [];
c = -0.1+((x(1)-pose1(1,1))^2 + (x(2)-pose1(2,1))^2);

end
% function y = dist_lidar(r, theta, pose_new)
%     y = [theta ];
% end
