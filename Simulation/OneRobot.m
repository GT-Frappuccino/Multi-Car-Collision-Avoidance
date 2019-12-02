%% Create Map & Boundary
map = binaryOccupancyMap(10,10,10);

% Create Object
numObject = 8;
sizeObject = 0.8;
object = 6*rand(numObject, 2)+2;
setOccupancy(map, object, ones(numObject,1));
inflate(map, sizeObject)

% Create Boundary
MapBoundary_x = linspace(0,10);
MapBoundary_x = transpose(MapBoundary_x);
MapBoundary_y = zeros(size(MapBoundary_x));

setOccupancy(map,[MapBoundary_x MapBoundary_y], 1);
setOccupancy(map,[MapBoundary_y MapBoundary_x], 1);

MapBoundary_y = zeros(size(MapBoundary_x))+10;
setOccupancy(map,[MapBoundary_x MapBoundary_y], 1);
setOccupancy(map,[MapBoundary_y MapBoundary_x], 1);
% inflate(map, 0.1)




% figure
% show(map)

%% Multi-Robot Env
numRobots = 1;
env = MultiRobotEnv(numRobots);
env.robotRadius = 0.5*ones(numRobots,1);
env.showTrajectory = ones(numRobots,true);
env.mapName = 'map';

%% Sensor Define
lidar = MultiRobotLidarSensor;
lidar.robotIdx = 1;
lidar.scanAngles = linspace(-pi,pi,25);
lidar.maxRange = 4;
attachLidarSensor(env,lidar);

%% Obstacle Detection


%% Define Vehicle, initial position, Destination
% waypoints = [2 4;
%              12 12;
%              1 11];
% objects = [2, 8, 1;
%            8, 11, 2;
%            8, 4, 3];
% env.objectColors = [1 0 0;0 1 0;0 0 1];
% env.objectMarkers = 'so^';

% Sample time [s]
sampleTime = 0.1;               
tVec = 0:sampleTime:25; 

% Create a DifferentialDrive object
R = 0.1; % Wheel radius [m]
L = 0.5; % Wheelbase [m]
vehicle1 = DifferentialDrive(R,L);

pose1 = [0;0;pi/2];
pose2 = [2;8;-pi];
pose3 = [10;10;0];
env.Poses = [pose1 pose2 pose3];

des_point = [8;7];


%% Potential Define

% Potential to Destination
K_des = 5;
fun_des = @(x) K_des*((des_point(1,1)-x(1))^2+(des_point(2,1)-x(2))^2);

% W Limitation -> function
% Lidar Detection -> function
K_speed = -0.3;
fun_speed = @(x) K_speed*((x(1)-pose1(1,1))^2+(x(2)-pose1(2,1))^2);

% Total Potential Derivative re!!
fun_total = @(x) (K_des*((x(1)-des_point(1,1)).^2+(x(2)-des_point(2,1)).^2) + K_speed*((x(1)-pose1(1,1)).^2+(x(2)-pose1(2,1)).^2));

% Velocity Limitation - Condition
v_limit = 1;
con_v_limit = @(x) ((x(1)-pose1(1,1))^2+(x(2)-pose1(2,1))^2)/sampleTime - v_limit;

% find min
% x = fmincon(fun,x0,A,b,Aeq,beq,lb,ub)
% x = fmincon(fun_total,[0,0],[],[],[],[],[0,0],[10, 10]);

%% Visualize
% poses = 4*(rand(3,numRobots).*[1;1;pi] - [0.5;0.5;0]) + [9;9;0];
% env.Poses = poses;
dTheta = pi/64;
ranges = cell(1,numRobots);
r = rateControl(1/sampleTime);
for idx = 2:5
    % Get the current time step's ranges
%     scans = lidar();
%     ranges{numRobots} = scans;
    
    % Update the environment and poses
    env(numRobots, pose1, {})
     
    % Compute next x, y for minimizing Potential function
    [pose1_next,fval] = fmincon(fun_total,[0,0],[],[],[],[],[0,0],[10, 10]);
    pose1_next = transpose(pose1_next);
    % Compute v, w
    dx1 = pose1_next(1,1)-pose1(1,1);
    dy1 = pose1_next(2,1)-pose1(2,1);
    v = ((dx1)^2+(dy1)^2)/sampleTime;
    if (atan(dy1/dx1)<0)
        dtheta1 = atan(dy1/dx1) + pi;
    else
        dtheta1 = atan(dy1/dx1);
    end
    w = (dtheta1-pose1(3,1))/sampleTime;
    pose1_next(3,1) = dtheta1;
    [wL,wR] = inverseKinematics(dd,v,w); % for actual navigation
    
    % Compute the velocities from measurement
    velB = [v;0;w]; % Body velocities [vx;vy;w]
    vel = bodyToWorld(velB,pose1(:,1));  % Convert from body to world
    
    % Perform forward discrete integration step from measurement [x y theta]
    pose1(:,1) = pose1(:,1) + vel*sampleTime; 
    pose1(:,1) = pose1_next(:,1);
    
%     poses(3,3) = poses(3,3) + dTheta;    
    waitfor(r);
end
