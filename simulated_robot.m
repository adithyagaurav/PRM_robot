robotRadius = 0.4;
robot = ExampleHelperRobotSimulator('simpleMap',2);
robot.enableLaser(false);
robot.setRobotSize(robotRadius);
robot.showTrajectory(true);
mapInflated = copy(robot.Map);
inflate(mapInflated,robotRadius);
prm = robotics.PRM(mapInflated);
prm.NumNodes = 90;
prm.ConnectionDistance = 12;
startLocation = [4.0 3.0];
endLocation = [12.0 10.0];
path = findpath(prm, startLocation, endLocation);
show(prm, 'Map', 'off', 'Roadmap', 'off');
controller = robotics.PurePursuit;
controller.DesiredLinearVelocity = 0.3;
controller.MaxAngularVelocity = 2;
controller.LookaheadDistance = 0.5;

controller.Waypoints = path;
robotCurrentLocation = path(1,:);
robotGoal = path(end,:);
initialOrientation = 0;
robotCurrentPose = [robotCurrentLocation initialOrientation];
robot.setRobotPose(robotCurrentPose);
distanceToGoal = norm(robotCurrentLocation - robotGoal);
goalRadius = 0.1;
controlRate = robotics.Rate(10);
reset(controlRate);
while( distanceToGoal > goalRadius )
    [v, omega] = step(controller, robot.getRobotPose);
    drive(robot, v, omega);
    robotCurrentPose = robot.getRobotPose();
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal);
    waitfor(controlRate);
end
drive(robot, 0, 0);
delete(robot);
