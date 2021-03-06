function [allData, scenario, sensors] = motorCyclist_Scenario()
%motorCyclist_Scenario - Returns sensor detections
%    allData = motorCyclist_Scenario returns sensor detections in a structure
%    with time for an internally defined scenario and sensor suite.
%
%    [allData, scenario, sensors] = motorCyclist_Scenario optionally returns
%    the drivingScenario and detection generator objects.

% Generated by MATLAB(R) 9.6 and Automated Driving Toolbox 2.0.
% Generated on: 28-Mar-2021 03:09:17

% Create the drivingScenario object and ego car
[scenario, egoVehicle] = createDrivingScenario;

% Create all the sensors
[sensors, numSensors] = createSensors(scenario);

allData = struct('Time', {}, 'ActorPoses', {}, 'ObjectDetections', {}, 'LaneDetections', {});
running = true;
while running
    
    % Generate the target poses of all actors relative to the ego vehicle
    poses = targetPoses(egoVehicle);
    time  = scenario.SimulationTime;
    
    objectDetections = {};
    laneDetections   = [];
    isValidTime      = false(1, numSensors);
    
    % Generate detections for each sensor
    for sensorIndex = 1:numSensors
        sensor = sensors{sensorIndex};
        [objectDets, numObjects, isValidTime(sensorIndex)] = sensor(poses, time);
        objectDetections = [objectDetections; objectDets(1:numObjects)]; %#ok<AGROW>
    end
    
    % Aggregate all detections into a structure for later use
    if any(isValidTime)
        allData(end + 1) = struct( ...
            'Time',       scenario.SimulationTime, ...
            'ActorPoses', actorPoses(scenario), ...
            'ObjectDetections', {objectDetections}, ...
            'LaneDetections',   {laneDetections}); %#ok<AGROW>
    end
    
    % Advance the scenario one time step and exit the loop if the scenario is complete
    running = advance(scenario);
end

% Restart the driving scenario to return the actors to their initial positions.
restart(scenario);

% Release all the sensor objects so they can be used again.
for sensorIndex = 1:numSensors
    release(sensors{sensorIndex});
end

%%%%%%%%%%%%%%%%%%%%
% Helper functions %
%%%%%%%%%%%%%%%%%%%%

% Units used in createSensors and createDrivingScenario
% Distance/Position - meters
% Speed             - meters/second
% Angles            - degrees
% RCS Pattern       - dBsm

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = visionDetectionGenerator('SensorIndex', 1, ...
    'SensorLocation', [3.1 1.25], ...
    'Height', 2.5, ...
    'Yaw', 140, ...
    'Pitch', 35, ...
    'MaxRange', 50, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([320 320],[320 240],[480 512]), ...
    'ActorProfiles', profiles);
sensors{2} = radarDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [3.48 1.25], ...
    'Height', 0.25, ...
    'Yaw', 90, ...
    'MaxRange', 80, ...
    'FieldOfView', [150 5], ...
    'ActorProfiles', profiles);
sensors{3} = radarDetectionGenerator('SensorIndex', 3, ...
    'SensorLocation', [-0.62 1.25], ...
    'Height', 0.25, ...
    'Yaw', 90, ...
    'MaxRange', 80, ...
    'FieldOfView', [150 5], ...
    'ActorProfiles', profiles);
numSensors = 3;

function [scenario, egoVehicle] = createDrivingScenario
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [150 0 0;
    75 0 0;
    -20 0 0];marking = [laneMarking('Unmarked')
    laneMarking('Dashed')
    laneMarking('Dashed')];
laneSpecification = lanespec(2, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [0 -5.2 0;
    150 -5.2 0];laneSpecification = lanespec(1, 'Width', 3);
road(scenario, roadCenters, 'Lanes', laneSpecification);

roadCenters = [75 -50 0;
    75 50 0];laneSpecification = lanespec(2);
road(scenario, roadCenters, 'Lanes', laneSpecification);

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 2, ...
    'Length', 5, ...
    'Width', 2.5, ...
    'Height', 3.5, ...
    'Position', [1.5 -1.9 0]);
waypoints = [1.5 -1.9 0;
    56.3 -1.9 0;
    65.3 -1.9 0;
    103.2 -1.9 0];
speed = 10;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
bicycle = actor(scenario, ...
    'ClassID', 3, ...
    'Length', 1.7, ...
    'Width', 0.45, ...
    'Height', 1.7, ...
    'Position', [-10 1.8 0]);
waypoints = [-10 1.8 0;
    49.9 1.8 0;
    100.9 1.8 0];
speed = 20;
trajectory(bicycle, waypoints, speed);

