function combinedcontroller2_rtpp()
% This script uses a combination of Real-Time Local Path Planning and 
% Pure Pursuit control. The local path is continually re-planned toward the 
% next global waypoint, incorporating dynamic obstacle avoidance placeholders.
% Robot position feedback is received via MQTT.
% Control commands are sent via TCP/IP.
    
    % Clear workspace and close all figures for a fresh start
    clearvars; close all;
    
    % SETUP FOR ROUTE AND LIMO
    % Prompt for route selection with validation
    chosenRoute = false;
    while ~chosenRoute
        routeInput = input('Select course 1, 2, or 3 for simulation: ', 's');
        routeNumber = str2double(routeInput);
        if ismember(routeNumber, [1, 2, 3])
            chosenRoute = true;
        else
            fprintf('Please enter 1, 2, or 3.\n');
        end
    end
    
    fprintf('\n--- LIMO Network Configuration (Hardcoded) ---\n');
    % --- LIMO IP Address Last 3 Digits ---
    limoIpSuffix = '175';
    fprintf('IP last 3 digits: %s\n', limoIpSuffix);
    % --- LIMO Number ---
    limoIdNumber = '809';
    fprintf('LIMO number: %s\n', limoIdNumber);
    
    % Set obstacles, goal, and nominal path based on selected route
    if routeNumber == 1
        % Route 1
        fprintf('Loading Route 1 configuration...\n');
        % Global Waypoints (High-Level Plan)
        globalXWaypoints = [0.0, .5, .5, 1.0, 2.0, 3.0, 4.0, 5.0, 5.0, 5.0];
        globalYWaypoints = [0, 1.0, 2.0, 3.0, 2.5, 1.5, 1.5, 2.5, 3.5, 4.5];
        
        % Obstacles (Static Obstacles)
        obstacleMap = [1.5, 1.5, 1.5, 1.5, 1.5, 1.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5;
                       0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 4.5, 4.0, 3.5, 3.0, 2.5, 2.0];
        % Final destination [x; y]
        courseGoal = [5.0; 4.5];
    elseif routeNumber == 2
        % Route 2
        fprintf('Loading Route 2 configuration...\n');
        % Global Waypoints (High-Level Plan)
        globalXWaypoints = [0, 0.5, 1.5, 2.5, 3.5, 4.5, 5.0];
        globalYWaypoints = [0, 1.0, 2.0, 3.0, 3.5, 4.0, 4.5];
      
        % Obstacles (Static Obstacles)
        obstacleMap = [1.0, 2.5, 4.0, 0.0, 1.5, 3.0, 1.0, 2.5, 4.0;
                       1.0, 1.0, 1.0, 2.5, 2.5, 2.5, 4.0, 4.0, 4.0];
        % Final destination [x; y]
        courseGoal = [5.0; 4.5];
        
    elseif routeNumber == 3
        % Route 3
        fprintf('Loading Route 3 configuration...\n');
        % Global Waypoints (High-Level Plan)
        globalXWaypoints = [0, 1.0, 2.0, 3.0, 4.0, 5.0, 4.5, 4.5, 3.5, 2.5, 1.5, 0.5, 0];
        globalYWaypoints = [0, 0.5, 1.0, 0.5, 0.5, 1.5, 2.5, 3.5, 4.0, 4.0, 3.0, 2.0, 1.5];
        
        % Obstacles (Static Obstacles)
        obstacleMap = [0.0, 0.5, 1.0, 1.0, 1.0, 1.5, 2.0, 2.5, 2.5, 2.5, 2.5, 3.0, 3.5, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 0.0, 0.5, 1.0, 2.5, 2.5;
                       1.0, 1.0, 1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 2.5, 3.0, 3.5, 3.5, 3.5, 3.5, 3.0, 2.5, 2.0, 1.5, 1.0, 3.5, 3.5, 3.5, 0.0, 0.5];
        % Final destination [x; y]
        courseGoal = [0.0; 1.5];
        
    else
        error('Invalid route number selected.'); 
    end
    
    % Display summary
    fprintf('  Obstacles: %d points\n', size(obstacleMap, 2));
    fprintf('  Goal: [%.1f, %.1f]\n', courseGoal(1), courseGoal(2));
    fprintf('  Global waypoints: %d points\n\n', length(globalXWaypoints));
    
    %% PARAMETERS
    % Configuration structure (use ALL_CFG for clarity)
    ALL_CFG.limoIpPrefix = '192.168.1.';
    ALL_CFG.limoTcpPort = 12345;
    ALL_CFG.mqttBroker = 'mqtt://rasticvm.lan';
    % Motion Capture (MoCap) Origin for Coordinate Transformation
    ALL_CFG.mocapOriginX = -4.5;
    ALL_CFG.mocapOriginY = 2.5;
    
    % Pure Pursuit Controller Parameters
    ALL_CFG.lookaheadDist = 0.35;         % Lookahead distance [m]
    ALL_CFG.targetLinearVel = 0.25;      % Desired constant forward velocity [m/s]
    
    % **NEW: Reverse Avoidance Parameters**
    ALL_CFG.reverseLinearVel = -0.15;     % Desired reverse velocity [m/s]
    ALL_CFG.reverseTurnOmega = -0.5;      % Angular velocity for reverse-and-turn (right turn) [rad/s]
    ALL_CFG.reverseTurnTime = 1.5;        % Time threshold before initiating a turn during reverse [s]
    ALL_CFG.reverseOnStuckError = 0.5;    % Cross-track error threshold to trigger reverse [m]
    
    % **NEW: Real-Time Planning Parameters**
    ALL_CFG.planningHorizon = 1.5;        % Distance for local path planning [m]
    ALL_CFG.planningRes = 0.05;           % Resolution of the generated local path [m]
    
    % Global Path Parameters (For waypoint management)
    ALL_CFG.globalWaypoints = [globalXWaypoints; globalYWaypoints]; % Store for loop
    ALL_CFG.minTurnRadius = 0.2;       % Minimum turning radius [m] - Needed for local Dubins/Reeds-Shepp
    
    % Goal Reached Tolerance
    ALL_CFG.posTolerance = 0.2;         % Position tolerance to consider goal reached [m]
    ALL_CFG.headingTolerance = deg2rad(25);% Heading tolerance [rad]
    
    % Safety Limits
    ALL_CFG.maxLinVel = 0.25;             % Maximum linear velocity [m/s]
    ALL_CFG.maxAngVel = deg2rad(60);     % Maximum angular velocity [rad/s]
    ALL_CFG.maxTotalTime = 360;          % Maximum script execution time [s]
    
    % Control Loop Timing
    ALL_CFG.controlRate = 30;            % Control loop frequency [Hz]
    ALL_CFG.dt = 1/ALL_CFG.controlRate;  % Time step [s]
    
    % **NEW: Moving Obstacle Placeholder**
    % This structure will be populated by an obstacle detection system (e.g., Lidar, MoCap)
    % Format: Obstacles.x = [x1, x2, ...]; Obstacles.y = [y1, y2, ...];
    ALL_CFG.movingObstacles = struct('x', [], 'y', []); 
    ALL_CFG.staticObstacles = obstacleMap; % Static obstacles are still used
    
    % Initialize communication objects
    tcpConnection = [];
    mqttHandle = [];
    
    try
        %% LIMO TCP CONNECTION 
        fullLimoIp = [ALL_CFG.limoIpPrefix limoIpSuffix];
        fprintf('Connecting to LIMO TCP: %s:%d\n', fullLimoIp, ALL_CFG.limoTcpPort);
        tcpConnection = tcpclient(fullLimoIp, ALL_CFG.limoTcpPort, 'Timeout', 5);
        write(tcpConnection, uint8('0.00,0.00'));
        pause(0.5);
        
        %% MQTT CONNECTION 
        fprintf('Connecting to MQTT broker: %s\n', ALL_CFG.mqttBroker);
        mqttHandle = mqttclient(ALL_CFG.mqttBroker);
        subscribe(mqttHandle, sprintf("rb/limo%s", limoIdNumber));
        
        % Wait for MoCap data
        fprintf('\nWaiting for MoCap data to stabilize...\n');
        waitTime = 5; % seconds
        for i = 1:waitTime
            [testPose, dataReceived] = fetchRobotPose(mqttHandle, limoIdNumber, ALL_CFG, [], 0);
            if dataReceived
                fprintf('  %d/%d seconds... Current data: (%.2f, %.2f, %.1f°)\n', i, waitTime, testPose(1), testPose(2), rad2deg(testPose(3)));
            else
                fprintf('  %d/%d seconds... No data yet\n', i, waitTime);
            end
            pause(1);
        end
        fprintf('Wait complete \n\n');
        
        % Get initial pose
        fprintf('Reading initial robot position...\n');
        initialPoseFound = false;
        maxAttempts = 5;
        currentPose = [0, 0, 0];
        for attempt = 1:maxAttempts
            [currentPose, initialPoseFound] = fetchRobotPose(mqttHandle, limoIdNumber, ALL_CFG, [], 0);
            if initialPoseFound
                fprintf('Start pose: (%.2f, %.2f, %.1f°)\n', currentPose(1), currentPose(2), rad2deg(currentPose(3)));
                break;
            end
            fprintf('  Attempt %d/%d: No data...\n', attempt, maxAttempts);
            pause(0.5);
        end
        if ~initialPoseFound
            error('Cannot read MoCap data');
        end
        startPose = currentPose;
        
        %% INITIALIZE GLOBAL WAYPOINT TRACKING
        currentGlobalWpIndex = 1;
        numGlobalWps = size(ALL_CFG.globalWaypoints, 2);
        fprintf('Waypoint Tracker initialized. Target Waypoint: #%d\n', currentGlobalWpIndex);

    %% VISUALIZATION SETUP 
    figure('Name','LIMO Path Following (RT-PP)','Position',[100 100 1000 800]);
    plot(ALL_CFG.staticObstacles(1,:), ALL_CFG.staticObstacles(2,:), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'k', 'DisplayName', 'Obstacles (Static)');
    hold on; grid on; axis equal;
    plot(ALL_CFG.globalWaypoints(1,:), ALL_CFG.globalWaypoints(2,:), 'b--o', 'LineWidth', 2.5, 'MarkerSize', 5, 'DisplayName', 'Global Waypoints');
    
    % Local Path (Dynamically updated)
    localPathPlot = plot(NaN, NaN, 'r-', 'LineWidth', 1.0, 'DisplayName', 'Local Path (Real-Time)');
    
    plot(courseGoal(1), courseGoal(2), 'bx', 'MarkerSize', 25, 'LineWidth', 3, 'DisplayName', 'Goal');
    plot(startPose(1), startPose(2), 'r*', 'MarkerSize', 18, 'MarkerFaceColor', 'r', 'DisplayName', 'Start');
    actualTrajectoryPlot = plot(NaN, NaN, 'k:', 'LineWidth', 3.0, 'DisplayName', 'Actual Path');
    robotMarker = plot(NaN, NaN, 'go', 'MarkerSize', 14, 'MarkerFaceColor', 'g', 'DisplayName', 'Robot');
    lookaheadMarker = plot(NaN, NaN, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r', 'DisplayName', 'Lookahead Point');
    
    % Moving Obstacles Placeholder (Will update in loop)
    movingObsMarker = plot(NaN, NaN, 'mx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Moving Obstacles (Placeholder)');
    
    xlabel('X Position [m]'); ylabel('Y Position [m]');
    title(sprintf('LIMO %s - RT-PP Control (Course %d)', limoIdNumber, routeNumber));
    legend('Location', 'best');
    xlim([-0.5 5.5]); 
    ylim([-0.5 5.0]);
    
        %% CONTROL LOOP
        fprintf('\nStarting control loop with Real-Time Planning...\n');
        timerStart = tic;                 
        currentRobotPose = startPose;     
        robotTrajectory = startPose(1:2); 
        
        % State Tracking
        reversing = false;
        reverseStartTime = 0;
        
        % Loop detection variables
        errorHistory = zeros(1, 10);      % Last 10 cross-track errors
        historyIndex = 1;
        
        while true
            loopStartTime = tic;  
            elapsedTime = toc(timerStart);

            % 1. Get current robot state from MoCap 
            previousPose = currentRobotPose;
            [currentRobotPose, dataValid] = fetchRobotPose(mqttHandle, limoIdNumber, ALL_CFG, previousPose, ALL_CFG.dt);
            if ~dataValid
                fprintf(' Lost MoCap data - keeping previous command\n');
                continue; % Skip this iteration and try again
            end
            
            % Extract current position and heading
            currentX = currentRobotPose(1);
            currentY = currentRobotPose(2);
            currentTheta = currentRobotPose(3);
            
            % 2. Update Global Waypoint Tracking
            currentGlobalWp = ALL_CFG.globalWaypoints(:, currentGlobalWpIndex);
            distToCurrentWp = sqrt((currentX - currentGlobalWp(1))^2 + (currentY - currentGlobalWp(2))^2);
            
            if distToCurrentWp < ALL_CFG.posTolerance && currentGlobalWpIndex < numGlobalWps
                currentGlobalWpIndex = currentGlobalWpIndex + 1;
                fprintf('✅ Reached Global Waypoint #%d. Targeting #%d now.\n', currentGlobalWpIndex - 1, currentGlobalWpIndex);
                currentGlobalWp = ALL_CFG.globalWaypoints(:, currentGlobalWpIndex);
            end
            
            % 3. Check if final destination is reached 
            distanceToGoal = sqrt((currentX - courseGoal(1))^2 + (currentY - courseGoal(2))^2);
            if distanceToGoal < ALL_CFG.posTolerance
                fprintf('\n Goal reached at %.1f s\n', elapsedTime);
                write(tcpConnection, uint8('0.00,0.00')); % Stop robot
                break;
            end
            
            % 4. DYNAMIC OBSTACLE DETECTION (Placeholder)
            % In a real system, sensor data would populate ALL_CFG.movingObstacles here.
            % Example: ALL_CFG.movingObstacles = detectMovingRobots(); 
            
            
            % 5. REAL-TIME LOCAL PATH PLANNING
            % The local planner generates a short, smooth, collision-free path 
            % segment toward the current global waypoint.
            
            % Target for the local planner: current global waypoint
            localPlannerTarget = currentGlobalWp; 
            
            localPath = localPathPlanner(currentRobotPose, localPlannerTarget, ALL_CFG);
            
            if isempty(localPath.x)
                % Critical failure in planning (e.g., trapped) - trigger emergency stop
                linearVelocityCommand = 0.0;
                angularVelocityCommand = 0.0;
                fprintf(2, '!!! Local Path Planning Failed: Emergency Stop Activated !!!\n');
                write(tcpConnection, uint8('0.00,0.00'));
                pause(1.0); % Wait for real robot to stop
                continue;
            end
            
            % 6. Pure Pursuit on the Local Path
            [lookaheadX, lookaheadY, ~, crossTrackError] = findLookahead(currentX, currentY, localPath, ALL_CFG.lookaheadDist);
            
            % 7. Loop Detection / Stuck Check (Uses cross-track error on the local path)
            errorHistory(historyIndex) = crossTrackError;
            historyIndex = mod(historyIndex, length(errorHistory)) + 1;
            
            % Check if robot is stuck (high, consistent cross-track error)
            meanError = mean(errorHistory);
            if meanError > ALL_CFG.reverseOnStuckError && ~reversing
                fprintf('🚨 High cross-track error (%.2fm). Initiating REVERSE MANEUVER at %.1f s\n', meanError, elapsedTime);
                reversing = true;
                reverseStartTime = elapsedTime;
            end
           
            % 8. CONTROL DECISION: Reverse or Pure Pursuit
            if reversing
                % **REVERSE MANEUVER LOGIC**
                reverseElapsed = elapsedTime - reverseStartTime;
                
                if reverseElapsed < ALL_CFG.reverseTurnTime
                    % Phase 1: Straight Reverse
                    linearVelocityCommand = ALL_CFG.reverseLinearVel;
                    angularVelocityCommand = 0.0;
                    if mod(round(reverseElapsed / ALL_CFG.dt), 30) == 0 
                        fprintf('  Reversing Straight (%.1f/%.1f s)\n', reverseElapsed, ALL_CFG.reverseTurnTime);
                    end
                elseif reverseElapsed < (ALL_CFG.reverseTurnTime + 2.0) % Allow 2s for the turn
                    % Phase 2: Reverse and Turn
                    linearVelocityCommand = ALL_CFG.reverseLinearVel;
                    angularVelocityCommand = ALL_CFG.reverseTurnOmega; % Turn right
                    if round(reverseElapsed * ALL_CFG.controlRate) == round(ALL_CFG.reverseTurnTime * ALL_CFG.controlRate)
                        fprintf('  Initiating Reverse-and-Turn (%.1f/%.1f s)\n', reverseElapsed, ALL_CFG.reverseTurnTime + 2.0);
                    end
                else
                    % Phase 3: Maneuver Complete - Switch back to Pure Pursuit
                    fprintf('Reverse maneuver complete. Resuming Pure Pursuit at %.1f s\n', elapsedTime);
                    reversing = false;
                end
                
            else
                % **PURE PURSUIT CONTROL LAW**
                
                % Calculate the desired heading towards the lookahead point
                dx = lookaheadX - currentX;
                dy = lookaheadY - currentY;
                desiredHeading = atan2(dy, dx);
                
                % Calculate heading error (wrapped to [-pi, pi])
                headingError = wrapToPi(desiredHeading - currentTheta);
                
                % Calculate distance to lookahead point
                distanceToLookahead = sqrt(dx^2 + dy^2);
                
                % Pure Pursuit formula for angular velocity:
                angularVelocityCommand = (2 * ALL_CFG.targetLinearVel * sin(headingError)) / distanceToLookahead;
                
                % Linear velocity: reduce speed if heading error is large
                if abs(headingError) < deg2rad(90)
                    linearVelocityCommand = ALL_CFG.targetLinearVel * cos(headingError);
                else
                    % Very large error: slow down significantly
                    linearVelocityCommand = ALL_CFG.targetLinearVel * 0.3;
                end
            end
            
            % 9. Apply velocity limits and sanitize
            % Linear velocity must be within limits (positive or negative)
            linearVelocityCommand = max(ALL_CFG.reverseLinearVel, min(linearVelocityCommand, ALL_CFG.maxLinVel));
            angularVelocityCommand = max(-ALL_CFG.maxAngVel, min(angularVelocityCommand, ALL_CFG.maxAngVel));
            
            % 10. Send velocity command to LIMO
            commandString = sprintf('%.2f,%.2f', linearVelocityCommand, angularVelocityCommand);
            write(tcpConnection, uint8(commandString));
            
            % 11. Update visualization 
            robotTrajectory = [robotTrajectory; currentX, currentY]; 
            set(robotMarker, 'XData', currentX, 'YData', currentY);
            set(actualTrajectoryPlot, 'XData', robotTrajectory(:,1), 'YData', robotTrajectory(:,2));
            set(lookaheadMarker, 'XData', lookaheadX, 'YData', lookaheadY);
            % Update local path plot
            set(localPathPlot, 'XData', localPath.x, 'YData', localPath.y);
            
            % Update plot title with real-time status
            statusText = 'RT-PP/Pure Pursuit';
            if reversing
                statusText = 'REVERSING';
                set(robotMarker, 'MarkerFaceColor', [1 0 0]); % Red when reversing
            else
                set(robotMarker, 'MarkerFaceColor', [0 1 0]); % Green when normal
            end
            
            title(sprintf('LIMO %s - Course %d | t=%.1fs | Status: %s | V=%.2f, W=%.1f°/s', ...
                         limoIdNumber, routeNumber, elapsedTime, statusText, linearVelocityCommand, rad2deg(angularVelocityCommand)));
            drawnow limitrate; 
            
            % 12. Safety timeout check 
            if elapsedTime > ALL_CFG.maxTotalTime
                fprintf('✗ Maximum time limit reached, stopping the code\n');
                break;
            end
            
            % 13. Maintain control loop rate 
            loopDuration = toc(loopStartTime);
            if loopDuration < ALL_CFG.dt
                pause(ALL_CFG.dt - loopDuration); % Wait to maintain desired rate
            end
        end
        
    catch exception
        % Error handling: display error message and location
        fprintf('Error: %s\n', exception.message);
        if ~isempty(exception.stack)
            fprintf('  In: %s (line %d)\n', exception.stack(1).name, exception.stack(1).line);
        end
    end
    
    %% CLEANUP AND SHUTDOWN (Unchanged)
    if ~isempty(tcpConnection) && isvalid(tcpConnection)
        write(tcpConnection, uint8('0.00,0.00')); % Send final stop command
        pause(0.5);
        clear tcpConnection; % Close TCP connection
    end
    if ~isempty(mqttHandle)
        clear mqttHandle; % Close MQTT connection
    end
    fprintf('Script execution finished.\n\n');
end

%% HELPER FUNCTION: LOCAL REAL-TIME PATH PLANNER (Placeholder)
function localPath = localPathPlanner(currentPose, targetGlobalWp, ALL_CFG)
% LOCALPATHPLANNER - Generates a short, obstacle-free path segment.
% This function is the core of the Real-Time Path Planning (RT-PP) system.
% In a full implementation, this would run an algorithm (like DWA or RRT) 
% to avoid static and dynamic obstacles. Here, it is simplified to a 
% straight line path segment.

    % Define the target for the local planner (a pose at the target global WP)
    current_x = currentPose(1);
    current_y = currentPose(2);
    target_x = targetGlobalWp(1);
    target_y = targetGlobalWp(2);

    dx = target_x - current_x;
    dy = target_y - current_y;
    target_heading = atan2(dy, dx);
    
    % Cap the local planning distance to the planning horizon
    dist_to_target = sqrt(dx^2 + dy^2);
    planning_distance = min(dist_to_target, ALL_CFG.planningHorizon);
    
    % 1. Generate the Path (Straight Line)
    num_points = ceil(planning_distance / ALL_CFG.planningRes) + 1;
    s_values = linspace(0, planning_distance, num_points);

    localPath.x = current_x + s_values' * cos(target_heading);
    localPath.y = current_y + s_values' * sin(target_heading);

    % Calculate path arc length (s)
    pathSegmentDistances = sqrt(diff(localPath.x).^2 + diff(localPath.y).^2);
    localPath.s = [0; cumsum(pathSegmentDistances)];
    
    % --- Dynamic Obstacle Avoidance Placeholder ---
    % if checkCollisionWithMovingObstacles(localPath, ALL_CFG.movingObstacles)
    %     % REPLAN (e.g., call a DWA/RRT planner here)
    %     % ...
    % end
    
    % If the local path is too short (i.e., we are already close to the
    % local target), return the current path.
    if isempty(localPath.x)
        localPath.x = [current_x]; 
        localPath.y = [current_y];
        localPath.s = [0];
    end
end

%% HELPER FUNCTION TO GET ROBOT POSE FROM MQTT 
function [pose, dataValid] = fetchRobotPose(mqttHandle, limoNum, ALL_CFG, prevPose, dt)
    % FETCHROBOTPOSE - Reads robot position (x, y) and calculates heading 
    pose = [0, 0, 0];
    dataValid = false;
    try
        mqttMessage = peek(mqttHandle);
        if isempty(mqttMessage)
            return;
        end
        expectedTopic = sprintf('rb/limo%s', limoNum);
        if ~strcmp(char(mqttMessage.Topic), expectedTopic)
            return;
        end
        jsonString = char(mqttMessage.Data);
        dataJson = jsondecode(jsonString);
        if ~isfield(dataJson, 'pos') || ~isfield(dataJson, 'rot')
            return;
        end
        mocapX = dataJson.pos(1);
        mocapY = dataJson.pos(3);
        x = mocapX - ALL_CFG.mocapOriginX;
        y = -(mocapY - ALL_CFG.mocapOriginY);
        
        if ~isempty(prevPose) && dt > 0
            dx = x - prevPose(1);
            dy = y - prevPose(2);
            currentSpeed = sqrt(dx^2 + dy^2) / dt;
            speedThreshold = 0.05;
            if currentSpeed > speedThreshold
                theta = atan2(dy, dx);
            else
                theta = prevPose(3);
            end
        else
            if length(dataJson.rot) >= 3
                theta = -dataJson.rot(3);
            else
                theta = 0;
            end
        end
        pose = [x, y, theta];
        dataValid = true;
    catch
        % Error occurred during MQTT read or processing.
    end
end
%% HELPER FUNCTION: FIND LOOKAHEAD POINT 
function [lookaheadX, lookaheadY, lookaheadIndex, crossTrackError] = ...
    findLookahead(robotX, robotY, plannedPath, lookaheadDistance)
    
    allDistances = sqrt((plannedPath.x - robotX).^2 + (plannedPath.y - robotY).^2);
    [crossTrackError, closestIndex] = min(allDistances);
    
    sClosest = plannedPath.s(closestIndex);
    sLookaheadTarget = sClosest + lookaheadDistance;
    
    if sLookaheadTarget > plannedPath.s(end)
        lookaheadIndex = length(plannedPath.x);
    else
        lookaheadIndex = find(plannedPath.s >= sLookaheadTarget, 1, 'first');
        if isempty(lookaheadIndex)
            lookaheadIndex = length(plannedPath.x);
        end
    end
    
    lookaheadX = plannedPath.x(lookaheadIndex);
    lookaheadY = plannedPath.y(lookaheadIndex);
end
%% HELPER FUNCTION: Angle Wrapping
function angle = wrapToPi(angle)
    % Wraps angle (in radians) to the interval [-pi, pi]
    angle = angle - 2*pi * floor((angle + pi) / (2*pi));
end