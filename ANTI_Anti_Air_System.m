clc; clear; close all;

%*********************IMAGES AND ENVIRONMENT ATTRIBUTES********************

maxTime = 120; %maxiumum time for simulation

g = 9.81; %gravity

%for determining simulation bounds. it'll be 1.5x of the red's range.
redTravelingDistance = 0;
simulationBound = 0;

%update time
dt = 0.012977; % s
t = 0; % s
displayTimes = [1, 2, 3, 4, 5];
nextDisplayTimeIndex = 1;


warningText = [];


redObjectImage = imread('REDOBJ.jpg');
blueObjectImage = imread('blueObj.jpg');
explosionImage = imread('explosion.jpg');

%*********************IMAGES AND ENVIRONMENT ATTRIBUTES********************


%****************************TRAILS ATTRIBUTES*****************************

trailRed = [];
trailBlue = [];

trailFrequency = 1;

lastDotTime = -Inf;

%****************************TRAILS ATTRIBUTES*****************************


%***************************RED OBJECT ATTRIBUTES**************************

v0RedMinValue = 114.39;   %minimum v0 value for object to stay on air for at least 15 secs
v0RedMaxValue = 200;      %a default maximum value for v0

thetaRedMinValue = 40;
thetaRedMaxValue = 80;

% Generate a random speed for the red object within the specified range
%v0Red = v0RedMinValue + (v0RedMaxValue - v0RedMinValue).*rand;


%v0RedDefault = 114.39;
%thetaRedDefault = 40;

v0RedDefault = 200;
thetaRedDefault = 80;

%v0Red = v0RedDefault;
%thetaRed = thetaRedDefault;

%Random speed and angle for the red object within the indicated ranges
v0Red = v0RedMinValue + (v0RedMaxValue - v0RedMinValue).*rand;
thetaRed = thetaRedMinValue + (thetaRedMaxValue - thetaRedMinValue).*rand;


%red ball initial attributes
vRed = v0Red*[cosd(thetaRed) sind(thetaRed)]; % m/s
xRed = 0; % m
yRed = 0; % m
redTravelingDistance = v0Red^2 * sind(2*thetaRed) / g;
simulationBound = 3*redTravelingDistance/2;
counterForErrorTimeOfRed = 0; 

%***************************RED OBJECT ATTRIBUTES**************************

%**************************BLUE OBJECT ATTRIBUTES**************************

%blue object attributes initialization

vBlue = 0;
thetaBlue = 0;

%vBlueDefault = 150;
%thetaBlueDefault = 135;

vBlue = vBlue*[cosd(thetaBlue) sind(thetaBlue)]; % m/s
vBlueX = vBlue(1);
vBlueY = vBlue(2);

%blue object initial coordinates
xBlue = simulationBound; % m
yBlue = 0; % m

counterForErrorTimeOfBlue = 0;

%**************************BLUE OBJECT ATTRIBUTES**************************


%********************************BACKGROUND********************************
bg = imread('BG.jpg');
x = linspace(0, simulationBound, size(bg, 2));
y = linspace(-350, 2000, size(bg, 1));

% Create a new figure
figure;

% Display the background image
imagesc(x, y, flipud(bg));
set(gca, 'YDir', 'normal');

% Use hold on to allow subsequent plotting over the image
hold on;

%********************************BACKGROUND********************************

%*****************************RADAR ATTRIBUTES*****************************
radarAngle = 0;
radarSpeed = 2*pi; % The speed of the radar line (in radians per second)
detectionRadius = 700;
radarOffset = 100;

radarLine = [];
detected = false;
onceDetected = false;
lastDetectionTime = -Inf;
lastWarningTime = -Inf;
collisionOccurred = false;
detectionCount = 1;

%*****************************RADAR ATTRIBUTES*****************************




while t <= maxTime
    randomErrorValue = 300*rand-150;

    
    %*************************DISPLAY CURRENT TIME*************************
    if exist('currentTime','var')
        delete(currentTime);
    end
    currentTime = text(simulationBound, 1925, ['t = ', num2str(t,'%.2f')], 'HorizontalAlignment', 'right');
    %*************************DISPLAY CURRENT TIME*************************
    
    
    %***********************DISPLAY RED OBJECT STATS***********************

    if exist('redObjectCoordinates','var')
        delete(redObjectCoordinates);
    end
    redObjectCoordinates = text(simulationBound, 1850, ['Red Coordinates: (', num2str(xRed,'%.2f'), ', ', num2str(yRed,'%.2f'), ')'], 'HorizontalAlignment', 'right');
    
    if exist('redObjectSpeed','var')
        delete(redObjectSpeed);
    end
    redObjectSpeed = text(simulationBound, 1775, ['Red Speed: ', num2str(norm(vRed),'%.2f')], 'HorizontalAlignment', 'right');
    
    
    if nextDisplayTimeIndex <= length(displayTimes) && t >= displayTimes(nextDisplayTimeIndex)
        roundedTime = round(t);
        displayedPosition = text(0, 1925 - 75*(nextDisplayTimeIndex-1), ['Red''s pos at t = ', num2str(roundedTime), ': x=', num2str(xRed,'%.2f'), ' y=', num2str(yRed,'%.2f')], 'HorizontalAlignment', 'left');
        nextDisplayTimeIndex = nextDisplayTimeIndex + 1;
    end


    
    
        
    %***********************DISPLAY RED OBJECT STATS***********************
    
    
    %***********************DISPLAY BLUE OBJECT STATS**********************
    if exist('blueObjectCoordinates','var')
        delete(blueObjectCoordinates);
    end
    blueObjectCoordinates = text(simulationBound, 1700, ['Blue Coordinates: (', num2str(xBlue,'%.2f'), ', ', num2str(yBlue,'%.2f'), ')'], 'HorizontalAlignment', 'right');
    if exist('blueObjectSpeed','var')
        delete(blueObjectSpeed);
    end
    blueObjectSpeed = text(simulationBound, 1625, ['Blue Speed: ', num2str(norm(vBlue),'%.2f')], 'HorizontalAlignment', 'right');
    %***********************DISPLAY BLUE OBJECT STATS**********************


    %***********************BLUE OBJECT DATA INPUT*************************
    
    %delete previous blue balls
    if exist('blueObject','var')
        delete(blueObject);
    end
    
    %display blue object
    blueObject = image('CData', blueObjectImage, 'XData', [xBlue-50 xBlue+50], 'YData', [yBlue-50 yBlue+50]);
    hold on;
        
    %initialize blue after 5th second
    if t > 6
        %update blue ball
        vBlue = vBlue; %#ok<ASGSL>
        xBlue = xBlue + vBlueX*dt;
        yBlue = yBlue + vBlueY*dt; 
    elseif t+0.01 > 6
        % Take inputs for blueObject
        vBlue = input('Enter the speed for the blue object: ');
        thetaBlue = input('Enter the angle for the blue object: ');
        
        thetaBlue = 180 - thetaBlue;
        
        %vBlueDefault = 150;
        %thetaBlueDefault = 135;
        %vBlue = vBlueDefault;
        %thetaBlue = thetaBlueDefault;
        
        %vBlueDefault = 1000;
        %thetaBlueDefault = 147;
        
        %vBlueDefault = 1000;
        %thetaBlueDefault = 175;
        
        %vBlueDefault = 100;
        %thetaBlueDefault = 130;
        
        %vBlueDefault = 150;
        %thetaBlueDefault = 135;
        
        
        vBlue = vBlue*[cosd(thetaBlue) sind(thetaBlue)]; % m/s
        vBlueX = vBlue(1);
        vBlueY = vBlue(2);
        while vBlue == 0
            pause(1);
        end
    end
    
    counterForErrorTimeOfBlue = counterForErrorTimeOfBlue+dt;
    
    if counterForErrorTimeOfBlue >= 1
        plot(xBlue+randomErrorValue, yBlue+randomErrorValue, 'co');
        counterForErrorTimeOfBlue = 0;
    end
    
    
    %***********************BLUE OBJECT DATA INPUT*************************


    %******************************RED OBJECT******************************
    
    %delete previous red balls
    if exist('redObject','var')
        delete(redObject);
    end
    
    %display red ball
    redObject = image('CData', redObjectImage, 'XData', [xRed-100 xRed+100], 'YData', [yRed-100 yRed+100]);
    hold on;
    
    if onceDetected == false
        %update red ball
        a = [0 -g]; % m/s^2
        vRed = vRed + a*dt; % m/s
        xRed = xRed + vRed(1)*dt; % m
        yRed = yRed + vRed(2)*dt; % m
    end
    if onceDetected == true
        vRed = vRed + a*dt;
        xRed = xRed + 2*vRed(1)*dt;
        yRed = yRed + 20*vRed(2)*dt;
    end
    
    counterForErrorTimeOfRed = counterForErrorTimeOfRed + dt;
    
    if counterForErrorTimeOfRed >= 1
        plot(xRed+randomErrorValue, yRed+randomErrorValue, 'yo');
        counterForErrorTimeOfRed = 0;
    end
    
    %******************************RED OBJECT******************************


    %****************************DISPLAY TRAILS****************************
    
    if t - lastDotTime >= 1/trailFrequency
        % Store the current positions in the trails
        trailRed = [trailRed; xRed, yRed];
        trailBlue = [trailBlue; xBlue, yBlue];
        
        % Draw the trails
        for i = 1:size(trailRed, 1)
            plot(trailRed(i, 1), trailRed(i, 2), 'r.');
        end
        for i = 1:size(trailBlue, 1)
            plot(trailBlue(i, 1), trailBlue(i, 2), 'b.');
        end
        
        lastDotTime = t;
    end

    %****************************DISPLAY TRAILS****************************


    %**************************DISPLAY RADAR LINE**************************

    % Delete the previous radar line
    if ~isempty(radarLine) && ishandle(radarLine)
        delete(radarLine);
    end

    % Calculate the distance between the two objects
    distance = sqrt((xRed - (xBlue+randomErrorValue))^2 + (yRed - (yBlue+randomErrorValue))^2);

    % Update the radar angle
    radarAngle = radarAngle + radarSpeed*dt;
    if radarAngle > 2*pi
        radarAngle = radarAngle - 2*pi;
    end

    % Only draw the radar line if no collision has occurred
    if ~collisionOccurred
        % Calculate the start and end points of the radar line
        radarLineStartX = xRed + radarOffset*cos(radarAngle);
        radarLineStartY = yRed + radarOffset*sin(radarAngle);
        radarLineEndX = xRed + detectionRadius*cos(radarAngle);
        radarLineEndY = yRed + detectionRadius*sin(radarAngle);

        % Draw the radar line
        radarLine = line([radarLineStartX, radarLineEndX], [radarLineStartY, radarLineEndY], 'Color', 'g', 'LineWidth', 4);
    end
    
    %**************************DISPLAY RADAR LINE**************************

    
    %***********************RADAR'S DETECTION SYSTEM***********************
    
    % Check if the blue object is within the detection radius of the radar
    if distance <= detectionRadius
        % Check if the radar line intersects with the blue object
        if abs(atan2((yBlue+randomErrorValue) - yRed, (xBlue+randomErrorValue) - xRed) - radarAngle) <= 3
            detected = true;
            if detected && t - lastDetectionTime > 0.2  % Consider it a new detection if more than 0.1 second has passed
                % Delete the previous warning text
                if ishandle(warningText)
                    delete(warningText);
                end

                % Display the warning text above the red object
                warningText = text(xRed, yRed + 200, 'WARNING!!!', 'Color', 'r', 'FontSize', 14);
                onceDetected = true;

                lastDetectionTime = t;
                lastWarningTime = t;
            end
        else
            detected = false;
        end
    else
        detected = false;
    end

    % Delete the warning text after 0.3 seconds
    if t - lastWarningTime >= 0.3
        if ishandle(warningText)
            delete(warningText);
        end 
    end

    %***********************RADAR'S DETECTION SYSTEM***********************
    
    
    %**************************COLLISION CHECKER***************************
    
    if sqrt((xRed-xBlue)^2 + (yRed-yBlue)^2) <= 50 || yRed <= 0 %18.37
        redObjectImage = explosionImage;
        redObject = image('CData', redObjectImage, 'XData', [xRed-400 xRed+400], 'YData', [yRed-400 yRed+400]);
        if t - lastWarningTime >= 0.0001
            if ishandle(warningText)
                delete(warningText);
            end 
        end
        % Set the collision flag to true
        collisionOccurred = true;
        if ishandle(radarLine)
            delete(radarLine);
        end
        pause(5);
        break;
    end

    %**************************COLLISION CHECKER***************************

    
    t = t + dt; %update time
    drawnow;
    axis([0 simulationBound -150 2000]); %simulation window
    hold off;
end
