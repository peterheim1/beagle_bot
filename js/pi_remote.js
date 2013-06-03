/*  pi_remote.js - Version 1.0 2012-09-22

    An HTML5/rosbridge script to teleop and monitor a ROS robot

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2012 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
*/

// Get the current hostname
thisHostName = document.location.hostname;

// If the rosbridge server is running on the webserver host, then this will work.
var rosbridgeHost = thisHostName;

// Or, if you want to prompt for the rosbridge host IP use this instead
//var rosbridgeHost = prompt("IP Address of rosbridge Server:", "192.168.1.2");

// Or, set it manually like this.
//var rosbridgeHost =  "192.168.1.9";

var rosbridgePort = "9090";
var mjpegPort = "8080";

// Get the current host and port to be used with the map URL
thisHost = document.location.host;

// Are we on a touch device?
var is_touch_device = 'ontouchstart' in document.documentElement;

// Key modifiers
var shiftKey = false;
var ctrlKey = false;
var altKey = false;

// Alert colors
var green = "#33FF33";
var yellow = "yellow";
var red = "#FF1111";

// Video parameters
var cameraHeight = 1.24; // meters
var videoImage = new Image();
var video2Image = new Image();
var videoTopic = "/camera/rgb/image_color";
var video2Topic = "/wide_angle_camera/image_raw";
var videoQuality = 100;
var video2Quality = 70;
var video1_On = true;
var video2_On = false;

// WideCam settings
//var videoWidth = 1280;
//var videoHeight = 720;
//var videoWidth = 640;
//var videoHeight = 480;
//var fovWidthRadians = 2.09; // WideCam 120 degrees
//var fovHeightRadians = 1.17; // WideCam 67 degrees
//var videoScale = 1.0;

// Asus/Kinect settings
var videoWidth = 320;
var videoHeight = 240;
var fovWidthRadians = 0.99; // Asus 57 degrees
var fovHeightRadians = 0.78; // Asus 45 degrees
var videoScale = 2.5;

var screenWidth = 1280;
var videoFPS = 20;
if (is_touch_device) var videoScale = 2.0;
var videoStageWidth = videoWidth * videoScale;
var videoStageHeight = videoHeight * videoScale;
var videoHandle = null;
var pubHandle = null;
var stopHandle = null;
var clickMarkerHandle = null;
var cameraInfo = "/camera/rgb/camera_info";
var roiTopic = "/roi";

// Rate for the main ROS publisher loop
var rate = 5;

// Base control parameters
var useFixedSpeedControl = false;
var cmdVelTopic = "/cmd_vel";
var defaultMaxLinearSpeed = 0.18;
var defaultMaxAngularSpeed = 1.2;
var maxLinearSpeed = defaultMaxLinearSpeed;
var maxAngularSpeed = defaultMaxAngularSpeed;
var minLinearSpeed = 0.05;
var minAngularSpeed = 0.1;
var vx_key_increment = 0.02;
var vz_key_increment = 0.05;
var vx_fixed_speed = 0.12;
var vz_fixed_speed = 0.5;
var deadZoneVz = 0.2;
var vx = 0;
var vz = 0;
var lastVx = 0;
var lastVz = 0;
var moveBaseTopic = "/move_base_simple/goal";
var initialPoseTopic = "/initialpose";
var amclPoseTopic = "/amcl_pose";
var moveBaseCancelTopic = "/move_base/cancel";
var mapMetadataTopic = "/map_metadata";
var odomTopic = "/odom";
var usingAMCL = false;
var firstSetPose = true;
var firstMoveBase = true;
var firstMoveBaseCancel = true;

// Laser scan topics
var baseLaserScanTopic = "/scan";
var obstacleLaserTopic = "/base_laser_obstacle";
var laserObstacleDetected = false;
var laserAutoStop = true;

// Servo parameters
var jointStatesTopic = "/joint_states";
var headPanPositionTopic = "/head_pan_joint/command";
var headTiltPositionTopic = "/head_tilt_joint/command";
var maxHeadPanPosition = 1.25;
var maxHeadTiltPosition = 0.72;
var headPanPosition = 0;
var headTiltPosition = 0;
var desiredHeadPanPosition = 0;
var desiredHeadTiltPosition = 0;
var servoStatus = [0, 0];
var servoStatusColors = [green, yellow, red];

var headPanSpeedService = '/head_pan_joint/set_speed';
var headTiltSpeedService = '/head_tilt_joint/set_speed';
var maxHeadPanSpeed = 1.5;
var maxHeadTiltSpeed = 1.0;
var defaultHeadPanSpeed = 0.45;
var defaultHeadTiltSpeed = 0.3;
var headPanSpeed = 0;
var headTiltSpeed = 0;
var headPanTorqueService = "/head_pan_joint/torque_enable";
var headTiltTorqueService = "/head_tilt_joint/torque_enable";

var jointStatesTopic = "/joint_states"

var headPanTempTopic = "/head_pan_joint/state";
var headTiltTempTopic = "/head_tilt_joint/state";

var servoSpeedControl = false;
var relaxServos = false;
var centerServos = false;
var servosRelaxed = false;
var lockServos = false;

// Battery topics
var laptopBatteryTopic = "/laptop_charge";
var robotBatteryTopic = "/arduino/sensor/main_voltage";
var frontSonarTopic = "/arduino/sensor/base_sonar_front";
var backSonarTopic = "/arduino/sensor/base_sonar_back";
var turtleBotBatteryTopic = "/turtlebot_node/sensor_state";
var turtleBot = false;

// Sonar variables
var sonarFrontObstacleDetected = false;
var sonarBackObstacleDetected = false;
var autoStopRange = 0.60;    // meters
var minSonarDistance = 0.03; // meters
var baseSonarFront = autoStopRange + 0.1;
var baseSonarBack = autoStopRange + 0.1;
var sonarAutoStop = true;

// Map parameters
var mapWidth;
var mapHeight;
var mapResolution;
var mapOriginX;
var mapOriginY;
var mapFile;

// Robot position from /odom
var odomPositionX;
var odomPositionY;

// Robot position from /amcl_pose
var amclPositionX;
var amclPositionY;

// Record the start and end coordinates of a goal pose arrow
var goalPoseStartX;
var goalPoseStartY;
var goalPoseEndX;
var goalPoseEndY;

// Track joint names, positions and velocities
var jointNames;
var jointPos;
var jointVel;

// Track the size and position of the region of interest
var roiOffsetX;
var roiOffsetY;
var roiWidth;
var roiHeight;
var roiMarker = new Kinetic.Rect({
    x: 100,
    y: 100,
    width: 50,
    height: 50,
    stroke: "#99FF33",
    strokeWidth: 4
});

var clickMarker = new Kinetic.Circle({
    x: 0,
    y: 0,
    radius: 40,
    stroke: "#99FF33",
    strokeWidth: 4
});

// A marker for move_base goals
var goalMarker = new Kinetic.Circle({});

// A line for setting goal poses
var goalPoseLine = new Kinetic.Line({});

// A line for indicating current velocity overlayed on the video
var cmdVelMarker = new Kinetic.Line({});

// A flag to indicate when the mouse is down
var mouseDown = false;

function log(msg) {
    javascript:console.log(msg);
}

function getScreenWidth() {
    return screenWidth;
}

function getVideoWidth() {
    return videoWidth;
}

function getVideoHeight() {
    return videoHeight;
}

// Make the connection to the rosbridge server
log('Creating ROSProxy connection object...');
var connection = null;
try {
    connection = new ros.Connection("ws://" + rosbridgeHost + ":" + rosbridgePort);
} catch (err) {
    log('Failed to connect to rosbridge!');
}

log('Connection created to ' + rosbridgeHost + ':' + rosbridgePort);

// Create callbacks on the rosbridge open, error and close events
connection.setOnError(function() {
    log('rosbridge error!');
});

connection.setOnClose(function() {
    log('rosbridge closed');
});

connection.setOnOpen(function() {
/*
    log('Initializing ROSProxy...');
    try {
	connection.callService('/rosjs/topics', '[]', function nop() {});
	return;
    } catch (err) {
        log('Problem initializing ROSProxy!');
        return;
    }
*/
    // Call the special rosbridge service get_param to read in various parameters
    connection.callService('/rosbridge/get_param', '["pi_remote"]', function(resp) {
	for (param in resp) {
	    log(param + ": " + resp[param]);
	}
    	if (resp['video_topic'] != null) videoTopic = resp['video_topic'];
    	if (resp['video2_topic'] != null) video2Topic = resp['video2_topic'];
    	if (resp['video_quality'] != null) videoQuality = resp['video_quality'];
    	if (resp['video2_quality'] != null) video2Quality = resp['video2_quality'];
    	if (resp['video_fps'] != null) videoFPS = resp['video_fps'];
    	if (resp['max_linear_speed'] != null) maxLinearSpeed = resp['max_linear_speed'];
    	if (resp['max_angular_speed'] != null) maxAngularSpeed = resp['max_angular_speed'];
    	if (resp['map_file'] != null) mapFile = resp['map_file'];
    	if (resp['turtlebot'] != null) turtleBot = resp['turtlebot'];

	// Connect to the mjpeg server for streaming video
//	videoImage.src = "http://" + rosbridgeHost + ":" + mjpegPort + "/stream?topic=" + videoTopic + "?quality=" + videoQuality;
	videoImage.src = "http://" + rosbridgeHost + ":" + mjpegPort + "/stream?topic=" + videoTopic;

	// Connect to second video source if available
	if (video2Topic != null) {
	    video2Image.src = "http://" + rosbridgeHost + ":" + mjpegPort + "/stream?topic=" + video2Topic + "?quality=" + video2Quality;
	    //video2Image.src = "http://192.168.1.100/mjpeg.cgi";
	}

	// If this is a TurtleBot, get the batter charge from /turtlebot_node/sensor_state.
	// Throttle to 1 Hz.
	if (turtleBot) {
    	    connection.addHandler(turtleBotBatteryTopic, function(msg) {
		var color;
		var charge = msg.charge;
		var capacity = msg.capacity;
		var remaining = 100 * (1.0 - ((capacity - charge) / (capacity)));
		var battery_meter = document.getElementById("robot_battery");
		battery_meter.value = remaining;
	    });
	    connection.callService('/rosjs/subscribe', '["' + turtleBotBatteryTopic + '", 1000]', function(e) {
		$('#robotBattery').jqxLinearGauge({ background: { style: { stroke: '#cccccc', fill: '#cccccc'}, visible: true, backgroundType: 'round' }});
		log('Subscribed to ' +  turtleBotBatteryTopic);
	    });
	}
	else {
	    // Otherwise, assume the robot battery topic is /arduino/sensor/main_voltage (Pi Robot)
	    // and display the charge status. Also get the sonar readings.  Throttle battery topics to 1 Hz.
	    connection.addHandler(robotBatteryTopic, function(msg) {
		var color;
		var voltage = msg.value;
		var remaining = 100 * (voltage - 12.5) / (14.0 - 12.5);
		var battery_meter = document.getElementById("robot_battery");
		battery_meter.value = remaining;
	    });
	    connection.callService('/rosjs/subscribe', '["' + robotBatteryTopic + '", 1000]', function(e) {
		$('#robotBattery').jqxLinearGauge({ background: { style: { stroke: '#cccccc', fill: '#cccccc'}, visible: true, backgroundType: 'round' }});
		log('Subscribed to ' +  robotBatteryTopic);
	    });
	}

	connection.addHandler(frontSonarTopic, function(msg) {
	    var color;
	    baseSonarFront = msg.range;
	    if (sonarAutoStop && baseSonarFront < autoStopRange && baseSonarFront > minSonarDistance) sonarFrontObstacleDetected = true;
	    else sonarFrontObstacleDetected = false;
	});
	connection.callService('/rosjs/subscribe', '["' + frontSonarTopic + '", 200]', function(e) {
	    log('Subscribed to ' +  frontSonarTopic);
	});

	connection.addHandler(backSonarTopic, function(msg) {
	    var color;
	    baseSonarBack = msg.range;
	    if (sonarAutoStop && baseSonarBack < autoStopRange && baseSonarBack > minSonarDistance) sonarBackObstacleDetected = true;
	    else sonarBackObstacleDetected = false;
	});
	connection.callService('/rosjs/subscribe', '["' + backSonarTopic + '", 200]', function(e) {
	    log('Subscribed to ' +  backSonarTopic);
	});
    });

    // Subscribe to the /map_metadata topic to get map parameters
    connection.addHandler(mapMetadataTopic, function(msg) {
	mapResolution = msg.resolution;
	mapWidth = msg.width;
	mapHeight = msg.height;
	mapOriginX = msg.origin.position.x;
	mapOriginY = msg.origin.position.y;

	// Once a map is detected, set the drawing parameters for navigation
	var mapStage = new Kinetic.Stage({
	    container: "map_container",
	    draggable: false,
	    width: mapWidth * mapScale + 10,
	    height: mapHeight * mapScale + 10,
	    x: 0,
	    y: 0
	});

	var mapImage = new Image();
	var map = new Kinetic.Image({});
	mapImage.onload = function() {
	    map = new Kinetic.Image({
		image: mapImage,
		width: mapWidth * mapScale,
		height: mapHeight * mapScale,
		offset: [0, 0],
		x: 0,
		y: 0
	    });

	    goalMarker = new Kinetic.Circle({
		x: mapStage.getWidth() / 2,
		y: mapStage.getHeight() / 2,
		radius: 10,
		fill: "#00CC00",
		stroke: "black",
		strokeWidth: 1
	    });

	    robotMarker = new Kinetic.Circle({
		x: mapStage.getWidth() / 2,
		y: mapStage.getHeight() / 2,
		radius: 15,
		fill: "orange",
		stroke: "black",
		strokeWidth: 1
	    });

	    goalPoseLine = new Kinetic.Line({
		points: [0, 0, 0, 0],
		strokeWidth: 3,
		stroke: "#00CC00"
	    });

	    mapMarkerLayer.add(goalPoseLine);

	    moving = false;

	    mapStage.on("mousedown touchstart", function(){
		if (moving) {
		    moving = false; mapMarkerLayer.draw();
		} else {
		    if (is_touch_device) var mousePos = mapStage.getTouchPosition();
		    else var mousePos = mapStage.getMousePosition();
		    mapMarkerLayer.removeChildren();
		    var x =  mousePos.x;
		    var y =  mousePos.y;
		    goalMarker.setX(x);
		    goalMarker.setY(y);
		    mapMarkerLayer.add(goalMarker);

		    // Start point and end point are the same
		    goalPoseLine.setPoints([mousePos.x, mousePos.y, mousePos.x + 40, mousePos.y + 40]);
		    mapMarkerLayer.add(goalPoseLine);
		    moving = true;    
		    mapMarkerLayer.drawScene();            
		}

	    });

	    mapStage.on("mousemove touchmove", function(){
		if (moving) {
		    if (is_touch_device) var mousePos = mapStage.getTouchPosition();
		    else var mousePos = mapStage.getMousePosition();
		    var x = mousePos.x;
		    var y = mousePos.y;
		    var startX = goalPoseLine.getPoints()[0].x;
		    var startY = goalPoseLine.getPoints()[0].y;
		    var theta = Math.atan2(mousePos.y - startY, mousePos.x - startX);
		    var endX = 50 * Math.cos(theta);
		    var endY = 50 * Math.sin(theta);
		    goalPoseLine.setPoints([startX, startY, startX + endX, startY + endY]);
		    moving = true;
		    mapMarkerLayer.drawScene();
		}
	    });

	    mapStage.on("mouseup touchend", function(){
		moving = false; 
	    });

	    // Add the image to the map layer
	    mapLayer.add(map);
	    mapRobotLayer.add(robotMarker);

	    // Add the map layer to the map stage
	    mapStage.add(mapLayer);
	    mapStage.add(mapMessageLayer);
	    mapStage.add(mapMarkerLayer);
	    mapStage.add(mapRobotLayer);
	};

	// Connect to the map server to get the map URL
	mapImage.src = "http://" + thisHost + mapFile;

    });
    connection.callService('/rosjs/subscribe', '["' + mapMetadataTopic + '",-1]', function(e) {
        log('Subscribed to ' + mapMetadataTopic);
    });

    // Subscribe to the /odom topic to get the robot's position. Throttle to 10 Hz.
    connection.addHandler(odomTopic, function(msg) {
	odomPositionX = msg.pose.pose.position.x;
	odomPositionY = msg.pose.pose.position.y;
	var x =  (odomPositionX - mapOriginX) / mapResolution * mapScale;
	var y = (mapHeight - (odomPositionY - mapOriginY) / mapResolution) * mapScale;
	if (! usingAMCL) {
	    try {
		robotMarker.setX(x);
		robotMarker.setY(y);
		mapRobotLayer.draw();
	    } catch (err) {
		return;
	    }
	}
    });

    connection.callService('/rosjs/subscribe', '["' + odomTopic + '", 100]', function(e) {
        log('Subscribed to ' + odomTopic);
    });

    // Subscribe to the /amcl_pose topic to get the robot's position during SLAM.
    // Throttle to 10 Hz.
    connection.addHandler(amclPoseTopic, function(msg) {
	usingAMCL = true;
	amclPositionX = msg.pose.pose.position.x;
	amclPositionY = msg.pose.pose.position.y;
	var x =  (amclPositionX - mapOriginX) / mapResolution * mapScale;
	var y = (mapHeight - (amclPositionY - mapOriginY) / mapResolution) * mapScale;
	if (usingAMCL) {
	    try {
		robotMarker.setX(x);
		robotMarker.setY(y);
		mapRobotLayer.draw();
	    } catch (err) {
		return;
	    }
	}
    });

    connection.callService('/rosjs/subscribe', '["' + amclPoseTopic + '", 100]', function(e) {
        log('Subscribed to ' + amclPoseTopic);
    });

    // Subscribe to the /laptop_battery topic and display the charge status.
    // Throttle to 1 Hz.
    connection.addHandler(laptopBatteryTopic, function(msg) {
	var remaining = msg.percentage;
	var charging = msg.charge_state;
	var battery_meter = document.getElementById("laptop_battery");
        battery_meter.value = remaining;
	var laptop_charging_status = document.getElementById("laptop_charging");
	var laptop_battery_status = document.getElementById("laptop_battery_status");
	if (charging) {
	    laptop_charging_status.innerHTML = ' <img src="images/battery_charging.jpg" align="absmiddle" height="36">';
	    laptop_battery_status.innerHTML = 'Laptop Charging';
	    laptop_battery_status.style.fontSize = '16px';
	}
	else {
	    laptop_charging_status.innerHTML = '&nbsp;<p>';
	    laptop_battery_status.style.fontSize = '16px';
	    laptop_battery_status.innerHTML = 'Laptop Battery';
	}

    });
    connection.callService('/rosjs/subscribe', '["' + laptopBatteryTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  laptopBatteryTopic);
    });

    // Subscribe to the joint_states topic so we can read the current servo positions at startup. Throttle to 1 Hz.
    connection.addHandler(jointStatesTopic, function(msg) {
	headPanPosition = msg.position[0];
	headTiltPosition = msg.position[1];
    });
    connection.callService('/rosjs/subscribe', '["' + jointStatesTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  jointStatesTopic);
    });

    // Subscribe to the head pan state topic to get servo temperature. Throttle to 1 Hz.
    connection.addHandler(headPanTempTopic, function(msg) {
	var temp = msg.motor_temps[0];
        var color;
        $('#headPanTemp').jqxLinearGauge('value', temp);
        if (temp > 50) { servoStatus[0] = 2; }
        else if (temp > 45) { servoStatus[0] = 1; }
        else { servoStatus[0] = 0; }
	color = servoStatusColors[servoStatus[0]];
	$('#headPanTemp').jqxLinearGauge({pointer: {pointerType: 'default', size: 30, offset: 0, style: {fill: color}}});
	var maxServoStatus = Math.max(servoStatus[0], servoStatus[1]);
	var servo_status_button = document.getElementById("servo_status");
	servo_status_button.style.background = servoStatusColors[maxServoStatus];
    });
    connection.callService('/rosjs/subscribe', '["' + headPanTempTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  headPanTempTopic);
    });

    // Subscribe to the head tilt state topic to get servo temperature.  Throttle to 1 Hz.
    connection.addHandler(headTiltTempTopic, function(msg) {
	var temp = msg.motor_temps[0];
        var color;
        $('#headTiltTemp').jqxLinearGauge('value', temp);
        if (temp > 50) { servoStatus[1] = 2; }
        else if (temp > 45) { servoStatus[1] = 1; }
        else { servoStatus[1] = 0; }
	color = servoStatusColors[servoStatus[1]];
	$('#headTiltTemp').jqxLinearGauge({pointer: {pointerType: 'default', size: 30, offset: 0, style: {fill: color}}});
	var maxServoStatus = Math.max(servoStatus[0], servoStatus[1]);
	var servo_status_button = document.getElementById("servo_status");
	servo_status_button.style.background = servoStatusColors[maxServoStatus];
    });
    connection.callService('/rosjs/subscribe', '["' + headTiltTempTopic + '", 1000]', function(e) {
        log('Subscribed to ' +  headTiltTempTopic);
    });

    // Subscribe to the /roi topic to get the region of interest.  Throttle to 10 Hz.
    connection.addHandler(roiTopic, function(msg) {
	roiOffsetX = msg.x_offset;
	roiOffsetY = msg.y_offset;
	roiWidth = msg.width;
	roiHeight = msg.height;
	roiMarker.setX(roiOffsetX);
	roiMarker.setY(roiOffsetY);
	roiMarker.setWidth(roiWidth);
	roiMarker.setHeight(roiHeight);
	videoMarkerLayer.draw();
    });
    connection.callService('/rosjs/subscribe', '["' + roiTopic + '", 100]', function(e) {
	log('Subscribed to ' + roiTopic);
    });

    // Subscribe to the /obstacle_laser topic to determine if there is an obstacle ahead
    connection.addHandler(obstacleLaserTopic, function(msg) {
	if (laserAutoStop && msg.data < autoStopRange) laserObstacleDetected = true;
	else laserObstacleDetected = false;
    });
    connection.callService('/rosjs/subscribe', '["' + obstacleLaserTopic + '", -1]', function(e) {
	log('Subscribed to ' + obstacleLaserTopic);
    });

    // Subscribe to the base laser /scan topic. Throttle to 1Hz
    connection.addHandler(baseLaserScanTopic, function(msg) {
	videoLaserLayer.removeChildren();
	if (video2Topic != null) {
	    video2MarkerLayer.removeChildren();
	}
	var nRanges = msg.ranges.length;
	var minAngle = msg.angle_min;
	var maxAngle = msg.angle_max;
	var angleIncrement = msg.angle_increment;
	var maxRange = msg.range_max;
	var minRange = msg.range_min;
	var x = 0;
	var y = 0;
	var xScale = 250;
	var yScale = 25;
	var index = 0;
	var step = 15;
	var vizScale = 1.5;
	var angle = minAngle;
	var safeRange = 1.2;
	var color = "red";

	for (i = 0; i < nRanges; i+=step) {
	    range = msg.ranges[i];
	    angle += angleIncrement * step;
	    if (range < minRange || range > safeRange || range > maxRange) continue;
	    /*
	    if (range < 1.5) color = "red";
	    else if (range < 2.5) color = "yellow";
	    else color = "green";
	    */
	    range = Math.min(range, maxRange);
	    range *= vizScale;
	    x = -range * Math.sin(angle) * xScale;
	    y = range * Math.cos(angle) * yScale;
	    x = (x + videoStage.getX()) + videoStageWidth / 2;
	    y = videoStageHeight - maxRange * vizScale;
	    //	    y = videoStage.getY() - y + videoStageHeight - 5 * (6 - range);
	    addObstacle(x, y, 3 * (vizScale * maxRange - range), color, videoLaserLayer);
	    if (video2Topic != null) {
		addObstacle(x, y, 3 * (vizScale * maxRange - range), color, video2LaserLayer);
	    }
	    index++;
	}
	videoLaserLayer.draw();
	if (video2Topic != null) {
	    video2LaserLayer.draw();
	}
    });

    connection.callService('/rosjs/subscribe', '["' + baseLaserScanTopic + '", 200]', function(e) {
        log('Subscribed to ' + baseLaserScanTopic);
    });

    connection.callService(headPanSpeedService, '{"speed":' + defaultHeadPanSpeed + '}', function() {}); 
    connection.callService(headTiltSpeedService, '{"speed":' + defaultHeadTiltSpeed + '}', function() {}); 

    // Set failsafe max linear and angular speeds in case the parameters are not picked up
    if (maxLinearSpeed == null) {
	maxLinearSpeed = defaultMaxLinearSpeed;
	$('#baseLinearSpeedSlider').jqxSlider('value', maxLinearSpeed);
    }

    if (maxAngularSpeed == null) {
	maxAngularSpeed = defaultMaxAngularSpeed;
	$('#baseAngularSpeedSlider').jqxSlider('value', maxAngularSpeed);
    }	

    // Fixed speed control.  Also requires use hold down key to keep moving
    function fixedSpeedControl(code, down) {
	// Stop if no key is depressed
	if (!down) {
	    stopRobot();
	    return;
	}

	// Stop if space bar his pressed
	if (code == 32) {
            // Space bar
	    vx = 0;
	    vz = 0;
	}
        else if (code == 37 || code == 65) {
            // Left arrow or "a"
	    if (vz < 0) vz = 0;
	    else vz = vz_fixed_speed;
	}
        else if (code == 38 || code == 87) {
            // Up arrow or "w" 
	    vz = 0;
	    if (vx < 0) vx = 0;
            else vx = vx_fixed_speed;
	}
        else if (code == 39 || code == 68) {
            // Right arrow or "d"
	    if (vz > 0) vz = 0;
	    else vz = -vz_fixed_speed;
	}
	else if (code == 40 || code == 88) {
            // Down arrow or "x"
	    vz = 0;
	    if (vx > 0) vx = 0;
	    vx = -vx_fixed_speed;
	}
	// Prevent a run away robot!
	//timedStop();
    }


    // Variable speed control.  Robot keeps moving even without key press
    function variableSpeedControl(code, down) {
	// Use space bar to stop
	if (code == 32) {
            // Space bar
	    vx = 0;
	    vz = 0;
	}
        else if (code == 37 || code == 65) {
            // Left arrow or "a"
	    if (vz < 0) vz = 0;
	    else vz += vz_key_increment;
	}
        else if (code == 38 || code == 87) {
            // Up arrow or "w" 
	    vz = 0;
	    //if (vx < 0) vx = 0;
	    //else if (vx == 0) vx = minLinearSpeed;
	    //else vx += vx_key_increment;
            vx += vx_key_increment;
	}
        else if (code == 39 || code == 68) {
            // Right arrow or "d"
	    if (vz > 0) vz = 0;
	    else vz -= vz_key_increment;
	}
	else if (code == 40 || code == 88) {
            // Down arrow or "x"
	    vz = 0;
	    //if (vx > 0) vx = 0;
	    //else if (vx == 0) vx = -minLinearSpeed;
	    //else vx -= vx_key_increment;
	    vx -= vx_key_increment;
	}
	// Prevent a run away robot!
	//timedStop();
    }

    document.addEventListener('keydown', function (e) {
	if (e.shiftKey) shiftKey = true;
	if (useFixedSpeedControl) fixedSpeedControl(e.keyCode, true);
	else variableSpeedControl(e.keyCode, true);
    }, true);

    document.addEventListener('keyup', function (e) {
	shiftKey = false;
	if (useFixedSpeedControl) fixedSpeedControl(e.keyCode, false);
	else variableSpeedControl(e.keyCode, false);
    }, true);

    // Start the video and publisher loops
    videoOn();
    pubLoop();
});

// Draw the position of the speed control on the base pad
function updateBasePadMarker(vx, vz) {
    markerX = vz / maxAngularSpeed;
    markerY = vx / maxLinearSpeed;
    markerX *= basePadWidth / 2;
    markerX = basePad.getX() + basePadWidth / 2 - markerX;
    markerY *= basePadHeight / 2;
    markerY = basePad.getY() + basePadHeight / 2 - markerY;
    basePadMarker.setX(markerX);
    basePadMarker.setY(markerY);
    baseMarkerLayer.draw();
//    writeMessage(baseMessageLayer, " vx: " + Math.round(vx * 100)/100 + ", vz: " + Math.round(vz*100)/100);
    writeMessageById("base_messages", " vx: " + Math.round(vx * 100)/100 + ", vz: " + Math.round(vz*100)/100);
}

function twistMsg(x, z) {
    return '{"linear":{"x":' + x + ',"y":0,"z":0},"angular":{"x":0,"y":0,"z":' + z + '}}';
}

function pubCmdVel(_vx, _vz) {
    // If an obstacle is detected in front, prevent forward motion
    if (_vx > 0 && (laserObstacleDetected || sonarFrontObstacleDetected)) {
	_vx = 0;
    }

    else if (_vx < 0 && sonarBackObstacleDetected) {
	_vx = 0;
    }

    vx = sign(_vx) * Math.min(maxLinearSpeed, Math.abs(_vx));
    vz = sign(_vz) * Math.min(maxAngularSpeed, Math.abs(_vz));
    connection.publish(cmdVelTopic, 'geometry_msgs/Twist', twistMsg(vx, vz));
    updateCmdVelMarker();
    writeMessageById("base_messages", " vx: " + Math.round(vx * 100)/100 + ", vz: " + Math.round(vz*100)/100);

}

function moveBaseMsg(x, y, qz, qw) {
    return '{"header":{"frame_id":"/map"},"pose":{"position":{"x":' + x + ',"y":' + y + ',"z":0},"orientation":{"x":0,"y":0,"z":' + qz + ',"w":' + qw + '}}}';
}

function poseMsg(x, y, qz, qw) {
    // NOTE: The time stamp *must* be set to 0,0 for this to work.
    return '{"header":{"frame_id":"/map","stamp":{"secs":0, "nsecs":0}},"pose":{"pose":{"position":{"x":' + x + ',"y":' + y + ',"z":0},"orientation":{"x":0,"y":0,"z":' + qz +',"w":' + qw + '}}}}';
}

function setPose() {
    var x =  goalMarker.getX() * mapResolution / mapScale + mapOriginX;
    var y =  (mapHeight * mapScale - goalMarker.getY()) * mapResolution / mapScale + mapOriginY;
    var dx = goalPoseLine.getPoints()[1].x - goalPoseLine.getPoints()[0].x;
    var dy = goalPoseLine.getPoints()[1].y - goalPoseLine.getPoints()[0].y;
    var theta = -Math.atan2(dy, dx);
    var qz = Math.sin(theta / 2);
    var qw = Math.cos(theta / 2);
    connection.publish(initialPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));
    connection.publish(amclPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));

    // Not sure why, but publishing has to be done twice--but on the first time!
    if (firstSetPose) {
	sleep(500);
	connection.publish(initialPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));
	connection.publish(amclPoseTopic, 'geometry_msgs/PoseWithCovarianceStamped', poseMsg(x, y, qz, qw));
	firstSetPose = false;
    }
}

function moveBase() {
    var x =  goalMarker.getX() * mapResolution / mapScale + mapOriginX;
    var y =  (mapHeight * mapScale - goalMarker.getY()) * mapResolution / mapScale + mapOriginY;
    var dx = goalPoseLine.getPoints()[1].x - goalPoseLine.getPoints()[0].x;
    var dy = goalPoseLine.getPoints()[1].y - goalPoseLine.getPoints()[0].y;
    var theta = -Math.atan2(dy, dx);
    var qz = Math.sin(theta / 2);
    var qw = Math.cos(theta / 2);
    connection.publish(moveBaseTopic, 'geometry_msgs/PoseStamped', moveBaseMsg(x, y, qz, qw));

    // Not sure why, but publishing has to be done twice--but on the first time!
    if (firstMoveBase) {
	sleep(500);
	connection.publish(moveBaseTopic, 'geometry_msgs/PoseStamped', moveBaseMsg(x, y, qz, qw));
	firstMoveBase = false;
    }
}

function moveBaseCancel() {
    connection.publish('/move_base/cancel', 'actionlib_msgs/GoalID', '{"id":"0"}');

    // Not sure why, but publishing has to be done twice--but only the first time!
    if (firstMoveBaseCancel) {
	sleep(50);
	connection.publish('/move_base/cancel', 'actionlib_msgs/GoalID', '{"id":"0"}');
	firstMoveBaseCancel = false;
    }
}

function pubHeadCmd() {
    if (centerServos) {
	videoMarkerLayer.draw();
        servosRelaxed = false;
	centerServos = false;
        connection.publish(headPanPositionTopic, 'std_msgs/Float64', '{"data": 0}');
        connection.publish(headTiltPositionTopic, 'std_msgs/Float64', '{"data": 0}');
	
    }
    else if (lockServos) {
        servosRelaxed = false;
	centerServos = false;
	lockServos = false;
        connection.publish(headPanPositionTopic, 'std_msgs/Float64', '{"data": ' + headPanPosition[0] + '}');
        connection.publish(headTiltPositionTopic, 'std_msgs/Float64', '{"data": ' + headTiltPosition[1] + '}');
	
    }
    else if (relaxServos && !servosRelaxed) {
        servosRelaxed = true;
        connection.callService(headPanTorqueService, '{"torque_enable": false}', function() {}); 
        connection.callService(headTiltTorqueService, '{"torque_enable": false}', function() {}); 
    }
    else {
	videoMarkerLayer.draw();
        servosRelaxed = false;
        connection.publish(headPanPositionTopic, 'std_msgs/Float64', '{"data":' + desiredHeadPanPosition + '}');
        connection.publish(headTiltPositionTopic, 'std_msgs/Float64', '{"data":' + desiredHeadTiltPosition + '}');
    }
}

function updateCmdVelMarker() {
    var x = videoStageWidth / 2 - vz * 100 * videoScale;
    var y = videoStageHeight - vx * 500 * videoScale - 100;

    if (useFixedSpeedControl && vx != 0) {
	y -= 100;
    }

    if (useFixedSpeedControl && vz != 0) {
	y = videoStageHeight - 200;
	x += 2;
    }

    if (vx == 0 && vy == 0) y = videoStageHeight;

    cmdVelMarker.setPoints([videoStageWidth/2, videoStageHeight, x, y]);

    videoMarkerLayer.draw();
}

function relaxAllServos() {
    relaxServos = true;
    servosRelaxed = false;
    pubHeadCmd();
}

function centerHeadServos() {
    centerServos = true;
    lockServos = false;
    headPanSpeed = defaultHeadPanSpeed;
    headTiltSpeed = defaultHeadTiltSpeed;
    desiredHeadTiltPosition = desiredHeadPanPosition = 0;
    pubHeadCmd();
}

function drivePanTilt() {
    centerServos = false;
    lockServos = false;
    headPanSpeed = defaultHeadPanSpeed;
    headTiltSpeed = defaultHeadTiltSpeed;
    desiredHeadTiltPosition = 0.45;
    desiredHeadPanPosition = 0;
    pubHeadCmd();
}

function driveRobot(direction) {
    if (direction == "forward") {
	vz = 0;
	if (vx < 0) vx = 0;
	else if (vx == 0) vx = minLinearSpeed;
	else vx += vx_key_increment;
    }
    else if (direction == "backward") {
	vz = 0;
	if (vx > 0) vx = 0;
	else if (vx == 0) vx = -minLinearSpeed;
	else vx -= vx_key_increment;
    }
    else if (direction == "left") {
	if (vz < 0) vz = 0;
	else if (vz == 0) vz = minAngularSpeed;
	else vz += vz_key_increment;
    }
    else if (direction == "right") {
	if (vz > 0) vz = 0;
	else if (vz == 0) vz = -minAngularSpeed;
	else vz -= vz_key_increment;
    }
    updateBasePadMarker(vx, vz);
}


function stopRobot() {
    pubCmdVel(0, 0);
    updateBasePadMarker(0, 0);
//    writeMessage(baseMessageLayer, "Stopping robot");
    writeMessageById("base_messages", "Stopping robot");
    clearInterval(stopHandle);
}

function refreshVideo() {
    if (video1_On) {
	videoStage.draw();
	//videoMarkerLayer.draw();
    }
    else if (video2_On && video2Topic != null) video2Stage.draw();
}

function refreshPublishers() {
//    if (mouseDown) {
    pubCmdVel(vx, vz);
//    }

    if ((laserObstacleDetected || sonarFrontObstacleDetected)) {
	//writeMessage(videoMessageLayer, "Obstacle in Front!");
	writeMessageById("video_messages", "Obstacle in Front!", "red");

	if (video2Topic != null) {
	    writeMessage(video2MessageLayer, "Obstacle in Front!");
	}
    }

    else if (sonarBackObstacleDetected) {
	//writeMessage(videoMessageLayer, "Obstacle Behind!");
	writeMessageById("video_messages", "Obstacle Behind!", "red");

	if (video2Topic != null) {
	    writeMessage(video2MessageLayer, "Obstacle Behind!");
	}
    }

    else {
	writeMessageById("video_messages", "");
	//videoMessageLayer.clear();
	if (video2Topic != null) {
	    video2MessageLayer.clear();
	}
    }
}

function pubLoop() {
    log("Starting publishers");
    pubHandle = setInterval(refreshPublishers, 1000 / rate);
}


function timedStop() {
    stopHandle = setInterval(deadmanStop, 100);
}

function deadmanStop() {
    vx = vz = 0;
    clearInterval(stopHandle);
    updateBasePadMarker(0, 0);
}

function clearClickMarker() {
    clickMarker.remove();
    clearInterval(clickMarkerHandle);
}

function toggleVideo() {
    var video1 = document.getElementById("video_container");
    var video2 = document.getElementById("video2_container");
    if (video1.style.display == "block") {
    	video1.style.display = "none";
    	video2.style.display = "block";
	video1_On = false;
	video2_On = true;
    }
    else {
	video1.style.display = "block";
	video2.style.display = "none";
	video1_On = true;
	video2_On = false;
    }    
}

function setAutoStopRange(range) {
    // Convert from feet to meters
    autoStopRange = range * 12 * 2.54 / 100.0;
}

function setFixedLinearSpeed(spd) {
    vx_fixed_speed = spd;
}

function setFixedAngularSpeed(spd) {
    vz_fixed_speed = spd;
}

function toggleSonarAutoStop() {
    var sonar_auto_stop = document.getElementById("sonar_auto_stop");
    if (sonar_auto_stop.checked) {
	sonar_auto_stop.style.checked = false;
	sonarAutoStop = true;
    }
    else {
	sonar_auto_stop.style.checked = true;
	sonarAutoStop = false;
    }    
}

function toggleLaserAutoStop() {
    var laser_auto_stop = document.getElementById("laser_auto_stop");
    if (laser_auto_stop.checked) {
	laser_auto_stop.style.checked = false;
	laserAutoStop = true;
    }
    else {
	laser_auto_stop.style.checked = true;
	laserAutoStop = false;
    }    
}

function videoOn() {
    <!-- Redraw of video canvas every 1000 / videoFPS ms -->
    videoHandle = setInterval(refreshVideo, 1000 / videoFPS);
/*
    video_on.style.background = '#55bb55';
    video_on.style.color = 'white';
    video_off.style.background = '#DDDDDD';
    video_off.style.color = 'black';
*/
}

/*
function videoOff() {
    clearInterval(videoHandle);
    video_off.style.background = '#f24537';
    video_off.style.color = 'white';
    video_on.style.background = '#DDDDDD';
    video_on.style.color = 'black';
}
*/

function sleep(delay) {
    var start = new Date().getTime();
    while (new Date().getTime() < start + delay);
}

function sign(x)
{
    if (x < 0) { return -1; }
    if (x > 0) { return 1; }
    return 0;
}

// Begin KineticJS scripts and parameters
var basePadWidth = 250;
var basePadHeight = 250;
var servoPadWidth = 250;
var servoPadHeight = 250;
/*
var mapWidth = 237;
var mapHeight = 233;
var mapResolution = 0.05;
var mapOriginX = -6.0;
var mapOriginY = -5.0;
*/
var mapScale = 2.0;

function writeMessage(messageLayer, message) {
    var context = messageLayer.getContext();
    messageLayer.removeChildren();
    messageLayer.clear();
    context.font = "24pt Calibri";
    context.fillStyle = "yellow";
    context.fillText(message, 10, 25);
}

function writeMessageById(id, message, color) {
    color = typeof color !== 'undefined' ? color: "#006600";
    element = document.getElementById(id);
    element.innerHTML = message;
    element.style.font = "18pt Calibri";
    element.style.color = color;
}

var videoStage = new Kinetic.Stage({
    container: "video_container",
    draggable: false,
    width: videoStageWidth,
    height: videoStageHeight,
    x: 0,
    y: 0
});

function getDistance(touch1, touch2) {
    var x1 = touch1.clientX;
    var x2 = touch2.clientX;
    var y1 = touch1.clientY;
    var y2 = touch2.clientY;
    
    return Math.sqrt(((x2 - x1) * (x2 - x1)) + ((y2 - y1) * (y2 - y1)));
}

var startDistance = undefined;
var startScale = 1;

videoStage.on("tap click", function(evt) {
    if (is_touch_device) var mousePos = videoStage.getTouchPosition();
    else var mousePos = videoStage.getMousePosition();
    var x =  mousePos.x;
    var y =  mousePos.y;
    if (shiftKey) doClickNav(x, y);
    else doClickPanTilt(x, y);
});

function doClickPanTilt(x, y) {
    clickMarker.setStroke(green);
    clickMarker.setX(x);
    clickMarker.setY(y);
    videoMarkerLayer.add(clickMarker);
    videoMarkerLayer.draw();
    clickMarkerHandle = setInterval(clearClickMarker, 250);
    centerServos = false;
    lockServos = false;
    headPanSpeed = defaultHeadPanSpeed;
    headTiltSpeed = defaultHeadTiltSpeed;
    // Convert mouse position to relative head angle
    var head_pan =  fovWidthRadians * (videoWidth * videoScale / 2 - x) / (videoWidth * videoScale);
    var head_tilt = fovHeightRadians * (y - videoHeight * videoScale / 2) / (videoHeight * videoScale);
    desiredHeadPanPosition = headPanPosition + head_pan;
    desiredHeadTiltPosition = headTiltPosition + head_tilt;
    desiredHeadPanPosition = Math.min(Math.max(-maxHeadPanPosition, desiredHeadPanPosition), maxHeadPanPosition);
    desiredHeadTiltPosition = Math.min(Math.max(-maxHeadTiltPosition, desiredHeadTiltPosition), maxHeadTiltPosition);
    pubHeadCmd();
}

function doClickNav(x, y) {
    clickMarker.setStroke('red');
    clickMarker.setX(x);
    clickMarker.setY(y);
    videoMarkerLayer.add(clickMarker);
    videoMarkerLayer.draw();
    clickMarkerHandle = setInterval(clearClickMarker, 250);

    // Compute the angle to rotate based on head pan and mouse click x coordinate
    var rotate = desiredHeadPanPosition -  fovWidthRadians * ((x / videoScale - videoWidth / 2) / videoWidth);
    log(rotate);

    // Compute the distance to travel based on head tilt and camera height
/*
    log(headTiltPosition);
    var theta = (Math.PI / 2 - headTiltPosition) -  fovHeightRadians * ((y / videoScale - videoHeight / 2) / videoHeight);
    var distance = Math.tan(theta) * cameraHeight;
    log(fovHeightRadians * ((y / videoScale - videoHeight / 2) / videoHeight));
    log (distance);
*/
}

var zoomVideo = function(e) {
    var zoomAmount = e.wheelDeltaY*0.001;
    var scale = videoLayer.getScale().x + zoomAmount;
    videoLayer.setScale(scale);
    var x = videoStage.getWidth() * (1 - scale) / 2;
    var y = videoStage.getHeight() * (1 - scale) / 2;
    videoLayer.setPosition(x, y);
    videoLayer.draw();
}

document.addEventListener("mousewheel", zoomVideo, false);

videoStage.on("touchmove", function(evt) {
    var touch1 = evt.touches[0];
    var touch2 = evt.touches[1];
    
    if (touch1 && touch2) {
        if (startDistance === undefined) {
            startDistance = getDistance(touch1, touch2);
        }
        else {
            var dist = getDistance(touch1, touch2);
            var scale = (dist / startDistance) * startScale;
            videoStage.setScale(scale);	    

            // center layer
            var x = videoStage.getWidth() * (1 - scale) / 2;
            var y = videoStage.getHeight() * (1 - scale) / 2;
            videoLayer.setPosition(x, y);

            videoStage.draw();
        }
    }
});

videoStage.on("touchend", function() {
    startDistance = undefined;
    startScale = videoStage.scale.x;
});


if (video2Topic != null) {
    var video2Stage = new Kinetic.Stage({
	container: "video2_container",
	draggable: false,
	width: videoWidth * videoScale,
	height: videoHeight * videoScale,
	x: 0,
	y: 0
    });
}

var baseStage = new Kinetic.Stage({
    container: "base_container",
    draggable: false,
    width: basePadWidth,
    height: basePadHeight,
    x: 0,
    y: 0
});

var videoLayer = new Kinetic.Layer();
var videoLaserLayer = new Kinetic.Layer();
var videoMarkerLayer = new Kinetic.Layer();
var videoMessageLayer = new Kinetic.Layer();

if (video2Topic != null) {
    var video2Layer = new Kinetic.Layer();
    var video2LaserLayer = new Kinetic.Layer();
    var video2MarkerLayer = new Kinetic.Layer();
    var video2MessageLayer = new Kinetic.Layer();
}

var baseLayer = new Kinetic.Layer();
var baseMarkerLayer = new Kinetic.Layer();
var baseMessageLayer = new Kinetic.Layer();
var mapLayer = new Kinetic.Layer();
var mapMarkerLayer = new Kinetic.Layer();
var mapMessageLayer = new Kinetic.Layer();
var mapRobotLayer = new Kinetic.Layer();

function addObstacle(x, y, size, color, layer) {
    var maxSize = 36;
    var obstacle = new Kinetic.Circle({
        x: x,
        y: y,
        radius: size,
	opacity: size/maxSize / 1.5,
        fill: color
    });
    try {
	layer.add(obstacle);
    }
    catch(err) {
	return;
    }
}

<!-- The Video Panel -->
var videoImage = new Image();
videoImage.onload = function() {
    video = new Kinetic.Image({
	image: videoImage,
	width: videoWidth * videoScale,
	height: videoHeight * videoScale,
	offset: [0, 0],
	draggable: false,
	x: 0,
	y: 0
    });

    videoLayer.add(video);
}

videoStage.add(videoLayer);
videoStage.add(videoLaserLayer);
videoStage.add(videoMarkerLayer);
videoStage.add(videoMessageLayer);
//videoMarkerLayer.add(roiMarker);

<!-- The 2nd Video Panel -->
if (video2Topic != null) {
    var video2Image = new Image();
    var video2 = new Kinetic.Image({});
    video2Image.onload = function() {
	video2 = new Kinetic.Image({
	    image: video2Image,
	    width: videoWidth * videoScale,
	    height: videoHeight * videoScale,
	    offset: [0, 0],
	    x: 0,
	    y: 0
	});

	video2Layer.add(video2);
    }

    video2Stage.add(video2Layer);
    video2Stage.add(video2MarkerLayer);
    //video2Stage.add(video2MessageLayer);
    //video2MarkerLayer.add(roiMarker);
}

<!-- The BasePad -->
var basePad = new Kinetic.Rect({
    x: 0,
    y: 0,
    width: basePadWidth,
    height: basePadHeight,
    offset: [0, 0],
    fill: "#00D2FF",
    stroke: "black",
    strokeWidth: 1
});

var basePadVerticalLine = new Kinetic.Line({
    points: [basePadWidth/2, 0, basePadWidth/2, basePadHeight],
    stroke: "black",
    strokeWidth: 1,
    listening: false
});

var basePadHorizontalLine = new Kinetic.Line({
    points: [0, basePadHeight/2, basePadWidth, basePadHeight/2],
    stroke: "black",
    strokeWidth: 1,
    listening: false
});

var basePadMarker = new Kinetic.Circle({
    x: basePadWidth/2,
    y: basePadHeight/2,
    radius: 25,
    listening: false,
    fill: "yellow",
    stroke: "black",
    strokeWidth: 1
});

<!-- /cmd/vel feedback -->
var cmdVelMarker = new Kinetic.Line({
    points: [videoStageWidth/2, videoStageHeight],
    listening: false,
    strokeWidth: 20,
    opacity: 0.4,
    lineCap: "round",
    stroke: "#00CC00"
});

basePad.on("mousedown touchstart touchmove", function() {
    mouseDown = true;
/*
    this.setFill({
	image: basePadImage,
	width: basePadWidth,
	height: basePadHeight
    });
*/
    baseStage.draw();
});

basePad.on("mousemove touchmove", function() {
    if (! is_touch_device && ! mouseDown) { return; }
    if (is_touch_device) var mousePos = baseStage.getTouchPosition();
    else var mousePos = baseStage.getMousePosition();
    var x = (mousePos.x - basePad.getX()) - basePadWidth / 2;
    var y = basePadHeight / 2 - (mousePos.y - basePad.getY());
    x /= basePadWidth / 2;
    y /= basePadHeight / 2;
    vx = sign(y) * (Math.pow(2, Math.abs(y)) - 1) * maxLinearSpeed;
    vz = -sign(x) * (Math.pow(2, Math.abs(x)) - 1) * maxAngularSpeed;
    updateBasePadMarker(vx, vz);
    cmdVelMarkerY = mousePos.y * 4;
    cmdVelMarker.setPoints([videoStageWidth/2, videoStageHeight, mousePos.x + (videoStageWidth - basePadWidth)/2, cmdVelMarkerY]);
    if (Math.abs(vz) < deadZoneVz) vz = 0;
    writeMessageById("base_messages", " vx: " + Math.round(y * 100)/100 + ", vz: " + Math.round(x*100)/100, "green");
    pubCmdVel(vx, vz);
});


// contextmenu dragend touchleave mouseout 
basePad.on("touchend mouseup dblclick", function() {
    mouseDown = false;
    pubCmdVel(0, 0);
    basePadMarker.setX(basePadWidth/2);
    basePadMarker.setY(basePadHeight/2);
    cmdVelMarker.setPoints([videoStageWidth/2, videoStageHeight]);
    baseMarkerLayer.drawScene();
    // writeMessage(baseMessageLayer, "Stopping robot");
    writeMessageById("base_messages", "Stopping robot");
});

baseLayer.add(basePad);
baseLayer.add(basePadVerticalLine);
baseLayer.add(basePadHorizontalLine);
baseMarkerLayer.add(basePadMarker);

videoMarkerLayer.add(cmdVelMarker);
videoMarkerLayer.draw();

function zoomInMap() {
    mapStage.setWidth(mapStage.getWidth() * 1.1);
    mapStage.setHeight(mapStage.getHeight() * 1.1);
    map.setWidth(map.getWidth() * 1.1);
    map.setHeight(map.getHeight() * 1.1);
    mapLayer.draw();
}

function zoomOutMap() {
    mapStage.setWidth(mapStage.getWidth() / 1.1);
    mapStage.setHeight(mapStage.getHeight() / 1.1);
    map.setWidth(map.getWidth() / 1.1);
    map.setHeight(map.getHeight() / 1.1);
    mapLayer.draw();
}

baseStage.add(baseLayer);
baseStage.add(baseMarkerLayer);
baseStage.add(baseMessageLayer);

mapLayer.draw();
mapRobotLayer.draw();
