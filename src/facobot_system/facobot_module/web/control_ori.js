var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var ros;
var scale = 0.5;
var lin = 0;
var ang = 0;
// var tid = setInterval(sendVelocity, 66); //15Hz

function initWebInputPublisher() {
	// Init message with zero values.
	msg = new ROSLIB.Message({
		data: '0'
	});
	// Init topic object
	web_topic = new ROSLIB.Topic({
		ros: ros,
		name: '/tcp_recv',
		messageType: 'std_msgs/String'
	});
	// Register publisher within ROS system
	web_topic.advertise();
}

function initClick() {
	send1 = document.getElementById("click1");
	send2 = document.getElementById("click2");
	send3 = document.getElementById("click3");
	send4 = document.getElementById("click4");
	send1.onclick = function () {
		msg.data = "1"
		console.log("Task : 1");
		web_topic.publish(msg);
	}
	send2.onclick = function () {
		msg.data = "2"
		console.log("Task : 2");
		web_topic.publish(msg);
	}
	send3.onclick = function () {
		msg.data = "3"
		console.log("Task : 3");
		web_topic.publish(msg);
	}
	send4.onclick = function () {
		msg.data = "4"
		console.log("Task : 4");
		web_topic.publish(msg);
	}
}

// function sendVelocity() {
// 	if (lin != 0 || ang != 0) {
// 		moveAction(lin, ang);
// 	}
// 	// console.log("timmer");
// }

// function abortTimer() { // to be called when you want to stop the timer
// 	clearInterval(tid);
// }

function initKeyboardInput() {
	document.addEventListener('keydown', keyboardMoveDown, true);
	document.addEventListener('keyup', keyboardMoveUp, true);
}

function keyboardMoveDown(event) {
	// Use w, s, a, d keys to drive your robot
	// console.log(event);
	if (event.keyCode == 65) {
		ang = 1
	}
	else if (event.keyCode == 68) {
		ang = -1
	}
	else if (event.keyCode == 87) {
		lin = 1
	}
	else if (event.keyCode == 83) {
		lin = -1
	}
	moveAction(lin, ang);
}

function keyboardMoveUp(event) {
	// Use w, s, a, d keys to drive your robot
	if (event.keyCode == 65) {
		ang = 0
	}
	else if (event.keyCode == 68) {
		ang = 0
	}
	else if (event.keyCode == 87) {
		lin = 0
	}
	else if (event.keyCode == 83) {
		lin = 0
	}
	moveAction(lin, ang);
}

function moveAction(linear, angular) {
	if (linear !== undefined && angular !== undefined) {
		twist.linear.x = linear*scale;
		twist.angular.z = angular*scale;
	} else {
		twist.linear.x = 0;
		twist.angular.z = 0;
	}
	cmdVel.publish(twist);
	// console.log("move");
}

function initVelocityPublisher() {
	// Init message with zero values.
	twist = new ROSLIB.Message({
		linear: {
			x: 0,
			y: 0,
			z: 0
		},
		angular: {
			x: 0,
			y: 0,
			z: 0
		}
	});
	// Init topic object
	cmdVel = new ROSLIB.Topic({
		ros: ros,
		name: '/cmd_vel',
		messageType: 'geometry_msgs/Twist'
	});
	// Register publisher within ROS system
	cmdVel.advertise();
}

function initSlider() {
	// Add event listener for slider moves
	robotSpeedRange = document.getElementById("robot-speed");
	robotSpeedRange.oninput = function () {
		scale = robotSpeedRange.value/100;
		console.log(scale);
	}
}

function createJoystick() {
    var options = {
        zone: document.getElementById('joystick'),
        threshold: 0.1,
		position: { left: 50 + '%', top: 105 + 'px' },
        // position: { right: '8%', bottom: '20%' },
		mode: 'static',
		size: 200,
		color: '#0066ff',
		restJoystick: true
    };
    manager = nipplejs.create(options);

    linear_speed = 0;
    angular_speed = 0;

    manager.on('start', function(event, nipple) {
        timer = setInterval(function() {
            moveAction(linear_speed, angular_speed);
        }, 25);
    });

    manager.on('move', function(event, nipple) {
        max_linear = 0.8; // m/s
        max_angular = 0.8; // rad/s
        max_distance = 75.0; // pixels;
        linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance / max_distance;
        angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance / max_distance;
    });

    manager.on('end', function() {
        if (timer) {
            clearInterval(timer);
        }
        self.moveAction(0, 0);
    });
}


// function createJoystick() {
// 	// Check if joystick was aready created
// 	if (manager == null) {
// 		joystickContainer = document.getElementById('joystick');
// 		// joystck configuration, if you want to adjust joystick, refer to:
// 		// https://yoannmoinet.github.io/nipplejs/
// 		var options = {
// 			zone: joystickContainer,
// 			position: { left: 50 + '%', top: 105 + 'px' },
// 			mode: 'static',
// 			size: 200,
// 			color: '#0066ff',
// 			restJoystick: true
// 		};
// 		manager = nipplejs.create(options);
// 		// event listener for joystick move
// 		manager.on('move', function (evt, nipple) {
// 			// nipplejs returns direction is screen coordiantes
// 			// we need to rotate it, that dragging towards screen top will move robot forward
// 			var direction = nipple.angle.degree - 90;
// 			if (direction > 180) {
// 				direction = -(450 - nipple.angle.degree);
// 			}
// 			// convert angles to radians and scale linear and angular speed
// 			// adjust if youwant robot to drvie faster or slower
// 			lin = Math.cos(direction / 57.29) * nipple.distance * 0.01;
// 			ang = Math.sin(direction / 57.29) * nipple.distance * 0.01;
// 		});
// 		// event litener for joystick release, always send stop message
// 		manager.on('end', function () {
// 			lin = 0
// 			ang = 0
// 			moveAction(lin, ang);
// 		});
// 	}
// }

window.onload = function () {
	// determine robot address automatically
	// robot_IP = location.hostname;
	// set robot address statically
	// robot_IP = "192.168.1.100";
	robot_IP = "localhost";

	// // Init handle for rosbridge_websocket
	ros = new ROSLIB.Ros({
		url: "ws://" + robot_IP + ":9090"
	});

	initWebInputPublisher();
	initVelocityPublisher();
	initKeyboardInput();
	initClick();
	// get handle for video placeholder
	video = document.getElementById('video');
	// Populate video source 
	video.src = "http://" + robot_IP + ":5000/stream?topic=/cam1/color/image_raw&type=mjpeg&quality=80";
	// video.src = "http://" + robot_IP + ":5000/stream?topic=/camera/image_raw&type=mjpeg&quality=80";
	createJoystick();
	initSlider();
	
	// video.onload = function () {
	//     // joystick and keyboard controls will be available only when video is correctly loaded
	// };
}