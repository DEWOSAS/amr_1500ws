var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var ros;
var scale = 0.5;
var lin = 0;
var ang = 0;
var tid = setInterval(sendVelocity, 66); //15Hz


function sendVelocity() {
	if (lin != 0 || ang != 0) {
		moveAction(lin, ang);
	}
	// console.log("timmer");
}

function abortTimer() { // to be called when you want to stop the timer
	clearInterval(tid);
}
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
	const	robotSpeedRange = document.getElementById("robot-speed");
	if (!robotSpeedRange) {
		console.warn("❌ not found element  id='robot-speed'");
		return;
	}
	robotSpeedRange.oninput = function () {
		scale = robotSpeedRange.value/100;
		console.log(scale);
	}
}

// function initMap() {
// 	var viewer = new ROS2D.Viewer({
// 		divID: 'map',
// 		width: 640,
// 		height: 480
// 	});
	
// 	// Init message with zero values.
// 	var gridClient = new ROS2D.OccupancyGridClient({
// 	// var gridClient = new NAV2D.OccupancyGridClientNav({
// 	ros : ros,
// 	topic: '/map',
// 	rootObject : viewer.scene,
// 	continuous : true
// 	});

// 	// Scale the canvas to fit to the map
// 	gridClient.on('change', function () {
// 		viewer.scaleToDimensions(gridClient.currentGrid.width, gridClient.currentGrid.height);
// 		viewer.shift(gridClient.currentGrid.pose.position.x, gridClient.currentGrid.pose.position.y);
// 	});

// }

function initJoystick() {
	// Check if joystick was aready created
	if (manager == null) {
		joystickContainer = document.getElementById('joystick');
		// joystck configuration, if you want to adjust joystick, refer to:
		// https://yoannmoinet.github.io/nipplejs/
		var options = {
			zone: joystickContainer,
			position: { left: 50 + '%', top: 105 + 'px' },
			mode: 'static',
			size: 200,
			color: '#0066ff',
			restJoystick: true
		};
		manager = nipplejs.create(options);
		// event listener for joystick move
		manager.on('move', function (evt, nipple) {
			// nipplejs returns direction is screen coordiantes
			// we need to rotate it, that dragging towards screen top will move robot forward
			var direction = nipple.angle.degree - 90;
			if (direction > 180) {
				direction = -(450 - nipple.angle.degree);
			}
			// convert angles to radians and scale linear and angular speed
			// adjust if youwant robot to drvie faster or slower
			lin = Math.cos(direction / 57.29) * nipple.distance * 0.01;
			ang = Math.sin(direction / 57.29) * nipple.distance * 0.01;
		});
		// event litener for joystick release, always send stop message
		manager.on('end', function () {
			lin = 0
			ang = 0
			moveAction(lin, ang);
		});
	}
}



window.onload = function () {
	// determine robot address automatically
	robot_IP = "172.16.0.210";

	// // Init handle for rosbridge_websocket
	ros = new ROSLIB.Ros({
		url: "ws://" + robot_IP + ":9090"
	});
	initKeyboardInput();
	initVelocityPublisher();
	video = document.getElementById('video');
	video.src = "http://" + robot_IP + ":8080/stream?topic=/map_image/full&type=mjpeg&quality=80";
	var isMobile = /iPhone|iPad|iPod|Android/i.test(navigator.userAgent);

	if (isMobile) {
		video.style.width = "100%";
		video.style.maxWidth = "340px";
		video.style.height = "240px";
	} else {
		video.width = 640;
		video.height = 480;
	}

	initJoystick();
	initSlider();
	document.getElementById('save').addEventListener('click', async () => {
	const mapName = prompt("กรุณาตั้งชื่อแผนที่ (ไม่ต้องใส่นามสกุล):");
		if (mapName) {
			try {
				const response = await fetch(`/save?name=${encodeURIComponent(mapName)}`);
				if (!response.ok) {
					const text = await response.text();
					alert("❌ บันทึกล้มเหลว:\n" + text);
					return;
				}

				const blob = await response.blob();
				const url = window.URL.createObjectURL(blob);
				const a = document.createElement('a');
				a.href = url;
				a.download = `${mapName}.zip`;
				a.click();
				window.URL.revokeObjectURL(url);
			} catch (error) {
				alert("❌ บันทึกล้มเหลว:\n" + error);
			}
		}
	});

	// document.getElementById('toggle-btn').addEventListener('click', () => {
    // fetch('/toggle-slam')
    //     .then(res => res.text())
    //     .then(msg => alert(msg))
    //     .catch(err => alert('Error: ' + err));
	// });
}