var twist;
var cmdVel;
var publishImmidiately = true;
var robot_IP;
var manager;
var ros;
var scale = 0.5;
var lin = 0;
var navActionClient;   // ✅ ประกาศ global
var goal; 
var goalPoseTopic;
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

  	goalPoseTopic = new ROSLIB.Topic({
        ros: ros,
        name: '/goal_pose',  // topic ที่ bt_navigator ฟังอยู่
        messageType: 'geometry_msgs/PoseStamped'
    });
	// Register publisher within ROS system
	web_topic.advertise();
	goalPoseTopic.advertise();
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

function createJoystick() {
	var isMobile = /iPhone|iPad|iPod|Android/i.test(navigator.userAgent);

    var options = {
        zone: document.getElementById('joystick'),
        threshold: 0.1,
        position: { left: '50%', top: '105px' },
        mode: 'static',
        size: isMobile ? 150 : 200,  // ✅ ปรับขนาดเล็กลงบนมือถือ
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

function MapSelection() {
		//-----------------------------------การทำงานของปุ่มอัปโหลดไฟล์ข-------------------------------//
    const uploadButton = document.getElementById('map1');
    const fileInput = document.getElementById('folderInput');
    if (uploadButton && fileInput) {
        uploadButton.addEventListener('click', () => {
            fileInput.click();
        });

        fileInput.addEventListener('change', (event) => {
            const files = event.target.files;
            let pgmFile = null;
            let yamlFile = null;

            if (files.length > 0) {
                for (const file of files) {
                    const ext = file.name.split('.').pop().toLowerCase();
                    if (ext === "pgm") {
                        pgmFile = file;
                        console.log("พบไฟล์ PGM:", file.name);
                    } else if (ext === "yaml") {
                        yamlFile = file;
                        console.log("พบไฟล์ YAML:", file.name);
                    } else {
                        alert(`ไฟล์ ${file.name} ไม่ใช่ .pgm หรือ .yaml`);
                    }
                }

                if (pgmFile && yamlFile) {
                    uploadMap(pgmFile, yamlFile);  // เรียกฟังก์ชันอัปโหลดสองไฟล์
                } else {
                    alert("กรุณาเลือกทั้งไฟล์ .pgm และ .yaml พร้อมกัน");
                }
            }
        });
		//-----------------------------------การทำงานของปุ่มอัปโหลดไฟล์ข-------------------------------//
    }
}
    function yawToQuaternion(yawDeg) {
    var yaw = yawDeg * Math.PI / 180.0;
    return {
        x: 0.0,
        y: 0.0,
        z: Math.sin(yaw / 2.0),
        w: Math.cos(yaw / 2.0)
    };
}

// function uploadPGM(file) {
//     const formData = new FormData();
//     formData.append("pgm_file", file); // ต้องตรงกับ backend

//     fetch("/upload_pgm", {
//         method: "POST",
//         body: formData
//     })
//     .then(response => response.text())
//     .then(result => {
//         console.log("Upload result:", result);
//         alert("อัปโหลดสำเร็จ: " + result);
//     })
//     .catch(error => {
//         console.error("Upload error:", error);
//         alert("เกิดข้อผิดพลาดในการอัปโหลด");
//     });
// }

function uploadMap(pgmFile, yamlFile) {
  const formData = new FormData();
  formData.append("pgm", pgmFile);
  formData.append("yaml", yamlFile);

  fetch("/upload_map", {
    method: "POST",
    body: formData
  })
  .then(response => {
    if (!response.ok) throw new Error("Upload failed");
    return response.json();  // ✅ รับเป็น JSON
  })
  .then(async (data) => {
    // ✅ บันทึกค่าไว้ใน localStorage
    try {
      localStorage.setItem("latestMapDir", data.map_dir);
      localStorage.setItem("latestMapYaml", data.yaml);
    } catch (e) {}

    alert(`✅ อัปโหลดสำเร็จ:\n${data.map_dir}`);
  })
  .catch(error => {
    alert("❌ อัปโหลดล้มเหลว: " + error.message);
    console.error("Upload error:", error);
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
	robot_IP = "172.16.0.210";

	
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
	// video.src = "http://" + robot_IP + ":8080/stream?topic=/camera/camera/color/image_raw&type=mjpeg&quality=80";
	// video.src = "http://" + robot_IP + ":5000/stream?topic=/camera/image_raw&type=mjpeg&quality=80";
	video.src = "http://" + robot_IP + ":8080/stream?topic=/map_to_page_control/full&type=mjpeg&quality=80";	
	//video.style.transform = "rotate(180deg)";

	var isMobile = /iPhone|iPad|iPod|Android/i.test(navigator.userAgent);

	if (isMobile) {
		video.style.width = "100%";
		video.style.maxWidth = "340px";
		video.style.height = "240px";
		// video.style.transform = "rotate(180deg)";
	} else {
		video.width = 640;
		video.height = 480;
		// video.style.transform = "rotate(180deg)";
	}
	
	createJoystick();
	initSlider();
	MapSelection();

	
	document.getElementById("sendPoseBtn").addEventListener("click", function() {
    var x = parseFloat(document.getElementById("poseX").value);
    var y = parseFloat(document.getElementById("poseY").value);
    var yaw = parseFloat(document.getElementById("poseYaw").value);

    var quat = yawToQuaternion(yaw);

    var poseMsg = new ROSLIB.Message({
        header: {
            frame_id: 'map',
            stamp: { sec: 0, nanosec: 0 } // rosbridge จะเติม timestamp ให้เอง
        },
        pose: {
            position: { x: x, y: y, z: 0.0 },
            orientation: quat
        }
    });

    goalPoseTopic.publish(poseMsg);
    console.log("🚀 Published PoseStamped goal:", poseMsg);	
	});


	
	document.getElementById("cancleGoal").addEventListener("click", function() {
    if (goal) {
        goal.cancel();
        console.log("🛑 Goal canceled");
    } else {
        console.warn("⚠️ ยังไม่มี goal ที่ active");
    }
	});

	document.getElementById("startnav2").addEventListener("click", async () => {
    try {
        const res = await fetch("/start_nav");
        const text = await res.text();
        alert(text);   // ✅ แสดง popup เมื่อ start nav
    } catch (err) {
        alert("❌ Error starting navigation: " + err);
    }
	});

	document.getElementById("endnav2").addEventListener("click", async () => {
    try {
        const res = await fetch("/end_nav");
        const text = await res.text();
        alert(text);   // ✅ แสดง popup เมื่อ end nav
    } catch (err) {
        alert("❌ Error ending navigation: " + err);
    }
	});
	// video.onload = function () {
	//     // joystick and keyboard controls will be available only when video is correctly loaded
	// };
}