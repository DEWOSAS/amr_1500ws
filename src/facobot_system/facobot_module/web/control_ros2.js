// === ROS 2 VERSION (Updated for rosbridge with ROS 2) ===
// ใช้ร่วมกับ rosbridge_websocket ที่เชื่อม ROS 2 และ Web
// แก้ไขจาก ROS 1 → ROS 2: ใช้ topic /tcp_recv (std_msgs/String) และ cmd_vel (geometry_msgs/Twist)

var twist;
var cmdVel;
var publishImmidiately = true;
var manager;
var ros;
var scale = 0.5;
var lin = 0;
var ang = 0;

// === Connect to rosbridge_websocket ===
ros = new ROSLIB.Ros({
    url: 'ws://localhost:9090' // เปลี่ยนเป็น IP หุ่นยนต์ได้ เช่น 'ws://192.168.1.42:9090'
});

ros.on('connection', function() {
    console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
    console.log('Error connecting to websocket server: ', error);
});

ros.on('close', function() {
    console.log('Connection to websocket server closed.');
});

// === Initialize ROS String Publisher ===
function initWebInputPublisher() {
    msg = new ROSLIB.Message({
        data: '0'
    });
    web_topic = new ROSLIB.Topic({
        ros: ros,
        name: '/tcp_recv', // ใช้งานกับ ROS 2 ได้ถ้า rosbridge_websocket เปิดใช้งาน
        messageType: 'std_msgs/String'
    });
    web_topic.advertise();
}

// === Initialize Button Events ===
function initClick() {
    send1 = document.getElementById("click1");
    send2 = document.getElementById("click2");
    send3 = document.getElementById("click3");
    send4 = document.getElementById("click4");
    send1.onclick = function () {
        msg.data = "1";
        console.log("Task : 1");
        web_topic.publish(msg);
    }
    send2.onclick = function () {
        msg.data = "2";
        console.log("Task : 2");
        web_topic.publish(msg);
    }
    send3.onclick = function () {
        msg.data = "3";
        console.log("Task : 3");
        web_topic.publish(msg);
    }
    send4.onclick = function () {
        msg.data = "4";
        console.log("Task : 4");
        web_topic.publish(msg);
    }
}

// === Initialize Keyboard Control ===
function initKeyboardInput() {
    document.addEventListener('keydown', keyboardMoveDown, true);
    document.addEventListener('keyup', keyboardMoveUp, true);
}

// === ROS 2 Twist Publisher (geometry_msgs/Twist) ===
function moveAction(linear, angular) {
    twist = new ROSLIB.Message({
        linear: {
            x: linear,
            y: 0.0,
            z: 0.0
        },
        angular: {
            x: 0.0,
            y: 0.0,
            z: angular
        }
    });

    // ส่ง Twist ไป ROS 2
    cmdVel.publish(twist);
}

// === Setup Twist Topic ===
function initVelocityPublisher() {
    cmdVel = new ROSLIB.Topic({
        ros: ros,
        name: '/cmd_vel',
        messageType: 'geometry_msgs/Twist'
    });
    cmdVel.advertise();
}