// ===== WebSocket to rosbridge =====
var ros = new ROSLIB.Ros({
  url: "ws://" + window.location.hostname + ":9090"
});

const wsBadge = document.getElementById("wsStatus");
function setWs(status, cls) {
  wsBadge.textContent = status;
  wsBadge.className = "badge " + cls;
}

ros.on('connection', function() {
  console.log("Connected to ROS");
  setWs("CONNECTED", "badge-success");
});
ros.on('close', function() {
  console.warn("ROS connection closed");
  setWs("DISCONNECTED", "badge-secondary");
});
ros.on('error', function(err) {
  console.error("ROS error:", err);
  setWs("ERROR", "badge-danger");
});

// ===== Subscriber: /plc_status =====
var plcStatus = new ROSLIB.Topic({
  ros: ros,
  name: '/plc_status',
  messageType: 'facobot_msg/msg/PlcStatus'  // ROS2 ต้องมี /msg/
});

// ===== Publisher: /plc_command =====
// var plcCommand = new ROSLIB.Topic({
//   ros: ros,
//   name: '/plc_command',
//   messageType: 'std_msgs/msg/String'
// });

function setDOUT(pin, state) {
  const data = `DOUT:${pin}:${state}`;
  plcStatus.publish(new ROSLIB.Message({ data }));
  console.log("Send command:", data);
}

// ===== Render Helpers =====
function renderDIN(dinArray) {
  const body = document.querySelector("#dinTable tbody");
  body.innerHTML = "";
  dinArray.forEach((v, i) => {
    const on = !!v.data;
    const cls = on ? "table-success" : "table-danger";
    const txt = on ? "ON" : "OFF";
    body.innerHTML += `<tr>
      <td>DIN ${i+1}</td>
      <td class="${cls}">${txt}</td>
    </tr>`;
  });
}

function renderDOUT(doutArray) {
  const body = document.querySelector("#doutTable tbody");
  body.innerHTML = "";
  doutArray.forEach((v, i) => {
    // รองรับทั้ง std_msgs/Bool และ int[]
    const val = (v && typeof v === 'object' && 'data' in v) ? v.data : v;
    const on  = (val === true) || (Number(val) === 1);

    const statusClass = on ? "table-success" : "table-danger";
    const statusText  = on ? "ON" : "OFF";

    body.innerHTML += `<tr>
      <td>DOUT ${i+1}</td>
      <td class="${statusClass}">${statusText}</td>
      <td>
        <button class="btn btn-sm btn-success mr-1" ${on ? "disabled" : ""} onclick="setDOUT(${i+1}, 1)">ON</button>
        <button class="btn btn-sm btn-danger"  ${!on ? "disabled" : ""} onclick="setDOUT(${i+1}, 0)">OFF</button>
      </td>
    </tr>`;
  });
}

// ===== Subscribe callback =====
plcStatus.subscribe(function(msg) {
  // ใน ROS2 ชื่อฟิลด์คือ din_msg / dout_msg และเป็น array ของ std_msgs/Bool
  renderDIN(msg.din_msg || []);
  renderDOUT(msg.dout_msg || []);
});
