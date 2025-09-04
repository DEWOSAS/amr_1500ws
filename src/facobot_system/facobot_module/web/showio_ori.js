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
// ===== Subscriber / battery =====
var battery = new ROSLIB.Topic({
  ros: ros,
  name: '/battery',
  messageType: 'facobot_msg/msg/BatteryStatus'  // ROS2 ต้องมี /msg/
});

// ===== Subscriber: /plc_status =====
var plcStatus = new ROSLIB.Topic({
  ros: ros,
  name: '/plc_status',
  messageType: 'facobot_msg/msg/PlcStatus'  // ROS2 ต้องมี /msg/
});

// ===== Publisher: /plc_command ===== ✅ เปิดใช้งานจริง
const plcCommand = new ROSLIB.Topic({
  ros: ros,
  name: '/plc_command',
  messageType: 'std_msgs/msg/String'
});
plcCommand.advertise(); 

// ===== ส่งคำสั่ง DOUT ไปที่ /plc_command =====
function setDOUT(pin, state) {
  // ✅ ถามหารหัสก่อน
  const PLC_PASSWORD = "1234"; //<<--------------------------------------------------------------- set password ตรงนี้
  const pass = prompt("กรุณาใส่รหัสเพื่อยืนยันการสั่งงาน:");

  // ตรวจสอบรหัส (เช่น "1234")
  if (pass !== PLC_PASSWORD) {
    alert("❌ รหัสไม่ถูกต้อง การสั่งงานถูกยกเลิก");
    return; // ไม่ส่งคำสั่ง
  }

  const cmd = `DOUT:${pin}:${state}`;
  plcCommand.publish(new ROSLIB.Message({ data: cmd }));
  console.log("Send command:", cmd);

  // อัปเดทปุ่มทันที (optional)
  const row = document.querySelector(`#doutTable tbody tr[data-index="${pin-1}"]`);
  if (row) {
    const btnOn  = row.querySelector(".btn-success");
    const btnOff = row.querySelector(".btn-danger");
    btnOn.disabled  = (state === 1);
    btnOff.disabled = (state === 0);
  }
}

// ===== Render Helpers =====
function renderDIN(dinArray) {
  const tbody = document.querySelector("#dinTable tbody");

  dinArray.forEach((v, i) => {
    const on  = !!v.data;
    const cls = on ? "table-success" : "table-danger";
    const txt = on ? "ON" : "OFF";

    let row = tbody.querySelector(`tr[data-index="${i}"]`);
    if (!row) {
      row = document.createElement("tr");
      row.setAttribute("data-index", i);
      row.innerHTML = `
        <td><div class="io-cell">DIN ${i+1}</div></td>
        <td><div class="io-cell"></div></td>
      `;
      tbody.appendChild(row);
    }

    const statusCell = row.children[1].firstElementChild;
    statusCell.className = `io-cell ${cls}`;
    statusCell.textContent = txt;
  });
}

function renderDOUT(doutArray) {
  const tbody = document.querySelector("#doutTable tbody");

  doutArray.forEach((v, i) => {
    const val = (v && typeof v === "object" && "data" in v) ? v.data : v;
    const on  = (val === true) || (Number(val) === 1);

    const statusClass = on ? "table-success" : "table-danger";
    const statusText  = on ? "ON" : "OFF";

    let row = tbody.querySelector(`tr[data-index="${i}"]`);
    if (!row) {
      row = document.createElement("tr");
      row.setAttribute("data-index", i);
      row.innerHTML = `
        <td><div class="io-cell">DOUT ${i+1}</div></td>
        <td><div class="io-cell"></div></td>
        <td>
          <div class="io-cell io-actions">
            <button class="btn btn-sm btn-success btn-io">ON</button>
            <button class="btn btn-sm btn-danger  btn-io">OFF</button>
          </div>
        </td>
      `;
      tbody.appendChild(row);

      // bind onclick แค่ครั้งเดียว
      const btnOn  = row.querySelector(".btn-success");
      const btnOff = row.querySelector(".btn-danger");
      btnOn.onclick  = () => setDOUT(i+1, 1);
      btnOff.onclick = () => setDOUT(i+1, 0);
    }

    const statusCell = row.children[1].firstElementChild;
    statusCell.className = `io-cell ${statusClass}`;
    statusCell.textContent = statusText;

    const btnOn  = row.querySelector(".btn-success");
    const btnOff = row.querySelector(".btn-danger");
    btnOn.disabled  = on;
    btnOff.disabled = !on;
  });
}
//<------------ process สำหรับ ทำขนาดของ colum table เท่ากับจำนวนข้อมูลในตาราง มีหน่วยเป็น px--------------------->
// ===== Sync row heights =====
// function syncRowHeights() {
//   const dinRows  = document.querySelectorAll('#dinTable tbody tr');
//   const doutRows = document.querySelectorAll('#doutTable tbody tr');
//   const n = Math.max(dinRows.length, doutRows.length);

//   for (let i = 0; i < n; i++) {
//     const dinH  = dinRows[i]?.offsetHeight  || 0;
//     const doutH = doutRows[i]?.offsetHeight || 0;
//     const h = Math.max(dinH, doutH);
//     if (dinRows[i])  dinRows[i].style.height  = h + 'px';
//     if (doutRows[i]) doutRows[i].style.height = h + 'px';
//   }
//}
// window.addEventListener('resize', syncRowHeights);

battery.subscribe(function(msg) {
  const level   = msg.battery_level?.data ?? 0;     // battery_level (Float32)
  //const current = msg.battery_current?.data ?? 0.0; // battery_current (Float32)

  const batteryDiv = document.getElementById("batteryStatus");
  const batteryIcon = document.getElementById("batteryIcon");
  const batteryText = document.getElementById("batteryText");

  // อัปเดตข้อความ %
  batteryText.textContent = `${level.toFixed(0)}%`;
  //batteryAmp.textContent  = `${current.toFixed(1)} A`;
  

  // กำหนดระดับ icon ตาม %
  let iconClass = "fa-battery-empty";
  if (level > 80) iconClass = "fa-battery-full";
  else if (level > 60) iconClass = "fa-battery-three-quarters";
  else if (level > 40) iconClass = "fa-battery-half";
  else if (level > 20) iconClass = "fa-battery-quarter";
  else iconClass = "fa-battery-empty";

  // เปลี่ยน icon class
  batteryIcon.className = `fas ${iconClass}`;

  // เปลี่ยนสีพื้นหลังตามระดับ (ใช้ Bootstrap alerts)
  if (level <= 30) {
    batteryDiv.className = "bg-danger text-white text-center mb-0 sticky-top";
  } else if (level <= 60) {
    batteryDiv.className = "bg-warning text-dark text-center mb-0 sticky-top";
  } else {
    batteryDiv.className = "bg-success text-white text-center mb-0 sticky-top";
  }
});


// ===== Subscribe callback =====
plcStatus.subscribe(function(msg) {
  const din  = msg.din  ?? msg.din_msg  ?? [];
  const dout = msg.dout ?? msg.dout_msg ?? [];
  renderDIN(din);
  renderDOUT(dout);
  // requestAnimationFrame(syncRowHeights);
});
