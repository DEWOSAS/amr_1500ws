#!/usr/bin/env python3

import os
import rclpy
import signal
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition
import multiprocessing
from rclpy.node import Node
from sanic import Sanic, response
from sanic.exceptions import NotFound
import subprocess
from sanic.request import Request
from sanic.response import file_stream
import re
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
import io, zipfile, asyncio, os, tempfile, re, subprocess
from multiprocessing import Manager   # NEW
import shutil

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        self.get_logger().info('Node : web_server')
        self.init_param()
        self.publisher_ = self.create_publisher(String, "latest_map_path", 10)
        self.latest_map_path = None
        self.latest_map_yaml = None
        self.mapserver_on = False      # ✅ สถานะ map_server
        self.mapserver_proc = None  
        self.amcl_proc = None
        # self.start_navigation_node()
        # self.start_server()  # ลบส่วนนี้ออก เพราะเราแยก app ออกไปแล้ว

    def init_param(self):
        self.web_ip = self.declare_parameter('web_ip', '172.16.0.210')
        self.UPLOAD_ROOT = self.declare_parameter('UPLOAD_ROOT', '/home/babartos/mir_ws/src/facobot_system/facobot_module/upload')
        self.map_dir = self.declare_parameter('map_dir', '/home/babartos/amr_1500ws/src/map_before_upload')
        self.launch_cmd_mapping = self.declare_parameter('launch_cmd_mapping', 'ros2 launch map_to_jpeg map_to_image.launch.py')
        self.launch_cmd_nav2 = self.declare_parameter('launch_cmd_nav2', 'ros2 launch facobot_bringup amr_simbringup.launch.py')
        self.declare_parameter('launch_cmd', '')

        self.map_dir = self.get_parameter("map_dir").get_parameter_value().string_value
        self.web_ip = self.get_parameter("web_ip").get_parameter_value().string_value
        self.UPLOAD_ROOT = self.get_parameter("UPLOAD_ROOT").get_parameter_value().string_value
        self.launch_cmd_mapping = self.get_parameter("launch_cmd_mapping").get_parameter_value().string_value
        self.launch_cmd_nav2 = self.get_parameter("launch_cmd_nav2").get_parameter_value().string_value

        self.navigation_on = False
        self.mapping_on = False

        self.facobot_module_path = get_package_share_directory('facobot_module')
        # self.facobot_core_path = get_package_share_directory('facobot_core')

        self.change_ip(
            os.path.join(self.facobot_module_path, 'web/control_ros2_ori.js'),
            os.path.join(self.facobot_module_path, 'web/control.js')
        )
        self.change_ip(
            os.path.join(self.facobot_module_path, 'web/mapping_ros2_ori.js'),
            os.path.join(self.facobot_module_path, 'web/mapping.js')
        )
        self.change_ip(
            os.path.join(self.facobot_module_path, 'web/showio_ori.js'),
            os.path.join(self.facobot_module_path, 'web/showio.js')
        )

    def change_ip(self, ori_file, new_file):
        with open(ori_file, 'rt') as fin:
            data = fin.read().replace('localhost', self.web_ip)
        with open(new_file, 'wt') as fout:
            fout.write(data)
        self.get_logger().info(f"Updated IP to: {self.web_ip}")

    def start_mapping_node(self):
        if not self.mapping_on:

            self.mapping_proc = subprocess.Popen(
            ['bash', '-c', self.launch_cmd_mapping],
            preexec_fn=os.setsid  # << สำคัญ
            )
            self.get_logger().info("Start mapping")
            self.mapping_on = True

    def end_mapping_node(self):
        if self.mapping_on:
            self.get_logger().info("Sending SIGTERM to mapping process...")
            os.killpg(os.getpgid(self.mapping_proc.pid), signal.SIGTERM)
            try:
                self.mapping_proc.wait(timeout=5)  # รอให้จบภายใน 5 วินาที
                self.get_logger().info("Mapping process exited cleanly.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Mapping pmmm=rocess did not exit in time. Force killing...")
                os.killpg(os.getpgid(self.mapping_proc.pid), signal.SIGKILL)
                self.mapping_proc.wait()
                self.get_logger().info("Mapping process force killed.")

            self.mapping_on = False


    def start_navigation_node(self):
        if not self.navigation_on:
            self.nav_proc = subprocess.Popen(
                ["bash","-lc",
                "source /opt/ros/humble/setup.bash && "
                "source ~/amr_1500ws/install/setup.bash && "
                + self.launch_cmd_nav2],
                preexec_fn=os.setsid
            )
            self.navigation_on = True
            self.get_logger().info("Start navigation")
            return True, "nav2 started"
        return True, "nav2 already running"


    def end_navigation_node(self):
        if self.navigation_on and self.nav_proc:
            try:
                os.killpg(os.getpgid(self.nav_proc.pid), signal.SIGTERM)
                self.nav_proc.wait(timeout=5)
                msg = "Nav2 stopped"
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.nav_proc.pid), signal.SIGKILL)
                self.nav_proc.wait()
                msg = "Nav2 force killed"
            self.navigation_on = False
            self.nav_proc = None
            self.get_logger().info(msg)

    def start_map_server(self):
        if self.mapserver_proc and self.mapserver_proc.poll() is None:
            return True, "ℹ️ map_server already running"
        if not self.latest_map_yaml or not os.path.exists(self.latest_map_yaml):
            self.get_logger().error("No latest_map_yaml set or file missing.")
            return False, "❌ ยังไม่มีไฟล์ YAML ล่าสุด หรือไฟล์ไม่พบ"

        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source ~/amr_1500ws/install/setup.bash && "
            f"ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={self.latest_map_yaml}"
        )
        self.get_logger().info(f"Starting map_server with: {cmd}")
        self.mapserver_proc = subprocess.Popen(["bash", "-lc", cmd], preexec_fn=os.setsid)
        self.mapserver_on = True
        return True, f"✅ map_server started with {self.latest_map_yaml}"

    
    def end_map_server(self):
        if self.mapserver_on and self.mapserver_proc:
            try:
                os.killpg(os.getpgid(self.mapserver_proc.pid), signal.SIGTERM)
                self.mapserver_proc.wait(timeout=5)
                msg = "🛑 map_server stopped"
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.mapserver_proc.pid), signal.SIGKILL)
                self.mapserver_proc.wait()
                msg = "🛑 map_server force killed"
            self.mapserver_on = False
            self.mapserver_proc = None
            self.get_logger().info(msg)
            return True, msg
        return False, "ℹ️ map_server ยังไม่ได้รัน"
    
    def start_amcl(self):
        if self.amcl_proc and self.amcl_proc.poll() is None:
            return False, "AMCL already running"
        try:
            self.get_logger().info("▶️ Starting AMCL...")
            self.amcl_proc = subprocess.Popen(
                ["bash", "-lc",
                "source /opt/ros/humble/setup.bash && "
                "source ~/amr_1500ws/install/setup.bash && "
                "ros2 launch amr_nav amcl_localization.launch.py"],
                preexec_fn=os.setsid
            )
            return True, f"AMCL started pid={self.amcl_proc.pid}"
        except Exception as e:
            self.amcl_proc = None
            return False, f"AMCL failed: {e}"

    def end_amcl(self):
        if not self.amcl_proc or self.amcl_proc.poll() is not None:
            self.amcl_proc = None
            return True, "AMCL not running"
        try:
            self.get_logger().info("⏹ Stopping AMCL...")
            os.killpg(os.getpgid(self.amcl_proc.pid), signal.SIGTERM)
            self.amcl_proc.wait(timeout=5)
            msg = "AMCL stopped"
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.amcl_proc.pid), signal.SIGKILL)
            self.amcl_proc.wait()
            msg = "AMCL force killed"
        self.amcl_proc = None
        return True, msg
    
# สร้าง Sanic app ที่ระดับ global เพื่อให้ Sanic multi worker มองเห็น
def run_sanic():
    from sanic import Sanic, response
    from sanic.exceptions import NotFound
    
    app = Sanic("WebServer")
    app.config.FALLBACK_ERROR_FORMAT = "text"
    app.config.DEBUG = True

    
    @app.route("/start_nav")
    async def start_nav(request):
        try:
            msgs = []
            ok_all = True

            ok, msg = node.start_map_server()
            msgs.append(f"map_server: {msg}")
            ok_all = ok_all and ok

            if ok:
                ok2, msg2 = node.start_navigation_node()
                msgs.append(f"nav2: {msg2}")
                ok_all = ok_all and ok2

                ok3, msg3 = node.start_amcl()
                msgs.append(f"amcl: {msg3}")
                ok_all = ok_all and ok3
            else:
                msgs.append("skip nav2/amcl because map_server failed")

            status = 200 if ok_all else 400
            return response.text("✅ Navigation started\n" + "\n".join(msgs), status=status)
        except Exception as e:
            import traceback; traceback.print_exc()
            return response.text(f"💥 500 start_nav crashed: {e}", status=500)

    @app.route("/end_nav")
    async def end_nav(request):
        try:
            msgs = []
            ok_all = True

            ok3, msg3 = node.end_amcl()
            msgs.append(f"amcl: {msg3}")
            ok_all = ok_all and ok3

            node.end_navigation_node()
            msgs.append("nav2: stopped")

            ok, msg = node.end_map_server()
            msgs.append(f"map_server: {msg}")
            ok_all = ok_all and ok

            status = 200 if ok_all else 400
            return response.text("🛑 Navigation ended\n" + "\n".join(msgs), status=status)
        except Exception as e:
            import traceback; traceback.print_exc()
            return response.text(f"💥 500 end_nav crashed: {e}", status=500)

    @app.route("/")
    async def home(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/index.html'))

    @app.route("/mapping")
    async def mapping(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/mapping.html'))

    @app.route("/control")
    async def control(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/control.html'))
    
    @app.route("/showio")
    async def showio(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, "web/showio.html"))
    
    @app.route("/save")
    async def save(request):
        raw_name = request.args.get("name", "map")
        map_name = raw_name.lower()

        # ✅ ตรวจสอบชื่อว่าไม่มี special characters
        if not re.match(r'^[a-zA-Z0-9_\-]+$', map_name):
            return response.text("❌ ชื่อแผนที่ต้องเป็น a-z, A-Z, 0-9, _ หรือ - เท่านั้น", status=400)

        if node.mapping_on:
            # map_dir = "/home/babartos/amr_1500ws/src/map_before_upload"
            os.makedirs(node.map_dir, exist_ok=True)

            yaml_path = os.path.join(node.map_dir, f"{map_name}.yaml")
            pgm_path = os.path.join(node.map_dir, f"{map_name}.pgm")


            # ✅ บันทึก map
            result = subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', os.path.join(node.map_dir, map_name)])

            if result.returncode == 0:
                # ✅ สร้างไฟล์ zip ชั่วคราว
                temp_dir = tempfile.mkdtemp()
                zip_path = os.path.join(temp_dir, f"{map_name}.zip")

                # ✅ สร้าง zip จากไฟล์ yaml และ pgm
                import zipfile
                with zipfile.ZipFile(zip_path, 'w') as zipf:
                    zipf.write(yaml_path, arcname=f"{map_name}.yaml")
                    zipf.write(pgm_path, arcname=f"{map_name}.pgm")

                # ✅ ส่ง zip กลับให้ client
                return await file_stream(
                    zip_path,
                    filename=f"{map_name}.zip",
                    mime_type='application/zip',
                    headers={"Content-Disposition": f"attachment; filename={map_name}.zip"})
            else:
                return response.text("บันทึกล้มเหลว", status=500)
        else:
            return response.text("ยังไม่ได้เริ่ม mapping", status=400)

    

    @app.route("/start")
    async def start_mapping(request):
        node.start_mapping_node()
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/mapping.html'))

    @app.route("/end")
    async def end_mapping(request):
        node.end_mapping_node()
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/mapping.html'))

    @app.exception(NotFound)
    async def handle_404(request, exception):
        path = os.path.join(node.facobot_module_path, 'web' + request.path)
        if os.path.exists(path):
            return await response.file_stream(path)
        return response.text(f"{request.path[1:]} : path not found")
    

    # UPLOAD_ROOT = "/home/babartos/amr_1500ws/src/uploand_image"  # โฟลเดอร์หลักที่ใช้เก็บทุกแผนที่
    @app.post("/upload_map")
    async def upload_map(request: Request):
        pgm_file = request.files.get("pgm")
        yaml_file = request.files.get("yaml")

        if not pgm_file or not yaml_file:
            return response.text("Missing pgm or yaml", status=400)

        map_name = os.path.splitext(pgm_file.name)[0]

        save_dir = os.path.join(node.UPLOAD_ROOT, map_name)
        os.makedirs(save_dir, exist_ok=True)

        # เซฟ PGM
        pgm_path = os.path.join(save_dir, pgm_file.name)
        with open(pgm_path, "wb") as f:
            f.write(pgm_file.body)

        # เซฟ YAML
        yaml_path = os.path.join(save_dir, yaml_file.name)
        with open(yaml_path, "wb") as f:
            f.write(yaml_file.body)

        # ✅ จำ path ล่าสุดไว้ใน node (ทั้งไดเร็กทอรีและไฟล์ yaml ชัดเจน)
        node.latest_map_path = save_dir
        node.latest_map_yaml = yaml_path

        # (ออปชัน) publish ไป topic เดิม
        msg = String()
        msg.data = save_dir
        node.publisher_.publish(msg)
        node.get_logger().info(f"Published latest_map_path: {save_dir}")
        node.get_logger().info(f"Latest YAML: {yaml_path}")

        # ✅ เปลี่ยนจาก text -> json เพื่อให้ฝั่งหน้าเว็บ parse ได้ง่าย
        return response.json({
            "ok": True,
            "map_dir": save_dir,
            "yaml": yaml_path,
            "pgm": pgm_path
        })

    app.run(host="0.0.0.0", port=8000, single_process=True, access_log=True)

    # current_state = None  # เก็บสถานะไว้ (อาจต้อง sync ให้ thread-safe)
    # @app.route("/toggle-slam")
    # async def toggle_slam(request):
    #     global current_state

    #     rclpy.init()
    #     node = rclpy.create_node('toggle_slam_client')

    #     # สร้าง client service เพื่อดึงสถานะปัจจุบัน
    #     get_state_cli = node.create_client(GetState, '/slam_toolbox/get_state')
    #     if not get_state_cli.wait_for_service(timeout_sec=2.0):
    #         node.destroy_node()
    #         return response.text("ไม่พบ service get_state", status=500)

    #     get_state_req = GetState.Request()
    #     future = get_state_cli.call_async(get_state_req)
    #     rclpy.spin_until_future_complete(node, future)
    #     state = future.result().current_state.id if future.result() else None

    #     # สลับสถานะ
    #     change_cli = node.create_client(ChangeState, '/slam_toolbox/change_state')
    #     if not change_cli.wait_for_service(timeout_sec=2.0):
    #         node.destroy_node()
    #         return response.text("ไม่พบ service change_state", status=500)

    #     req = ChangeState.Request()
    #     if state == 4:  # Deactivated (Paused)
    #         req.transition.id = Transition.TRANSITION_RESUME  # 11
    #         action = "resume"
    #     else:
    #         req.transition.id = Transition.TRANSITION_PAUSE   # 10
    #         action = "pause"

    #     future = change_cli.call_async(req)
    #     rclpy.spin_until_future_complete(node, future)

    #     if future.result().success:
    #         node.destroy_node()
    #         return response.text(f"✅ {action} slam_toolbox สำเร็จ")
    #     else:
    #         node.destroy_node()
    #         return response.text(f"❌ {action} ล้มเหลว", status=500)
    
def main(args=None):
    global node
    rclpy.init(args=args)
    node = WebServer()
    # run sanic server แบบ single process เพื่อไม่ให้เกิด error signal
    # ที่มาจาก multi worker mode

    # facobot_module_path = get_package_share_directory('facobot_module')
    # static_path = os.path.join(facobot_module_path, 'web')
    # app.static('/', static_path)
    # Start Sanic server in background thread
  
    # Start Sanic in a separate process (not thread)
    sanic_process = multiprocessing.Process(target=run_sanic, daemon=True)
    sanic_process.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down Sanic process...")
        sanic_process.terminate()
        sanic_process.join()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
