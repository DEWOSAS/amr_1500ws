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
import tempfile
from sanic.response import file_stream
import re
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

class WebServer(Node):
    def __init__(self):
        super().__init__('web_server')
        self.get_logger().info('Node : web_server')
        self.init_param()
        self.publisher_ = self.create_publisher(String, "latest_map_path", 10)
        # self.start_navigation_node()
        # self.start_server()  # ลบส่วนนี้ออก เพราะเราแยก app ออกไปแล้ว

    def init_param(self):
        self.web_ip = self.declare_parameter('web_ip', '172.16.0.210').get_parameter_value().string_value
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

    def change_ip(self, ori_file, new_file):
        with open(ori_file, 'rt') as fin:
            data = fin.read().replace('localhost', self.web_ip)
        with open(new_file, 'wt') as fout:
            fout.write(data)
        self.get_logger().info(f"Updated IP to: {self.web_ip}")

    def start_mapping_node(self):
        if not self.mapping_on:
            launch_cmd = (
            'source /home/babartos/mir_ws/install/setup.bash && '
            'ros2 launch mir_navigation mapping.py use_sim_time:=true slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml'
            )
            self.mapping_proc = subprocess.Popen(
            ['bash', '-c', launch_cmd],
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


    # def start_navigation_node(self):
    #     if not self.navigation_on:
    #         self.nav_proc = subprocess.Popen(
    #             ['ros2', 'launch', 'facobot_core', 'facobot_navigation.launch.py']
    #         )
    #         self.navigation_on = True
    #         self.get_logger().info("Start navigation")

    # def end_navigation_node(self):
    #     if self.navigation_on:
    #         self.nav_proc.terminate()
    #         self.nav_proc.wait()
    #         self.navigation_on = False
    #         self.get_logger().info("End navigation")


# สร้าง Sanic app ที่ระดับ global เพื่อให้ Sanic multi worker มองเห็น
def run_sanic():
    from sanic import Sanic, response
    from sanic.exceptions import NotFound

    app = Sanic("WebServer")

    @app.route("/")
    async def home(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/index.html'))

    @app.route("/mapping")
    async def mapping(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/mapping.html'))

    @app.route("/control")
    async def control(request):
        return await response.file_stream(os.path.join(node.facobot_module_path, 'web/control.html'))

    @app.route("/save")
    async def save(request):
        raw_name = request.args.get("name", "map")
        map_name = raw_name.lower()

        # ✅ ตรวจสอบชื่อว่าไม่มี special characters
        if not re.match(r'^[a-zA-Z0-9_\-]+$', map_name):
            return response.text("❌ ชื่อแผนที่ต้องเป็น a-z, A-Z, 0-9, _ หรือ - เท่านั้น", status=400)

        if node.mapping_on:
            map_dir = "/home/babartos/amr_1500ws/src/map_before_upload"
            os.makedirs(map_dir, exist_ok=True)

            yaml_path = os.path.join(map_dir, f"{map_name}.yaml")
            pgm_path = os.path.join(map_dir, f"{map_name}.pgm")

            # ✅ ตรวจสอบชื่อซ้ำ
            if os.path.exists(yaml_path) or os.path.exists(pgm_path):
                return response.text(f"❌ ชื่อ '{map_name}' ถูกใช้ไปแล้ว กรุณาใช้ชื่ออื่น", status=400)

            # ✅ บันทึก map
            result = subprocess.run([
                'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                '-f', os.path.join(map_dir, map_name)])

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
    

    UPLOAD_ROOT = "/home/babartos/amr_1500ws/src/uploand_image"  # โฟลเดอร์หลักที่ใช้เก็บทุกแผนที่
    @app.post("/upload_map")
    async def upload_map(request: Request):
        pgm_file = request.files.get("pgm")
        yaml_file = request.files.get("yaml")

        if not pgm_file or not yaml_file:
            return response.text("Missing pgm or yaml", status=400)

        # ดึงชื่อไฟล์โดยไม่เอานามสกุล (เพื่อใช้เป็นชื่อโฟลเดอร์)
        map_name = os.path.splitext(pgm_file.name)[0]

        # สร้างโฟลเดอร์ใหม่ตามชื่อไฟล์ (เช่น "090")
        save_dir = os.path.join(UPLOAD_ROOT, map_name)
        os.makedirs(save_dir, exist_ok=True)

        # เซฟ pgm
        pgm_path = os.path.join(save_dir, pgm_file.name)
        with open(pgm_path, "wb") as f:
            f.write(pgm_file.body)

        # เซฟ yaml
        yaml_path = os.path.join(save_dir, yaml_file.name)
        with open(yaml_path, "wb") as f:
            f.write(yaml_file.body)

        return response.text(f"แผนที่ถูกอัปโหลดไปยัง: {save_dir}")

    app.run(host="0.0.0.0", port=8000, single_process=True)

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
