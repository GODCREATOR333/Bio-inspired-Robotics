import sys
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLTextItem
from PyQt5 import QtGui, QtCore, QtWidgets

from viewer import MyView
import geometry
from agent import Agent_Model
from geometry import create_circle,create_sun
from environment import Food_Model,Environment
from utils import random_point_outside_radius,TrailManager
from config import AgentConfig,CRWConfig,HomingConfig,EnvironmentConfig


from fsm_controller import FSMController, AgentState
from correlated_random_walk import CorrelatedRandomWalk
from homing_policy import VectorHoming


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bio-Inspired Robotics")
        screen = QtWidgets.QApplication.primaryScreen().availableGeometry()
        self.resize(
            min(1200, screen.width()),
            min(800, screen.height())
        )


        # --Load Configs--
        self.agent_cfg = AgentConfig()
        self.crw_cfg = CRWConfig()
        self.homing_cfg = HomingConfig()
        self.env_cfg=EnvironmentConfig()
        self.environment = Environment(self.env_cfg)
        self.home_circle_item = None
        self.food_mesh_items = []
        self.food_circle_items = []
        self.scan_markers = []


        # --- State Flags ---
        self.results_logged = False
        self.step_count = 0 



        # --- Trails and agent ---
        self.trails = TrailManager()
        self.agent = Agent_Model("CAD_Model_ant/ant_model.stl",agent_cfg=self.agent_cfg, scale=0.01)
        self.objects = {}


        # Check Simulation status
        self.simulation_active = False


        # --- Navigation policies ---
        self.search_policy = CorrelatedRandomWalk(
            step_length=self.agent.agent_speed,
            turn_std=self.crw_cfg.turn_std 
        )

        self.homing_policy = VectorHoming(
            step_length=self.agent.agent_speed,
            home_threshold=self.homing_cfg.home_threshold
        )

        # --- FSM ---
        self.fsm = FSMController(
            agent=self.agent,
            search_policy=self.search_policy,
            homing_policy=self.homing_policy,
            logger=self.log,
        )


        # ============================
        # OUTER SPLITTER
        # ============================
        self.outer_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.outer_splitter.setHandleWidth(6)

        # ============================
        # SIDEBAR
        # ============================
        self.sidebar = QtWidgets.QWidget()
        sidebar_layout = QtWidgets.QVBoxLayout(self.sidebar)
        sidebar_layout.setContentsMargins(8, 8, 8, 8)
        sidebar_layout.setSpacing(8)

        # ---------- Instructions ----------
        self.instructions = QtWidgets.QTextEdit()
        self.instructions.setReadOnly(True)
        self.instructions.setMaximumHeight(180)
        self.instructions.setPlainText(
            "=== CONTROLS ===\n"
            "   [W / A / S / D]  : Pan Camera (X/Y Plane)\n"
            "   [Left Mouse]     : Orbit / Rotate View\n"
            "   [Scroll / I / O] : Zoom In / Out\n"
            "   [R]              : RESET Simulation & View\n\n"

            "=== VISUAL LEGEND ===\n"
            "   Yellow Line    : Ground Truth Path\n"
            "   Purple Line  : Estimated Path (Odometry error)\n"
            "   Yellow Dots  : Sun Compass Correction Events\n"
            "   Blue Circles : Food Detection Range\n"
            "   Red Circle   : Ant Detection Range\n"
            "   Brown Circle   : Home Detection Range\n\n"

            "=== EXPERIMENT MODES ===\n"
            "1. Blind Navigation (Est Pos):\n"
            "   - Agent relies purely on noisy odometry.\n"
            "   - EXPECTATION: Drift accumulates, Agent misses Home.\n"
            "2. Control Group (True Pos):\n"
            "   - Agent uses 'God Mode' (True Coordinates).\n"
            "   - EXPECTATION: Perfect straight-line homing.\n"
            "3. Sun Compass (Corrected):\n"
            "   - Agent performs 'Stop-and-Scan' behavior.\n"
            "   - EXPECTATION: Heading drift is bounded; Home is reached.\n\n"

            "=== PARAMETER GUIDE ===\n"
            "Heading Bias Mean : Systematic compass offset (curved path)\n"
            "Heading Bias Std  : Run-to-run bias variability\n"
            "Heading Noise     : Random angular jitter (wobble)\n"
            "Stride Noise      : Distance estimation error\n"
            "Speed             : Forward step length per update\n"
            "Turn Std          : Random turn variability (CRW strength)\n"
            "Scan Interval     : Distance (mm) between sun compass corrections\n\n"

            "=== HOMING / ENVIRONMENT ===\n"
            "Home Threshold    : Distance to nest for successful homing\n"
            "Food Detect R     : Radius to detect food items\n"
            "Home Detect R     : Radius to detect nest\n"
            "Spawn Min R       : Minimum food spawn distance\n"
            "Spawn Max R       : Maximum food spawn distance\n\n"
        )

        sidebar_layout.addWidget(self.instructions)

        # ---------- Experiment Mode ----------
        exp_group = QtWidgets.QGroupBox("Experiment Mode")
        exp_layout = QtWidgets.QVBoxLayout(exp_group)

        self.mode_selector = QtWidgets.QComboBox()
        self.mode_selector.addItems([
            "Blind (Estimated)",
            "Control (True)",
            "Sun Compass"
        ])
        exp_layout.addWidget(self.mode_selector)
        sidebar_layout.addWidget(exp_group)

        # ---------- Log ----------
        log_group = QtWidgets.QGroupBox("Simulation Log")
        log_layout = QtWidgets.QVBoxLayout(log_group)

        self.console = QtWidgets.QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet(
            "background:#111; color:#0f0; font-family:monospace; font-size:10pt;"
        )
        self.console.setPlainText("--- SYSTEM READY ---")
        log_layout.addWidget(self.console)

        sidebar_layout.addWidget(log_group, stretch=1)

        # ============================
        # PARAMETER SCROLL AREA
        # ============================
        param_scroll = QtWidgets.QScrollArea()
        param_scroll.setWidgetResizable(True)
        param_scroll.setFixedHeight(360)
        param_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)

        param_container = QtWidgets.QWidget()
        param_layout = QtWidgets.QVBoxLayout(param_container)
        param_layout.setContentsMargins(6, 6, 6, 6)
        param_layout.setSpacing(10)

        # ---------- Agent Parameters ----------
        agent_group = QtWidgets.QGroupBox("Agent Parameters")
        agent_form = QtWidgets.QFormLayout(agent_group)

        self.bias_mean_input = QtWidgets.QDoubleSpinBox()
        self.bias_mean_input.setRange(-5, 5)
        self.bias_mean_input.setValue(self.agent_cfg.heading_bias_mean)

        self.bias_std_input = QtWidgets.QDoubleSpinBox()
        self.bias_std_input.setRange(0, 5)
        self.bias_std_input.setValue(self.agent_cfg.heading_bias_std)

        self.heading_noise_input = QtWidgets.QDoubleSpinBox()
        self.heading_noise_input.setRange(0, 5)
        self.heading_noise_input.setValue(self.agent_cfg.heading_noise_std)

        self.stride_noise_input = QtWidgets.QDoubleSpinBox()
        self.stride_noise_input.setRange(0, 5)
        self.stride_noise_input.setValue(self.agent_cfg.stride_noise_std)

        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(int(self.agent_cfg.agent_speed))

        self.speed_label = QtWidgets.QLabel(f"{self.agent_cfg.agent_speed:.1f}")
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f"{v:.1f}")
        )

        speed_box = QtWidgets.QHBoxLayout()
        speed_box.addWidget(self.speed_slider)
        speed_box.addWidget(self.speed_label)

        self.turn_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.turn_slider.setRange(0, 90)
        self.turn_slider.setValue(int(np.rad2deg(self.crw_cfg.turn_std)))

        self.turn_label = QtWidgets.QLabel(
            f"{np.rad2deg(self.crw_cfg.turn_std):.1f}°"
        )
        self.turn_slider.valueChanged.connect(
            lambda v: self.turn_label.setText(f"{v:.1f}°")
        )

        turn_box = QtWidgets.QHBoxLayout()
        turn_box.addWidget(self.turn_slider)
        turn_box.addWidget(self.turn_label)

        self.scan_interval_input = QtWidgets.QDoubleSpinBox()
        self.scan_interval_input.setRange(10, 2000)
        self.scan_interval_input.setValue(self.agent_cfg.scan_interval)

        self.home_thresh_input = QtWidgets.QDoubleSpinBox()
        self.home_thresh_input.setRange(1, 50)
        self.home_thresh_input.setValue(self.homing_cfg.home_threshold)

        agent_form.addRow("Bias mean", self.bias_mean_input)
        agent_form.addRow("Bias std", self.bias_std_input)
        agent_form.addRow("Heading noise", self.heading_noise_input)
        agent_form.addRow("Stride noise", self.stride_noise_input)
        agent_form.addRow("Speed", speed_box)
        agent_form.addRow("Turn std", turn_box)
        agent_form.addRow("Scan interval", self.scan_interval_input)
        agent_form.addRow("Home threshold", self.home_thresh_input)

        param_layout.addWidget(agent_group)

        # ---------- Environment Parameters ----------
        env_group = QtWidgets.QGroupBox("Environment Parameters")
        env_form = QtWidgets.QFormLayout(env_group)

        self.n_food_input = QtWidgets.QSpinBox()
        self.n_food_input.setRange(1, 100)
        self.n_food_input.setValue(self.env_cfg.n_food_items)

        self.food_det_input = QtWidgets.QDoubleSpinBox()
        self.food_det_input.setRange(1, 100)
        self.food_det_input.setValue(self.env_cfg.food_detection_radius)

        self.home_det_input = QtWidgets.QDoubleSpinBox()
        self.home_det_input.setRange(1, 100)
        self.home_det_input.setValue(self.env_cfg.home_detection_radius)

        self.spawn_min_input = QtWidgets.QDoubleSpinBox()
        self.spawn_min_input.setRange(10, 1000)
        self.spawn_min_input.setValue(self.env_cfg.food_spawn_min_radius)

        self.spawn_max_input = QtWidgets.QDoubleSpinBox()
        self.spawn_max_input.setRange(10, 2000)
        self.spawn_max_input.setValue(self.env_cfg.food_spawn_max_radius)

        env_form.addRow("Food items", self.n_food_input)
        env_form.addRow("Food detect r", self.food_det_input)
        env_form.addRow("Home detect r", self.home_det_input)
        env_form.addRow("Spawn min r", self.spawn_min_input)
        env_form.addRow("Spawn max r", self.spawn_max_input)

        param_layout.addWidget(env_group)
        param_layout.addStretch(1)

        param_scroll.setWidget(param_container)
        sidebar_layout.addWidget(param_scroll)

        # ---------- Control Buttons ----------
        controls = QtWidgets.QHBoxLayout()

        self.start_button = QtWidgets.QPushButton("Start")
        self.pause_button = QtWidgets.QPushButton("Pause")
        self.stop_button = QtWidgets.QPushButton("Stop")

        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        controls.addWidget(self.start_button)
        controls.addWidget(self.pause_button)
        controls.addWidget(self.stop_button)

        self.start_button.clicked.connect(self.start_search)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.stop_button.clicked.connect(self.stop_search)

        sidebar_layout.addLayout(controls)

        # Add sidebar
        self.outer_splitter.addWidget(self.sidebar)
        self.sidebar.setMinimumWidth(260)

        # ============================
        # RIGHT SIDE (VIEW + PLOTS)
        # ============================
        self.right_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)

        self.view = MyView()
        self.view.main = self
        self.right_splitter.addWidget(self.view)

        bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)

        self.xy_plot = pg.PlotWidget()
        self.xy_plot.showGrid(x=True, y=True)
        self.xy_plot.setLabel('left', 'Y Position (mm)')
        self.xy_plot.setLabel('bottom', 'X Position (mm)')
        self.xy_plot.setTitle("Ant Trajectory (Top View)")
        self.xy_plot.setAspectLocked(True)  # geometry correctness
        self.true_curve = self.xy_plot.plot(pen=pg.mkPen((153, 102, 0, 204), width=2))
        self.sim_curve = self.xy_plot.plot(pen=pg.mkPen((180, 0, 180), width=2))
        bottom_splitter.addWidget(self.xy_plot)

        self.error_plot = pg.PlotWidget()
        self.error_plot.showGrid(x=True, y=True)
        self.error_plot.setLabel('left', 'Position Error (mm)')
        self.error_plot.setLabel('bottom', 'Step')
        self.error_plot.setTitle("Path Integration Error")
        self.error_curve = self.error_plot.plot(pen=pg.mkPen((200, 50, 50), width=2))
        bottom_splitter.addWidget(self.error_plot)

        self.right_splitter.addWidget(bottom_splitter)

        self.outer_splitter.addWidget(self.right_splitter)
        self.outer_splitter.setSizes([280, 900])

        # ============================
        # MAIN LAYOUT
        # ============================
        layout = QtWidgets.QHBoxLayout(self)
        layout.addWidget(self.outer_splitter)

        # --- Timer for live updates ---
        self.setup_scene()
        self.timer = QtCore.QTimer()
        # self.timer.timeout.connect(self._animate_step)
        self.timer.timeout.connect(self.simulation_step)
        self.timer.start(30)  # ~30ms/frame


        
    def setup_scene(self):
        """Creates a 2D canvas on the XY plane."""

        # Single XY Plane Grid ---
        grid = gl.GLGridItem()
        grid.setSize(x=1500, y=1500)
        grid.setSpacing(x=10, y=10)
        # No rotation -> stays in XY plane
        self.view.addItem(grid)

        axis_len = 200

        # X axis
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [axis_len, 0, 0], (1, 0, 0, 1)
        ))

        # Y axis
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [0, axis_len, 0], (0, 1, 0, 1)
        ))

        # -X axis 
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [-axis_len, 0, 0], (1, 0.4, 0.4, 1)
        ))

        # -Y axis 
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [0, -axis_len, 0], (0.4, 1, 0.4, 1)
        ))


        # Z axis
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [0, 0, 50], (0, 0, 1, 1)
        ))

        # Labels ---
        x_label = GLTextItem(
            pos=[axis_len + 5, 0, 0],
            text="X",
            color=(255, 0, 0, 255)
        )
        self.view.addItem(x_label)

        y_label = GLTextItem(
            pos=[0, axis_len + 5, 0],
            text="Y",
            color=(0, 255, 0, 255)
        )
        self.view.addItem(y_label)

        z_label = GLTextItem(
            pos=[0, 0, 55],
            text="Z",
            color=(0, 0, 255, 255)
        )
        self.view.addItem(z_label)


        neg_x_label = GLTextItem(
            pos=[-axis_len - 5, 0, 0],
            text="-X",
            color=(255, 100, 100, 255)
        )
        self.view.addItem(neg_x_label)

        neg_y_label = GLTextItem(
            pos=[0, -axis_len - 5, 0],
            text="-Y",
            color=(100, 255, 100, 255)
        )
        self.view.addItem(neg_y_label)



        # Camera (top-down 2D view) ---
        self.view.setCameraPosition(distance=300, elevation=90, azimuth=0)
        self.view.opts['center'] = QtGui.QVector3D(0, 0, 0)

        # Add agent(Ant) to scene
        self.agent.spawn(0, 0, 2.5, view=self.view)
        self.view.addItem(self.agent.mesh_item)

        # Place the sun high above the world for visibility
        sun_items = create_sun(x=200, y=200, z=200, radius=10, ray_length=40)

        for item in sun_items:
            self.view.addItem(item)

        self.view.addItem(self.trails.true_trail)
        self.view.addItem(self.trails.sim_trail)



    def _animate_step(self):
        dt = 0.1 

        #COLLISION WITH FOOD (Search Mode) ---
        if self.fsm.state == AgentState.SEARCH:
            # (No changes here, logic was fine)
            found, items_to_remove = self.environment.check_food_collision(self.agent.get_true_pos())
            if found:
                for item in items_to_remove:
                    self.view.removeItem(item)
                self.fsm.set_state(AgentState.RETURN)

        # COLLISION WITH HOME 
        if self.fsm.state == AgentState.STOP:
            # Judge using TRUE positions
            true_pos = self.agent.get_true_pos()[:2]
            dist_center_to_center = np.linalg.norm(true_pos)
            ant_r = getattr(self.agent, 'ant_radius', 10.0) 
            home_r = self.env_cfg.home_detection_radius
            
            combined_threshold = ant_r + home_r
            
            if dist_center_to_center < combined_threshold:
                print(f"SUCCESS: Arrived! (Dist: {dist_center_to_center:.2f})")
            else:
                gap = dist_center_to_center - combined_threshold
                print(f"FAILED: Missed by {gap:.2f}mm")

        # PLOTS 
        if len(self.trails.true_trail_data) > 0:
            self.true_curve.setData(self.trails.true_trail_data[:, 0], self.trails.true_trail_data[:, 1])
            self.sim_curve.setData(self.trails.sim_trail_data[:, 0], self.trails.sim_trail_data[:, 1])

        errors = np.linalg.norm(
            self.trails.true_trail_data[:, :2] - self.trails.sim_trail_data[:, :2], axis=1
        ) if len(self.trails.true_trail_data) > 0 else np.array([])
        self.error_curve.setData(errors)


    def simulation_step(self):
        if not self.simulation_active:
            return
          
        self.step_count += 1
        mode_index = self.mode_selector.currentIndex()

        # --- LOGIC BLOCK: SUN COMPASS ---
        if mode_index == 2:
            if self.fsm.state in [AgentState.SEARCH, AgentState.RETURN]:
                SCAN_THRESHOLD_MM = self.agent.scan_threshold 
                if self.agent.distance_since_last_scan >= SCAN_THRESHOLD_MM:
                    self.agent.scan_sun()
                    self.log("Sun Scan: Heading Corrected.")

                    # Add Marker at True Position
                    tx, ty, tz = self.agent.get_true_pos()
                    scan_circle = create_circle(radius=5, x=tx, y=ty, z=1, color=(1, 1, 0, 1))
                    self.view.addItem(scan_circle)
                    self.scan_markers.append(scan_circle)

        # --- LOGIC BLOCK MOVEMENT ---
        # Case: Cheat Mode (Mode 1) AND Returning
        if mode_index == 1 and self.fsm.state == AgentState.RETURN:
            # Drive using True Position
            true_pos = self.agent.get_true_pos()[:2]
            dx, dy, done = self.fsm.homing_policy.step(true_pos)
            self.agent.move(dx, dy)
            if done: self.fsm.set_state(AgentState.STOP)
            
        # Case: Standard FSM 
        else:
            # Drive using Est Position
            self.fsm.update()

        # --- LOGIC BLOCK : UPDATE VISUALS ---
        if self.fsm.state in [AgentState.SEARCH, AgentState.RETURN]:
            self.trails.update(self.agent.get_true_pos(), self.agent.get_sim_pos())
            
            # Animate Step handles Collisions & Plots
            self._animate_step()
            
            # Camera Follow
            px, py, pz = self.agent.position
            self.view.follow(px, py, pz)

        # --- LOGIC BLOCK : END GAME LOGGING ---
        if self.fsm.state == AgentState.STOP and not self.results_logged:
            true_pos = self.agent.get_true_pos()[:2]
            dist_error = np.linalg.norm(true_pos)
            
            # --- CHANGED: Use combined threshold here too ---
            ant_r = getattr(self.agent, 'ant_radius', 10.0) 
            combined_threshold = self.env_cfg.home_detection_radius + ant_r
            
            if dist_error < combined_threshold:
                self.log(f"SUCCESS: Home reached! (Err: {dist_error:.1f}mm)")
            else:
                gap = dist_error - combined_threshold
                self.log(f"FAILED: Missed Nest by {gap:.1f}mm")
            
            self.results_logged = True
            self.cleanup_experiment()




    def update_transforms(self):
        pass
    
    def start_search(self):

        # Disable all UI controls once simulation starts
        for w in [
            self.bias_mean_input,
            self.bias_std_input,
            self.stride_noise_input,
            self.heading_noise_input,
            self.speed_slider,
            self.scan_interval_input,
            self.turn_slider,
            self.home_thresh_input,
            self.n_food_input,
            self.food_det_input,
            self.home_det_input,
            self.spawn_min_input,
            self.spawn_max_input
        ]:
            w.setEnabled(False)

            
        if not self.validate_parameters():
            QtWidgets.QMessageBox.warning(
                self, "Invalid Parameters", "Fix parameters before starting simulation."
            )
            return
        
        self.simulation_active = True
        # Reset flags
        self.results_logged = False
        self.step_count = 0
        self.apply_parameters()
        self.rebuild_environment()

        

        self.agent.spawn(0, 0, 2.5, view=self.view)
        self.trails.reset() 
        self.fsm.set_state(AgentState.SEARCH)
        self.pause_button.setText("Pause")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.stop_button.setEnabled(True)
        self.log("Simulation Started.")

    def toggle_pause(self):
        # If we are running (Search or Return), we PAUSE
        if self.fsm.state in [AgentState.SEARCH, AgentState.RETURN]:
            self.previous_state = self.fsm.state # Remember what we were doing
            self.fsm.set_state(AgentState.IDLE)
            self.pause_button.setText("Resume")
            self.log("Simulation Paused.")
            
        # If we are IDLE (Paused), we RESUME
        elif self.fsm.state == AgentState.IDLE:
            if hasattr(self, 'previous_state'):
                self.fsm.set_state(self.previous_state)
            else:
                # Fallback if state was lost
                self.fsm.set_state(AgentState.SEARCH) 
                
            self.pause_button.setText("Pause")
            self.log("Simulation Resumed.")

    def stop_search(self):
        """Manually stops the simulation."""
        if self.fsm.state != AgentState.STOP:
            self.fsm.set_state(AgentState.STOP)
            self.log("Simulation stopped by user.")
            
            self.cleanup_experiment()


    def validate_parameters(self):
        if self.bias_std_input.value() < 0:
            return False
        if self.heading_noise_input.value() < 0:
            return False
        if self.stride_noise_input.value() < 0:
            return False
        if self.speed_slider.value() <= 0:
            return False
        return True


    def apply_parameters(self):

        # Agent parameters
        self.agent.heading_bias_mean = self.bias_mean_input.value()
        self.agent.heading_bias_std = self.bias_std_input.value()
        self.agent.heading_noise_std = self.heading_noise_input.value()
        self.agent.stride_noise_std = self.stride_noise_input.value()
        self.agent.scan_threshold = self.scan_interval_input.value()

        # Navigation speed / CRW / homing
        speed = self.speed_slider.value()
        self.search_policy.step_length = speed
        self.homing_policy.step_length = speed
        self.search_policy.turn_std = np.deg2rad(self.turn_slider.value())
        self.homing_policy.home_threshold = self.home_thresh_input.value()

        # Environment
        self.env_cfg.n_food_items = self.n_food_input.value()
        self.env_cfg.food_detection_radius = self.food_det_input.value()
        self.env_cfg.home_detection_radius = self.home_det_input.value()
        self.env_cfg.food_spawn_min_radius = self.spawn_min_input.value()
        self.env_cfg.food_spawn_max_radius = self.spawn_max_input.value()


    def rebuild_environment(self):

        # Clear Logs
        self.clear_log()


        # 1. Clear existing items from the View
        if self.home_circle_item is not None:
            self.view.removeItem(self.home_circle_item)
            self.home_circle_item = None

        # Remove Food Meshes
        for m in self.food_mesh_items:
            try:
                self.view.removeItem(m)
            except ValueError:
                pass # Item was already removed (eaten) during the sim
        self.food_mesh_items.clear()

        # Remove Food Circles 
        for c in self.food_circle_items:
            try:
                self.view.removeItem(c)
            except ValueError:
                pass # Item was already removed (eaten) during the sim
        self.food_circle_items.clear()

        # --- REMOVE OLD SCAN MARKERS ---
        self.clear_scan_markers()

        # 2. Reset and Rebuild the Logic/Physics world
        self.environment.reset()
        self.environment.build()

        # 3. Add Home Circle
        self.home_circle_item = self.environment.home_circle
        if self.home_circle_item:
            self.view.addItem(self.home_circle_item)

        # 4. Add Food Meshes (Handling the Dictionary structure)
        # --- UPDATED LOOP ---
        for item_data in self.environment.food_items:
            # Extract meshes from the dictionary
            food_mesh = item_data['food_mesh']
            food_circle = item_data['circle_mesh']
            
            # Add to 3D View
            self.view.addItem(food_mesh)
            self.view.addItem(food_circle)
            
            # Track them so we can delete them later
            self.food_mesh_items.append(food_mesh)
            self.food_circle_items.append(food_circle)

    def log(self, message):
        """Prints message to the sidebar console with a timestamp."""
        # Simple timestamp or step count
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.console.append(f"[{timestamp}] {message}")
        
        # Auto-scroll to bottom
        scrollbar = self.console.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


    def cleanup_experiment(self):
        """
        Called when simulation ends (Naturally OR via Stop button).
        Re-enables settings and resets internal flags.
        """
        self.simulation_active = False
        
        # 1. Re-enable all parameter controls
        for w in [
            self.bias_mean_input, self.bias_std_input, self.stride_noise_input,
            self.heading_noise_input, self.speed_slider,self.scan_interval_input, self.turn_slider,
            self.home_thresh_input, self.n_food_input, self.food_det_input,
            self.home_det_input, self.spawn_min_input, self.spawn_max_input,
            self.mode_selector # Don't forget the mode selector!
        ]:
            w.setEnabled(True)

        # 2. Reset Button States
        self.start_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)
        self.pause_button.setText("Pause Search") # Reset text if it was paused

        # 3. Log
        self.log("Simulation ended. Parameters unlocked.")

    def clear_log(self):
        """Clears the sidebar console log if it is not empty."""
        if not self.console.document().isEmpty():
            self.console.clear()
    
    def clear_scan_markers(self):
        """Helper to remove all yellow scan dots from the scene."""
        for marker in self.scan_markers:
            try:
                self.view.removeItem(marker)
            except ValueError:
                pass # Already removed or invalid
        self.scan_markers.clear()