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
        self.step_count = 0  # Needed for the Sun Compass timer later



        # --- Trails and agent ---
        self.trails = TrailManager()
        self.agent = Agent_Model("CAD_Model_ant/ant_model.stl",agent_cfg=self.agent_cfg, scale=0.01)
        self.objects = {}


        # Has Simulation started ?
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


        # --- Sidebar UI Setup ---
        self.outer_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.outer_splitter.setHandleWidth(6)

        # Sidebar container
        self.instruction_widget = QtWidgets.QWidget()
        self.instruction_layout = QtWidgets.QVBoxLayout()
        self.instruction_widget.setLayout(self.instruction_layout)
        self.outer_splitter.addWidget(self.instruction_widget)
        self.instruction_widget.setMinimumWidth(150)

        # Instructions
        self.instructions_text = QtWidgets.QTextEdit()
        self.instructions_text.setReadOnly(True)
        self.instructions_text.setPlainText(
            "Instructions:\n"
            "- Press R to reset camera and ant.\n"
            "- Use I/O to zoom in/out.\n"
            "- Use W/A/S/D to pan camera.\n"
            "-------------------------------\n"
            "--- Reference / Constants ---\n"
            "- Home detection range: 20 mm\n"
            "- Ant detection range: 10 mm\n"
            "- Dead bug detection range: 10 mm\n"
            "- Sun position: visual only"
        )
        self.instructions_text.setMaximumHeight(200)
        self.instruction_layout.addWidget(self.instructions_text)

        # --- 1. EXPERIMENT SELECTOR (The Research Switch) ---
        self.exp_group = QtWidgets.QGroupBox("Experiment Mode")
        self.exp_layout = QtWidgets.QVBoxLayout()
        self.exp_group.setLayout(self.exp_layout)
        
        self.mode_selector = QtWidgets.QComboBox()
        self.mode_selector.addItems([
            "1. Blind (Est Pos) - EXPECT DRIFT", 
            "2. Control (True Pos) - CHEAT", 
            "3. Sun Compass (Corrected) - SOLUTION"
        ])
        self.exp_layout.addWidget(self.mode_selector)
        self.instruction_layout.addWidget(self.exp_group)

        # --- 2. DYNAMIC LOG CONSOLE (Replacing static instructions) ---
        self.log_group = QtWidgets.QGroupBox("Simulation Log")
        self.log_layout = QtWidgets.QVBoxLayout()
        self.log_group.setLayout(self.log_layout)

        self.console = QtWidgets.QTextEdit()
        self.console.setReadOnly(True)
        self.console.setStyleSheet("background-color: #222; color: #0f0; font-family: Monospace; font-size: 10pt;")
        self.console.setPlainText("--- SYSTEM READY ---\n")
        self.log_layout.addWidget(self.console)
        
        self.instruction_layout.addWidget(self.log_group)


        # ============================
        # SCROLLABLE PARAMETERS BOX
        # ============================
        self.param_scroll = QtWidgets.QScrollArea()
        self.param_scroll.setWidgetResizable(True)
        self.param_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.param_scroll.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.param_scroll.setFixedHeight(360)  # <-- THIS is the key

        param_container = QtWidgets.QWidget()
        param_layout = QtWidgets.QVBoxLayout(param_container)
        param_layout.setContentsMargins(6, 6, 6, 6)
        param_layout.setSpacing(10)

        # ---------- Agent Parameters ----------
        agent_group = QtWidgets.QGroupBox("Agent Parameters")
        agent_form = QtWidgets.QFormLayout(agent_group)

        # --- Agent Parameters ---
        self.param_group = QtWidgets.QGroupBox("Agent Parameters")
        self.param_layout = QtWidgets.QFormLayout()
        self.param_group.setLayout(self.param_layout)

        self.bias_mean_input = QtWidgets.QDoubleSpinBox()
        self.bias_mean_input.setRange(-5.0, 5.0)
        self.bias_mean_input.setSingleStep(0.1)
        self.bias_mean_input.setValue(self.agent_cfg.heading_bias_mean)

        self.bias_std_input = QtWidgets.QDoubleSpinBox()
        self.bias_std_input.setRange(0.0, 5.0)
        self.bias_std_input.setSingleStep(0.1)
        self.bias_std_input.setValue(self.agent_cfg.heading_bias_std)

        self.heading_noise_input = QtWidgets.QDoubleSpinBox()
        self.heading_noise_input.setRange(0.0, 5.0)
        self.heading_noise_input.setSingleStep(0.1)
        self.heading_noise_input.setValue(self.agent_cfg.heading_noise_std)

        self.stride_noise_input = QtWidgets.QDoubleSpinBox()
        self.stride_noise_input.setRange(0.0, 5.0)
        self.stride_noise_input.setSingleStep(0.1)
        self.stride_noise_input.setValue(self.agent_cfg.stride_noise_std)

        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(1, 100)
        self.speed_slider.setValue(int(self.agent_cfg.agent_speed))
        self.speed_label = QtWidgets.QLabel(f"{self.agent_cfg.agent_speed:.1f}")
        self.speed_slider.valueChanged.connect(
            lambda v: self.speed_label.setText(f"{v:.1f}")
        )

        self.turn_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.turn_slider.setRange(0, 90)
        self.turn_slider.setValue(int(np.rad2deg(self.crw_cfg.turn_std)))
        self.turn_label = QtWidgets.QLabel(f"{np.rad2deg(self.crw_cfg.turn_std):.1f}°")
        self.turn_slider.valueChanged.connect(
            lambda v: self.turn_label.setText(f"{v:.1f}°")
        )

        self.home_thresh_input = QtWidgets.QDoubleSpinBox()
        self.home_thresh_input.setRange(1.0, 50.0)
        self.home_thresh_input.setSingleStep(1.0)
        self.home_thresh_input.setValue(self.homing_cfg.home_threshold)

        # Add agent parameters
        self.param_layout.addRow("Heading Bias mean", self.bias_mean_input)
        self.param_layout.addRow("Heading Bias std", self.bias_std_input)
        self.param_layout.addRow("Heading noise std", self.heading_noise_input)
        self.param_layout.addRow("Stride noise std", self.stride_noise_input)
        self.param_layout.addRow("Speed", self.speed_slider)
        self.param_layout.addRow("", self.speed_label)
        self.param_layout.addRow("Turn std", self.turn_slider)
        self.param_layout.addRow("", self.turn_label)
        self.param_layout.addRow("Home threshold", self.home_thresh_input)


        # --- Environment Parameters ---
        self.env_group = QtWidgets.QGroupBox("Environment Parameters")
        env_layout = QtWidgets.QFormLayout()
        self.env_group.setLayout(env_layout)

        self.n_food_input = QtWidgets.QSpinBox()
        self.n_food_input.setRange(1, 100)
        self.n_food_input.setValue(self.env_cfg.n_food_items)

        self.food_det_input = QtWidgets.QDoubleSpinBox()
        self.food_det_input.setRange(1.0, 100.0)
        self.food_det_input.setValue(self.env_cfg.food_detection_radius)

        self.home_det_input = QtWidgets.QDoubleSpinBox()
        self.home_det_input.setRange(1.0, 100.0)
        self.home_det_input.setValue(self.env_cfg.home_detection_radius)

        self.spawn_min_input = QtWidgets.QDoubleSpinBox()
        self.spawn_min_input.setRange(10.0, 1000.0)
        self.spawn_min_input.setValue(self.env_cfg.food_spawn_min_radius)

        self.spawn_max_input = QtWidgets.QDoubleSpinBox()
        self.spawn_max_input.setRange(10.0, 2000.0)
        self.spawn_max_input.setValue(self.env_cfg.food_spawn_max_radius)

        env_layout.addRow("Food items", self.n_food_input)
        env_layout.addRow("Food detect r", self.food_det_input)
        env_layout.addRow("Home detect r", self.home_det_input)
        env_layout.addRow("Spawn min r", self.spawn_min_input)
        env_layout.addRow("Spawn max r", self.spawn_max_input)

        # --- Control Buttons ---
        self.start_button = QtWidgets.QPushButton("Start Search")
        self.pause_button = QtWidgets.QPushButton("Pause Search")
        self.stop_button = QtWidgets.QPushButton("Stop Search")
        self.instruction_layout.addWidget(self.start_button)
        self.instruction_layout.addWidget(self.pause_button)
        self.instruction_layout.addWidget(self.stop_button)

        param_layout.addWidget(self.param_group)
        param_layout.addWidget(self.env_group)
        param_layout.addStretch(1)

        # Attach container to scroll area
        self.param_scroll.setWidget(param_container)

        # Add scroll area to sidebar
        self.instruction_layout.addWidget(self.param_scroll)


        # Connect buttons
        self.start_button.clicked.connect(self.start_search)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.stop_button.clicked.connect(self.stop_search)

        # Initial button state
        self.start_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        self.instruction_layout.addStretch(1)

        self.instruction_widget.setMinimumWidth(150)


        # --- Right side: vertical splitter (3D view | bottom plots) ---
        self.right_splitter = QtWidgets.QSplitter(QtCore.Qt.Vertical)
        self.right_splitter.setHandleWidth(6)

        # 3D view (top)
        self.view = MyView()
        self.view.main = self
        self.right_splitter.addWidget(self.view)
        self.view.setMinimumHeight(200)

        # Bottom plots: horizontal splitter (XY | Error)
        self.bottom_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.bottom_splitter.setHandleWidth(5)

        # Left plot: XY positions
        self.xy_plot = pg.PlotWidget()
        self.xy_plot.showGrid(x=True, y=True)
        self.xy_plot.setLabel('left', 'Y Position')
        self.xy_plot.setLabel('bottom', 'X Position')
        self.true_curve = self.xy_plot.plot(pen=pg.mkPen(color=(0, 180, 180), width=2))
        self.sim_curve  = self.xy_plot.plot(pen=pg.mkPen(color=(180, 0, 180), width=2))
        self.bottom_splitter.addWidget(self.xy_plot)

        # Right plot: error
        self.error_plot = pg.PlotWidget()
        self.error_plot.showGrid(x=True, y=True)
        self.error_plot.setLabel('left', 'Error')
        self.error_plot.setLabel('bottom', 'Step')
        self.error_curve = self.error_plot.plot(pen=pg.mkPen(color=(200, 50, 50), width=2))
        self.bottom_splitter.addWidget(self.error_plot)

        # Add bottom splitter to vertical splitter
        self.right_splitter.addWidget(self.bottom_splitter)
        self.bottom_splitter.setSizes([600, 400])

        # Add right splitter to outer splitter
        self.outer_splitter.addWidget(self.right_splitter)

        # Set initial splitter sizes
        self.outer_splitter.setSizes([100, 950])  # Sidebar vs rest
        self.right_splitter.setSizes([500, 300])  # 3D view vs bottom

        # Set layout
        layout = QtWidgets.QHBoxLayout()
        layout.addWidget(self.outer_splitter)
        self.setLayout(layout)

        self.setMinimumSize(800, 600)

        # --- Timer for live updates ---
        self.setup_scene()
        self.timer = QtCore.QTimer()
        # self.timer.timeout.connect(self._animate_step)
        self.timer.timeout.connect(self.simulation_step)
        self.timer.start(30)  # ~30ms/frame


        
    def setup_scene(self):
        """Creates a 2D canvas on the XY plane."""

        # --- 1. Single XY Plane Grid ---
        grid = gl.GLGridItem()
        grid.setSize(x=1500, y=1500)
        grid.setSpacing(x=10, y=10)
        # No rotation -> stays in XY plane
        self.view.addItem(grid)

        # --- 2. Axes (in XY plane) ---
        axis_len = 200

        # X axis
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [axis_len, 0, 0], (1, 0, 0, 1)
        ))

        # Y axis
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [0, axis_len, 0], (0, 1, 0, 1)
        ))

        # -X axis (red, lighter shade optional)
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [-axis_len, 0, 0], (1, 0.4, 0.4, 1)
        ))

        # -Y axis (green, lighter shade optional)
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [0, -axis_len, 0], (0.4, 1, 0.4, 1)
        ))


        # Optional Z axis
        self.view.addItem(geometry.axis_line(
            [0, 0, 0], [0, 0, 50], (0, 0, 1, 1)
        ))

        # --- 3. Labels ---
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



        # --- 4. Camera (top-down 2D view) ---
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

        dt = 0.1 # Simulation timestep

        # -- CHECK WORLD EVENTS (Collisions) ---
        
        # Scenario A: Collision with FOOD (Only relevant if searching)
        if self.fsm.state == AgentState.SEARCH:
            # Check collision using True Physics Position
            found, items_to_remove = self.environment.check_food_collision(self.agent.get_true_pos())
            
            if found:
                # 1. Remove graphics
                for item in items_to_remove:
                    self.view.removeItem(item)
                
                # 2. Force State Change
                self.fsm.set_state(AgentState.RETURN)

        # Scenario B: Collision with HOME (Only relevant if Returning)
        # We check the result ONLY when the agent decides to stop.
        if self.fsm.state == AgentState.STOP:
            
            # Now we look at Truth
            true_pos = self.agent.get_true_pos()[:2]
            dist_error = np.linalg.norm(true_pos) # Dist from (0,0)
            
            # Check if it was a Success or a Miss
            if dist_error < self.env_cfg.home_detection_radius:
                #This happens if drift was low, or it got lucky
                print(f"SUCCESS: Arrived! Error: {dist_error:.2f}mm")
                pass 
            else:
                # This is the expected "Blind Navigation" outcome
                print(f"FAILED: Missed Nest by {dist_error:.2f}mm")
                pass
        
        # Update XY plot
        if len(self.trails.true_trail_data) > 0:
            self.true_curve.setData(
                self.trails.true_trail_data[:, 0],
                self.trails.true_trail_data[:, 1]
            )
            self.sim_curve.setData(
                self.trails.sim_trail_data[:, 0],
                self.trails.sim_trail_data[:, 1]
            )

        # Compute error (distance between true and sim positions)
        errors = np.linalg.norm(
            self.trails.true_trail_data[:, :2] - self.trails.sim_trail_data[:, :2],
            axis=1
        ) if len(self.trails.true_trail_data) > 0 else np.array([])
        self.error_curve.setData(errors)


    def simulation_step(self):
        if not self.simulation_active:
            return
        else:
            self.step_count += 1
            # 1. Check Experiment Mode
            mode_index = self.mode_selector.currentIndex()
            #2
                # --- MODE 2: SUN COMPASS LOGIC ---
            # If in "Sun Compass" mode, simulate intermittent scanning
                
            # --- MODE 2: SUN COMPASS LOGIC ---
            if mode_index == 2:
                if self.fsm.state in [AgentState.SEARCH, AgentState.RETURN]:
                    
                    # TRIGGER: Scan every 100mm of travel
                    # If speed is 3.0, this happens every ~33 frames.
                    # If speed is 50.0, this happens every ~2 frames.
                    SCAN_THRESHOLD_MM = 100.0 
                    
                    if self.agent.distance_since_last_scan >= SCAN_THRESHOLD_MM:
                        self.agent.scan_sun()
                        self.log("Sun Scan: Heading Corrected.")

                        # We use TRUE position so the dot appears under the Ant mesh
                        tx, ty, tz = self.agent.get_true_pos()
                        
                        # Optional: Add a visual marker (Yellow Flash)
                        scan_circle = create_circle(radius=5, x=tx, y=ty, z=1, color=(1, 1, 0, 1))
                        self.view.addItem(scan_circle)

                        self.scan_markers.append(scan_circle)

  

            # --- Run FSM Update ---
            # Special Case for Mode 1 (Cheat):
            if mode_index == 1 and self.fsm.state == AgentState.RETURN:
                # Feed TRUE position to homing policy temporarily
                true_pos = self.agent.get_true_pos()[:2]
                dx, dy, done = self.fsm.homing_policy.step(true_pos)
                self.agent.move(dx, dy)
                if done: self.fsm.set_state(AgentState.STOP)
            else:
                # Normal FSM (Uses Sim Position)
                self.fsm.update()
            
            # 2. Only log trails if agent moved
        if self.fsm.state in [AgentState.SEARCH, AgentState.RETURN]:
            true_pos = self.agent.get_true_pos()
            sim_pos = self.agent.get_sim_pos()
            self.trails.update(true_pos, sim_pos)
            
            #4
            self._animate_step()
            

            # Update Camera Position to follow the ant
            px, py, pz = self.agent.position
            self.view.follow(px, py, pz)

        if self.fsm.state == AgentState.STOP and not self.results_logged:
            true_pos = self.agent.get_true_pos()[:2]
            dist_error = np.linalg.norm(true_pos)
            
            radius = self.env_cfg.home_detection_radius
            
            if dist_error < radius:
                self.log(f"SUCCESS: Home reached! (Err: {dist_error:.1f}mm)")
            else:
                self.log(f"FAILED: Missed Nest by {dist_error:.1f}mm")
            
            self.results_logged = True # Prevent spamming log every frame
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
        self.log("Simulation Started.")
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
            
            self.timer.stop()
            # Use the same cleanup logic
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
        # Agent
        self.agent.heading_bias_mean = self.bias_mean_input.value()
        self.agent.heading_bias_std = self.bias_std_input.value()
        self.agent.heading_noise_std = self.heading_noise_input.value()
        self.agent.stride_noise_std = self.stride_noise_input.value()

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
            self.heading_noise_input, self.speed_slider, self.turn_slider,
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