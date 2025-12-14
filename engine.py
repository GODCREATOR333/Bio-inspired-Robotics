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
        self.setGeometry(100, 100, 1200, 800)

        # --Load Configs--
        self.agent_cfg = AgentConfig()
        self.crw_cfg = CRWConfig()
        self.homing_cfg = HomingConfig()
        self.env_cfg=EnvironmentConfig()
        self.environment = Environment(self.env_cfg)
        self.home_circle_item = None
        self.food_mesh_items = []
        self.food_circle_items = []



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
            homing_policy=self.homing_policy
        )


        # --- Sidebar UI Setup ---
        self.outer_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.outer_splitter.setHandleWidth(6)

        # Sidebar container
        self.instruction_widget = QtWidgets.QWidget()
        self.instruction_layout = QtWidgets.QVBoxLayout()
        self.instruction_widget.setLayout(self.instruction_layout)
        self.outer_splitter.addWidget(self.instruction_widget)
        self.instruction_widget.setMinimumWidth(180)

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

        self.instruction_layout.addWidget(self.param_group)

        # --- Environment Parameters ---
        self.env_group = QtWidgets.QGroupBox("Environment")
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

        self.instruction_layout.addWidget(self.env_group)

        # --- Control Buttons ---
        self.start_button = QtWidgets.QPushButton("Start Search")
        self.pause_button = QtWidgets.QPushButton("Pause Search")
        self.stop_button = QtWidgets.QPushButton("Stop Search")
        self.instruction_layout.addWidget(self.start_button)
        self.instruction_layout.addWidget(self.pause_button)
        self.instruction_layout.addWidget(self.stop_button)

        # Connect buttons
        self.start_button.clicked.connect(self.start_search)
        self.pause_button.clicked.connect(self.toggle_pause)
        self.stop_button.clicked.connect(self.stop_search)

        # Initial button state
        self.start_button.setEnabled(True)
        self.pause_button.setEnabled(False)
        self.stop_button.setEnabled(False)

        self.instruction_layout.addStretch(1)

        self.outer_splitter.addWidget(self.instruction_widget)
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
            #1
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
        
        
        self.apply_parameters()
        self.rebuild_environment()

        self.agent.spawn(0, 0, 2.5, view=self.view)
        self.trails.reset() 
        self.fsm.set_state(AgentState.SEARCH)
        self.pause_button.setText("Pause")
        self.start_button.setEnabled(False)
        self.pause_button.setEnabled(True)
        self.stop_button.setEnabled(True)
        self.simulation_active = True

    def toggle_pause(self):
        if self.fsm.state == AgentState.SEARCH:
            # Currently searching → pause
            self.fsm.set_state(AgentState.IDLE)
            self.pause_button.setText("Resume Search")
        elif self.fsm.state == AgentState.IDLE:
            # Currently paused → resume
            self.fsm.set_state(AgentState.SEARCH)
            self.pause_button.setText("Pause Search")

    def stop_search(self):
        if self.fsm.state != AgentState.STOP:
            self.fsm.set_state(AgentState.STOP)
            #self.export_simulation_data()
            self.environment.clear()
            # Disable buttons
            self.stop_button.setEnabled(False)
            self.pause_button.setEnabled(False)
            self.start_button.setEnabled(True)

            # Re-enable all UI parameter controls
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
            w.setEnabled(True)


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
        # Remove old home circle
        if self.home_circle_item is not None:
            self.view.removeItem(self.home_circle_item)
            self.home_circle_item = None

        # Remove old food meshes
        for m in self.food_mesh_items:
            self.view.removeItem(m)
        self.food_mesh_items.clear()

        # Remove old food circles
        for c in self.food_circle_items:
            self.view.removeItem(c)
        self.food_circle_items.clear()

        # Build new environment
        self.environment.reset()
        self.environment.build()

        # Add new home circle
        self.home_circle_item = self.environment.home_circle
        self.view.addItem(self.home_circle_item)

        # Add new food meshes and circles
        for food_mesh, food_circle in self.environment.food_items:
            self.view.addItem(food_mesh)
            self.view.addItem(food_circle)
            self.food_mesh_items.append(food_mesh)
            self.food_circle_items.append(food_circle)

