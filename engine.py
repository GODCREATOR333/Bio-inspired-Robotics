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
from food import Food_Model
from utils import random_point_outside_radius,TrailManager


from fsm_controller import FSMController, AgentState
from correlated_random_walk import CorrelatedRandomWalk
from homing_policy import VectorHoming


class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bio-Inspired Robotics")
        self.setGeometry(100, 100, 1200, 800)

        # --- Trails and agent ---
        self.trails = TrailManager()
        self.agent = Agent_Model("CAD_Model_ant/ant_model.stl", scale=0.01)
        self.objects = {}


        # Has Simulation started ?
        self.simulation_active = False


        # --- Navigation policies ---
        self.search_policy = CorrelatedRandomWalk(
            step_length=2.0,
            turn_std=np.deg2rad(15)
        )

        self.homing_policy = VectorHoming(
            step_length=2.0,
            home_threshold=5.0
        )

        # --- FSM ---
        self.fsm = FSMController(
            agent=self.agent,
            search_policy=self.search_policy,
            homing_policy=self.homing_policy
        )


        # UI code below -----
        # --- Outer horizontal splitter: sidebar | main content ---
        self.outer_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.outer_splitter.setHandleWidth(6)

        # Sidebar
        self.instruction_widget = QtWidgets.QWidget()
        self.instruction_layout = QtWidgets.QVBoxLayout()
        self.instruction_widget.setLayout(self.instruction_layout)

        self.instructions_text = QtWidgets.QTextEdit()
        self.instructions_text.setReadOnly(True)
        self.instructions_text.setPlainText(
            "Instructions:\n"
            "- Use arrow keys to move the ant.\n"
            "- Press R to reset camera and ant.\n"
            "- Use I/O to zoom in/out.\n"
            "- Use W/A/S/D to pan camera.\n"
            "-------------------------------\n"
            "--- Reference / Constants ---\n"
            "- Home detection range: 20 mm\n"
            "- Ant detection range: 10 mm\n"
            "- Dead bug detection range: 10 mm\n"
            "- Sun position: purely visual; actual computation in get_sun_azimuth()"
        )
        self.instructions_text.setMaximumHeight(550)

        self.instruction_layout.addWidget(self.instructions_text)

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

        # Init the buttons 
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
        self.true_curve = self.xy_plot.plot(pen=pg.mkPen(color=(0, 180, 0), width=2))
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
        

        # Add detection radius to Home
        circle = create_circle(radius=20,x=0,y=0,z=0, color=(0.1, 0.7, 1.0, 0.5))
        self.view.addItem(circle)

        # Place the sun high above the world for visibility
        sun_items = create_sun(x=200, y=200, z=200, radius=10, ray_length=40)

        for item in sun_items:
            self.view.addItem(item)


        # --- Spawn dead bug food ---
        MIN_RADIUS = 200   # forbidden zone radius
        MAX_RADIUS = 400  # how far bugs can spawn
        for i in range(15):
            fx, fy = random_point_outside_radius(MIN_RADIUS, MAX_RADIUS)
            
            food = Food_Model("CAD_Model_ant/fly_model.stl", scale=8)

            # apply rotation to orient correctly
            food.mesh_item.rotate(-90, 1, 0, 0)  

            food.spawn(fx, fy, 1) #Z=1,spawn above x,y plane
            circle_dead_bug=create_circle(radius=10,x=fx,y=fy,z=0,color=(0,0.5,1,0.8)) #radius = detection radius
            

            self.view.addItem(food.mesh_item)
            self.view.addItem(circle_dead_bug)
            self.objects[f"food_{i}"] = food


        # Add the true trail to the scene (only true trail rendered)
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
        # Reset ant & trails if coming from STOP
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
            # Disable buttons
            self.stop_button.setEnabled(False)
            self.pause_button.setEnabled(False)
            self.start_button.setEnabled(True)