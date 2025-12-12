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

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bio-Inspired Robotics")
        self.setGeometry(100, 100, 1200, 800)

        

        #Create trails manager
        self.trails = TrailManager()


        self.agent = Agent_Model("CAD_Model_ant/ant_model.stl", scale=0.01)
        self.objects = {}

        # --- Main layout ---
        self.main_layout = QtWidgets.QHBoxLayout()  # Horizontal: view + sidebar
        self.setLayout(self.main_layout)

        # --- Instructions sidebar ---
        self.instruction_widget = QtWidgets.QWidget()
        self.instruction_layout = QtWidgets.QVBoxLayout()
        self.instruction_widget.setLayout(self.instruction_layout)

        # Fixed width sidebar
        self.instruction_widget.setFixedWidth(250)

        self.instructions_text = QtWidgets.QTextEdit()
        self.instructions_text.setReadOnly(True)
        self.instructions_text.setPlainText(
            "Instructions:\n"
            "- Use arrow keys to move the ant.\n"
            "- Press R to reset camera and ant.\n"
            "- Use I/O to zoom in/out.\n"
            "- Use W/A/S/D to pan camera.\n"

            "-------------------------------\n"
            "-------------------------------\n"

            "--- Reference / Constants ---\n"
            "- Home detection range: 20 mm\n"
            "- Ant detection range: 10 mm\n"
            "- Dead bug detection range: 10 mm\n"
            "- Sun position: purely visual; actual computation in get_sun_azimuth()"
            
        )

        self.instruction_layout.addWidget(self.instructions_text)
        self.main_layout.addWidget(self.instruction_widget, 1)  # stretch factor 1


        # --- 3D view ---
        self.view = MyView()
        self.view.main = self  # allow view to call movement
        self.main_layout.addWidget(self.view, 3)  # stretch factor 3 (takes most space)
        # --- Setup scene and timer ---
        self.setup_scene()
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._animate_step)
        self.update_transforms()


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
        pass


    def update_transforms(self):
        pass
