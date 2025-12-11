import sys
import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLTextItem
from PyQt5 import QtGui, QtCore, QtWidgets

from viewer import MyView
import geometry
from agent import Agent_Model
from geometry import create_circle
from food import Food_Model
from utils import random_point_outside_radius

class MainWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bio-Inspired Robotics")
        self.setGeometry(100, 100, 1200, 800)

        self.agent = Agent_Model(
            "CAD_Model_ant/ant_model.stl",
            scale=0.01)
        

        self.objects = {}

        # --- Layout & View ---
        self.view = MyView()
        self.view.main = self  # allow view to call movement

        self.main_layout = QtWidgets.QHBoxLayout()
        self.setLayout(self.main_layout)
        self.main_layout.addWidget(self.view, 1)

        self.setup_scene()

        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self._animate_step)

        self.update_transforms()

    def setup_scene(self):
        """Creates a 2D canvas on the XY plane."""

        # --- 1. Single XY Plane Grid ---
        grid = gl.GLGridItem()
        grid.setSize(x=500, y=500)
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


        # --- 4. Camera (top-down 2D view) ---
        self.view.setCameraPosition(distance=300, elevation=90, azimuth=0)
        self.view.opts['center'] = QtGui.QVector3D(0, 0, 0)

        # Add agent to scene
        self.view.addItem(self.agent.mesh_item)
        self.agent.mesh_item.translate(0, 0, 2.5)

        # Add agent to scene
        circle = create_circle(radius=20, color=(0.1, 0.7, 1.0, 0.5))
        self.view.addItem(circle)


        # --- Spawn dead bug food ---
        MIN_RADIUS = 100   # forbidden zone radius
        MAX_RADIUS = 300  # how far bugs can spawn

        for i in range(5):
            fx, fy = random_point_outside_radius(MIN_RADIUS, MAX_RADIUS)
            
            food = Food_Model("CAD_Model_ant/fly_model.stl", scale=8)

            # apply rotation to orient correctly
            food.mesh_item.rotate(-90, 1, 0, 0)  

            food.spawn(fx, fy, 1)   # z=1 so no z-fighting

            self.view.addItem(food.mesh_item)
            self.objects[f"food_{i}"] = food


    # Dummy methods until you fill them
    def _animate_step(self):
        pass

    def update_transforms(self):
        pass
