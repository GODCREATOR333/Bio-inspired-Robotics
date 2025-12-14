import numpy as np
from stl import mesh
import pyqtgraph.opengl as gl
from utils import random_point_outside_radius
from geometry import create_circle

class Food_Model:
    def __init__(self, stl_path, scale=0.01):
        self.raw_mesh = mesh.Mesh.from_file(stl_path)
        self.vertices = np.array(self.raw_mesh.vectors.reshape(-1, 3), dtype=np.float32)
        self.vertices -= self.vertices.mean(axis=0)
        self.faces = np.arange(len(self.vertices)).reshape(-1, 3)
        self.mesh_item = gl.GLMeshItem(
            vertexes=self.vertices,
            faces=self.faces,
            smooth=False,
            drawEdges=True,
            edgeColor=(0, 0, 1, 1),
            color=(0, 1, 0.1, 1),
        )
        self.mesh_item.scale(scale, scale, scale)
        self.mesh_item.rotate(-90, 0, 1, 0)

    def spawn(self, x, y, z=0):
        self.mesh_item.translate(x, y, z)


# environment.py

class Environment:
    def __init__(self, env_cfg):
        self.cfg = env_cfg
        self.home_circle = None
        self.food_items = []   # list of (food_mesh, detection_circle)

    def build(self):
        self._build_home()
        self._build_food()

    def _build_home(self):
        self.home_circle = create_circle(
            radius=self.cfg.home_detection_radius,
            x=0, y=0, z=0,
            color=(0.6, 0.3, 0, 1)
        )

    def _build_food(self):
        self.food_items.clear()

        for _ in range(self.cfg.n_food_items):
            x, y = random_point_outside_radius(
                self.cfg.food_spawn_min_radius,
                self.cfg.food_spawn_max_radius
            )

            food = Food_Model("CAD_Model_ant/fly_model.stl",scale=10)
            food.mesh_item.rotate(-90, 1, 0, 0)
            food.spawn(x, y, 1)

            circle_dead_bug = create_circle(
                radius=self.cfg.food_detection_radius,
                x=x, y=y, z=0,
                color=(0, 0.5, 1, 0.8)
            )

            self.food_items.append((food.mesh_item, circle_dead_bug))

    def get_renderables(self):
        items = []
        if self.home_circle:
            items.append(self.home_circle)

        for food_mesh, circle in self.food_items:
            items.append(food_mesh)
            items.append(circle)

        return items

    def reset(self):
        self.home_circle = None
        self.food_items.clear()

    def clear(self):
        self.reset()
