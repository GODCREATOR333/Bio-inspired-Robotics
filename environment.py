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

            # Create Mesh
            food = Food_Model("CAD_Model_ant/fly_model.stl",scale=10)
            food.mesh_item.rotate(-90, 1, 0, 0)
            food.spawn(x, y, 1)

            # Create Detection Circle
            circle_dead_bug = create_circle(
                radius=self.cfg.food_detection_radius,
                x=x, y=y, z=0,
                color=(0, 0.5, 1, 0.8)
            )

            # Store everything in a dictionary
            item_data = {
                'x': x,
                'y': y,
                'food_mesh': food.mesh_item,
                'circle_mesh': circle_dead_bug
            }
            self.food_items.append(item_data)



        
    def check_food_collision(self, agent_true_pos):
            """
            Checks collision between agent (true pos) and all food items.
            
            Returns:
                found (bool): True if collision happened
                items_to_remove (list): List of GLItems to remove from 3D view (or None)
            """
            agent_x, agent_y = agent_true_pos[0], agent_true_pos[1]
            
            # Combined threshold: Ant Body + Food Detection Zone
            collision_threshold = self.cfg.food_detection_radius + 20.0
            
            for i, item in enumerate(self.food_items):
                fx, fy = item['x'], item['y']
                
                # Fast Euclidean Distance
                dist = np.hypot(agent_x - fx, agent_y - fy)
                
                if dist < collision_threshold:
                    # --- COLLISION DETECTED ---
                    
                    # 1. Remove from our data list (so we don't eat it twice)
                    popped_item = self.food_items.pop(i)
                    
                    # 2. Return the graphical items so Engine can delete them from View
                    items_to_remove = [popped_item['food_mesh'], popped_item['circle_mesh']]
                    
                    return True, items_to_remove
            
            return False, None

  
    def get_renderables(self):
        items = []
        if self.home_circle:
            items.append(self.home_circle)

        # Iterate through the dictionary list
        for item in self.food_items:
            items.append(item['food_mesh'])
            items.append(item['circle_mesh'])

        return items

    def reset(self):
        self.home_circle = None
        self.food_items.clear()

    def clear(self):
        self.reset()
