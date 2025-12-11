# import numpy as np

# class Agent_Axis:

#     def __init__(self, name):
#         self.name = name
        
#         # --- State Vector [Angle (theta), Velocity (omega)] ---
#         # theta: degrees
#         # omega: degrees per second
#         self.state = np.array([0.0, 0.0]) 
        
#         # --- Physical Parameters ---
        

#     def update(self, dt):
        
#         return 


# class Agent_Model:
    
#     def __init__(self):
#         # We replace the dictionary configuration with Objects.
#         pass

#     def update(self):
#         """
#         Advances physics by one time step (dt).
#         """
#         dt = 0.01 # 10ms step (Assuming 100Hz loop for now)
        
#         # 1. Update Physics
#         angle_x = self.galvo_x.update(dt)
#         angle_y = self.galvo_y.update(dt)
        
#         # 2. Sync to the "Compatibility State" for the GUI
#         self.state["Cuboid 1 (Red)"]['current_angle'] = angle_x
#         self.state["Cuboid 2 (Blue)"]['current_angle'] = angle_y






# agent.py
import numpy as np
from stl import mesh
import pyqtgraph.opengl as gl

class Agent_Model:
    def __init__(self, stl_path):
        # --- STATIC STATE ---
        self.raw_mesh = mesh.Mesh.from_file(stl_path)
        self.mesh_item = self._create_glmesh()

        # --- DYNAMIC STATE ---
        self.position = np.array([0.0, 0.0, 0.0], dtype=float)
        self.rotation = 0.0  # degrees, if you ever need rotation

    # Convert STL to GLMeshItem and scale down
    def _create_glmesh(self):
        v = np.array(self.raw_mesh.vectors.reshape(-1, 3), dtype=np.float32)
        f = np.arange(len(v)).reshape(-1, 3)

        item = gl.GLMeshItem(
            vertexes=v,
            faces=f,
            smooth=False,
            drawEdges=True,
            edgeColor=(1,1,1,1),
            color=(0.7,0.7,0.7,1)
        )

        # Scale it down
        item.scale(0.01, 0.01, 0.01)

        return item


    # Place it in the world
    def spawn(self, x=0, y=0, z=0):
        self.position[:] = (x, y, z)
        self.mesh_item.translate(x, y, z)

    # Move in XY plane
    def move(self, dx, dy):
        self.position[0] += dx
        self.position[1] += dy
        self.mesh_item.translate(dx, dy, 0)

    # (Optional) future rotation
    def rotate(self, angle_deg):
        pass  # you can fill this later
