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
from PyQt5 import QtGui
from geometry import create_circle
from viewer import MyView


class Agent_Model:
    def __init__(self, stl_path, scale=0.01):
        """
        :param stl_path: Path to STL file
        :param scale: Scaling factor to shrink the model
        :param texture_path: Optional path to a texture image
        """

        #Store scale factor
        self.scale_factor = scale
        self.detection_circle = None

        # --- Load STL ---
        self.raw_mesh = mesh.Mesh.from_file(stl_path)

        # --- Center vertices around origin ---
        self.vertices = np.array(self.raw_mesh.vectors.reshape(-1, 3), dtype=np.float32)
        center = self.vertices.mean(axis=0)
        self.vertices -= center  # center the mesh

        # --- Faces ---
        self.faces = np.arange(len(self.vertices)).reshape(-1, 3)

        # --- Dynamic state ---
        self.position = np.array([0.0, 0.0, 0.0], dtype=float)
        self.rotation = 0.0  # degrees

        # --- Create GLMeshItem ---
        self.mesh_item = gl.GLMeshItem(
            vertexes=self.vertices,
            faces=self.faces,
            smooth=False,
            drawEdges=True,
            edgeColor=(0, 0, 0, 1),
            color=(1,0.5,0.0,1),
        )
        self.mesh_item.scale(scale, scale, scale)

    # Place it in the world    
    def spawn(self, x=0, y=0, z=0, view=None):
        # Reset ant transform
        self.mesh_item.resetTransform()
        self.mesh_item.scale(self.scale_factor, self.scale_factor, self.scale_factor)
        self.position[:] = (x, y, z)
        self.mesh_item.translate(x, y, z)

        # Move circle if exists
        if self.detection_circle is not None:
            self.detection_circle.resetTransform()
            self.detection_circle.translate(x, y, 0)

        # Create circle only once if view is provided
        elif view is not None:
            self.detection_circle = create_circle(radius=10, x=x, y=y, z=0, color=(0.8,0.3,0.3,0.3))
            view.addItem(self.detection_circle)


    # Move in XY plane
    def move(self, dx, dy):
        self.position[0] += dx
        self.position[1] += dy
        self.mesh_item.translate(dx, dy, 0)
        if self.detection_circle is not None:
            self.detection_circle.translate(dx, dy, 0)


    # Rotate around Z-axis
    def rotate(self, angle_deg):
        self.rotation += angle_deg
        self.mesh_item.rotate(angle_deg, 0, 0, 1)  # Z-axis
