import numpy as np
from stl import mesh
import pyqtgraph.opengl as gl


class Food_Model:
    def __init__(self, stl_path, scale=0.01):
        # Load STL
        self.raw_mesh = mesh.Mesh.from_file(stl_path)

        # Flatten vectors
        self.vertices = np.array(self.raw_mesh.vectors.reshape(-1, 3), dtype=np.float32)

        # Center mesh around origin
        center = self.vertices.mean(axis=0)
        self.vertices -= center

        # Faces
        self.faces = np.arange(len(self.vertices)).reshape(-1, 3)

        # Make GLMeshItem
        self.mesh_item = gl.GLMeshItem(
            vertexes=self.vertices,
            faces=self.faces,
            smooth=False,
            drawEdges=True,
            edgeColor=(0, 0, 1, 1),   # blue edges
            color=(0, 1, 0.1, 1), # reddish-orange dead bug
        )

        # Scale
        self.mesh_item.scale(scale, scale, scale)
        self.mesh_item.rotate(-90, 0, 1, 0)

    def spawn(self, x, y, z=0):
        self.mesh_item.translate(x, y, z)
