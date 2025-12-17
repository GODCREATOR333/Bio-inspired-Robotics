import numpy as np
import pyqtgraph.opengl as gl


# --- Math Helper: Draw Axis Lines ---
def axis_line(start, end, color):  #Draw each co-ordiante axis line

    # Creates a simple line to visualize X, Y, Z axes
    pts = np.array([start, end])
    return gl.GLLinePlotItem(pos=pts, color=color, width=3, antialias=True)

## This circle is considered as detection radius
def create_circle(radius=10, segments=64, x=0, y=0, z=0, color=(1,0,0,1)):
    theta = np.linspace(0, 2*np.pi, segments)

    # base circle around origin
    verts = np.vstack([
        radius * np.cos(theta) + x,
        radius * np.sin(theta) + y,
        np.full_like(theta, z)
    ]).T

    # center vertex shifted to (x, y, z)
    center = np.array([[x, y, z]], dtype=np.float32)

    # combine vertices
    vertices = np.vstack([center, verts]).astype(np.float32)

    # triangle fan faces
    faces = []
    for i in range(1, segments):
        faces.append([0, i, i+1])
    faces.append([0, segments, 1])

    faces = np.array(faces, dtype=np.int32)

    item = gl.GLMeshItem(
        vertexes=vertices,
        faces=faces,
        smooth=False,
        drawEdges=False,
        color=color
    )

    return item



def create_sun(x=0, y=0, z=100, radius=8, ray_length=20, color=(1, 1, 0, 1)):
    # --- Sphere (sun core) ---
    sphere = gl.GLMeshItem(
        meshdata=gl.MeshData.sphere(rows=16, cols=32, radius=radius),
        smooth=True,
        color=color,
        shader='shaded',
        glOptions='opaque'
    )
    sphere.translate(x, y, z)

    # --- Rays ---
    rays = []
    directions = [
        (1, 0, 0), (-1, 0, 0),   # +X, -X
        (0, 1, 0), (0, -1, 0),   # +Y, -Y
        (0, 0, 1), (0, 0, -1),   # +Z, -Z
    ]

    for dx, dy, dz in directions:
        ray = gl.GLLinePlotItem(
            pos=np.array([
                [x, y, z],
                [x + dx * ray_length, y + dy * ray_length, z + dz * ray_length]
            ], dtype=float),
            color=color,
            width=2,
            antialias=True
        )
        rays.append(ray)

    return [sphere] + rays


def normalize_angle(angle):
    """Keeps angle between -pi and +pi"""
    return (angle + np.pi) % (2 * np.pi) - np.pi