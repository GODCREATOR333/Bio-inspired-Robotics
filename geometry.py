# create_cuboid, visual_frames (Drawing helpers)

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem, GLTextItem, GLLinePlotItem,MeshData
from PIL import Image, ImageDraw, ImageFont

# --- Math Helper: Draw Axis Lines ---
def axis_line(start, end, color):  #Draw each co-ordiante axis line

    # Creates a simple line to visualize X, Y, Z axes
    pts = np.array([start, end])
    return gl.GLLinePlotItem(pos=pts, color=color, width=3, antialias=True)

# --- Math Helper: Create Axis Ticks ---
def create_axis_ticks(axis, length, spacing=10.0, tick_size=2.0):
    # Draws small tick marks along the axis
    ticks = []
    positions = np.arange(spacing, length + spacing, spacing)
    for pos in positions:
        if axis == 'x': start, end = [pos, -tick_size / 2, 0], [pos, tick_size / 2, 0]
        elif axis == 'y': start, end = [-tick_size / 2, pos, 0], [tick_size / 2, pos, 0]
        elif axis == 'z': start, end = [0, -tick_size/2, pos], [0, tick_size/2, pos]
        
        # Color is white with some transparency
        ticks.append(GLLinePlotItem(pos=np.array([start, end]), color=(1,1,1,0.5), width=1))
    return ticks

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
    """
    Create a sun-like global reference: a sphere + radial rays.
    """

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
