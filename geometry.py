# create_cuboid, visual_frames (Drawing helpers)

import numpy as np
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import GLMeshItem, GLTextItem, GLLinePlotItem

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

def create_axis_numbers(axis, length, spacing=10.0):
    # Draws the numbers (10, 20, 30...)
    numbers = []
    positions = np.arange(spacing, length + spacing, spacing)
    for pos in positions:
        text = f'{pos:.0f}'
        # Position the text slightly offset from the axis
        if axis == 'x': position = [pos, -5, 0]
        elif axis == 'y': position = [-5, pos, 0]
        elif axis == 'z': position = [-5, 0, pos]
        
        label = GLTextItem(text=text, color=(255, 255, 255, 255)) # White text
        label.setData(pos=np.array(position))
        label.setGLOptions('translucent') # Fix for Linux transparency issues
        numbers.append(label)
    return numbers