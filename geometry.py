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


# Error Ellipse
def create_ellipse_item(covariance, x=0, y=0, confidence=0.95, color=(0, 1, 0, 1)):
    # 1. Get the 2x2 positional covariance
    cov_2d = covariance[:2, :2]
    
    # 2. Calculate Eigenvalues and Eigenvectors
    # Eigenvalues = length of axes, Eigenvectors = rotation of axes
    vals, vecs = np.linalg.eig(cov_2d)
    
    # 3. Calculate scale based on confidence (Chi-square value)
    # For 95% confidence in 2D, the scale factor is approx 2.447 * sqrt(eigenvalue)
    # or s = 5.991 (from chi-square table)
    s = 5.991 
    
    # Calculate radii
    # We use abs() just in case of tiny numerical noise making a value negative
    width = 2 * np.sqrt(s * np.abs(vals[0]))
    height = 2 * np.sqrt(s * np.abs(vals[1]))
    
    # 4. Generate points for a unit circle
    theta = np.linspace(0, 2 * np.pi, 40)
    circle_pts = np.array([np.cos(theta), np.sin(theta), np.zeros_like(theta)])
    
    # 5. Transform circle to ellipse
    # Scale -> Rotate -> Translate
    # Rotation matrix from eigenvectors
    R = np.eye(3)
    R[:2, :2] = vecs
    
    # Scaling matrix
    S = np.diag([np.sqrt(s * np.abs(vals[0])), np.sqrt(s * np.abs(vals[1])), 1.0])
    
    # Apply transformations: points = R @ S @ circle + [x, y, 0]
    ellipse_pts = (vecs @ S[:2, :2]) @ circle_pts[:2, :]
    ellipse_pts[0, :] += x
    ellipse_pts[1, :] += y
    
    # Format for PyQtGraph (N, 3)
    pts = np.column_stack([ellipse_pts[0, :], ellipse_pts[1, :], np.zeros(len(theta))])
    
    # Create the Line Item
    return gl.GLLinePlotItem(pos=pts, color=color, width=2, antialias=True)