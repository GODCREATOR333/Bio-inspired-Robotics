# The MyView Class (Camera logic)

import pyqtgraph.opengl as gl
from PyQt5 import QtGui
from PyQt5.QtCore import Qt

# Custom View for Zoom/PAN control


class MyView(gl.GLViewWidget):
    def __init__(self):  # Construtor
        # Inheritance: Augment the actually gl.GLViewWidget with my extra code without overwriting
        super().__init__()


        self.zoomFactor = 0.9  # smaller = stronger zoom
        self.currentDist = 300
        self.panStep = 5
        self.default_pos = self.cameraPosition()

        self.default_center = QtGui.QVector3D(
            self.opts['center'].x(),
            self.opts['center'].y(),
            self.opts['center'].z()
        )

        # Sets the "Look At" point to the world origin (0,0,0)
        self.opts['center'] = QtGui.QVector3D(0, 0, 0)

        # Sets the Camera position using Spherical Coordinates
        # Distance: How far back the camera is
        # Elevation: Angle up from the ground
        # Azimuth: Angle around the Z-axis
        self.setCameraPosition(distance=self.currentDist,
                               elevation=90, azimuth=0)

        # Makes things look smaller when further away
        self.opts['projection'] = 'perspective'
        self.setWindowTitle("Bio-Inspired Robotics")

        # For key event triggering
        self.setFocusPolicy(Qt.StrongFocus)
        self.setFocus()

    # Custom key binding to zoom-in and zoom-out and Pan
    def keyPressEvent(self, ev):
        key = ev.key()
        c = self.opts['center']

        # --- ZOOM ---
        if key == Qt.Key_I:
            self.currentDist *= self.zoomFactor
            self.setCameraPosition(distance=self.currentDist)
            return

        if key == Qt.Key_O:
            self.currentDist /= self.zoomFactor
            self.setCameraPosition(distance=self.currentDist)
            return

        # --- PAN (WASD) ---
        if key == Qt.Key_W:
            c.setX(c.x() - self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_S:
            c.setX(c.x() + self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_A:
            c.setY(c.y() - self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_D:
            c.setY(c.y() + self.panStep)
            self.opts['center'] = c
            self.update()
            return

        if key == Qt.Key_R:
            # Reset to the true original defaults
            self.setCameraPosition(pos=self.default_pos)

            # center is a QVector3D so copy via constructor
            self.opts['center'] = QtGui.QVector3D(
                self.default_center.x(),
                self.default_center.y(),
                self.default_center.z()
            )

            self.update()
            return
        

        if key == Qt.Key_Up:
            self.main.agent.move(0, +5)
            return
        if key == Qt.Key_Down:
            self.main.agent.move(0, -5)
            return
        if key == Qt.Key_Left:
            self.main.agent.move(-5, 0)
            return
        if key == Qt.Key_Right:
            self.main.agent.move(+5, 0)
            return



        super().keyPressEvent(ev)
