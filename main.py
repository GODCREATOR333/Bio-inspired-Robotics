# Run Final Script

import sys
import os
from PyQt5 import QtWidgets


current_dir = os.path.dirname(os.path.abspath(__file__)) #
project_root = os.path.dirname(os.path.dirname(current_dir)) 
sys.path.append(project_root)


try:
    from engine import MainWindow
except ImportError as e:
    print("CRITICAL IMPORT ERROR: Could not find the modules.")
    print(f"Make sure you are running this script. Error details:\n{e}")
    sys.exit(1)


if __name__ == '__main__':
    # Create the Qt Application
    app = QtWidgets.QApplication(sys.argv)
    
    # Create the Physics Engine Window
    win = MainWindow()
    win.show()
    
    # Start the Event Loop
    print("Simulation Started successfully.")
    sys.exit(app.exec_())