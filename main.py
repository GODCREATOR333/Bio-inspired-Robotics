# Run Final Script

import sys
import os
from PyQt5 import QtWidgets

# --- 1. PATH SETUP ---
# We need to add the project root directory to Python's path
# so we can do imports like "from Scanner_Sim_py.simulation..."
current_dir = os.path.dirname(os.path.abspath(__file__)) # .../Scanner_Sim_py/Run_Simulation
project_root = os.path.dirname(os.path.dirname(current_dir)) # .../ (The folder containing Scanner_Sim_py)
sys.path.append(project_root)

# --- 2. IMPORT THE ENGINE ---
# Now Python can find the package
try:
    from engine import MainWindow
except ImportError as e:
    print("CRITICAL IMPORT ERROR: Could not find the modules.")
    print(f"Make sure you are running this script. Error details:\n{e}")
    sys.exit(1)

# --- 3. RUN THE APP ---
if __name__ == '__main__':
    # Create the Qt Application
    app = QtWidgets.QApplication(sys.argv)
    
    # Create the Physics Engine Window
    win = MainWindow()
    win.show()
    
    # Start the Event Loop
    print("Simulation Started successfully.")
    sys.exit(app.exec_())