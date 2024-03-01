# Hexapod Search and Rescue Robot

The Hexapod Search and Rescue Robot is designed to navigate various terrains and environments to aid in search and rescue operations. This project provides the codebase for controlling the movements and actions of the hexapod robot.

## Overview

The project consists of the following main components:

1. `core.py`: Contains the core functionality and classes for controlling the hexapod robot.
2. `pro.py`: Additional functionalities and advanced features for the hexapod robot.
3. `calibrate.py`: Calibration routines for setting up and configuring the hexapod's movements.
4. `main.py`: The main script to initialize and control the hexapod robot using the provided functionalities.

## Getting Started

To use the Hexapod Search and Rescue Robot codebase, follow these steps:

1. Clone the repository to your local machine:

git clone https://github.com/Varun-Ajith/Project-Hex-MALAYAN.git


2. Install any required dependencies. Refer to the `requirements.txt` file for details.

3. Import the necessary modules into your Python environment:

```python
from hexapod.core import Hexapod
hexapod = Hexapod()
hexapod.boot_up()


## Functionality
The `Hexapod` class provides various methods for controlling the movements and actions of the hexapod robot:
