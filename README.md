## Hexapod Search and Rescue Robot

The **Hexapod Search and Rescue Robot** is designed to navigate various terrains and environments, aiding in search and rescue operations. This project provides the codebase to control the robot's movements and actions.

## Overview

The project consists of these main components:

1. **core.py:** Contains core functionalities and classes for controlling the hexapod robot.
2. **pro.py:** Offers additional functionalities and advanced features for the robot.
3. **calibrate.py:** Provides calibration routines for setting up and configuring the robot's movements.
4. **main.py:** The main script to initialize and control the robot using provided functionalities.

## Getting Started

To use the Hexapod Search and Rescue Robot codebase:

1. **Clone the repository:**git clone https://github.com/Varun-Ajith/Project-Hex-MALAYAN.git
2. **Install dependencies:** Refer to the `requirements.txt` file for details.

3. **Import modules:**

```python
from hexapod.core import Hexapod

  hexapod = Hexapod()
  hexapod.boot_up()
```
## Functionality
The Hexapod class provides various methods for controlling the movements and actions of the hexapod robot:

- `boot_up()`: Boots up the hexapod by executing a sequence of predefined movements.
- `shut_down()`: Shuts down the hexapod by executing a sequence of predefined movements.
- Other methods for controlling specific movements such as walking, rotating, tilting, etc.

## Usage
You can use the provided functionalities to control the hexapod robot according to your requirements. Adjust the parameters of the methods as needed to customize the movements and actions of the robot.
# Example usage
hexapod.walk()
hexapod.rotate()
hexapod.tilt()


## Contributing
We welcome contributions! If you have ideas for improvements, new features, or bug fixes, open an issue or submit a pull request.

## Credits
All credits go to Varun Ajith, Ryan Biju Joseph, Pranav Jayan, and Emin Tomson.

## License
This project is licensed under the MIT License. See the LICENSE file for more information.
