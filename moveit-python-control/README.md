# MoveIt Python Control

This project provides a set of Python scripts to interact with the MoveIt simulation environment. It allows users to add objects to the simulation and perform path planning for robotic movements.

## Project Structure

```
moveit-python-control
├── src
│   ├── add_objects.py        # Script to add objects to the MoveIt simulation
│   ├── path_planning.py      # Script for path planning functionalities
│   └── utils
│       └── __init__.py       # Initialization file for the utils package
├── requirements.txt          # Python dependencies for the project
├── Dockerfile                 # Instructions to build the Docker image
└── README.md                  # Documentation for the project
```

## Installation

1. Clone the repository:
   ```
   git clone <repository-url>
   cd moveit-python-control
   ```

2. Build the Docker image:
   ```
   docker build -t moveit-python-control .
   ```

3. Run the Docker container:
   ```
   docker run -it moveit-python-control
   ```

## Usage

### Adding Objects

To add objects to the MoveIt simulation, use the `add_objects.py` script. This script provides functions to create and place objects in the simulation environment.

### Path Planning

The `path_planning.py` script implements the `PathPlanner` class, which includes methods for planning and executing robot movements. You can create an instance of `PathPlanner` and call its methods to perform path planning tasks.

## Requirements

Ensure that the following dependencies are included in the `requirements.txt` file:
- MoveIt libraries
- Any other necessary Python packages

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.