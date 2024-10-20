# WRO Future Competition Simulation Environment

This repository contains the simulation environment for the WRO (World Robot Olympiad) Future competition. The environment is modeled with actual size and dimensions, making it an ideal setup for testing robot performance and strategies in a simulated space. The simulation is created using **Gazebo Harmonic** and **ROS 2 Humble**.

## Features

- **Accurate Competition Layout**: The environment model (`.sdf` file) represents the real-world competition space with exact measurements.
- **Real-Time Simulation**: Test your robotics solutions in real-time using ROS 2 Humble with the Gazebo Harmonic simulator.
- **Pre-configured Setup**: A plug-and-play environment that allows users to quickly get started with testing and simulation.

## Requirements

- **ROS 2 Humble**: Follow the instructions [here](install_humble.sh) to install ROS 2 Humble.
- **Gazebo Harmonic**: You can install Gazebo Harmonic by following the official [installation guide](install_gz_harmonic.sh).
- **SDF Model**: The environment is defined using an SDF model file, compatible with Gazebo Harmonic.


## Installation

1. Clone the repository:

   ```bash
   git clone https://github.com/khaledgabr77/WRO-Future-Simulation.git
   ```

## Requirments 

1. **sdformat_urdf**

    ```bash
    git clone -b humble https://github.com/ros/sdformat_urdf.git
    ```
2. **ros_gz**
    ```bash
    https://github.com/gazebosim/ros_gz
    ```
3. **GZ_SIM_RESOURCE_PATH**
    ```bash
    export GZ_SIM_RESOURCE_PATH="PATH/models"
    ```

## Run Simulation

```bash
ros2 launch mrbuggy3_gz_bringup robot_sim.launch.py
```

![alt text](<Screenshot from 2024-10-19 16-48-58.png>)

## Contributing

Contributions are welcome! Please follow these steps:

1. Fork the repository.
2. Create a new branch (`git checkout -b feature/YourFeature`).
3. Commit your changes (`git commit -m 'Add some feature'`).
4. Push to the branch (`git push origin feature/YourFeature`).
5. Open a Pull Request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

