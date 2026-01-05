# Simulation Realism Configuration

The simulation environment in Gazebo has been tuned to mimic high-fidelity physics engines (like O3DE) and real-world conditions. This allows for rigorous "stress-testing" of SLAM algorithms.

## Physics & Traction

### Friction Model
- **Coefficient of Friction (`mu`, `mu2`)**: Reduced from `100,000` (infinite grip) to **`1.0`** (rubber on concrete).
    - This allows for lateral sliding during sharp turns.
- **Wheel Slip (`slip1`, `slip2`)**: Increased from `0.0` to **`0.05`**.
    - **Effect**: Introducing micro-slippage causes wheel encoder odometry to drift significantly over distance, breaking the "perfect odom" assumption often seen in simulations.

## Sensor Noise Models

### Lidar (LDS-01)
- **Gaussian Noise**: Standard deviation increased from `0.01m` to **`0.02m`**.
    - This creates "fuzzy" walls, making scan matching more difficult and requiring robust probability grids.

### IMU (MEMS)
- **Angular Velocity Noise (Gyro)**: `stddev` increased 10x to **`2e-3`**.
- **Linear Acceleration Noise (Accel)**: `stddev` increased 10x to **`1.7e-1`**.
    - **Effect**: The robot's estimated orientation will drift rapidly if stationary or moving slowly without sensor fusion corrections. This specifically targets the internal robustness of Cartographer's IMU integration vs GMapping's reliance on odometry alone.

## Testing Impact
These changes ensure that benchmarks run on this setup are indicative of **real-world performance**. An algorithm that succeeds here is much more likely to succeed on physical hardware than one tuned for "perfect" Gazebo physics.
