# Wingsuit Kalman Filter

A 12-state Extended Kalman Filter for tracking wingsuit dynamics during skydiving.

## Features

- Tracks position, velocity, acceleration in ENU coordinates
- Estimates wingsuit aerodynamic parameters (lift coefficient, drag coefficient, roll angle)
- Uses physics-based prediction with wingsuit equations
- Adaptive measurement noise based on GPS accuracy
- Implicit integration for numerical stability

## State Vector
[x, y, z, vx, vy, vz, ax, ay, az, kl, kd, roll]
Where:
- `x, y, z`: Position in ENU coordinates (meters)
- `vx, vy, vz`: Velocity in ENU coordinates (m/s)
- `ax, ay, az`: Acceleration in ENU coordinates (m/s²)
- `kl`: Lift coefficient (dimensionless)
- `kd`: Drag coefficient (dimensionless)
- `roll`: Roll angle (radians)

# FlySight Plotter

To run the app locally, first install node.js. Then, in the project directory, run:

```
npm install
npm run dev
```

And open http://localhost:5173 in your browser.
