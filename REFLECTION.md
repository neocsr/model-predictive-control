# MPC Controller

## The Model

The model of the system consists in:

- 4 state variables (x, y, psi and v)
- 2 state errors variables (cte and epsi)
- 2 actuators (delta and a)

`delta` and `a` are the actuators mapped to `steering angle` and `throttle` respectively.

For an N of 10, we have 158 variables for the solver:

    size_t n_vars = N * 6 + (N - 1) * 2;
    // N = 10 => 20 * 6 + 19 * 2 = 120 + 38 = 158

On every update received from the simulator, the state is updated using a vehicle model to include the latency. A third degree polynomial is built from the waypoints and passed to the optimization solver. The `a` and `delta` are returned from the solver and those are sent back to the simulator controls as `throttle` and `steering angle`.

The most important part was to choose the right values of `N`, `dt` and the weights for each of the constrains in the solver.

The constraints for the error ended up being the most important:

    // Very important to constrain errors
    const int w_cte = 2500;
    const int w_epsi = 2500;

The other important constraint was to minimize jerk in the maneuvering:

    // Important to minimize gap between sequential actuators to reduce jerk
    const int w_delta_diff = 200;
    const int w_a_diff = 10;

## Timestep Length and Elapsed Duration (N & dt)

`N` was chosen to account 1 second as prediction horizon with steps of `dt` of 100ms. I tried with higher values of `N`, and the prediction horizon becomes longer (longer green line) but there was no benefit in the maneuvering performance.


## Polynomial Fitting and MPC Preprocessing

A third degree polynomial was fitted using the waypoints previously transformed to vehicle coordinates. The transfomation is not really needed for the polynomial fitting, but it was already done to update the state variables for the car.

    auto coeffs = polyfit(x_car_ref, y_car_ref, 3);

The transformation of coordinates helps to simplify the equations to update the state variables:

    // Calculate variables after 100ms in vehicle coordinates
    const double Lf = 2.67;
    const double dt = 0.1;
    double delta = -steer_value;
    double xf = v * dt;
    double yf = 0;
    double psif = (v/Lf) * delta * dt;
    double vf = v;

    // Estimated errors from the state after 100ms
    double cte = polyeval(coeffs, xf);
    double epsi = psif - atan(coeffs[1] +
                              2 * coeffs[2] * xf +
                              3 * coeffs[3] * xf * xf);

Then those are passed to the MPC model:

    auto ok = mpc.Solve(state, coeffs);


## Model Predictive Control with Latency

The latency is taken care the the state updates in position and steering angle predicted after an estimate of 100ms with a simulated delay:

    const double dt = 0.1;
    ...
    double xf = v * dt;
    ...
    double psif = (v/Lf) * delta * dt;
    ...

    ...
    this_thread::sleep_for(chrono::milliseconds(100));


# Video

[Self-Driving Car - Model Predictive Control](https://www.youtube.com/watch?v=Gn3oelnTH9o)
