# Gantry Crane Control: State Feedback vs. Feedback Linearization

Controller design for a gantry crane, built across two assignments for the Advanced Control Technology (ACT) course. The first assignment designs a linear state feedback controller with a Luenberger observer; the second replaces it with a non-linear feedback linearization controller and compares the two head-to-head under realistic actuator and safety limits.

## The system

A cart of mass M = 80 kg moves along a horizontal rail under an applied force F. A payload of mass m_L = 50 kg hangs from the cart on a rigid, massless cable of length L = 3 m, with gravity g = 9.8 m/s². The plant has two degrees of freedom, the cart position x_c and the swing angle θ, but only one actuator — the force on the cart. That makes it underactuated: the swing can't be commanded directly, only regulated indirectly through how the cart moves. Every acceleration of the cart excites the pendulum, so positioning the cart and damping the swing are coupled problems that have to be solved together.

The task in both assignments is setpoint tracking with sway minimization: drive the cart to a target position with zero steady-state error, drive the swing angle back to zero, and do it quickly without letting the load swing too far.

## State feedback

The nonlinear equations of motion are derived from Newton's second law and implemented in Simulink as the reference plant. That model is then linearized about the equilibrium x* = [2, 0, 0, 0], u* = 0, giving the state-space matrices A, B, C, D used for the rest of the design.

The open-loop linearized plant turns out to be marginally stable (eigenvalues on the imaginary axis) but fully controllable and fully observable — meaning a single force can, in principle, regulate both the cart and the swing, and both unmeasured velocity states can be reconstructed from position and angle measurements alone.

A pole-placement controller was tuned with ζ = 0.7, ωₙ = 2 rad/s for the dominant pole pair, and faster non-dominant poles at −3 and −4, giving a gain of K = [1175.5, 1508.6, −3743.5, 2173.7]. The negative weight on the swing angle is the signature of an underactuated controller — it briefly pulls the cart back under a forward-swinging load instead of chasing it. A Luenberger observer was placed at {−6, −7, −8, −9}, four to six times faster than the controller, and reconstructs the full state from the two available measurements.

The controller drives the cart to target with no measurable steady-state error and damps the swing within about three seconds across all tested moves. But the linear and nonlinear plants only agree closely for small and medium moves — on the largest move (2 → 8 m), the true nonlinear plant swings to about 39°, against 36° predicted by the linear model. The gap grows with move size and lands entirely in the peak swing amplitude, not the settling time, which is exactly what you'd expect from the small-angle approximation breaking down.

**Files:** `state_feedback/matlab/` contains `Gantry_Crane.m` (nonlinear model + controller design), `Gantry_Crane_LinearModel.m` (linearization, open-loop analysis, observer design), and `plot_overlay.m` (comparison plots). The Simulink model is in `state_feedback/simulink/`, and simulation results and comparison plots are in `state_feedback/results/` and `state_feedback/images/`.

## Feedback linearization

The linear controller's accuracy is tied to the small-angle assumption, and it degrades for larger moves. This assignment fixes that by canceling the nonlinear terms directly instead of approximating them: the applied force is chosen so the cart dynamics become an exact double integrator, ẍ_c = v, valid at any swing angle, not just near the equilibrium. A virtual input v is then designed by the same pole-placement approach as before.

This time the design also has to respect physical constraints that the state feedback assignment ignored: a rail limited to [0, 10] m, a swing safety limit of ±30°, and an actuator saturation of ±2000 N (based on a 2 kW servo drive at roughly 1 m/s peak speed). The original state feedback poles turned out to be too aggressive for this force budget, so the virtual input was detuned to ζ = 0.7, ωₙ = 1.5 rad/s with non-dominant poles at −2.5 and −3, giving K̃ = [5.166, 8.610, −38.603, 3.029].

A Lyapunov analysis using total mechanical energy as the candidate function confirms the open-loop plant is stable in the Lyapunov sense but not asymptotically stable — consistent with the imaginary-axis eigenvalues from the state feedback analysis.

FBL cart position | FBL swing angle | FBL control force
:---:|:---:|:---:
![FBL cart position](feedback_linearization/images/FBL%20Cart%20Position.png) | ![FBL swing angle](feedback_linearization/images/FBL%20Swing%20Angle.png) | ![FBL control force](feedback_linearization/images/FBL%20Control%20Force.png)

**Files:** `feedback_linearization/matlab/` contains `Gantry_Crane_FBL.m` (controller design and simulation driver) and `Gantry_Crane_FBL_Plot.m` (result plots). The Simulink model is in `feedback_linearization/simulink/`, results in `feedback_linearization/results/`, and plots in `feedback_linearization/images/`.

## Comparison

Both controllers were run on the same nonlinear plant, from the same initial state, to the same three targets (2→3 m, 2→5 m, 2→8 m).

Cart position | Swing angle | Control force
:---:|:---:|:---:
![Cart position](comparison/images/Linear%20Vs%20FBL%20Cart%20Position.png) | ![Swing angle](comparison/images/Linear%20Vs%20FBL%20Swing%20Angle.png) | ![Control force](comparison/images/Linear%20Vs%20FBL%20Control%20Force.png)

Both controllers reach every target with negligible steady-state error, and the cart trajectories look almost identical — the real differences show up in swing and force.

| Move | Peak swing — linear | Peak swing — FBL | Change | Peak force — linear | Peak force — FBL | Change |
|---|---|---|---|---|---|---|
| 2→3 m (small) | 5.95° | 3.58° | −39.8% | 1175.5 N | 413.3 N | −64.8% |
| 2→5 m (medium) | 18.13° | 10.66° | −41.2% | 3526.5 N | 1239.8 N | −64.8% |
| 2→8 m (large) | 36.09° | 21.61° | −40.1% | 7053.1 N | 1602.6 N | −77.3% |

The feedback linearization controller cuts peak swing by roughly 40% and peak force by up to 77%, even though it runs on deliberately slower poles than the linear controller. More importantly, it's the only one that stays within the actuator and safety limits on every move — the linear controller exceeds the 2000 N force limit on both the medium and large moves, and exceeds the 30° swing limit on the large move. Under a realistic actuator rating, the linear controller simply isn't usable beyond the smallest move.

The one metric that favors the linear controller is settling time on the larger moves — the feedback linearization controller trades a bit of settling speed for staying inside its constraints.

**Files:** `comparison/LinearVsFBL_Comparison.m` (head-to-head comparison script), `comparison/Linear_Results.mat` and `comparison/FBL_Results.mat` (simulation data), `comparison/images/` (side-by-side plots).

## Repository structure

```
state_feedback/              Linear model, pole-placement controller, Luenberger observer
  ├── matlab/                MATLAB scripts (nonlinear model, linearization, observer design, plotting)
  ├── simulink/              Simulink model
  ├── results/               Simulation .mat file and linear-vs-nonlinear comparison plots
  └── images/                Simulink block diagrams and overlay plots

feedback_linearization/      Nonlinear feedback linearization controller
  ├── matlab/                MATLAB scripts (FBL controller design, result plotting)
  ├── simulink/              Simulink model
  ├── results/               Simulation .mat file
  └── images/                Model diagram and standalone result plots

comparison/                  Head-to-head Linear vs. FBL comparison
  ├── LinearVsFBL_Comparison.m
  ├── Linear_Results.mat
  ├── FBL_Results.mat
  └── images/                Side-by-side comparison plots
```

## Running it

Requires MATLAB with Simulink and the Control System Toolbox (`place`, `ctrb`, `obsv` are used directly).

- **State feedback:** open `state_feedback/simulink/Gantry_Crane_Model.slx`, then run `Gantry_Crane_LinearModel.m` (linearization, controller + observer design) or `Gantry_Crane.m` (nonlinear model with the same controller), followed by `plot_overlay.m` to reproduce the comparison plots.
- **Feedback linearization:** open `feedback_linearization/simulink/Gantry_Crane_FBL_Model.slx`, then run `Gantry_Crane_FBL.m` to design the controller and simulate all three target moves, and `Gantry_Crane_FBL_Plot.m` for individual result plots.
- **Comparison:** run `comparison/LinearVsFBL_Comparison.m` to reproduce the side-by-side comparison plots (requires both Simulink models to have been run first so the .mat files exist).
