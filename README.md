# Gantry Crane Control: State Feedback vs. Feedback Linearization

Controller design for a gantry crane, built across two assignments for the Advanced Control Technology (ACT) course. The first assignment designs a linear state feedback controller with a Luenberger observer; the second replaces it with a non-linear feedback linearization controller and compares the two head-to-head under realistic actuator and safety limits.

## The system

A cart of mass M = 80 kg moves along a horizontal rail under an applied force F. A payload of mass m_L = 50 kg hangs from the cart on a rigid, massless cable of length L = 3 m, with gravity g = 9.8 m/s². The plant has two degrees of freedom, the cart position x_c and the swing angle θ, but only one actuator — the force on the cart. That makes it underactuated: the swing can't be commanded directly, only regulated indirectly through how the cart moves. Every acceleration of the cart excites the pendulum, so positioning the cart and damping the swing are coupled problems that have to be solved together.

## State feedback

The nonlinear equations of motion are linearized about the equilibrium, giving a state-space model that is marginally stable but fully controllable and fully observable. A pole-placement controller was tuned with ζ = 0.7, ωₙ = 2 rad/s for the dominant pair and faster poles at −3 and −4, producing K = [1175.5, 1508.6, −3743.5, 2173.7]. A Luenberger observer at {−6, −7, −8, −9} reconstructs the full state from position and angle measurements.

The controller tracks every target with no measurable steady-state error and damps the swing within about three seconds. However, the linear and nonlinear plants diverge on larger moves — on the 2 → 8 m move, peak swing reaches 39° on the true plant versus 36° predicted by the linear model, exactly the kind of gap you'd expect once the small-angle approximation breaks down.

## Feedback linearization

Instead of approximating the nonlinear dynamics, this controller cancels them directly: the applied force is chosen so the cart dynamics become an exact double integrator at any swing angle. The design also respects physical constraints the linear controller ignored — a rail limited to [0, 10] m, a swing safety limit of ±30°, and actuator saturation at ±2000 N.

The state feedback poles were too aggressive for this force budget, so the virtual input was detuned to ζ = 0.7, ωₙ = 1.5 rad/s with poles at −2.5 and −3.

FBL cart position | FBL swing angle | FBL control force
:---:|:---:|:---:
![FBL cart position](feedback_linearization/images/FBL%20Cart%20Position.png) | ![FBL swing angle](feedback_linearization/images/FBL%20Swing%20Angle.png) | ![FBL control force](feedback_linearization/images/FBL%20Control%20Force.png)

## Comparison

Both controllers were run on the same nonlinear plant, from the same initial state, to targets at 3 m, 5 m, and 8 m.

Cart position | Swing angle | Control force
:---:|:---:|:---:
![Cart position](comparison/images/Linear%20Vs%20FBL%20Cart%20Position.png) | ![Swing angle](comparison/images/Linear%20Vs%20FBL%20Swing%20Angle.png) | ![Control force](comparison/images/Linear%20Vs%20FBL%20Control%20Force.png)

| Move | Peak swing — linear | Peak swing — FBL | Change | Peak force — linear | Peak force — FBL | Change |
|---|---|---|---|---|---|---|
| 2→3 m | 5.95° | 3.58° | −39.8% | 1175.5 N | 413.3 N | −64.8% |
| 2→5 m | 18.13° | 10.66° | −41.2% | 3526.5 N | 1239.8 N | −64.8% |
| 2→8 m | 36.09° | 21.61° | −40.1% | 7053.1 N | 1602.6 N | −77.3% |

The feedback linearization controller cuts peak swing by roughly 40% and peak force by up to 77%, and is the only one that stays within the actuator and safety limits on every move. The linear controller exceeds the 2000 N force limit on the medium and large moves, and breaches the 30° swing limit on the large move. The trade-off is a modest increase in settling time on larger moves, since the FBL controller is deliberately detuned to stay within constraints.
