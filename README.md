Goal: Generate a collision-free geometric path with RRT (or RRT*), convert it to a smooth, time-parameterized trajectory, and track it in real time with an MPC using a dynamic bicycle model.

Assumptions:

* Vehicle with ackerman steering
* Known static map as occupancy grid or polygonal obstacles.
* Discrete-time MPC at 20 to 100 Hz.
* Nonholonomic bicycle model.

Pipeline overview:

1. Build a planning map.
2. Plan a geometric path with RRT/RRT*.
3. Post-process: prune, shortcut, smooth to a spline.
4. Time-parameterize from spatial path to t -> {x,y,psi,v,a,delta}.
5. Build an MPC with a kinematic or dynamic bicycle model, linearize on the trajectory, solve at each control step.
6. Apply only the first control from the MPC, then repeat (receding horizon).
7. Replan on significant deviation or obstacle change.

Step 1: Map and state

* Input: occupancy grid or obstacle polygons O = {O_i}, start state s0 = (x0,y0,psi0), goal region G.
* Collision check: point or footprint (circle/rectangle) inflated by safety margin r.

Step 2: RRT or RRT*

* State sampled in SE(2) or R2. For speed, sample in R2 for nodes; store heading as path tangent later.
* Steering function: connect q_near -> q_new with straight line in R2, step size eta, reject if collides.
* Bias: with prob p_goal, sample q_rand := goal centroid to speed convergence.
* Stop when q_new falls inside goal region or max iterations.

Minimal RRT pseudocode:

```
V = {q_start}; E = {}
for k in 1..K:
  q_rand = sample_free_with_goal_bias()
  q_near = argmin_{q in V} ||q - q_rand||
  q_new = steer(q_near, q_rand, step=eta)
  if collision_free(q_near, q_new):
    V.add(q_new); E.add((q_near, q_new))
    if in_goal(q_new): break
path = backtrack_to_start(E, q_new)
```

Use RRT* if you need lower curvature and shorter paths:

* Neighborhood radius r_neigh ~ gamma * sqrt(log(|V|)/|V|).
* Choose parent with minimal cost-to-come.
* Rewire neighbors whose cost improves via q_new.

Step 3: Path post-processing

* Vertex pruning: remove redundant waypoints using line-of-sight checks.
* Shortcutting: repeatedly try random chord replacements if collision-free.
* Fit C2-continuous spline (e.g., cubic or clothoid). Output s -> (x(s), y(s)), s in [0, L].
* Compute curvature kappa(s) from spline for speed limits.

Step 4: Time parameterization

* Speed bound from curvature and limits:
  v_max_curve(s) = sqrt(a_lat_max / max(|kappa(s)|, kappa_eps))
* Also limit by v_max_sys, a_long_max, jerk if needed.
* Forward-backward pass to enforce acceleration limits and stop at goal:
  Forward: v[i] = min(v_max_curve[i], sqrt(v[i-1]^2 + 2*a_long_max*ds))
  Backward: v[i] = min(v[i], sqrt(v[i+1]^2 + 2*a_long_brake*ds))
* Integrate dt[i] = ds / v[i], get t grid and reference trajectory:
  r(t) = {x(t), y(t), psi(t) = atan2(y'(s), x'(s)), v(t)}

Step 5: Vehicle model for MPC
Choose kinematic bicycle for low speeds; dynamic bicycle for higher fidelity. Kinematic discrete-time (sample time Ts):
States x = [X, Y, psi, v], inputs u = [a, delta].
Parameters: Lf, Lr (wheelbase L = Lf + Lr).

Continuous:
Xdot   = v*cos(psi)
Ydot   = v*sin(psi)
psidot = v/L * tan(delta)
vdot   = a

Discretize with RK1 or RK4 to get x_{k+1} = f(x_k, u_k).

Constraints:

* v_min <= v <= v_max
* |delta| <= delta_max, |d_delta| <= ddelta_max
* a_min <= a <= a_max

Cost (tracking in Frenet or Cartesian):
J = sum_{k=0..N-1} [ w_e*||p_k - p_ref_k||^2 + w_psi*angle_wrap(psi_k - psi_ref_k)^2
+ w_v*(v_k - v_ref_k)^2
+ w_u*||u_k||^2 + w_du*||u_k - u_{k-1}||^2 ].

Linearization for QP MPC (real-time friendly):
At each control step:

* Take local ref (x_ref_k, u_ref_k) along the timed trajectory.
* Linearize: x_{k+1} ~= A_k x_k + B_k u_k + c_k.
* Formulate convex QP over horizon N with quadratic cost and box constraints.

Step 6: MPC loop
At control rate:

1. Read current state x_meas (from localization).
2. Find closest index on reference s_star or t_star, extract window of length N.
3. Build linearized model and QP matrices for that window.
4. Solve QP, get optimal control sequence U* = {u_0..u_{N-1}}.
5. Apply u_0 to actuators.
6. Advance one step; repeat.

Step 7: Handling heading and nonholonomy

* If RRT was in R2, compute psi_ref from spline tangent to honor nonholonomic tracking in MPC.
* Consider adding curvature or steering-rate penalties to smooth commands.

Step 8: Replanning and robustness

* Trigger replanning if:

  * Reference path becomes occupied.
  * Cross-track error or heading error exceed thresholds.
  * Predicted collision with moving obstacle within horizon.
* Warm start MPC with previous solution shifted by one step.
* Add soft constraints on tracking error with slack penalties to avoid infeasibility.

Step 9: ROS2 integration outline
Nodes:

* map_server: publishes occupancy grid.
* rrt_planner: subscribes to map, start, goal; publishes nav_msgs/Path (smoothed).
* time_param: subscribes Path; publishes trajectory as nav_msgs/Path with stamped poses and velocities.
* mpc_controller: subscribes trajectory and localization; publishes geometry_msgs/Twist or low-level commands.
* vehicle_interface: converts to actuator commands.

Topics:

* /map: nav_msgs/OccupancyGrid
* /plan: nav_msgs/Path (RRT result)
* /traj: custom msg or path with velocity profile
* /cmd: geometry_msgs/Twist or custom control

Step 10: Minimal code skeletons (Python-like, library-agnostic)

RRT (collision check against grid):

```
def rrt(start, goal, is_free, sample, eta, K):
    V = [start]; parent = {start: None}
    for _ in range(K):
        q_rand = goal if np.random.rand() < 0.1 else sample()
        q_near = min(V, key=lambda q: np.linalg.norm(q - q_rand))
        dir = q_rand - q_near
        q_new = q_near + eta * dir / (np.linalg.norm(dir) + 1e-9)
        if collision_free(q_near, q_new, is_free):
            V.append(q_new); parent[tuple(q_new)] = tuple(q_near)
            if in_goal(q_new, goal): return backtrack(parent, q_new)
    return None
```

Path smoothing and spline:

```
P = prune(path, is_free)
P = shortcut(P, is_free, iters=500)
s, spl = arclength_parametric_spline(P)  # returns s grid and x(s), y(s)
kappa = curvature(spl, s)
```

Time parameterization:

```
v = np.minimum(v_sys_max*np.ones_like(s), np.sqrt(a_lat_max/np.maximum(np.abs(kappa), 1e-3)))
for i in range(1,len(s)):
    ds = s[i]-s[i-1]
    v[i] = min(v[i], np.sqrt(v[i-1]**2 + 2*a_long_max*ds))
for i in range(len(s)-2,-1,-1):
    ds = s[i+1]-s[i]
    v[i] = min(v[i], np.sqrt(v[i+1]**2 + 2*a_long_brake*ds))
t = np.zeros_like(s)
for i in range(1,len(s)):
    ds = s[i]-s[i-1]
    t[i] = t[i-1] + ds/max(v[i], 0.1)
traj = sample_xy_psi_v_over_time(spl, s, t, v)
```

MPC setup (linear time-varying QP form):

```
# At each step
ref = window(traj, t_now, N, Ts)
A,B,c = linearize_bicycle_along_ref(ref, Ts, L)
# Build QP matrices H, f, G, h for cost and constraints
u_opt = solve_qp(H, f, G, h)
apply(u_opt[0])
```

Key tuning guidelines:

* RRT: step eta ~ 1 to 3 grid cells; goal bias 5 to 20 percent; use obstacle inflation >= robot radius.
* Spline: keep curvature bounded; reject knots that raise |kappa| beyond limits.
* Time profile: a_lat_max 1.5 to 3.0 m/s^2 for small robots, 0.5 to 1.5 for larger vehicles.
* MPC: horizon N 10 to 40, Ts 0.02 to 0.1 s; weight ratios typically w_e:w_psi:w_v ~ 10:5:2, w_u small, w_du for smoothness.
* Constraints: enforce steering slew to avoid chatter; add slack with high penalty to maintain feasibility.

Edge cases:

* Narrow passages: use RRT* with informed sampling (ellipse between start and goal) or add bridge sampling.
* Dynamic obstacles: keep RRT for global path; handle local avoidance in MPC via predicted obstacle states and linearized separating constraints; replan globally at lower frequency.
* Poor localization: increase tracking slack and reduce v_ref; add delay compensation in model if actuation latency is nontrivial.

Verification checklist:

* Path is collision-free with inflated obstacles.
* Curvature of smoothed path respects steering limits: |kappa| <= tan(delta_max)/L.
* Velocity profile respects both lateral and longitudinal limits.
* MPC QP remains feasible under worst-case errors via slack.
* Closed-loop sim shows bounded cross-track error and no collisions.

End.
