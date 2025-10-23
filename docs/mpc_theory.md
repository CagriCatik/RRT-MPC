# MPC Theory

The MPC module solves a discrete-time optimal control problem for the kinematic
bicycle model. The system state is \(x = [x, y, \psi, v]^T\) and the control
input is \(u = [a, \delta]^T\). Given a time step \(\Delta t\) and wheelbase
\(L\), the nonlinear dynamics are

\[
\begin{aligned}
x_{k+1} &= x_k + \Delta t\, v_k \cos(\psi_k),\\
y_{k+1} &= y_k + \Delta t\, v_k \sin(\psi_k),\\
\psi_{k+1} &= \psi_k + \Delta t\, \frac{v_k}{L} \tan(\delta_k),\\
v_{k+1} &= v_k + \Delta t\, a_k.
\end{aligned}
\]

The controller linearises the dynamics around the current reference window

\((\bar{x}_k, \bar{u}_k)\) to obtain

\[
X_{k+1} = A_k X_k + B_k U_k + c_k,
\]

where \(A_k = \partial f / \partial x\), \(B_k = \partial f / \partial u\) and
\(c_k = f(\bar{x}_k, \bar{u}_k) - A_k \bar{x}_k - B_k \bar{u}_k\). 

Forward Euler discretisation is sufficient at the 0.1 s time step used in this project.

## Optimisation Problem

At each time step the controller solves

\[
\min_{X,U,s} \sum_{k=0}^{N-1} \left[(X_k - X_k^{\mathrm{ref}})^T Q (X_k - X_k^{\mathrm{ref}}) + U_k^T R U_k\right] + (X_N - X_N^{\mathrm{ref}})^T Q_N (X_N - X_N^{\mathrm{ref}}) \\
+ w_v \sum_{k=0}^{N} s_{v,k}^2 + w_u \sum_{k=0}^{N-1} \lVert s_{u,k} \rVert_2^2 + w_{\Delta u} \sum_{k=0}^{N-1} \lVert s_{\Delta u,k} \rVert_2^2
\]

subject to

\[
\begin{aligned}
X_{k+1} &= A_k X_k + B_k U_k + c_k,\\
X_{0} &= x_0,\\
v_{\min} - s_{v,k} &\le v_k \le v_{\max} + s_{v,k},\\
u_{\min} - s_{u,k} &\le U_k \le u_{\max} + s_{u,k},\\
\Delta u_{\min} - s_{\Delta u,k} &\le U_k - U_{k-1} \le \Delta u_{\max} + s_{\Delta u,k},
\end{aligned}
\]

with all slack variables constrained to be non-negative. The slack weights are
chosen to keep the problem a quadratic program while prioritising feasibility:
\(w_v = 10^3\), \(w_u = w_{\Delta u} = 5\times 10^2\).

## Solver Configuration

OSQP is used with parameters tuned for fast convergence on the moderately sized
QP (horizon 12, 4 states, 2 inputs):

- `rho = 0.1`
- `alpha = 1.6`
- `adaptive_rho = True`
- `max_iter = 60000`
- `eps_abs = eps_rel = 10^{-3}`

If the solver reports infeasibility the pipeline reduces the reference velocity
by 40% and relaxes rate bounds once before aborting. This mirrors the behaviour
of the original prototype while keeping the logic testable.
