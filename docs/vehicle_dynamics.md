# Vehicle Dynamics

The package uses a kinematic single-track (bicycle) model expressed in pixel
coordinates. Mapping real-world wheelbase \(L\) in metres to pixels requires the
occupancy grid resolution \(r\) (metres per pixel), yielding \(L_{px} = L / r\).

## Continuous-Time Model

With state \(x = [x, y, \psi, v]^T\) and inputs \(u = [a, \delta]^T\), the
continuous dynamics are:

\[
\dot{x} = v \cos \psi, \quad
\dot{y} = v \sin \psi, \quad
\dot{\psi} = \frac{v}{L} \tan \delta, \quad
\dot{v} = a.
\]

The model assumes small slip angles (front-wheel steering only) and is suitable
for medium-speed manoeuvres typical of path tracking on occupancy grids.

## Discretisation

Forward Euler discretisation with time step \(\Delta t\) leads to the discrete
model implemented in `vehicle_model.f_discrete`:

\[
\begin{aligned}
x_{k+1} &= x_k + \Delta t\, v_k \cos \psi_k,\\
y_{k+1} &= y_k + \Delta t\, v_k \sin \psi_k,\\
\psi_{k+1} &= \psi_k + \Delta t\, \frac{v_k}{L} \tan \delta_k,\\
v_{k+1} &= v_k + \Delta t\, a_k.
\end{aligned}
\]

## Linearisation

To support MPC, Jacobians of the discretised model are required. Around an
operating point \((\bar{x}, \bar{u})\) the linearisation yields

\[
A = \frac{\partial f}{\partial x} = \begin{bmatrix}
1 & 0 & -\Delta t\, v \sin \psi & \Delta t \cos \psi\\
0 & 1 & \Delta t\, v \cos \psi & \Delta t \sin \psi\\
0 & 0 & 1 & \Delta t\, \frac{\tan \delta}{L}\\
0 & 0 & 0 & 1
\end{bmatrix},
\quad
B = \frac{\partial f}{\partial u} = \begin{bmatrix}
0 & 0\\
0 & 0\\
0 & \Delta t\, \frac{v}{L} \sec^2 \delta\\
\Delta t & 0
\end{bmatrix}.
\]

The affine term is computed as

\[
c = f(\bar{x}, \bar{u}) - A \bar{x} - B \bar{u},
\]

ensuring exact matching at the linearisation point. These matrices feed directly
into the MPC QP formulation.
