# Study TODO – Numerical Programming Retake (Written Part)

Study checklist for the **written exam** of the Numerical Programming retake. These are the mathematical and numerical methods you must know to justify your project and pass the written part.

---

## 1. Derivatives and their approximations

- [ ] **Finite differences**: forward, backward, central difference formulas
- [ ] **Order of accuracy** (e.g. O(h), O(h²))
- [ ] **Numerical differentiation** and stability
- [ ] Relation to discretization of ODEs (e.g. in path/spline context)

---

## 2. Edge detection

- [ ] **Purpose**: extract path/obstacle boundaries from maps or images
- [ ] **Classical methods**: gradient-based, Canny, Sobel (concepts and when to use)
- [ ] **Role in project**: getting path boundaries from the map (Subtask 1)

---

## 3. Splines

- [ ] **Parametric splines** (e.g. cubic splines) for path representation
- [ ] **Spline interpolation** vs approximation
- [ ] **Parametrization** of a path from A to B (arc length or other parameter)
- [ ] **Continuity** (C⁰, C¹, C²) and smoothness
- [ ] **Role in project**: path extraction and parametrization (Subtask 1)

---

## 4. IVP for ODEs (Initial Value Problems)

- [ ] **Formulation**: ẋ = f(t, x), x(0) = x₀
- [ ] **Existence and uniqueness** (e.g. Lipschitz condition)
- [ ] **Relation to robot/swarm dynamics**: position/velocity ODEs with initial conditions
- [ ] When the project uses IVP (e.g. robot motion with x(0), v(0))

---

## 5. BVP for ODEs (Boundary Value Problems)

- [ ] **Formulation**: ODE + conditions at two points (e.g. x(0) = x_A, x(T) = x_B)
- [ ] **Difference from IVP**: boundary conditions instead of only initial
- [ ] **Role in project**: path/trajectory with fixed start and end (e.g. A and B)

---

## 6. RK methods and Butcher’s table

- [ ] **Runge–Kutta (RK) methods**: idea and one-step formulation
- [ ] **Butcher’s table**: coefficients (a, b, c) and how to read them
- [ ] **Examples**: explicit Euler, Heun, classical RK4
- [ ] **Order of the method** (local vs global truncation error)
- [ ] **Role in project**: time integration of robot/swarm ODEs

---

## 7. Truncation error

- [ ] **Local truncation error** (LTE) definition
- [ ] **Global truncation error** and order of convergence
- [ ] **Consistency**: LTE → 0 as step size → 0
- [ ] How it relates to choice of RK (or other) method

---

## 8. A-stability

- [ ] **Stability of numerical methods** for ODEs
- [ ] **A-stability** definition: stable for all λ with Re(λ) ≤ 0 (test equation ẏ = λy)
- [ ] **Implicit vs explicit**: why implicit methods can be A-stable
- [ ] **Relevance**: stiff systems, damping, stable long-time integration

---

## 9. Shooting methods

- [ ] **Idea**: convert BVP into IVP + root-finding (e.g. find initial slope so x(T) = x_B)
- [ ] **Single and multiple shooting**
- [ ] **When to use**: BVPs (e.g. path from A to B with boundary conditions)

---

## 10. Numerical solution of linear systems of equations

- [ ] **Direct methods**: LU, Cholesky (when applicable)
- [ ] **Iterative methods**: Jacobi, Gauss–Seidel (basic idea)
- [ ] **Conditioning** and stability
- [ ] **Role**: spline systems, implicit time steps, shooting

---

## 11. Numerical solution of nonlinear systems of equations

- [ ] **Newton’s method** (and Newton–Raphson for systems)
- [ ] **Fixed-point iteration** and convergence
- [ ] **Role**: implicit schemes, shooting, collision-avoidance / force balance

---

## 12. Optical flow

- [ ] **Definition**: apparent motion of pixels between frames (velocity field)
- [ ] **Role in project**: extracting velocity field V(x, t) from pedestrian video (Subtask 3)
- [ ] **Basic approaches**: Lucas–Kanade, Horn–Schunck (concepts)
- [ ] **Saturated velocity field** V_sat and use in robot motion model

---

## Extra from the project text (good to review)

- [ ] **Velocity saturation**: v · min(1, v_max / ‖v‖)
- [ ] **Repulsive force** f_rep (e.g. inverse-square, safety radius R_safe)
- [ ] **Target tracking** T_i(t), **damping** k_d, **gains** k_p, k_v, k_rep
- [ ] **Formulating BVP/IVP** for each subtask and listing inputs (IC, BC, parameters)
- [ ] **Validation**: test cases, when the method works and when it does not

---

## Quick reference from the document

> *“For successfully solving the problem student must know the following: derivatives and their approximations, edge detection, splines, IVP for ODEs, BVP for ODEs, RK methods and Butcher's table, truncation error, A-stability, shooting methods, numerical solution of linear and nonlinear systems of equations, optical flow.”*

Use this list as your **written-exam study TODO**. Tick each topic when you can explain it and connect it to the project (path following, swarm, pedestrian flow).
