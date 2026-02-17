# Project TODO – Numerical Programming Retake (3 subtasks)

Checklist for the **three project subtasks**. Use this with the Cursor TODO list.

---

## Subtask 1 – Follow a path in a constrained environment (5 points)

**Input**
- [ ] Open a map
- [ ] Choose two points A and B connected by a path
- [ ] Take a snapshot
- [ ] Assume map area is flat
- [ ] Assign a width to the path
- [ ] Assign a size to the robot

**Task**
- [ ] Extract the path from A to B
- [ ] Parametrize the path using splines
- [ ] Move a robot from A to B along the path without crossing path borders

**Output**
- [ ] Path with width defined
- [ ] Visualization of the robot following the path

**Written**
- [ ] Formulate suitable BVP or IVP for this sub-problem
- [ ] Describe inputs: initial/boundary conditions and parameters
- [ ] Describe numerical methods and justify the approach
- [ ] Add test cases and show validity of results

---

## Subtask 2 – Swarm of robots on a constrained path (5 points)

**Input**
- [ ] A path and its width
- [ ] A swarm of robots at A and a swarm at B

**Task**
- [ ] Swarm at A navigates from A to B
- [ ] Swarm at B navigates from B to A
- [ ] Both swarms start at the same time
- [ ] No collisions (autonomous, collision-free navigation)

**Output**
- [ ] Path with width defined
- [ ] Visualization of both swarms navigating without accidents

**Written**
- [ ] Formulate suitable BVP or IVP for the swarm motion
- [ ] Describe inputs: initial/boundary conditions and parameters
- [ ] Describe numerical methods (e.g. ODE integration, collision avoidance) and justify
- [ ] Add test cases and show validity of results

---

## Subtask 3 – Navigation in a pedestrian flow (5 points)

**Input**
- [ ] A pedestrian flow video (your choice)

**Task**
- [ ] A robot navigates safely in the pedestrian flow
- [ ] Navigation in at least two directions

**Output**
- [ ] Visualization of navigation including the robot and pedestrians

**Written**
- [ ] Formulate suitable model (e.g. velocity field, saturation, repulsion)
- [ ] Describe inputs and parameters (e.g. optical flow, v_max, R_safe)
- [ ] Describe numerical methods and justify
- [ ] Add test cases and show validity of results

---

## Submission checklist (all subtasks)

- [ ] **Presentation file**
- [ ] **Code**
- [ ] **Test data and description**
- [ ] **Visualization** (for each subtask as required)
- [ ] **Test cases**: where the approach works well and where it does not, with explanations of limitations
- [ ] **AI use**: if you used AI, mention it explicitly and provide all related details

---

## Cursor TODOs (same 3 items)

In Cursor you also have these three TODOs; mark them done as you finish each subtask:

1. **Subtask 1**: Path following in constrained environment (5 pts)
2. **Subtask 2**: Swarm A↔B on constrained path (5 pts)
3. **Subtask 3**: Robot in pedestrian flow (5 pts)
