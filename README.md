# Sheet Chassis Studio

A browser app for designing **sheet-metal car chassis**. You get a parametric monocoque, a shell/beam finite-element solver, double-wishbone kinematics and a set of optimisers. It's built on **three.js r186 `WebGPURenderer`**, which falls back to WebGL2 automatically, and it has an optional **WebGPU compute** FE solver.

```bash
npm install
npm run dev        # http://localhost:5173  (add ?webgl to force the WebGL2 backend)
npm test           # FEA verification, kinematics and optimiser tests (node --test)
npm run build      # static bundle in dist/
```

## What's inside

| Area | Features |
|---|---|
| **Presets** | Mid-engine **supercar** (bonded aluminium tub, tunnel, scuttle box, front towers, rear rails, cage, engine-bay X-brace) · **single seater** (tapered sheet tub, closed footwell deck, front and main hoops, stressed engine + gearbox) · **EV** skateboard (structural battery enclosure with cross-members and spine, crash rails, roof ring) |
| **Chassis geometry** | Stations, tub section, sills, tunnel, deck, battery box, rails, hoops/cage/braces, plan and height taper, pick-up bracket footprint, sheet material and gauge per panel group, tube sizes |
| **Meshing** | Conforming quad mesh. Every panel is built on shared global grid lines and then mapped to the tapered shape, so T-joints, folds and bulkheads always share nodes. Tubes are beam elements. Suspension pick-ups are tied in through rigid "bracket" spiders |
| **FEA** | 4-node flat shell (Q4 membrane with incompatible modes + MITC4 Mindlin plate + Hughes–Brezzi drilling), 3-D frame beams, RCM-ordered envelope Cholesky (f64) in a Web Worker. **Torsion** (rear pick-ups pinned, pure torque through the front pick-ups), **bending** (reuses the torsion factorisation, with Lagrange-multiplier supports), **free-free modal** (shift-invert Lanczos). Contours for displacement, von Mises and strain-energy density, a twist distribution along the tub, strain-energy share per panel group, lightweight index |
| **WebGPU compute** | Optional solver: node-block-Jacobi preconditioned CG in WGSL (f32), wrapped in f64 iterative refinement on the CPU |
| **Suspension** | Full 3-D double-wishbone solve for heave, roll and steer. Direct coil-over, or push/pull-rod with rocker. Camber, toe/**bump steer**, KPI, caster, scrub, trail, front-view IC and **roll centre** (height and migration in roll), roll camber compensation, track change, **motion ratio** and progression, side-view IC with **anti-dive / anti-squat / anti-lift**, **Ackermann** and steering ratio. Live 3-D pose with IC/RC construction lines. Draggable hardpoints |
| **Vehicle** | Mass, CG and inertias (FE structure plus components and unsprung mass), ride frequencies, wheel rates, roll gradient, **LLTD**. Chassis-vs-suspension roll-stiffness ratio, with a twin-spring model of how chassis flex shifts LLTD |
| **Optimisers** | Suspension geometry (bounded Nelder–Mead over chosen hardpoint coordinates; studies for bump steer, roll centre and camber, anti-geometry, Ackermann, motion ratio) · **sheet gauge sizing** for minimum mass at a torsional-stiffness target (optimality criteria with analytic membrane/bending sensitivities, then snapped to standard gauges) · ride and roll set-up (springs, ARBs, dampers) · weight distribution · geometry design sweeps |
| **Views & export** | 3D or 4-view (front / side / plan / perspective), X-ray, FE mesh, mode animation, deformed shape. Export design JSON, sheet **cut list CSV**, hardpoints CSV, **Nastran .bdf**, PNG |

## Conventions

- x points forward from the front axle (the rear axle is at `-wheelbase`), y points left, z points up from the ground. Units are mm, N, MPa and kg.
- Suspension hardpoints are entered for the left side, with x measured from their own axle. The right side is mirrored.
- The torsion test measures chassis-only stiffness: the suspension is locked and the load goes in at the pick-ups. The bracket-footprint radius controls how locally the pick-up loads enter the sheet.

## Verification (`npm test`)

- A plate-bending cantilever and an in-plane cantilever match beam theory to within 3–5 %.
- A closed square tube made of shells matches **Bredt–Batho** torsion to within 5 %.
- A beam cantilever is exact, and free-free beam modes match the analytic value to within 8 % (lumped mass).
- Kinematic closure (link lengths) holds through travel and steer. Parallel equal-length arms give zero camber change and zero bump steer.
- The bump-steer optimiser removes an injected error, and every preset meets its own ride and LLTD targets.

## Layout

```
src/fea/         elements, envelope solver, assembly/post, modal, load cases, WebGPU PCG, worker
src/chassis/     parametric sheet-metal mesher
src/suspension/  double-wishbone kinematics
src/vehicle/     presets, mass properties, ride/roll/LLTD
src/optim/       Nelder–Mead, suspension studies, gauge sizing + sweeps
src/render/      three.js WebGPU viewer, chassis + suspension views
src/app, src/ui  worker client, exporters, DOM + chart helpers
```
