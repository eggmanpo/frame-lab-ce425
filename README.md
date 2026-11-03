# FrameLab — CE425 2D Frame Analysis Web App

Interactive, static browser-based finite-element analysis for the assignment's ABC frame, with optional axial truss BD. **No MATLAB license is required to use the website.** The numerical engine is JavaScript, following the classical direct stiffness method.

## Important: the default numbers are illustrative

The initial geometry and rigidities are **fictional examples**, not the assignment's Table 1a. Replace them with the actual assigned values.

- Joint coordinates: A=(0,h), B=(L_AB,h), C=(L_AB+sqrt(L_BC²−h²),0), D=(0,0).
- A and C fixed; D pinned. Frame AB and BC rigidly joined at B; BD is a pin-connected, axial-only truss.
- Uniform load w downward on horizontal AB; P leftward at B.
- Units: m, kN, rad. EA in kN, EI in kN·m².
- The rotation placeholder at D allows a 6-DOF truss representation; its stiffness, end moment and reaction moment are zero.

## Running locally

Install Node.js 22+, then run:

```bash
npm test
npm start
```

Visit http://localhost:8000. The published application needs no installation.

## Deployment

The repository includes `.github/workflows/pages.yml`. Under **Settings → Pages**, set **Build and deployment → Source → GitHub Actions**. The workflow tests the solver and publishes `web/` following a push to `main`. Expected project URL: https://eggmanpo.github.io/frame-lab-ce425/ (only after a successful Pages deployment).

## Functionality

- Part 1a unbraced versus Part 1b braced comparison
- Interactive geometry/load inputs and magnified deformation diagram
- ID, IEN and LM tables
- 6×6 element local/global stiffness matrices and assembled global K
- B displacements, element local end forces, reactions and support moments
- Equilibrium and residual checks
- JSON and CSV exports

## Engineering limitations

First version is specialized for the assigned ABC/BD structural configuration, not arbitrary frame geometry. Uses Euler–Bernoulli small-deflection linear elasticity, no shear deformation, no plasticity, no second-order effects. Deformed shape is a magnified interpolation, not to scale. Numerical tests are not certification for professional engineering use. Verify the results against the course material and actual input table.
