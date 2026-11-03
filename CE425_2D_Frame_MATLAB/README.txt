CE425 INTRODUCTION TO FINITE ELEMENTS
2D FRAME ANALYSIS: PROBLEM 1, PARTS (a) AND (b)

Files:
- Main_FrameAnalysis.m : run this in MATLAB
- FrameInput.m         : edit Table 1 dimensions and rigidities here
- AnalyzeFrame.m       : ID, IEN, LM, global assembly, solution, recovery
- ElementMatrices.m    : 2D frame and truss local/global element matrices

INSTRUCTIONS
1. Put all four .m files in the same folder.
2. Open FrameInput.m.
3. Replace NaN placeholders with the actual TABLE 1 values:
   L_AB, L_BC in meters
   EA_AB, EA_BC, EA_BD in kN
   EI_AB, EI_BC in kN*m^2
   If E is listed in kN/m^2, A in m^2, and I in m^4:
       EA = E*A; EI = E*I.
   If E is given in GPa, convert to kN/m^2 by multiplying by 1e6.
4. The code assumes A and C are FIXED, as written in the assignment;
   D is pinned, and BD is a truss connected at B and D.
5. The figure gives the A-D vertical offset H=4 m. The code assumes
   C and D are on ground level and BC slopes downward to the right.
6. In MATLAB, set Current Folder to this folder and run Main_FrameAnalysis.
7. Outputs include ID, IEN, LM, element local/global stiffness, global K,
   F, nodal displacements, local member end forces, support reactions,
   equilibrium checks, and support-moment comparison.

LOADS & SIGNS
+X right; +Y up; +moment counterclockwise.
w=12 kN/m vertically down on AB; P=50 kN horizontally left at B.
The distributed load is represented by CONSISTENT nodal loads. It is
subtracted when recovering local element end forces.

DOFs
Frame: [ux1 uy1 rz1 ux2 uy2 rz2]
Truss:  same 6-position storage, but zero rotational and shear stiffness.
D has no rotational DOF; ID(D,rz)=0 (inactive, not a fixed moment support).
ID positive: free; negative: constrained; zero: inactive.

NOTE
The provided assignment screenshot refers to TABLE 1 but does not show
its numerical values. This is a ready-to-complete solver, not a claim
that numerical homework results have been calculated. No values are
silently invented. The program deliberately stops if NaNs remain.
