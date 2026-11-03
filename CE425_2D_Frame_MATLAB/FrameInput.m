function S = FrameInput(caseNumber)
% FRAMEINPUT  Model for CE425 Problem 1: frame ABC, optionally truss BD.
% Units: kN, m, rad. EI = kN*m^2; EA = kN.
% Node order: 1=A, 2=B, 3=C, 4=D (only in Part b).
% Element order: 1=AB, 2=BC, 3=BD (only in Part b).
%
% ***** REPLACE EVERY NaN WITH YOUR ASSIGNMENT TABLE 1 VALUES *****
% If Table 1 gives E, A, I separately, use EA = E*A, EI = E*I.

L_AB  = NaN;  % m, horizontal span AB
L_BC  = NaN;  % m, sloping member BC length
EA_AB = NaN;  % kN, axial rigidity of frame AB
EI_AB = NaN;  % kN*m^2, bending rigidity of frame AB
EA_BC = NaN;  % kN, axial rigidity of frame BC
EI_BC = NaN;  % kN*m^2, bending rigidity of frame BC
EA_BD = NaN;  % kN, axial rigidity of truss BD; only Part (b)

H = 4.0;      % m, A and B are 4 m above D and C (figure)
w = 12.0;     % kN/m, downward uniformly distributed load on AB
P = 50.0;    % kN, point load at B, acting in negative global x

required = [L_AB L_BC EA_AB EI_AB EA_BC EI_BC];
if caseNumber == 2
    required = [required EA_BD];
end
if any(~isfinite(required))
    error(['Fill in Table 1 values in FrameInput.m before running: ' ...
           'L_AB, L_BC, EA_AB, EI_AB, EA_BC, EI_BC, EA_BD.']);
end
if L_AB <= 0 || L_BC <= H || any(required(3:end) <= 0)
    error('Lengths/rigidities must be positive, and L_BC must exceed 4 m.');
end
if caseNumber ~= 1 && caseNumber ~= 2
    error('caseNumber must be 1 (no BD) or 2 (with BD).');
end

% Geometry inferred from figure: AB horizontal; C and D at y=0.
dx_BC = sqrt(L_BC^2 - H^2);
S.NodeCoord = [0,           H; ...   % A
               L_AB,        H; ...   % B
               L_AB+dx_BC,  0];      % C

% 0=free, 1=restrained, -1=inactive/nonexistent DOF.
% A,C fixed (per written problem); B has 3 free DOFs.
S.DOFFlag = [1 1 1; ... % A: fixed
             0 0 0; ... % B: free
             1 1 1];    % C: fixed

S.IEN = [1 2; 2 3];
S.ElementType = {'frame', 'frame'};
S.EA = [EA_AB; EA_BC];
S.EI = [EI_AB; EI_BC];
S.qLocalY = [-w; 0]; % transverse load in local +y direction (negative)
S.NodeNames = {'A','B','C'};

if caseNumber == 2
    S.NodeCoord(4,:) = [0 0];     % D
    S.DOFFlag(4,:) = [1 1 -1];    % pin D: restrained x,y; no rotation DOF
    S.IEN(3,:) = [2 4];           % third element B-D
    S.ElementType{3} = 'truss';
    S.EA(3,1) = EA_BD;
    S.EI(3,1) = 0;                 % truss carries no moment
    S.qLocalY(3,1) = 0;
    S.NodeNames{4} = 'D';
end

nNode = size(S.NodeCoord,1);
S.NodeLoad = zeros(nNode,3);      % columns: Fx, Fy, M
S.NodeLoad(2,1) = -P;             % 50 kN left at B
S.KnownDisp = zeros(nNode,3);     % all support displacements zero
S.CaseNumber = caseNumber;
S.w = w;
S.P = P;
S.H = H;
end
