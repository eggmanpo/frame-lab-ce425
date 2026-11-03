function out = AnalyzeFrame(S)
% ANALYZEFRAME  Assemble and solve a mixed frame/truss stiffness system.
% The program AUTOMATICALLY assembles the stiffness matrix via LM.
% ID convention: +k free eqn, -k restrained eqn, 0 inactive DOF.
% Full matrix ordering: [free DOFs, restrained DOFs].

nn = size(S.NodeCoord,1);
nel = size(S.IEN,1);
ndof = 3;

% 1. Enumerate destination array ID.
ID = zeros(nn,ndof);
nFree = 0;
nRestrained = 0;
for n = 1:nn
    for j = 1:ndof
        if S.DOFFlag(n,j) == 0
            nFree = nFree + 1;
            ID(n,j) = nFree;
        elseif S.DOFFlag(n,j) == 1
            nRestrained = nRestrained + 1;
            ID(n,j) = -nRestrained;
        elseif S.DOFFlag(n,j) == -1
            ID(n,j) = 0;       % absent, e.g. D rotation for a truss
        else
            error('Invalid DOFFlag.');
        end
    end
end
nTotal = nFree + nRestrained;

% 2. Construct location matrix LM using ID and IEN.
% Each row describes [ux1 uy1 rz1 ux2 uy2 rz2] of its element.
LM = zeros(nel,6);
for e = 1:nel
    for a = 1:2
        node = S.IEN(e,a);
        LM(e,(a-1)*3+(1:3)) = ID(node,:);
    end
end

% 3. Calculate element matrices, then assemble K and F automatically.
K = zeros(nTotal,nTotal);
F = zeros(nTotal,1);
for e = 1:nel
    n1 = S.IEN(e,1);
    n2 = S.IEN(e,2);
    el(e) = ElementMatrices(S.ElementType{e}, S.EA(e), S.EI(e), ...
                            S.NodeCoord(n1,:), S.NodeCoord(n2,:), ...
                            S.qLocalY(e)); %#ok<AGROW>
    index = mapLM(LM(e,:), nFree);
    for i = 1:6
        if index(i) == 0, continue; end
        F(index(i)) = F(index(i)) + el(e).fEquivalentGlobal(i);
        for j = 1:6
            if index(j) == 0, continue; end
            K(index(i),index(j)) = K(index(i),index(j)) + ...
                                    el(e).kGlobal(i,j);
        end
    end
end

% 4. Apply point loads (including any nodal moments).
for n = 1:nn
    index = mapLM(ID(n,:), nFree);
    for j = 1:3
        if index(j) ~= 0
            F(index(j)) = F(index(j)) + S.NodeLoad(n,j);
        elseif abs(S.NodeLoad(n,j)) > 0
            error('Applied load at inactive DOF.');
        end
    end
end

% 5. Set prescribed displacements and solve the partitioned system.
U = zeros(nTotal,1);
for n = 1:nn
    for j = 1:ndof
        if ID(n,j) < 0
            index = nFree - ID(n,j);
            U(index) = S.KnownDisp(n,j);
        end
    end
end
Kff = K(1:nFree,1:nFree);
Kfc = K(1:nFree,nFree+1:end);
if nFree > 0
    if rcond(Kff) < 1e-12
        error('Singular/ill-conditioned Kff. Check geometry and supports.');
    end
    U(1:nFree) = Kff \ (F(1:nFree)-Kfc*U(nFree+1:end));
end

% 6. Reactions: R = K*U - F. Free DOF residuals should be ~zero.
R = K*U-F;
uNode = NaN(nn,ndof);
rNode = NaN(nn,ndof);
for n = 1:nn
    for j = 1:ndof
        index = mapLM(ID(n,j),nFree);
        if index ~= 0
            uNode(n,j) = U(index);
            if ID(n,j) < 0
                rNode(n,j) = R(index);
            else
                rNode(n,j) = 0;
            end
        end
    end
end

% 7. Recover each element's LOCAL end forces:
%    f_local = k_local*u_local - consistent_equivalent_load_local.
for e = 1:nel
    index = mapLM(LM(e,:),nFree);
    uElemGlobal = zeros(6,1);
    for j = 1:6
        if index(j) ~= 0
            uElemGlobal(j) = U(index(j));
        end
    end
    el(e).uLocal = el(e).T*uElemGlobal;
    el(e).fLocal = el(e).kLocal*el(e).uLocal - ...
                   el(e).fEquivalentLocal;
end

% 8. Global force and moment equilibrium about (0,0).
balance = zeros(3,1); % Fx, Fy, Mz
for n = 1:nn
    x = S.NodeCoord(n,1);
    y = S.NodeCoord(n,2);
    fNode = S.NodeLoad(n,:);
    balance = balance + [fNode(1); fNode(2); ...
                         x*fNode(2)-y*fNode(1)+fNode(3)];
    support = rNode(n,:);
    support(~isfinite(support)) = 0;
    balance = balance + [support(1); support(2); ...
                         x*support(2)-y*support(1)+support(3)];
end
for e = 1:nel
    % Distributed load qy along local y: global components [-s, c].
    q = S.qLocalY(e);
    if q == 0, continue; end
    fDist = q*el(e).L*[-el(e).s; el(e).c];
    midpoint = (S.NodeCoord(S.IEN(e,1),:) + ...
                S.NodeCoord(S.IEN(e,2),:))/2;
    balance = balance + [fDist; midpoint(1)*fDist(2)-midpoint(2)*fDist(1)];
end

out.ID = ID;
out.IEN = S.IEN;
out.LM = LM;
out.K = K;
out.F = F;
out.U = U;
out.R = R;
out.Kff = Kff;
out.nFree = nFree;
out.nRestrained = nRestrained;
out.uNode = uNode;
out.rNode = rNode;
out.elements = el;
out.balance = balance;
end

function index = mapLM(id,nFree)
% Convert signed DOF labels to positive MATLAB indices.
index = zeros(size(id));
index(id>0) = id(id>0);
index(id<0) = nFree- id(id<0);
end
