function el = ElementMatrices(type, EA, EI, p1, p2, qy)
% ELEMENTMATRICES  2D frame or pin-ended axial truss element.
% Common DOF order: [ux1 uy1 rz1 ux2 uy2 rz2].
% Local member x points from node 1 to node 2; local y is 90 deg CCW.
% For both element types, transformations use u_local = T*u_global.

dx = p2(1)-p1(1);
dy = p2(2)-p1(2);
L = hypot(dx,dy);
if L <= 0
    error('Zero-length member detected.');
end
c = dx/L;
s = dy/L;
R = [c s 0; -s c 0; 0 0 1];
T = blkdiag(R,R);

kl = zeros(6,6);
fqLocal = zeros(6,1); % consistent equivalent applied nodal loads

switch lower(type)
    case 'frame'
        if EA <= 0 || EI <= 0
            error('Frame EA and EI must be positive.');
        end
        a = EA/L;
        b = 12*EI/L^3;
        d = 6*EI/L^2;
        e = 4*EI/L;
        f = 2*EI/L;
        kl = [ a,  0,  0, -a,  0,  0;
               0,  b,  d,  0, -b,  d;
               0,  d,  e,  0, -d,  f;
              -a,  0,  0,  a,  0,  0;
               0, -b, -d,  0,  b, -d;
               0,  d,  f,  0, -d,  e];
        % Constant transverse load qy, positive local +y.
        % For AB, qy=-12 kN/m (vertical downward).
        fqLocal = [0; qy*L/2; qy*L^2/12; ...
                   0; qy*L/2; -qy*L^2/12];
    case 'truss'
        if EA <= 0
            error('Truss EA must be positive.');
        end
        axial = [1 -1; -1 1]*(EA/L);
        kl([1 4],[1 4]) = axial;
        if abs(qy) > 0
            error('Transverse distributed loads on a truss are not supported.');
        end
    otherwise
        error('ElementType must be ''frame'' or ''truss''.');
end

el.type = type;
el.L = L;
el.c = c;
el.s = s;
el.T = T;
el.kLocal = kl;
el.kGlobal = T.'*kl*T;
el.fEquivalentLocal = fqLocal;
el.fEquivalentGlobal = T.'*fqLocal;
end
