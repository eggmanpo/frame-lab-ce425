% CE425 - 2D FRAME ANALYSIS - PROBLEM 1 (Parts a and b)
% Run THIS script after entering all required Table 1 data in FrameInput.m.
clearvars;
clc;
close all;

results = cell(2,1);
for part = 1:2
    S = FrameInput(part);
    results{part} = AnalyzeFrame(S);
    r = results{part};

    fprintf('\n======================================================\n');
    if part == 1
        fprintf('PART 1(a): UNBRACED FRAME ABC\n');
    else
        fprintf('PART 1(b): FRAME ABC WITH TRUSS BD\n');
    end
    fprintf('======================================================\n');

    fprintf('\n(i) Destination array ID:\n');
    disp(r.ID);
    fprintf('(ii) Element-node array IEN:\n');
    disp(r.IEN);
    fprintf('(iii) Location matrix LM:\n');
    disp(r.LM);
    fprintf('(iv) Global stiffness matrix K (free DOFs first):\n');
    disp(r.K);
    fprintf('Global applied force vector F (includes UDL equivalents):\n');
    disp(r.F);

    for e = 1:numel(r.elements)
        fprintf('\nElement %d (%s), L = %.6f m\n', ...
                e, r.elements(e).type, r.elements(e).L);
        fprintf('Local element stiffness k_local (6x6):\n');
        disp(r.elements(e).kLocal);
        fprintf('Global element stiffness k_global (6x6):\n');
        disp(r.elements(e).kGlobal);
        fprintf('Element end forces in LOCAL coordinates:\n');
        fprintf('  [N1 V1 M1 N2 V2 M2]^T =\n');
        disp(r.elements(e).fLocal);
    end

    fprintf('\n(v) Joint displacements [ux(m), uy(m), rz(rad)]:\n');
    disp(array2table([(1:size(r.uNode,1)).', r.uNode], ...
         'VariableNames', {'Node','Ux','Uy','Rotation'}));
    fprintf('Joint support reactions [Fx(kN), Fy(kN), M(kN*m)]:\n');
    disp(array2table([(1:size(r.rNode,1)).', r.rNode], ...
         'VariableNames', {'Node','Rx','Ry','Mz'}));
    fprintf('Support moment at A: %+14.6f kN*m\n',r.rNode(1,3));
    fprintf('Support moment at C: %+14.6f kN*m\n',r.rNode(3,3));
    if part == 2
        % Tension positive at the second end of the local truss member.
        fprintf('Axial force in BD (tension +): %+14.6f kN\n', ...
                r.elements(3).fLocal(4));
    end
    fprintf('Equilibrium residual [Fx, Fy, Mz] =\n');
    disp(r.balance);

    save(sprintf('CE425_Part1_%s_results.mat',char('a'+part-1)), 'S', 'r');
end

% Part 1(a) vs Part 1(b) - compare support moments.
a = results{1};
b = results{2};
MAa = a.rNode(1,3); MCa = a.rNode(3,3);
MAb = b.rNode(1,3); MCb = b.rNode(3,3);
fprintf('\n================ SUPPORT MOMENT COMPARISON ================\n');
disp(table({'A';'C'},[MAa;MCa],[MAb;MCb],[MAb-MAa;MCb-MCa], ...
     'VariableNames',{'Support','WithoutBD','WithBD','Change'}));
