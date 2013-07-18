function out = ConstrainedReconstruct(W, P, xp, y, lb, ub, ind, x0, delta)
H = P'*W*P;
f = xp'*W*P;
Aeq = P(ind,:);
beq = y(ind);

b1 = y+delta; % delta is the error tolerance to landmark positions
b2 = y-delta;
bmin = min(b1, b2);
bmax = max(b1, b2);
A = [P;-P];
b = [bmax;-bmin];

out = quadprog(H, f, [], [], Aeq, beq, lb, ub, x0);
% out = quadprog(H, f, A, b, Aeq, beq, [], []);
 a = [Aeq*out beq]
 size(a)
