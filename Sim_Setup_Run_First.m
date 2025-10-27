I = diag([4.97e-04, 4.97e-04, 4.97e-04]);
A = [zeros(3), eye(3);
zeros(3), zeros(3)];
B = [zeros(3);
inv(I)];
Q = diag([2000, 2000, 2000, 10, 10, 10]);
R_mat = diag([50, 50, 50]);
K1 = lqr(A, B, Q, R_mat);

I = diag([4.97e-04, 4.97e-04, 4.97e-04]);
A = [zeros(3), eye(3);
zeros(3), zeros(3)];
B = [zeros(3);
inv(I)];
Q = diag([400, 400, 400, 1, 1, 1]);
R_mat = diag([100, 100, 100]);
K2 = lqr(A, B, Q, R_mat);

I = diag([4.97e-04, 4.97e-04, 4.97e-04]);
A = [zeros(3), eye(3);
zeros(3), zeros(3)];
B = [zeros(3);
inv(I)];
Q = diag([3000, 3000, 3000, 30, 30, 30]);
R_mat = diag([5, 5, 5]);
K3 = lqr(A, B, Q, R_mat);






