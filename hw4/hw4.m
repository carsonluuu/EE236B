P1 = [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, 0];
P2 = [1, 0, 0, 0; 0, 0, 1, 0; 0, -1, 0, 10];
P3 = [1, 1, 1, -10; -1, 1, 1, 0; -1, -1, 1, 10];
P4 = [0, 1, 1, 0; 0, -1, 1, 0; -1, 0, 0, 10];

y1 = [0.98, 0.93];
y2 = [1.01, 1.01];
y3 = [0.95, 1.05];
y4 = [2.04, 0.00];

start = 0;
end = 1;
mid = (end - start)/2 + start;
tol = 1e-5;
while (end - start > tol)
	cvx_begin
		variable x(3)
		subject to
		norm(P1(1:2, 1:3)*x + P1(1:2, 4) - (P1(3, 1:3)*x + P1(3, 1:3)*x + P1(3, 4))*y1 ,2) - mid*P1(3, 1:3)*x + P1(3, 4) <= 0 
		norm(P2(1:2, 1:3)*x + P2(1:2, 4) - (P2(3, 1:3)*x + P2(3, 1:3)*x + P2(3, 4))*y1 ,2) - mid*P2(3, 1:3)*x + P2(3, 4) <= 0 
		norm(P3(1:2, 1:3)*x + P3(1:2, 4) - (P3(3, 1:3)*x + P3(3, 1:3)*x + P3(3, 4))*y1 ,2) - mid*P3(3, 1:3)*x + P3(3, 4) <= 0 
		norm(P4(1:2, 1:3)*x + P4(1:2, 4) - (P4(3, 1:3)*x + P4(3, 1:3)*x + P4(3, 4))*y1 ,2) - mid*P4(3, 1:3)*x + P4(3, 4) <= 0 
	cvx_end
	if cvx_optimal == Inf
		start = mid;
	else
		end = mid;
		res = x;
	end
	mid = (end - start)/2 + start;
end
res;
mid;
% --------Result----------
mid = 0.0495
res = [4.9, 5.0, 5.2]' %rounding to one decimal


