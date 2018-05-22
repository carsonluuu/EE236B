clc;
clear;
m = 50;
n = 40;

for i = 1:4
    randn('state',0)
    s = [0.5 1 2 3];
    A = randn(m,n);
    xhat = sign(randn(n,1));
    b = A * xhat + s(i) * randn(m,1);
    
    cvx_begin sdp
        variable z(n);
        variable Z(n,n) symmetric;
        minimize (trace(A'*A*Z) - 2*b'*A*z + b'*b)
        subject to
            [Z z; z' 1] >= 0
            diag(Z) == 1;
    cvx_end

    f1(i) = norm(A * sign((A\b)) - b);
    f2(i) = norm(A * sign(z) - b);
    [v, u] = eig([Z z; z' 1]);
    f3(i) = norm(A * sign(v(1:n, n+1))-b);

    for j = 1:100
        f4(i) = norm(A * sign(mvnrnd(z, Z-z*z')') - b);
    end

    res(i) = sqrt(trace(A'*A*Z) - 2*b'*A*z + b'*b);
end

% result:

+---------+---------+---------+---------+---------+---------+---------+
|    s    |  f(xa)  |  f(xb)  |  f(xc)  |  f(xd)  |  f(x*)  | dual-d* |
+---------+---------+---------+---------+---------+---------+---------+
|   0.5   |  4.1623 |  4.1623 |  4.1623 |  4.1623 |  4.1623 |  4.0524 |
+---------+---------+---------+---------+---------+---------+---------+
|   1.0   | 12.7299 |  8.3245 |  8.3245 |  8.3245 |  8.3245 |  7.8678 |
+---------+---------+---------+---------+---------+---------+---------+
|   2.0   | 30.1419 | 16.6490 | 16.6490 | 16.6490 | 16.6490 | 15.1804 |
+---------+---------+---------+---------+---------+---------+---------+
|   3.0   | 33.9339 | 25.9555 | 25.9555 | 24.9735 | 25.9555 | 22.1139 |
+---------+---------+---------+---------+---------+---------+---------+

% compare
- for higher s, the f(xa) tend to be worst.
- for SDP, the values are rather close
- when s = 0.5, all the results are the same, it is likely it is either the global optimum or is close.
- when s = 3.0, the random one give the optimal one, the other ones are higher than it
