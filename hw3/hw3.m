close all
clear all
n=3; % state dimension
N=30; % time horizon
A=[ -1 0.4 0.8; 1 0 0 ; 0 1 0];
b=[ 1 0 0.3]';
x0 = zeros(n,1);
xdes = [ 7 2 -6]';
cvx_begin
variable X(n,N+1);
variable u(1,N);
minimize (sum(max(abs(u),2*abs(u)-1)))
subject to
X(:,2:N+1) == A*X(:,1:N)+b*u; % dynamics
X(:,1) == x0;
X(:,N+1) == xdes;
cvx_end
stairs(0:N-1,u,'linewidth',2)
axis tight
xlabel('t')
ylabel('u')