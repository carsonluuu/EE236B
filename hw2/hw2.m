clc;clear;
%%------------DATA----------
[t, y] = spline_data;
G = [];
A = [];
n = 13;
for i=0:10
    [t1, t2, gpp] = bsplines(i);
    G = [G; gpp'];
end

for i = 1:51
    [g, gg, ggg] =bsplines(t(i));
    A=[A; g'];
end
%%------------CVX-----------
cvx_begin
    variable x(13)
    minimize norm(A*x - y)
    subject to 
        G*x >= 0
cvx_end
%%-----------Plot------------
fp = 0:.01:10;
fm = [];
for i=1:length(fp) 
    [g, gg1, ggg1] = bsplines(fp(i));
    fm = [fm; g']; 
end
f = fm*x;
plot(t, y, 'go'), hold on,
plot(fp,f, 'm')

