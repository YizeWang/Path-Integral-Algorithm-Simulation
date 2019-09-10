close all

x = -3:0.001:3;
y = -1:0.001:1;
pd = makedist('Normal','mu',0,'sigma',1);
figure(1)
plot(x,pdf(pd,x));
axis([min(x) max(x) 0 0.6])
figure(2)
plot(y,pdf(pd,y));
axis([min(x) max(x) 0 0.6])
figure(3)
t = truncate(pd,-1,1);
plot(x,pdf(t,x));
axis([min(x) max(x) 0 0.6])