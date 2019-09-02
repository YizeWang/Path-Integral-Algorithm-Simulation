num = 1e6;
uMax =  1;
uMin = -1;
edge = linspace(-5,5,400);
figure('Name','Truncated Normal Distribution')
subplot(2,2,1)
u = trandn(-1*inf(num,1),1*inf(num,1));
histogram(u,edge)
title("No Bounds")
axis([-5 5 0 15000]);
subplot(2,2,2)
u = trandn(-1*inf(num,1),uMax*ones(num,1));
histogram(u,edge)
title("Upper Bound 1")
axis([-5 5 0 15000]);
subplot(2,2,3)
u = trandn(uMin*ones(num,1),1*inf(num,1));
histogram(u,edge);
title("Lower Bound -1")
axis([-5 5 0 15000])
subplot(2,2,4)
u = trandn(uMin*ones(num,1),uMax*ones(num,1));
histogram(u,edge);
title("Bounded in [-1,1]")
axis([-5 5 0 15000])