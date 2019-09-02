num = 1e6;
uMax =  1*inf;
uMin = -1*inf;
uMaxVec = uMax*ones(num,1);
uMinVec = uMin*ones(num,1);
u = trandn(uMinVec,uMaxVec);
histogram(u);