load('paramDoubleSlit')

fill3(param.barrierTime*param.barrierX,param.barrierSide*param.barrierY,param.barrierSide*param.barrierZ,'black');
hold on
title("Actual Path")
V = axis
axis equal
axis('auto x')
xlabel("Time")
ylabel("x_1")
zlabel("x_2")
hold on