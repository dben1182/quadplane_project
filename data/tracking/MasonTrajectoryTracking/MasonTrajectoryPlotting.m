%reads in the csv riles
Norm1Vector = csvread('norm1.csv');
Norm2Vector = csvread('norm2.csv');

Norm1Integral = csvread('norm1_integral.csv');
Norm2Integral = csvread('norm2_integral.csv');


figure(1);
plot(Norm1Vector);
title("1 Norm Vector");

figure(2);
plot(Norm2Vector);
title("2 Norm Vector");