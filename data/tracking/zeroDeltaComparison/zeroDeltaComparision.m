

%reads in both 2 norm error vectors
zeroDelta_norm2 = csvread("zeroDelta/norm2.csv");
nonZeroDelta_norm2 = csvread("nonZeroDelta/norm2.csv");


%plots out both the zero and the nonzero deltas
figure(1);
plot(zeroDelta_norm2);
hold on;
plot(nonZeroDelta_norm2);
hold off;
title("2 Norm Comparison");
legend('Zero Delta', 'Non Zero Delta')