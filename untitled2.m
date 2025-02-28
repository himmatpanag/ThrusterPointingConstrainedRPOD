% compare what an MRP error of 1 deg is 
N = 100;
p = zeros(3,N);
for ii = 1:N
    v = rand(3,1);
    n = v./norm(v);
    p(:,ii) = n * tan(pi/180/4);
end 
figure; grid on; hold on; plot(p')
figure; grid on; hold on; plot(vecnorm(p))