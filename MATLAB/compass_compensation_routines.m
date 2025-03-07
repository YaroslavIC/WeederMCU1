scatter3(x(:,1),x(:,2),x(:,3));
axis equal
title('Ideal Magnetometer Data');

[A,b,expMFS]  = magcal(x);
xCorrected = (x-b)*A;

de = HelperDrawEllipsoid;
de.plotCalibrated(A,b,expMFS,x,xCorrected,'Auto');


N= size(x,1);
r = sum(xCorrected.^2,2) - expMFS.^2;
E = sqrt(r.'*r./N)./(2*expMFS.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);

[Aeye,beye,expMFSeye] = magcal(x,'eye');
xEyeCorrected = (x-beye)*Aeye;
[ax1,ax2] = de.plotCalibrated(Aeye,beye,expMFSeye,x,xEyeCorrected,'Eye');
view(ax1,[-1 0 0]);
view(ax2,[-1 0 0]);


[Adiag,bdiag,expMFSdiag] = magcal(x,'diag');
xDiagCorrected = (x-bdiag)*Adiag;
[ax1,ax2] = de.plotCalibrated(Adiag,bdiag,expMFSdiag,x,xDiagCorrected,...
    'Diag');

[A,b] = magcal(x,'sym');

xidx = x(:,3) > 100;
xpoor = x(xidx,:);
[Apoor,bpoor,mfspoor] = magcal(xpoor,'diag');

disp(Apoor)

[Abest,bbest,mfsbest] = magcal(xpoor,'auto');
disp(Abest)

de.compareBest(Abest,bbest,mfsbest,Apoor,bpoor,mfspoor,xpoor);
format shortg

figure(1); clf; hold on;
plot3(x(:,1),x(:,2),x(:,3),'.')
plot3(xCorrected(:,1),xCorrected(:,2),xCorrected(:,3),'.')
axis equal

% умножение ветора строки на матрицу

x0 = x(1,:);

x1 = [x0(1)- b(1) x0(2)-b(2) x0(3)-b(3)];

xmcor = [x1(1)*A(1,1)+x1(2)*A(2,1)+x1(3)*A(3,1)   x1(1)*A(1,2)+x1(2)*A(2,2)+x1(3)*A(3,2) x1(1)*A(1,3)+x1(2)*A(2,3)+x1(3)*A(3,3)]


format longg

for i=1:3
  disp('b['+string(i-1)+']='+string(b(i ))+';' );
end    

for i=1:3
    for j=1:3
      disp('A['+string(i-1)+']['+string(j-1)+']='+string(A(i,j))+';' );
    end
end    