function [t,R] = callSVD(Pm, kPm)

[m,n] = size(Pm);

for i=1:n
  P1(i,1) = 1.0;
end

Pmc = Pm*P1/n;

kPmc = kPm*P1/n;

for i=1:m
   for j=1:n
     Pm(i,j) = Pm(i,j) - Pmc(i);
     kPm(i,j) = kPm(i,j) - kPmc(i);
   end
end

[U,S,V] = svd(kPm*Pm');

H=diag([1,1,det(V*U')]);

R=V*H*U';

t = Pmc - R*kPmc;

fid = fopen('calibmat.txt', 'a');
fprintf(fid, '#\n');
fprintf(fid, '4 4\n');
fprintf(fid, '%f %f %f %f\n', R(1,1), R(1,2), R(1,3), t(1));
fprintf(fid, '%f %f %f %f\n', R(2,1), R(2,2), R(2,3), t(2));
fprintf(fid, '%f %f %f %f\n', R(3,1), R(3,2), R(3,3), t(3));
fprintf(fid, '0 0 0 1\n');
fclose(fid);





