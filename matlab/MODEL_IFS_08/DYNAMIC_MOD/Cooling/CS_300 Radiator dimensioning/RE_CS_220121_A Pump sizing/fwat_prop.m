function [ro_wat,cp_wat,mu_wat,k_wat,Pr_wat] = fwat_prop(Teval_wat)
load('RE_CS_220901_APropiedadesAgua.mat');

%Se determina la fila en la que la temperatura es justo la siguiente menor.
%Se ponderará entre esta y la siguiente
k=0;
i=1;
while k==0
    if B(i,1)>Teval_wat
        k=1;
    end
    i=i+1;
end
i=i-2;

%Trabajo de cada una de las propiedades a entregar por la función
ro_wat=B(i,2)+(B(i+1,2)-B(i,2))/(B(i+1,1)-B(i,1))*(Teval_wat-B(i,1));
cp_wat=B(i,3)+(B(i+1,3)-B(i,3))/(B(i+1,1)-B(i,1))*(Teval_wat-B(i,1));
cp_wat=cp_wat*1e3;
mu_wat=B(i,4)+(B(i+1,4)-B(i,4))/(B(i+1,1)-B(i,1))*(Teval_wat-B(i,1));
mu_wat=mu_wat*1e-6;
k_wat=B(i,6)+(B(i+1,6)-B(i,6))/(B(i+1,1)-B(i,1))*(Teval_wat-B(i,1));
k_wat=k_wat*1e-3;
Pr_wat=B(i,9)+(B(i+1,9)-B(i,9))/(B(i+1,1)-B(i,1))*(Teval_wat-B(i,1));
end

