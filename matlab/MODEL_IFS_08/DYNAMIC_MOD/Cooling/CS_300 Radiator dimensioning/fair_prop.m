function [ro_air,cp_air,mu_air,k_air,Pr_air] = fair_prop(Teval_air)
%Función propiedades del aire. A partir de una tabla de datos se iteran las
%propiedades para una temperatura de entrada dada. Interpolación lineal.

%Se carga la matrix de propiedades el aire.
load('RE_CS_220901_APropiedadesAire.mat','B');

%Se determina la fila en la que la temperatura es justo la siguiente menor.
%Se ponderará entre esta y la siguiente
% k=0;
% i=1;
% while k==0
%     if B(i,1)>Teval_air
%         k=1;
%     end
%     i=i+1;
% end
% i=i-2;
% 
% %Trabajo de cada una de las propiedades a entregar por la función
% ro_air=B(i,2)+(B(i+1,2)-B(i,2))/(B(i+1,1)-B(i,1))*(Teval_air-B(i,1));
% cp_air=B(i,3)+(B(i+1,3)-B(i,3))/(B(i+1,1)-B(i,1))*(Teval_air-B(i,1));
% mu_air=B(i,6)+(B(i+1,6)-B(i,6))/(B(i+1,1)-B(i,1))*(Teval_air-B(i,1));
% k_air=B(i,4)+(B(i+1,4)-B(i,4))/(B(i+1,1)-B(i,1))*(Teval_air-B(i,1));
% Pr_air=B(i,8)+(B(i+1,8)-B(i,8))/(B(i+1,1)-B(i,1))*(Teval_air-B(i,1));

%Planteamiento secundario. Debería ser igual que lo planteado antes.
ro_air=interp1(B(:,1),B(:,2),Teval_air);
cp_air=interp1(B(:,1),B(:,3),Teval_air);
mu_air=interp1(B(:,1),B(:,6),Teval_air);
k_air=interp1(B(:,1),B(:,4),Teval_air);
Pr_air=interp1(B(:,1),B(:,8),Teval_air);
end
