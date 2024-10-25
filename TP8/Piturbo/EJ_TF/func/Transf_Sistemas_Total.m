<<<<<<< HEAD
function T_total = Transf_Sistemas_Total(dh, q)
    T=zeros(4,4*length(q));
    for i=1:length(q)
        %Calcula las matrices de un sistema respecto al siguiente
        T(:,[1+4*(i-1):4+4*(i-1)])= Transf_Sistemas(dh(i,:),q(i)); 
    end
    %Calcula la matriz total
    T_total = eye(4);
    for i=1:length(q)
        T_total = T_total * T(:,[1+4*(i-1):4+4*(i-1)]);
    end
=======
function T_total = Transf_Sistemas_Total(dh, q)
    T=zeros(4,4*length(q));
    for i=1:length(q)
        %Calcula las matrices de un sistema respecto al siguiente
        T(:,[1+4*(i-1):4+4*(i-1)])= Transf_Sistemas(dh(i,:),q(i)); 
    end
    %Calcula la matriz total
    T_total = eye(4);
    for i=1:length(q)
        T_total = T_total * T(:,[1+4*(i-1):4+4*(i-1)]);
    end
>>>>>>> 5d5cfbc (por favor funciona)
end