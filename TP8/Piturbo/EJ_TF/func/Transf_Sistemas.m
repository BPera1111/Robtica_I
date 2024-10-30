function T = Transf_Sistemas(fila_dh, q)
  T = trotz(q)*transl(0,0,fila_dh(2))*transl(fila_dh(3),0,0)*trotx(fila_dh(4));
end