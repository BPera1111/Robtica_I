function [q_traj,qd, qdd] = concatenar_traj(q_traj,qd,qdd, aux1, aux2, aux3)
q_traj = [q_traj; aux1];
qd = [qd; aux2];
qdd = [qdd; aux3];
end

