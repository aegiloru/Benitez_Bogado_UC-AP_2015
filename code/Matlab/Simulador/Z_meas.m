function Z_meas
global A;

A.Z_meas = A.Z+A.Z_error(A.counter);     %error is +- 2 cm
end