function y = skewSym(v)
% Returns the skew symmetric matrix of a vector with dimensions 3 x 1

a = v(1,1);
b = v(2,1);
c = v(3,1);
y = [   0  -c   b;
        c   0  -a;
       -b   a   0];