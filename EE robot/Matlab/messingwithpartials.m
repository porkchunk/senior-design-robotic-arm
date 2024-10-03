syms X Y

CX = cos(X);

F = CX + Y + Y.^2;

diff(F,X)