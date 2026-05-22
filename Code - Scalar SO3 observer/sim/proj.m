function [pr]= proj(z)
%    n = length(z);
%    z = reshape(z, [n 1]);
    pr = eye(3) - z'.*z;
end