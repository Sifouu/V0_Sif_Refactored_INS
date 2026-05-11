classdef ALLFUNCS
% ALLFUNCS A static class containing mathematical utilities for the INS observers.
%
% This class provides centralized implementations of standard matrix and Lie group
% operations to avoid code duplication across different observers.

    methods(Static)
        
        function S = skew(v)
        % SKEW Returns the skew-symmetric matrix of a 3x1 vector.
            S = [  0   -v(3)  v(2);
                   v(3)   0   -v(1);
                  -v(2)  v(1)   0 ];
        end
        
        function Rexp = Rexp(x)
        % REXP Computes the exponential map from R^3 to SO(3).
        % Uses the Rodrigues' rotation formula.
            if norm(x) < 1e-8
                Rexp = eye(3);
            else
                n_x = norm(x);
                S_x = ALLFUNCS.skew(x);
                Rexp = eye(3) + (sin(n_x)/n_x) * S_x + ((1 - cos(n_x))/(n_x^2)) * (S_x^2);
            end
        end

        function y = sincc(alpha)
        % SINCC Computes the unnormalized sinc function: sin(x)/x.
        % Handles the singularity at x = 0 safely.
            if abs(alpha) < 1e-8
                y = 1;
            else
                y = sin(alpha) ./ alpha;
            end
        end
        
        function R_ortho = orthogonalize(R)
        % ORTHOGONALIZE Forces a 3x3 matrix to remain strictly on the SO(3) manifold.
        % Uses Singular Value Decomposition (SVD) to correct numerical drift.
            [U, ~, V_svd] = svd(R);
            R_ortho = U * diag([1, 1, det(U * V_svd')]) * V_svd';
        end

    end
end
