function [ E, sigmas ] = powmeth_rightvecs(A, k, iterations)
%POWMETH_RIGHTVECS Use power method to calculate k right singular vectors.
%   Also includes associated singular values.
%
% Written by Judson Wilson for EE378b Homework

m = size(A,1);

%Produce initial random E - its k columns are a random, orthonormal.
%rng(1); %Make it repeatably random
E = rand(m,k);
%Gram-Schmidt
for a=1:k
    e = E(:,a);
    for b = 1:(a-1)
        %%for debug:
        %fprintf('a:%d  b:%d  component:%d\n',a,b,(e'*E(:,b)));

        e = e - (e'*E(:,b))*E(:,b);
    end
    %%for debug:
    %fprintf('normalizer: %d\n',norm(e));

    E(:,a) = e/norm(e);
end

%%

%Do power iterations
for i=1:iterations
    E = A*A'*E;

    %Gram-Schmidt
    for a=1:k
        e = E(:,a);
        for b = 1:(a-1)
            %%for debug:
            %fprintf('a:%d  b:%d  component:%d\n',a,b,(e'*E(:,b)));

            e = e - (e'*E(:,b))*E(:,b);
        end
        %%for debug:
        %fprintf('normalizer: %d\n',norm(e));
        e = e/norm(e);
        
%        %Make orientation constant
%        [~,ind] = max(abs(e(1:length(e)/10*4)));%Look at a big enough range
%        if e(ind) < 0                %such that a zero crossing near n=0
%            e = -e;                  %won't throw this off. Only works
%        end                          % for this particular problem.
    
        E(:,a) = e;
    end
    
    %if mod(i,100) == 0
    %    fprintf('Iteration: %i\n', i);
    %end
end

%Done - now calculate singular values
sigmas = zeros(k,1);   
for a=1:k
    sigmas(a) = sqrt(norm(A*A'*E(:,a)));
end

end

