function [ stm ] = getStablityMargin( spoly, com)

% This function calculates the stability margin at current position.

d = zeros(size(spoly,1)-1,1);
for i=1:(size(spoly,1)-1)
    d(i) = norm(cross((spoly(i+1,:)'-spoly(i,:)'),(com'-spoly(i,:)')))/norm((spoly(i+1,:)'-spoly(i,:)'));
end
stm = min(d);
stm = 100*stm;
end

