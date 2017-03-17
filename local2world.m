function [ positions ] = local2world( pos)

% Function to transform from local frame to the global frame
global BC2LTmat;
global G2BTmat;

p = (G2BTmat*BC2LTmat(:,:,1)*[pos(:,:,1)';ones(1,size(pos,1))])';
positions(:,:,1) = p(:,1:end-1);
p = (G2BTmat*BC2LTmat(:,:,2)*[pos(:,:,2)';ones(1,size(pos,1))])';
positions(:,:,2) = p(:,1:end-1);
p = (G2BTmat*BC2LTmat(:,:,3)*[pos(:,:,3)';ones(1,size(pos,1))])';
positions(:,:,3) = p(:,1:end-1);
p = (G2BTmat*BC2LTmat(:,:,4)*[pos(:,:,4)';ones(1,size(pos,1))])';
positions(:,:,4) = p(:,1:end-1);
end

