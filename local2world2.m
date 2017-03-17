clc;
clear;



positions(:,:,1) = [0         0              0;
                    0.0435    0.0203         0;
                    0.1187    0.0554   -0.0015;
                    0.1500    0.0700   -0.0900];

positions(:,:,2) = [0         0              0;
                    0.0477   -0.0054         0;
                    0.1301   -0.0146    0.0029;
                    0.1500   -0.0168   -0.0900];
                
positions(:,:,3) = [0         0              0;
                    0.0472    0.0086         0;
                    0.1288    0.0236    0.0025;
                    0.1500    0.0275   -0.0900];
               
positions(:,:,4) = [    0         0              0;
                        0.0444   -0.0181         0;
                        0.1213   -0.0494   -0.0002;
                        0.1500   -0.0611   -0.0900];
         
BodyDim = [0.1895 0.1895];
                    
BC2LTmat = zeros(4,4,4);
BC2LTmat(:,:,1) = [-1 0 0 -BodyDim(1)/2; 0 1 0 BodyDim(2)/2; 0 0 1 0; 0 0 0 1];
BC2LTmat(:,:,2) = [1 0 0 BodyDim(1)/2; 0 1 0 BodyDim(2)/2; 0 0 1 0; 0 0 0 1];
BC2LTmat(:,:,3) = [-1 0 0 -BodyDim(1)/2; 0 1 0 -BodyDim(2)/2; 0 0 1 0; 0 0 0 1];
BC2LTmat(:,:,4) = [1 0 0 BodyDim(1)/2; 0 1 0 -BodyDim(2)/2; 0 0 1 0; 0 0 0 1];

Hb = 0.09;

G2BTmat = eye(4);
G2BTmat(3,4) = Hb;

world_pos = zeros(4,3,4);

for i = 1:4
world_pos(i,:,1) = (G2BTmat(1:3,1:3)*BC2LTmat(1:3,1:3,1)*(positions(i,:,1))') + (G2BTmat(1:3,1:3)*[-BodyDim(1)/2; BodyDim(2)/2; 0]) + [0; 0; Hb];

end

for i = 1:4
world_pos(i,:,2) = (G2BTmat(1:3,1:3)*BC2LTmat(1:3,1:3,2)*(positions(i,:,2))') + (G2BTmat(1:3,1:3)*[BodyDim(1)/2; BodyDim(2)/2; 0]) + [0; 0; Hb];

end

for i = 1:4
world_pos(i,:,3) = (G2BTmat(1:3,1:3)*BC2LTmat(1:3,1:3,3)*(positions(i,:,3))') + (G2BTmat(1:3,1:3)*[-BodyDim(1)/2; -BodyDim(2)/2; 0]) + [0; 0; Hb];

end

for i = 1:4
world_pos(i,:,4) = (G2BTmat(1:3,1:3)*BC2LTmat(1:3,1:3,4)*(positions(i,:,4))') + (G2BTmat(1:3,1:3)*[BodyDim(1)/2; -BodyDim(2)/2; 0]) + [0; 0; Hb]; 

end
