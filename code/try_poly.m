model.u.min = [-1;-1];
model.u.max = [1;17];
model.x.min = [-15;-15];
model.x.max = [ 15; 15];
% constraint sets represented as polyhedra
X = Polyhedron('lb',model.x.min,'ub',model.x.max);
plot(X,'color','red');
hold on

U = Polyhedron('lb',model.u.min,'ub',model.u.max);
plot(U,'color','green');
S=X.intersect(U)
plot(S,'color','blue','alpha',0.5)