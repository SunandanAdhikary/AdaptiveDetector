A = randn(10,2);b = rand(10,1)*10;
w = sdpvar(2,1);
zc = sdpvar(2,1);
r = sdpvar(1);

z = r*w + zc;
UncertainConstraint = [A*z <= b];
UncertaintyModel = [norm(w,1) <= 1];

optimize([UncertainConstraint,UncertaintyModel,uncertain(w)],-r)
[A1,b1] = polytope(constraints)