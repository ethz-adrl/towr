% script to solve a boundary value problem
clc;
close all;
clear all;

syms t g h v w u
syms c_init cd_init;
assume(g > 0);
assume(h > 0);
a = sqrt(g/h); % for convenience
u = c_init + cd_init*sqrt(h/g); % assume this is the case

% solution from Ansatz (homegenous + partial solution)
c_hom = v*exp(a*t) + w*exp(-a*t); % solve homegenous equation
c_par = u; % solves partial equation (rhs not zero).
c   = c_hom + c_par; 
cd  = diff(c, t)

% specify initial position and velocity
t=0;
eq_init_p = subs(c)   == c_init
eq_init_v = subs(cd)  == cd_init

% specify final position and velocity
%syms T
%t=T;
%eq_final_p = subs(c)  == u
%eq_final_v = subs(cd) == 0


% solve the BVP
variables = [v,w];
equations = [eq_init_p, eq_init_v];%, eq_final_p, eq_final_v];
%solve the initial value problem
% variables = [v,w];
% equations = [eq_init_p, eq_init_v];

S = solve(equations, variables)

% display the results
%v = simplify(S.v)
%w = simplify(S.w)
%T = simplify(S.T)
%u = simplify(S.u);
%pretty(u);

%h=9; g=9; c0=0; cd0 = 2;
%subs(u)

% need to make time symbolic again
syms t
%c_init = 0;
%cd_init = 0;
subs(u)

v = simplify(S.v)
w = simplify(S.w)
subs(c)
%subs(w)
%subs(u)
%subs(c)
subs(c, [c_init, cd_init, h, g], [0,0, 0.58, 9.81]);
c
%ezplot(c);


%time = [0:0.1:10];
%plot(time, cdd); 







