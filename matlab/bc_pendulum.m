function [ res ] = bc_pendulum( y_start, y_finish, p )
%UNTITLED5 Summary of this function goes here
%   The residual that is tried to be driven to zero on interval ab
res = [ y_start(2)-1
        y_start(1)
        y_finish(2)];

end

