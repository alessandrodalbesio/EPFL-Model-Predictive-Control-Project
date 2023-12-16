function [obj,con] = ss_constraints(mpc,xs,us,ref)
    % Initialize parameters
    Rs = 1;
    [ny, nx] = size(mpc.C);
    
    % Define constraints and cost function
    obj = us'*Rs*us;
    con = [eye(nx,nx) - mpc.A, -mpc.B ; mpc.C, 0]*[xs; us] == [zeros(nx,ny);ref];
    if ~all(all(isnan(mpc.F))) && ~all(all(isnan(mpc.f))); con = [con, mpc.F*xs <= mpc.f]; end
    if ~all(isnan(mpc.M)) && ~all(isnan(mpc.m)); con = [con, mpc.M*us <= mpc.m]; end
end