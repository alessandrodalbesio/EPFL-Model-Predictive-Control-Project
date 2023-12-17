function [obj,con] = ss_constraints(mpc,xs,us,ref)
    % Initialize parameters
    Rs = 1;
    [ny, nx] = size(mpc.C);
    stateConstraints = ~all(all(isnan(mpc.F))) && ~all(all(isnan(mpc.f)));
    
    % Define constraints and cost
    obj = us'*Rs*us;
    con = [eye(nx,nx) - mpc.A, -mpc.B ; mpc.C, 0]*[xs; us] == [zeros(nx,ny);ref];
    if stateConstraints 
        con = [con, mpc.F*xs <= mpc.f]; 
    end
    con = [con, mpc.M*us <= mpc.m];
end