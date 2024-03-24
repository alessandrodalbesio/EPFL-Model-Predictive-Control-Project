function [obj,con] = steady_state_constraints(mpc,xs,us,ref)
    % Initialize parameters
    Rs = 1;
    [nx, ny] = size(mpc.C);
    
    % Define constraints and objective
    obj = us'*Rs*us;
    con = mpc.F*xs <= mpc.f;
    con = [con,mpc.M*us<=mpc.m];
    con = [con, [eye(nx,nx)-mpc.A,-mpc.B;mpc.C,mpc.D]*[xs;us] == [zeros(ny,nx);ref]];
end