function [con,obj] = constraints(mpc,X,U,N)   
    stateConstraints = ~all(all(isnan(mpc.F))) && ~all(all(isnan(mpc.f)));
    con = [];
    obj = 0;
    
    % Iterate
    for i=1:N-1
        con = [con,X(:,i+1) == mpc.A * X(:,i) + mpc.B * U(:,i)];
        if stateConstraints
            con = [con,mpc.F*X(:,i) <= mpc.f]; 
        end
        con = [con,mpc.M*U(:,i) <= mpc.m];
        obj = obj + X(:,i)'*mpc.Q*X(:,i) + U(:,i)'*mpc.R*U(:,i);
    end

    % Terminal constraints
    [Ff,ff,Qf] = terminal(mpc);
    con = [con,Ff*X(:,N)<=ff];
    obj = obj + X(:,N)'*Qf*X(:,N);
end