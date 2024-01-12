function [con,obj] = constraints(mpc,Q,R,H,X,U,F,f,M,m,Xmax,Xmin,Umax,Umin)
    % Initial constraints
    con = (X(:,2) == mpc.A * X(:,1) + mpc.B * U(:,1));
    if ~all(isnan(M)) && ~all(isnan(m)); con = con + (M*U(:,1) <= m); end
    obj = U(:,1)'*R*U(:,1);
    
    % Iterate
    for i=2:H-1
        con = con + (X(:,i+1) == mpc.A * X(:,i) + mpc.B * U(:,i));
        if ~all(all(isnan(F))) && ~all(all(isnan(f))); con = con + (F*X(:,i) <= f); end
        if ~all(isnan(M)) && ~all(isnan(m)); con = con + (M*U(:,i) <= m); end
        obj = obj + X(:,i)'*Q*X(:,i) + U(:,i)'*R*U(:,i);
    end

    % Terminal constraints
    [Ff,ff,Qf] = terminal(mpc,Q,R,Xmax,Xmin,Umax,Umin);
    con = con + (Ff*X(:,H)<=ff);
    obj = obj + X(:,H)'*Qf*X(:,H);
end