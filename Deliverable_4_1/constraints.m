function [con,obj] = constraints(mpc,N,X,U,xref,uref)
    % Parameters definition
    useSlacks = 1;
    stateConstraints = ~all(all(isnan(mpc.F))) && ~all(isnan(mpc.f));
    
    % Define the slack variables
    useSlacks = stateConstraints && useSlacks;
    if useSlacks; eps = sdpvar(size(mpc.f,1), N); end

    % Define constraints and objectives
    con = [];
    obj = 0;

    % Set the constraints for t=1...N-1
    for i=1:N-1
        con = [con, (X(:,i+1)-xref) == mpc.A * (X(:,i)-xref) + mpc.B * (U(:,i)-uref)];
        if stateConstraints
            if useSlacks
                con = [con,mpc.F*(X(:,i)-xref) <= mpc.f-mpc.F*xref-eps(:,i)]; 
            else
                con = [con,mpc.F*(X(:,i)-xref) <= mpc.f-mpc.F*xref];
            end
        end
        con = [con,mpc.M*(U(:,i)-uref) <= mpc.m-mpc.M*uref];
        if useSlacks
            obj = obj + (X(:,i)-xref)'*mpc.Q*(X(:,i)-xref) + (U(:,i)-uref)'*mpc.R*(U(:,i)-uref) + eps(:,i)'*mpc.S*eps(:,i);
        else
            obj = obj + (X(:,i)-xref)'*mpc.Q*(X(:,i)-xref) + (U(:,i)-uref)'*mpc.R*(U(:,i)-uref);
        end
    end

    % Set the constraints for t=N
    if stateConstraints
        if useSlacks
            con = [con,mpc.F*(X(:,N)-xref) <= mpc.f-mpc.F*xref-eps(:,N)]; 
        else
            con = [con,mpc.F*(X(:,N)-xref) <= mpc.f-mpc.F*xref];
        end
    end
    [~,Qf,~] = dlqr(mpc.A,mpc.B,mpc.Q,mpc.R);
    if useSlacks
        obj = obj + (X(:,N)-xref)'*Qf*(X(:,N)-xref) + eps(:,N)'*mpc.S*eps(:,N);
    else
        obj = obj + (X(:,N)-xref)'*Qf*(X(:,N)-xref);
    end
end