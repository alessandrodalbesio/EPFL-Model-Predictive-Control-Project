function [con,obj] = constraints(mpc,N,X,U,xref,uref)    
    % Variables definition
    con = [];
    obj = 0;
    [~,Qf,~] = dlqr(mpc.A,mpc.B,mpc.Q,mpc.R);
    stateConstraints = ~all(all(isnan(mpc.F))) && ~all(all(isnan(mpc.f)));

    % Set the constraints for t=1,...,N-1
    for i=1:N-1
        con = [con, (X(:,i+1)-xref) == mpc.A * (X(:,i)-xref) + mpc.B * (U(:,i)-uref)];
        if stateConstraints
            con = [con,mpc.F*(X(:,i)-xref) <= mpc.f-mpc.F*xref]; 
        end
        con = [con,mpc.M*(U(:,i)-uref) <= mpc.m-mpc.M*uref];
        obj = obj + (X(:,i)-xref)'*mpc.Q*(X(:,i)-xref) + (U(:,i)-uref)'*mpc.R*(U(:,i)-uref);
    end

    % Set the constraints for t=N
    if stateConstraints; con = [con, mpc.F*(X(:,N)-xref) <= mpc.f-mpc.F*xref]; end
    obj = obj + (X(:,N)-xref)'*Qf*(X(:,N)-xref);
end