function [con,obj] = constraints(mpc,N,X,U,xref,uref)
    % Parameters definition
    stateConstraints = ~all(all(isnan(mpc.F))) && ~all(isnan(mpc.f));

    % Define constraints and objectives
    con = [];
    obj = 0;

    % Set the constraints for t=1...N-1
    for i=1:N-1
        con = [con, X(:,i+1) == mpc.A * X(:,i) + mpc.B * U(:,i)];
        if stateConstraints; con = [con,mpc.F*X(:,i) <= mpc.f]; end
        con = [con,mpc.M*U(:,i) <= mpc.m];
        obj = obj + (X(:,i)-xref)'*mpc.Q*(X(:,i)-xref) + (U(:,i)-uref)'*mpc.R*(U(:,i)-uref);
    end

    % Set the constraints for t=N
    if stateConstraints
        con = [con,mpc.F*X(:,N) <= mpc.f];
    end
    [~,Qf,~] = dlqr(mpc.A,mpc.B,mpc.Q,mpc.R);
    obj = obj + (X(:,N)-xref)'*Qf*(X(:,N)-xref);
end