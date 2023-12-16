function [con,obj] = constraints(mpc,N,X,U,xref,uref)
    con = [];
    obj = 0;
    % Set the constraints for t=1,...,N-1
    for i=1:N-1
        con = [con, (X(:,i+1)-xref) == mpc.A * (X(:,i)-xref) + mpc.B * (U(:,i)-uref)];
        if ~all(all(isnan(mpc.F))) && ~all(all(isnan(mpc.f))); con = [con,mpc.F*(X(:,i)-xref) <= mpc.f-mpc.F*xref]; end
        if ~all(isnan(mpc.M)) && ~all(isnan(mpc.m)); con = [con,mpc.M*(U(:,i)-uref) <= mpc.m-mpc.M*uref]; end
        obj = obj + (X(:,i)-xref)'*mpc.Q*(X(:,i)-xref) + (U(:,i)-uref)'*mpc.R*(U(:,i)-uref);
    end

    % Set the constraints for t=N
    if ~all(all(isnan(mpc.F))) && ~all(all(isnan(mpc.f))); con = [con, mpc.F*(X(:,N)-xref) <= mpc.f-mpc.F*xref]; end
    obj = obj + (X(:,N)-xref)'*mpc.Q*(X(:,N)-xref);
end