function [Ff,ff,Qf] = terminal(mpc)
    sys = LTISystem('A',mpc.A,'B',mpc.B);
    sys.x.max = mpc.Xmax;
    sys.x.min = mpc.Xmin;
    sys.u.max = mpc.Umax;
    sys.u.min = mpc.Umin;
    sys.x.penalty = QuadFunction(mpc.Q);
    sys.u.penalty = QuadFunction(mpc.R);
    Xf = sys.LQRSet;
    Qf = sys.LQRPenalty.weight;
    Ff = Xf.A;
    ff = Xf.b;
    
    figure;
    if Xf.Dim > 2
        for i=1:Xf.Dim-1
            subplot(ceil((Xf.Dim-1) / 2),2,i)
            Xf.projection(i:i+1).plot();
            grid off;
            title("Terminal set "+mpc.title_plots+" dimensions " + string(i) + " - " + string(i+1));
        end
    else
        Xf.plot();
        title("Terminal set "+mpc.title_plots);
        grid off;
    end
end