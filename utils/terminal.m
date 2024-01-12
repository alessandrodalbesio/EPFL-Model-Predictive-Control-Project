function [Ff,ff,Qf] = terminal(mpc,Q,R,Xmax,Xmin,Umax,Umin)
    sys = LTISystem('A',mpc.A,'B',mpc.B);
    sys.x.max = Xmax;
    sys.x.min = Xmin;
    sys.u.max = Umax;
    sys.u.min = Umin;
    sys.x.penalty = QuadFunction(Q);
    sys.u.penalty = QuadFunction(R);
    Qf = sys.LQRPenalty.weight;
    Ff = sys.LQRSet.A;
    ff = sys.LQRSet.b;
end