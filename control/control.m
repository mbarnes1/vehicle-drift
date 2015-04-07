function u_plus = control(x,u,pars)

%% Compute desired vals
r_eq = ComputeReq(pars.m,pars.Ux,FyF_eq,FyR_eq);


%% Outer loop
err = x - xeq;
rdes = SideslipController(err(1),req);
xdes(2) = rdes;
err = x - xeq;
[x_plus, u] = InnerLoop(rdes, x, err, pars);

end

function req = ComputeReq(m,Ux,FyF_eq,FyR_eq)
    req = 1/(m*Ux) * (FyF_eq + FyR_eq); % static?
end

function rdes = SideslipController(ebeta, req)
    rdes = req + Kbeta * ebeta;
end
