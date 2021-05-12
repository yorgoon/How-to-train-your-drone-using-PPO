function interpol_u = linear_interpol(nominal_t, nominal_u,t)
% ts = nominal_t(1:end-1);
% us = nominal_u';
if t == 0
    t_tmp = 0;
    interpol_u = nominal_u(1,:);
else
    t_tmp = nominal_t(t>nominal_t);
    k = length(t_tmp);
    if k == length(nominal_t)
        interpol_u = nominal_u(end);
    else
        interpol_u = nominal_u(k,:) + (t-t_tmp(end))*(nominal_u(k+1,:)-nominal_u(k,:))/(nominal_t(k+1) - t_tmp(end));
    end
end

end

