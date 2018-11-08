function Frep = get_repel2(obs_list, botxy, Fattr)
  d0 = 3; %4
  Frep = [0;0];
  c_rep = 100; %30-60
  
  n_obs = size(obs_list,2);
  for k = 1:n_obs
    obs = obs_list(:,k);
    obs_vect = botxy-obs;
    d = norm(obs_vect);
    
    if(d < d0)
      Fobs = c_rep*(1/d^2 - 1/d0^2)*(1/d^4)*obs_vect;
    else
      Fobs = [0;0];
    end
    if(false)
      if(get_cos(obs_vect, Fattr) > 0.9)
        Frep = Frep + Fobs;
      else
        Ftan = -get_tan(Fobs);
        if(get_cos(Ftan,Fattr) < -0.9)
          Ftan = -Ftan;
        end
        Frep = Frep + Ftan;
      end
    else
      Ftan = -get_tan(Fobs);
      Frep = Frep + Ftan;
    end
    
    %OR
    %Frep = Frep + Fobs; %fails from local minima
  end
    
end

function vt = get_tan(v)
  vt = [-v(2); v(1)];
end

function c = get_cos(a,b)
  c = dot(a,b)/norm(a)/norm(b);
end