goal = [-20; 7];
plot(goal(1),goal(2),'rx')
hold on

n_vert = 8;
obs_list = zeros(2,n_vert);
obs_x = 10;
obs_y0 = 7;
for k = 1:n_vert
  obs_list(:,end+1) = [obs_x;obs_y0+k];
end

obs_list(:,end+1) = [obs_x; obs_y0+n_vert+2];

obs_x0 = 5;
obs_y = 15;
n_horz = 4;

for k = 1:n_horz
  obs_list(:,end+1) = [obs_x0+k; obs_y];
end

obs_list(:,end+1) = [4;15];
obs_list(:,end+1) = [5;14];
%obs_list(:,end+1) = [10;6];
obs_list(:,end+1) = [10;5];
obs_list(:,end+1) = [10;4];

for k = 1:size(obs_list,2)
  plot(obs_list(1,k),obs_list(2,k),'ko')
end

botxy = [18;9];
%bot_dir_filt = [0;0];
Frep_filt = [0;0];
alpha = 0.5;

c_attr = 2;
max_Fattr = 10;

for k = 1:250
  plot(botxy(1),botxy(2),'mo')
  pause(0.02);
  Fattr = c_attr*(goal-botxy);
  if(norm(Fattr) > max_Fattr)
    Fattr = Fattr/norm(Fattr)*max_Fattr;
  end

  Frep = get_repel2(obs_list, botxy, Fattr);
  %if(dot(Fattr,Frep) < 0)
  %  if(norm(Fattr) > norm(Frep)/2)
  %    Fattr = Fattr/norm(Fattr)*norm(Frep)/4;
  %  end
  %end
  
  % Try listening to Frep more and ignore Fattr if Frep is large
  %if(dot(Fattr,Frep) < 0)
  %  if(norm(Frep) > norm(Fattr)*1)
  %    Fattr = [0;0];
  %  end
  %end
  
  Frep_filt = alpha*Frep_filt + (1-alpha)*Frep;
  
  F = Fattr + Frep_filt;
  if(norm(F) > 0)
    bot_dir = F/norm(F);
  else
    bot_dir = Fattr/norm(Fattr);
  end
  %bot_dir_filt = alpha*bot_dir_filt + (1-alpha)*bot_dir;
  
  botxy = botxy + bot_dir*0.2;
  if(norm(botxy-goal) < 0.5)
    break;
  end
end

axis equal