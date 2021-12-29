function out = stop_run(iter,obj,S)

% global final_iter
% k = 1;
% cleanupObj = onCleanup(@() cleanMeUp(k));
% out = true;
% % try
% tic
% pause;
% keyboard;
% time = toc;
% tic
% t_c = toc;
% if time<0.003
%     final_iter = S.x;
%     out = false;
% end
% %     return
% %     out = true;
% % catch
% %     out = false;
% %     fprintf('failed\n.')
% % end
% k = 0;
% cleanupObj = onCleanup(@() cleanMeUp(k));
% % out = true;
% % if iter==5
% %     out = false;
% % end
% % if k==0
% %     out = false;
% % end
% 
%     function cleanMeUp(k)
%         % saves data to file (or could save to workspace)
% %         fprintf('saving variables to file...\n');
%         if k==1
% %             fprintf('TERMINATED\n');
%         end


 global flag
 out = true;
 try
 if ishandle(flag.fig)
     flag.x = [flag.x;iter];
     flag.y11 = [flag.y11;obj];
     ax = get(flag.pl1,'parent');
     set(ax,'Xlim',[0,iter+1])
     set(flag.pl1,'XData',flag.x,'YData',flag.y11)
     drawnow
%      set(flag.p21,'XData',x,'YData',y21)
%      set(flag.p22,'XData',x,'YData',y22)
 else
     fprintf('- figure closed.\n')
      out = false;
 end
 catch
     fprintf('- error encountered.\n')
      out = false;
 end