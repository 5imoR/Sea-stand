function compare_plots(var1, var2)

figure;
grid on;
hold on;
 
stairs(var1.time, var1.signals.values, '--r', 'LineWidth',1.5)
stairs(var2.time, var2.signals.values, 'b', 'LineWidth',1.5)
%stairs(var3.time, var3.signals.values, 'g', 'LineWidth',1.5);

hold off;
lgd = legend( "Nominal Tracking", "Robust");
lgd.Location = "best";
% lgd.ItemHitFcn = @nascondi_al_click;

% 
% %% --- Funzione di Callback Locale ---
% function nascondi_al_click(~, event)
%     % Questa funzione si attiva ogni volta che clicchi un elemento nella legenda
%     % event.Peer è la linea del grafico associata a quel nome
% 
%     if strcmp(event.Peer.Visible, 'on')
%         event.Peer.Visible = 'off'; % Se la linea si vede, nascondila
%     else
%         event.Peer.Visible = 'on';  % Se è nascosta, mostrala di nuovo
%     end
% end