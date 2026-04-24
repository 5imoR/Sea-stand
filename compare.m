function compare(list, name, my_title)
arguments
        list  struct
        name  string = string.empty           
        my_title string = "Grafico" % Opzionale: default "Grafico"
    end
    
figure;
grid on;
hold on;
n=length(list);
for i = 1:n
    
    plot(list(i).time, list(i).signals.values, 'DisplayName', name(i))
end
hold off;
title(my_title)
lgd = legend;
lgd.Location = "best";
lgd.ItemHitFcn = @nascondi_al_click;


%% --- Funzione di Callback Locale ---
function nascondi_al_click(~, event)
    % Questa funzione si attiva ogni volta che clicchi un elemento nella legenda
    % event.Peer è la linea del grafico associata a quel nome
    
    if strcmp(event.Peer.Visible, 'on')
        event.Peer.Visible = 'off'; % Se la linea si vede, nascondila
    else
        event.Peer.Visible = 'on';  % Se è nascosta, mostrala di nuovo
    end
end
end