%% 1. Configurazione Iniziale
close all;
clc;

% --- PARAMETRI GRAFICI ---
mio_xlim = [4.98, 5.20];
mio_ylim = [-0.5, 1.2]; 
%taskSpace
%mio_xlim = [6, 9];   % Limiti asse X
%mio_ylim = [-15, 15];   % Limiti asse Y (Attenzione: unità diverse potrebbero richiedere scale diverse!)
mio_linewidth = 1.5;      % Spessore linea desiderato

% Definisco la palette di colori standard (es. i 7 colori standard di MATLAB)
% Puoi cambiarla manualmente es: [1 0 0; 0 0 1; ...] per Rosso, Blu, ecc.
colori_standard = lines(7); 

% Caricamento figure
% Assicurati che i file .fig siano nella cartella corrente o nel path
try
    f1 = openfig('coppie.fig', 'invisible'); % 'invisible' evita che flashino sullo schermo
    f2 = openfig('energia.fig', 'invisible');
    f3 = openfig('velocità.fig', 'invisible');
    f4 = openfig('momento.fig', 'invisible');
    figs_sorgente = [f1, f2, f3, f4];
catch
    error('Impossibile trovare uno o più file .fig. Controlla i nomi e il percorso.');
end

%% 2. Creazione della nuova figura combinata
fig_finale = figure('Name', 'Grafici Combinati Standardizzati', 'Color', 'w');
% Ho aumentato leggermente le dimensioni della figura per leggibilità
fig_finale.Position(3:4) = [800, 1000]; 

tl = tiledlayout(4, 1, 'TileSpacing', 'compact', 'Padding', 'compact');
xlabel(tl, 'Tempo (s)');
ylabel(tl, 'Ampiezza');

%% 3. Copia dei dati e Standardizzazione
for i = 1:length(figs_sorgente)
    h_fig = figs_sorgente(i); 
    
    % Trovo gli assi nella figura sorgente
    ax_src = findall(h_fig, 'Type', 'axes');
    
    if isempty(ax_src)
        warning('Nessun asse trovato nella figura %d', i);
        continue;
    end
    % Prendo il primo asse trovato (spesso Simulink ne crea vari nascosti)
    ax_src = ax_src(1); 
    
    % Creo il tile di destinazione
    ax_dest = nexttile(tl);
    
    % Copio le linee
    copyobj(ax_src.Children, ax_dest);
    
    % --- INIZIO STANDARDIZZAZIONE STILE ---
    
    % Trovo tutte le linee appena copiate nel nuovo asse
    % Nota: MATLAB spesso elenca i figli in ordine inverso di creazione (ultimo sopra)
    linee_presenti = findobj(ax_dest, 'Type', 'Line');
    
    % Inverto l'ordine per assegnare i colori nell'ordine logico (Linea 1 -> Colore 1)
    linee_presenti = flipud(linee_presenti); 
    
    for k = 1:length(linee_presenti)
        % Seleziono il colore ciclicamente dalla palette
        idx_colore = mod(k-1, size(colori_standard, 1)) + 1;
        
        % Applico colore e spessore
        linee_presenti(k).Color = colori_standard(idx_colore, :);
        linee_presenti(k).LineWidth = mio_linewidth;
    end
    % --- FINE STANDARDIZZAZIONE STILE ---
    
    % Applicazione limiti
    xlim(ax_dest, mio_xlim);
    
    % Nota: Applicare lo stesso YLim a grafici con unità diverse (Nm, Joule, rad/s)
    % potrebbe renderne alcuni illeggibili. Se succede, commenta la riga sotto.
    if i == 2
        ylim(ax_dest, mio_ylim/5); 
    else
        ylim(ax_dest, mio_ylim); 
    end
    
    % Titoli e griglia
    if ~isempty(ax_src.Title.String)
        title(ax_dest, ax_src.Title.String);
    else
        % Fallback sul nome del file originale se il titolo è vuoto
        [~, nome_file, ~] = fileparts(h_fig.FileName);
        title(ax_dest, nome_file);
    end
    
    grid(ax_dest, 'on');
    legend(ax_dest, 'show', 'Location', 'best'); 
end

%% 4. Pulizia
% Chiudo le figure sorgente
close(figs_sorgente);