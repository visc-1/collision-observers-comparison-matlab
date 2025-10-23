%% Francesco Viscione - Simulation of Robot Collision Detection with proprioceptive sensors
clc;
clear;
close all;

%% 0. SETTAGGI RAPIDI
record_video = false;
trajectory = 'jointSpace'; % 'jointSpace' or 'taskSpace'

%% 1. SETUP E IMPORTAZIONE DEL ROBOT
fprintf('1. Caricamento del robot...\n');

% Carico un KUKA LBR iiwa dal Robotics System Toolbox
robot = loadrobot('kukaIiwa7', 'DataFormat', 'column');
robot.Gravity = [0, 0, -9.81];
showdetails(robot);

numJoints = numel(robot.homeConfiguration);
endEffectorName = 'iiwa_link_ee'; % Nome dell'end-effector


%% 2. DEFINIZIONE DELLA TRAIETTORIA
fprintf('2. Definizione della traiettoria...\n');
fprintf('Hai selezionato: %s\n', trajectory);

T_period = 5.0;
t_start = 0;
t_end = 2*T_period; % Vogliamo compiere due giri
t_step = 0.01;
t_traj = t_start:t_step:t_end;
if strcmp(trajectory,'jointSpace')
    A = [0.5, 0, 0, 1, 0, 0, 0];
    phi = 0;
    omega = 2 * pi / T_period;


    % pos = A*sin(omega*t + phi)
    % vel = A*omega*cos(omega*t + phi)
    % acc = -A*omega^2*sin(omega*t + phi)
    
    q_desired = zeros(7, numel(t_traj));
    qd_desired = zeros(7, numel(t_traj));
    qdd_desired = zeros(7, numel(t_traj));
    
    for i = 1:length(t_traj)
        t = t_traj(i);
    
        for j = 1:7
            q_desired(j,i) = A(j) * sin(omega*t + phi);
            qd_desired(j,i) = A(j)*omega*cos(omega*t + phi);
            qdd_desired(j,i) = -A(j)*(omega^2)*sin(omega*t + phi);
        end
    
    end

    fprintf('2.1 Calcolo della cinematica diretta...\n');

    cartesian_pose = zeros(4,4, numel(t_traj));
    
    h_waitbar_ik = waitbar(0, 'Risoluzione Cinematica diretta...');
    for i = 1:length(t_traj)
    
        cartesian_pose(:,:,i) = getTransform(robot, q_desired(:,i), endEffectorName);
    
        if mod(i, 20) == 0
            waitbar(i/length(t_traj), h_waitbar_ik);
        end
    end
    close(h_waitbar_ik);

    % Estraiamo i punti nello spazio
    pos_cart = cartesian_pose(1:3, 4, :);

elseif strcmp(trajectory,'taskSpace')
    center = [0.4, 0, 0.8];     % Centro [x, y, z] [m]
    radius = 0.1;               % Raggio [m]
    omega = 2 * pi / T_period;  % Velocità angolare costante [rad/s]

    pos_cart = zeros(2, length(t_traj)); % [y; z]

    for i = 1:length(t_traj)
        t = t_traj(i);
        angle = omega * t; % Angolo sulla circonferenza
    
        % Posizione
        pos_cart(1, i) = center(2) + radius * cos(angle); % y(t)
        pos_cart(2, i) = center(3) + radius * sin(angle); % z(t)
    end
    
    pos_cart = [center(1)*ones(1,length(t_traj));pos_cart]; % Aggiungiamo x(t)
    
    % Costruisco la traiettoria in termini di trasformazioni omogenee
    theta_y = 0; % Possibile rotazione del tool rispetto l'asse y
    R_y = [cos(theta_y), 0, sin(theta_y); 0 1 0; -sin(theta_y), 0, cos(theta_y)];
    T_traj = zeros(4,4,length(t_traj));
    for i = 1:length(t_traj)
        T_traj(:,:,i) = [R_y, pos_cart(:, i); 0 0 0 1]; % Trasformazione completa
    end
    
    % Cinematica inversa
    fprintf('2.1. Calcolo della cinematica inversa...\n');
    ik = inverseKinematics('RigidBodyTree', robot);
    ik.SolverParameters.AllowRandomRestart = false;
    weights = [0.25 0.25 0.25 1 1 1]; % Pesi per la cinematica inversa numerica
    
    q0 = robot.homeConfiguration;
    q_desired = zeros(numJoints, length(t_traj));
    q_prev = q0;
    
    h_waitbar_ik = waitbar(0, 'Risoluzione Cinematica Inversa...');
    for i = 1:length(t_traj)
        q_sol = ik(endEffectorName, T_traj(:,:,i), weights, q_prev);
        q_desired(:,i) = q_sol;
        q_prev = q_sol;
    
        if mod(i, 20) == 0
            waitbar(i/length(t_traj), h_waitbar_ik);
        end
    end
    close(h_waitbar_ik);
    
    % Calcola velocità e accelerazioni dei giunti desiderate mediante
    % derivazione numerica
    qd_desired = gradient(q_desired, t_step);
    qdd_desired = gradient(qd_desired, t_step);
    
    % Check per vedere se le accelerazioni ottenute numericamente risultano
    % sporche
    % figure('Name', 'Accelerazioni desiderate', 'NumberTitle', 'off');
    % plot(t_traj, qdd_desired', 'LineWidth', 1.5);
    % title('Accelerazioni desiderate ottenute mediante derivazione numerica');
    % xlabel('Tempo (s)');
    % ylabel('Accelerazione (rad/(s^2))');
    % grid on;
    % legend('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7');
else
    error('Select a jointSpace or taskSpace trajectory...');
end

%% Opzionale: Visualizzazione del robot nella sua configurazione "home" assieme alla traiettoria definita
figure('Name', 'Configurazione Iniziale del Robot', 'NumberTitle', 'off');
show(robot, robot.homeConfiguration);
title('KUKA LBR iiwa - Configurazione Iniziale');
axis([-0.5 1 -0.75 0.75 -0.2 1.5]);
hold on;
plot3(pos_cart(1,:), pos_cart(2,:), pos_cart(3,:), 'r', 'LineWidth', 1.5, 'DisplayName', 'Traiettoria Desiderata');
legend show;

%% 4. DEFINIZIONE DEI PARAMETRI UTILI NELLA SIMULAZIONE DINAMICA
% Durante la simulazione dinamica il braccio robotico viene sottoposto ad
% una coppia di feedback generata da un controllore PD. Questo ci permette
% di compensare le incertezze del modello e i disturbi esterni, garantendo
% la stabilità e la convergenza all'errore nullo.
% Inoltre durante il secondo giro il robot andrà ad impattare con
% un piano che simula la presenza di un essere umano. Essendo cedevole ho
% modellato l'interazione mediante il modello di un oscillatore smorzato
% vincolato alla sua dinamica in compressione. La funzione che descrive
% la dinamica è robotDynamicsODE.
fprintf('4. Definizione parametri simulazione...\n');

% Definiamo i guadagni del controllore PD di feedback
Kp = diag(100 * ones(1, numJoints)); % Guadagno proporzionale
Kd = diag(20 * ones(1, numJoints));  % Guadagno derivativo

% Definiamo un piano che rappresenta il contatto umano.

if strcmp(trajectory,'jointSpace')
    plane_point = [-0.3; -0.17; 1.1]; % Punto di applicazione del vettore
elseif strcmp(trajectory,'taskSpace')
    plane_point = [center(1); -radius*0; center(3)];
end

plane_normal = [0; 1; 0]; % Vettore normale al piano

% Usiamo come modello di interazione umano-robot un Molla-Smorzatore
K_human = 5000; % Rigidità [N/m]
C_human = 100;  % Smorzamento [Ns/m]

% Nome del corpo che entra in collisione e sul quale verrà applicata la
% forza
body_name_for_collision = endEffectorName;

% guadagno del residuo
k0 = 25;


%% 5. SIMULAZIONE CON DINAMICA DIRETTA IN ANELLO CHIUSO
fprintf('5. Simulazione della dinamica diretta in anello chiuso...\n');

% Condizioni iniziali per la simulazione [posizione; velocità; residuo], supponiamo
% parta già agganciato alla traiettoria
x0 = [q_desired(:,1); qd_desired(:,1); 0];

% Creo la funzione che descrive la dinamica e la funzione di
% logging per estrapolare le coppie attuate sul manipolatore durante la
% simulazione
[odefun, getLoggedTorques] = createDynamicsWithLogging(robot, ...
                                 t_traj, q_desired, qd_desired, qdd_desired, ...
                                 Kp, Kd, ...
                                 body_name_for_collision, plane_point, plane_normal, K_human, C_human, k0);


% Variabili per la waitbar della simulazione
t_final = t_traj(end);
h_waitbar = waitbar(0, 'Avvio simulazione ODE...', 'Name', 'Avanzamento Simulazione');


rel_tol = 1e-5; % Tolleranza relativa (più stretta del default 1e-3)
abs_tol = 1e-7; % Tolleranza assoluta (più stretta del default 1e-6)

% Crea la struttura di opzioni per il risolutore ODE
% Specifichiamo che la funzione 'odeProgressBar' deve essere chiamata
% durante l'esecuzione.
options = odeset('OutputFcn', @(t,y,flag) odeProgressBar(t, y, flag, h_waitbar, t_final), ...
    'RelTol',rel_tol, 'AbsTol',abs_tol);

% Risolvo le ODE per ottenere la dinamica simulata. Ho utilizzato ode15s in
% quanto anche se prevede l'utilizzo di metodi impliciti, il tempo
% necessario a effettuare i calcoli risulta comunque minore di quello
% riscontrato con ode45. Sembra che l'enorme quantità di tempo impiegata da
% ode45 sia causata dal fatto che per ridurre l'errore ad ogni passo, la
% funzione vada a ridurre la dimensione di ogni step innumerevoli volte,
% rendendo così gigantesco il numero di passi.

%[t_sim, x_sim] = ode15s(odefun, t_traj, x0, options);
[t_sim, x_sim] = ode15s(odefun, [t_start, t_end], x0, options);

% Ottengo le coppie attuate nella simulazione dalla funzione di logging
[t_tau_logged, tau_logged, tau_ext_logged, M_logged] = getLoggedTorques();
[~, unique_indices] = unique(t_tau_logged, 'last');
sorted_unique_indices = sort(unique_indices);

% Rimuovo istanti di tempo duplicati causati dal risolutore
if length(sorted_unique_indices) < length(t_tau_logged)
    fprintf('Attenzione: %d punti temporali duplicati trovati e rimossi dal logger.\n', ...
            length(t_tau_logged) - length(sorted_unique_indices));
end

t_tau_unique = t_tau_logged(sorted_unique_indices);
tau_unique = tau_logged(:, sorted_unique_indices);
tau_ext_unique = tau_ext_logged(:, sorted_unique_indices);
M_unique = M_logged(:,:,sorted_unique_indices);

% Interpolo le coppie sui tempi della simulazione
tau_sim = interp1(t_tau_unique, tau_unique', t_sim)';
tau_ext_sim = interp1(t_tau_unique, tau_ext_unique', t_sim)';

% Per interpolare un'array di matrici dobbiamo prima trsformarlo in un
% array di vettori
M_unique_reshaped = reshape(M_unique, 7*7, length(t_tau_unique));
M_sim_reshaped = interp1(t_tau_unique, M_unique_reshaped', t_sim)';
M_sim = reshape(M_sim_reshaped, 7, 7, length(t_sim));

% Per calcolare la derivata della matrice di inerzia dobbiamo avere come
% prima dimensione quella temporale
M_sim_permuted = permute(M_sim, [3, 1, 2]);
Md_sim_permuted = gradient(M_sim_permuted);
Md_sim = ipermute(Md_sim_permuted, [3, 1, 2]);


% Estraggo posizioni e velocità simulate
q_sim = x_sim(:, 1:numJoints)';
qd_sim = x_sim(:, numJoints+1:end-1)';
r_sim = x_sim(:, end:end);

fprintf('Simulazione completata.\n');


%% 6. VISUALIZZAZIONE E CONFRONTO DEI RISULTATI
fprintf('6. Visualizzazione dei risultati...\n');

% Confronto tra posizioni desiderate e simulate
figure('Name', 'Confronto Posizioni Giunti', 'NumberTitle', 'off');
tiledlayout(2,1);

% Grafico posizioni
nexttile;
plot(t_traj, q_desired', '--');
hold on;
plot(t_sim, q_sim', '-');
hold off;
title('Confronto Posizioni Giunti: Desiderate (--) vs. Simulate (-)');
xlabel('Tempo (s)');
ylabel('Angolo (rad)');
grid on;
legend('q1des','q2des','q3des','q4des','q5des','q6des','q7des', ...
       'q1sim','q2sim','q3sim','q4sim','q5sim','q6sim','q7sim');

% Grafico errore di posizione
nexttile;
% è necessario interpolare q_desired sui tempi di t_sim per un calcolo corretto
q_desired_interp = interp1(t_traj, q_desired', t_sim)';
error = q_desired_interp - q_sim;
plot(t_sim, error');
title('Errore di Posizione tra Desiderate e Simulate');
xlabel('Tempo (s)');
ylabel('Errore (rad)');
grid on;
legend('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7');

% Confronto tra Velocità desiderate e simulate
figure('Name', 'Confronto Velocità Giunti', 'NumberTitle', 'off');
tiledlayout(2,1);

% Grafico Velocità
nexttile;
plot(t_traj, qd_desired', '--');
hold on;
plot(t_sim, qd_sim', '-');
hold off;
title('Confronto Velocità Giunti: Desiderate (--) vs. Simulate (-)');
xlabel('Tempo (s)');
ylabel('Velocità Angolare (rad/s)');
grid on;
legend('qd1des','qd2des','qd3des','qd4des','qd5des','qd6des','qd7des', ...
       'qd1sim','qd2sim','qd3sim','qd4sim','qd5sim','qd6sim','qd7sim');

% Grafico errore di velocità
nexttile;
% è necessario interpolare q_desired sui tempi di t_sim per un calcolo corretto
qd_desired_interp = interp1(t_traj, qd_desired', t_sim)';
error = qd_desired_interp - qd_sim;
plot(t_sim, error');
title('Errore di Velocirà tra Desiderate e Simulate');
xlabel('Tempo (s)');
ylabel('Errore (rad/s)');
grid on;
legend('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7');

% Grafico delle coppie attuate
figure('Name', 'Coppie Applicate ai Giunti', 'NumberTitle', 'off');
plot(t_sim, tau_sim');
title('Coppie Totali Applicate ai Giunti (\tau_{ff} + \tau_{pd})');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
grid on;
legend('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7');

% Grafico delle coppie di disturbo
figure('Name', 'Coppie di disturbo applicate ai Giunti', 'NumberTitle', 'off');
plot(t_sim, tau_ext_sim');
title('Coppie di disturbo applicate ai Giunti \tau_{ext}');
xlabel('Tempo (s)');
ylabel('Coppia (Nm)');
grid on;
legend('G1', 'G2', 'G3', 'G4', 'G5', 'G6', 'G7');

% Grafico del residuo calcolato conoscendo tau_ext
figure('Name', 'Residuo ideale', 'NumberTitle', 'off');
plot(t_sim, r_sim');
title('Residuo calcolato conoscendo \tau_{ext}');
xlabel('Tempo (s)');
ylabel('Residuo');
grid on;

%% Animazione del robot

if record_video
    video_filename='simulazione_robot.avi';
    writerObj = VideoWriter(video_filename);
    writerObj.FrameRate = 100;
    writerObj.Quality = 50;
    open(writerObj);
end

fig_anim = figure('Name', 'Animazione della Dinamica Simulata', 'NumberTitle', 'off');
show(robot, homeConfiguration(robot));
hold on;

if strcmp(trajectory,'jointSpace')
    axis([-0.5 0.6 -0.5 0.5 0 1.5]);
elseif strcmp(trajectory,'taskSpace')
    axis([-0.3 0.6 -0.5 0.5 0 1.2]);
end


grid on;
plot3(pos_cart(1,:), pos_cart(2,:), pos_cart(3,:), 'r', 'LineWidth', 1.5, 'DisplayName', 'Traiettoria Desiderata');
%view(90,0);

plane_presence = false;

if record_video
    step = 1;
else
    step = 20;
end

for i = 1:step:length(t_sim)
    show(robot, q_sim(:,i), 'PreservePlot', false);
    title(sprintf('Animazione Simulazione - Tempo: %.2f s', t_sim(i)));
    drawnow;

    if t_sim(i)>= 5.0 && plane_presence == false
        % Disegniamo un rettangolo per rappresentare il piano
        if strcmp(trajectory,'jointSpace')
            patch_x = [0.5, 0.5, -0.5, -0.5];
            patch_z = [0.8, 1.2, 1.2, 0.8];
        elseif strcmp(trajectory,'taskSpace')
            patch_x = [0.3, 0.5, 0.5, 0.3];
            patch_z = [0.6, 0.6, 1, 1];
        end
        patch_y = [plane_point(2), plane_point(2), plane_point(2), plane_point(2)];

        
        patch(patch_x, patch_y, patch_z, 'g', 'FaceAlpha', 1, 'EdgeColor', 'none', 'DisplayName', 'Piano di Collisione');
        plane_presence=true;
    end
    
    if record_video
        frame = getframe(fig_anim);
        writeVideo(writerObj, frame);
    end
end

if record_video
    close(writerObj);
end

%% 7.CALCOLO DELL'ENERGIA MECCANICA DEL ROBOT
fprintf('7. Calcolo dell''energia meccanica lungo la traiettoria...\n');

% Calcoliamo l'energia potenziale per ogni istante della simulazione
PE_sim = zeros(1, length(t_sim)); % Energia Potenziale
h_waitbar_pe = waitbar(0, 'Calcolo Energia Potenziale...');
for i = 1:length(t_sim)
    PE_sim(i) = calculatePotentialEnergy(robot, q_sim(:,i));

    % Aggiorno la waitbar
    if mod(i, 20) == 0
        waitbar(i/length(t_sim), h_waitbar_pe);
    end
end
close(h_waitbar_pe);

KE_sim = zeros(1, length(t_sim)); % Energia Cinetica
h_waitbar_ke = waitbar(0, 'Calcolo Energia Cinetica...');
for i = 1:length(t_sim)
    % KE = 0.5 * qd' * M(q) * qd
    M = massMatrix(robot, q_sim(:,i));
    qd = qd_sim(:,i);
    KE_sim(i) = 0.5 * qd' * M * qd;

    % Aggiorno la waitbar
    if mod(i, 20) == 0
        waitbar(i/length(t_sim), h_waitbar_ke);
    end
end
close(h_waitbar_ke);

% Energia Meccanica Totale
E_total_sim = PE_sim + KE_sim;

%% 8. Visualizzazione dei i risultati energetici
fprintf('8. Visualizzazione dei i risultati energetici...\n');
figure('Name', 'Analisi Energetica del Robot', 'NumberTitle', 'off');
tiledlayout(3,1);

nexttile;
plot(t_sim, PE_sim, 'b-', 'DisplayName', 'Energia Potenziale (PE)');
xlabel('Tempo (s)');
ylabel('Energia (Joule)');
grid on;
legend show;

nexttile;
plot(t_sim, KE_sim, 'r-', 'DisplayName', 'Energia Cinetica (KE)');
xlabel('Tempo (s)');
ylabel('Energia (Joule)');
grid on;
legend show;

nexttile;
plot(t_sim, E_total_sim, 'g-', 'DisplayName', 'Energia Meccanica (PE+KE)');
xlabel('Tempo (s)');
ylabel('Energia (Joule)');
grid on;
legend show;

%% 9. Calcolo del residuo per il monitoraggio di collisioni Osservartore dell'Energia
fprintf("9. Calcolo del residuo r(t) mediante Osservatore dell'Energia...\n");

r_energy_observer = zeros(size(t_sim));
E_0 = E_total_sim(1);
integral_term = 0;

% Calcolo del residuo per il monitoraggio dell'energia
h_waitbar_r = waitbar(0, 'Calcolo del residuo r(t)...');
for i = 1:length(t_sim)
    r_energy_observer(i) = k0 * (E_total_sim(i) - integral_term - E_0);
    
    if i < length(t_sim)
        dt = t_sim(i+1) - t_sim(i);
        % Aggiornamento del termine integrale
        integral_term = integral_term + (qd_sim(:,i)'*tau_sim(:,i) + r_energy_observer(i))*dt;

    end

    if mod(i, 20) == 0
        waitbar(i/length(t_sim), h_waitbar_r);
    end
end
close(h_waitbar_r);

% Calcolo del valore di treshold in funzione della potenza dei motori
treshold_base_value = 0.5;
treshold_values = zeros(size(t_sim));
k_power = 0.05;
for i=1:length(t_sim)
    treshold_values(i) = treshold_base_value + k_power*abs(qd_sim(:,i)'*tau_sim(:,i));
end
%% 10. Visualizzazione del residuo
fprintf("10. Visualizzazione del residuo...\n");
figure('Name', 'Segnale di Monitoraggio Collisioni', 'NumberTitle', 'off');
plot(t_sim, r_energy_observer, 'r-', 'DisplayName', 'r(t)');
title('Residuo r(t)');
xlabel('Tempo (s)');
ylabel('Residuo r(t)');
% hold on;
% plot(t_sim, treshold_values, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Soglia Superiore');
% hold on;
% plot(t_sim, -treshold_values, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Soglia Inferiore');
grid on;
legend show;

%% 11. Preparazioni variabili per simulink
q_simulink = [t_sim, q_sim'];
qd_simulink = [t_sim, qd_sim'];
tau_simulink = [t_sim, tau_sim'];
Md_simulink = timeseries(Md_sim, t_sim);
%% FUNZIONI AUSILIARIE
function [odefun, getLogData] = createDynamicsWithLogging(robot, t_traj, q_des, qd_des, qdd_des, Kp, Kd, ...
                                                           body_name, plane_point, plane_normal, K, C, K_res)
    % variabili di logging
    t_log = [];
    tau_applied_log = [];
    tau_ext_log = [];
    M_log = [];
    
    % Restituisce due handle:
    % 1. @robotDynamicsODE: la funzione da passare a ode45
    % 2. @retrieveLogData: una funzione per recuperare i dati loggati dopo la simulazione
    odefun = @robotDynamicsODE;
    getLogData = @retrieveLogData;
    
    % Funzione annidata con accesso alle variabili della funzione
    % che la contiente
    function dxdt = robotDynamicsODE(t,x)
        numJ = 7; % il kukaIiwa ha 7 giunti

        q_real = x(1:numJ);
        qd_real = x(numJ+1:end-1);
        r_real = x(end);
    
        % Trovo lo stato desiderato al tempo 't' interpolando le traiettorie
        q_target = interp1(t_traj, q_des', t)';
        qd_target = interp1(t_traj, qd_des', t)';
        qdd_target = interp1(t_traj, qdd_des', t)';
    
        % --- LEGGE DI CONTROLLO DEL MANIPOLATORE ---
        % 1. Termine di Feed-forward: coppia teorica per la traiettoria desiderata
        tau_ff = inverseDynamics(robot, q_target, qd_target, qdd_target);
    
        % 2. Termine di Feedback: correzione PD basata sull'errore
        error_q = q_target - q_real;   % Errore di posizione
        error_qd = qd_target - qd_real; % Errore di velocità
        tau_feedback = Kp * error_q + Kd * error_qd;
    
        % 3. Coppia totale da applicare al robot
        tau_applied = tau_ff + tau_feedback;
        

        % Definizione dei tempi in cui l'essere umano è presente
        f_ext = zeros(6, robot.NumBodies);
        human_presence_start = 5;
        human_presence_end = 10;

        if t>=human_presence_start && t<=human_presence_end

            % Ottengo posizione e velocità attuali dell'ee
            T_ee = getTransform(robot, q_real, body_name);
            pos_ee = T_ee(1:3, 4);

            J = geometricJacobian(robot, q_real, body_name);
            vel_ee_twist = J * qd_real; % [angolare; lineare]
            vel_ee_linear = vel_ee_twist(4:6);

            % La distanza rispetto al piano è data dal prodotto scalare tra il
            % vettore che collega il punto del piano all'ee e il vettore
            % normale al piano
            penetration_depth = -dot(pos_ee - plane_point, plane_normal);

            if penetration_depth > 0
                vel_normal_component = dot(vel_ee_linear, plane_normal);

                % Calcolo la forza usando Massa-Smorzatore (solo in compressione)
                force_magnitude = K*penetration_depth + C*vel_normal_component;
                force_magnitude = max(0, force_magnitude); % Per assicurarmi sia solo in compressione

                force_vector = force_magnitude * plane_normal;

                wrench_vector = [0; 0; 0; force_vector(1); force_vector(2); force_vector(3)];

                f_ext = externalForce(robot, body_name, wrench_vector, q_real);

            end

        end
    
        % Calcola l'accelerazione risultante usando la dinamica diretta con
        % la coppia totale e l'eventuale forza esterna causata dall'impatto
        qdd_real = forwardDynamics(robot, q_real, qd_real, tau_applied, f_ext);

        M = massMatrix(robot, q_real);
        C_qd = velocityProduct(robot,q_real, qd_real);
        G = gravityTorque(robot,q_real);
        tau_ext = M*qdd_real + C_qd + G - tau_applied;

        rd_real = K_res*(qd_real' * tau_ext - r_real);

        % Logging dei dati
        t_log(end+1) = t;
        tau_applied_log(:, end+1) = tau_applied(:);
        tau_ext_log(:, end+1) =  tau_ext;
        M_log(:,:,end+1) = M;

        % Ritorno la derivata dello stato [velocità_reale; accelerazione_reale; derivata del residuo]
        dxdt = [qd_real; qdd_real; rd_real];

    end
    
    % Funzione annidata per il recupero delle coppie comandate
    function [t, tau_applied, tau_ext, M] = retrieveLogData()

        t = t_log;
        tau_applied = tau_applied_log;
        tau_ext = tau_ext_log;
        M = M_log;
    end

end

% --- FUNZIONE AUSILIARIA PER LA WAITBAR ---

function status = odeProgressBar(t, ~, flag, h_waitbar, t_final)
    % Questa funzione viene chiamata da ode45.
    % t: vettore dei tempi attuali
    % y: vettore dello stato attuale (non lo usiamo, quindi '~')
    % flag: indica lo stato del risolutore ('init', 'done', o vuoto)

    % La funzione deve restituire uno 'status' (0 per continuare, 1 per fermare)
    status = 0;
    
    % Controllo se l'handle della waitbar è ancora valido.
    % Se l'utente ha chiuso la finestra, isgraphics(h_waitbar) sarà falso.
    if ~isgraphics(h_waitbar)
        status = 1; % Imposto lo status a 1 per fermare l'ODE
        fprintf('\nSimulazione interrotta dall''utente.\n');
        return;
    end

    switch flag
        case 'init' % Chiamata all'inizio della simulazione
            
        case '' % Chiamata durante la simulazione (dopo ogni passo riuscito)
            % Aggiorno la waitbar con la frazione di tempo completata.
            % 't' può essere un vettore, quindi prendo l'ultimo valore di tempo.
            waitbar(t(end) / t_final, h_waitbar, ...
                sprintf('Simulazione in corso... Tempo: %.2f / %.2f s', t(end), t_final));
            
        case 'done' % Chiamata alla fine della simulazione
            % Chiudo la finestra della waitbar
            close(h_waitbar);
    end
end

% -- FUNZIONE PER CALCOLARE L'ENERGIA POTENZIALE TOTALE --
function total_pe = calculatePotentialEnergy(robot, q)
    % Calcola l'energia potenziale totale del robot per una data configurazione q.
    % PE_total = sum(m_i * g * h_i)

    total_pe = 0;

    gravity_vec = robot.Gravity;
    g = abs(gravity_vec(3));

    if g == 0
        % Se non c'è gravità, l'energia potenziale è zero
        return;
    end
    
    % Itero su ogni corpo rigido del robot
    for i = 1:robot.NumBodies
        body = robot.Bodies{i};
        
        % Ignoro i corpi con massa nulla
        if body.Mass > 0
            % 1. Ottiengo la trasformazione dal frame base al frame del corpo i
            T_body = getTransform(robot, q, body.Name);
            
            % 2. Le coordinate del CoM sono definite nel frame del corpo stesso
            com_local = body.CenterOfMass';
            
            % 3. Trasformo le coordinate locali del CoM in coordinate del mondo
            com_world_homogeneous = T_body * [com_local; 1];
            com_world = com_world_homogeneous(1:3);
            
            h_com = com_world(3);
            
            % 5. Calcolo l'energia potenziale e sommo
            pe_link = body.Mass * g * h_com;
            total_pe = total_pe + pe_link;
        end
    end
end