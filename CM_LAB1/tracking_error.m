clear; clc;

% --- PARAMETRI ---
% Imposta qui il set-point (40, 70 o 120)
ref = 120; 

% --- CARICAMENTO DATI ---
% 1. Simulazione Ideale (Senza attrito)
% Assegnando la load a una variabile, evitiamo di sovrascrivere "y"
data_NoFR = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\120NoFR.mat');
y_ideal = data_NoFR.y;

% 2. Simulazione Reale (Con attrito)
data_FR = load('C:\Users\ester\Desktop\Sea-stand\CM_LAB1\dati_nominal\120FR.mat');
y_real = data_FR.y;

% 3. Dati Sperimentali (LAB)
% Carica la struttura "Data_Lab1" nel workspace
load('C:\Users\ester\Desktop\Sea-stand\RECUPERO_LAB1\results_LAB1.mat');

% Creiamo il nome del campo dinamico (es. 'deg40', 'deg70'...)
campo_lab = sprintf('deg%d', ref);

% Estraiamo il vettore 'pos' navigando la struttura
y_lab = Data_Lab1.SS_nominal.(campo_lab).pos;


% --- CALCOLO VALORE FINALE ---
y_final_ideal = y_ideal(end);
y_final_real  = y_real(end);
% Facciamo la media degli ultimi 50 campioni per filtrare il rumore del sensore reale

% 1. Estrai il vettore numerico dalla struttura
% Di default, Simulink salva i valori nel campo .signals.values
y_lab_array = y_lab.signals.values; 

% 2. Ora che hai un vettore di numeri, puoi farne la media
y_final_lab = mean(y_lab_array(end-50:end));

% --- CALCOLO ERRORE A REGIME (e = r - y) ---
e_ss_ideal = ref - y_final_ideal;
e_ss_real  = ref - y_final_real;
e_ss_lab   = ref - y_final_lab;

% --- STAMPA RISULTATI ---
fprintf('\n--- RISULTATI ERRORE A REGIME PER GRADINO DA %d° ---\n', ref);
fprintf('Err Ideal (No Fric.)  : %8.4f deg\n', e_ss_ideal);
fprintf('Err Real (With Fric.) : %8.4f deg\n', e_ss_real);
fprintf('Err Experimental (LAB): %8.4f deg\n\n', e_ss_lab);