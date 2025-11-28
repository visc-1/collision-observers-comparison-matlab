# Stima delle Forze di Collisione: Analisi Comparativa (MATLAB/Simulink)

Questo repository contiene il codice sorgente e le simulazioni MATLAB/Simulink sviluppati per la Tesi di Laurea Triennale in Ingegneria Informatica e Automatica: **"Stima delle Forze di Collisione: Un'Analisi Comparativa di Metodi Basati su Energia, Velocità e Momento"**.

**Autore:** Francesco Viscione  
**Relatore:** Prof. Giuseppe Oriolo  
**Correlatore:** Dott. Nicola Scianca  
**Università:** Sapienza Università di Roma - Dipartimento di Ingegneria Informatica, Automatica e Gestionale (DIAG)

## 📄 Tesi e Video
*   **[Scarica la Tesi Completa (PDF)](./tesi_viscione_2084759.pdf)**
*   **[Guarda il Video della Simulazione](./video_simulazione.mp4)** _(Scenario di collisione dinamica)_

## 🎯 Obiettivo
Il progetto implementa e confronta tre diversi osservatori basati sul modello dinamico per il rilevamento delle collisioni su un manipolatore seriale **KUKA LBR iiwa 7**:
1.  **Osservatore basato sull'Energia** (Residuo scalare)
2.  **Osservatore basato sulla Velocità** (Residuo vettoriale accoppiato)
3.  **Osservatore basato sul Momento Generalizzato** (Residuo vettoriale disaccoppiato)

## 🛠️ Requisiti Software
*   MATLAB (Testato su versione [Inserire Versione, es. R2025a])
*   Simulink
*   Robotics System Toolbox

## 🚀 Utilizzo
1.  Clona il repository.
2.  Apri MATLAB e imposta la cartella del progetto come *Current Folder*.
3.  Seleziona la simulazione dai settaggi rapidi (`trajectory`)
4.  Esegui lo script principale (`kukaIiwa.m`).
5.  Apri i file Simulink relativi gli osservatori basati su velocità e momento (`velocity_observer.slx` e `momentum_observer.slx`).
6.  Avvia la simulazione.

## 📊 Scenari
Le simulazioni includono:
*   **Scenario 1:** Collisione con ostacolo elastico durante il movimento in traiettoria circolare. (`trajectory = 'taskSpace'`)
*   **Scenario 2:** Impatto impulsivo a robot fermo (per evidenziare l'accoppiamento/disaccoppiamento dei residui). (`trajectory = 'stopped'`)
*   **Scenario 3 (Non discusso nella tesi):** Collisione con ostacolo elastico durante il movimento in traiettoria definita nello spazio dei giunti. (`trajectory = 'jointSpace'`)

---
&copy; 2025 Francesco Viscione.
