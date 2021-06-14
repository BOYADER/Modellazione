# Modellazione Waypoint: Guida al codice
Implementazione delle equazioni della dinamica e della cinematica di un AUV. 
La missione consiste nel raggiungere dei waypoints di cui le coordinate (Lat. Long. e Prof.) sono espresse nel file `mission.yaml`.

## Contenuti:
* [1. Requisiti](#1-requisiti)
* [2. Simulazione](#2-simulazione)
* [3. Struttura del pkg](#3-struttura-del-pkg-di-modellazione)
* [4. Nodi](#4-nodi)
* [5. File YAML](#5-file-yaml)
* [6. Files launch](#6-files-launch)

## 1) Requisiti
Per eseguire correttamente la simulazione ed ottenere i risultati mostrati nel report, è opportuno assicurarsi di avere:

- eigen3 nel path `/usr/include/eigen3`
- Avere scaricato tutti e 3 i pkgs del Team Waypoints. Altrimenti eseguire:
     ```
        ~/catkin_ws/src:
        git clone https://github.com/BOYADER/Navigazione
        git clone https://github.com/BOYADER/Pianificazione-Controllo
        git clone https://github.com/BOYADER/Modellazione 
     ```
- Rendere eseguibili i nodi python del pkg pc_wp:
     ```
        ~/catkin_ws/src/Pianificazione-Controllo/pc_wp/scripts ~$ chmod +x *
     ```  
- Scaricare pymap3d e termcolor:
    ```
	pip install pymap3d
	pip install termcolor 
    ```

## 2) Simulazione
Per lanciare la simulazione, basta lanciare da terminale:
 ```
      ~$ roslaunch pc_wp launch.launch
 ```
Il launch appartiene al pkg pc_wp e lancia i nodi principali dei pkgs del Team. Per visualizzare meglio la simulazione, vengono inoltre lanciati dei nodi rqt_plot. 
(https://github.com/BOYADER/Pianificazione-Controllo/blob/main/pc_wp/launch/launch.launch)
E' possibile plottare:

- Stato vero e stato stimato.
- Forze e Coppie generate dal blocco di Controllo.
- Errore sulle singole grandezze.
- Errore quadratico Medio (MSE) calcolato iterativamente al variare del tempo.

![alt text](/docs/rqt_graph.PNG)

## 3) Struttura del pkg di Modellazione

![alt text](/docs/mod_pkg_screen.PNG)

## 4) Nodi

1. **model.cpp**: Nodo che è responsabile di prelevare informazioni dal topic/tau, fornito dal blocco di Pianificazione&Controllo, e restituire la dinamica del veicolo, fornendo le informazioni su posa e velocità che servono per implementare i sensori.

2. **ahrs.cpp**: Nodo che implementa il sensore AHRS.

3. **depth.cpp**: Nodo che implementa il sensore di profondità. 

4. **dvl.cpp**: Nodo che implementa il sensore DVL.

5. **gps.cpp**: Nodo che implementa il sensore GPS.

6. **usbl.cpp**: Nodo che implementa il sensore USBL.

7. **constant.h**: Nodo che contiene le costanti usate da più file.

8. **math_utility.cpp**: Nodo che contiene alcune funzioni matematiche di utilità usate da più file.

9. **sensor_utility.cpp**: Nodo che contiene alcune funzioni di utilità relative ai sensori.

10. **fake_nav.cpp**: Nodo di prova che simula il blocco navigazione, utile per il comando `rqt_graph`.

11. **fake_control.cpp**: Nodo di prova utile per simulare delle forze o coppie da imprimere al robot.

## 5) File YAML
L’unico file YAML nel pkg si chiama `mission.yaml` e si trova al path:
`~/config/mission.yaml` .

Il file contiene le coordinate geografiche della posa iniziale e dei waypoints da raggiungere
in missione.


## 6) Files launch
Sono state fatte numerose prove per la validazione del modello e dei sensori, dunque
sono presenti vari file launch.
Per lanciare uno di questi launch, sul terminale basta digitare:

```
   roslaunch modellazione <nome_file>.launch
```  

1. `modellazione.launch`: Esegue una simulazione del comportamento del robot come definito dal file `fake_control.cpp`
     
2. `surge_pitch.launch` e `surge_yaw.launch`: Esegue la simulazione del robot per analizzare il comportamento in presenza di attuazione di due gradi di libertà contemporaneamente.

3. `validazione_dvl.launch` e `validazione_gps.launch`: Eseguono la simulazione del robot per analizzare il comportamento dei due rispettivi sensori.
 
4. `validazione_risalita.launch`: Esegue la simulazione del comportamento del robot in assenza di forze e coppie.

