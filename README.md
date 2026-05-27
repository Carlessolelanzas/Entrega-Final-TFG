# Entrega-Final-TFG

**Disseny i implementació d'un sistema de control amb volums de seguretat per les cirurgies robòtiques mínimament invasives**

Treball Final de Grau — Grau en Enginyeria Biomèdica  
Universitat de Girona · Juny 2026  
Autor: Carles Solé Lanzas  
Tutor: Dr. Xavier Cufí Solé

---

## Descripció del projecte

Aquest repositori conté el codi font complet del TFG. El projecte dissenya i implementa una barrera virtual que defineix un **volum segur d'intervenció** al voltant d'un punt d'incisió, fent servir el robot col·laboratiu **UR3e** d'Universal Robots com a plataforma de proves per simular el comportament d'una eina quirúrgica en una cirurgia mínimament invasiva.

El sistema consta de dues parts coordinades:

- **Controlador del robot (PolyScope / URScript):** s'executa directament al robot i gestiona les dues fases del sistema — tria del punt d'incisió i intervenció activa — vigilant contínuament que l'eina es mantingui dins del con truncat de seguretat i aplicant correccions lineals quan es detecta una violació.
- **Monitor 3D extern (Python):** script de visualització que es connecta al robot via RTDE i mostra en temps real la posició de l'eina i el volum segur en un entorn tridimensional interactiu.

Una demostració del funcionament complet del sistema es pot veure al vídeo disponible a YouTube:  
🎥 [https://youtu.be/YvySl9zEjNU]

---

## Estructura del repositori

```
Entrega-Final-TFG/
│
├── Codi Robot Ur3e/          # Programa del robot UR3e
│   ├── Tfg_lab.urp           # Fitxer de programa PolyScope (format natiu)
│   ├── Tfg_lab.script        # Exportació en URScript del programa principal
│   ├── Tfg_lab.txt           # Versió textual llegible del programa (arbre de nodes)
│   ├── Tfg_lab.installation  # Fitxer d'instal·lació: TCP, payload i paràmetres de seguretat
│   └── Tfg_lab.variables     # Variables definides al programa
│
├── programa_final_v1.py      # Script Python del monitor 3D en temps real (via RTDE)
├── rtde_config.xml           # Recipe RTDE: variables subscrites pel client Python
└── README.md                 # Aquest fitxer
```

### Descripció dels fitxers principals

**`Codi Robot Ur3e/Tfg_lab.urp`**  
Fitxer de programa natiu de PolyScope. Conté l'arbre de nodes complet del programa: fase de tria d'incisió amb detecció d'estabilitat, subprogrames `guardar_incisio`, `comprovem` i `corregim_eina`, i el thread paral·lel `Enviar_dades_tool_flange`. Cal obrir-lo des del controlador del robot UR3e o des del simulador URSim.

**`Codi Robot Ur3e/Tfg_lab.script`**  
Exportació en format URScript del programa. Útil per inspeccionar la lògica en format textual equivalent al que executa el controlador.

**`Codi Robot Ur3e/Tfg_lab.installation`**  
Fitxer d'instal·lació de PolyScope amb la configuració del TCP (Tool Center Point) i els paràmetres de seguretat associats al programa. S'ha de carregar conjuntament amb el `.urp`.

**`rtde_config.xml`**  
Defineix la *recipe* de sortida de la interfície RTDE. Subscriu les variables: `timestamp`, `actual_TCP_pose`, `actual_TCP_speed`, `robot_status_bits`, els tres registres dobles de sortida (`output_double_register_0/1/2`) que transporten les coordenades `[x, y, z]` del flange, i el registre enter `output_int_register_0` que indica la fase activa del sistema (0 = tria d'incisió, 1 = intervenció activa).

**`programa_final_v1.py`**  
Script Python que implementa el monitor 3D. Es connecta al robot per Ethernet (IP `192.168.57.101`, port `30004`) i llegeix les dades via RTDE a 100 Hz. Mostra una escena 3D interactiva amb el con truncat de seguretat, la posició de la punta i la base de l'eina, i un panell d'estat. Quan la incisió queda confirmada (flag = 1), s'activen dues vistes addicionals: perfil axial i vista cenital.

---

## Requisits

### Robot
- Robot UR3e amb PolyScope e-Series
- Connexió Ethernet directa o en xarxa local (IP per defecte del programa: `192.168.57.101`)

### Python
- Python 3.8 o superior
- Llibreries necessàries:

```bash
pip install numpy matplotlib ur-rtde
```

> La llibreria `ur-rtde` proporciona els mòduls `rtde.rtde` i `rtde.rtde_config` utilitzats pel script.

---

## Execució

1. Carregar el fitxer `Tfg_lab.installation` i el programa `Tfg_lab.urp` al controlador del robot (o al simulador URSim).
2. Executar el programa des de PolyScope.
3. Ajustar la variable `ROBOT_IP` i el paràmetre `CONFIG_FILE` del script Python si cal, i executar:
4. El monitor 3D s'obrirà i mostrarà el con dinàmic mentre el robot es troba en fase de tria d'incisió. Un cop confirmada la incisió (robot quiet 5 segons), el con es fixa i s'activa la visualització completa.
