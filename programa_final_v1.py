"""
Visualitzador 3D en temps real del sistema de control de volum segur - UR3e
---------------------------------------------------------------------------
Part gràfica del Treball de Fi de Grau d'Enginyeria Biomèdica.

El programa es connecta al robot via RTDE i mostra en temps real la posició
de l'eina quirúrgica respecte al con de seguretat definit a PolyScope.

Fases de funcionament:
  flag = 0  -->  El cirurgià posiciona l'eina per triar el punt d'incisió.
                 Es mostra un con dinàmic centrat al TCP.
  flag = 1  -->  La incisió queda confirmada. El con es fixa en aquella posició
                 i apareixen dues vistes addicionals: perfil axial i vista cenital.
"""

import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Polygon as MplPolygon, Circle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# =============================================================================
# PARÀMETRES DE CONFIGURACIÓ
# =============================================================================

# Connexió al robot
ROBOT_IP           = "192.168.57.101"
ROBOT_PORT         = 30004
CONFIG_FILE        = "/Users/carlessolelanzas/Desktop/UNI/Segon Semestre/TFG/Entrega-Final-TFG/rtde_config.xml"
RTDE_FREQUENCY     = 100   # Hz — freqüència de recepció de dades del robot
ANIMATION_INTERVAL = 50    # ms — interval entre frames de l'animació (~20 fps)

# Geometria del con de seguretat (ha de coincidir amb els paràmetres del programa PolyScope)
PROFUNDITAT   = 0.12   # m — profunditat màxima permesa
R_INCISIO     = 0.03   # m — radi del con a la superfície d'incisió (apex)
R_PROFUNDITAT = 0.10   # m — radi del con al fons (base)
N_CARES_CON   = 32     # nombre de cares del con truncat (com més alt, més suau)

# Rang inicial de la vista 3D: centrat a l'espai de treball real del UR3e
# (~0.8 m per eix permet veure el moviment sense que tot es vegi massa petit)
X_LIM = (-0.50,  0.30)
Y_LIM = (-0.90,  0.00)
Z_LIM = ( 0.00,  0.70)

# Quan es confirma la incisió, la vista 3D fa zoom al voltant del punt d'incisió
ZOOM_SPAN = 0.35   # m — amplada total de la finestra de zoom (per eix)


# =============================================================================
# VARIABLES GLOBALS
# =============================================================================

# Objecte de connexió RTDE
con = None

# Dades que es guarden en el moment de confirmar la incisió (flag 0 -> 1)
incisio_confirmada = False
p_incisio = None   # coordenades [x, y, z] del punt d'incisió
N_unitari = None   # vector unitari de l'eix del con (orientació de l'eina en la incisió)

# Objectes de matplotlib — vista 3D principal
fig        = None
ax         = None
ax_profile = None   # subplot del perfil axial (dreta superior)
ax_topdown = None   # subplot de la vista cenital (dreta inferior)

plot_tcp    = None   # punt TCP (punta de l'eina)
plot_flange = None   # punt Flange (base de l'eina)
plot_eina   = None   # segment TCP–Flange (eix visual de l'eina)
col_con     = None   # Poly3DCollection del con de seguretat
status_text = None   # text informatiu sobreimprès a la vista 3D

# Variables de la vista de perfil (secció axial del con, 2D)
plot_p_tool   = None
plot_p_tcp    = None
plot_p_flange = None
profile_u     = None   # primer vector ortogonal a N_unitari (pla de projecció)
profile_v     = None   # segon vector ortogonal a N_unitari (per la vista cenital)
profile_ready = False

# Variables de la vista cenital (projecció sobre el pla perpendicular a l'eix, 2D)
plot_td_tool   = None
plot_td_tcp    = None
plot_td_flange = None
topdown_ready  = False


# =============================================================================
# CONNEXIÓ RTDE
# =============================================================================

def connect_rtde():
    """Estableix la connexió RTDE amb el robot i configura la recipe de sortida."""
    global con

    conf = rtde_config.ConfigFile(CONFIG_FILE)
    output_names, output_types = conf.get_recipe("out")

    con = rtde.RTDE(ROBOT_IP, ROBOT_PORT)
    con.connect()
    con.get_controller_version()

    if not con.send_output_setup(output_names, output_types, frequency=RTDE_FREQUENCY):
        raise RuntimeError("No s'ha pogut configurar la recipe de sortida RTDE.")
    if not con.send_start():
        raise RuntimeError("No s'ha pogut iniciar la transmissió RTDE.")


# =============================================================================
# LECTURA D'ESTAT DEL ROBOT
# =============================================================================

def read_state():
    """
    Llegeix un paquet RTDE i retorna l'estat actual del robot.

    Retorna (tcp, flange, speed, flag, running) o None si la connexió falla:
      tcp    -- [x, y, z, rx, ry, rz] posició i orientació del TCP en el frame base
      flange -- [x, y, z] posició del Flange (enviada via registres dobles des de PolyScope)
      speed  -- [vx, vy, vz, ...] velocitat cartesiana del TCP
      flag   -- int: 0 = buscant incisió, 1 = incisió activa
      running-- bool: True si el programa PolyScope està en execució
    """
    state = con.receive()
    tcp    = list(state.actual_TCP_pose)
    speed  = list(state.actual_TCP_speed)
    flag   = int(state.output_int_register_0)
    flange = [
        state.output_double_register_0,
        state.output_double_register_1,
        state.output_double_register_2,
    ]
    running = bool(int(state.robot_status_bits) & (1 << 1))

    return tcp, flange, speed, flag, running


# =============================================================================
# CÀLCULS GEOMÈTRICS
# =============================================================================

def _rotvec_to_matrix(rv):
    """
    Converteix el vector de rotació UR (rx, ry, rz) a una matriu de rotació 3x3.

    El format UR representa l'eix de rotació escalat per l'angle (fórmula de Rodrigues).
    S'utilitza per obtenir la direcció de l'eix Z de l'eina en el frame base del robot.
    """
    angle = math.sqrt(rv[0]**2 + rv[1]**2 + rv[2]**2)
    if angle < 1e-10:
        return np.eye(3)
    k    = np.array(rv) / angle
    c, s = math.cos(angle), math.sin(angle)
    K    = np.array([[ 0,    -k[2],  k[1]],
                     [ k[2],  0,    -k[0]],
                     [-k[1],  k[0],  0   ]])
    return c * np.eye(3) + (1 - c) * np.outer(k, k) + s * K


def _base_ortogonal(n):
    """
    Calcula dos vectors ortonormals (u, v) perpendiculars al vector n.

    Defineixen el sistema de referència local del con: n és l'eix,
    i u, v formen el pla de la secció transversal.
    S'escull el vector de referència per evitar la singularitat quan n és paral·lel a X.
    """
    n   = np.array(n, dtype=float)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(n, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v  = np.cross(n, u)
    return u, v


# =============================================================================
# GEOMETRIA DEL CON TRUNCAT (VISTA 3D)
# =============================================================================

def _con_truncat_cares(apex, n, r_apex, r_base, altura):
    """
    Genera les cares laterals del con truncat per a la visualització 3D.

    Cada cara és un quadrilàter format per dos vèrtexs del cercle de l'apex
    (incisió) i dos del cercle de la base (fons del con).
    """
    apex = np.array(apex)
    n    = np.array(n)
    u, v = _base_ortogonal(n)

    angles      = np.linspace(0, 2 * math.pi, N_CARES_CON, endpoint=False)
    cercle_apex = [apex + r_apex * (math.cos(a) * u + math.sin(a) * v) for a in angles]
    cercle_base = [apex + n * altura + r_base * (math.cos(a) * u + math.sin(a) * v)
                   for a in angles]

    cares = []
    for i in range(N_CARES_CON):
        j = (i + 1) % N_CARES_CON
        cares.append([cercle_apex[i], cercle_apex[j], cercle_base[j], cercle_base[i]])
    return cares


def _dibuixar_con(apex, n, r_apex, r_base, altura, alpha=0.18, color="royalblue"):
    """Actualitza la geometria i l'estil visual del con truncat a la vista 3D."""
    col_con.set_verts(_con_truncat_cares(apex, n, r_apex, r_base, altura))
    col_con.set_alpha(alpha)
    col_con.set_facecolor(color)


# =============================================================================
# INICIALITZACIÓ DE LA FIGURA
# =============================================================================

def setup_figure():
    """
    Crea la finestra matplotlib amb els tres eixos de visualització:
      ax        -- vista 3D principal (ocupa tota la finestra fins que arriba el flag)
      ax_profile -- perfil axial del con (apareix a la dreta superior quan flag = 1)
      ax_topdown -- vista cenital del con (apareix a la dreta inferior quan flag = 1)

    Les dues vistes 2D es creen amb la posició final però es mantenen invisibles
    fins que es confirmi la incisió, per no interferir amb la vista 3D inicial.
    """
    global fig, ax, ax_profile, ax_topdown, col_con
    global plot_tcp, plot_flange, plot_eina, status_text

    fig = plt.figure(figsize=(16, 7))

    # Vista 3D inicial: s'estén per tota la finestra per maximitzar la visibilitat
    ax = fig.add_axes([0.0, 0.0, 1.0, 0.95], projection="3d")

    # Vistes 2D: posicions finals ja definides, però invisibles fins al flag
    # El marge esquerre de 0.54 evita que el desbordament del 3D les tapi
    ax_profile = fig.add_axes([0.54, 0.57, 0.43, 0.36])
    ax_profile.set_visible(False)
    ax_topdown = fig.add_axes([0.54, 0.07, 0.43, 0.40])
    ax_topdown.set_visible(False)

    # Con de seguretat: es crea una sola vegada i es reutilitza cada frame
    # (molt més eficient que esborrar i tornar a crear)
    col_con = Poly3DCollection([], alpha=0.12, facecolor="royalblue", edgecolor="none")
    ax.add_collection3d(col_con)

    # Elements visuals de l'eina quirúrgica
    plot_eina,   = ax.plot([], [], [], color="steelblue",   linewidth=2.5, label="Eix eina")
    plot_tcp,    = ax.plot([], [], [], marker="o", linestyle="None",
                           markersize=7, color="crimson",    label="TCP (punta)")
    plot_flange, = ax.plot([], [], [], marker="s", linestyle="None",
                           markersize=6, color="darkorange", label="Flange")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Monitor 3D – Sistema de volum segur UR3e")
    ax.legend(loc="upper left", fontsize=8)

    # Rang ampli per poder apreciar el moviment del robot en l'espai de treball
    ax.set_xlim(*X_LIM)
    ax.set_ylim(*Y_LIM)
    ax.set_zlim(*Z_LIM)
    ax.set_autoscale_on(False)   # impedeix que matplotlib reescali automàticament

    # Text informatiu sobreimprès (posició, velocitat, estat del programa)
    status_text = ax.text2D(0.02, 0.88, "", transform=ax.transAxes, fontsize=8,
                            verticalalignment="top",
                            bbox=dict(boxstyle="round,pad=0.3", facecolor="white", alpha=0.6))


# =============================================================================
# PROJECCIONS PER A LES VISTES 2D
# =============================================================================

def _project_profile(P):
    """
    Projecta el punt P al sistema de coordenades del con (profunditat, radial).

    - profunditat: distància al llarg de l'eix del con des de la incisió
    - radial:      component del desplaçament perpendicular en la direcció de profile_u
    """
    v      = np.array(P) - np.array(p_incisio)
    depth  = float(np.dot(v, np.array(N_unitari)))
    radial = float(np.dot(v - depth * np.array(N_unitari), profile_u))
    return depth, radial


def _project_topdown(P):
    """
    Projecta el punt P sobre el pla perpendicular a l'eix del con.

    Retorna (u, v), les components en el sistema ortogonal del con.
    Permet detectar desviacions laterals de l'eina respecte a l'eix.
    """
    v = np.array(P) - np.array(p_incisio)
    return float(np.dot(v, profile_u)), float(np.dot(v, profile_v))


# =============================================================================
# INICIALITZACIÓ I ACTUALITZACIÓ DE LES VISTES 2D
# =============================================================================

def _init_profile_view():
    """
    Configura la vista de perfil axial quan es confirma la incisió.

    Mostra el con com un trapezi en secció longitudinal: estret a la superfície
    (incisió) i ample al fons. L'eix Y és la profunditat, amb zero a dalt (pell)
    i valors positius cap avall (interior del cos). L'eix X és el desplaçament radial.
    """
    global ax_profile, plot_p_tool, plot_p_tcp, plot_p_flange
    global profile_u, profile_v, profile_ready

    # Calculem els dos vectors que defineixen el pla perpendicular a l'eix del con
    profile_u, profile_v = _base_ortogonal(N_unitari)

    # Reduïm la vista 3D a la meitat esquerra per deixar espai a les vistes 2D
    # El marge dret generós (0.01 + 0.43 = 0.44) absorbeix el desbordament típic del 3D
    ax.set_position([0.01, 0.04, 0.43, 0.88])

    ax_profile.cla()
    ax_profile.set_visible(True)

    # Dibuixem la secció axial del con: trapezi estret a dalt (incisió) i ample a baix (fons)
    trap_r = [-R_INCISIO, R_INCISIO,  R_PROFUNDITAT, -R_PROFUNDITAT]
    trap_d = [0,          0,           PROFUNDITAT,    PROFUNDITAT  ]
    ax_profile.add_patch(
        MplPolygon(list(zip(trap_r, trap_d)), closed=True, alpha=0.25,
                   facecolor="mediumseagreen", edgecolor="mediumseagreen",
                   linewidth=1.5, label="Volum segur")
    )

    ax_profile.set_xlabel("Radial [m]",      fontsize=8)
    ax_profile.set_ylabel("Profunditat [m]", fontsize=8)
    ax_profile.set_title("Perfil del con – vista axial", fontsize=9)
    ax_profile.tick_params(labelsize=7)
    ax_profile.set_xlim(-(R_PROFUNDITAT + 0.04), R_PROFUNDITAT + 0.04)
    # Eix Y invertit: y=0 (pell) a dalt, y=PROFUNDITAT (fons) a baix
    ax_profile.set_ylim(PROFUNDITAT + 0.04, -0.22)
    ax_profile.set_aspect("equal")
    ax_profile.grid(True, alpha=0.3)

    plot_p_tool,   = ax_profile.plot([], [], color="steelblue", linewidth=2.0)
    plot_p_tcp,    = ax_profile.plot([], [], marker="o", linestyle="None",
                                     markersize=6, color="crimson",    label="TCP")
    plot_p_flange, = ax_profile.plot([], [], marker="s", linestyle="None",
                                     markersize=5, color="darkorange", label="Flange")
    ax_profile.legend(loc="upper right", fontsize=7)
    profile_ready = True

    _init_topdown_view()


def _init_topdown_view():
    """
    Configura la vista cenital (des de dalt de l'eix del con) quan es confirma la incisió.

    Es mostren dos cercles concèntrics: el petit correspon al límit a la superfície
    d'incisió i el gran al límit al fons del con. L'eina es projecta sobre aquest pla,
    cosa que permet veure desviacions laterals de l'eix de penetració.
    """
    global ax_topdown, plot_td_tool, plot_td_tcp, plot_td_flange, topdown_ready

    ax_topdown.cla()
    ax_topdown.set_visible(True)

    # Cercle gran: límit del volum de seguretat al fons del con
    ax_topdown.add_patch(Circle((0, 0), R_PROFUNDITAT,
                                facecolor="mediumseagreen", alpha=0.20,
                                edgecolor="mediumseagreen", linewidth=1.5,
                                label=f"Límit fons ({R_PROFUNDITAT*100:.0f} cm)"))
    # Cercle petit: límit a la superfície d'incisió
    ax_topdown.add_patch(Circle((0, 0), R_INCISIO,
                                facecolor="mediumseagreen", alpha=0.35,
                                edgecolor="forestgreen", linewidth=1.2,
                                label=f"Límit incisió ({R_INCISIO*100:.0f} cm)"))
    # Creu al centre per marcar l'eix del con
    ax_topdown.plot(0, 0, marker="+", markersize=9, color="gray", markeredgewidth=1.5)

    lim = R_PROFUNDITAT + 0.04
    ax_topdown.set_xlim(-lim, lim)
    ax_topdown.set_ylim(-lim, lim)
    ax_topdown.set_aspect("equal")
    ax_topdown.set_xlabel("u [m]", fontsize=8)
    ax_topdown.set_ylabel("v [m]", fontsize=8)
    ax_topdown.set_title("Vista cenital – des de dalt del con", fontsize=9)
    ax_topdown.tick_params(labelsize=7)
    ax_topdown.grid(True, alpha=0.3)

    plot_td_tool,   = ax_topdown.plot([], [], color="steelblue", linewidth=2.0)
    plot_td_tcp,    = ax_topdown.plot([], [], marker="o", linestyle="None",
                                      markersize=6, color="crimson",    label="TCP")
    plot_td_flange, = ax_topdown.plot([], [], marker="s", linestyle="None",
                                      markersize=5, color="darkorange", label="Flange")
    ax_topdown.legend(loc="upper right", fontsize=7)
    topdown_ready = True


def _update_profile_view(tcp_xyz, flange_xyz):
    """Actualitza la posició de l'eina a la vista de perfil."""
    if not profile_ready:
        return
    d_tcp, r_tcp = _project_profile(tcp_xyz)
    d_fl,  r_fl  = _project_profile(flange_xyz)
    plot_p_tool.set_data([r_tcp, r_fl], [d_tcp, d_fl])
    plot_p_tcp.set_data([r_tcp], [d_tcp])
    plot_p_flange.set_data([r_fl], [d_fl])


def _update_topdown_view(tcp_xyz, flange_xyz):
    """Actualitza la posició de l'eina a la vista cenital."""
    if not topdown_ready:
        return
    u_tcp, v_tcp = _project_topdown(tcp_xyz)
    u_fl,  v_fl  = _project_topdown(flange_xyz)
    plot_td_tool.set_data([u_tcp, u_fl], [v_tcp, v_fl])
    plot_td_tcp.set_data([u_tcp], [v_tcp])
    plot_td_flange.set_data([u_fl], [v_fl])


# =============================================================================
# BUCLE D'ANIMACIÓ
# =============================================================================

def update(_frame):
    """
    Funció cridada per FuncAnimation a cada frame (~20 fps).

    Llegeix l'estat del robot, actualitza el con i les posicions de l'eina
    a totes les vistes actives. Gestiona el canvi de fase quan arriba el flag.
    """
    global incisio_confirmada, p_incisio, N_unitari

    data = read_state()
    if data is None:
        status_text.set_text("⚠ Connexió RTDE perduda")
        return plot_tcp, plot_flange, plot_eina, status_text

    tcp, flange, speed, flag, running = data
    x,  y,  z  = tcp[0],    tcp[1],    tcp[2]
    xf, yf, zf = flange[0], flange[1], flange[2]
    vx, vy, vz = speed[0],  speed[1],  speed[2]

    # Direcció de penetració de l'eina: columna Z de la matriu de rotació del TCP
    # (l'eix Z del frame de l'eina apunta cap on entra l'instrumental)
    n_actual = list(_rotvec_to_matrix(tcp[3:6]) @ np.array([0.0, 0.0, 1.0]))

    if flag == 0:
        # Fase de posicionament: el con es mou dinàmicament amb el TCP
        _dibuixar_con([x, y, z], n_actual,
                      R_INCISIO, R_PROFUNDITAT, PROFUNDITAT,
                      alpha=0.12, color="royalblue")

    elif flag == 1 and not incisio_confirmada:
        # Incisió confirmada: congelar el con i activar les vistes 2D
        incisio_confirmada = True
        p_incisio          = [x, y, z]
        N_unitari          = n_actual[:]

        _dibuixar_con(p_incisio, N_unitari,
                      R_INCISIO, R_PROFUNDITAT, PROFUNDITAT,
                      alpha=0.22, color="mediumseagreen")

        # Zoom de la vista 3D centrat al punt d'incisió
        h = ZOOM_SPAN / 2
        ax.set_xlim(x - h, x + h)
        ax.set_ylim(y - h, y + h)
        ax.set_zlim(z - h, z + h)

        # Activar les vistes de perfil i cenital
        _init_profile_view()

    # Actualitzar la posició del TCP i del Flange a la vista 3D
    plot_tcp.set_data([x], [y]);       plot_tcp.set_3d_properties([z])
    plot_flange.set_data([xf], [yf]);  plot_flange.set_3d_properties([zf])
    plot_eina.set_data([x, xf], [y, yf])
    plot_eina.set_3d_properties([z, zf])

    # Actualitzar les vistes 2D (només un cop confirmada la incisió)
    if incisio_confirmada:
        _update_profile_view(tcp[:3], flange)
        _update_topdown_view(tcp[:3], flange)

    # Text informatiu: estat del programa, coordenades i velocitat
    vel = math.sqrt(vx**2 + vy**2 + vz**2) * 1000   # convertim a mm/s
    status_text.set_text(
        f"Programa: {'RUNNING' if running else 'STOPPED'}   |   "
        f"Fase: {'SELECCIONANT INCISIÓ' if not incisio_confirmada else 'INTERVENCIÓ ACTIVA'}\n"
        f"TCP   →  X={x:.4f}  Y={y:.4f}  Z={z:.4f} m\n"
        f"Flange→  X={xf:.4f}  Y={yf:.4f}  Z={zf:.4f} m\n"
        f"Vel TCP: {vel:.1f} mm/s"
    )

    return plot_tcp, plot_flange, plot_eina, status_text


# =============================================================================
# PUNT D'ENTRADA
# =============================================================================

def main():
    """Connecta al robot, llança l'animació i desconnecta en acabar (o en cas d'error)."""
    try:
        connect_rtde()
        setup_figure()
        anim = FuncAnimation(fig, update, interval=ANIMATION_INTERVAL, blit=False)
        plt.show()
        _ = anim   # evitar que el garbage collector elimini l'objecte FuncAnimation prematurament


if __name__ == "__main__":
    main()
