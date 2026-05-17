"""
Monitor 3D en temps real per al sistema de control de volum segur - UR3e
------------------------------------------------------------------------
flag == 0  →  con dinàmic centrat en el TCP (triant incisió)
flag == 1  →  con fix al punt de la incisió; l'eina es mou dins
"""

import math

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config


# =============================================================================
# CONFIGURACIÓ
# =============================================================================
ROBOT_IP           = "192.168.57.101"
ROBOT_PORT         = 30004
CONFIG_FILE        = "/Users/carlessolelanzas/Desktop/UNI/Segon Semestre/TFG/rtde_config.xml"
RTDE_FREQUENCY     = 100
ANIMATION_INTERVAL = 50   # ms (~20 fps)

# Paràmetres del con (han de coincidir amb els del programa PolyScope)
PROFUNDITAT   = 0.12   # m
R_INCISIO     = 0.03   # m  (radi a la incisió / apex)
R_PROFUNDITAT = 0.10   # m  (radi al fons del con)
N_CARES_CON   = 32     # resolució del con

# Límits de la vista 3D - ajustar segons l'espai de treball real
X_LIM = (-0.30,  0.10)
Y_LIM = (-0.80, -0.30)
Z_LIM = ( 0.05,  0.50)


# =============================================================================
# ESTAT GLOBAL
# =============================================================================
con = None

# Dades de la incisió (s'omplen quan flag canvia a 1)
incisio_confirmada = False
p_incisio = None   # [x0, y0, z0]
N_unitari = None   # [nx, ny, nz]

# Objectes de matplotlib que cal actualitzar cada frame
fig = None
ax  = None

plot_tcp      = None   # punt TCP (punta eina)
plot_flange   = None   # punt Flange (base eina)
plot_eina     = None   # recta TCP-Flange
col_con       = None   # Poly3DCollection del con (creat una vegada, vèrtexs actualitzats)
status_text   = None

# Centre actual de la finestra de visualització (inicialitzat al primer frame)
view_cx = None
view_cy = None
view_cz = None


# =============================================================================
# CONNEXIÓ RTDE
# =============================================================================
def connect_rtde():
    global con

    conf = rtde_config.ConfigFile(CONFIG_FILE)
    output_names, output_types = conf.get_recipe("out")

    con = rtde.RTDE(ROBOT_IP, ROBOT_PORT)
    con.connect()
    con.get_controller_version()

    if not con.send_output_setup(output_names, output_types, frequency=RTDE_FREQUENCY):
        raise RuntimeError("No s'ha pogut configurar la recipe de sortida RTDE.")
    if not con.send_start():
        raise RuntimeError("No s'ha pogut iniciar la sessió RTDE.")


def disconnect_rtde():
    global con
    if con is None:
        return
    try:
        con.send_pause()
    except Exception:
        pass
    try:
        con.disconnect()
    except Exception:
        pass


# =============================================================================
# LECTURA D'ESTAT
# =============================================================================
def read_state():
    state = con.receive()
    if state is None:
        return None

    tcp   = list(state.actual_TCP_pose)
    speed = list(state.actual_TCP_speed)
    flag  = int(state.output_int_register_0)

    # Posició del flange enviada des de PolyScope via registres dobles
    flange = [
        state.output_double_register_0,
        state.output_double_register_1,
        state.output_double_register_2,
    ]

    running = bool(int(state.robot_status_bits) & (1 << 1))
    return tcp, flange, speed, flag, running


# =============================================================================
# ROTACIÓ
# =============================================================================
def _rotvec_to_matrix(rv):
    """Converteix vector de rotació UR (rx,ry,rz) a matriu de rotació 3×3."""
    angle = math.sqrt(rv[0]**2 + rv[1]**2 + rv[2]**2)
    if angle < 1e-10:
        return np.eye(3)
    k = np.array(rv) / angle
    c, s = math.cos(angle), math.sin(angle)
    K = np.array([[ 0,    -k[2],  k[1]],
                  [ k[2],  0,    -k[0]],
                  [-k[1],  k[0],  0   ]])
    return c * np.eye(3) + (1 - c) * np.outer(k, k) + s * K


# =============================================================================
# GEOMETRIA DEL CON TRUNCAT
# =============================================================================
def _base_ortogonal(n):
    n = np.array(n, dtype=float)
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(n, ref)) > 0.9:
        ref = np.array([0.0, 1.0, 0.0])
    u = np.cross(n, ref)
    u /= np.linalg.norm(u)
    v = np.cross(n, u)
    return u, v


def con_truncat_vertices(apex, n, r_apex, r_base, altura, n_cares=N_CARES_CON):
    apex = np.array(apex)
    n    = np.array(n)
    u, v = _base_ortogonal(n)

    angles = np.linspace(0, 2 * math.pi, n_cares, endpoint=False)

    cercle_apex = [apex + r_apex * (math.cos(a) * u + math.sin(a) * v)
                   for a in angles]
    cercle_base = [apex + n * altura + r_base * (math.cos(a) * u + math.sin(a) * v)
                   for a in angles]

    cares = []
    for i in range(n_cares):
        j = (i + 1) % n_cares
        cares.append([cercle_apex[i], cercle_apex[j],
                      cercle_base[j], cercle_base[i]])
    return cares


def dibuixar_con(apex, n, r_apex, r_base, altura, alpha=0.18, color="royalblue"):
    cares = con_truncat_vertices(apex, n, r_apex, r_base, altura)
    col_con.set_verts(cares)
    col_con.set_alpha(alpha)
    col_con.set_facecolor(color)


# =============================================================================
# CONFIGURACIÓ DE LA FIGURA
# =============================================================================
def setup_figure():
    global fig, ax, col_con
    global plot_tcp, plot_flange, plot_eina, status_text

    fig = plt.figure(figsize=(11, 8))
    ax  = fig.add_subplot(111, projection="3d")

    # Con truncat: creat una vegada, vèrtexs actualitzats cada frame via set_verts()
    col_con = Poly3DCollection([], alpha=0.12, facecolor="royalblue", edgecolor="none")
    ax.add_collection3d(col_con)

    # Recta TCP-Flange (eix de l'eina)
    plot_eina,  = ax.plot([], [], [], color="steelblue", linewidth=2.5,
                          label="Eix eina")

    # Punt TCP (punta)
    plot_tcp,   = ax.plot([], [], [], marker="o", linestyle="None",
                          markersize=7, color="crimson", label="TCP (punta)")

    # Punt Flange (base)
    plot_flange, = ax.plot([], [], [], marker="s", linestyle="None",
                           markersize=6, color="darkorange", label="Flange")

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title("Monitor 3D – Sistema de volum segur UR3e")
    ax.legend(loc="upper left", fontsize=8)

    status_text = ax.text2D(0.02, 0.88, "", transform=ax.transAxes, fontsize=8,
                            verticalalignment="top",
                            bbox=dict(boxstyle="round,pad=0.3",
                                      facecolor="white", alpha=0.6))


# =============================================================================
# BUCLE D'ANIMACIÓ
# =============================================================================
VIEW_SPAN  = 0.80   # amplada total de la finestra per eix (m)
EDGE_GUARD = 0.12   # distància al límit que dispara el recentrat (m)


def _update_view(pts):
    """Recentra la finestra en el TCP si algun punt s'acosta al límit."""
    global view_cx, view_cy, view_cz

    half = VIEW_SPAN / 2
    px, py, pz = pts[0]   # TCP — recentrem sobre ell

    # Inicialització al primer frame
    if view_cx is None:
        view_cx, view_cy, view_cz = px, py, pz
        ax.set_xlim(view_cx - half, view_cx + half)
        ax.set_ylim(view_cy - half, view_cy + half)
        ax.set_zlim(view_cz - half, view_cz + half)
        return

    # Comprova si algun punt s'ha acostat massa al límit
    needs_recenter = False
    for qx, qy, qz in pts:
        if (qx < view_cx - half + EDGE_GUARD or qx > view_cx + half - EDGE_GUARD or
                qy < view_cy - half + EDGE_GUARD or qy > view_cy + half - EDGE_GUARD or
                qz < view_cz - half + EDGE_GUARD or qz > view_cz + half - EDGE_GUARD):
            needs_recenter = True
            break

    if needs_recenter:
        view_cx, view_cy, view_cz = px, py, pz
        ax.set_xlim(view_cx - half, view_cx + half)
        ax.set_ylim(view_cy - half, view_cy + half)
        ax.set_zlim(view_cz - half, view_cz + half)


def update(_frame):
    global incisio_confirmada, p_incisio, N_unitari

    data = read_state()
    if data is None:
        status_text.set_text("⚠ Connexió RTDE perduda")
        return (plot_tcp, plot_flange, plot_eina, status_text)

    tcp, flange, speed, flag, running = data

    x,  y,  z  = tcp[0],    tcp[1],    tcp[2]
    xf, yf, zf = flange[0], flange[1], flange[2]
    vx, vy, vz = speed[0],  speed[1],  speed[2]

    # Direcció de l'eina: eix Z del frame de l'eina en el frame base del robot
    R        = _rotvec_to_matrix(tcp[3:6])
    n_actual = list(R @ np.array([0.0, 0.0, 1.0]))

    # --- flag == 0: con dinàmic centrat en el TCP ---
    if flag == 0:
        dibuixar_con([x, y, z], n_actual,
                     R_INCISIO, R_PROFUNDITAT, PROFUNDITAT,
                     alpha=0.12, color="royalblue")

    # --- flag == 1: congelar el con al punt de la incisió ---
    elif flag == 1 and not incisio_confirmada:
        incisio_confirmada = True
        p_incisio = [x, y, z]
        N_unitari = n_actual[:]

        dibuixar_con(p_incisio, N_unitari,
                     R_INCISIO, R_PROFUNDITAT, PROFUNDITAT,
                     alpha=0.22, color="mediumseagreen")

    # --- Posicions TCP i Flange ---
    plot_tcp.set_data([x], [y])
    plot_tcp.set_3d_properties([z])

    plot_flange.set_data([xf], [yf])
    plot_flange.set_3d_properties([zf])

    # --- Recta TCP-Flange (el pal) ---
    plot_eina.set_data([x, xf], [y, yf])
    plot_eina.set_3d_properties([z, zf])

    # --- Ajust de la finestra (només si algun punt s'acosta al límit) ---
    watch_pts = [(x, y, z), (xf, yf, zf)]
    if incisio_confirmada:
        watch_pts.append(tuple(p_incisio))
    _update_view(watch_pts)

    # --- Text d'estat ---
    estat = "RUNNING" if running else "STOPPED"
    fase  = "SELECCIONANT INCISIÓ" if not incisio_confirmada else "INTERVENCIÓ ACTIVA"
    status_text.set_text(
        f"Programa: {estat}   |   Fase: {fase}\n"
        f"TCP   →  X={x:.4f}  Y={y:.4f}  Z={z:.4f} m\n"
        f"Flange→  X={xf:.4f}  Y={yf:.4f}  Z={zf:.4f} m\n"
        f"Vel TCP: {math.sqrt(vx**2+vy**2+vz**2)*1000:.1f} mm/s"
    )

    return (plot_tcp, plot_flange, plot_eina, status_text)


# =============================================================================
# MAIN
# =============================================================================
def main():
    try:
        connect_rtde()
        setup_figure()
        anim = FuncAnimation(fig, update, interval=ANIMATION_INTERVAL, blit=False)
        plt.tight_layout()
        plt.show()
        _ = anim
    finally:
        disconnect_rtde()


if __name__ == "__main__":
    main()
