"""
Membuat gambar-gambar ilustrasi untuk README (skema model, diagram blok,
pole map, step response, respons frekuensi, dan pengaruh bobot Q/R).

Hasil simulasi di sini adalah reproduksi Python dari analisis LQR di
docs/Report.pdf (parameter BMW 530i bagian depan, satuan SI).
Hasil optimasi GA tetap memakai plot MATLAB asli di docs/images/hasil_*.png.

Jalankan dari root repo:
    python docs/scripts/generate_figures.py
"""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch, Rectangle
from scipy import signal
from scipy.linalg import solve_continuous_are

OUT = Path(__file__).resolve().parents[1] / "images"
OUT.mkdir(parents=True, exist_ok=True)

# ---------------------------------------------------------------- style
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK2 = "#52514e"
GRID = "#e4e3df"
C_LQR = "#2a78d6"   # series 1
C_OL = "#eb6834"    # series 2
C_3 = "#1baf7a"     # series 3
BOX_FILL = "#eef4fc"
BOX_EDGE = "#2a78d6"

plt.rcParams.update({
    "figure.facecolor": SURFACE,
    "axes.facecolor": SURFACE,
    "savefig.facecolor": SURFACE,
    "axes.edgecolor": INK2,
    "axes.labelcolor": INK,
    "axes.titlecolor": INK,
    "axes.titleweight": "bold",
    "axes.titlesize": 12,
    "axes.labelsize": 10,
    "xtick.color": INK2,
    "ytick.color": INK2,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.8,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
    "lines.linewidth": 2,
    "font.size": 10,
    "mathtext.fontset": "cm",
})


def save(fig, name):
    fig.savefig(OUT / name, dpi=160, bbox_inches="tight")
    plt.close(fig)
    print("saved", OUT / name)


# ---------------------------------------------------------------- model
kk = 340e3    # N/m   kekakuan ban
kr = 30e3     # N/m   kekakuan pegas suspensi
br = 1450     # Ns/m  redaman suspensi
mc = 408      # kg    massa bodi (sprung)
mus = 48.3    # kg    massa roda (unsprung)

A = np.array([
    [0, 1, 0, 0],
    [(-kk - kr) / mus, -br / mus, kr / mus, br / mus],
    [0, 0, 0, 1],
    [kr / mc, br / mc, -kr / mc, -br / mc],
])
B = np.array([
    [0, 0],
    [-1 / mus, kk / mus],
    [0, 0],
    [1 / mc, 0],
])
C = np.array([[0, 0, 1, 0]])
D = np.zeros((1, 2))


def lqr(A, B, Q, R):
    P = solve_continuous_are(A, B, Q, R)
    return np.linalg.solve(R, B.T @ P)


def step_io(Am, j, t):
    """Step response y_c terhadap input ke-j (0 = F_a, 1 = u)."""
    sys = signal.StateSpace(Am, B[:, j:j + 1], C, D[:, j:j + 1])
    _, y = signal.step(sys, T=t)
    return y


Q_rep = np.diag([10, 1, 10, 1])
R_rep = np.diag([100, 100])
K = lqr(A, B, Q_rep, R_rep)
Acl = A - B @ K


def print_analysis():
    Mo = np.vstack([C, C @ A, C @ A @ A, C @ A @ A @ A])
    Mc = np.hstack([B[:, :1], A @ B[:, :1], A @ A @ B[:, :1],
                    A @ A @ A @ B[:, :1]])
    np.set_printoptions(precision=6, suppress=True)
    print("Observability matrix (C = [0 0 1 0]):\n", Mo)
    print("rank =", np.linalg.matrix_rank(Mo), " det =", np.linalg.det(Mo))
    print("Controllability matrix (input Fa):\n", Mc)
    print("rank =", np.linalg.matrix_rank(Mc))
    print("eig(A)   =", np.linalg.eigvals(A))
    print("K (LQR)  =\n", K)
    print("eig(A-BK)=", np.linalg.eigvals(Acl))


# ---------------------------------------------------------------- diagrams
def spring(ax, x, y0, y1, n=6, w=0.12, color=INK):
    ys = np.linspace(y0, y1, 2 * n + 3)
    xs = np.full_like(ys, x)
    xs[2:-2] = x + w * np.array([(-1) ** i for i in range(2 * n - 1)])
    ax.plot(xs, ys, color=color, lw=1.6, solid_joinstyle="miter")


def damper(ax, x, y0, y1, w=0.14, color=INK):
    ym = (y0 + y1) / 2
    ax.plot([x, x], [y0, ym - 0.12], color=color, lw=1.6)
    ax.plot([x - w, x - w, x + w, x + w], [ym + 0.15, ym - 0.12, ym - 0.12, ym + 0.15],
            color=color, lw=1.6)
    ax.plot([x - w * 0.7, x + w * 0.7], [ym, ym], color=color, lw=2.4)
    ax.plot([x, x], [ym, y1], color=color, lw=1.6)


def up_arrow(ax, x, y, label, color=INK):
    ax.annotate("", xy=(x, y + 0.45), xytext=(x, y),
                arrowprops=dict(arrowstyle="-|>", color=color, lw=1.4))
    ax.plot([x - 0.1, x], [y, y], color=color, lw=1.4)
    ax.text(x + 0.08, y + 0.3, label, color=color, fontsize=12, va="center")


def fig_quarter_car():
    fig, ax = plt.subplots(figsize=(6.4, 5.6))
    ax.set_xlim(-2.2, 3.2)
    ax.set_ylim(-0.6, 5.3)
    ax.axis("off")

    # body (sprung mass)
    ax.add_patch(FancyBboxPatch((-1.4, 3.6), 2.8, 1.0, boxstyle="round,pad=0.02",
                                fc="#f5e7cf", ec=INK, lw=1.6))
    ax.text(0, 4.1, r"$m_c$  (bodi)", ha="center", va="center", fontsize=14)
    # wheel (unsprung mass)
    ax.add_patch(FancyBboxPatch((-1.1, 1.5), 2.2, 0.9, boxstyle="round,pad=0.02",
                                fc="#d4ecc4", ec=INK, lw=1.6))
    ax.text(0, 1.95, r"$m_{us}$  (roda)", ha="center", va="center", fontsize=14)

    # suspension: spring, damper, actuator
    spring(ax, -0.8, 2.4, 3.6)
    ax.text(-1.25, 3.0, r"$k_r$", fontsize=13, va="center")
    damper(ax, 0.0, 2.4, 3.6)
    ax.text(0.22, 3.0, r"$b_r$", fontsize=13, va="center")
    ax.plot([0.8, 0.8], [2.4, 2.8], color=INK, lw=1.6)
    ax.plot([0.8, 0.8], [3.2, 3.6], color=INK, lw=1.6)
    ax.add_patch(plt.Circle((0.8, 3.0), 0.2, fc="#e34948", ec=INK, lw=1.4))
    ax.text(1.08, 3.0, r"$F_a(t)$", fontsize=13, va="center", color="#b3262a")

    # tire spring
    spring(ax, 0.0, 0.25, 1.5, n=4)
    ax.text(0.25, 0.85, r"$k_k$", fontsize=13, va="center")
    # road
    ax.add_patch(Rectangle((-1.6, 0.05), 3.2, 0.2, fc="#bdbcb6", ec=INK2, lw=1))
    for xi in np.linspace(-1.5, 1.5, 13):
        ax.plot([xi, xi - 0.15], [0.05, -0.12], color=INK2, lw=0.8)
    ax.text(-1.72, 0.15, "jalan", color=INK2, fontsize=10, va="center", ha="right")

    # coordinates
    up_arrow(ax, 1.75, 4.1, r"$y_c = x_3$")
    up_arrow(ax, 1.45, 1.95, r"$y_k = x_1$")
    up_arrow(ax, 0.6, 0.15, r"$u(t)$  (profil jalan)", color=INK2)

    ax.set_title("Model Quarter-Car Suspensi Aktif (2-DOF)", pad=4)
    save(fig, "quarter_car_model.png")


def box(ax, xy, w, h, text, fc=BOX_FILL, ec=BOX_EDGE, fs=11):
    x, y = xy
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h,
                                boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc=fc, ec=ec, lw=1.5))
    ax.text(x, y, text, ha="center", va="center", fontsize=fs, color=INK)


def sumnode(ax, xy, r=0.16):
    ax.add_patch(plt.Circle(xy, r, fc=SURFACE, ec=INK, lw=1.4))
    ax.text(*xy, r"$\Sigma$", ha="center", va="center", fontsize=10)


def arrow(ax, pts, label=None, lpos=0.5, loff=(0, 0.14), color=INK):
    pts = np.asarray(pts, dtype=float)
    for a, b in zip(pts[:-2], pts[1:-1]):
        ax.plot([a[0], b[0]], [a[1], b[1]], color=color, lw=1.4)
    ax.add_patch(FancyArrowPatch(pts[-2], pts[-1], arrowstyle="-|>",
                                 mutation_scale=12, color=color, lw=1.4))
    if label:
        seg = pts[int(lpos * (len(pts) - 1)) if len(pts) > 2 else 0]
        nxt = pts[min(int(lpos * (len(pts) - 1)) + 1, len(pts) - 1)]
        m = (seg + nxt) / 2 + np.array(loff)
        ax.text(*m, label, ha="center", va="bottom", fontsize=11, color=INK)


def diagram_axes(w, h, title):
    fig, ax = plt.subplots(figsize=(w, h))
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_title(title, pad=2)
    return fig, ax


def fig_block_lqr():
    fig, ax = diagram_axes(8.6, 3.2, "Struktur Kontrol LQR (Full-State Feedback)")
    ax.set_xlim(-0.6, 9.4)
    ax.set_ylim(-1.5, 1.6)
    box(ax, (4.2, 0.6), 2.6, 1.0, "Plant quarter-car\n" r"$\dot x = Ax + Bu$")
    box(ax, (4.2, -0.9), 1.6, 0.7, r"$-K$", fc="#fdf0ea", ec=C_OL)
    box(ax, (7.6, 0.6), 1.1, 0.7, r"$C$")
    arrow(ax, [(0.0, 0.85), (2.9, 0.85)], r"$F_a$ / gangguan", loff=(0, 0.06))
    arrow(ax, [(5.5, 0.6), (7.05, 0.6)], r"$x$", loff=(0, 0.06))
    arrow(ax, [(8.15, 0.6), (9.2, 0.6)], r"$y$", loff=(0, 0.06))
    arrow(ax, [(6.3, 0.6), (6.3, -0.9), (5.0, -0.9)])
    ax.plot(6.3, 0.6, "o", color=INK, ms=4)
    arrow(ax, [(3.4, -0.9), (1.8, -0.9), (1.8, 0.35), (2.9, 0.35)])
    ax.text(1.1, -0.35, r"$u = -Kx$", fontsize=11, color=INK)
    ax.text(4.2, -1.45, r"$K = R^{-1}B^{T}P$,   $A^{T}P + PA - PBR^{-1}B^{T}P + Q = 0$",
            ha="center", fontsize=10, color=INK2)
    save(fig, "block_lqr.png")


def fig_block_lqr_pid():
    fig, ax = diagram_axes(9.6, 3.6, "Struktur LQR-PID (MIMO-PID ditala dari gain LQR)")
    ax.set_xlim(-0.4, 10.6)
    ax.set_ylim(-1.8, 1.7)
    sumnode(ax, (1.0, 0.6))
    box(ax, (3.6, 0.6), 3.0, 1.2,
        r"$K_p\,e + K_i\!\int e\,dt + K_d\,\dot e$" "\n(MIMO-PID 2×2)", fs=11)
    box(ax, (7.2, 0.6), 2.2, 1.0, "Plant\n" r"$(A,\,B,\,C)$")
    arrow(ax, [(-0.3, 0.6), (0.84, 0.6)], r"$r$", loff=(0, 0.04))
    arrow(ax, [(1.16, 0.6), (2.1, 0.6)], r"$e$", loff=(0, 0.04))
    arrow(ax, [(5.1, 0.6), (6.1, 0.6)], r"$u$", loff=(0, 0.04))
    arrow(ax, [(8.3, 0.6), (10.4, 0.6)], r"$y$", loff=(0, 0.04))
    ax.plot(9.4, 0.6, "o", color=INK, ms=4)
    arrow(ax, [(9.4, 0.6), (9.4, -0.5), (1.0, -0.5), (1.0, 0.44)])
    ax.text(0.7, 0.2, r"$-$", fontsize=13)
    ax.text(5.2, -1.15,
            r"$\hat K = K_{LQR}\,\Gamma^{+}$,   "
            r"$K_d=\frac{\hat K_3}{1+\hat K_3CB}$,   "
            r"$K_p=\hat K_2(1-K_dCB)$,   $K_i=\hat K_1(1-K_dCB)$",
            ha="center", fontsize=10.5, color=INK2)
    ax.text(5.2, -1.7, r"$K_{LQR}$ dari plant teraugmentasi  "
            r"$\tilde A=[A\ \ B;\ 0\ \ 0],\ \ \tilde B=[0;\ I]$",
            ha="center", fontsize=10.5, color=INK2)
    save(fig, "block_lqr_pid.png")


def fig_block_imp():
    fig, ax = diagram_axes(9.6, 3.6, "Struktur Internal Model Principle (IMP) + LQR")
    ax.set_xlim(-0.4, 10.6)
    ax.set_ylim(-1.8, 1.7)
    box(ax, (2.2, 0.6), 2.6, 1.1, "Kompensator\n(model internal)\n" r"$\dot x_a=A_ax_a+B_av$",
        fc="#e6f6ef", ec=C_3, fs=10)
    box(ax, (4.6, 0.6), 0.9, 0.7, r"$C_a$", fc="#e6f6ef", ec=C_3)
    box(ax, (7.2, 0.6), 2.2, 1.1, "Plant\n" r"$\dot x_p=A_px_p+B_pu$")
    arrow(ax, [(5.05, 0.6), (6.1, 0.6)], r"$u$", loff=(0, 0.04))
    arrow(ax, [(3.5, 0.6), (4.15, 0.6)])
    arrow(ax, [(8.3, 0.6), (10.4, 0.6)], r"$y=C_px_p$", loff=(0.3, 0.04))
    box(ax, (5.2, -0.9), 3.0, 0.7, r"$v = -K\,[x_p;\ x_a]$", fc="#fdf0ea", ec=C_OL)
    ax.plot(9.2, 0.6, "o", color=INK, ms=4)
    arrow(ax, [(9.2, 0.6), (9.2, -0.9), (6.7, -0.9)])
    arrow(ax, [(3.7, -0.9), (0.3, -0.9), (0.3, 0.6), (0.9, 0.6)])
    ax.text(0.45, -0.55, r"$v$", fontsize=11)
    ax.text(5.2, -1.6,
            r"$A=[A_p\ \ B_pC_a;\ 0\ \ A_a],\ \ "
            r"B=[0;\ B_a],\ \ "
            r"\mathrm{eig}(A_a)=\{-1,\,0\}$",
            ha="center", fontsize=10.5, color=INK2)
    save(fig, "block_imp.png")


def fig_ga_flow():
    fig, ax = diagram_axes(10.5, 2.9, "Alur Penalaan Bobot Q & R dengan Genetic Algorithm")
    ax.set_xlim(-0.2, 13.4)
    ax.set_ylim(-1.7, 1.3)
    steps = [
        "Populasi awal\n" r"$[q_1..q_n,\ r_1,r_2]$",
        r"$Q=\mathrm{diag}(q)$" "\n" r"$R=\mathrm{diag}(r)$",
        "Solusi Riccati\n" r"$K=\mathrm{lqr}(Q,R)$",
        "Simulasi\n" r"$\dot x=(A-BK)x$" "\n" r"$x_0=\mathbf{1}$, 10 s",
        "Hitung biaya\n" r"$J_{GA}$",
        "Seleksi,\ncrossover,\nmutasi",
    ]
    xs = np.linspace(1.0, 12.4, len(steps))
    for i, (x, s) in enumerate(zip(xs, steps)):
        box(ax, (x, 0.3), 1.85, 1.35, s, fs=9.5,
            fc="#fdf0ea" if i == 4 else BOX_FILL, ec=C_OL if i == 4 else BOX_EDGE)
        if i:
            arrow(ax, [(xs[i - 1] + 0.93, 0.3), (x - 0.93, 0.3)])
    arrow(ax, [(xs[-1], -0.38), (xs[-1], -0.95), (xs[1], -0.95), (xs[1], -0.38)],
          color=INK2)
    ax.text((xs[1] + xs[-1]) / 2, -1.25,
            "ulang hingga 1000 generasi (populasi 20)  →  " r"$Q_{opt},\ R_{opt},\ K_{opt}$",
            ha="center", fontsize=10, color=INK2)
    save(fig, "ga_flowchart.png")


# ---------------------------------------------------------------- results
def fig_step_response():
    t = np.linspace(0, 4.5, 2000)
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8))
    titles = [r"Input $F_a(t)$ (gaya aktuator/gangguan)", r"Input $u(t)$ (profil jalan)"]
    for j, ax in enumerate(axs):
        y_ol, y_cl = step_io(A, j, t), step_io(Acl, j, t)
        if j == 0:
            y_ol, y_cl = y_ol * 1e3, y_cl * 1e3
            ax.set_ylabel(r"$y_c$ [mm]  (step 1 N)")
        else:
            ax.set_ylabel(r"$y_c$ [m]  (step 1 m)")
        ax.plot(t, y_ol, color=C_OL, label="Tanpa kontrol (open-loop)")
        ax.plot(t, y_cl, color=C_LQR, label="Dengan LQR")
        ax.set_title(titles[j])
        ax.set_xlabel("Waktu [s]")
        ax.set_xlim(0, t[-1])
    axs[1].legend(loc="lower right")
    fig.suptitle(r"Step Response Perpindahan Bodi $y_c$ — $Q=\mathrm{diag}(10,1,10,1)$, "
                 r"$R=\mathrm{diag}(100,100)$", fontweight="bold", color=INK, y=1.02)
    fig.tight_layout()
    save(fig, "step_response_lqr.png")


def fig_pole_map():
    p_ol = np.linalg.eigvals(A)
    p_cl = np.linalg.eigvals(Acl)
    fig, axs = plt.subplots(1, 2, figsize=(10.5, 4.0),
                            gridspec_kw=dict(width_ratios=[1.25, 1]))
    for ax, zoom in zip(axs, [False, True]):
        ax.axvline(0, color=INK2, lw=1)
        ax.axhline(0, color=INK2, lw=1)
        ax.scatter(p_ol.real, p_ol.imag, marker="x", s=80, color=C_OL, lw=2.2,
                   label="Open-loop  eig(A)", zorder=3)
        ax.scatter(p_cl.real, p_cl.imag, marker="o", s=70, facecolor="none",
                   edgecolor=C_LQR, lw=2.2, label="LQR  eig(A−BK)", zorder=3)
        ax.set_xlabel(r"Re$(\lambda)$")
        ax.set_ylabel(r"Im$(\lambda)$")
        if zoom:
            ax.set_xlim(-10, 1)
            ax.set_ylim(-12, 12)
            ax.set_title("Perbesaran: pole dominan (mode bodi)")
            for p in p_ol[np.abs(p_ol) < 15]:
                if p.imag > 0:
                    ax.annotate(f"{p.real:.2f}{p.imag:+.2f}j", (p.real, p.imag),
                                textcoords="offset points", xytext=(-30, 12),
                                color=C_OL, fontsize=9)
            for p in p_cl[np.abs(p_cl) < 15]:
                if p.imag >= 0:
                    ax.annotate(f"{p.real:.2f}{p.imag:+.2f}j", (p.real, p.imag),
                                textcoords="offset points", xytext=(-40, 12),
                                color=C_LQR, fontsize=9)
        else:
            ax.set_title("Pole map sistem")
            ax.legend(loc="upper left")
    fig.tight_layout()
    save(fig, "pole_map.png")


def fig_bode():
    w = np.logspace(-0.5, 3, 800)
    fig, ax = plt.subplots(figsize=(8, 3.8))
    for Am, col, lab in [(A, C_OL, "Tanpa kontrol"), (Acl, C_LQR, "Dengan LQR")]:
        sys = signal.StateSpace(Am, B[:, 1:2], C, D[:, 1:2])
        w_, mag, _ = signal.bode(sys, w=w)
        ax.semilogx(w_ / (2 * np.pi), mag, color=col, label=lab)
    ax.axvspan(4, 8, color="#e34948", alpha=0.08, lw=0)
    ax.text(5.6, ax.get_ylim()[0] + 3, "4–8 Hz\n(sensitif bagi\ntubuh manusia)",
            ha="center", fontsize=8.5, color=INK2)
    ax.set_xlabel("Frekuensi [Hz]")
    ax.set_ylabel(r"$|Y_c/U|$ [dB]")
    ax.set_title(r"Transmisibilitas Jalan → Bodi ($u \rightarrow y_c$)")
    ax.legend(loc="upper right")
    fig.tight_layout()
    save(fig, "bode_transmissibility.png")


def fig_qr_sweep():
    t = np.linspace(0, 4, 1500)
    fig, axs = plt.subplots(1, 2, figsize=(11, 3.8), sharey=False)
    y_ol = step_io(A, 1, t)
    # (a) Q scaling, R fixed
    shades_b = ["#9ec5f4", "#5b9ae6", "#2a78d6", "#154a8e"]
    for q, col in zip([0.1, 1, 10, 100], shades_b):
        Kq = lqr(A, B, np.diag([q, 1, q, 1]), np.diag([100, 100]))
        y = step_io(A - B @ Kq, 1, t)
        axs[0].plot(t, y, color=col, label=rf"$q_{{1,3}}={q:g}$")
    axs[0].set_title(r"Variasi $Q=\mathrm{diag}(q,1,q,1)$, $R=100I$")
    # (b) R scaling, Q fixed
    for r, col in zip([1000, 100, 10, 1], shades_b):
        Kr = lqr(A, B, Q_rep, np.diag([r, r]))
        y = step_io(A - B @ Kr, 1, t)
        axs[1].plot(t, y, color=col, label=rf"$r={r:g}$")
    axs[1].set_title(r"Variasi $R=rI$, $Q=\mathrm{diag}(10,1,10,1)$")
    for ax in axs:
        ax.plot(t, y_ol, color=C_OL, lw=1.2, ls="--", label="Open-loop")
        ax.set_xlabel("Waktu [s]")
        ax.set_ylabel(r"$y_c$ [m]")
        ax.set_xlim(0, t[-1])
        ax.legend(fontsize=8.5, loc="upper left", bbox_to_anchor=(1.0, 1.0))
    fig.suptitle(r"Pengaruh Bobot $Q$ dan $R$ terhadap Step Response (input jalan $u$)",
                 fontweight="bold", color=INK, y=1.02)
    fig.tight_layout()
    save(fig, "qr_weight_effect.png")


if __name__ == "__main__":
    print_analysis()
    fig_quarter_car()
    fig_block_lqr()
    fig_block_lqr_pid()
    fig_block_imp()
    fig_ga_flow()
    fig_step_response()
    fig_pole_map()
    fig_bode()
    fig_qr_sweep()
