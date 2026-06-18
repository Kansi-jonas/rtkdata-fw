# -*- coding: utf-8 -*-
# Sequenzdiagramm: RTKdata-Firmware von Boot/Verbinden bis Korrekturdaten-Auslieferung.
# Codeexakt aus rtkdata-fw/main (main.c, provisioning.c, gnss.c, interface/ntrip_server.c).
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Rectangle

# ---- Akteure (Lifelines) ----
actors = [
    ("UM980\n(GNSS-Empfaenger)", 1.6,  "#2d6a4f"),
    ("ESP32\n(Firmware)",        5.0,  "#1d3557"),
    ("Integrity Engine\n(IE / Render)", 8.6, "#6a4c93"),
    ("NTRIP-Caster\n(Alberding)", 12.0, "#9c6644"),
    ("Rover\n(Kunde)",           14.8, "#1b4965"),
]
ax_of = {name.split("\n")[0]: x for name, x, _ in actors}

# ---- Phasen + Nachrichten ----
# kind: req (durchgezogen), resp (gestrichelt), self (Schleife auf Lifeline), note
phases = [
    ("A  Boot & Verbinden", "#eef6f2", [
        ("ESP32", "UM980", "gnss_init: RTCM 1005@10s + 1077/1087/1097/1127 MSM7@1Hz,\nMASK 10 deg, survey-in (mode base time 300 1.5)", "req"),
        ("ESP32", "ESP32", "WiFi-Connect -> IP -> SNTP-Zeit -> ota_boot_check", "self"),
    ]),
    ("B  Enroll (Token + Credential)", "#eef0f8", [
        ("ESP32", "Integrity Engine", "POST /api/edge/enroll\n{device_id, mac, fw, nonce, HMAC(Fleet-Secret)}", "req"),
        ("Integrity Engine", "ESP32", "200 {token, caster:{host, mountpoint, user, pw}}", "resp"),
        ("ESP32", "ESP32", "Token + Caster-Creds -> NVS,  state = PROVISIONING", "self"),
    ]),
    ("C  GNSS-Rohstrom", "#eef6f2", [
        ("UM980", "ESP32", "RTCM ueber UART: 1005 (ARP) + MSM7\n(provisorische Survey-in-Position)", "req"),
    ]),
    ("D  NTRIP-Upload (Korrekturdaten an Caster)", "#f7f1ea", [
        ("ESP32", "NTRIP-Caster", "NTRIP SOURCE Handshake (mountpoint + creds aus NVS)", "req"),
        ("NTRIP-Caster", "ESP32", "ICY 200 OK  (CASTER_READY)", "resp"),
        ("ESP32", "NTRIP-Caster", "RTCM 1005 + MSM  =  Korrekturdaten  [Dauerstrom]", "req"),
    ]),
    ("E  Reifung + praezise Koordinate (Closed Loop)", "#eef0f8", [
        ("ESP32", "Integrity Engine", "POST /api/edge/heartbeat {HMAC(token)}   [alle poll_s]", "req"),
        ("Integrity Engine", "ESP32", "{state, matures_in_s, position(lat,lon,h), config_delta}", "resp"),
        ("ESP32", "UM980", "gnss_set_fixed_base(lat,lon,h)\n(praezise ITRF2020-Koord aus IE-PPP, einmalig)", "req"),
        ("UM980", "ESP32", "broadcastet ab jetzt praezises 1005", "resp"),
    ]),
    ("F  Auslieferung an den Kunden", "#eaf1f3", [
        ("Integrity Engine", "Integrity Engine", "state: provisioning -> maturing -> active  (sauberer Strom validiert)", "self"),
        ("NTRIP-Caster", "Rover", "RTCM-Korrekturen (1005 -> Katalog-ECEF, MSM)  ->  cm-RTK-Fix", "req"),
    ]),
]

# ---- Layout ----
fig, ax = plt.subplots(figsize=(17, 13.5))
top = 0.0
row_h = 1.0
phase_gap = 0.55
y = top

# vertikale Positionen vorab berechnen
items = []  # (type, payload, y)
for ptitle, pcol, msgs in phases:
    y -= phase_gap
    band_top = y + 0.30
    for m in msgs:
        y -= row_h
        items.append(("msg", m, y))
    band_bot = y - 0.25
    items.insert(len(items) - len(msgs), ("phase", (ptitle, pcol, band_top, band_bot), None))

bottom = y - 0.6
header_y = top + 1.1
footer_y = bottom - 0.4

# Phasenbaender zuerst (Hintergrund)
xL, xR = 0.2, 15.9
for typ, payload, _ in items:
    if typ == "phase":
        ptitle, pcol, btop, bbot = payload
        ax.add_patch(Rectangle((xL, bbot), xR - xL, btop - bbot, facecolor=pcol,
                               edgecolor="none", zorder=0))
        ax.text(xL + 0.12, btop - 0.04, ptitle, ha="left", va="top",
                fontsize=11, fontweight="bold", color="#333", zorder=5)

# Lifelines + Header/Footer
for name, x, col in actors:
    short = name.split("\n")[0]
    ax.plot([x, x], [header_y - 0.55, footer_y + 0.35], color="#9aa0a6",
            lw=1.1, ls=(0, (4, 3)), zorder=1)
    for yy in (header_y, footer_y):
        box = FancyBboxPatch((x - 1.35, yy - 0.32), 2.7, 0.64,
                             boxstyle="round,pad=0.02,rounding_size=0.08",
                             facecolor=col, edgecolor="none", zorder=6)
        ax.add_patch(box)
        ax.text(x, yy, name, ha="center", va="center", color="white",
                fontsize=10.5, fontweight="bold", zorder=7)

# Nachrichten
for typ, payload, yy in items:
    if typ != "msg":
        continue
    src, dst, text, kind = payload
    xs, xd = ax_of[src], ax_of[dst]
    if kind == "self":
        w = 1.5
        ax.add_patch(FancyArrowPatch((xs, yy + 0.16), (xs + w, yy + 0.16),
                     arrowstyle="-", color="#444", lw=1.4, zorder=4))
        ax.add_patch(FancyArrowPatch((xs + w, yy + 0.16), (xs + w, yy - 0.12),
                     arrowstyle="-", color="#444", lw=1.4, zorder=4))
        ax.add_patch(FancyArrowPatch((xs + w, yy - 0.12), (xs + 0.03, yy - 0.12),
                     arrowstyle="-|>", mutation_scale=14, color="#444", lw=1.4, zorder=4))
        ax.text(xs + w + 0.2, yy + 0.02, text, ha="left", va="center",
                fontsize=9, color="#222", zorder=5)
    else:
        dashed = (kind == "resp")
        col = "#3a7d44" if not dashed else "#7a5195"
        ax.add_patch(FancyArrowPatch((xs, yy), (xd, yy), arrowstyle="-|>",
                     mutation_scale=16, color=col, lw=1.7,
                     linestyle="--" if dashed else "-", zorder=4))
        midx = (xs + xd) / 2
        ax.text(midx, yy + 0.14, text, ha="center", va="bottom",
                fontsize=9, color="#111", zorder=5,
                bbox=dict(boxstyle="round,pad=0.18", fc="white", ec="none", alpha=0.85))

# Titel + Legende
ax.text((xL + xR) / 2, header_y + 0.95,
        "RTKdata-Firmware: vom Verbinden bis zur Korrekturdaten-Auslieferung",
        ha="center", va="bottom", fontsize=15, fontweight="bold", color="#1d3557")
ax.text((xL + xR) / 2, header_y + 0.62,
        "UM980 + ESP32  |  FW v1.0.2  |  durchgezogen = Request/Stream, gestrichelt = Antwort",
        ha="center", va="bottom", fontsize=9.5, color="#555")

ax.set_xlim(0, 16.1)
ax.set_ylim(footer_y - 0.7, header_y + 1.5)
ax.axis("off")
plt.tight_layout()
out = r"C:\Users\Jonas Becker\Desktop\rtkdata-fw\docs\fw-correction-flow.png"
plt.savefig(out, dpi=160, bbox_inches="tight", facecolor="white")
print("saved:", out)
