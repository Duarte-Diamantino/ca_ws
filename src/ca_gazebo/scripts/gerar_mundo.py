#!/usr/bin/env python3
"""Gera, a partir do desenho da pista FNR (pickup_description/maps/map_200.pgm):

  pickup_description/maps/fnr_ca.png + map.yaml   mapa para o map_server, a escala real e 2x mais fino
  ca_gazebo/worlds/media/pista.png                textura do chao (4x), asfalto + linhas + betao
  ca_gazebo/meshes/cone.stl, cone_faixa.stl       cone de sinalizacao
  ca_gazebo/worlds/pista.sdf                      armazem: paredes, pilares, estantes, paletes,
                                                  faixas no chao, cones fora da estrada

Escala: o lado menor da pista mede SHORT_SIDE_M (7 m, medida do Duarte). O desenho original
estava a 0.0121 m/px (6.49 m) -> tudo (mapa, caminho Q, obstaculos) cresce K = 7 / 6.49.

Correr a partir de ~/ca_ws:   python3 src/ca_gazebo/scripts/gerar_mundo.py
"""
import math
import os
import random

import numpy as np
from PIL import Image
from scipy import ndimage

WS_SRC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
MAPS = os.path.join(WS_SRC, 'pickup_description', 'maps')
GZ = os.path.join(WS_SRC, 'ca_gazebo')

SRC_IMAGE = os.path.join(MAPS, 'map_200.pgm')
RES0 = 0.0121                 # m/px do desenho original (map.yaml antigo)
ORIGIN0 = (-1.2, -0.75)       # origem antiga (canto inferior esquerdo)
SHORT_SIDE_M = 7.0            # lado menor real da pista FNR

# armazem (m, frame map)
ROOM = dict(x0=-4.0, x1=11.8, y0=-3.6, y1=9.8, h=3.2, t=0.2)


def load():
    im = Image.open(SRC_IMAGE).convert('L')
    a = np.array(im)
    k = SHORT_SIDE_M / (min(a.shape) * RES0)
    return im, a, k


# ---------------------------------------------------------------- mapa / textura
def upscale_binary(im, factor):
    """Aumenta a resolucao de um desenho a preto e branco sem serrilhado (Lanczos + limiar)."""
    big = im.resize((im.width * factor, im.height * factor), Image.LANCZOS)
    return (np.array(big) >= 128)


# centro do laco grande no desenho original (px): e o centro do arco do caminho Q, (5.1, 2.0) m
# a escala antiga -> col = (5.1 + 1.2) / 0.0121, linha = 536 - (2.0 + 0.75) / 0.0121
LOOP_CENTER_PX = (309, 521)   # (linha, coluna)


def regions(a):
    """Etiqueta zonas: 0 = linha branca, 1 = fora da pista, 2 = estrada, 3 = interior do laco.

    As linhas pintadas que atravessam a estrada (passadeira, linhas de paragem) partem-na em
    varias zonas fechadas, por isso: fora = zona que toca na borda da imagem, interior = zona que
    contem o centro do laco, estrada = todas as outras zonas fechadas.
    """
    white = a >= 128
    # fecha falhas de 1 px nas linhas continuas
    closed = ndimage.binary_closing(white, structure=np.ones((3, 3)), iterations=1)
    lab, _ = ndimage.label(~closed)
    border = set(np.unique(np.concatenate([lab[0], lab[-1], lab[:, 0], lab[:, -1]]))) - {0}
    loop = lab[LOOP_CENTER_PX]
    if loop == 0 or loop in border:
        raise RuntimeError('o centro do laco nao caiu numa zona fechada: rever LOOP_CENTER_PX')
    out = np.full(a.shape, 2, np.uint8)
    out[lab == 0] = 0
    out[np.isin(lab, list(border))] = 1
    out[lab == loop] = 3
    return out, closed


def write_map(im, k):
    b = upscale_binary(im, 2)
    # PNG (o map_server le PNG): ~60 KB em vez de 1.6 MB em PGM
    Image.fromarray(np.where(b, 254, 0).astype(np.uint8)).save(os.path.join(MAPS, 'fnr_ca.png'),
                                                                optimize=True)
    res = RES0 * k / 2
    with open(os.path.join(MAPS, 'map.yaml'), 'w') as f:
        f.write(
            '# Pista FNR (Conducao Autonoma) a escala real: lado menor = %.1f m.\n'
            '# Gerado por ca_gazebo/scripts/gerar_mundo.py a partir do map_200.pgm (que estava a\n'
            '# 0.0121 m/px = 6.49 m de lado menor) -> escala x%.4f e 2x mais resolucao.\n'
            'image: fnr_ca.png\nresolution: %.6f\norigin: [%.4f, %.4f, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
            % (SHORT_SIDE_M, k, res, ORIGIN0[0] * k, ORIGIN0[1] * k))
    return res


def write_texture(im, a, reg):
    f = 4
    lines = upscale_binary(im, f)
    reg_big = np.kron(reg, np.ones((f, f), np.uint8))
    rgb = np.zeros(lines.shape + (3,), np.uint8)
    rgb[:] = (118, 118, 112)                        # betao (fora da pista)
    rgb[reg_big == 2] = (46, 46, 50)                # asfalto
    rgb[reg_big == 3] = (92, 96, 88)                # interior das curvas
    rgb[lines] = (236, 236, 228)                    # linhas pintadas
    Image.fromarray(rgb).save(os.path.join(GZ, 'worlds', 'media', 'pista.png'), optimize=True)
    return lines.shape[1], lines.shape[0]


# ---------------------------------------------------------------- cones
def frustum_stl(path, r0, r1, z0, z1, n=24):
    tris = []
    for i in range(n):
        a0, a1 = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
        p = [(r0 * math.cos(a0), r0 * math.sin(a0), z0), (r0 * math.cos(a1), r0 * math.sin(a1), z0),
             (r1 * math.cos(a1), r1 * math.sin(a1), z1), (r1 * math.cos(a0), r1 * math.sin(a0), z1)]
        tris += [(p[0], p[1], p[2]), (p[0], p[2], p[3])]
        tris.append(((0, 0, z1), p[3], p[2]))          # tampa de cima
        tris.append(((0, 0, z0), p[1], p[0]))          # tampa de baixo
    with open(path, 'w') as fh:
        fh.write('solid cone\n')
        for t in tris:
            fh.write(' facet normal 0 0 0\n  outer loop\n')
            for v in t:
                fh.write('   vertex %.5f %.5f %.5f\n' % v)
            fh.write('  endloop\n endfacet\n')
        fh.write('endsolid cone\n')


def q_path_points():
    """Pontos do caminho Q (pickup_description, ja com a escala nova). [] se nao houver ROS."""
    try:
        import rclpy
        from pickup_description.q_path_publisher import QPathPublisher
    except ImportError:
        print('AVISO: sem rclpy/pickup_description -> cones sem verificacao do caminho Q')
        return []
    rclpy.init()
    try:
        q = QPathPublisher()
        cap = {}
        q.path_pub.publish = lambda m: cap.__setitem__('p', m)
        q.marker_pub.publish = lambda m: None
        q.on_timer()
        q.destroy_node()
        return [(p.pose.position.x, p.pose.position.y) for p in cap['p'].poses]
    finally:
        rclpy.shutdown()


def cone_positions(a, reg, res, origin, rng, path=(), path_clear=0.7):
    """Cones fora da estrada, a ~25 cm das linhas, afastados 1 m entre si e >= path_clear do Q."""
    dist = ndimage.distance_transform_edt(a < 128) * res
    h, w = a.shape
    yy, xx = np.mgrid[0:h, 0:w]
    edge = np.minimum.reduce([yy, xx, h - 1 - yy, w - 1 - xx]) * res
    cand = (reg != 2) & (reg != 0) & (dist > 0.22) & (dist < 0.30) & (edge > 0.35)
    pts = list(zip(*np.nonzero(cand)))
    rng.shuffle(pts)
    chosen = []
    for r, c in pts:
        x = origin[0] + (c + 0.5) * res
        y = origin[1] + (h - r - 0.5) * res
        if any(math.hypot(x - px, y - py) < path_clear for px, py in path):
            continue
        if all(math.hypot(x - cx, y - cy) > 1.0 for cx, cy in chosen):
            chosen.append((x, y))
    return chosen


# ---------------------------------------------------------------- SDF
def box(name, x, y, z, sx, sy, sz, rgb, yaw=0.0, collide=True):
    mat = ('<material><ambient>%.2f %.2f %.2f 1</ambient><diffuse>%.2f %.2f %.2f 1</diffuse>'
           '</material>' % (rgb + rgb))
    geo = '<geometry><box><size>%.3f %.3f %.3f</size></box></geometry>' % (sx, sy, sz)
    col = '<collision name="%s_c">%s</collision>' % (name, geo) if collide else ''
    return ('<link name="%s"><pose>%.3f %.3f %.3f 0 0 %.3f</pose>%s'
            '<visual name="%s_v">%s%s</visual></link>' % (name, x, y, z, yaw, col, name, geo, mat))


def model(name, links):
    return '<model name="%s"><static>true</static>%s</model>\n' % (name, ''.join(links))


def warehouse(rng):
    R = ROOM
    cx, cy = (R['x0'] + R['x1']) / 2, (R['y0'] + R['y1']) / 2
    lx, ly, h, t = R['x1'] - R['x0'], R['y1'] - R['y0'], R['h'], R['t']
    wall = (0.80, 0.80, 0.76)
    links = [
        box('parede_sul', cx, R['y0'], h / 2, lx + t, t, h, wall),
        box('parede_norte', cx, R['y1'], h / 2, lx + t, t, h, wall),
        box('parede_oeste', R['x0'], cy, h / 2, t, ly, h, wall),
        box('parede_este', R['x1'], cy, h / 2, t, ly, h, wall),
        # faixa inferior das paredes (rodape escuro) e portao de enrolar na parede este
        box('rodape_sul', cx, R['y0'] + t / 2 + 0.005, 0.15, lx, 0.01, 0.3, (0.25, 0.25, 0.28), collide=False),
        box('rodape_norte', cx, R['y1'] - t / 2 - 0.005, 0.15, lx, 0.01, 0.3, (0.25, 0.25, 0.28), collide=False),
        box('portao', R['x1'] - t / 2 - 0.01, 1.5, 1.3, 0.02, 3.0, 2.6, (0.45, 0.47, 0.50), collide=False),
    ]
    out = model('armazem_paredes', links)

    # faixas amarelas no chao, a 0.6 m das paredes
    yel = (0.95, 0.78, 0.05)
    m = 0.6
    out += model('faixas_chao', [
        box('fs', cx, R['y0'] + m, 0.002, lx - 2 * m, 0.08, 0.004, yel, collide=False),
        box('fn', cx, R['y1'] - m, 0.002, lx - 2 * m, 0.08, 0.004, yel, collide=False),
        box('fo', R['x0'] + m, cy, 0.002, 0.08, ly - 2 * m, 0.004, yel, collide=False),
        box('fe', R['x1'] - m, cy, 0.002, 0.08, ly - 2 * m, 0.004, yel, collide=False),
    ])

    # pilares
    steel = (0.30, 0.34, 0.40)
    pil = [(-2.8, -2.4), (-2.8, 3.2), (-2.8, 8.6), (10.6, -2.4), (10.6, 3.2), (10.6, 8.6), (3.9, 8.6)]
    out += model('pilares', [box('p%d' % i, x, y, h / 2, 0.3, 0.3, h, steel) for i, (x, y) in enumerate(pil)])

    # estantes ao longo da parede norte, com caixas
    for j, xc in enumerate((-1.6, 1.4, 6.4, 9.0)):
        yc, L, D = R['y1'] - t / 2 - 0.45, 2.2, 0.8
        blue, orange = (0.10, 0.25, 0.60), (0.95, 0.45, 0.05)
        ls = []
        for k, (dx, dy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1))):
            ls.append(box('m%d' % k, xc + dx * (L / 2 - 0.03), yc + dy * (D / 2 - 0.03), 1.0, 0.06, 0.06, 2.0, blue))
        for k, z in enumerate((0.25, 0.95, 1.65)):
            ls.append(box('s%d' % k, xc, yc, z, L, D, 0.04, orange))
            x = xc - L / 2 + 0.1
            while x < xc + L / 2 - 0.35:
                bw, bh = rng.uniform(0.3, 0.5), rng.uniform(0.2, 0.45)
                if rng.random() < 0.8:
                    tone = rng.uniform(0.85, 1.1)
                    ls.append(box('c%d_%d' % (k, len(ls)), x + bw / 2, yc + rng.uniform(-0.1, 0.1),
                                  z + 0.02 + bh / 2, bw, 0.55, bh,
                                  (0.62 * tone, 0.45 * tone, 0.26 * tone)))
                x += bw + rng.uniform(0.04, 0.12)
        out += model('estante_%d' % j, ls)

    # paletes com caixas (cantos sul)
    for j, (xc, yc, yaw) in enumerate(((-2.9, -2.6, 0.0), (-1.4, -2.7, 0.15), (9.6, -2.6, -0.1), (10.7, 0.2, 1.57))):
        ls = [box('palete', xc, yc, 0.07, 1.2, 0.8, 0.14, (0.65, 0.50, 0.30), yaw=yaw)]
        z = 0.14
        for k in range(rng.randint(1, 3)):
            bh = rng.uniform(0.25, 0.4)
            ls.append(box('cx%d' % k, xc, yc, z + bh / 2, 1.1 - 0.1 * k, 0.75 - 0.05 * k, bh,
                          (0.60, 0.44, 0.25), yaw=yaw))
            z += bh
        out += model('palete_%d' % j, ls)
    return out


def cones_sdf(pos, media):
    out = ''
    for i, (x, y) in enumerate(pos):
        out += (
            '<model name="cone_%d"><static>true</static><pose>%.3f %.3f 0 0 0 0</pose><link name="l">'
            '<collision name="c"><pose>0 0 0.14 0 0 0</pose><geometry><cylinder><radius>0.06</radius>'
            '<length>0.28</length></cylinder></geometry></collision>'
            '<visual name="base"><pose>0 0 0.0075 0 0 0</pose><geometry><box><size>0.19 0.19 0.015</size>'
            '</box></geometry><material><ambient>0.9 0.35 0.02 1</ambient><diffuse>0.9 0.35 0.02 1</diffuse>'
            '</material></visual>'
            '<visual name="corpo"><geometry><mesh><uri>%s/cone.stl</uri></mesh></geometry>'
            '<material><ambient>1 0.4 0.02 1</ambient><diffuse>1 0.4 0.02 1</diffuse></material></visual>'
            '<visual name="faixa"><geometry><mesh><uri>%s/cone_faixa.stl</uri></mesh></geometry>'
            '<material><ambient>0.95 0.95 0.95 1</ambient><diffuse>0.95 0.95 0.95 1</diffuse></material>'
            '</visual></link></model>\n' % (i, x, y, media, media))
    return out


def world(k, tex_size, cones, rng):
    w = 778 * RES0 * k
    h = 536 * RES0 * k
    ox, oy = ORIGIN0[0] * k, ORIGIN0[1] * k
    R = ROOM
    lights = ''.join(
        '<light type="point" name="lamp_%d"><pose>%.1f %.1f %.1f 0 0 0</pose><diffuse>0.55 0.55 0.5 1</diffuse>'
        '<specular>0.1 0.1 0.1 1</specular><attenuation><range>14</range><constant>0.4</constant>'
        '<linear>0.02</linear><quadratic>0.004</quadratic></attenuation><cast_shadows>false</cast_shadows>'
        '</light>\n' % (i, x, y, R['h'] - 0.3)
        for i, (x, y) in enumerate([(0, 0), (4, 0), (8, 0), (0, 6), (4, 6), (8, 6)]))
    return '''<?xml version="1.0"?>
<!--
  GERADO por ca_gazebo/scripts/gerar_mundo.py - nao editar a mao (correr o script outra vez).
  Armazem com a pista FNR (Conducao Autonoma) a escala real (lado menor %.1f m, escala x%.4f).
  Frame do mundo = frame "map" do ROS. O launch acrescenta os obstaculos no marcador abaixo.
-->
<sdf version="1.8">
  <world name="pista">
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="ignition-gazebo-physics-system" name="ignition::gazebo::systems::Physics"/>
    <plugin filename="ignition-gazebo-user-commands-system" name="ignition::gazebo::systems::UserCommands"/>
    <plugin filename="ignition-gazebo-scene-broadcaster-system" name="ignition::gazebo::systems::SceneBroadcaster"/>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.2 0.22 0.25 1</background>
      <grid>false</grid>
    </scene>
    <light type="directional" name="sol">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.5 0.5 0.5 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
%s
    <model name="chao">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><ambient>0.46 0.46 0.44 1</ambient><diffuse>0.46 0.46 0.44 1</diffuse></material>
        </visual>
      </link>
    </model>
    <!-- pista: %d x %d px, %.3f x %.3f m, canto inferior esquerdo em (%.4f, %.4f) -->
    <model name="mapa">
      <static>true</static>
      <pose>%.4f %.4f 0.002 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>%.4f %.4f</size></plane></geometry>
          <material>
            <ambient>1 1 1 1</ambient>
            <diffuse>1 1 1 1</diffuse>
            <pbr><metal><albedo_map>media/pista.png</albedo_map></metal></pbr>
          </material>
        </visual>
      </link>
    </model>
%s%s
    <!-- OBSTACULOS -->
  </world>
</sdf>
''' % (SHORT_SIDE_M, k, lights, tex_size[0], tex_size[1], w, h, ox, oy,
       ox + w / 2, oy + h / 2, w, h, warehouse(rng), cones)


def main():
    rng = random.Random(7)
    im, a, k = load()
    res_map = write_map(im, k)
    reg, _ = regions(a)
    tex = write_texture(im, a, reg)
    os.makedirs(os.path.join(GZ, 'meshes'), exist_ok=True)
    frustum_stl(os.path.join(GZ, 'meshes', 'cone.stl'), 0.085, 0.015, 0.015, 0.30)
    frustum_stl(os.path.join(GZ, 'meshes', 'cone_faixa.stl'), 0.066, 0.052, 0.12, 0.17)
    origin = (ORIGIN0[0] * k, ORIGIN0[1] * k)
    path = q_path_points()
    # metade dos pontos do caminho chega (ficam a ~9 cm uns dos outros) e e 2x mais rapido
    cones = cone_positions(a, reg, RES0 * k, origin, rng, path=path[::2])
    sdf = world(k, tex, cones_sdf(cones, 'meshes'), rng)
    with open(os.path.join(GZ, 'worlds', 'pista.sdf'), 'w') as f:
        f.write(sdf)
    counts = {name: int((reg == v).sum()) for name, v in (('linhas', 0), ('fora', 1), ('estrada', 2), ('interior', 3))}
    print('escala K = %.5f  pista %.2f x %.2f m  mapa %.5f m/px  textura %dx%d  cones %d  zonas(px) %s'
          % (k, 778 * RES0 * k, 536 * RES0 * k, res_map, tex[0], tex[1], len(cones), counts))


if __name__ == '__main__':
    main()
