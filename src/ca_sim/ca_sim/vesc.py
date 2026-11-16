"""Calibracao dos comandos do VESC e geometria do carro (partilhado pelos simuladores).

Os controladores da TESE falam "VESC":
  /commands/motor/speed    (Float64, RPM/ERPM)      -> velocidade [m/s]
  /commands/servo/position (Float64, servo 0..1)    -> angulo de direcao [rad]
O MPC e o DWA assumem calibracoes DIFERENTES para o mesmo carro (ver README),
por isso cada simulador recebe a calibracao do controlador que esta a testar.
"""
import math

# Calibracoes que cada controlador assume (copiadas do codigo da TESE).
# ATENCAO: sao diferentes entre si -> so uma pode estar certa para o carro real.
PROFILES = {
    'mpc': dict(  # src/mpc_python/mpc_python/mpc_controller.py
        speed_cmd_per_mps=7000.0,
        servo_center=0.5751,
        servo_span=0.8760 - 0.5751,
        steer_max_rad=math.pi / 8.0,
        steer_sign=-1.0,
        wheelbase=0.335,
    ),
    'dwa': dict(  # src/dwa_follower/src/dwa_global_follower.cpp
        speed_cmd_per_mps=23070.0 / 3.0,
        servo_center=0.5304,
        servo_span=0.94299 - 0.5304,
        steer_max_rad=math.pi / 6.0,
        steer_sign=-1.0,
        wheelbase=0.335,
    ),
}


# Geometria do carro real (Traxxas 1/10), medida no CAD (CA_SIM/carro_com_rodas.stl).
# E a do carro no Gazebo; a dos controladores esta em PROFILES (wheelbase 0.335 != 0.324).
CAR = dict(wheelbase=0.324, track=0.1975, wheel_radius=0.0545)

# Escala da pista FNR: o desenho map_200.pgm estava a 0.0121 m/px (lado menor 6.49 m);
# o lado menor real e 7 m (ver ca_gazebo/scripts/gerar_mundo.py). O caminho Q e os obstaculos
# dos testes estao multiplicados por este fator.
TRACK_SCALE = 7.0 / (536 * 0.0121)


class VescCalibration:
    """Converte comandos do VESC em velocidade e angulo de direcao (modelo bicicleta)."""

    def __init__(self, speed_cmd_per_mps, servo_center, servo_span, steer_max_rad,
                 steer_sign=-1.0):
        if speed_cmd_per_mps == 0 or servo_span == 0:
            raise ValueError('speed_cmd_per_mps e servo_span nao podem ser 0')
        self.k_speed = float(speed_cmd_per_mps)
        self.servo_center = float(servo_center)
        self.servo_span = float(servo_span)
        self.steer_max = float(steer_max_rad)
        self.steer_sign = float(steer_sign)

    def speed(self, motor_cmd):
        """v [m/s] = comando / speed_cmd_per_mps."""
        return float(motor_cmd) / self.k_speed

    def steer(self, servo_pos):
        """delta [rad] (positivo = esquerda) a partir da posicao do servo, saturado."""
        ratio = (float(servo_pos) - self.servo_center) / self.servo_span
        ratio = max(-1.0, min(1.0, ratio))
        return self.steer_sign * ratio * self.steer_max

    # inversas (para conduzir o carro com velocidade / angulo, ex. teclado)
    def motor_cmd(self, v):
        """Comando do motor (RPM) para a velocidade v [m/s]."""
        return float(v) * self.k_speed

    def servo_pos(self, delta):
        """Posicao do servo para o angulo delta [rad] (positivo = esquerda), saturado."""
        ratio = float(delta) / (self.steer_sign * self.steer_max)
        ratio = max(-1.0, min(1.0, ratio))
        return self.servo_center + ratio * self.servo_span

    @classmethod
    def from_profile(cls, name):
        p = PROFILES[name]
        return cls(p['speed_cmd_per_mps'], p['servo_center'], p['servo_span'],
                   p['steer_max_rad'], p['steer_sign'])


# parametros ROS comuns aos simuladores (nome -> default = calibracao do MPC)
CALIBRATION_PARAMS = dict(PROFILES['mpc'])


def declare_calibration(node):
    """Declara os parametros de calibracao num no e devolve (VescCalibration, wheelbase)."""
    for name, default in CALIBRATION_PARAMS.items():
        node.declare_parameter(name, default)
    g = lambda n: float(node.get_parameter(n).value)  # noqa: E731
    calib = VescCalibration(g('speed_cmd_per_mps'), g('servo_center'), g('servo_span'),
                            g('steer_max_rad'), g('steer_sign'))
    return calib, g('wheelbase')


def ackermann_angles(delta, wheelbase, track):
    """Angulos (esquerda, direita) das rodas da frente para um angulo de bicicleta delta.

    A roda de dentro vira mais do que a de fora; delta = 0 -> (0, 0).
    """
    if abs(delta) < 1e-9:
        return 0.0, 0.0
    radius = wheelbase / math.tan(delta)  # raio da trajetoria do eixo traseiro (com sinal)
    left = math.atan(wheelbase / (radius - track / 2.0))
    right = math.atan(wheelbase / (radius + track / 2.0))
    return left, right


def rear_wheel_speeds(v, delta, wheelbase, track):
    """Velocidades lineares (esquerda, direita) das rodas de tras (diferencial ideal)."""
    k = track * math.tan(delta) / (2.0 * wheelbase)
    return v * (1.0 - k), v * (1.0 + k)


def start_pose_from_path(points, lookahead=5):
    """Pose inicial (x, y, yaw) no inicio de um caminho [(x, y), ...], virada para o ponto lookahead."""
    if len(points) < 2:
        raise ValueError('caminho com menos de 2 pontos')
    x0, y0 = points[0]
    x1, y1 = points[min(lookahead, len(points) - 1)]
    return x0, y0, math.atan2(y1 - y0, x1 - x0)
