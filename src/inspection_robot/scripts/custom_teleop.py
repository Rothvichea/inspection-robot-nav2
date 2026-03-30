#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import sys
import tty
import termios
import select
import threading
import time

# ═══════════════════════════════════════════════════════════
#  4-WHEEL STEERING TELEOP — HOLD-TO-MOVE WITH VELOCITY RAMP
#
#  ┌─────────────────────────────────────────────────────┐
#  │  CAR STEERING (linear.x + angular.z)                │
#  │   w = Forward          s = Backward                 │
#  │   a = Steer Left       d = Steer Right              │
#  │   (pure a/d with no w/s = spin in place)            │
#  │                                                     │
#  │  DRIVE + STEER ARCS                                 │
#  │   q = Fwd+Left         e = Fwd+Right                │
#  │   z = Bwd+Left         c = Bwd+Right                │
#  │                                                     │
#  │  CRAB MODE  (all 4 wheels pivot 90°)                │
#  │   j = Crab Left        l = Crab Right               │
#  │   u = Fwd+Crab Left    o = Fwd+Crab Right           │
#  │                                                     │
#  │  h     = Home wheels (reset to straight)            │
#  │  SPACE = Emergency stop    Ctrl+C = Quit            │
#  │  Hold key to move — release to stop                 │
#  └─────────────────────────────────────────────────────┘
# ═══════════════════════════════════════════════════════════

BANNER = """
╔═══════════════════════════════════════════════════════╗
║        INSPECTION ROBOT — 4WS TELEOP CONTROL         ║
╠═══════════════════════════════════════════════════════╣
║                                                       ║
║  CAR STEERING                                         ║
║    w = Forward           s = Backward                 ║
║    a = Steer Left        d = Steer Right              ║
║    (pure a/d = spin in place)                         ║
║                                                       ║
║  DRIVE + STEER ARCS                                   ║
║    q = Fwd + Steer Left  e = Fwd + Steer Right        ║
║    z = Bwd + Steer Left  c = Bwd + Steer Right        ║
║                                                       ║
║  CRAB MODE  (all 4 wheels turn sideways)              ║
║    j = Crab Left         l = Crab Right               ║
║    u = Fwd + Crab Left   o = Fwd + Crab Right         ║
║                                                       ║
║  h     = Home wheels (reset to straight, no stop)     ║
║  SPACE = Emergency Stop      Ctrl+C = Quit            ║
║  ** Hold key to move — release to stop                ║
╚═══════════════════════════════════════════════════════╝
"""

# ── Speed settings ────────────────────────────────────────
LINEAR_SPEED  = 6.0   # m/s  (~21 km/h)
STRAFE_SPEED  = 6.0   # m/s
ANGULAR_SPEED = 3.5   # rad/s

# ── Ramp rates (units per second) ────────────────────────
LINEAR_RAMP   = 14.0  # m/s²  → 0→6 m/s in ~0.43 s
ANGULAR_RAMP  = 12.0  # rad/s² → 0→3.5 rad/s in ~0.29 s

# ── Loop timing ───────────────────────────────────────────
PUBLISH_RATE        = 0.05   # 20 Hz — smoother than 10 Hz
KEY_RELEASE_TIMEOUT = 0.15   # s — stop quickly after key release

_L = LINEAR_SPEED
_S = STRAFE_SPEED
_A = ANGULAR_SPEED

# Key → (target_vx, target_vy, target_wz)
KEY_MAP = {
    'w': ( _L,  0.0,  0.0),
    's': (-_L,  0.0,  0.0),
    'a': ( 0.0, 0.0,  _A ),
    'd': ( 0.0, 0.0, -_A ),
    'q': ( _L,  0.0,  _A ),
    'e': ( _L,  0.0, -_A ),
    'z': (-_L,  0.0,  _A ),
    'c': (-_L,  0.0, -_A ),
    'j': ( 0.0,  _S,  0.0),
    'l': ( 0.0, -_S,  0.0),
    'u': ( _L,   _S,  0.0),
    'o': ( _L,  -_S,  0.0),
}

LABELS = {
    'w': 'FORWARD',        's': 'BACKWARD',
    'a': 'STEER/SPIN LEFT','d': 'STEER/SPIN RIGHT',
    'q': 'ARC FWD-LEFT',   'e': 'ARC FWD-RIGHT',
    'z': 'ARC BWD-LEFT',   'c': 'ARC BWD-RIGHT',
    'j': 'CRAB LEFT',      'l': 'CRAB RIGHT',
    'u': 'FWD+CRAB LEFT',  'o': 'FWD+CRAB RIGHT',
}


def _ramp(current, target, rate, dt):
    """Step current toward target at rate units/sec."""
    step = rate * dt
    diff = target - current
    if abs(diff) <= step:
        return target
    return current + step * (1.0 if diff > 0 else -1.0)


class TeleopNode(Node):

    def __init__(self):
        super().__init__('custom_teleop')
        self.pub      = self.create_publisher(Twist, 'cmd_vel', 10)
        self.home_pub = self.create_publisher(Empty, 'steering_home', 10)
        self.get_logger().info('4WS teleop node started.')

    def publish_vel(self, vx, vy, wz):
        msg = Twist()
        msg.linear.x  = float(vx)
        msg.linear.y  = float(vy)
        msg.angular.z = float(wz)
        self.pub.publish(msg)

    def home_wheels(self):
        self.home_pub.publish(Empty())

    def stop(self):
        self.publish_vel(0.0, 0.0, 0.0)


def main():
    rclpy.init()
    node = TeleopNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    old_settings = termios.tcgetattr(sys.stdin)

    lock          = threading.Lock()
    target_vx     = 0.0
    target_vy     = 0.0
    target_wz     = 0.0
    target_label  = ''
    last_key_time = 0.0
    running       = True
    estop         = False
    wheel_home    = False

    def key_reader():
        nonlocal target_vx, target_vy, target_wz, target_label
        nonlocal last_key_time, running, estop, wheel_home
        try:
            while running:
                if select.select([sys.stdin], [], [], 0.02)[0]:
                    key = sys.stdin.read(1)

                    if key == '\x03':
                        with lock:
                            running = False
                        return

                    if key == ' ':
                        with lock:
                            target_vx = target_vy = target_wz = 0.0
                            target_label  = ''
                            last_key_time = 0.0
                            estop = True
                        continue

                    if key == 'h':
                        with lock:
                            wheel_home = True
                        continue

                    if key in KEY_MAP:
                        vx, vy, wz = KEY_MAP[key]
                        with lock:
                            target_vx    = vx
                            target_vy    = vy
                            target_wz    = wz
                            target_label = LABELS.get(key, key.upper())
                            last_key_time = time.monotonic()
                            estop = False
        except Exception:
            with lock:
                running = False

    print(BANNER)
    print(f'  Drive speed  : {LINEAR_SPEED} m/s  (~{LINEAR_SPEED*3.6:.0f} km/h)')
    print(f'  Crab speed   : {STRAFE_SPEED} m/s')
    print(f'  Steer speed  : {ANGULAR_SPEED} rad/s\n')

    tty.setraw(sys.stdin.fileno())

    reader = threading.Thread(target=key_reader, daemon=True)
    reader.start()

    # Ramped current velocities
    cur_vx = cur_vy = cur_wz = 0.0
    is_moving    = False
    printed_stop = False

    try:
        while True:
            with lock:
                if not running:
                    break
                tvx   = target_vx
                tvy   = target_vy
                twz   = target_wz
                label = target_label
                lkt   = last_key_time
                do_estop = estop
                do_home  = wheel_home
                if wheel_home:
                    wheel_home = False

            now = time.monotonic()
            dt  = PUBLISH_RATE

            if do_home:
                node.home_wheels()
                sys.stdout.write('\r  [HOME] Wheels reset to straight                             \r')
                sys.stdout.flush()
                time.sleep(PUBLISH_RATE)
                continue

            if do_estop:
                cur_vx = cur_vy = cur_wz = 0.0
                node.stop()
                is_moving = printed_stop = False
                sys.stdout.write('\r  ** EMERGENCY STOP **                                        \r')
                sys.stdout.flush()
                with lock:
                    estop = False
                time.sleep(PUBLISH_RATE)
                continue

            key_held = lkt > 0 and (now - lkt) < KEY_RELEASE_TIMEOUT

            if key_held:
                # Ramp toward target velocity
                cur_vx = _ramp(cur_vx, tvx, LINEAR_RAMP,  dt)
                cur_vy = _ramp(cur_vy, tvy, LINEAR_RAMP,  dt)
                cur_wz = _ramp(cur_wz, twz, ANGULAR_RAMP, dt)
                node.publish_vel(cur_vx, cur_vy, cur_wz)
                is_moving    = True
                printed_stop = False

                if cur_vy != 0.0 and cur_vx == 0.0:
                    mode = 'CRAB'
                elif cur_vy != 0.0:
                    mode = 'CRAB+DRIVE'
                elif cur_wz != 0.0 and abs(cur_vx) < 0.01:
                    mode = 'SPIN'
                elif cur_wz != 0.0:
                    mode = '4WS-ARC'
                else:
                    mode = 'STRAIGHT'

                sys.stdout.write(
                    f'\r  [{mode}] {label}  vx={cur_vx:.1f} vy={cur_vy:.1f} wz={cur_wz:.1f}      \r'
                )
                sys.stdout.flush()

            else:
                # Ramp down to zero
                cur_vx = _ramp(cur_vx, 0.0, LINEAR_RAMP,  dt)
                cur_vy = _ramp(cur_vy, 0.0, LINEAR_RAMP,  dt)
                cur_wz = _ramp(cur_wz, 0.0, ANGULAR_RAMP, dt)

                if abs(cur_vx) > 0.01 or abs(cur_vy) > 0.01 or abs(cur_wz) > 0.01:
                    node.publish_vel(cur_vx, cur_vy, cur_wz)
                    printed_stop = False
                else:
                    cur_vx = cur_vy = cur_wz = 0.0
                    if is_moving or not printed_stop:
                        node.stop()
                        is_moving    = False
                        printed_stop = True
                        sys.stdout.write('\r  [STOPPED] Waiting for input...                             \r')
                        sys.stdout.flush()

            time.sleep(PUBLISH_RATE)

    except KeyboardInterrupt:
        pass
    except Exception as ex:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        print(f'\nError: {ex}')
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.stop()
        print('\n  Robot stopped. Shutting down...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
