import math, time, argparse
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.srv import SetPen, TeleportAbsolute
from PIL import Image

CENTER = 5.5
XMIN, XMAX, YMIN, YMAX = 0.8, 10.2, 0.8, 10.2

def polar_xy(r, th):
    return (CENTER + r*math.cos(th), CENTER + r*math.sin(th))

def world_to_img(x,y, cx,cy, px_per_unit, rot_rad, iw,ih, ox,oy):
    dx, dy = (x-cx), (y-cy)
    rx = dx*math.cos(rot_rad) - dy*math.sin(rot_rad)
    ry = dx*math.sin(rot_rad) + dy*math.cos(rot_rad)
    u = (iw/2.0) + rx*px_per_unit + ox
    v = (ih/2.0) - ry*px_per_unit + oy
    return int(round(u)), int(round(v))

class LogoMaskedSpiral(Node):
    def __init__(self, a):
        super().__init__('logo_masked_spiral')
        self.a = a
        self.c_clear = self.create_client(Empty, '/clear')
        self.c_pen   = self.create_client(SetPen, '/turtle1/set_pen')
        self.c_tp    = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        for c in (self.c_clear, self.c_pen, self.c_tp):
            while not c.wait_for_service(timeout_sec=1.0):
                pass
        img = Image.open(a.logo).convert('L')
        self.logo_img = img
        self.iw, self.ih = img.size

    def set_pen(self, r,g,b,w, off=False):
        q = SetPen.Request()
        q.r, q.g, q.b, q.width = int(r), int(g), int(b), int(w)
        q.off = 1 if off else 0
        self.c_pen.call_async(q)

    def tp_abs(self, x,y, th=0.0):
        q = TeleportAbsolute.Request()
        q.x, q.y, q.theta = float(x), float(y), float(th)
        self.c_tp.call_async(q)

    def clear(self):
        self.c_clear.call_async(Empty.Request())

    def in_logo(self, x,y):
        a = self.a
        u,v = world_to_img(x,y, a.logo_cx,a.logo_cy, a.px_per_unit,
                           math.radians(a.logo_rot), self.iw,self.ih, a.logo_ox,a.logo_oy)
        if 0<=u<self.iw and 0<=v<self.ih:
            val = self.logo_img.getpixel((u,v))
            return (val <= a.threshold) if a.logo_is_dark else (val >= a.threshold)
        return False

    def draw_spiral(self, color, width, mask_logo=False):
        a = self.a
        r,g,b = color
        self.set_pen(r,g,b,width, off=False)
        theta = 0.06
        last_inside = False
        last_xyth = (CENTER,CENTER,theta)
        while True:
            rad = a.spiral_k*theta
            x,y = polar_xy(rad, theta)
            if not (XMIN<=x<=XMAX and YMIN<=y<=YMAX):
                break
            if mask_logo:
                inside = self.in_logo(x,y)
                if inside and not last_inside:
                    self.set_pen(r,g,b,width, off=True)
                elif (not inside) and last_inside:
                    self.set_pen(r,g,b,width, off=False)
                last_inside = inside
            self.tp_abs(x,y,theta)
            last_xyth = (x,y,theta)
            time.sleep(a.sleep)
            theta += a.spiral_step
        self.set_pen(r,g,b,width, off=True)  # matikan pen supaya tidak ada garis kembali
        return last_xyth

    def run(self):
        a = self.a
        if a.clear:
            self.clear(); time.sleep(0.1)
        pr,pg,pb,pw = a.pen_y
        self.draw_spiral((pr,pg,pb), pw, mask_logo=False)   # spiral ungu penuh
        wr,wg,wb,ww = a.pen_spiral
        self.draw_spiral((wr,wg,wb), ww, mask_logo=True)    # spiral putih termask logo

def parse_rgba(s):
    r,g,b,w = [int(x) for x in s.split(',')]
    return (r,g,b,w)

def main():
    ap = argparse.ArgumentParser("Spiral masked by PNG logo; turtle stops at edge")
    ap.add_argument('--clear', action='store_true')
    ap.add_argument('--logo', type=str, default='/home/user/logo_academi_crypto.png')
    ap.add_argument('--px-per-unit', type=float, default=50.0)
    ap.add_argument('--logo-rot', type=float, default=15.0)
    ap.add_argument('--logo-cx', type=float, default=5.5)
    ap.add_argument('--logo-cy', type=float, default=5.5)
    ap.add_argument('--logo-ox', type=float, default=0.0)
    ap.add_argument('--logo-oy', type=float, default=0.0)
    ap.add_argument('--threshold', type=int, default=205)
    ap.add_argument('--logo-is-dark', action='store_true')
    ap.add_argument('--spiral-k', type=float, default=0.0475)
    ap.add_argument('--spiral-step', type=float, default=0.010)
    ap.add_argument('--sleep', type=float, default=0.0015)
    ap.add_argument('--pen-y', type=parse_rgba, default='170,100,255,6')
    ap.add_argument('--pen-spiral', type=parse_rgba, default='255,255,255,3')
    args = ap.parse_args()
    rclpy.init()
    node = LogoMaskedSpiral(args)
    try:
        node.run()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
