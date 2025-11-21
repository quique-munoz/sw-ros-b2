import os
os.environ["SDL_VIDEO_MINIMIZE_ON_FOCUS_LOSS"] = "0"

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from zed_msgs.msg import ObjectsStamped
import numpy as np
import pygame
import screeninfo

# --- YOLO / Torch ---
import torch
from ultralytics import YOLO

# ========= CONFIG =========
RUTA_MODELO = "dihbu_26_09_2025.pt"
CONF_MIN = 0.6
IOU = 0.5
IMG_SIZE = 960                   # múltiplo de 32
USE_GPU = torch.cuda.is_available()
USE_HALF = USE_GPU               # FP16 si hay CUDA

#Umbral por clase
CLASS_THRESH= {
    "fisura": 0.35,
    "_default_": CONF_MIN,
}	

# Banner/fondo/visualización
RUTA_BANNER = "./Banner_Dihbu2.png"
RUTA_FONDO  = "./feriaLerma_back.png"
TARGET_FPS = 30
FACTOR_REDUCCION = 0.9

# Colores YOLO
COLOR_MAP_BGR = {
    "mancha": (0, 165, 255),
    "fisura": (0, 255, 255),
    "Panel Solar": (0, 255, 0),
    "PLC": (255, 0, 0),
}
DEFAULT_COLOR = (0, 255, 0)

torch.backends.cudnn.benchmark = True

# Conexiones BODY_18
CONNECTIONS = [
    (1, 2), (2, 3), (3, 4),        # brazo derecho
    (1, 5), (5, 6), (6, 7),        # brazo izquierdo
    (1, 8), (8, 9), (9, 10),       # pierna derecha
    (1, 11), (11, 12), (12, 13),   # pierna izquierda
    (1, 0),                        # cuello - nariz
    (0, 14), (14, 16),             # ojo/oreja derecha
    (0, 15), (15, 17)              # ojo/oreja izquierda
]

# =================== NODO ROS2 ===================
class SkeletonDrawerNode(Node):
    def __init__(self):
        super().__init__('skeleton_drawer')
        self.subscription_skeleton = self.create_subscription(
            ObjectsStamped, '/zed/zed_node/body_trk/skeletons', self.skeleton_callback, 10
        )
        self.subscription_image = self.create_subscription(
            Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10
        )
        self.cv_bridge = CvBridge()
        self.skeleton_data = None
        self.image_data = None
        self.objects = []

        # --- Cargar YOLO una sola vez ---
        self.get_logger().info(f"Cargando YOLO desde: {RUTA_MODELO}")
        self.modelo_yolo = YOLO(RUTA_MODELO)
        if USE_GPU:
            self.get_logger().info("Usando GPU (CUDA)")
            self.modelo_yolo.to("cuda")
        else:
            self.get_logger().warn("CUDA no disponible: usando CPU")

        # Warm-up
        try:
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = self.modelo_yolo.predict(
                dummy,
                device=0 if USE_GPU else "cpu",
                imgsz=IMG_SIZE,
                half=USE_HALF,
                conf=CONF_MIN,
                iou=IOU,
                verbose=False
            )
            if USE_GPU:
                torch.cuda.synchronize()
        except Exception as e:
            self.get_logger().warn(f"Warm-up YOLO omitido: {e}")

    def skeleton_callback(self, msg: ObjectsStamped):
        self.objects = msg.objects
        valid = filter_objects(msg.objects)

        skeleton_data = []
        for obj in valid:
            kps = obj.skeleton_2d.keypoints
            pts = [kp.kp for kp in kps[:18]]   # BODY_18
            skeleton_data.append(pts)
        self.skeleton_data = skeleton_data

    def image_callback(self, msg: Image):
        frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        width  = int(frame.shape[1] * 2)
        height = int(frame.shape[0] * 2)
        frame  = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
        self.image_data = frame

    def get_skeleton_data(self): return self.skeleton_data
    def get_image_data(self): return self.image_data
    def set_skeleton_data(self): self.skeleton_data = None
    def set_image_data(self): self.image_data = None


# =================== DIBUJO ===================
def draw_skeleton_on_image(image, keypoints):
    if keypoints is None: return
    # puntos
    for person in keypoints:
        for p in person:
            x, y = int(p[0]), int(p[1])
            if x >= 0 and y >= 0:
                cv2.circle(image, (x, y), 5, (0, 255, 0), -1)
    # conexiones
    for a, b in CONNECTIONS:
        for person in keypoints:
            if a < len(person) and b < len(person):
                x1, y1 = int(person[a][0]), int(person[a][1])
                x2, y2 = int(person[b][0]), int(person[b][1])
                if x1 >= 0 and y1 >= 0 and x2 >= 0 and y2 >= 0:
                    cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 2)

def draw_yolo_detections(image_bgr: np.ndarray, results, names, color_map=COLOR_MAP_BGR, class_threshold=None):
    def get_thr(cls_name: str) -> float:
        if class_threshold is None:
            return CONF_MIN
        return class_threshold.get(cls_name, class_threshold.get("_default_", CONF_MIN))

    for box, conf, cls in zip(results.boxes.xyxy, results.boxes.conf, results.boxes.cls):
        x1, y1, x2, y2 = map(int, box.tolist())
        cls_idx = int(cls)
        cls_name = names[cls_idx] if isinstance(names, dict) else str(cls_idx)

        if float(conf) < float(get_thr(cls_name)):
            continue

        color = color_map.get(cls_name, DEFAULT_COLOR)
        cv2.rectangle(image_bgr, (x1, y1), (x2, y2), color, 2)
        label = f"{cls_name}"#:{float(conf):.2f}"
        (tw, th), base = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        y_text = max(0, y1 - (th + base) - 2)
        cv2.rectangle(image_bgr, (x1, y_text), (x1 + tw + 4, y_text + th + base + 2), color, -1)
        cv2.putText(image_bgr, label, (x1 + 2, y_text + th + 1),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)


def kp2xy_keypoint2di(p):
    """Keypoint2Di -> uint32[2]."""
    return float(p.kp[0]), float(p.kp[1])

def scale_and_clip_xy(xs, ys, sx, sy, W, H):
    xs = np.asarray(xs, dtype=np.float32) * float(sx)
    ys = np.asarray(ys, dtype=np.float32) * float(sy)
    xs = np.clip(xs, 0, W - 1).astype(np.int32)
    ys = np.clip(ys, 0, H - 1).astype(np.int32)
    return xs, ys

def bbox_from_corners(corners, img_shape, sx=1.0, sy=1.0, margin=0):
    """
    corners: zed_interfaces/Keypoint2Di[4] en orden 0-1-2-3
    img_shape: image.shape para recortar (H, W, C)
    sx, sy: factores de escala (p.ej. 2.0 si redimensionas ×2 en image_callback)
    margin: píxeles de margen extra
    -> (x1, y1, x2, y2) o None si no válida
    """
    H, W = img_shape[:2]
    xs, ys = [], []
    for c in corners:
        x, y = kp2xy_keypoint2di(c)
        xs.append(x); ys.append(y)

    xs, ys = scale_and_clip_xy(xs, ys, sx, sy, W, H)
    x1 = max(0, xs.min() - margin)
    y1 = max(0, ys.min() - margin)
    x2 = min(W - 1, xs.max() + margin)
    y2 = min(H - 1, ys.max() + margin)
    if x2 <= x1 or y2 <= y1:
        return None
    return int(x1), int(y1), int(x2), int(y2)

def draw_bbox2d_rect(img, corners, color=(0,0,255), thickness=2, sx=1.0, sy=1.0, margin=0):
    bb = bbox_from_corners(corners, img.shape, sx=sx, sy=sy, margin=margin)
    if bb is None:
        return
    x1, y1, x2, y2 = bb
    cv2.rectangle(img, (x1, y1), (x2, y2), color, thickness)

def draw_bbox2d_polygon(img, corners, color=(0,0,255), thickness=2, sx=1.0, sy=1.0):
    """Dibuja el polígono exacto 0-1-2-3 (por si no fuera perfectamente axis-aligned)."""
    H, W = img.shape[:2]
    xs, ys = [], []
    for c in corners:
        x, y = kp2xy_keypoint2di(c)
        xs.append(x); ys.append(y)
    xs, ys = scale_and_clip_xy(xs, ys, sx, sy, W, H)
    pts = np.stack([xs, ys], axis=1).astype(np.int32)
    # cierra el polígono 3->0
    cv2.polylines(img, [pts], isClosed=True, color=color, thickness=thickness)

# --- Filtro de objetos ZED ---
CONF_THR = 50.0         # 
STATE_OK = 1            # OK

def filter_objects(objs, conf_thr=CONF_THR, required_state=STATE_OK):
    return [
        o for o in (objs or [])
        if float(getattr(o, "confidence", 0.0)) > conf_thr
        and int(getattr(o, "tracking_state", -1)) == required_state
    ]


# ====== TRACKING AND CONFINDENCE STATE ======
# 0 OFF, 1 OK, 2 SEARCHING, 3 TERMINATE
STATE_STYLE = {
    0: ("OFF",       (160, 160, 160)),  # gris
    1: ("OK",        (50, 200, 50)),    # verde
    2: ("SEARCHING", (0, 165, 255)),    # naranja
    3: ("TERMINATE", (0, 0, 255)),      # rojo
}

def _anchor_from_obj(obj, img, sx=1.0, sy=1.0):
    """Devuelve un (x,y) para anclar texto: esquina sup-izq de la bbox o centroide de keypoints."""
    H, W = img.shape[:2]
    # 1) bbox si existe
    try:
        bb = bbox_from_corners(obj.bounding_box_2d.corners, img.shape, sx=sx, sy=sy, margin=0)
        if bb is not None:
            x1, y1, _, _ = bb
            return (int(np.clip(x1, 0, W-1)), int(max(0, y1 - 8)))
    except Exception:
        pass
    # 2) centroide de keypoints si no hay bbox
    xs, ys = [], []
    for kp in getattr(obj.skeleton_2d, "keypoints", [])[:18]:
        x, y = kp2xy_keypoint2di(kp)
        x, y = x * sx, y * sy
        if 0 <= x < W and 0 <= y < H:
            xs.append(x); ys.append(y)
    if xs and ys:
        cx, cy = int(np.mean(xs)), int(np.mean(ys))
        return (cx, max(0, cy - 10))
    return None

def _draw_tag(img, x, y, text, bg_color, fg_color=(0,0,0)):
    """Dibuja una cajita con texto en (x,y). Devuelve nueva y (para apilar líneas)."""
    (tw, th), base = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
    bx1, by1 = x, y
    bx2, by2 = x + tw + 8, y + th + base + 6
    bx2 = min(bx2, img.shape[1]-1)
    by2 = min(by2, img.shape[0]-1)
    cv2.rectangle(img, (bx1, by1), (bx2, by2), bg_color, -1)
    cv2.putText(img, text, (bx1 + 4, by1 + th + 1),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, fg_color, 2, cv2.LINE_AA)
    return by2 + 2  # siguiente alto para apilar

def draw_tracking_and_confidence(img, objects, sx=1.0, sy=1.0, show_conf=True, show_state=True):
    """Pinta para cada objeto su tracking_state y/o confidence, apilados en cajitas."""
    if not objects:
        return
    for obj in objects:
        anchor = _anchor_from_obj(obj, img, sx=sx, sy=sy)
        if anchor is None:
            continue
        x, y = anchor
        # Estado de tracking
        if show_state:
            st_raw = int(getattr(obj, "tracking_state", -1))
            label, color = STATE_STYLE.get(st_raw, (f"STATE:{st_raw}", (120,120,120)))
            y = _draw_tag(img, x, y, label, color)
        # Confianza (1..99)
        if show_conf:
            conf = float(getattr(obj, "confidence", float("nan")))
            if np.isfinite(conf):
                y = _draw_tag(img, x, y, f"CONF:{conf:.0f}", (50,200,50))

# ============== CARGA BANNER / FONDO / PANTALLA (igual que tu script) ==============
def load_banner_and_bg():
    banner = cv2.imread(RUTA_BANNER, cv2.IMREAD_UNCHANGED)
    if banner is None:
        raise FileNotFoundError(RUTA_BANNER)
    if banner.ndim == 3 and banner.shape[2] == 4:
        banner = cv2.cvtColor(banner, cv2.COLOR_BGRA2BGR)

    fondo = cv2.imread(RUTA_FONDO)
    if fondo is None:
        raise FileNotFoundError(RUTA_FONDO)

    monitores = screeninfo.get_monitors()
    pantalla = monitores[1] if len(monitores) > 1 else monitores[0]
    W, H = pantalla.width, pantalla.height

    banner = cv2.resize(banner, (W, int(banner.shape[0] * W / banner.shape[1])), interpolation=cv2.INTER_AREA)
    fondo  = cv2.resize(fondo,  (W, H),                                  interpolation=cv2.INTER_AREA)

    return banner, fondo, W, H

def compose_like_yours(out: np.ndarray, banner: np.ndarray, fondo: np.ndarray):
    """Replica tu pipeline de encaje: ratio con FACTOR_REDUCCION, y_offset=0, x centrado, vconcat."""
    if out.dtype != fondo.dtype:
        out = out.astype(fondo.dtype)

    max_ancho, max_alto = fondo.shape[1], fondo.shape[0]
    hO, wO = out.shape[:2]
    ratio = min((max_ancho * FACTOR_REDUCCION) / wO, (max_alto * FACTOR_REDUCCION) / hO)
    nuevo = (int(wO * ratio), int(hO * ratio))
    out_redimensionada = cv2.resize(out, nuevo, interpolation=cv2.INTER_AREA)

    y_offset = 0
    x_offset = (max_ancho - nuevo[0]) // 2
    imagen_mostrar = fondo.copy()
    imagen_mostrar[y_offset:y_offset+nuevo[1], x_offset:x_offset+nuevo[0]] = out_redimensionada

    # Ajuste de ancho del banner por si acaso
    if banner.shape[1] != imagen_mostrar.shape[1]:
        banner = cv2.resize(banner, (imagen_mostrar.shape[1], banner.shape[0]), interpolation=cv2.INTER_AREA)

    imagen_final = cv2.vconcat([banner, imagen_mostrar])
    return imagen_final


# =================== MAIN ===================
def main(args=None):
    # Pygame + assets como en tu código
    banner, fondo, Wscr, Hscr = load_banner_and_bg()
    pygame.init()
    #ventana = pygame.display.set_mode((Wscr, Hscr), pygame.NOFRAME | pygame.NOFRAME)
    display = 1
    size = pygame.display.get_desktop_sizes()[display]
    print((Wscr, Hscr))
    print(size)
    ventana = pygame.display.set_mode(size, pygame.NOFRAME, display=display)
    pygame.display.set_caption("ZED Viewer: Skeleton + YOLO + Banner")
    reloj = pygame.time.Clock()

    # ROS2
    rclpy.init(args=args)
    node = SkeletonDrawerNode()

    try:
        while rclpy.ok():
            image_data = node.get_image_data()
            skeleton_data = node.get_skeleton_data()

            if image_data is not None:
                # 1) Dibuja esqueleto 
                out = image_data.copy()
                SCALE_XY = 1.0

                # esqueleto
                if skeleton_data is not None:
                    draw_skeleton_on_image(out, skeleton_data)
                    # draw_tracking_and_confidence(out, node.objects, sx=SCALE_XY, sy=SCALE_XY,
                    #          show_conf=True, show_state=True)
                    # bbox del ZED (elige rect o polígono, o ambos)
                    #for obj in node.objects:
                    #    draw_bbox2d_rect(
                    #        out,
                    #        obj.bounding_box_2d.corners,
                    #        color=(0, 0, 255),
                    #        thickness=2,
                    #        sx=SCALE_XY, sy=SCALE_XY,
                    #        margin=0
                    #    )
                    #    # o si prefieres el contorno exacto:
                    #    # draw_bbox2d_polygon(out, obj.bounding_box_2d.corners, color=(0,0,255), thickness=2, sx=SCALE_XY, sy=SCALE_XY)

                    node.set_skeleton_data()

                # 2) YOLO
                PRED_CONF = min(CLASS_THRESH.values()) if CLASS_THRESH else CONF_MIN

                results = node.modelo_yolo.predict(
                    out,
                    device=0 if USE_GPU else "cpu",
                    conf=PRED_CONF,
                    iou=IOU,
                    imgsz=IMG_SIZE,
                    half=USE_HALF,
                    verbose=False
                )[0]
                draw_yolo_detections(out, results, node.modelo_yolo.names, class_threshold=CLASS_THRESH)

                # 3) Composición EXACTA como tu script “encaja bien”
                imagen_final = compose_like_yours(out, banner, fondo)

                # 4) Mostrar con pygame (igual que tú)
                imagen_final_rgb = cv2.cvtColor(imagen_final, cv2.COLOR_BGR2RGB)
                imagen_final_rgb = np.ascontiguousarray(imagen_final_rgb)
                surface = pygame.surfarray.make_surface(np.transpose(imagen_final_rgb, (1, 0, 2)))
                ventana.blit(surface, (0, 0))
                pygame.display.update()

                node.set_image_data()

            # eventos pygame
            for e in pygame.event.get():
                if e.type == pygame.QUIT or (e.type == pygame.KEYDOWN and e.key == pygame.K_ESCAPE):
                    raise KeyboardInterrupt

            rclpy.spin_once(node, timeout_sec=0.001)
            reloj.tick(TARGET_FPS)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        pygame.quit()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
