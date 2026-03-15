"""
Uso: 
    python detect.py                         # webcam
    python detect.py image.jpg               # single image
    python detect.py path/to/images/         # folder of images
    python detect.py video.mp4               # video file


Requerimientos:
    pip install ultralytics
    usar best.onnx en caso de no tener GPU 
"""

import sys
import cv2
from pathlib import Path
from ultralytics import YOLO

MODEL      = Path(__file__).parent / "runs/hazmat_yolo11l/weights/best.pt"
CONF       = 0.5    # Umbral de confianza para filtrar detecciones (0.0 - 1.0)
IOU        = 0.45   # NMS IoU threshold (0.0 - 1.0) para eliminar detecciones superpuestas
IMGSZ      = 640    # Tamaño de imagen para la inferencia (p. ej., 640, 1280
DEVICE     = "cpu"  # "cpu" o "cuda:0" para GPU
LINE_WIDTH = 8   # grosor de línea para cuadros delimitadores
FONT_SIZE  = 24     # tamaño de fuente para etiquetas y texto
OUTPUT_DIR = Path("runs/detect/results")


source = sys.argv[1] if len(sys.argv) > 1 else 0   # 0 para webcam, o ruta a imagen/video/carpeta

model = YOLO(MODEL, task="detect")

OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

results = model.predict(
    source=source,
    conf=CONF,
    iou=IOU,
    imgsz=IMGSZ,
    device=DEVICE,
    stream=True,     # devuelve un generador para procesar resultados en tiempo real
    line_width=LINE_WIDTH,
)

for r in results:
    # Dibujar cuadros delimitadores y etiquetas en la imagen
    annotated = r.plot(line_width=LINE_WIDTH, font_size=FONT_SIZE)

    # Mostrar imagen anotada en ventana
    cv2.imshow("HazMat Detection", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    # Guardar imagen anotada en carpeta de resultados
    name = Path(str(r.path)).name or "frame.jpg"
    out_path = OUTPUT_DIR / name
    cv2.imwrite(str(out_path), annotated)

    # Imprimir resultados de detección en consola
    if len(r.boxes) == 0:
        print(f"{name}  —  no detections")
        continue

    print(f"\n{name}")
    for box in r.boxes:
        cls  = int(box.cls)
        conf = float(box.conf)
        xyxy = [round(x, 1) for x in box.xyxy[0].tolist()]
        print(f"  {r.names[cls]:<30s}  conf={conf:.2f}  box={xyxy}")

cv2.destroyAllWindows()
print(f"\nResults saved to: {OUTPUT_DIR}")
