import os
import cv2
import numpy as np

# Class ids: 0 orange, 1 coke_can, 2 milk_carton
CLASSES = {"orange": 0, "coke_can": 1, "milk_carton": 2}

def bbox_from_mask(mask, min_area=200):
    # trova contorni e restituisce bbox della componente più grande
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < min_area:
        return None
    x, y, w, h = cv2.boundingRect(c)
    return x, y, x+w, y+h

def yolo_line(class_id, xmin, ymin, xmax, ymax, W, H):
    xc = ((xmin + xmax) / 2.0) / W
    yc = ((ymin + ymax) / 2.0) / H
    w  = (xmax - xmin) / float(W)
    h  = (ymax - ymin) / float(H)
    return f"{class_id} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}"

def main(img_dir, label_dir, debug_dir=None):
    os.makedirs(label_dir, exist_ok=True)
    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)

    for fn in sorted(os.listdir(img_dir)):
        if not fn.lower().endswith((".jpg",".png",".jpeg")):
            continue
        path = os.path.join(img_dir, fn)
        img = cv2.imread(path)
        if img is None:
            continue
        H, W = img.shape[:2]

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        kernel = np.ones((5, 5), np.uint8)

        # --- ORANGE
        orange_mask = cv2.inRange(hsv, (5, 60, 60), (35, 255, 255))
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        # --- RED (wrap-around)
        red1 = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255))
        red2 = cv2.inRange(hsv, (160, 80, 80), (179, 255, 255))
        red_mask = cv2.bitwise_or(red1, red2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        # --- CARTON (chiaro/grigio: S bassa, V alta)
        # --- BLUE carton
        carton_mask = cv2.inRange(hsv, (95, 80, 80), (130, 255, 255))

        carton_mask = cv2.morphologyEx(carton_mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        carton_mask = cv2.morphologyEx(carton_mask, cv2.MORPH_OPEN,  kernel, iterations=1)

        # ROI: limita al tavolo (taglia fuori robot e bordi)
        roi = np.zeros((H, W), dtype=np.uint8)
        roi[int(0.22*H):int(0.88*H), int(0.08*W):int(0.92*W)] = 255
        carton_mask = cv2.bitwise_and(carton_mask, roi)

        # escludi arancia e rosso dal cartone
        carton_mask = cv2.bitwise_and(carton_mask, cv2.bitwise_not(orange_mask))
        carton_mask = cv2.bitwise_and(carton_mask, cv2.bitwise_not(red_mask))


        lines = []
        dbg = img.copy()

        # bbox per ciascuna classe (prendi oggetto più grande del colore)
        bb = bbox_from_mask(orange_mask, min_area=150)
        if bb:
            xmin,ymin,xmax,ymax = bb
            lines.append(yolo_line(CLASSES["orange"], xmin,ymin,xmax,ymax, W,H))
            cv2.rectangle(dbg, (xmin,ymin), (xmax,ymax), (0,255,0), 2)

        bb = bbox_from_mask(red_mask, min_area=200)
        if bb:
            xmin,ymin,xmax,ymax = bb
            lines.append(yolo_line(CLASSES["coke_can"], xmin,ymin,xmax,ymax, W,H))
            cv2.rectangle(dbg, (xmin,ymin), (xmax,ymax), (0,255,0), 2)

        bb = bbox_from_mask(carton_mask, min_area=300)
        if bb:
            xmin,ymin,xmax,ymax = bb
            lines.append(yolo_line(CLASSES["milk_carton"], xmin,ymin,xmax,ymax, W,H))
            cv2.rectangle(dbg, (xmin,ymin), (xmax,ymax), (0,255,0), 2)

        # scrivi label YOLO
        stem = os.path.splitext(fn)[0]
        out_txt = os.path.join(label_dir, stem + ".txt")
        with open(out_txt, "w") as f:
            f.write("\n".join(lines) + "\n")

        if debug_dir:
            cv2.imwrite(os.path.join(debug_dir, fn), dbg)

if __name__ == "__main__":
    # esempio:
    # python3 label_from_hsv.py /home/turro/yolo_dataset/images/train /home/turro/yolo_dataset/labels/train /home/turro/yolo_dataset/debug/train
    import sys
    img_dir = sys.argv[1]
    label_dir = sys.argv[2]
    debug_dir = sys.argv[3] if len(sys.argv) > 3 else None
    main(img_dir, label_dir, debug_dir)
