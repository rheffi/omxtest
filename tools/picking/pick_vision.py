"""Optional OpenCV/Ultralytics adapters; original-frame pixel coordinates only."""

from __future__ import annotations

import time

from pick_core import Detection, choose, get_rule, integer, project_path


class Detector:
    def __init__(self, cfg, weights=None):
        self.cfg = cfg
        path = project_path(weights or cfg.get("weights"))
        if not path.is_file() or path.suffix.lower() != ".pt":
            raise FileNotFoundError(f"로컬 Ultralytics 검출 모델 .pt가 필요합니다: {path}")
        from ultralytics import YOLO
        self.model = YOLO(str(path.resolve()), task="detect")
        if self.model.task != "detect":
            raise ValueError("일반 detect 모델이 필요합니다 (classify/pose/OBB 모델 제외)")
        self.imgsz = integer(cfg.get("imgsz"), "imgsz", 32, 2048)

    def predict(self, frame):
        height, width = frame.shape[:2]
        result = self.model.predict(source=frame, conf=self.cfg["confidence"],
                                    imgsz=self.imgsz, device=self.cfg.get("device", "cpu"),
                                    verbose=False, save=False)[0]
        if tuple(result.orig_shape) != (height, width):
            raise ValueError("YOLO 결과 원본 해상도가 입력과 다릅니다")
        if result.boxes is None:
            return []
        detections = []
        for box, score, cls in zip(result.boxes.xyxy.cpu().tolist(),
                                    result.boxes.conf.cpu().tolist(), result.boxes.cls.cpu().tolist()):
            class_id = int(cls)
            det = Detection(result.names[class_id], float(score), tuple(box), class_id)
            det.validate(width, height)
            detections.append(det)
        return detections


class Camera:
    def __init__(self, profile):
        self.profile = profile
        self.capture = None

    def __enter__(self):
        import cv2
        if not project_path(self.profile.camera).exists():
            raise FileNotFoundError(f"카메라가 없습니다: {self.profile.camera}")
        self.capture = cv2.VideoCapture(self.profile.camera, cv2.CAP_V4L2)
        try:
            if not self.capture.isOpened():
                raise RuntimeError("카메라 열기 실패: 다른 미리보기 프로그램을 종료하세요")
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.profile.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.profile.height)
            self.capture.set(cv2.CAP_PROP_FPS, self.profile.fps)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            for _ in range(5):
                self.read()
        except BaseException:
            self.__exit__(None, None, None)
            raise
        return self

    def __exit__(self, *args):
        if self.capture is not None:
            self.capture.release()

    def read(self):
        # Drain buffered frames after inference/operator waits before using one.
        for _ in range(3):
            if not self.capture.grab():
                raise RuntimeError("카메라 프레임 갱신 실패")
        ok, frame = self.capture.read()
        captured_at = time.monotonic()
        if not ok or frame is None:
            raise RuntimeError("카메라 프레임 읽기 실패")
        if frame.shape[:2] != (self.profile.height, self.profile.width):
            raise ValueError("실제 카메라 해상도가 팔 프로필/보정 해상도와 다릅니다")
        return frame, captured_at


def capture_target(camera, detector, cfg, label, index=0):
    frame, captured_at = camera.read()
    detections = detector.predict(frame)
    target = choose(detections, cfg, label, frame.shape[1], frame.shape[0], index)
    return target, frame, captured_at


def show(frame, detections, target=None, cfg=None):
    import cv2
    image = frame.copy()
    for det in detections:
        x1, y1, x2, y2 = map(round, det.box)
        color = (0, 255, 0) if det == target else (255, 180, 0)
        cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
        cv2.putText(image, f"{det.label} {det.confidence:.2f}", (x1, max(18, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    if target and cfg:
        point = tuple(map(round, target.point(get_rule(cfg, target.label))))
        cv2.drawMarker(image, point, (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
    try:
        cv2.imshow("OMX detection - any key to close", image)
        cv2.waitKey(0)
    finally:
        cv2.destroyAllWindows()
