yolov8n-pose.onnx
=================

用途：本项目的人体姿态推理模型（YOLOv8n-pose，ONNX 格式，COCO-17 关键点）
来源：https://huggingface.co/Xenova/yolov8n-pose  ->  onnx/model.onnx
大小：约 12.9 MB
输入：1x3x640x640（BGR，归一化）
输出：1x56x8400，其中
        行 0-3  : 边界框 (cx, cy, w, h)
        行 4    : 人体置信度
        行 5-55 : 17 个关键点 × (x, y, conf)

许可：AGPL-3.0（Ultralytics YOLOv8）
      详见 https://github.com/ultralytics/ultralytics/blob/main/LICENSE
      商用请自行确认授权；若不需要该模型，删除本文件后程序仍可运行，
      只是骨骼检测不可用（界面会给出提示，不会崩溃）。
