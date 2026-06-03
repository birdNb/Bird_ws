#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将手势分类 PyTorch 模型导出为 ONNX，供本工程 vision_controller 加载。

用法:
  cd ~/Bird_ws/hand_identify_cpp
  pip install -r requirements-export.txt
  python3 scripts/export_onnx.py
  # 或: ./export_model.sh
"""

from __future__ import annotations

import os
import sys

import numpy as np

# 工程根目录 hand_identify_cpp/
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# ====================== 配置（与 C++ GestureDetector 一致）======================
INPUT_SIZE = (224, 224)
NUM_CLASSES = 6
ONNX_SAVE_PATH = os.path.join(PROJECT_ROOT, "model", "gesture.onnx")
WEIGHTS_PATH = os.path.join(PROJECT_ROOT, "weights", "gesture_classifier.pth")
OPSET_VERSION = 14
INPUT_NAME = "input"
OUTPUT_NAME = "output"


def _require_torch():
    try:
        import torch  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "缺少 PyTorch。请执行: pip install -r requirements-export.txt"
        ) from exc


def build_gesture_classifier():
    """与 export / C++ 对齐的 6 类手势 CNN。"""
    import torch
    import torch.nn as nn
    import torch.nn.functional as F

    class GestureClassifier(nn.Module):
        def __init__(self, num_classes: int = NUM_CLASSES):
            super().__init__()
            self.features = nn.Sequential(
                nn.Conv2d(3, 32, 3, padding=1),
                nn.BatchNorm2d(32),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(32, 64, 3, padding=1),
                nn.BatchNorm2d(64),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(64, 128, 3, padding=1),
                nn.BatchNorm2d(128),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
                nn.Conv2d(128, 256, 3, padding=1),
                nn.BatchNorm2d(256),
                nn.ReLU(inplace=True),
                nn.MaxPool2d(2),
            )
            self.head = nn.Sequential(
                nn.AdaptiveAvgPool2d(1),
                nn.Flatten(),
                nn.Linear(256, 128),
                nn.ReLU(inplace=True),
                nn.Linear(128, num_classes),
            )

        def forward(self, x):
            x = self.features(x)
            logits = self.head(x)
            return F.softmax(logits, dim=1)

    return GestureClassifier()


def load_your_model():
    """
    加载手势分类模型。可替换为你自己的训练代码与权重路径。
    """
    import torch

    model = build_gesture_classifier()
    if os.path.isfile(WEIGHTS_PATH):
        state = torch.load(WEIGHTS_PATH, map_location="cpu")
        if isinstance(state, dict) and "state_dict" in state:
            state = state["state_dict"]
        model.load_state_dict(state, strict=False)
        print(f"已加载权重: {WEIGHTS_PATH}")
    else:
        print(
            f"未找到 {WEIGHTS_PATH}\n"
            "  → 将导出随机初始化模型（仅供 C++ 联调）。\n"
            "  → 上线前请训练后保存到 weights/gesture_classifier.pth"
        )
    model.eval()
    return model


def export_onnx(model) -> None:
    import torch

    os.makedirs(os.path.dirname(ONNX_SAVE_PATH), exist_ok=True)
    dummy_input = torch.randn(1, 3, INPUT_SIZE[0], INPUT_SIZE[1])
    torch.onnx.export(
        model,
        dummy_input,
        ONNX_SAVE_PATH,
        export_params=True,
        opset_version=OPSET_VERSION,
        do_constant_folding=True,
        input_names=[INPUT_NAME],
        output_names=[OUTPUT_NAME],
        dynamic_axes={
            INPUT_NAME: {0: "batch_size"},
            OUTPUT_NAME: {0: "batch_size"},
        },
    )
    print(f"ONNX 模型已写入: {ONNX_SAVE_PATH}")


def verify_onnx(model=None) -> None:
    import onnx
    import onnxruntime as ort
    import torch

    onnx_model = onnx.load(ONNX_SAVE_PATH)
    onnx.checker.check_model(onnx_model)
    print("ONNX 结构检查通过")

    if model is None:
        model = load_your_model()
    torch.manual_seed(42)
    dummy_input = torch.randn(1, 3, INPUT_SIZE[0], INPUT_SIZE[1])
    with torch.no_grad():
        torch_output = model(dummy_input).numpy()

    sess = ort.InferenceSession(ONNX_SAVE_PATH, providers=["CPUExecutionProvider"])
    onnx_output = sess.run(None, {INPUT_NAME: dummy_input.numpy()})[0]

    diff = float(np.max(np.abs(torch_output - onnx_output)))
    print(f"PyTorch vs ONNX 最大差异: {diff:.6f}")
    if diff < 1e-4:
        print("导出数值一致，可用于 C++ 推理")
    else:
        print("输出差异偏大，请检查 opset / 算子或关闭 dynamic_axes 后重试")


def main() -> int:
    _require_torch()
    print(f"工程目录: {PROJECT_ROOT}")
    print("开始导出手势 ONNX (6 类, 224x224)...")
    model = load_your_model()
    export_onnx(model)
    verify_onnx(model)
    print("\n全部完成。请执行: ./build.sh && ./start.sh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
