#!/usr/bin/env python3
"""Host renderer for the Display V1-Mock Final Screen.

This tool mirrors the firmware mock-screen content at host level so the UI can
be checked without a board. It writes a UART-style log block and a PNG preview.
It does not connect to firmware, BLE, model, audio, radar, or shared memory.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

from PIL import Image, ImageDraw, ImageFont


@dataclass(frozen=True)
class MockDisplaySnapshot:
    title: str = "E84 呼吸健康监测"
    run_status: str = "模拟展示中"
    runtime: str = "00:42"
    data_source: str = "Mock"
    health_state: str = "关注"
    health_note: str = "咳嗽略高，呼吸平稳"
    cough_count_1min: int = 1
    cough_count_5min: int = 2
    cough_probability_percent: int = 64
    presence: str = "人体在位"
    breath_rate_bpm: float = 17.8
    heart_rate_bpm: float = 72.0
    radar_quality_percent: int = 88
    night_duration: str = "6小时18分"
    night_stable_percent: int = 92
    night_reminders: int = 1
    mic_status: str = "正常"
    radar_status: str = "正常"
    ble_status: str = "未连接"
    battery_status: str = "充足(模拟)"
    disclaimer: str = "工程显示效果验证，非医疗诊断或治疗建议。"


REQUIRED_SECTIONS = (
    "标题:",
    "运行状态:",
    "当前健康状态:",
    "咳嗽监测:",
    "呼吸辅助:",
    "今夜摘要:",
    "设备状态:",
    "免责声明:",
)


def find_font(size: int) -> ImageFont.FreeTypeFont:
    candidates = (
        Path("C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/msyh.ttf"),
        Path("C:/Windows/Fonts/simhei.ttf"),
        Path("C:/Windows/Fonts/simsun.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc"),
    )
    for path in candidates:
        if path.exists():
            return ImageFont.truetype(str(path), size)
    raise RuntimeError("No Chinese-capable font found for PNG rendering")


def uart_lines(snapshot: MockDisplaySnapshot, t_ms: int = 1000) -> list[str]:
    return [
        (
            "[DISPLAY_FINAL_SCREEN] "
            f"t_ms={t_ms} "
            "path=APP_DISPLAY_ENABLE+APP_DISPLAY_FINAL_MOCK_ENABLE "
            "source=mock"
        ),
        "================================================",
        f"标题: {snapshot.title}",
        (
            "运行状态: "
            f"{snapshot.run_status} | 已运行 {snapshot.runtime} | "
            f"数据源 {snapshot.data_source}"
        ),
        (
            "当前健康状态: "
            f"{snapshot.health_state} | {snapshot.health_note}"
        ),
        (
            "咳嗽监测: "
            f"近1分钟 {snapshot.cough_count_1min}次 | "
            f"近5分钟 {snapshot.cough_count_5min}次 | "
            f"咳嗽概率 {snapshot.cough_probability_percent}%"
        ),
        (
            "呼吸辅助: "
            f"{snapshot.presence} | 呼吸 {snapshot.breath_rate_bpm:.3f} 次/分 | "
            f"心率 {snapshot.heart_rate_bpm:.3f} 次/分 | "
            f"雷达质量 {snapshot.radar_quality_percent}%"
        ),
        (
            "今夜摘要: "
            f"监测 {snapshot.night_duration} | "
            f"平稳 {snapshot.night_stable_percent}% | "
            f"提醒 {snapshot.night_reminders}次"
        ),
        (
            "设备状态: "
            f"MIC{snapshot.mic_status} | "
            f"雷达{snapshot.radar_status} | "
            f"BLE{snapshot.ble_status} | "
            f"电量{snapshot.battery_status}"
        ),
        f"免责声明: {snapshot.disclaimer}",
        "================================================",
    ]


def validate_log(lines: Iterable[str]) -> dict[str, object]:
    joined = "\n".join(lines)
    missing = [section for section in REQUIRED_SECTIONS if section not in joined]
    return {
        "required_sections": list(REQUIRED_SECTIONS),
        "missing_sections": missing,
        "pass": not missing,
    }


def draw_card(
    draw: ImageDraw.ImageDraw,
    xy: tuple[int, int, int, int],
    title: str,
    body: list[str],
    title_font: ImageFont.ImageFont,
    body_font: ImageFont.ImageFont,
    fill: str,
    outline: str,
) -> None:
    draw.rounded_rectangle(xy, radius=8, fill=fill, outline=outline, width=1)
    x0, y0, _, _ = xy
    draw.text((x0 + 12, y0 + 10), title, font=title_font, fill="#111827")
    y = y0 + 38
    for line in body:
        draw.text((x0 + 12, y), line, font=body_font, fill="#374151")
        y += 22


def render_png(snapshot: MockDisplaySnapshot, output_path: Path) -> None:
    width, height = 800, 480
    image = Image.new("RGB", (width, height), "#f6f7f9")
    draw = ImageDraw.Draw(image)

    title_font = find_font(30)
    subtitle_font = find_font(16)
    section_font = find_font(18)
    body_font = find_font(15)
    small_font = find_font(13)

    draw.rectangle((0, 0, width, 76), fill="#0f766e")
    draw.text((28, 16), snapshot.title, font=title_font, fill="white")
    draw.text(
        (540, 22),
        f"{snapshot.run_status} | {snapshot.runtime} | {snapshot.data_source}",
        font=subtitle_font,
        fill="#ccfbf1",
    )

    draw.rounded_rectangle(
        (28, 96, 772, 164), radius=8, fill="#ffffff", outline="#d1d5db"
    )
    draw.text((48, 112), "当前状态", font=section_font, fill="#111827")
    draw.text((156, 106), snapshot.health_state, font=find_font(34), fill="#b45309")
    draw.text((258, 119), snapshot.health_note, font=section_font, fill="#374151")

    draw_card(
        draw,
        (28, 184, 378, 286),
        "咳嗽监测",
        [
            f"近1分钟 {snapshot.cough_count_1min}次    近5分钟 {snapshot.cough_count_5min}次",
            f"咳嗽概率 {snapshot.cough_probability_percent}%",
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )
    draw_card(
        draw,
        (400, 184, 772, 286),
        "呼吸辅助",
        [
            f"{snapshot.presence}    雷达质量 {snapshot.radar_quality_percent}%",
            f"呼吸 {snapshot.breath_rate_bpm:.1f} 次/分    心率 {snapshot.heart_rate_bpm:.0f} 次/分",
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )
    draw_card(
        draw,
        (28, 306, 378, 404),
        "今夜摘要",
        [
            f"监测 {snapshot.night_duration}    平稳 {snapshot.night_stable_percent}%",
            f"提醒 {snapshot.night_reminders}次",
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )
    draw_card(
        draw,
        (400, 306, 772, 404),
        "设备状态",
        [
            f"MIC{snapshot.mic_status}    雷达{snapshot.radar_status}",
            f"BLE{snapshot.ble_status}    电量{snapshot.battery_status}",
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )

    draw.rectangle((0, 438, width, height), fill="#e5e7eb")
    draw.text((28, 450), f"免责声明: {snapshot.disclaimer}", font=small_font, fill="#4b5563")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(".codex_tmp/display_v1_mock_final_screen"),
        help="Directory for generated host validation artifacts.",
    )
    args = parser.parse_args()

    snapshot = MockDisplaySnapshot()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    log_path = args.out_dir / "display_v1_mock_final_screen_host.log"
    png_path = args.out_dir / "display_v1_mock_final_screen_host.png"
    report_path = args.out_dir / "display_v1_mock_final_screen_host_report.json"

    lines = uart_lines(snapshot)
    log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    render_png(snapshot, png_path)

    report = {
        "mode": "host_mock_no_board",
        "snapshot": asdict(snapshot),
        "log_path": str(log_path),
        "png_path": str(png_path),
        "validation": validate_log(lines),
        "boundaries": {
            "flashing": False,
            "real_summary_connected": False,
            "ble_abi_changed": False,
            "model_audio_radar_shared_memory_abi_changed": False,
        },
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"log={log_path}")
    print(f"png={png_path}")
    print(f"report={report_path}")
    print(f"validation_pass={report['validation']['pass']}")
    return 0 if report["validation"]["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
