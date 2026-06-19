#!/usr/bin/env python3
"""Host renderer for Display final screen productization.

Generates competition-demo-quality UART log + PNG preview for all 7 scenarios.
Does not connect to firmware, BLE, model, audio, radar, or shared memory.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

from PIL import Image, ImageDraw, ImageFont


# --- Data models ---


@dataclass(frozen=True)
class ProductizedSnapshot:
    """One productized display scenario."""
    scenario: str
    main_status: str = "监测中"
    source_label: str = "真实雷达联调"
    presence: str = "有人"
    breath_rate: str = "16.8 bpm"
    heart_rate: str = "71.0 bpm"
    distance: str = "1.20 m"
    quality: str = "良好"


# --- Scenario definitions ---

SCENARIOS: list[ProductizedSnapshot] = [
    ProductizedSnapshot(
        scenario="real_radar_valid",
        main_status="监测中",
        source_label="真实雷达联调",
        presence="有人",
        breath_rate="16.8 bpm",
        heart_rate="71.0 bpm",
        distance="1.20 m",
        quality="良好",
    ),
    ProductizedSnapshot(
        scenario="attention",
        main_status="监测中",
        source_label="真实雷达联调",
        presence="有人",
        breath_rate="17.8 bpm",
        heart_rate="72.0 bpm",
        distance="1.15 m",
        quality="良好",
    ),
    ProductizedSnapshot(
        scenario="warning",
        main_status="监测中",
        source_label="真实雷达联调",
        presence="有人",
        breath_rate="22.4 bpm",
        heart_rate="88.0 bpm",
        distance="1.05 m",
        quality="良好",
    ),
    ProductizedSnapshot(
        scenario="radar_unavailable",
        main_status="数据暂不可用",
        source_label="雷达未连接",
        presence="暂无数据",
        breath_rate="N/A",
        heart_rate="N/A",
        distance="N/A",
        quality="Not Verified",
    ),
    ProductizedSnapshot(
        scenario="radar_stale",
        main_status="雷达数据超时",
        source_label="雷达数据超时",
        presence="数据超时",
        breath_rate="N/A",
        heart_rate="N/A",
        distance="N/A",
        quality="Not Verified",
    ),
    ProductizedSnapshot(
        scenario="radar_low_quality",
        main_status="信号较差",
        source_label="雷达信号较差",
        presence="信号较差",
        breath_rate="N/A",
        heart_rate="N/A",
        distance="N/A",
        quality="信号较差",
    ),
    ProductizedSnapshot(
        scenario="radar_invalid",
        main_status="数据无效",
        source_label="数据无效",
        presence="未验证",
        breath_rate="N/A",
        heart_rate="N/A",
        distance="N/A",
        quality="Not Verified",
    ),
]


BOUNDARY_RULES: dict[str, dict] = {
    "real_radar_valid": {
        "must_contain": ["真实雷达联调", "有人", "16.8 bpm", "良好"],
        "must_not_contain": ["暂无数据", "N/A"],
    },
    "attention": {
        "must_contain": ["真实雷达联调", "有人", "17.8 bpm"],
        "must_not_contain": ["暂无数据"],
    },
    "warning": {
        "must_contain": ["真实雷达联调", "有人", "22.4 bpm"],
        "must_not_contain": ["暂无数据"],
    },
    "radar_unavailable": {
        "must_contain": ["暂无数据", "N/A", "雷达未连接", "数据暂不可用"],
        "must_not_contain": ["有人", "16.8 bpm", "17.8 bpm", "22.4 bpm"],
    },
    "radar_stale": {
        "must_contain": ["数据超时", "N/A", "雷达数据超时"],
        "must_not_contain": ["有人", "16.8 bpm", "17.8 bpm", "22.4 bpm"],
    },
    "radar_low_quality": {
        "must_contain": ["信号较差", "N/A", "雷达信号较差"],
        "must_not_contain": ["有人", "16.8 bpm", "17.8 bpm", "22.4 bpm"],
    },
    "radar_invalid": {
        "must_contain": ["N/A", "数据无效", "人体: 未验证"],
        "must_not_contain": ["有人", "16.8 bpm", "17.8 bpm", "22.4 bpm"],
    },
}

ALL_SCENARIO_MUST_CONTAIN = [
    "模型未验证",
    "Not Verified",
    "不作为医学诊断",
    "咳嗽模型",
]


# --- UART log generation ---


def uart_lines(snapshot: ProductizedSnapshot, t_ms: int = 1000) -> list[str]:
    return [
        (
            "[DISPLAY_FINAL_SCREEN] "
            f"t_ms={t_ms} "
            "path=APP_DISPLAY_ENABLE+APP_DISPLAY_FINAL_MOCK_ENABLE "
            f"source=mock scenario={snapshot.scenario} "
            f"radar_src={snapshot.source_label}"
        ),
        "================================================",
        "  E84 夜间呼吸与咳嗽健康伴侣",
        "------------------------------------------------",
        f"  状态: {snapshot.main_status} | 数据源: {snapshot.source_label}",
        "------------------------------------------------",
        "  雷达",
        f"    来源: {snapshot.source_label}",
        f"    人体: {snapshot.presence}",
        f"    呼吸率: {snapshot.breath_rate}",
        f"    心率: {snapshot.heart_rate}",
        f"    距离: {snapshot.distance}",
        f"    质量: {snapshot.quality}",
        "------------------------------------------------",
        "  咳嗽模型",
        "    状态: 模型未验证 / Not Verified",
        "    说明: 模型未完成板级验证",
        "------------------------------------------------",
        "  用于趋势观察与竞赛演示，不作为医学诊断",
        "================================================",
    ]


# --- Validation ---


def validate_log(lines: Iterable[str], scenario: str) -> dict[str, object]:
    joined = "\n".join(lines)
    violations: list[str] = []

    for marker in ALL_SCENARIO_MUST_CONTAIN:
        if marker not in joined:
            violations.append(f"missing required text: {marker}")

    rules = BOUNDARY_RULES.get(scenario, {})
    for marker in rules.get("must_contain", []):
        if marker not in joined:
            violations.append(f"scenario {scenario} missing: {marker}")
    for marker in rules.get("must_not_contain", []):
        if marker in joined:
            violations.append(
                f"scenario {scenario} must not contain: {marker}"
            )

    return {
        "scenario": scenario,
        "boundary_violations": violations,
        "pass": not violations,
    }


# --- PNG rendering ---


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


def scenario_color(scenario: str) -> str:
    colors = {
        "real_radar_valid": "#0f766e",
        "attention": "#b45309",
        "warning": "#dc2626",
        "radar_unavailable": "#6b7280",
        "radar_stale": "#d97706",
        "radar_low_quality": "#ea580c",
        "radar_invalid": "#9ca3af",
    }
    return colors.get(scenario, "#6b7280")


def status_bg_color(scenario: str) -> str:
    colors = {
        "real_radar_valid": "#d1fae5",
        "attention": "#fef3c7",
        "warning": "#fee2e2",
        "radar_unavailable": "#f3f4f6",
        "radar_stale": "#fef3c7",
        "radar_low_quality": "#ffedd5",
        "radar_invalid": "#f3f4f6",
    }
    return colors.get(scenario, "#f3f4f6")


def render_png(snapshot: ProductizedSnapshot, output_path: Path) -> None:
    width, height = 480, 640
    image = Image.new("RGB", (width, height), "#f8fafc")
    draw = ImageDraw.Draw(image)

    title_font = find_font(22)
    section_font = find_font(17)
    label_font = find_font(14)
    value_font = find_font(16)
    small_font = find_font(12)

    accent = scenario_color(snapshot.scenario)
    status_bg = status_bg_color(snapshot.scenario)

    y = 0

    # Header bar
    draw.rectangle((0, y, width, y + 56), fill=accent)
    draw.text((20, y + 14), "E84 夜间呼吸与咳嗽健康伴侣",
              font=title_font, fill="white")
    y += 56

    # Status bar
    draw.rectangle((0, y, width, y + 48), fill=status_bg)
    draw.text((20, y + 8), f"状态: {snapshot.main_status}",
              font=section_font, fill="#111827")
    draw.text((20, y + 28), f"数据源: {snapshot.source_label}",
              font=label_font, fill="#6b7280")
    y += 48

    # Radar section
    y += 12
    draw.text((20, y), "雷达", font=section_font, fill="#111827")
    y += 28

    radar_items = [
        ("来源", snapshot.source_label),
        ("人体", snapshot.presence),
        ("呼吸率", snapshot.breath_rate),
        ("心率", snapshot.heart_rate),
        ("距离", snapshot.distance),
        ("质量", snapshot.quality),
    ]
    for label, value in radar_items:
        draw.text((32, y), f"{label}:", font=label_font, fill="#6b7280")
        # Color the value based on whether it's N/A or real
        if value in ("N/A", "暂无数据", "数据超时", "未验证", "信号较差",
                     "Not Verified"):
            value_color = "#dc2626"
        else:
            value_color = "#059669"
        draw.text((120, y), value, font=value_font, fill=value_color)
        y += 26

    # Separator
    y += 8
    draw.line([(20, y), (width - 20, y)], fill="#e5e7eb", width=1)
    y += 8

    # Cough model section
    draw.text((20, y), "咳嗽模型", font=section_font, fill="#111827")
    y += 28
    draw.text((32, y), "状态:", font=label_font, fill="#6b7280")
    draw.text((120, y), "模型未验证 / Not Verified",
              font=value_font, fill="#dc2626")
    y += 26
    draw.text((32, y), "说明:", font=label_font, fill="#6b7280")
    draw.text((120, y), "模型未完成板级验证", font=label_font, fill="#6b7280")
    y += 26

    # Separator
    y += 8
    draw.line([(20, y), (width - 20, y)], fill="#e5e7eb", width=1)
    y += 8

    # Disclaimer
    draw.text((20, y),
              "用于趋势观察与竞赛演示，不作为医学诊断",
              font=small_font, fill="#9ca3af")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


# --- Main ---


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render productized Display final screen preview."
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("docs/phase3/display_final_screen_productization_evidence"),
        help="Directory for generated artifacts.",
    )
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    all_results: list[dict] = []
    aggregate_uart_lines: list[str] = []
    all_pass = True

    for index, snapshot in enumerate(SCENARIOS):
        scenario = snapshot.scenario
        log_path = args.out_dir / f"{scenario}_uart.log"
        png_path = args.out_dir / f"{scenario}_preview.png"
        report_path = args.out_dir / f"{scenario}_report.json"

        lines = uart_lines(snapshot)
        aggregate_uart_lines.extend(uart_lines(snapshot, t_ms=1000 * (index + 1)))
        log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        render_png(snapshot, png_path)

        validation = validate_log(lines, scenario)
        report = {
            "mode": "productized_host_preview_no_board",
            "scenario": scenario,
            "snapshot": asdict(snapshot),
            "log_path": str(log_path),
            "png_path": str(png_path),
            "validation": validation,
            "boundaries": {
                "ble_wire_format_changed": False,
                "shared_memory_abi_changed": False,
                "ml_model_changed": False,
                "labels_changed": False,
                "raw_data_modified": False,
            },
        }
        report_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        all_results.append(report)
        if not validation["pass"]:
            all_pass = False
            print(f"FAIL {scenario}: {validation}")

    # Aggregate report
    aggregate_uart_path = args.out_dir / "productization_all_scenarios_uart.log"
    aggregate_uart_path.write_text(
        "\n".join(aggregate_uart_lines) + "\n",
        encoding="utf-8",
    )

    aggregate_path = args.out_dir / "productization_aggregate.json"
    aggregate = {
        "tool": "render_display_final_screen_productized.py",
        "aggregate_uart_log": str(aggregate_uart_path),
        "scenario_count": len(SCENARIOS),
        "pass_count": sum(
            1 for r in all_results if r["validation"]["pass"]
        ),
        "fail_count": sum(
            1 for r in all_results if not r["validation"]["pass"]
        ),
        "all_pass": all_pass,
        "scenarios": [r["scenario"] for r in all_results],
        "results": all_results,
    }
    aggregate_path.write_text(
        json.dumps(aggregate, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"aggregate_report={aggregate_path}")
    print(f"scenario_count={len(SCENARIOS)}")
    print(f"all_pass={all_pass}")
    for r in all_results:
        status = "PASS" if r["validation"]["pass"] else "FAIL"
        print(f"  {status} {r['scenario']}")

    return 0 if all_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
