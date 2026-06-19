#!/usr/bin/env python3
"""Host renderer for Display source boundary preview.

Renders UART-style log + PNG preview for each radar source boundary scenario.
Does not connect to firmware, BLE, model, audio, radar, or shared memory.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Iterable

from PIL import Image, ImageDraw, ImageFont


# --- Data models ---


@dataclass(frozen=True)
class BoundarySnapshot:
    """One display scenario with source boundary semantics."""
    scenario: str
    title: str = "E84 呼吸健康监测"
    runtime: str = "00:42"
    health_state: str = "正常"
    health_note: str = "呼吸平稳，未见明显异常"
    source_label: str = "真实雷达联调"
    cough_text: str = "模型未验证 | Not Verified"
    cough_count_1min: int = 0
    cough_count_5min: int = 0
    presence: str = "有人"
    breath_rate_bpm: float = 16.8
    heart_rate_bpm: float = 71.0
    breath_rate_display: str = "16.800 次/分"
    heart_rate_display: str = "71.000 次/分"
    radar_quality_display: str = "90%"
    night_duration: str = "6小时18分"
    night_stable_percent: int = 92
    night_reminders: int = 1
    mic_status: str = "正常"
    radar_device_status: str = "真实雷达联调"
    ble_status: str = "未连接"
    battery_status: str = "充足(模拟)"
    disclaimer: str = "工程显示效果验证，非医疗诊断或治疗建议。"


# --- Scenario definitions ---

SCENARIOS: list[BoundarySnapshot] = [
    BoundarySnapshot(
        scenario="real_radar_valid",
        health_state="正常",
        health_note="呼吸平稳，未见明显异常",
        source_label="真实雷达联调",
        cough_text="模型未验证 | Not Verified",
        cough_count_5min=1,
        presence="有人",
        breath_rate_bpm=16.8,
        heart_rate_bpm=71.0,
        breath_rate_display="16.800 次/分",
        heart_rate_display="71.000 次/分",
        radar_quality_display="90%",
        radar_device_status="真实雷达联调",
    ),
    BoundarySnapshot(
        scenario="attention",
        health_state="关注",
        health_note="咳嗽略高，呼吸平稳",
        source_label="真实雷达联调",
        cough_text="模型未验证 | Not Verified",
        cough_count_1min=1,
        cough_count_5min=2,
        presence="有人",
        breath_rate_bpm=17.8,
        heart_rate_bpm=72.0,
        breath_rate_display="17.800 次/分",
        heart_rate_display="72.000 次/分",
        radar_quality_display="88%",
        radar_device_status="真实雷达联调",
    ),
    BoundarySnapshot(
        scenario="warning",
        health_state="警示",
        health_note="咳嗽偏高，请留意观察",
        source_label="真实雷达联调",
        cough_text="模型未验证 | Not Verified",
        cough_count_1min=3,
        cough_count_5min=6,
        presence="有人",
        breath_rate_bpm=22.4,
        heart_rate_bpm=88.0,
        breath_rate_display="22.400 次/分",
        heart_rate_display="88.000 次/分",
        radar_quality_display="76%",
        radar_device_status="真实雷达联调",
    ),
    BoundarySnapshot(
        scenario="radar_unavailable",
        health_state="雷达暂无数据",
        health_note="在位、呼吸和心率暂未验证",
        source_label="雷达未连接",
        cough_text="模型未验证 | Not Verified",
        presence="暂无数据",
        breath_rate_bpm=0.0,
        heart_rate_bpm=0.0,
        breath_rate_display="N/A",
        heart_rate_display="N/A",
        radar_quality_display="暂无数据",
        radar_device_status="雷达未连接",
    ),
    BoundarySnapshot(
        scenario="radar_stale",
        health_state="关注",
        health_note="咳嗽略高，呼吸平稳",
        source_label="雷达数据超时",
        cough_text="模型未验证 | Not Verified",
        cough_count_1min=1,
        cough_count_5min=2,
        presence="数据超时",
        breath_rate_bpm=0.0,
        heart_rate_bpm=0.0,
        breath_rate_display="N/A",
        heart_rate_display="N/A",
        radar_quality_display="数据超时",
        radar_device_status="雷达数据超时",
    ),
    BoundarySnapshot(
        scenario="radar_low_quality",
        health_state="正常",
        health_note="呼吸平稳，未见明显异常",
        source_label="雷达信号较差",
        cough_text="模型未验证 | Not Verified",
        presence="信号较差",
        breath_rate_bpm=0.0,
        heart_rate_bpm=0.0,
        breath_rate_display="N/A",
        heart_rate_display="N/A",
        radar_quality_display="信号较差",
        radar_device_status="雷达信号较差",
    ),
    BoundarySnapshot(
        scenario="radar_invalid",
        health_state="正常",
        health_note="呼吸平稳，未见明显异常",
        source_label="雷达数据异常",
        cough_text="模型未验证 | Not Verified",
        presence="未验证",
        breath_rate_bpm=0.0,
        heart_rate_bpm=0.0,
        breath_rate_display="N/A",
        heart_rate_display="N/A",
        radar_quality_display="未验证",
        radar_device_status="雷达数据异常",
    ),
]


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

BOUNDARY_RULES = {
    "radar_unavailable": {
        "must_not_contain": ["有人", "16.800", "71.000", "90%"],
        "must_contain": ["暂无数据", "N/A", "雷达未连接"],
    },
    "radar_stale": {
        "must_not_contain": ["有人", "16.800", "71.000"],
        "must_contain": ["数据超时", "N/A", "雷达数据超时"],
    },
    "radar_low_quality": {
        "must_not_contain": ["有人", "16.800", "71.000"],
        "must_contain": ["信号较差", "N/A", "雷达信号较差"],
    },
    "radar_invalid": {
        "must_not_contain": ["有人", "16.800", "71.000", "90%"],
        "must_contain": ["未验证", "N/A", "雷达数据异常"],
    },
    "real_radar_valid": {
        "must_contain": ["真实雷达联调", "有人"],
    },
}

ALL_SCENARIO_MUST_CONTAIN = ["模型未验证", "Not Verified"]


# --- UART log generation ---


def uart_lines(snapshot: BoundarySnapshot, t_ms: int = 1000) -> list[str]:
    return [
        (
            "[DISPLAY_FINAL_SCREEN] "
            f"t_ms={t_ms} "
            "path=APP_DISPLAY_ENABLE+APP_DISPLAY_FINAL_MOCK_ENABLE "
            f"source=mock scenario={snapshot.scenario} "
            f"radar_src={snapshot.source_label}"
        ),
        "================================================",
        f"标题: {snapshot.title}",
        (
            "运行状态: "
            f"模拟展示中 | 已运行 {snapshot.runtime} | "
            f"数据源 {snapshot.source_label}"
        ),
        (
            "当前健康状态: "
            f"{snapshot.health_state} | {snapshot.health_note}"
        ),
        (
            "咳嗽监测: "
            f"近1分钟 {snapshot.cough_count_1min}次 | "
            f"近5分钟 {snapshot.cough_count_5min}次 | "
            f"{snapshot.cough_text}"
        ),
        (
            "呼吸辅助: "
            f"在位状态 {snapshot.presence} | "
            f"呼吸 {snapshot.breath_rate_display} | "
            f"心率 {snapshot.heart_rate_display} | "
            f"雷达质量 {snapshot.radar_quality_display}"
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
            f"{snapshot.radar_device_status} | "
            f"BLE{snapshot.ble_status} | "
            f"电量{snapshot.battery_status}"
        ),
        f"免责声明: {snapshot.disclaimer}",
        "================================================",
    ]


# --- Validation ---


def validate_log(lines: Iterable[str], scenario: str) -> dict[str, object]:
    joined = "\n".join(lines)
    missing_sections = [
        s for s in REQUIRED_SECTIONS if s not in joined
    ]

    violations: list[str] = []

    # All scenarios must contain cough model boundary text
    for marker in ALL_SCENARIO_MUST_CONTAIN:
        if marker not in joined:
            violations.append(f"missing required boundary text: {marker}")

    # Scenario-specific rules
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
        "required_sections": list(REQUIRED_SECTIONS),
        "missing_sections": missing_sections,
        "boundary_violations": violations,
        "pass": not missing_sections and not violations,
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


def scenario_color(scenario: str) -> str:
    """Return accent color for scenario."""
    colors = {
        "real_radar_valid": "#0f766e",
        "attention": "#b45309",
        "warning": "#dc2626",
        "radar_unavailable": "#6b7280",
        "radar_stale": "#d97706",
        "radar_low_quality": "#ea580c",
        "radar_invalid": "#9333ea",
    }
    return colors.get(scenario, "#6b7280")


def render_png(snapshot: BoundarySnapshot, output_path: Path) -> None:
    width, height = 800, 480
    image = Image.new("RGB", (width, height), "#f6f7f9")
    draw = ImageDraw.Draw(image)

    title_font = find_font(30)
    subtitle_font = find_font(16)
    section_font = find_font(18)
    body_font = find_font(15)
    small_font = find_font(13)
    state_font = find_font(34)

    accent = scenario_color(snapshot.scenario)

    # Header bar
    draw.rectangle((0, 0, width, 76), fill=accent)
    draw.text((28, 16), snapshot.title, font=title_font, fill="white")
    draw.text(
        (540, 22),
        f"模拟展示中 | {snapshot.runtime} | {snapshot.source_label}",
        font=subtitle_font,
        fill="#ccfbf1",
    )

    # Health state card
    draw.rounded_rectangle(
        (28, 96, 772, 164), radius=8, fill="#ffffff", outline="#d1d5db"
    )
    draw.text((48, 112), "当前状态", font=section_font, fill="#111827")
    draw.text((156, 106), snapshot.health_state, font=state_font, fill=accent)
    draw.text((258, 119), snapshot.health_note, font=section_font, fill="#374151")

    # Cough card
    draw_card(
        draw,
        (28, 184, 378, 286),
        "咳嗽监测",
        [
            f"近1分钟 {snapshot.cough_count_1min}次    近5分钟 {snapshot.cough_count_5min}次",
            snapshot.cough_text,
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )

    # Breathing card
    draw_card(
        draw,
        (400, 184, 772, 286),
        "呼吸辅助",
        [
            f"{snapshot.presence}    雷达质量 {snapshot.radar_quality_display}",
            f"呼吸 {snapshot.breath_rate_display}    心率 {snapshot.heart_rate_display}",
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )

    # Night summary card
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

    # Device status card
    draw_card(
        draw,
        (400, 306, 772, 404),
        "设备状态",
        [
            f"MIC{snapshot.mic_status}    {snapshot.radar_device_status}",
            f"BLE{snapshot.ble_status}    电量{snapshot.battery_status}",
        ],
        section_font,
        body_font,
        "#ffffff",
        "#d1d5db",
    )

    # Footer
    draw.rectangle((0, 438, width, height), fill="#e5e7eb")
    draw.text(
        (28, 450),
        f"免责声明: {snapshot.disclaimer}",
        font=small_font,
        fill="#4b5563",
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    image.save(output_path)


# --- Main ---


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Render Display source boundary preview for all scenarios."
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("docs/phase3/display_source_boundary_evidence"),
        help="Directory for generated host validation artifacts.",
    )
    args = parser.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    all_results: list[dict] = []
    all_pass = True

    for snapshot in SCENARIOS:
        scenario = snapshot.scenario
        log_path = args.out_dir / f"{scenario}_uart.log"
        png_path = args.out_dir / f"{scenario}_preview.png"
        report_path = args.out_dir / f"{scenario}_report.json"

        lines = uart_lines(snapshot)
        log_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        render_png(snapshot, png_path)

        validation = validate_log(lines, scenario)
        report = {
            "mode": "host_boundary_preview_no_board",
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

    # Write aggregate report
    aggregate_path = args.out_dir / "display_source_boundary_aggregate.json"
    aggregate = {
        "tool": "render_display_source_boundary_preview.py",
        "scenario_count": len(SCENARIOS),
        "pass_count": sum(1 for r in all_results if r["validation"]["pass"]),
        "fail_count": sum(1 for r in all_results if not r["validation"]["pass"]),
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
