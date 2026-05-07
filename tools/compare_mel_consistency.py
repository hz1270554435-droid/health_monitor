from __future__ import annotations

import argparse
import csv
from pathlib import Path

import librosa
import numpy as np


SAMPLE_RATE = 16000
WINDOW_SAMPLES = 16000
WINDOW_HOP_SAMPLES = 8000
N_FFT = 1024
HOP_LENGTH = 160
N_MELS = 40
FMIN = 50.0
FMAX = 7600.0
LOG_EPSILON = 1.0e-6
ENERGY_GATE = 1.0e-6
TARGET_RMS = 0.10
MAX_GAIN = 20.0
TOP_DB = 80.0


def load_mono_window(path: Path) -> tuple[np.ndarray, int, float]:
    audio, _ = librosa.load(path, sr=SAMPLE_RATE, mono=True)
    audio = audio.astype(np.float32, copy=False)
    if audio.size == 0:
        return np.zeros(WINDOW_SAMPLES, dtype=np.float32), 0, 0.0

    if audio.size <= WINDOW_SAMPLES:
        chunk = np.pad(audio, (0, WINDOW_SAMPLES - audio.size))
        return chunk.astype(np.float32, copy=False), 0, rms_energy(chunk)

    best_start = 0
    best_energy = -1.0
    last_start = audio.size - WINDOW_SAMPLES
    for start in range(0, last_start + 1, WINDOW_HOP_SAMPLES):
        chunk = audio[start : start + WINDOW_SAMPLES]
        energy = rms_energy(chunk)
        if energy > best_energy:
            best_energy = energy
            best_start = start

    chunk = audio[best_start : best_start + WINDOW_SAMPLES]
    return chunk.astype(np.float32, copy=False), best_start, best_energy


def rms_energy(audio: np.ndarray) -> float:
    centered = audio.astype(np.float64) - float(np.mean(audio))
    return float(np.mean(centered * centered))


def board_condition_window(audio: np.ndarray) -> tuple[np.ndarray, float, bool]:
    window = audio.astype(np.float32, copy=True)
    mean = float(np.mean(window, dtype=np.float64))
    window -= mean
    energy = float(np.mean(window.astype(np.float64) * window.astype(np.float64)))
    if energy < ENERGY_GATE:
        return window, energy, False

    rms = float(np.sqrt(energy))
    gain = TARGET_RMS / rms
    if gain > MAX_GAIN:
        gain = MAX_GAIN
    window *= np.float32(gain)
    return window, energy, True


def pc_training_feature(audio: np.ndarray) -> np.ndarray:
    mel = librosa.feature.melspectrogram(
        y=audio,
        sr=SAMPLE_RATE,
        n_fft=N_FFT,
        hop_length=HOP_LENGTH,
        n_mels=N_MELS,
        fmin=FMIN,
        fmax=FMAX,
        power=2.0,
    )
    return normalize_log_mel(mel)


def pc_htk_none_feature(audio: np.ndarray) -> np.ndarray:
    mel = librosa.feature.melspectrogram(
        y=audio,
        sr=SAMPLE_RATE,
        n_fft=N_FFT,
        hop_length=HOP_LENGTH,
        n_mels=N_MELS,
        fmin=FMIN,
        fmax=FMAX,
        power=2.0,
        htk=True,
        norm=None,
    )
    return normalize_log_mel(mel)


def board_equivalent_feature(audio: np.ndarray) -> tuple[np.ndarray, float, bool]:
    conditioned, energy, valid = board_condition_window(audio)
    features = np.zeros((N_MELS, 1 + WINDOW_SAMPLES // HOP_LENGTH), dtype=np.float32)
    max_mel_energy = 0.0
    edges = board_mel_edges()
    hann = board_hann_window(N_FFT)

    for t in range(features.shape[1]):
        frame_start = t * HOP_LENGTH - (N_FFT // 2)
        frame = np.zeros(N_FFT, dtype=np.float32)
        for n in range(N_FFT):
            src = frame_start + n
            if 0 <= src < WINDOW_SAMPLES:
                frame[n] = conditioned[src] * hann[n]

        spectrum = np.fft.rfft(frame, n=N_FFT)
        power = (spectrum.real * spectrum.real) + (spectrum.imag * spectrum.imag)

        for mel in range(N_MELS):
            value = board_mel_energy(power, edges[mel], edges[mel + 1], edges[mel + 2])
            features[mel, t] = np.float32(value)
            if value > max_mel_energy:
                max_mel_energy = float(value)

    return board_power_to_db_and_normalize(features, max_mel_energy), energy, valid


def board_hann_window(length: int) -> np.ndarray:
    index = np.arange(length, dtype=np.float32)
    return 0.5 - 0.5 * np.cos((2.0 * np.pi * index) / float(length))


def board_hz_to_mel(hz: float) -> float:
    return 2595.0 * np.log10(1.0 + hz / 700.0)


def board_mel_to_hz(mel: float) -> float:
    return 700.0 * (np.power(10.0, mel / 2595.0) - 1.0)


def board_mel_edges() -> np.ndarray:
    low_mel = board_hz_to_mel(FMIN)
    high_mel = board_hz_to_mel(FMAX)
    edges = []
    for i in range(N_MELS + 2):
        mel = low_mel + (high_mel - low_mel) * (i / float(N_MELS + 1))
        hz = board_mel_to_hz(mel)
        bin_index = int(((N_FFT + 1) * hz) / SAMPLE_RATE)
        edges.append(min(bin_index, N_FFT // 2))
    return np.asarray(edges, dtype=np.int32)


def board_mel_energy(power: np.ndarray, start: int, center: int, end: int) -> float:
    if start >= center or center >= end:
        return 0.0

    energy = 0.0
    for k in range(start, center):
        weight = (k - start) / float(center - start)
        energy += float(power[k]) * weight

    for k in range(center, end + 1):
        weight = (end - k) / float(end - center)
        energy += float(power[k]) * weight

    return energy


def board_power_to_db_and_normalize(mel: np.ndarray, max_mel_energy: float) -> np.ndarray:
    amin = 1.0e-10
    ref = max(max_mel_energy, amin)
    ref_db = 10.0 * np.log10(ref)
    db = 10.0 * np.log10(np.maximum(mel, amin)) - ref_db
    db = np.maximum(db, -TOP_DB).astype(np.float32, copy=False)
    return standardize(db)


def normalize_log_mel(mel: np.ndarray) -> np.ndarray:
    log_mel = librosa.power_to_db(mel, ref=np.max).astype(np.float32)
    return standardize(log_mel)


def standardize(feature: np.ndarray) -> np.ndarray:
    mean = float(np.mean(feature, dtype=np.float64))
    std = float(np.std(feature, dtype=np.float64))
    return ((feature - mean) / (std + LOG_EPSILON)).astype(np.float32, copy=False)


def compare_metrics(reference: np.ndarray, candidate: np.ndarray) -> dict[str, float]:
    ref = reference.astype(np.float64).ravel()
    cand = candidate.astype(np.float64).ravel()
    diff = cand - ref
    ref_norm = float(np.linalg.norm(ref))
    cand_norm = float(np.linalg.norm(cand))
    if ref_norm == 0.0 or cand_norm == 0.0:
        cosine = 0.0
    else:
        cosine = float(np.dot(ref, cand) / (ref_norm * cand_norm))

    if np.std(ref) == 0.0 or np.std(cand) == 0.0:
        corr = 0.0
    else:
        corr = float(np.corrcoef(ref, cand)[0, 1])

    abs_diff = np.abs(diff)
    return {
        "mean_abs": float(np.mean(abs_diff)),
        "max_abs": float(np.max(abs_diff)),
        "p95_abs": float(np.percentile(abs_diff, 95)),
        "rmse": float(np.sqrt(np.mean(diff * diff))),
        "cosine": cosine,
        "corr": corr,
    }


def collect_audio_files(root: Path, max_per_class: int) -> list[tuple[str, Path]]:
    cough: list[Path] = []
    non_cough: list[Path] = []
    other: list[Path] = []

    for path in sorted(root.rglob("*.wav")):
        parts = {part.lower() for part in path.parts}
        if "non_cough" in parts:
            non_cough.append(path)
        elif "cough" in parts:
            cough.append(path)
        else:
            other.append(path)

    selected: list[tuple[str, Path]] = []
    selected.extend(("cough", path) for path in cough[:max_per_class])
    selected.extend(("non_cough", path) for path in non_cough[:max_per_class])
    if not selected:
        selected.extend(("audio", path) for path in other[: max_per_class * 2])
    return selected


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--audio-root", type=Path, default=Path(r"D:\cough_model_train\DATA"))
    parser.add_argument("--max-per-class", type=int, default=5)
    parser.add_argument("--out-csv", type=Path, default=Path("report/mel_consistency_2026-05-07.csv"))
    args = parser.parse_args()

    rows: list[dict[str, object]] = []
    files = collect_audio_files(args.audio_root, args.max_per_class)
    if not files:
        raise SystemExit(f"No wav files found under {args.audio_root}")

    for label, path in files:
        audio, start_sample, selected_energy = load_mono_window(path)
        pc = pc_training_feature(audio)
        board, board_energy, valid = board_equivalent_feature(audio)
        conditioned, _, _ = board_condition_window(audio)
        pc_conditioned = pc_training_feature(conditioned)
        htk_conditioned = pc_htk_none_feature(conditioned)

        metric_sets = {
            "pc_vs_board_current": compare_metrics(pc, board),
            "pc_conditioned_vs_board": compare_metrics(pc_conditioned, board),
            "pc_htk_none_conditioned_vs_board": compare_metrics(htk_conditioned, board),
        }

        for compare_name, metrics in metric_sets.items():
            row: dict[str, object] = {
                "label": label,
                "file": str(path),
                "start_sample": start_sample,
                "selected_energy": selected_energy,
                "board_energy": board_energy,
                "board_valid": int(valid),
                "compare": compare_name,
            }
            row.update(metrics)
            rows.append(row)

    args.out_csv.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0].keys())
    with args.out_csv.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {args.out_csv}")
    for compare_name in sorted({str(row["compare"]) for row in rows}):
        subset = [row for row in rows if row["compare"] == compare_name]
        print(f"\n[{compare_name}]")
        for key in ("mean_abs", "max_abs", "p95_abs", "rmse", "cosine", "corr"):
            values = np.asarray([float(row[key]) for row in subset], dtype=np.float64)
            print(f"{key}: mean={values.mean():.6f}, min={values.min():.6f}, max={values.max():.6f}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
