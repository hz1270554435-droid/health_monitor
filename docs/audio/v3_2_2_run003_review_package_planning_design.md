# MIC v3.2.2 run003 review package planning design

Date: 2026-06-04

## 1. Verdict

**PASS — planning design document only.**

This document defines a proposed planning-only policy for a future review package gate based on the frozen run003 EAT review priority dry-run. It does not authorize review package generation, clip export, labels, candidate labels, training, or F4/F5/F6 artifacts.

## 2. Purpose

Convert the frozen review priority queues into a human-review package design. This planning document answers:

```text
Given the frozen dry-run priority queues, how should a review package be structured for human review?
```

This is planning only. No review package is generated. No clips are exported. No labels, candidate labels, training outputs, or F4/F5/F6 artifacts are created.

## 3. Frozen upstream state

| Stage | Artifact | Status |
|---|---|---|
| F0 source inventory | `source_inventory_run003.csv` (263 rows) | PASS / frozen |
| F1 full window manifest | `window_manifest_run003_f1_full.csv` (102,219 rows) | PASS / frozen |
| EAT context manifest | `efficientat_context_manifest_10s_full.csv` (102,219 rows) | PASS / frozen |
| EAT window-context map | `efficientat_window_context_map_10s_full.csv` (102,219 rows) | PASS / frozen |
| EAT full scores | `efficientat_10s_scores_full_wide.csv` (102,219 rows) | PASS / frozen |
| Policy design | `v3_2_2_run003_eat_review_priority_policy_design.md` | PASS / frozen |
| Data processing closeout | `v3_2_2_run003_eat_data_processing_closeout.md` | PASS / frozen |
| Review priority dry-run | `eat_review_priority_dryrun.csv` (102,219 rows) | PASS / frozen |
| Dry-run report | `eat_review_priority_dryrun_report.md` | PASS / frozen |

## 4. Candidate pool summary

From the frozen dry-run CSV:

| Priority | Count | Unique sources | Description |
|---|---:|---:|---|
| P0_blocked_or_excluded | 5,412 | 3 | Blocked by unknown beep risk (all B4) |
| P1_high_review_priority | 3,631 | 27 | Strong cough candidates (all B1) |
| P2_medium_review_priority | 6,484 | 161 | Medium cough/hard-negative/conflict (B1, B3, B7) |
| P3_low_review_priority | 86,692 | 260 | Low-signal baseline (B3, B6) |
| **P1 + P2 total** | **10,115** | **161** | **Review candidate pool** |

### P1 breakdown by source_prior_type

| source_prior_type | P1 count |
|---|---:|
| high_conf_filename_cough | 67 |
| whole_night_mixed | 3,564 |
| **Total** | **3,631** |

### P2 breakdown by source_prior_type and bucket

| source_prior_type | bucket | P2 count |
|---|---|---:|
| high_conf_filename_cough | B1 | 588 |
| high_conf_filename_cough | B3 | 0 |
| high_conf_filename_cough | B7 | 41 |
| speech_laugh_throat_hard_negative_pool | B3 | 2,264 |
| speech_laugh_throat_hard_negative_pool | B7 | 2 |
| whole_night_mixed | B1 | 3,563 |
| whole_night_mixed | B3 | 0 |
| whole_night_mixed | B7 | 0 |
| **Total** | | **6,459** |

Note: P2 rows also include 26 high_conf_filename_cough B1 rows (655 total P2 for that source_prior_type).

### Source concentration risk

The top 10 sources by P1+P2 count are all `hole_night_sound` whole-night recordings. The top source alone has 746 P1+P2 rows. Without per-source caps, a single 10-minute overnight recording could dominate the review package.

### B4 playback exclusion

All 5,412 `mixed_playback_rerecord` rows are P0 due to unknown beep risk. These are excluded from the review candidate pool until beep localization is separately approved and completed.

### Empty buckets/priorities

| Bucket/Priority | Count | Explanation |
|---|---:|---|
| B2_whole_night_possible_cough_event | 0 | cough_score never exceeds speech/silence in whole_night_mixed |
| B5_background_or_quiet_anomaly | 0 | background_or_quiet cough_score never reaches global p90 |
| P4_diagnostic_sample_only | 0 | all B4 rows are P0 (unknown beep risk) |

These empty statuses are preserved as known dry-run observations. They do not block the review package planning.

## 5. Sampling strategy

### 5.1 Guiding principles

1. Human review time is the bottleneck. The package must be small enough to complete within a reasonable review session.
2. High-priority cough candidates (P1) must be fully represented.
3. Hard-negative and conflict cases (P2) must be sampled to calibrate false-positive behavior.
4. Source diversity must be maintained to avoid over-representing any single recording.
5. Long overnight files must not dominate the package.
6. B4 playback rows must remain excluded until beep localization is separately approved.

### 5.2 Proposed review package size

**Target: 500–800 clips** for the initial review package.

This is a planning target. The actual size will be determined at the review package gate after Codex review and approval.

### 5.3 Proposed quota allocation

| Quota | Source | Count | Notes |
|---|---|---:|---|
| Q1: P1 full coverage | P1_high_review_priority | 300 | Cap at 300; if P1 > 300, sample by rank_score descending |
| Q2: P2 B1 sample | P2 × B1 | 100 | High cough in non-P1 range |
| Q3: P2 B3 sample | P2 × B3 | 100 | Hard-negative speech/laugh/throat candidates |
| Q4: P2 B7 full | P2 × B7 | 43 | All conflict/ambiguous cases (small set) |
| Q5: whole_night exploratory | P3 × whole_night_mixed | 50 | Random sample from P3 whole-night for baseline |
| Q6: B6 control | P3 × B6 | 50 | Low-priority controls for false-negative calibration |
| **Total** | | **643** | Within 500–800 target |

### 5.4 P1 sampling detail

P1 has 3,631 rows across 27 sources. Full coverage is impractical. Proposed:

- Sort by `rank_score` descending (which is `cough_score` for P1 rows).
- Select top 300 rows.
- Apply per-source cap (see Section 6) to prevent any single source from dominating.
- If a source hits its cap, skip remaining rows from that source and continue down the ranked list.

### 5.5 P2 sampling detail

P2 has 6,484 rows across 161 sources. Three sub-quotas:

- **Q2 (P2 × B1, 100 rows):** Sort by `rank_score` descending, apply per-source cap.
- **Q3 (P2 × B3, 100 rows):** Sort by `rank_score` descending, apply per-source cap. These are hard-negative candidates from the speech/laugh/throat pool.
- **Q4 (P2 × B7, 43 rows):** Include all 43 conflict/ambiguous cases. These are small enough for full review.

### 5.6 P3 exploratory and control quotas

- **Q5 (P3 × whole_night_mixed, 50 rows):** Random sample from the 64,142 P3 whole-night rows. Purpose: verify that the model does not miss events in low-priority overnight segments. Use a fixed random seed for reproducibility.
- **Q6 (P3 × B6, 50 rows):** Random sample from the 72,174 B6 low-priority rows. Purpose: false-negative calibration. Use a fixed random seed for reproducibility.

### 5.7 B4 exclusion

B4 playback/rerecord rows (5,412, all P0) are excluded from the review package. They remain blocked until:

1. Beep localization is separately approved and completed.
2. Beep-safe exclusion zones are established.
3. A separate gate approves B4 clip selection with beep-aware metadata.

### 5.8 B2/B5 empty handling

B2 (0 rows) and B5 (0 rows) are empty. No clips are selected from these buckets. Their empty status is a known observation from the dry-run, not a blocking issue. Future policy revisions may adjust thresholds to populate these buckets.

## 6. Anti-duplication and diversity rules

### 6.1 Per-source maximum clip count

**Proposed cap: 20 clips per source** for the review package.

Rationale: The top source has 746 P1+P2 rows. Without a cap, a single overnight recording could consume 50%+ of the review package. A 20-clip cap ensures:

- At least 20 sources are represented in Q1 (P1).
- Long overnight files are sampled, not exhaustively reviewed.
- Review time is distributed across diverse acoustic conditions.

Exception: If a source has fewer than 20 eligible rows, include all of them.

### 6.2 Minimum temporal spacing within source

**Proposed minimum spacing: 5 seconds** between selected clips from the same source.

Rationale: The dry-run uses 10s contexts with centered-with-padding semantics. Adjacent contexts may overlap significantly. A 5-second minimum center-to-center spacing ensures:

- Selected clips represent distinct acoustic regions.
- Review time is not wasted on near-duplicate segments.
- Local rank peaks are preferred over adjacent near-duplicate windows.

Implementation: After ranking by `rank_score`, greedily select clips. For each selected clip, exclude other clips from the same source whose `window_center_s` is within ±5 seconds.

### 6.3 Source_prior_type diversity

The review package must include clips from all non-empty source_prior_types:

| source_prior_type | Available P1+P2 | Proposed minimum |
|---|---:|---:|
| high_conf_filename_cough | 722 | 50 |
| speech_laugh_throat_hard_negative_pool | 2,266 | 50 |
| whole_night_mixed | 7,127 | 50 (from P3 exploratory) |

The `background_or_quiet` source_prior_type has 0 P1/P2 rows. Its P3 rows (455) are available for the B6 control quota but are not required.

### 6.4 Long-file domination prevention

The 60 `hole_night_sound` files are 10 minutes each (600 seconds). With 0.5s hop, each produces ~1,199 windows. The per-source cap of 20 clips and minimum 5-second spacing prevent these files from dominating the review package.

## 7. Future clip export design

This section defines the proposed clip export format for a future review package gate. **This planning task does not export clips.**

### 7.1 Clip duration and context window

**Proposed clip duration: 10 seconds** (matching the EAT context window).

Each clip corresponds to the 10-second context audio centered on the selected window. The clip includes:

- The 1s target window at its center.
- 4.5s of audio before and after the target window (or padding if near source boundaries).

This matches the `context_start_s` / `context_end_s` / `left_padding_s` / `right_padding_s` fields in the dry-run CSV.

Alternative considered: 5-second clips (target window + 2s each side). This would reduce review time but may miss events near context boundaries.

### 7.2 Clip format

**Proposed format: WAV, 16kHz, mono.**

This matches the source audio format. No resampling or channel conversion is needed for human review.

### 7.3 Proposed naming convention

```text
{run_id}_{priority}_{bucket}_{context_id_short}_{source_id_short}.wav
```

Example:

```text
run003_P1_B1_ctxaf032b5cd_src984a43acb.wav
```

Components:

- `run_id`: `run003`
- `priority`: `P0`–`P4`
- `bucket`: `B1`–`B7`
- `context_id_short`: first 12 hex chars of `context_id`
- `source_id_short`: first 12 hex chars of source ID derived from source path

### 7.4 Proposed metadata sidecar columns

Each exported clip should have a companion metadata row (in a review sheet CSV) with:

```text
clip_filename
run_id
policy_version
dryrun_version
context_id
window_id
source_relpath
source_prior_type
window_start_s
window_end_s
window_center_s
context_start_s
context_end_s
left_padding_s
right_padding_s
contains_beep
context_beep_risk
bucket_id
priority_level
rank_score
cough_score
speech_score
throat_clearing_score
laughter_score
silence_score
music_score
top1_label
top1_score
top5_labels
bucket_reason
priority_reason
manual_decision        (empty before human review)
label_authority        (false before human review)
reviewer_notes         (empty before human review)
```

## 8. Future review sheet design

This section defines the proposed review sheet format for a future review package gate. **This planning task does not create a review sheet.**

### 8.1 Proposed columns for human review

The review sheet extends the metadata sidecar with human-review columns:

| Column | Purpose | Default |
|---|---|---|
| `clip_filename` | Link to exported audio clip | (filled by export) |
| `context_id` | Unique context identifier | (from dry-run) |
| `window_id` | Unique window identifier | (from dry-run) |
| `source_relpath` | Source file path | (from dry-run) |
| `source_prior_type` | Source-level prior hint | (from dry-run) |
| `bucket_id` | Diagnostic bucket | (from dry-run) |
| `priority_level` | Review priority | (from dry-run) |
| `rank_score` | Ranking score | (from dry-run) |
| `cough_score` | EAT cough score | (from dry-run) |
| `speech_score` | EAT speech score | (from dry-run) |
| `top1_label` | EAT top-1 label | (from dry-run) |
| `top5_labels` | EAT top-5 labels | (from dry-run) |
| `bucket_reason` | Why this bucket | (from dry-run) |
| `priority_reason` | Why this priority | (from dry-run) |
| `manual_decision` | Human label decision | **empty** |
| `label_authority` | Whether label is authoritative | **false** |
| `reviewer_notes` | Free-text reviewer notes | **empty** |

### 8.2 manual_decision semantics

`manual_decision` must remain empty before human review. After human review, the reviewer fills in:

- `cough`: reviewer confirms a cough event in the clip.
- `non_cough`: reviewer confirms no cough event in the clip.
- `ambiguous`: reviewer cannot determine (e.g., overlapping sounds, poor quality).
- `skip`: reviewer skips the clip (e.g., beep, corrupted, unlistenable).

### 8.3 label_authority semantics

`label_authority` must remain `false` before human review. After human review and a separate label publication gate, `label_authority` may be set to `true` for approved rows. This planning task does not set `label_authority` to `true`.

### 8.4 EAT bucket/priority as diagnostic hints only

The `bucket_id`, `priority_level`, `rank_score`, and EAT scores are diagnostic hints for the reviewer. They do not constitute labels. The reviewer must listen to the audio and make an independent decision.

### 8.5 No label authority before human review

```text
manual_decision must be empty before human review.
label_authority must be false before human review.
machine_label must be empty.
candidate_label_allowed must be false.
training_allowed must be false.
```

## 9. Boundary rules

### 9.1 Hard authority rules for this planning task

```text
This planning document has no review package authority.
This planning document has no clip export authority.
This planning document has no label authority.
This planning document has no candidate label authority.
This planning document has no training authority.
This planning document has no F4/F5/F6 authority.
```

### 9.2 What this planning document CAN do

- Propose sampling strategy and quotas.
- Propose clip export format and naming convention.
- Propose review sheet schema.
- Propose anti-duplication and diversity rules.
- Document boundary rules for the future gate.

### 9.3 What this planning document CANNOT do

- Generate a review package.
- Export clips.
- Create a review sheet.
- Produce labels or candidate labels.
- Produce training data.
- Produce F4/F5/F6 artifacts.

### 9.4 EAT score boundary

```text
EAT score is not label authority.
EAT score is not candidate-label authority.
EAT score is not training authority.
EAT score is not F4/F5/F6 authority.
bucket_id and priority_level are not label authority.
source_prior_type is not label authority.
manual review is the only label authority.
```

## 10. Planning uncertainties

### 10.1 Per-source cap tuning

The proposed 20-clip per-source cap may be too high or too low. If review time is constrained, the cap may need to be reduced to 10. If source diversity is more important than depth, the cap may need to be lower. This should be validated at the review package gate.

### 10.2 Temporal spacing tuning

The proposed 5-second minimum spacing may be too tight for sources with clustered cough events, or too loose for sources with sparse events. A 10-second spacing (matching context duration) would eliminate all overlap but may miss adjacent events. This should be validated at the review package gate.

### 10.3 Review package size

The proposed 500–800 clip target assumes a single review session. If review capacity is larger, the package could be expanded to 1,000–1,500 clips with adjusted quotas. If review capacity is smaller, the package could be reduced to 200–300 clips by tightening P1 sampling.

### 10.4 B2/B5 empty impact

B2 and B5 are empty. If future policy revisions populate these buckets, the review package quotas may need adjustment. For now, no clips are selected from B2/B5.

### 10.5 P3 exploratory sampling

The proposed P3 exploratory quota (50 whole-night + 50 B6 controls) is a small fraction of the 86,692 P3 rows. The sampling fraction is ~0.1%. This is sufficient for calibration but may miss rare events. If false-negative risk is a concern, the exploratory quota could be increased.

### 10.6 Beep localization dependency

B4 playback rows are entirely excluded until beep localization is completed. If beep localization is delayed, the review package will lack playback/rerecord coverage. This is acceptable for the initial review package but should be resolved before a comprehensive review.

## 11. Proposed review package gate checklist

Before a review package may be generated, the following must be completed:

```text
1. This planning document passes Codex read-only review.
2. A separate explicit approval is granted for review package generation.
3. Clip export tooling is verified (read-only, no WAV modification).
4. Review sheet template is verified.
5. Per-source cap and temporal spacing are confirmed.
6. Random seed for P3 sampling is fixed and documented.
7. Review package output directory is approved.
8. Boundary rules are restated in the review package task.
```

## 12. Proposed output paths (future, not generated now)

```text
D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\review_package\clips\*.wav
D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\review_package\review_sheet.csv
D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\reports\review_package_report.md
```

These paths are proposals only. They are not created by this planning task.

## 13. Next gate proposal

After this planning document:

1. **Codex read-only review** of this planning design.
2. If PASS, a **separate explicit approval** is required before review package generation.
3. Only after approval may a **review package dry-run** (clip selection without export) or **actual clip export** be considered.
4. Clip export and review sheet generation remain blocked until the gate is passed.

```text
Current stage: review package planning design (this document)
Next gate: Codex read-only review of planning design
After gate: separate explicit approval for review package generation
Blocked: clip export, review sheet, labels, candidate labels, training, F4/F5/F6
```

## 14. Files inspected

- `docs/audio/v3_2_2_run003_eat_review_priority_policy_design.md`
- `docs/audio/v3_2_2_run003_eat_data_processing_closeout.md`
- `D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\reports\eat_review_priority_dryrun_report.md`
- `D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\review_priority\eat_review_priority_dryrun.csv` (read-only aggregate counts)

## 15. Commands run

```text
# Read-only aggregate counts from dryrun CSV (no files modified)
python -c "csv.DictReader + Counter for priority/bucket/source-prior cross-tabs"
```

No scoring, training, export, review generation, label generation, F4/F5/F6 generation, clip export, or firmware work was run.

## 16. Files changed

```text
docs/audio/v3_2_2_run003_review_package_planning_design.md (NEW)
```

No dry-run CSV, dry-run report, score CSV, context manifest, window manifest, source inventory, label file, config, contract, firmware file, source WAV, `ml/data/raw`, run001 artifact, or run002_preflight artifact was modified. No review package, clips, labels, candidate labels, training outputs, or F4/F5/F6 artifacts were generated.
