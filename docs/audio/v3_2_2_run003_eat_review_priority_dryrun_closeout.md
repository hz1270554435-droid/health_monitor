# MIC v3.2.2 run003 EAT review priority dry-run closeout / freeze

Date: 2026-06-04

## 1. Verdict

**run003 EAT review priority dry-run: FROZEN / PASS**

This closeout freezes the completed run003 EfficientAT review priority dry-run as a diagnostic ranking artifact. It does not grant label authority, review package authority, clip export authority, candidate-label authority, training authority, or formal screening package authority.

## 2. Codex read-only review result

PASS

## 3. Approved artifacts

| Artifact | Path | Status |
|---|---|---|
| Review priority dry-run CSV | `D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\review_priority\eat_review_priority_dryrun.csv` | 102,219 rows / frozen |
| Review priority dry-run report | `D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\reports\eat_review_priority_dryrun_report.md` | PASS / frozen |

These artifacts are frozen as diagnostic ranking evidence. They do not constitute labels, candidate labels, review packages, training data, or formal F4/F5/F6 artifacts.

## 4. Key validation counts

| Check | Expected | Actual | Pass |
|---|---|---|---|
| rows | 102,219 | 102,219 | **PASS** |
| unique context_id | 102,219 | 102,219 | **PASS** |
| duplicate context_id | 0 | 0 | **PASS** |
| missing required columns | 0 | 0 | **PASS** |
| invalid bucket values | 0 | 0 | **PASS** |
| invalid priority values | 0 | 0 | **PASS** |
| authority / permission boundary violations | 0 | 0 | **PASS** |
| B4 unknown beep-risk routed to P0 | 5,412 / 5,412 | 5,412 / 5,412 | **PASS** |

Additional boundary checks from the dry-run report:

| Check | Result | Pass |
|---|---|---|
| label_authority all false | 102,219 / 102,219 | **PASS** |
| manual_decision all empty | 102,219 / 102,219 | **PASS** |
| machine_label all empty | 102,219 / 102,219 | **PASS** |
| eligible_for_review_package all false | 102,219 / 102,219 | **PASS** |
| clip_export_allowed all false | 102,219 / 102,219 | **PASS** |
| candidate_label_allowed all false | 102,219 / 102,219 | **PASS** |
| training_allowed all false | 102,219 / 102,219 | **PASS** |
| diagnostic_only all true | 102,219 / 102,219 | **PASS** |

## 5. Bucket counts

| Bucket | Count | % |
|---|---:|---:|
| B1_high_priority_cough_candidate | 7,808 | 7.64% |
| B2_whole_night_possible_cough_event | 0 | 0.00% |
| B3_speech_laugh_throat_hard_negative | 16,782 | 16.42% |
| B4_mixed_playback_rerecord_diagnostic | 5,412 | 5.29% |
| B5_background_or_quiet_anomaly | 0 | 0.00% |
| B6_low_priority_likely_non_event | 72,174 | 70.61% |
| B7_ambiguous_or_conflict | 43 | 0.04% |
| **Total** | **102,219** | **100%** |

## 6. Priority counts

| Priority | Count | % |
|---|---:|---:|
| P0_blocked_or_excluded | 5,412 | 5.29% |
| P1_high_review_priority | 3,631 | 3.55% |
| P2_medium_review_priority | 6,484 | 6.34% |
| P3_low_review_priority | 86,692 | 84.81% |
| P4_diagnostic_sample_only | 0 | 0.00% |
| **Total** | **102,219** | **100%** |

## 7. Boundary statement

This dry-run has no label authority, no review package authority, no clip export authority, no candidate label authority, and no training authority.

The dry-run CSV and report are diagnostic ranking evidence only. They may be used to suggest review ordering, diagnostic grouping, and sampling strategy. They may not be used to produce labels, candidate labels, training splits, or F4/F5/F6 artifacts.

`bucket_id` and `priority_level` are not label authority. `source_prior_type` is not label authority. `filename_description` is not label authority. Manual review remains the only path to label authority.

## 8. Explicitly still forbidden

The following remain forbidden and require separate explicit gates:

- Review package generation
- Review clips / clip export
- Labels
- Candidate labels
- Training
- F4/F5/F6 formal artifacts
- YAMNet scoring
- Board/model scoring
- Firmware changes
- Config / contract changes
- ml/data/raw changes
- Source WAV changes
- run001 artifact changes
- run002_preflight artifact changes

## 9. Empty bucket notes

### B2 empty (0 rows)

B2 requires `cough_score >= within-whole-night-mixed p95 AND cough_score > speech_score AND cough_score > silence_score`. The whole_night_mixed cough_score p95 is 0.001858. Even at this level, cough_score never exceeds speech_score or silence_score in the whole_night_mixed pool. The highest cough_score is 0.022334, while speech_score at that row is 0.210568. This is explainable by policy/data: overnight recordings are dominated by speech/silence, and EAT cough scores remain subordinate. A future policy revision may relax B2 criteria.

### B5 empty (0 rows)

B5 requires `source_prior_type = background_or_quiet AND (cough_score >= global p90 OR top1 IN (Cough, Sneeze, Throat clearing) OR Cough in top5)`. The global cough p90 is 0.001725. The highest cough_score in background_or_quiet is 0.000359, well below p90. No background_or_quiet row has Cough/Sneeze/Throat clearing in top1 or top5. This is explainable by policy/data: background sources produce very low cough responses. A future policy revision may use a within-source-prior threshold instead of global p90.

### P4 empty (0 rows)

P4 was intended for B4 rows with unknown beep risk. However, per the mandatory boundary rule, all B4 rows with `context_beep_risk = unknown_until_beep_localized` are routed to P0_blocked_or_excluded. Since all 5,412 B4 rows have `context_beep_risk = unknown_until_beep_localized`, they all go to P0. No B4 rows remain for P4. This is explainable by policy/data: all mixed_playback_rerecord contexts in the current dataset have unknown beep risk.

## 10. Threshold reference

Per-source-prior percentiles used for bucket assignment in this dry-run:

| source_prior_type | cough_p90 | cough_p95 | speech_p90 | laughter_p90 | throat_p90 | music_p90 |
|---|---:|---:|---:|---:|---:|---:|
| `background_or_quiet` | 0.000236 | 0.000267 | 0.133010 | — | — | — |
| `high_conf_filename_cough` | 0.538227 | 0.588243 | 0.151146 | — | — | — |
| `mixed_playback_rerecord` | 0.446126 | 0.501716 | 0.714910 | — | — | 0.095196 |
| `speech_laugh_throat_hard_negative_pool` | 0.000528 | 0.001722 | 0.809462 | 0.000452 | 0.000730 | — |
| `whole_night_mixed` | 0.001249 | 0.001858 | 0.795140 | — | — | — |

Global cough p90: 0.001725

These are dry-run thresholds derived from observed data distributions. They are not final policy thresholds and may be revised before any future review package gate.

## 11. Upstream chain summary

| Stage | Artifact | Status |
|---|---|---|
| F0 source inventory | `source_inventory_run003.csv` | PASS / frozen |
| F1 full window manifest | `window_manifest_run003_f1_full.csv` | PASS / frozen |
| EAT context manifest | `efficientat_context_manifest_10s_full.csv` | PASS / frozen |
| EAT window-context map | `efficientat_window_context_map_10s_full.csv` | PASS / frozen |
| EAT full scores | `efficientat_10s_scores_full_wide.csv` | PASS / frozen |
| Policy design | `v3_2_2_run003_eat_review_priority_policy_design.md` | PASS / frozen |
| Data processing closeout | `v3_2_2_run003_eat_data_processing_closeout.md` | PASS / frozen |
| **Review priority dry-run** | `eat_review_priority_dryrun.csv` | **PASS / frozen** |
| **Dry-run report** | `eat_review_priority_dryrun_report.md` | **PASS / frozen** |
| **This closeout** | `v3_2_2_run003_eat_review_priority_dryrun_closeout.md` | **PASS / frozen** |

## 12. Current risk register

| Risk | Status | Mitigation |
|---|---|---|
| EAT scores could be misread as labels | Active | Keep authority flags false; state diagnostic-only boundary in every downstream report |
| Mixed playback / rerecord beep contamination | Active | Keep unknown beep-risk contexts blocked (P0) until beep localization exists |
| Whole-night mixed pool may contain unlabeled events | Active | Treat as unlabeled mining/eval pool, not non-cough label evidence |
| B2/B5 empty thresholds may miss events | Known | Documented; future policy revision may relax criteria |
| Review package wording drift | Active | Next stage is review package planning only, with separate gate |
| F4/F5/F6 contract gap | Active | Do not generate formal artifacts without separate schema/contract gate |

## 13. Next separately gated stage

**review package planning only**

Actual package generation, clip export, labels, candidate labels, and training remain blocked until another explicit gate is approved.

The review package planning stage may propose clip selection strategy, review sheet schema, and review workflow design. It may not generate clips, review sheets, labels, candidate labels, training outputs, or F4/F5/F6 artifacts.

## 14. Git status

### ml

```text
?? tools/build_run003_efficientat_context_manifest_10s.py
?? tools/build_run003_f1_window_manifest.py
?? tools/score_run003_efficientat_10s_bounded.py
?? tools/score_run003_efficientat_10s_full.py
```

No change from before. No ml tracked files were modified.

### firmware

```text
A  .claude/agents/learning-tutor-v4-md-export.md
A  .claude/skills/e84-md-learning-output/SKILL.md
 M mic_v3_2_recall_fix_data_collection_guide.md
```

No change from before. No firmware tracked files were modified.

## 15. New conversation migration summary

- E84 MIC v3.2.2 run003 EAT review priority dry-run is frozen/PASS.
- F0 source inventory: 263 WAVs.
- F1 full window manifest: 102,219 windows.
- 10s context full manifest: 102,219 contexts, centered-with-padding.
- Full EAT scoring: 102,219 rows, 102,219 ok, 0 errors.
- Run001 anomaly not reproduced.
- EAT full scores are diagnostic ranking signals only.
- Policy design PASS and Codex reviewed.
- Data processing closeout PASS and frozen.
- Review priority dry-run: 102,219 rows, 7 buckets, 5 priority levels.
- Bucket counts: B1=7,808, B2=0, B3=16,782, B4=5,412, B5=0, B6=72,174, B7=43.
- Priority counts: P0=5,412, P1=3,631, P2=6,484, P3=86,692, P4=0.
- All boundary validations PASS.
- Next stage: review package planning only (separately gated).
- Forbidden: review package, clips, labels, candidate labels, training, F4/F5/F6.

## 16. Commands run

No scoring, training, export, review generation, label generation, F4/F5/F6 generation, or firmware work was run for this closeout.

Read-only checks used:

- Dry-run report inspection for validation counts, bucket/priority tables, boundary confirmation, and forbidden artifact status.
- Git status checks for ml and firmware repositories.

## 17. Files inspected

- `docs/audio/v3_2_2_run003_eat_review_priority_policy_design.md`
- `docs/audio/v3_2_2_run003_eat_data_processing_closeout.md`
- `D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\reports\eat_review_priority_dryrun_report.md`
- `D:\cough_model_train\DATA\screening_runs\v3_2_2\run_003\review_priority\eat_review_priority_dryrun.csv` (row count and column check only)

## 18. Files changed

- Added `docs/audio/v3_2_2_run003_eat_review_priority_dryrun_closeout.md`.

No run003 data artifact, dry-run CSV, dry-run report, score CSV, context manifest, window manifest, source inventory, label file, config, contract, firmware file, source WAV, `ml/data/raw`, run001 artifact, or run002_preflight artifact was modified.
