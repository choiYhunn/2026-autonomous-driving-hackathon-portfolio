# Contribution boundary

## Evidence used

- Team08 final-report role table
- preliminary `files_fixed_gpt.zip`
- current `CYH_drive_3sec_lane_lost_straight.zip`
- merged `CYH_drive_PJH_autonomous_merged` manifest and source comments
- local driving and parking logs

## Included

The code in this directory isolates CYH-side runtime concerns evidenced by the inspected artifacts: asynchronous recording, lane-loss state handling, and mode-resource lifecycle coordination.

## Not included or claimed

- PJH lane fitting, dynamic offset, CTE, and Stanley-control implementation;
- the team lead's parking implementation;
- DPU/YOLO utilities and SKKU AutomationLab baseline files;
- firmware/bitstream/model assets or hardware-specific configuration.

The report attributes steering/control tuning, ADC feedback, jump filtering, and lane-loss recovery work to Choi Yonghun. Where the final merged source explicitly identifies a different source owner for the core algorithm, this extraction follows the narrower source-level attribution.

