#!/usr/bin/env python3
"""Static regression for the read-only APM80 release RPM experiment."""
from pathlib import Path

src = Path("L1AC_customization/ArduCopter/mode_adaptive.cpp").read_text()
workflow = Path(".github/workflows/build-mode29-apm80-rpm-bdshot.yml").read_text()
doc = Path("docs/MODE29_APM80_BDSHOT_RPM_20261012.md").read_text()
assert "AP::esc_telem().get_raw_rpm(aux, raw_rpm)" in src
assert "uint8_t valid_mask = 0U" in src
assert 'AP::logger().Write("L1RP", "Ms,R1,R2,R3,R4,Val", "IffffB"' in src
assert "last_rpm_log_ms >= 50U" in src
assert "mode29_bounded_allocate" in src
assert "L1AdaptiveAugmentation" in src
assert "motorMixingYawFreeEffectivenessAware" in src
assert "motor_fdi_confirmed_at_ms" in src
assert "motor_fdi_applied_delta_pct" in src
assert "motors->rc_write(0," in src and "motors->rc_write(3," in src
assert "Pixhawk6C-bdshot" in workflow
assert "test_fdi_hil_antiwindup.py" in workflow
assert "f3f12d901e82c779cd842246556091ba21492838" in workflow
assert "f3f12d901e82c779cd842246556091ba21492838" in doc
assert "NOT FLIGHT" in workflow
assert "SERVO9" in doc and "SERVO12" in doc
print("PASS: exact APM80 anti-windup source retained; read-only FMU RPM logger added.")
