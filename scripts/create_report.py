#!/usr/bin/env python3
"""Create a local HTML evidence report from a measured verification run."""
import html
import json
from pathlib import Path


def main():
    project = Path(__file__).resolve().parents[1]
    artifacts = project / 'artifacts'
    measured = json.loads((artifacts / 'verification.json').read_text())
    passed = measured.get('passed', False)
    keys = [
        ('نتیجهٔ بررسی حرکت', 'PASS' if passed else 'FAIL'),
        ('پیام‌های معتبر لیدار', measured.get('valid_scan_messages', 0)),
        ('نمونه‌های اودومتری', measured.get('odometry_samples', 0)),
        ('پیشروی زمان شبیه‌سازی (ثانیه)', measured.get('clock_advanced_seconds', 0)),
        ('مسافت پیموده‌شده (متر)', measured.get('odometry_path_metres', '—')),
        ('خطای موقعیت نهایی (متر)', measured.get('goal_error_metres', '—')),
        ('زمان حرکت به هدف (ثانیهٔ واقعی)', measured.get('navigation_wall_seconds', '—')),
        ('وضعیت action؛ عدد ۴ یعنی موفق', measured.get('action_status', '—')),
    ]
    rows = ''.join(f'<tr><td>{html.escape(label)}</td><td dir="ltr">{html.escape(str(value))}</td></tr>' for label, value in keys)
    states = ''.join(f'<li><code>{html.escape(name)}</code>: {html.escape(state)}</li>' for name, state in measured.get('lifecycle_states', {}).items())
    images = ''.join(f'<figure><img src="{name}.png" alt="{name}"><figcaption>{label} — تصویر واقعی از اجرای WSLg</figcaption></figure>' for name, label in [('gazebo', 'Gazebo'), ('rviz', 'RViz')] if (artifacts / f'{name}.png').exists())
    error = f'<p class="error">{html.escape(measured["error"])}</p>' if measured.get('error') else ''
    page = f'''<!doctype html>
<html lang="fa" dir="rtl"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>گزارش راه‌اندازی ROS 2</title><style>
body{{background:#eef2f6;color:#172332;font-family:Tahoma,Arial,sans-serif;line-height:1.9;margin:0}}main{{max-width:1120px;margin:40px auto;padding:28px;background:white;border-radius:18px}}h1{{font-size:25px}}.badge{{display:inline-block;background:{'#d5f5e4' if passed else '#ffe1e1'};padding:8px 20px;border-radius:9px;font-weight:bold}}table{{border-collapse:collapse;width:100%;margin:24px 0}}td{{padding:12px;border-bottom:1px solid #e3e7ef}}td:last-child{{font-family:monospace;font-size:17px;text-align:left}}figure{{margin:24px 0}}img,video{{max-width:100%;border:1px solid #ddd;border-radius:10px}}figcaption{{color:#526274}}code{{direction:ltr;display:inline-block}}.error{{color:#a71919}}
</style><main><p dir="ltr">ros2-heterogeneous-edge-warehouse-scheduling</p>
<h1>گزارش راه‌اندازی و بررسی حرکت ربات</h1><span class="badge">{'حرکت به هدف تأیید شد' if passed else 'بررسی ناموفق بود'}</span>
<p>ROS 2 Jazzy · Nav2 · Gazebo Harmonic · Ubuntu 24.04 · WSL2</p>
<p>این گزارش مربوط به نمونهٔ رسمی تک‌رباتی است. انبار، سرورهای edge، چند ربات و زمان‌بند MC-DAG در مراحل بعد اضافه می‌شوند.</p>
{error}<table>{rows}</table><h2>وضعیت گره‌های Nav2</h2><ul>{states}</ul>{images}
<p>دادهٔ خام: <a href="verification.json">verification.json</a></p>
</main></html>'''
    (artifacts / 'setup-report.html').write_text(page, encoding='utf-8')
    print(artifacts / 'setup-report.html')


if __name__ == '__main__':
    main()
