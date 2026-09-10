"""Reproducible static inventory, not a claim of semantic formula extraction."""
import hashlib
import json
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASELINE = '2f2ecb9'
GROUPS = {
    'brake': 'brake_fade brake_lock brake_system',
    'tire': 'contact_quality slip_recovery tire_carcass tire_compliance tire_contact tire_contact_core tire_contact_response tire_dynamics tire_force tire_hop tire_memory tire_state tire_thermal_brush',
    'suspension': 'arm_compliance compliance_stack control_arm damper_hysteresis damper_model progressive_spring sprung_mass suspension suspension_contact_input',
    'chassis': 'body_rigidity_estimator chassis_energy chassis_flex chassis_roll mass_balance road_body_input ultra_chassis virtual_inertia yaw_moment_budget',
    'load_transfer': 'load_path load_transfer road_input_interpreter weight_distribution',
    'drivetrain': 'diff_lsd driveline_windup drivetrain',
    'steering': 'caster_effect steering_dynamics steering_mechanism',
    'thermal': 'thermal',
    'damage': 'damage_event damage_state impact_sensor vehicle_condition',
    'observation': 'diff_lsd_observer observer wheel_audit',
    'shared': 'ngp_core',
    'result_aggregation': 'physics',
}

def inventory():
    host = (ROOT / 'Legacy/ACNextGen.lua').read_text()
    entries = re.findall(r'\{ name = "(\w+)",\s+enabled = (true|false),\s+critical = (true|false),\s+diagnostic = (true|false)', host)
    hz = dict(re.findall(r'^    (\w+) = (\d+),$', host, re.M))
    schedule = [dict(module_id=n, host_index=i, enabled=e == 'true', critical=c == 'true', diagnostic=d == 'true', target_hz=int(hz[n])) for i, (n, e, c, d) in enumerate(entries, 1)]
    categories = {n: g for g, names in GROUPS.items() for n in names.split()}
    modules = []
    for path in sorted((ROOT / 'modules').glob('*.lua')):
        raw = path.read_bytes()
        text = raw.decode()
        name = path.stem
        modules.append(dict(
            module_id=name, source=str(path.relative_to(ROOT)), sha256=hashlib.sha256(raw).hexdigest(),
            lines=len(text.splitlines()), category=categories[name],
            status='PILOT_SPECIFICATION' if name == 'brake_fade' else 'PENDING_SEMANTIC_EXTRACTION',
            scheduled=any(e['module_id'] == name for e in schedule),
            functions=re.findall(r'^(?:local )?function ([\w.:]+)\(', text, re.M),
            ac_api_candidates=sorted(set(re.findall(r'\bac\.(\w+)', text))),
            io_api_candidates=sorted(set(re.findall(r'\bio\.(\w+)', text))),
            store_key_literal_candidates=sorted(set(re.findall(r'["\'](ngp_[\w]+)["\']', text))),
            state_field_candidates=sorted(set(re.findall(r'\bstate\.(\w+)', text))),
            review_note='Candidates include reads, writes, aliases, dynamic prefixes and comments; not resolved dependencies or complete formulas.'
        ))
    return dict(version=1, baseline_commit=BASELINE, source_count=len(modules), modules=modules,
                execution_order=schedule,
                missing_scheduled_sources=[e['module_id'] for e in schedule if not (ROOT / 'modules' / (e['module_id'] + '.lua')).exists()],
                boundaries={
                    'Car_Engine': {'status': 'PENDING_EXTRACTION', 'source': 'modules/drivetrain.lua',
                        'functions': ['interpolatePowerLut', 'estimateFallbackTorqueCurve', 'calculateEngineTorqueNm', 'calculateEngineTorqueNorm'],
                        'note': 'Keep callers, filtered RPM, LUT/car-profile lifecycle, shared parameters and torque smoothing in original order until differential validation. No prototype engine models imported.'},
                    'Send': 'physics.lua includes load normalization, sums and decay: calculation belongs in Engine, not Send. Current API is ac.store telemetry, not force application.',
                    'feedback': 'Later writers are sampled from previous/latest available values. Never topologically reorder the original multi-rate host.'})

if __name__ == '__main__':
    result = json.dumps(inventory(), indent=2, ensure_ascii=False) + '\n'
    target = ROOT / 'Migration_Catalog.json'
    import sys
    if '--check' in sys.argv:
        assert target.read_text() == result, 'Inventory differs: run python tools/inventory.py'
        print('Inventory matches all source hashes and baseline schedule')
    else:
        target.write_text(result)
