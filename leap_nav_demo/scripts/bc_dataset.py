"""Load explicitly assigned whole-task partitions, preserving excluded attempts."""
import hashlib
import json
from pathlib import Path
import numpy as np
from bc_policy import vectorize
from task_config import read_task, task_hash


def load_dataset(manifest_path):
    manifest_path = Path(manifest_path)
    manifest = json.loads(manifest_path.read_text())
    if manifest['schema_version'] != 1:
        raise ValueError('unsupported manifest')
    splits = {key: [[], []] for key in ('train', 'validation', 'test')}
    task_splits, seen_paths, audit = {}, set(), []
    for entry in manifest['episodes']:
        split = entry['split']
        if split not in splits:
            raise ValueError('unknown split')
        directory = (manifest_path.parent / entry['path']).resolve()
        if directory in seen_paths:
            raise ValueError('duplicate episode')
        seen_paths.add(directory)
        task = read_task(directory / 'task_config_used.json', 'baseline')
        identity = task_hash(task)
        if identity in task_splits and task_splits[identity] != split:
            raise ValueError('task identity leaks across partitions')
        task_splits[identity] = split
        report = json.loads((directory / 'result.json').read_text())
        session = json.loads((directory / 'session.json').read_text())
        summary = json.loads((directory / 'expert_summary.json').read_text())
        metadata = json.loads((directory / 'expert_metadata.json').read_text())
        accepted = (metadata['schema_version'] == 2 and summary['episode_candidate'] and
                    report['ok'] and session['ok'] and report['result'] == 'SUCCEEDED' and
                    report.get('recoveries_reported_by_nav2') == 0 and
                    report['task_config_sha256'] == session['task_config_sha256'] == identity)
        files = ('expert_samples.jsonl', 'task_config_used.json', 'expert_metadata.json',
                 'expert_summary.json', 'result.json', 'session.json')
        row = {'path': str(directory), 'split': split, 'task_sha256': identity,
               'accepted': bool(accepted), 'samples': 0,
               'task_result': report['result'], 'task_ok': report['ok'], 'session_ok': session['ok'],
               'recoveries': report.get('recoveries_reported_by_nav2'),
               'sha256': {name: hashlib.sha256((directory / name).read_bytes()).hexdigest() for name in files}}
        if accepted:
            for line in (directory / 'expert_samples.jsonl').read_text().splitlines():
                sample = json.loads(line)
                if not sample['candidate']:
                    continue
                if sample['reasons'] or not sample['task_active'] or not sample['follow_path_active']:
                    raise ValueError('inconsistent candidate flags')
                for kind in ('scan', 'odom', 'plan'):
                    ages = sample['input_ages'][kind]
                    if not (0 <= ages['source_sim_s'] <= metadata['source_limits_sim_s'][kind] and
                            0 <= ages['receive_wall_s'] <= metadata['receive_limits_wall_s'][kind]):
                        raise ValueError('candidate has future/stale observation')
                x = vectorize(sample['features'])
                y = np.asarray(sample['action']['velocity'], dtype=float)
                if y.shape != (2,) or not np.isfinite(y).all():
                    raise ValueError('invalid teacher action')
                splits[split][0].append(x)
                splits[split][1].append(y)
                row['samples'] += 1
        audit.append(row)
    arrays = {key: (np.asarray(x, dtype=np.float32), np.asarray(y, dtype=np.float32))
              for key, (x, y) in splits.items()}
    if any(len(x) < 20 for x, _ in arrays.values()):
        counts = {key: len(x) for key, (x, _) in arrays.items()}
        raise ValueError('each partition needs at least 20 eligible samples; ' + json.dumps(counts))
    return arrays, audit
