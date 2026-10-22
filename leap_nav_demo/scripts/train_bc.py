#!/usr/bin/env python3
"""Small CPU behavior-cloning experiment with task-held-out evaluation."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import numpy as np
import torch
from torch import nn
from bc_dataset import load_dataset
from bc_policy import ACTION_SCALE, FEATURE_SCHEMA, FEATURE_SIZE, OUTPUT_TRANSFORMS, Policy


def output_actions(model, x, transform):
    logits = model(x)
    if transform == 'linear':
        return logits
    if transform == 'sigmoid_tanh':
        return torch.stack((torch.sigmoid(logits[:, 0]), torch.tanh(logits[:, 1])), dim=1)
    raise ValueError('invalid output transform')


def train(manifest, directory, epochs=400, output_transform='linear'):
    if output_transform not in OUTPUT_TRANSFORMS:
        raise ValueError('invalid output transform')
    torch.set_num_threads(2)
    torch.manual_seed(42)
    np.random.seed(42)
    data, audit = load_dataset(manifest)
    directory.mkdir(parents=True, exist_ok=False)
    train_x, train_y = data['train']
    mean = train_x.mean(axis=0)
    std = np.maximum(train_x.std(axis=0), .05)
    tensors = {key: (torch.tensor((x - mean) / std, dtype=torch.float32),
                      torch.tensor(y / ACTION_SCALE, dtype=torch.float32)) for key, (x, y) in data.items()}
    model = nn.Sequential(nn.Linear(FEATURE_SIZE, 64), nn.ReLU(), nn.Linear(64, 32),
                          nn.ReLU(), nn.Linear(32, 2))
    optimizer = torch.optim.Adam(model.parameters(), lr=.001, weight_decay=.0001)
    history, best, best_state, best_epoch = [], float('inf'), None, None
    for epoch in range(epochs):
        model.train()
        order = torch.randperm(len(train_x))
        for indices in order.split(128):
            x, y = tensors['train']
            loss = nn.functional.mse_loss(output_actions(model, x[indices], output_transform), y[indices])
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()
        model.eval()
        with torch.no_grad():
            losses = {key: float(nn.functional.mse_loss(output_actions(model, x, output_transform), y))
                      for key, (x, y) in tensors.items() if key != 'test'}
        history.append({'epoch': epoch + 1, **losses})
        if losses['validation'] < best:
            best, best_state, best_epoch = losses['validation'], copy.deepcopy(model.state_dict()), epoch + 1
    model.load_state_dict(best_state)
    model.eval()
    # .pt is an archival training artifact. Runtime loads only validated JSON arrays.
    torch.save({'state_dict': best_state, 'mean': torch.tensor(mean), 'std': torch.tensor(std),
                'feature_schema': FEATURE_SCHEMA, 'epoch': best_epoch, 'schema_version': 2,
                'output_transform': output_transform}, directory / 'checkpoint.pt')
    loaded = torch.load(directory / 'checkpoint.pt', map_location='cpu', weights_only=True)
    model.load_state_dict(loaded['state_dict'])
    if loaded['output_transform'] != output_transform:
        raise ValueError('checkpoint output contract mismatch')
    artifact = {'schema_version': 2, 'output_transform': loaded['output_transform'],
                'feature_schema': FEATURE_SCHEMA, 'mean': mean.tolist(),
                'std': std.tolist(), 'action_scale': ACTION_SCALE.tolist(),
                'layers': [{'weight': layer.weight.detach().tolist(), 'bias': layer.bias.detach().tolist()}
                           for layer in model if isinstance(layer, nn.Linear)]}
    (directory / 'policy.json').write_text(json.dumps(artifact, allow_nan=False) + '\n')
    policy = Policy.load(directory / 'policy.json')
    scores, parity = {}, 0.
    baseline = train_y.mean(axis=0)
    with torch.no_grad():
        for key, (x, y) in data.items():
            torch_raw = output_actions(model, tensors[key][0], output_transform).numpy() * ACTION_SCALE
            numpy_raw = np.asarray([policy.predict(row)[0] for row in x])
            parity = max(parity, float(np.max(np.abs(torch_raw - numpy_raw))))
            bounded = np.asarray([policy.predict(row)[1] for row in x])
            scores[key] = {'samples': len(x), 'raw_mae_v_w': np.abs(numpy_raw - y).mean(axis=0).tolist(),
                           'bounded_mae_v_w': np.abs(bounded - y).mean(axis=0).tolist(),
                           'train_mean_baseline_mae_v_w': np.abs(baseline - y).mean(axis=0).tolist(),
                           'clipped_samples': int(np.any(np.abs(bounded - numpy_raw) > 1e-10, axis=1).sum())}
    if parity > 1e-6:
        raise ValueError('Torch/NumPy inference mismatch')
    report = {'seed': 42, 'torch_version': torch.__version__, 'architecture': [80, 64, 32, 2],
              'output_transform': output_transform,
              'epochs': epochs, 'best_validation_epoch': best_epoch, 'scores': scores,
              'torch_numpy_max_abs_difference': parity, 'episodes': audit,
              'manifest_sha256': hashlib.sha256(Path(manifest).read_bytes()).hexdigest(),
              'policy_sha256': hashlib.sha256((directory / 'policy.json').read_bytes()).hexdigest(),
              'limits': 'One known map/corridor, whole-task split; offline MAE is not navigation success. '
                        'Global path input retained; receipt-time alignment approximate; no real-car validation.'}
    (directory / 'training.json').write_text(json.dumps(report, indent=2) + '\n')
    (directory / 'loss_history.json').write_text(json.dumps(history, indent=2) + '\n')
    print(json.dumps(report), flush=True)
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--epochs', type=int, default=400)
    parser.add_argument('--output-transform', choices=OUTPUT_TRANSFORMS, default='linear')
    args = parser.parse_args()
    if not 1 <= args.epochs <= 2000:
        parser.error('epochs must be 1..2000')
    train(args.manifest, args.output, args.epochs, args.output_transform)
