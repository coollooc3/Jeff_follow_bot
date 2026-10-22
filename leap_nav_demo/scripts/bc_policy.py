"""Versioned feature contract and NumPy inference; no PyTorch in ROS processes."""
import json
from pathlib import Path
import numpy as np

FEATURE_SCHEMA = 'laser36-finite36-goal2-yaw2-velocity2-path2-v1'
FEATURE_SIZE = 80
ACTION_SCALE = np.array([.22, .6], dtype=np.float64)
OUTPUT_TRANSFORMS = ('linear', 'sigmoid_tanh')


def guard_reason(sample):
    if not sample['candidate']:
        return ','.join(sample['reasons'])
    if min(sample['features']['scan_laser_frame']['sector_min_m']) < .20:
        return 'laser_proximity_below_0.20m'
    return None


def vectorize(features):
    """Whitelist inputs: absolute map pose and teacher outputs cannot enter the model."""
    scan = features['scan_laser_frame']
    minimum = np.asarray(scan['sector_min_m'], dtype=float)
    finite = np.asarray(scan['finite_ray_counts'], dtype=float)
    censored = np.asarray(scan['positive_infinite_ray_counts'], dtype=float)
    invalid = np.asarray(scan['invalid_ray_counts'], dtype=float)
    if (any(a.shape != (36,) for a in (minimum, finite, censored, invalid)) or
            not all(np.isfinite(a).all() for a in (minimum, finite, censored, invalid)) or
            any((a < 0).any() for a in (minimum, finite, censored, invalid)) or
            ((finite + censored + invalid) <= 0).any()):
        raise ValueError('invalid or unknown laser sectors')
    vector = np.concatenate([
        minimum / 3.5, finite / (finite + censored + invalid),
        np.asarray(features['goal_xy_base_m']) / 3.,
        features['goal_yaw_error_sin_cos'],
        np.asarray(features['odom_velocity']) / ACTION_SCALE,
        np.asarray(features['path_reference_xy_base_m']) / .35])
    if vector.shape != (FEATURE_SIZE,) or not np.isfinite(vector).all():
        raise ValueError('invalid feature vector')
    return vector


class Policy:
    def __init__(self, artifact):
        version = artifact['schema_version']
        if (version not in (1, 2) or artifact['feature_schema'] != FEATURE_SCHEMA or
                artifact['action_scale'] != ACTION_SCALE.tolist()):
            raise ValueError('incompatible policy schema')
        # Version 2 makes the output contract explicit; older readers reject it.
        if version == 1:
            if 'output_transform' in artifact:
                raise ValueError('output transform requires schema 2')
            self.output_transform = 'linear'
        else:
            self.output_transform = artifact.get('output_transform')
            if self.output_transform not in OUTPUT_TRANSFORMS:
                raise ValueError('invalid output transform')
        self.mean = np.asarray(artifact['mean'], dtype=float)
        self.std = np.asarray(artifact['std'], dtype=float)
        if (self.mean.shape != (FEATURE_SIZE,) or self.std.shape != (FEATURE_SIZE,) or
                not np.isfinite(self.mean).all() or not np.isfinite(self.std).all() or
                (self.std <= 0).any()):
            raise ValueError('invalid normalization')
        self.layers = []
        size = FEATURE_SIZE
        for layer in artifact['layers']:
            weight, bias = np.asarray(layer['weight'], dtype=float), np.asarray(layer['bias'], dtype=float)
            if (weight.ndim != 2 or weight.shape[1] != size or bias.shape != (weight.shape[0],) or
                    not np.isfinite(weight).all() or not np.isfinite(bias).all()):
                raise ValueError('invalid network layer')
            self.layers.append((weight, bias))
            size = weight.shape[0]
        if not self.layers or size != 2:
            raise ValueError('network must output two actions')

    @classmethod
    def load(cls, path):
        if Path(path).stat().st_size > 2_000_000:
            raise ValueError('oversized policy')
        return cls(json.loads(Path(path).read_text()))

    def predict(self, vector):
        value = (np.asarray(vector, dtype=float) - self.mean) / self.std
        if value.shape != (FEATURE_SIZE,) or not np.isfinite(value).all():
            raise ValueError('invalid inference input')
        for index, (weight, bias) in enumerate(self.layers):
            value = weight @ value + bias
            if index < len(self.layers) - 1:
                value = np.maximum(value, 0.)
        if not np.isfinite(value).all():
            raise ValueError('nonfinite action')
        if self.output_transform == 'sigmoid_tanh':
            # Stable sigmoid avoids overflow for large finite logits.
            value = np.array([.5 * (1. + np.tanh(.5 * value[0])), np.tanh(value[1])])
        raw = value * ACTION_SCALE
        if not np.isfinite(raw).all():
            raise ValueError('nonfinite action')
        bounded = np.clip(raw, [0., -.6], [.22, .6])
        return raw, bounded
