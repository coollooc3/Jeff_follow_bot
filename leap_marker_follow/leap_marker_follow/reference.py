"""A taught image size, not a calibrated metric depth estimate."""
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import tempfile


@dataclass(frozen=True)
class Reference:
    marker_id: int
    width: int
    height: int
    side: float
    nominal_front_gap_cm: float = 25.

    def __post_init__(self):
        if (type(self.marker_id) is not int or not 0 <= self.marker_id < 50
                or type(self.width) is not int or type(self.height) is not int
                or min(self.width, self.height) <= 0
                or not math.isfinite(self.side) or not 0 < self.side < 1
                or self.nominal_front_gap_cm != 25.):
            raise ValueError('Invalid reference; teach the marker again')

    @classmethod
    def load(cls, path):
        return cls(**json.loads(Path(path).expanduser().read_text(encoding='utf-8')))

    def save(self, path):
        path = Path(path).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8',
                                             dir=path.parent, delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(asdict(self), stream, indent=2, allow_nan=False)
                stream.write('\n')
            temporary.replace(path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)


def teach(samples, now, marker_id):
    """Samples: (arrival time, normalized side, offset, width, height)."""
    recent = [s for s in samples if 0 <= now-s[0] <= .8]
    if len(recent) < 5 or now-recent[-1][0] > .2:
        raise ValueError('Keep a centered marker still and visible for one second')
    sizes = [s[1] for s in recent]
    if (any(abs(s[2]) > .10 or s[3:] != recent[-1][3:] for s in recent)
            or max(sizes)/min(sizes) > 1.08):
        raise ValueError('Reference needs a stationary, centered marker and fixed resolution')
    return Reference(marker_id, recent[-1][3], recent[-1][4], sum(sizes)/len(sizes))
