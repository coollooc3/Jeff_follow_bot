"""Pure task state, independent of ROS and transport callbacks."""

BUSY = ('RUNNING', 'CANCELING')


class Mission:
    def __init__(self, names):
        self.names = tuple(names)
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError('Mission needs nonempty, unique waypoint names')
        self.state = 'IDLE'
        self.index = 0
        self.completed = 0
        self.reason = ''

    @property
    def current_name(self):
        return self.names[self.index]

    def start(self):
        if self.state in BUSY:
            raise ValueError('Patrol is already running or canceling')
        self.index, self.completed = 0, 0
        self.state, self.reason = 'RUNNING', ''

    def request_cancel(self):
        if self.state == 'RUNNING':
            self.state, self.reason = 'CANCELING', 'operator_cancel'

    def finish(self, status):
        # An operator cancellation wins even if success arrived at the same time.
        if self.state == 'CANCELING':
            self.state, self.reason = 'CANCELED', 'operator_cancel'
        elif self.state == 'RUNNING':
            self.on_result(status)

    def fail(self, reason):
        self.state, self.reason = 'FAILED', reason

    def on_result(self, status):
        """Update task progress from a final Nav2 result, not goal acceptance."""
        if status == 'SUCCEEDED':
            self.completed += 1
            if self.index < len(self.names) - 1:
                self.index += 1
            else:
                self.state = 'COMPLETED'
        elif status in ['ABORTED', 'REJECTED']:
            self.state = 'FAILED'
            self.reason = f'Failed at {self.current_name} with status {status}'
        elif status == 'CANCELED':
            self.state = 'CANCELED'
