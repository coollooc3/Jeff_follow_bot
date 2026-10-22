"""Asynchronous goal ownership and cancellation, independent of ROS imports."""

RESULTS = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}


class PatrolClient:
    def __init__(self, mission, send_goal, changed=lambda: None):
        self.mission, self.send_goal, self.changed = mission, send_goal, changed
        self.active = self.handle = None
        self.cancel_sent = False
        self.last_nav_status = None

    def start(self):
        if self.active is not None:
            raise ValueError('Previous goal is unresolved; cancel it before restarting')
        self.mission.start()
        self.last_nav_status = None
        self._dispatch()

    def cancel(self):
        if self.active is None:
            return False
        if self.mission.state == 'FAILED':
            self.mission.state = 'RUNNING'  # Permit cancellation of uncertain transport state.
        self.mission.request_cancel()
        if self.handle is not None:
            self._cancel_handle(self.active)
        self.changed()
        return True

    def _fault(self, message):
        # Keep ownership until a terminal result proves that this goal ended.
        self.mission.fail(message)
        self.changed()

    def _dispatch(self):
        if self.mission.state != 'RUNNING':
            return
        token = self.active = object()
        self.handle, self.cancel_sent = None, False
        self.changed()
        try:
            future = self.send_goal(self.mission.index)
            future.add_done_callback(lambda reply: self._accepted(reply, token))
        except Exception as error:
            self._fault(f'send_error; goal state unknown: {error}')

    def _finish(self, status):
        self.active, self.handle, self.cancel_sent = None, None, False
        self.last_nav_status = status
        try:
            self.mission.finish(status)
        except Exception as error:
            self.mission.fail(f'scheduler_error: {error}')
        self.changed()
        if self.mission.state == 'RUNNING':
            self._dispatch()

    def _accepted(self, future, token):
        if token is not self.active:
            return
        try:
            handle = future.result()
        except Exception as error:
            self._fault(f'goal_response_error; goal state unknown: {error}')
            return
        if not handle.accepted:
            self._finish('REJECTED')
            return
        self.handle = handle
        try:
            handle.get_result_async().add_done_callback(lambda reply: self._result(reply, token))
        except Exception as error:
            self._fault(f'result_request_error: {error}')
            self._cancel_handle(token)
            return
        if token is self.active and self.mission.state != 'RUNNING':
            self._cancel_handle(token)
        self.changed()

    def _result(self, future, token):
        if token is not self.active:
            return
        try:
            status = RESULTS[future.result().status]
        except Exception as error:
            self._fault(f'result_error; goal state unknown: {error}')
            return
        self._finish(status)

    def _cancel_handle(self, token):
        if token is not self.active or self.handle is None or self.cancel_sent:
            return
        self.cancel_sent = True
        try:
            self.handle.cancel_goal_async().add_done_callback(lambda reply: self._cancel_reply(reply, token))
        except Exception as error:
            self.cancel_sent = False
            self._fault(f'cancel_request_error; goal unresolved: {error}')

    def _cancel_reply(self, future, token):
        if token is not self.active:
            return
        try:
            response = future.result()
            if response.return_code != 0:
                # A terminal result may already be queued. Never advance on cancel rejection.
                self.cancel_sent = False
                self._fault(f'cancel_rejected:{response.return_code}; waiting for terminal result')
        except Exception as error:
            self.cancel_sent = False
            self._fault(f'cancel_response_error; goal unresolved: {error}')
