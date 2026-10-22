"""A complete, runnable toy: wait for acceptance AND the final result at each point."""
from collections import deque
from concurrent.futures import Future


class ToyExecutor:
    def __init__(self):
        self.pending = deque()

    def spin_once(self):
        if self.pending:
            self.pending.popleft()()


class ToyGoalHandle:
    accepted = True

    def __init__(self, executor, name):
        self.executor, self.name = executor, name

    def get_result_async(self):
        future = Future()
        self.executor.pending.append(lambda: future.set_result('SUCCEEDED'))
        return future


def send_goal_async(executor, name):
    future = Future()
    executor.pending.append(lambda: future.set_result(ToyGoalHandle(executor, name)))
    return future


def wait(executor, future):
    while not future.done():
        executor.spin_once()
    return future.result()


def main():
    executor = ToyExecutor()
    print('Toy example only: no ROS, Gazebo, or motion commands. Read the numbered stages in order.')
    for name in ('example-A', 'example-B', 'example-C'):
        print(f'1. send {name}')
        response_future = send_goal_async(executor, name)
        handle = wait(executor, response_future)
        print(f'2. {name} accepted={handle.accepted}; this is NOT arrival')
        if not handle.accepted:
            print('Goal rejected; end the route')
            return 1
        result_future = handle.get_result_async()
        result = wait(executor, result_future)
        print(f'3. {name} final result={result}; only now may the next point start')
        if result != 'SUCCEEDED':
            return 1
    print('4. All three example points finished.')
    print('Real node: callbacks replace this waiting loop; your on_result method decides the next state.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
