"""Immediate feedback for the student's pure scheduling method. No ROS needed."""
from .mission import Mission


def check_exercise():
    errors = []

    def check(name, action):
        try:
            action()
        except Exception as error:
            errors.append(f'{name}: {type(error).__name__}: {error}')

    def expect(value, message):
        if not value:
            raise AssertionError(message)

    def sequence():
        mission = Mission(('A', 'B', 'C'))
        mission.start()
        for index, state, completed in ((1, 'RUNNING', 1), (2, 'RUNNING', 2), (2, 'COMPLETED', 3)):
            mission.finish('SUCCEEDED')
            expect((mission.index, mission.state, mission.completed) == (index, state, completed),
                   f'expected index={index}, state={state}, completed={completed}')
        mission.start()
        expect((mission.index, mission.completed, mission.state) == (0, 0, 'RUNNING'), 'restart must begin at A')

    def failure(status):
        mission = Mission(('A', 'B', 'C'))
        mission.start()
        mission.finish('SUCCEEDED')
        mission.finish(status)
        expect((mission.state, mission.index, mission.completed) == ('FAILED', 1, 1), 'failure at B must stop at B')
        expect('B' in mission.reason and status in mission.reason, 'failure reason needs point B and Nav2 result')

    def external_cancel():
        mission = Mission(('A', 'B', 'C'))
        mission.start()
        mission.finish('SUCCEEDED')
        mission.finish('CANCELED')
        expect((mission.state, mission.index, mission.completed) == ('CANCELED', 1, 1), 'canceled B must preserve completed A without advancing')

    def single():
        mission = Mission(('only',))
        mission.start()
        mission.finish('SUCCEEDED')
        expect((mission.state, mission.index, mission.completed) == ('COMPLETED', 0, 1), 'single point must complete')

    for name, action in (('A -> B -> C and restart', sequence),
                         ('ABORTED at B', lambda: failure('ABORTED')),
                         ('REJECTED at B', lambda: failure('REJECTED')),
                         ('external cancellation', external_cancel), ('one-point route', single)):
        check(name, action)
    return errors


def main():
    errors = check_exercise()
    if errors:
        print('EXERCISE NOT READY: edit leap_patrol/mission.py -> Mission.on_result')
        for error in errors:
            print('  FAIL:', error)
        print('Patrol start remains disabled. These are exercise checks, not Gazebo results.')
        return 1
    print('PASS: 5 scheduling exercises. Ready for ROS/Gazebo validation; navigation is not yet verified.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
