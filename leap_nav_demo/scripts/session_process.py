"""Isolation and owned process cleanup shared by simulation entry points."""
import os
import signal
import socket
import subprocess
import time

def live_group_members(group_id):
    rows = subprocess.check_output(['ps', '-eo', 'pgid=,stat=,pid='], text=True).splitlines()
    return [int(pid) for group, state, pid in (row.split() for row in rows)
            if int(group) == group_id and not state.startswith('Z')]


def stop(process, grace_periods=(10, 5, 5), interrupt_group=False):
    if process is None:
        return {'signals_sent': [], 'survivors': []}
    report = {'group_id': process.pid, 'signals_sent': [], 'survivors': []}
    # Only signal the process group created by this invocation.
    for sig, grace in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), grace_periods):
        process.poll()
        if not live_group_members(process.pid):
            break
        try:
            if sig == signal.SIGINT and not interrupt_group and process.poll() is None:
                process.send_signal(sig)  # Let ros2 launch signal its own children first.
            else:
                os.killpg(process.pid, sig)
            report['signals_sent'].append(sig.name)
        except ProcessLookupError:
            break
        deadline = time.monotonic() + grace
        while time.monotonic() < deadline and live_group_members(process.pid):
            process.poll()
            time.sleep(0.1)
    process.poll()
    report['survivors'] = live_group_members(process.pid)
    report['exit_code'] = process.returncode
    return report



def check_empty_simulation(parser):
    with socket.socket() as sock:
        if sock.connect_ex(('127.0.0.1', 11365)) == 0:
            parser.error('simulation port 11365 is busy; stop the existing simulation first')
    # A dead Gazebo can leave Nav2 alive; a free TCP port alone is insufficient.
    import rclpy
    rclpy.init()
    guard = rclpy.create_node('leap_nav_session_guard')
    try:
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            rclpy.spin_once(guard, timeout_sec=0.1)
        existing = sorted({name for name, _ in guard.get_node_names_and_namespaces()
                           if name != guard.get_name() and not name.startswith('_ros2cli')})
        if existing:
            parser.error('simulation domain already has nodes: ' + ', '.join(existing))
    finally:
        guard.destroy_node()
        rclpy.shutdown()
