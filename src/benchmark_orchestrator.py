#!/usr/bin/env python3
"""
Benchmark orchestrator for the Hybrid MPPI Tensegrity Simulation System.

Launches the full pipeline (simulator + tracker + planner + controller) N times,
monitors the planner for goal detection, and computes success rate and timing stats.

Requires: ROS environment activated and catkin workspace sourced (handled by benchmark.sh).

Usage (via benchmark.sh):
    ./benchmark.sh --trials 5 --time-limit 120 --no-viz
"""

import argparse
import dataclasses
import datetime
import json
import os
import signal
import socket
import subprocess
import sys
import threading
import time


@dataclasses.dataclass
class ProcessSpec:
    """Specification for a subprocess to launch."""
    name: str
    cmd: list
    startup_delay: float
    monitor_stderr: bool = False


@dataclasses.dataclass
class TrialResult:
    """Result of a single benchmark trial."""
    trial_number: int
    goal_reached: bool
    elapsed_time: float
    reason: str  # "goal_reached", "timeout", or "error"
    error_message: str = ""


class BenchmarkOrchestrator:
    """Orchestrates N benchmark trials of the tensegrity hybrid MPPI system."""

    def __init__(self, args):
        self.args = args
        self.script_dir = os.path.normpath(
            os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
        )
        self.results = []
        self.active_processes = {}
        self.drain_threads = []
        self.monitor_thread = None
        self.goal_event = threading.Event()

        # Resolve XML model path
        if args.xml_model is None:
            self.xml_model = os.path.join(
                self.script_dir,
                'src/mujoco_simulator/xml_models/3bar_new_platform_all_cables.xml'
            )
        else:
            self.xml_model = args.xml_model

        if not os.path.exists(self.xml_model):
            print(f"Error: XML model not found: {self.xml_model}")
            sys.exit(1)

        # Build process specs
        self.process_specs = self._build_process_specs()

    def _build_process_specs(self):
        """Build the list of processes to launch per trial."""
        sim_cmd = [
            sys.executable,
            os.path.join(self.script_dir, 'src/mujoco_simulator/tensegrity_udp_simulator.py'),
            self.xml_model,
            '--physics-rate', str(self.args.physics_rate),
            '--sensor-rate', str(self.args.sensor_rate),
        ]
        if self.args.no_viz:
            sim_cmd.append('--no-viz')
        if self.args.sensor_noise:
            sim_cmd.append('--sensor-noise')
        if self.args.remote_viewer:
            sim_cmd.extend(['--remote-viewer', '--remote-port', str(self.args.remote_port)])

        tracker_cmd = [
            sys.executable,
            os.path.join(self.script_dir, 'src/perception/scripts/mock_tracking_service.py'),
        ]

        planner_cmd = [
            sys.executable,
            os.path.join(self.script_dir, 'src/mppi_planner.py'),
        ]

        controller_cmd = [
            sys.executable,
            os.path.join(self.script_dir, 'src/run_tensegrity_hybrid_mppi_simulator.py'),
        ]

        return [
            ProcessSpec(name='Simulator', cmd=sim_cmd, startup_delay=2.0),
            ProcessSpec(name='Tracker', cmd=tracker_cmd, startup_delay=1.0),
            ProcessSpec(name='Controller', cmd=controller_cmd, startup_delay=1.0),
            ProcessSpec(name='Planner', cmd=planner_cmd, startup_delay=1.0, monitor_stderr=True),
        ]

    def run_all_trials(self):
        """Run all benchmark trials and print summary."""
        print('=' * 62)
        print('TENSEGRITY HYBRID MPPI BENCHMARK')
        print('=' * 62)
        print(f'  Trials:        {self.args.trials}')
        print(f'  Time limit:    {self.args.time_limit}s per trial')
        print(f'  Startup delay: {self.args.startup_delay}s')
        print(f'  XML model:     {os.path.basename(self.xml_model)}')
        print(f'  Visualization: {"on" if not self.args.no_viz else "off"}')
        print(f'  Sensor noise:  {"on" if self.args.sensor_noise else "off"}')
        if self.args.remote_viewer:
            print(f'  Remote viewer: http://localhost:{self.args.remote_port}')
        print(f'  Output file:   {self.args.output}')
        print('=' * 62)
        print()

        try:
            for trial_num in range(1, self.args.trials + 1):
                print(f'--- Trial {trial_num}/{self.args.trials} ---')
                result = self.run_single_trial(trial_num)
                self.results.append(result)

                status = 'SUCCESS' if result.goal_reached else 'FAIL'
                print(f'  Result: {status} | Time: {result.elapsed_time:.2f}s | Reason: {result.reason}')
                if result.error_message:
                    print(f'  Error: {result.error_message}')
                print()

                # Cleanup between trials (skip after last trial)
                if trial_num < self.args.trials:
                    print('  Cleaning up for next trial...')
                    self.cleanup_between_trials()
                    print()

        except KeyboardInterrupt:
            print('\n\nBenchmark interrupted by user.')
            self.terminate_all_processes()

        self.print_summary()
        self.save_results()

    def run_single_trial(self, trial_number):
        """Run a single benchmark trial."""
        self.goal_event.clear()
        self.active_processes.clear()
        self.drain_threads.clear()
        self.monitor_thread = None
        self.stderr_buffers = {}  # name -> list of lines for error capture

        # Launch processes sequentially
        for spec in self.process_specs:
            try:
                proc = subprocess.Popen(
                    spec.cmd,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    cwd=self.script_dir,
                )
            except Exception as e:
                self.terminate_all_processes()
                return TrialResult(
                    trial_number=trial_number,
                    goal_reached=False,
                    elapsed_time=0.0,
                    reason='error',
                    error_message=f'Failed to launch {spec.name}: {e}',
                )

            self.active_processes[spec.name] = proc

            # Set up output monitoring threads
            if spec.monitor_stderr:
                self.monitor_thread = threading.Thread(
                    target=self._stderr_monitor_thread,
                    args=(proc,),
                    daemon=True,
                )
                self.monitor_thread.start()
            else:
                self.stderr_buffers[spec.name] = []
                t = threading.Thread(
                    target=self._drain_thread,
                    args=(proc.stderr, spec.name, True),
                    daemon=True,
                )
                t.start()
                self.drain_threads.append(t)

            # Drain stdout for all processes (no stderr buffer)
            t = threading.Thread(
                target=self._drain_thread,
                args=(proc.stdout, f'{spec.name}-stdout', False),
                daemon=True,
            )
            t.start()
            self.drain_threads.append(t)

            # Wait before launching next process
            if spec.startup_delay > 0:
                time.sleep(spec.startup_delay)

            # Verify process is still alive
            if proc.poll() is not None:
                stderr_out = ''
                buf = self.stderr_buffers.get(spec.name, [])
                if buf:
                    stderr_out = ''.join(buf)[-1000:].strip()
                if not stderr_out:
                    try:
                        stderr_out = proc.stderr.read().decode('utf-8', errors='replace')[-500:]
                    except Exception:
                        pass
                self.terminate_all_processes()
                return TrialResult(
                    trial_number=trial_number,
                    goal_reached=False,
                    elapsed_time=0.0,
                    reason='error',
                    error_message=f'{spec.name} died during startup (exit={proc.returncode}): {stderr_out}',
                )

            print(f'  Started {spec.name} (PID: {proc.pid})')

        # Wait for system to stabilize
        print(f'  Waiting {self.args.startup_delay}s for system to stabilize...')
        time.sleep(self.args.startup_delay)

        # Start timer
        timer_start = time.time()
        print(f'  Timer started (limit: {self.args.time_limit}s)')

        # Monitor loop
        while True:
            elapsed = time.time() - timer_start

            # Check goal reached
            if self.goal_event.is_set():
                self.terminate_all_processes()
                return TrialResult(
                    trial_number=trial_number,
                    goal_reached=True,
                    elapsed_time=elapsed,
                    reason='goal_reached',
                )

            # Check timeout
            if elapsed >= self.args.time_limit:
                self.terminate_all_processes()
                return TrialResult(
                    trial_number=trial_number,
                    goal_reached=False,
                    elapsed_time=self.args.time_limit,
                    reason='timeout',
                )

            # Check process health
            for name, proc in self.active_processes.items():
                if proc.poll() is not None:
                    self.terminate_all_processes()
                    return TrialResult(
                        trial_number=trial_number,
                        goal_reached=False,
                        elapsed_time=elapsed,
                        reason='error',
                        error_message=f'{name} crashed (exit={proc.returncode})',
                    )

            time.sleep(0.1)

    def _stderr_monitor_thread(self, proc):
        """Read planner stderr looking for goal detection."""
        try:
            for line in iter(proc.stderr.readline, b''):
                decoded = line.decode('utf-8', errors='replace')
                if self.args.verbose:
                    sys.stderr.write(f'[Planner] {decoded}')
                if 'Goal reached' in decoded:
                    self.goal_event.set()
                    # Keep draining to avoid buffer blocking
        except Exception:
            pass

    def _drain_thread(self, stream, label, buffer_stderr=False):
        """Drain a subprocess stream to prevent buffer blocking."""
        buf = self.stderr_buffers.get(label.replace('-stdout', ''), None) if buffer_stderr else None
        try:
            for line in iter(stream.readline, b''):
                decoded = line.decode('utf-8', errors='replace')
                if buf is not None:
                    buf.append(decoded)
                if self.args.verbose:
                    sys.stderr.write(f'[{label}] {decoded}')
        except Exception:
            pass

    def terminate_all_processes(self):
        """SIGTERM all processes, wait, then SIGKILL stragglers."""
        for name, proc in self.active_processes.items():
            if proc.poll() is None:
                try:
                    proc.terminate()
                except Exception:
                    pass

        # Wait up to 5 seconds for graceful shutdown
        deadline = time.time() + 5.0
        for name, proc in self.active_processes.items():
            remaining = max(0.1, deadline - time.time())
            try:
                proc.wait(timeout=remaining)
            except subprocess.TimeoutExpired:
                try:
                    proc.kill()
                    proc.wait(timeout=2.0)
                except Exception:
                    pass

    def cleanup_between_trials(self):
        """Ensure all ports are free before next trial."""
        self.terminate_all_processes()
        self.active_processes.clear()

        ports = [2390, 2391, 2392]
        if self.args.remote_viewer:
            ports.append(self.args.remote_port)
        max_wait = 10.0
        start = time.time()

        while time.time() - start < max_wait:
            all_free = all(not self._is_port_in_use(p) for p in ports)
            if all_free:
                break
            time.sleep(0.5)
        else:
            # Fallback: force kill processes on ports
            for port in ports:
                self._force_free_port(port)
            time.sleep(1.0)

        # Extra safety buffer
        time.sleep(1.0)

    def _is_port_in_use(self, port):
        """Check if a port is in use (UDP and TCP)."""
        for sock_type in (socket.SOCK_DGRAM, socket.SOCK_STREAM):
            try:
                sock = socket.socket(socket.AF_INET, sock_type)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(('127.0.0.1', port))
                sock.close()
            except OSError:
                return True
        return False

    def _force_free_port(self, port):
        """Use fuser to find and kill processes holding a port (UDP and TCP)."""
        for proto in ('udp', 'tcp'):
            try:
                result = subprocess.run(
                    ['fuser', f'{port}/{proto}'],
                    capture_output=True, text=True, timeout=5
                )
                if result.stdout.strip():
                    pids = result.stdout.strip().split()
                    for pid in pids:
                        try:
                            os.kill(int(pid.strip()), signal.SIGKILL)
                        except (ValueError, ProcessLookupError):
                            pass
            except Exception:
                pass

    def print_summary(self):
        """Print formatted summary table."""
        if not self.results:
            print('No results to report.')
            return

        print()
        print('=' * 62)
        print('BENCHMARK RESULTS')
        print('=' * 62)
        print(f'{"Trial":>5}  {"Result":<8}  {"Time (s)":>9}  {"Reason"}')
        print(f'{"-----":>5}  {"------":<8}  {"--------":>9}  {"------"}')

        for r in self.results:
            status = 'SUCCESS' if r.goal_reached else 'FAIL'
            reason = r.reason
            if r.error_message:
                reason += f': {r.error_message}'
            print(f'{r.trial_number:>5}  {status:<8}  {r.elapsed_time:>9.2f}  {reason}')

        print('-' * 62)

        total = len(self.results)
        successes = [r for r in self.results if r.goal_reached]
        n_success = len(successes)

        print(f'Success Rate:  {n_success}/{total} ({100.0 * n_success / total:.1f}%)')

        if successes:
            times = [r.elapsed_time for r in successes]
            print(f'Avg Time (successes): {sum(times) / len(times):.2f}s')
            print(f'Min Time: {min(times):.2f}s   Max Time: {max(times):.2f}s')

        all_times = [r.elapsed_time for r in self.results]
        print(f'Avg Time (all trials): {sum(all_times) / len(all_times):.2f}s')
        print('=' * 62)

    def save_results(self):
        """Save results to JSON file."""
        if not self.results:
            return

        total = len(self.results)
        successes = [r for r in self.results if r.goal_reached]
        n_success = len(successes)

        summary = {
            'success_rate': n_success / total if total > 0 else 0.0,
            'successes': n_success,
            'failures': total - n_success,
            'total_trials': total,
        }

        if successes:
            times = [r.elapsed_time for r in successes]
            summary['avg_time_successes'] = sum(times) / len(times)
            summary['min_time'] = min(times)
            summary['max_time'] = max(times)

        all_times = [r.elapsed_time for r in self.results]
        summary['avg_time_all'] = sum(all_times) / len(all_times)

        output = {
            'metadata': {
                'timestamp': datetime.datetime.now().isoformat(),
                'num_trials': self.args.trials,
                'time_limit_seconds': self.args.time_limit,
                'startup_delay_seconds': self.args.startup_delay,
                'xml_model': os.path.basename(self.xml_model),
                'physics_rate': self.args.physics_rate,
                'sensor_rate': self.args.sensor_rate,
                'visualization': not self.args.no_viz,
                'sensor_noise': self.args.sensor_noise,
            },
            'summary': summary,
            'trials': [
                {
                    'trial': r.trial_number,
                    'goal_reached': r.goal_reached,
                    'elapsed_time': round(r.elapsed_time, 3),
                    'reason': r.reason,
                    'error_message': r.error_message,
                }
                for r in self.results
            ],
        }

        output_path = self.args.output
        with open(output_path, 'w') as f:
            json.dump(output, f, indent=2)

        print(f'\nResults saved to: {output_path}')


def main():
    parser = argparse.ArgumentParser(
        description='Benchmark the tensegrity hybrid MPPI system'
    )
    parser.add_argument(
        '--trials', '-n', type=int, default=5,
        help='Number of benchmark trials (default: 5)'
    )
    parser.add_argument(
        '--time-limit', '-t', type=float, default=120.0,
        help='Time limit per trial in seconds (default: 120)'
    )
    parser.add_argument(
        '--xml-model', type=str, default=None,
        help='Path to MuJoCo XML model (default: auto-detect)'
    )
    parser.add_argument(
        '--no-viz', action='store_true',
        help='Disable MuJoCo visualization'
    )
    parser.add_argument(
        '--sensor-noise', action='store_true',
        help='Enable sensor noise in simulator'
    )
    parser.add_argument(
        '--physics-rate', type=float, default=1000.0,
        help='Simulator physics rate in Hz (default: 1000)'
    )
    parser.add_argument(
        '--sensor-rate', type=float, default=20.0,
        help='Simulator sensor rate in Hz (default: 20)'
    )
    parser.add_argument(
        '--output', '-o', type=str, default='benchmark_results.json',
        help='Output JSON file path (default: benchmark_results.json)'
    )
    parser.add_argument(
        '--startup-delay', type=float, default=7.0,
        help='Seconds to wait after all processes launched before starting timer (default: 7)'
    )
    parser.add_argument(
        '--verbose', '-v', action='store_true',
        help='Print process stdout/stderr in real time'
    )
    parser.add_argument(
        '--remote-viewer', action='store_true',
        help='Enable remote WebSocket viewer for the simulator (accessible via browser)'
    )
    parser.add_argument(
        '--remote-port', type=int, default=8765,
        help='Port for remote viewer web server (default: 8765)'
    )

    args = parser.parse_args()

    orchestrator = BenchmarkOrchestrator(args)
    orchestrator.run_all_trials()


if __name__ == '__main__':
    main()
