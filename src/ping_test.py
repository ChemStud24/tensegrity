import subprocess
import platform
import re
import time

# List of IP addresses to monitor
IPS = [
    "172.16.71.78",
    "172.16.71.79",
    "172.16.71.80"
]

THRESHOLD_MS = 8.0


def ping(ip):
    system = platform.system().lower()

    if system == "windows":
        cmd = ["ping", "-n", "1", "-w", "1000", ip]
    else:
        cmd = ["ping", "-c", "1", "-W", "1", ip]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=2
        )

        if result.returncode != 0:
            return None

        output = result.stdout

        match = re.search(r'time[=<]?([\d.]+)\s*ms', output)
        if match:
            return float(match.group(1))

    except Exception:
        pass

    return None


while True:
    print("-" * 50)

    for ip in IPS:
        latency = ping(ip)

        if latency is None:
            print(f"{ip}: Host unreachable")
        elif latency < THRESHOLD_MS:
            print(f"{ip}: OK ({latency:.2f} ms)")
        else:
            print(f"ALERT: {ip}: High latency ({latency:.2f} ms)")

    time.sleep(1)
