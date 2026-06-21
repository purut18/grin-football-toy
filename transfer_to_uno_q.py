#!/usr/bin/env python3
"""
transfer_to_uno_q.py — Deploy GRIN Football App to Arduino UNO Q
=================================================================

This script transfers the contents of the 'arduino-app/' folder to the
Arduino UNO Q's filesystem. Once transferred, the app can be run directly
from the Arduino App Lab or set as the startup application.

Transfer Methods (in priority order):
    1. ADB (Android Debug Bridge) — fastest, works over USB-C
    2. SCP (Secure Copy) — works over Wi-Fi/Ethernet via SSH
    3. Manual instructions — if neither ADB nor SSH is available

Prerequisites:
    - Arduino UNO Q connected via USB-C (for ADB) or on local network (for SCP)
    - ADB installed: 'brew install android-platform-tools' (macOS)
    - Or SSH access configured on the UNO Q

Usage:
    python3 transfer_to_uno_q.py              # Auto-detect method (ADB → SCP)
    python3 transfer_to_uno_q.py --method adb  # Force ADB transfer
    python3 transfer_to_uno_q.py --method scp  # Force SCP transfer
    python3 transfer_to_uno_q.py --method scp --host 192.168.1.50  # SCP with IP
    python3 transfer_to_uno_q.py --set-startup  # Also set as startup app

Target Directory on UNO Q:
    The app is deployed to /home/user/ArduinoApps/grin-football/
    This path follows the Arduino App Lab convention for user apps.
"""

# Standard library imports — no external dependencies required.
import subprocess   # Execute shell commands (adb, scp, ssh) as child processes
import sys          # Command-line arguments and exit codes
import os           # Filesystem operations (path manipulation, existence checks)
import argparse     # Command-line argument parsing with help text generation
import shutil       # High-level file operations (checking command availability)
import time         # Sleep for progress indication and retry delays


# =============================================================================
# CONFIGURATION CONSTANTS
# =============================================================================

# The name of the app as it will appear in Arduino App Lab.
# This is derived from the folder name on the UNO Q's filesystem.
APP_NAME = "grin-football"

# The local source directory containing the App Lab app structure.
# Relative to the project root (where this script lives).
SOURCE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arduino-app")

# The target directory on the UNO Q's filesystem where the app will be deployed.
# Arduino App Lab stores user apps in /home/user/ArduinoApps/<app-name>/.
# This path follows the convention used by the App Lab for discovering user apps.
TARGET_DIR = f"/home/user/ArduinoApps/{APP_NAME}"

# Default SSH/SCP connection parameters.
# The UNO Q runs Debian Linux with a user account configured during first setup.
DEFAULT_USER = "user"       # Default Linux username on the UNO Q
DEFAULT_HOST = None         # Must be provided for SCP (IP or hostname)
DEFAULT_PORT = 22           # Default SSH port


# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def print_banner():
    """
    Print a styled banner for the transfer script.
    Uses ANSI escape codes for coloured output in the terminal.
    """
    # ANSI colour codes for terminal output formatting.
    CYAN = "\033[96m"    # Bright cyan for the banner border
    GREEN = "\033[92m"   # Green for success messages
    YELLOW = "\033[93m"  # Yellow for warnings
    RED = "\033[91m"     # Red for errors
    RESET = "\033[0m"    # Reset to default terminal colour
    BOLD = "\033[1m"     # Bold text

    print(f"""
{CYAN}╔══════════════════════════════════════════════════════════╗
║  {BOLD}GRIN Football Toy — UNO Q Deployment Script{RESET}{CYAN}             ║
║  Transfer arduino-app/ → Arduino UNO Q                   ║
╚══════════════════════════════════════════════════════════╝{RESET}
""")


def print_success(msg):
    """Print a green success message."""
    print(f"\033[92m✓ {msg}\033[0m")


def print_warning(msg):
    """Print a yellow warning message."""
    print(f"\033[93m⚠ {msg}\033[0m")


def print_error(msg):
    """Print a red error message."""
    print(f"\033[91m✗ {msg}\033[0m")


def print_info(msg):
    """Print a cyan informational message."""
    print(f"\033[96mℹ {msg}\033[0m")


def is_command_available(command):
    """
    Check if a shell command is available on the system PATH.

    Uses shutil.which() which searches the PATH environment variable
    for the given command, similar to the Unix 'which' command.

    Args:
        command (str): The command name to search for (e.g., 'adb', 'scp').

    Returns:
        bool: True if the command is found on PATH, False otherwise.
    """
    return shutil.which(command) is not None


def run_command(cmd, capture_output=True, timeout=30):
    """
    Execute a shell command and return the result.

    Uses subprocess.run() which creates a child process, waits for it
    to complete, and captures stdout/stderr.

    Args:
        cmd (list): Command and arguments as a list (e.g., ['adb', 'devices']).
        capture_output (bool): Whether to capture stdout/stderr (True) or
                               let them flow to the terminal (False).
        timeout (int): Maximum seconds to wait before killing the process.

    Returns:
        subprocess.CompletedProcess: The result object containing returncode,
                                     stdout, and stderr.
    """
    try:
        result = subprocess.run(
            cmd,
            capture_output=capture_output,
            text=True,                    # Decode stdout/stderr as UTF-8 strings
            timeout=timeout,              # Kill process if it exceeds timeout
        )
        return result
    except subprocess.TimeoutExpired:
        print_error(f"Command timed out after {timeout}s: {' '.join(cmd)}")
        return None
    except FileNotFoundError:
        print_error(f"Command not found: {cmd[0]}")
        return None


def get_remote_home_dir(method="adb", host=None, user=DEFAULT_USER, port=DEFAULT_PORT):
    """
    Get the home directory of the remote user on the UNO Q.
    For ADB, it queries the shell's HOME environment variable via printenv.
    For SCP, it queries the SSH host's HOME environment variable.
    """
    if method == "adb":
        result = run_command(["adb", "shell", "printenv", "HOME"])
        if result and result.returncode == 0:
            home = result.stdout.strip()
            if home:
                return home
        return "/home/arduino"  # Default fallback
    elif method == "scp" and host:
        result = run_command([
            "ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
            "-p", str(port), f"{user}@{host}", "printenv", "HOME"
        ])
        if result and result.returncode == 0:
            home = result.stdout.strip()
            if home:
                return home
    return f"/home/{user}"  # Default fallback


# =============================================================================
# ADB TRANSFER METHOD
# =============================================================================

def check_adb_device():
    """
    Check if an Arduino UNO Q is connected and detected by ADB.

    ADB (Android Debug Bridge) communicates with the UNO Q over USB-C.
    The 'adb devices' command lists all connected ADB-compatible devices.
    The UNO Q should appear as a device with status 'device' (not 'unauthorized').

    Returns:
        bool: True if a device is found and ready, False otherwise.
    """
    result = run_command(["adb", "devices"])
    if result is None or result.returncode != 0:
        return False

    # Parse the 'adb devices' output.
    # Format: "List of devices attached\n<serial>\tdevice\n"
    lines = result.stdout.strip().split("\n")

    # Filter for lines that end with 'device' (connected and authorised).
    # Exclude lines with 'unauthorized' or 'offline'.
    devices = [
        line for line in lines[1:]  # Skip the header line
        if line.strip() and "device" in line and "unauthorized" not in line
    ]

    return len(devices) > 0


def transfer_via_adb():
    """
    Transfer the arduino-app/ folder to the UNO Q using ADB push.

    ADB push copies files and directories from the local machine to the
    device's filesystem over USB-C. It's the fastest transfer method
    and doesn't require Wi-Fi or SSH configuration.

    Process:
        1. Check ADB is installed
        2. Check a device is connected
        3. Create the target directory on the UNO Q
        4. Push the sketch/ and python/ folders
        5. Verify the transfer

    Returns:
        bool: True if transfer succeeded, False otherwise.
    """
    print_info("Transfer method: ADB (USB-C)")

    # Step 1: Verify ADB is installed.
    if not is_command_available("adb"):
        print_error("ADB not found. Install with: brew install android-platform-tools")
        return False
    print_success("ADB found on system PATH.")

    # Step 2: Check for connected device.
    print_info("Checking for connected Arduino UNO Q...")
    if not check_adb_device():
        print_error("No ADB device found. Ensure the UNO Q is connected via USB-C.")
        print_info("Troubleshooting:")
        print_info("  1. Connect UNO Q via USB-C cable")
        print_info("  2. Run 'adb devices' to check connection")
        print_info("  3. On Linux, ensure udev rules are installed")
        return False
    print_success("Arduino UNO Q detected via ADB.")

    # Resolve target directory dynamically based on remote home folder.
    global TARGET_DIR
    home_dir = get_remote_home_dir("adb")
    TARGET_DIR = f"{home_dir}/ArduinoApps/{APP_NAME}"

    # Step 3: Create the target directory on the UNO Q.
    # 'adb shell mkdir -p' creates the directory and all parent directories.
    print_info(f"Creating target directory: {TARGET_DIR}")
    result = run_command(["adb", "shell", "mkdir", "-p", TARGET_DIR])
    if result is None or result.returncode != 0:
        print_error(f"Failed to create directory: {TARGET_DIR}")
        return False

    # Step 4: Push the sketch/, python/ folders and app.yaml.
    # We push each item individually to maintain the correct structure.
    items_to_push = ["sketch", "python", "app.yaml"]

    for item in items_to_push:
        local_path = os.path.join(SOURCE_DIR, item)
        remote_path = f"{TARGET_DIR}/{item}"

        if not os.path.exists(local_path):
            print_warning(f"Local item not found: {local_path} — skipping.")
            continue

        print_info(f"Pushing {item} → {remote_path}")

        # Remove existing item on device to ensure clean transfer.
        # This prevents stale files from previous deployments.
        run_command(["adb", "shell", "rm", "-rf", remote_path])

        # Push the folder. 'adb push' recursively copies all files.
        result = run_command(
            ["adb", "push", local_path, remote_path],
            capture_output=False,  # Show progress in terminal
            timeout=60,
        )

        if result is None or result.returncode != 0:
            print_error(f"Failed to push {item}")
            return False

        print_success(f"{item} transferred successfully.")

    # Step 5: Verify the transfer by listing the remote directory.
    print_info("Verifying transfer...")
    result = run_command(["adb", "shell", "find", TARGET_DIR, "-type", "f"])
    if result and result.returncode == 0:
        files = result.stdout.strip().split("\n")
        print_success(f"Transfer verified: {len(files)} files on device.")
        for f in files:
            print(f"    {f}")
    else:
        print_warning("Could not verify transfer — check manually.")

    return True


# =============================================================================
# SCP TRANSFER METHOD
# =============================================================================

def transfer_via_scp(host, user=DEFAULT_USER, port=DEFAULT_PORT):
    """
    Transfer the arduino-app/ folder to the UNO Q using SCP over SSH.

    SCP (Secure Copy Protocol) transfers files over an encrypted SSH
    connection. This works over Wi-Fi or Ethernet when the UNO Q and
    the development machine are on the same network.

    Prerequisites:
        - SSH enabled on the UNO Q (enabled by default)
        - UNO Q connected to the same network as the development machine
        - Known IP address or hostname of the UNO Q

    Args:
        host (str): IP address or hostname of the UNO Q.
        user (str): SSH username (default: 'user').
        port (int): SSH port (default: 22).

    Returns:
        bool: True if transfer succeeded, False otherwise.
    """
    print_info(f"Transfer method: SCP (SSH to {user}@{host}:{port})")

    # Step 1: Verify SCP is installed (it's part of OpenSSH, usually pre-installed).
    if not is_command_available("scp"):
        print_error("SCP not found. Install OpenSSH: brew install openssh")
        return False
    print_success("SCP found on system PATH.")

    # Step 2: Test SSH connectivity.
    print_info(f"Testing SSH connection to {user}@{host}...")
    result = run_command(
        ["ssh", "-o", "ConnectTimeout=5", "-o", "StrictHostKeyChecking=no",
         "-p", str(port), f"{user}@{host}", "echo", "connected"],
        timeout=10,
    )
    if result is None or result.returncode != 0:
        print_error(f"Cannot connect to {user}@{host}:{port}")
        print_info("Troubleshooting:")
        print_info("  1. Ensure UNO Q is connected to the same network")
        print_info("  2. Check the IP address: hostname -I (on UNO Q)")
        print_info("  3. Ensure SSH is enabled on the UNO Q")
        return False
    print_success("SSH connection established.")

    # Resolve target directory dynamically based on remote home folder.
    global TARGET_DIR
    home_dir = get_remote_home_dir("scp", host, user, port)
    TARGET_DIR = f"{home_dir}/ArduinoApps/{APP_NAME}"

    # Step 3: Create the target directory via SSH.
    print_info(f"Creating target directory: {TARGET_DIR}")
    run_command([
        "ssh", "-p", str(port), f"{user}@{host}",
        f"mkdir -p {TARGET_DIR}"
    ])

    items_to_push = ["sketch", "python", "app.yaml"]

    for item in items_to_push:
        local_path = os.path.join(SOURCE_DIR, item)

        if not os.path.exists(local_path):
            print_warning(f"Local item not found: {local_path} — skipping.")
            continue

        print_info(f"Transferring {item} via SCP...")

        # Remove existing item on device to ensure clean transfer.
        run_command([
            "ssh", "-p", str(port), f"{user}@{host}",
            f"rm -rf {TARGET_DIR}/{item}"
        ])

        # SCP the folder recursively.
        result = run_command(
            ["scp", "-r", "-P", str(port),
             "-o", "StrictHostKeyChecking=no",
             local_path, f"{user}@{host}:{TARGET_DIR}/"],
            capture_output=False,  # Show SCP progress
            timeout=60,
        )

        if result is None or result.returncode != 0:
            print_error(f"Failed to transfer {item}")
            return False

        print_success(f"{item} transferred successfully.")

    # Step 5: Verify the transfer.
    print_info("Verifying transfer...")
    result = run_command([
        "ssh", "-p", str(port), f"{user}@{host}",
        f"find {TARGET_DIR} -type f"
    ])
    if result and result.returncode == 0:
        files = result.stdout.strip().split("\n")
        print_success(f"Transfer verified: {len(files)} files on device.")
        for f in files:
            print(f"    {f}")
    else:
        print_warning("Could not verify transfer — check manually.")

    return True


# =============================================================================
# SET STARTUP APP
# =============================================================================

def set_startup_app(method="adb", host=None, user=DEFAULT_USER, port=DEFAULT_PORT):
    """
    Set the transferred app as the UNO Q's startup application.

    When set as the startup app, the GRIN Football Toy will automatically
    begin running whenever the UNO Q is powered on — no need to manually
    launch it from Arduino App Lab.

    Uses the arduino-app-cli command-line tool on the UNO Q.

    Args:
        method (str): Transfer method used ('adb' or 'scp').
        host (str): UNO Q hostname/IP (required for SCP method).
        user (str): SSH username (default: 'user').
        port (int): SSH port (default: 22).

    Returns:
        bool: True if the startup app was set successfully.
    """
    # The arduino-app-cli command to set the default startup app.
    cli_command = f"arduino-app-cli properties set default user {APP_NAME}"

    print_info(f"Setting '{APP_NAME}' as the startup app...")

    if method == "adb":
        result = run_command(["adb", "shell", cli_command])
    elif method == "scp" and host:
        result = run_command([
            "ssh", "-p", str(port), f"{user}@{host}", cli_command
        ])
    else:
        print_error("Cannot set startup app — invalid method or missing host.")
        return False

    if result and result.returncode == 0:
        print_success(f"'{APP_NAME}' set as startup app. It will auto-run on boot.")
        return True
    else:
        print_warning(f"Could not set startup app. Run manually on UNO Q:")
        print_info(f"  {cli_command}")
        return False


# =============================================================================
# MAIN ENTRY POINT
# =============================================================================

def main():
    """
    Main function — parses arguments, validates source files, and
    executes the transfer using the appropriate method.
    """
    # Print the styled banner.
    print_banner()

    # Parse command-line arguments using argparse.
    # argparse automatically generates --help output and validates inputs.
    parser = argparse.ArgumentParser(
        description="Transfer GRIN Football Toy app to Arduino UNO Q",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 transfer_to_uno_q.py                          # Auto-detect (ADB first)
  python3 transfer_to_uno_q.py --method adb              # Force ADB
  python3 transfer_to_uno_q.py --method scp --host 192.168.1.50
  python3 transfer_to_uno_q.py --set-startup             # Also set as boot app
        """
    )

    parser.add_argument(
        "--method", choices=["adb", "scp", "auto"], default="auto",
        help="Transfer method: 'adb' (USB-C), 'scp' (SSH/Wi-Fi), or 'auto' (default)"
    )
    parser.add_argument(
        "--host", type=str, default=None,
        help="UNO Q IP address or hostname (required for SCP method)"
    )
    parser.add_argument(
        "--user", type=str, default=DEFAULT_USER,
        help=f"SSH username on UNO Q (default: '{DEFAULT_USER}')"
    )
    parser.add_argument(
        "--port", type=int, default=DEFAULT_PORT,
        help=f"SSH port (default: {DEFAULT_PORT})"
    )
    parser.add_argument(
        "--set-startup", action="store_true",
        help="Set the app as the UNO Q's startup application (auto-run on boot)"
    )
    parser.add_argument(
        "folder", nargs="?", default="prod",
        help="Subfolder under 'arduino-app/' to transfer (e.g. 'prod', 'IR-sensor-test'). Defaults to 'prod'."
    )

    args = parser.parse_args()

    # Resolve global paths and app names based on requested folder name.
    global SOURCE_DIR, TARGET_DIR, APP_NAME
    folder_name = args.folder.strip()
    if folder_name.startswith("arduino-app/"):
        folder_name = folder_name[len("arduino-app/"):]
    elif folder_name.startswith("arduino-app\\"):
        folder_name = folder_name[len("arduino-app\\"):]

    arduino_app_root = os.path.join(os.path.dirname(os.path.abspath(__file__)), "arduino-app")
    available_folders = [d for d in os.listdir(arduino_app_root) if os.path.isdir(os.path.join(arduino_app_root, d))]

    if folder_name not in available_folders:
        print_error(f"Invalid folder name '{folder_name}'. Available folders under 'arduino-app/':")
        for f in available_folders:
            print_error(f"  - {f}")
        sys.exit(1)

    SOURCE_DIR = os.path.join(arduino_app_root, folder_name)
    APP_NAME = folder_name
    TARGET_DIR = f"/home/user/ArduinoApps/{APP_NAME}"

    # Validate source directory exists and contains the expected structure.
    print_info(f"Source directory: {SOURCE_DIR}")

    if not os.path.isdir(SOURCE_DIR):
        print_error(f"Source directory not found: {SOURCE_DIR}")
        sys.exit(1)

    # Check for required subdirectories.
    sketch_dir = os.path.join(SOURCE_DIR, "sketch")
    python_dir = os.path.join(SOURCE_DIR, "python")

    if not os.path.isdir(sketch_dir):
        print_error(f"Missing sketch/ directory: {sketch_dir}")
        sys.exit(1)

    if not os.path.isdir(python_dir):
        print_error(f"Missing python/ directory: {python_dir}")
        sys.exit(1)

    # Count files to transfer.
    file_count = 0
    for root, dirs, files in os.walk(SOURCE_DIR):
        file_count += len(files)
    print_success(f"Found {file_count} files to transfer.")
    print_info(f"Target: {TARGET_DIR}")
    print()

    # Execute the transfer using the selected method.
    success = False
    method_used = args.method

    if args.method == "auto":
        # Auto-detect: try ADB first, then SCP.
        print_info("Auto-detecting transfer method...")

        if is_command_available("adb") and check_adb_device():
            print_success("ADB device found — using ADB transfer.")
            success = transfer_via_adb()
            method_used = "adb"
        elif args.host:
            print_info("No ADB device — falling back to SCP.")
            success = transfer_via_scp(args.host, args.user, args.port)
            method_used = "scp"
        else:
            print_error("No ADB device found and no --host provided for SCP.")
            print_info("Options:")
            print_info("  1. Connect UNO Q via USB-C and try again")
            print_info("  2. Run with --method scp --host <UNO_Q_IP>")
            sys.exit(1)

    elif args.method == "adb":
        success = transfer_via_adb()

    elif args.method == "scp":
        if not args.host:
            print_error("SCP method requires --host <UNO_Q_IP>")
            sys.exit(1)
        success = transfer_via_scp(args.host, args.user, args.port)

    # Handle transfer result.
    print()
    if success:
        print_success("═══════════════════════════════════════")
        print_success("  Transfer complete!")
        print_success("═══════════════════════════════════════")

        # Optionally set as startup app.
        if args.set_startup:
            print()
            set_startup_app(
                method=method_used,
                host=args.host,
                user=args.user,
                port=args.port,
            )

        print()
        print_info("Next steps:")
        print_info("  1. Open Arduino App Lab on the UNO Q")
        print_info(f"  2. Navigate to My Apps → '{APP_NAME}'")
        print_info("  3. Click 'Run' to start the goalkeeper AI")
        print()
    else:
        print_error("═══════════════════════════════════════")
        print_error("  Transfer failed!")
        print_error("═══════════════════════════════════════")
        sys.exit(1)


# Execute main() only when this script is run directly (not imported).
if __name__ == "__main__":
    main()
