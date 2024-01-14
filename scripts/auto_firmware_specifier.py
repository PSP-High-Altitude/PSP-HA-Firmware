Import("env")

import subprocess

def run_git_command(git_command):
    try:
        # Run the provided git command and capture the output
        result = subprocess.run(["git"] + git_command.split(), capture_output=True, text=True, check=True)
        return result.stdout.strip()
    except subprocess.CalledProcessError as e:
        # Handle any errors that may occur
        print(f"Error running 'git {git_command}': {e}")
        return None

def get_firmware_specifier():
    # Try to generate firmware specifier using git describe --tags
    describe_output = run_git_command('describe --tags')
    if describe_output is not None:
        return describe_output
    
    # If that didn't work, try to to use the full commit hash using rev-parse
    rev_parse_output = run_git_command('rev-parse HEAD')
    if rev_parse_output is not None:
        return rev_parse_output
    
    # If that still didn't work, just return unknown
    return 'unknown'

def get_firmware_specifier_build_flag():
    return "-D FIRMWARE_SPECIFIER=\\\"" + get_firmware_specifier() + "\\\""

env.Append(
    BUILD_FLAGS=[get_firmware_specifier_build_flag()]
)
