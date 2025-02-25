Import('env')

from pathlib import Path

def load_as_initializer_list(file_path: Path) -> str:
    with open(file_path, 'r') as f:
        return [
            '    {' + line.rstrip(',\n') + '},'
            for line in f.readlines()
        ]

def generate_hwil_data_defs(file_path: Path, data_type: str, name: str) -> str:
    initializer_list = load_as_initializer_list(file_path)
    return f"""
const size_t HWIL_{name.upper()}_DATA_SIZE = {len(initializer_list)};
const {data_type} HWIL_{name.upper()}_DATA[] = {{
{chr(10).join(initializer_list)}
}};
"""

def generate_hwil_data_file():
    hwil_data_dir_path = env.GetProjectOption("hwil_data_dir")

    sensor_file_path = next(Path(hwil_data_dir_path).glob("*dat.csv"))
    gps_file_path = next(Path(hwil_data_dir_path).glob("*gps.csv"))

    print(f"Generating sensor HWIL data from {sensor_file_path}")
    sensor_data_defs = generate_hwil_data_defs(sensor_file_path, "SensorFrame", "SENSOR")

    print(f"Generating gps HWIL data from {gps_file_path}")
    gps_data_defs = generate_hwil_data_defs(gps_file_path, "GpsFrame", "GPS")

    return f"""
#include <stdint.h>
#include <math.h>

#include "sensor.pb.h"
#include "gps.pb.h"

{sensor_data_defs}

{gps_data_defs}
"""

def write_hwil_data_file():
    pio_build_dir = Path(env.subst('${BUILD_DIR}'))

    hwil_src_dir = pio_build_dir/"hwil-src"
    hwil_build_dir = pio_build_dir/"hwil-build"

    hwil_src_dir.mkdir(exist_ok=True)
    hwil_build_dir.mkdir(exist_ok=True)

    output_file_path = hwil_src_dir/"hwil_test.c"
    output_file_str = generate_hwil_data_file()

    print(f"Writing generated HWIL data file to {output_file_path}")
    with open(output_file_path, 'w') as f:
        f.write(output_file_str)

    env.BuildSources(str(hwil_build_dir), str(hwil_src_dir))

write_hwil_data_file()
