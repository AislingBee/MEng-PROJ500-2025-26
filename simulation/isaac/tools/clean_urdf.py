import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

THRESHOLD = 1e-12  # anything smaller becomes 0


def clean_vector_string(vec_string):
    """
    Takes a string like:
        "-3.7e-81 -0.0 0.0000000000001"
    Returns cleaned string like:
        "0 0 0"
    """
    values = vec_string.strip().split()
    cleaned = []

    for v in values:
        try:
            num = float(v)
            if abs(num) < THRESHOLD:
                cleaned.append("0")
            else:
                cleaned.append(str(num))
        except ValueError:
            # if it can't parse, leave it unchanged
            cleaned.append(v)

    return " ".join(cleaned)


def clean_urdf(file_path):
    tree = ET.parse(file_path)
    root = tree.getroot()

    count = 0

    # Iterate through all elements
    for elem in root.iter():
        for attr in ["xyz", "rpy"]:
            if attr in elem.attrib:
                original = elem.attrib[attr]
                cleaned = clean_vector_string(original)

                if cleaned != original:
                    elem.set(attr, cleaned)
                    count += 1

    tree.write(file_path, encoding="utf-8", xml_declaration=True)
    print(f"[OK] Cleaned {count} vector attributes.")
    print(f"[OK] Updated file: {file_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Clean tiny numeric noise in URDF xyz/rpy fields.")
    parser.add_argument("urdf_path", type=str, help="Path to URDF file.")
    args = parser.parse_args()

    urdf_file = Path(args.urdf_path)

    if not urdf_file.exists():
        print(f"[ERROR] File does not exist: {urdf_file}")
        exit(1)

    clean_urdf(urdf_file)
