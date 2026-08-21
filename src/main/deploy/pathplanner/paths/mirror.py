import json
from pathlib import Path

MIRROR_LINE = 4.03500

def transform(obj):
    if isinstance(obj, dict):
        for key, value in obj.items():
            if key == "y" and isinstance(value, (int, float)):
                obj[key] = 2 * MIRROR_LINE - value
            elif key in ("rotation", "rotationDegrees") and isinstance(value, (int, float)):
                obj[key] = -value + 0  # avoid -0.0
            elif key == "linkedName":
                if obj[key]:
                    obj[key] = obj[key].replace("Left", "TeMp123").replace("Right", "Left").replace("TeMp123", "Right")
            else:
                transform(value)
    elif isinstance(obj, list):
        for item in obj:
            transform(item)
    return obj

def main():
    input_file = input("Enter the name of the input path file (don't include the .path extension): ")
    input_path = Path(__file__).with_name(f'{input_file}.path')
    with input_path.open("r") as f:
        data = json.load(f)

    transform(data)

    output_path = input_path.with_name(f"{input_file}_mirrored.path")
    with output_path.open("w") as f:
        json.dump(data, f, indent=2)

if __name__ == "__main__":
    main()