import sys

if len(sys.argv) != 3:
    print("Usage: python filter.py <input_file> <output_file>")
    sys.exit(1)

input_file = sys.argv[1]
output_file = sys.argv[2]

lines = []
with open(input_file, "r") as f:
    while True:
        try:
            line = next(f)
            lines.append(line)
            if 'Starting' in lines[-1]:
                lines = []
        except UnicodeDecodeError:
            continue
        except StopIteration:
            break

with open(output_file, "w") as f:
    f.writelines(lines)