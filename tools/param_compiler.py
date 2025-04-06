import os
import re
import json
import sys

# Define the directory containing the .c files
parameters_dir = r"c:\espidf2024\projects\IKARUS\main\parameters"

# Regex pattern to match param_t structures and their descriptions
param_pattern = re.compile(
    r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
    r"param_t\s+(\w+)\s*=\s*{\s*"
    r"\.name\s*=\s*\"([^\"]+)\",\s*"
    r"\.type\s*=\s*(\w+),\s*"
    r"\.min_value\s*=\s*{\s*\.f\s*=\s*([-+]?[0-9]*\.?[0-9]+f?)\s*},\s*"
    r"\.max_value\s*=\s*{\s*\.f\s*=\s*([-+]?[0-9]*\.?[0-9]+f?)\s*},\s*"
    r"\.value\s*=\s*{\s*\.f\s*=\s*([-+]?[0-9]*\.?[0-9]+f?)\s*}"
)

# List to store extracted parameters
parameters = []
names_set = set()  # To track duplicate names

# Iterate through all .c files in the directory
for filename in os.listdir(parameters_dir):
    if filename.endswith(".c"):
        filepath = os.path.join(parameters_dir, filename)
        with open(filepath, "r") as file:
            content = file.read()
            # Find all matches for param_t structures
            matches = param_pattern.findall(content)
            for match in matches:
                description = match[0].strip() if match[0] else ""  # Extract description or set to empty
                variable_name = match[1]
                name = match[2]
                param_type = match[3]
                min_value = float(match[4].rstrip('f'))
                max_value = float(match[5].rstrip('f'))
                value = float(match[6].rstrip('f'))

                # Check if name is unique
                if name in names_set:
                    print(f"Error: Duplicate parameter name '{name}' found. Operation aborted.")
                    sys.exit(1)
                names_set.add(name)

                # Check if min_value is less than max_value
                if min_value >= max_value:
                    print(f"Error: min_value ({min_value}) is not less than max_value ({max_value}) for parameter '{name}'. Operation aborted.")
                    sys.exit(1)

                # Check if value is within the range [min_value, max_value]
                if not (min_value <= value <= max_value):
                    print(f"Error: value ({value}) is not within the range [{min_value}, {max_value}] for parameter '{name}'. Operation aborted.")
                    sys.exit(1)

                # Check if name length is within 16 characters
                if len(name) > 16:
                    print(f"Error: Parameter name '{name}' exceeds 16 characters. Operation aborted.")
                    sys.exit(1)

                # Add the parameter to the list
                param = {
                    "description": description,
                    "variable_name": variable_name,
                    "name": name,
                    "type": param_type,
                    "min_value": min_value,
                    "max_value": max_value,
                    "value": value
                }
                parameters.append(param)

# Output JSON file path
output_json_path = os.path.join(parameters_dir, "ikarus_params.json")

# Write the parameters to a JSON file
with open(output_json_path, "w") as json_file:
    json.dump(parameters, json_file, indent=4)

print(f"Parameters have been written to {output_json_path}")

