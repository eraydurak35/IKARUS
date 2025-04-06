import os
import re
import json
import sys

# Define the directory containing the .c files
parameters_dir = r"c:\espidf2024\projects\IKARUS\main\parameters"
param_h_path = os.path.join(parameters_dir, "param.h")  # Path to param.h
param_c_path = os.path.join(parameters_dir, "param.c")  # Path to param.c

# Regex patterns to match PARAM_DEFINE_* structures and their descriptions
param_patterns = {
    "float": re.compile(
        r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
        r"PARAM_DEFINE_FLOAT\((\w+),\s*([-+]?[0-9]*\.?[0-9]+f?),\s*([-+]?[0-9]*\.?[0-9]+f?),\s*([-+]?[0-9]*\.?[0-9]+f?)\)"
    ),
    "int32": re.compile(
        r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
        r"PARAM_DEFINE_INT32\((\w+),\s*([-+]?[0-9]+),\s*([-+]?[0-9]+),\s*([-+]?[0-9]+)\)"
    ),
    "bool": re.compile(
        r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
        r"PARAM_DEFINE_BOOL\((\w+),\s*(\d+)\)"
    ),
}

# List to store extracted parameters
parameters = []
names_set = set()  # To track duplicate names

# Iterate through all .c files in the directory
for filename in os.listdir(parameters_dir):
    if filename.endswith(".c"):
        filepath = os.path.join(parameters_dir, filename)
        category_name = os.path.splitext(filename)[0]  # Extract the file name without extension
        with open(filepath, "r") as file:
            content = file.read()
            # Process each parameter type
            for param_type, pattern in param_patterns.items():
                matches = pattern.findall(content)
                for match in matches:
                    description = match[0].strip() if match[0] else ""  # Extract description or set to empty
                    name = f"{match[1]}"

                    # Extract values based on parameter type
                    if param_type == "float":
                        default_value = float(match[2].rstrip('f'))
                        min_value = float(match[3].rstrip('f'))
                        max_value = float(match[4].rstrip('f'))
                    elif param_type == "int32":
                        default_value = int(match[2])
                        min_value = int(match[3])
                        max_value = int(match[4])
                    elif param_type == "bool":
                        default_value = bool(int(match[2]))
                        min_value = 0
                        max_value = 1

                    # Check if name is unique
                    if name in names_set:
                        print(f"Error: Duplicate parameter name '{name}' found. Operation aborted.")
                        sys.exit(1)
                    names_set.add(name)

                    # Add the parameter to the list
                    param = {
                        "category": category_name,  # Add category based on the file name
                        "description": description,
                        "name": name,
                        "type": param_type,
                        "default_value": default_value,
                        "min_value": min_value,
                        "max_value": max_value,
                    }
                    parameters.append(param)

# Output JSON file path
output_json_path = os.path.join(parameters_dir, "ikarus_params.json")

# Write the parameters to a JSON file
with open(output_json_path, "w") as json_file:
    json.dump(parameters, json_file, indent=4)

print(f"Parameters have been written to {output_json_path}")

# Update param.h file
if os.path.exists(param_h_path):
    with open(param_h_path, "r") as file:
        param_h_content = file.read()

    # Find the PARAM_START and PARAM_END markers
    start_marker = "/***** PARAM_START *****/"
    end_marker = "/***** PARAM_END *****/"
    start_index = param_h_content.find(start_marker)
    end_index = param_h_content.find(end_marker)

    if start_index != -1 and end_index != -1 and start_index < end_index:
        # Extract the content before and after the markers
        before_start = param_h_content[:start_index + len(start_marker)]
        after_end = param_h_content[end_index:]

        # Generate PARAM_EXTERN lines
        extern_lines = "\n".join([f"PARAM_EXTERN({param['name']});" for param in parameters])

        # Combine the updated content
        updated_content = f"{before_start}\n{extern_lines}\n{after_end}"

        # Write the updated content back to param.h
        with open(param_h_path, "w") as file:
            file.write(updated_content)

        print(f"param.h has been updated with PARAM_EXTERN declarations.")
    else:
        print(f"Error: Could not find PARAM_START and PARAM_END markers in param.h. No changes were made.")
else:
    print(f"Error: param.h file not found.")



# Sort parameters alphabetically
sorted_params = sorted(parameters, key=lambda x: x['name'])

# Update param.c file with PARAM_LIST
if os.path.exists(param_c_path):
    with open(param_c_path, "r") as file:
        param_c_content = file.read()

    # Find the PARAM_LIST_START and PARAM_LIST_END markers
    list_start_marker = "/***** PARAM_LIST_START *****/"
    list_end_marker = "/***** PARAM_LIST_END *****/"
    list_start_index = param_c_content.find(list_start_marker)
    list_end_index = param_c_content.find(list_end_marker)

    if list_start_index != -1 and list_end_index != -1 and list_start_index < list_end_index:
        # Extract the content before and after the markers
        before_list_start = param_c_content[:list_start_index + len(list_start_marker)]
        after_list_end = param_c_content[list_end_index:]

        # Generate the param_list in the desired format
        param_count = len(sorted_params)
        param_list_lines = f"param_t* param_list[{param_count}] = {{\n"
        param_list_lines += ",\n".join([f"    &PARAM_{param['name']}" for param in sorted_params])
        param_list_lines += "\n};"

        # Combine the updated content
        updated_content = f"{before_list_start}\n{param_list_lines}\n{after_list_end}"

        # Write the updated content back to param.h
        with open(param_c_path, "w") as file:
            file.write(updated_content)

        print(f"param.c has been updated with PARAM_LIST.")
    else:
        print(f"Error: Could not find PARAM_LIST_START and PARAM_LIST_END markers in param.h. No changes were made.")
else:
    print(f"Error: param.c file not found.")
















""" import os
import re
import json
import sys

# Define the directory containing the .c files
parameters_dir = r"c:\espidf2024\projects\IKARUS\main\parameters"

# Regex patterns to match PARAM_DEFINE_* structures and their descriptions
param_patterns = {
    "float": re.compile(
        r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
        r"PARAM_DEFINE_FLOAT\((\w+),\s*([-+]?[0-9]*\.?[0-9]+f?),\s*([-+]?[0-9]*\.?[0-9]+f?),\s*([-+]?[0-9]*\.?[0-9]+f?)\)"
    ),
    "int32": re.compile(
        r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
        r"PARAM_DEFINE_INT32\((\w+),\s*([-+]?[0-9]+),\s*([-+]?[0-9]+),\s*([-+]?[0-9]+)\)"
    ),
    "bool": re.compile(
        r"(?:/\*\s*(.*?)\s*\*/\s*)?"  # Optional description in /* */
        r"PARAM_DEFINE_BOOL\((\w+),\s*(\d+)\)"
    ),
}

# List to store extracted parameters
parameters = []
names_set = set()  # To track duplicate names

# Iterate through all .c files in the directory
for filename in os.listdir(parameters_dir):
    if filename.endswith(".c"):
        filepath = os.path.join(parameters_dir, filename)
        category_name = os.path.splitext(filename)[0]  # Extract the file name without extension
        with open(filepath, "r") as file:
            content = file.read()
            # Process each parameter type
            for param_type, pattern in param_patterns.items():
                matches = pattern.findall(content)
                for match in matches:
                    description = match[0].strip() if match[0] else ""  # Extract description or set to empty
                    name = f"{match[1]}"

                    # Extract values based on parameter type
                    if param_type == "float":
                        default_value = float(match[2].rstrip('f'))
                        min_value = float(match[3].rstrip('f'))
                        max_value = float(match[4].rstrip('f'))
                    elif param_type == "int32":
                        default_value = int(match[2])
                        min_value = int(match[3])
                        max_value = int(match[4])
                    elif param_type == "bool":
                        default_value = bool(int(match[2]))
                        min_value = 0
                        max_value = 1

                    # Check if name is unique
                    if name in names_set:
                        print(f"Error: Duplicate parameter name '{name}' found. Operation aborted.")
                        sys.exit(1)
                    names_set.add(name)

                    # Check if min_value is less than max_value (only for float and int32)
                    if param_type in ["float", "int32"] and min_value >= max_value:
                        print(f"Error: min_value ({min_value}) is not less than max_value ({max_value}) for parameter '{name}'. Operation aborted.")
                        sys.exit(1)

                    # Check if default_value is within the range [min_value, max_value] (only for float and int32)
                    if param_type in ["float", "int32"] and not (min_value <= default_value <= max_value):
                        print(f"Error: default_value ({default_value}) is not within the range [{min_value}, {max_value}] for parameter '{name}'. Operation aborted.")
                        sys.exit(1)

                    # Check if name length is within 16 characters
                    if len(name) > 16:
                        print(f"Error: Parameter name '{name}' exceeds 16 characters. Operation aborted.")
                        sys.exit(1)

                    # Add the parameter to the list
                    param = {
                        "category": category_name,  # Add category based on the file name
                        "description": description,
                        "name": name,
                        "type": param_type,
                        "default_value": default_value,
                        "min_value": min_value,
                        "max_value": max_value,
                    }
                    parameters.append(param)

# Output JSON file path
output_json_path = os.path.join(parameters_dir, "parameters.json")

# Write the parameters to a JSON file
with open(output_json_path, "w") as json_file:
    json.dump(parameters, json_file, indent=4)

print(f"Parameters have been written to {output_json_path}")

 """