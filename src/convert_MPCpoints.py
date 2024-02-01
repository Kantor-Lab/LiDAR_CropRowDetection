# Define the path to your text file
file_path = "Waypoint_from_map/waypoints2.txt"

# Initialize an empty list to store the extracted data
formatted_data = []

# Open the file and read its contents
with open(file_path, "r") as file:
    for line in file:
        # Split each line by commas and convert the values to float
        values = [float(val) for val in line.strip().split(",")]
        
        # Remove the last value (last column)
        # values.pop(0)
        values.pop()
        
        # Format each value to scientific notation with 18 decimal places
        formatted_values = [f"{val:.18e}" for val in values]
        
        # Add extra values to the end of each row
        formatted_values.extend(["0.000000000000000000e+00"] * 4)
        
        # Join the formatted values without spaces and add them to the formatted_data list
        formatted_data.append(",".join(formatted_values))

# Print the formatted data
for row in formatted_data:
    print(row)