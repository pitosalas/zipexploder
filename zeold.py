import os
import zipfile
import argparse
import re

def extract_person_name(filename):
    # Extract the person's name (all text up to the first number)
    match = re.search(r'^([^\d]+)', filename)
    if match:
        return match.group(1).strip().replace('_', ' ')
    return None

def process_zip_file(main_zip_path, output_dir):
    # Create the output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Open the main zip file
    with zipfile.ZipFile(main_zip_path, 'r') as main_zip:
        # Iterate through all files in the main zip
        for file_name in main_zip.namelist():
            if file_name.endswith('.zip'):
                # Extract the person's name from the zip file name
                person_name = extract_person_name(file_name)
                if not person_name:
                    print(f"Skipping {file_name}: Unable to extract person's name")
                    continue
                
                # Extract the individual zip file
                main_zip.extract(file_name, output_dir)
                individual_zip_path = os.path.join(output_dir, file_name)
                
                # Open the individual zip file
                with zipfile.ZipFile(individual_zip_path, 'r') as individual_zip:
                    # Find PDF or HTML files
                    target_files = [f for f in individual_zip.namelist() if f.lower().endswith(('.pdf', '.html'))]
                    
                    if target_files:
                        # Extract and rename the first found file
                        target_file = target_files[0]
                        file_extension = os.path.splitext(target_file)[1]
                        individual_zip.extract(target_file, output_dir)
                        old_file_path = os.path.join(output_dir, target_file)
                        new_file_path = os.path.join(output_dir, f"{person_name}{file_extension}")
                        os.rename(old_file_path, new_file_path)
                        print(f"Extracted and renamed: {new_file_path}")
                    else:
                        print(f"No PDF or HTML file found in {file_name}")
                
                # Remove the individual zip file
                os.remove(individual_zip_path)

    print(f"Processing complete. Output directory: {output_dir}")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Process a zip file containing multiple zip files with PDFs or HTML files.")
    parser.add_argument("input_zip", help="Path to the input zip file")
    parser.add_argument("output_dir", help="Path to the output directory")
    
    # Parse arguments
    args = parser.parse_args()

    # Call the process_zip_file function with the provided arguments
    process_zip_file(args.input_zip, args.output_dir)

if __name__ == "__main__":
    main()