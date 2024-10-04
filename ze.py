import os
import zipfile
import argparse
import re
import shutil
import sys

def extract_submitter_name(filename):
    # Extract the submitter's name (all text up to the first underscore)
    parts = filename.split('_', 1)
    if parts:
        return parts[0].strip()
    return None

def get_top_level_folders(zip_file):
    top_level = set()
    for name in zip_file.namelist():
        parts = name.split('/')
        if len(parts) > 1:
            top_level.add(parts[0])
    return top_level

def process_zip_file(main_zip_path, output_dir):
    # Remove the output directory if it exists and recreate it
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    # Open the main zip file
    with zipfile.ZipFile(main_zip_path, 'r') as main_zip:
        # Get top-level folders
        top_level_folders = get_top_level_folders(main_zip)

        for folder_name in top_level_folders:
            # Extract the submitter's name from the folder name
            submitter_name = extract_submitter_name(folder_name)
            if not submitter_name:
                print(f"Skipping {folder_name}: Unable to extract submitter's name")
                continue
            
            # Create a temporary directory for extraction
            temp_dir = os.path.join(output_dir, 'temp_extraction')
            os.makedirs(temp_dir, exist_ok=True)
            
            # Extract the folder contents
            folder_contents = [f for f in main_zip.namelist() if f.startswith(f"{folder_name}/")]
            for file in folder_contents:
                main_zip.extract(file, temp_dir)
            
            # Find PDF or HTML files in the extracted folder
            extracted_folder_path = os.path.join(temp_dir, folder_name)
            target_files = [f for f in os.listdir(extracted_folder_path) 
                            if f.lower().endswith(('.pdf', '.html'))]
            
            if target_files:
                # Rename and move the first found file
                target_file = target_files[0]
                file_extension = os.path.splitext(target_file)[1]
                old_file_path = os.path.join(extracted_folder_path, target_file)
                new_file_path = os.path.join(output_dir, f"{submitter_name}{file_extension}")
                shutil.move(old_file_path, new_file_path)
                print(f"Extracted and renamed: {new_file_path}")
            else:
                print(f"No PDF or HTML file found in {folder_name}")
            
            # Clean up the temporary directory
            shutil.rmtree(temp_dir)

    print(f"Processing complete. Output directory: {output_dir}")

def main():
    # Set up argument parser with a more detailed description
    parser = argparse.ArgumentParser(
        description="""
        Process a zip file containing folders with PDFs or HTML files.
        
        This script takes a zip file as input, where the zip file contains multiple folders.
        Each folder is expected to be named with a submitter's name followed by an underscore and additional information.
        The script will extract PDF or HTML files from these folders, rename them based on the part of the folder name
        before the first underscore (the submitter's name), and place them in a new directory.
        
        The output directory is automatically named by appending 'out' to the input filename (without extension).
        For example, if the input is 'myarchive.zip', the output directory will be 'myarchiveout'.
        
        If no input file is specified, the script will look for a file named 'in.zip' in the current directory.
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input_zip", nargs='?', default="in.zip", 
                        help="Path to the input zip file (default: in.zip)")
    
    # Parse arguments
    args = parser.parse_args()

    # Generate output directory name
    input_name = os.path.splitext(args.input_zip)[0]
    output_dir = f"{input_name}out"

    # Check if the input file exists
    if not os.path.exists(args.input_zip):
        print(f"Error: The file '{args.input_zip}' does not exist.")
        sys.exit(1)

    # Call the process_zip_file function with the provided arguments
    process_zip_file(args.input_zip, output_dir)

if __name__ == "__main__":
    main()