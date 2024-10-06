import os
import zipfile
import argparse
import re
import shutil
import sys

def extract_submitter_name(filename):
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

def process_file(file_path, submitter_name, output_dir):
    file_extension = os.path.splitext(file_path)[1].lower()
    if file_extension in ['.pdf', '.html']:
        new_file_path = os.path.join(output_dir, f"{submitter_name}{file_extension}")
        shutil.move(file_path, new_file_path)
        print(f"Moved file: {new_file_path}")
    elif file_extension == '.zip':
        submitter_folder = os.path.join(output_dir, submitter_name)
        os.makedirs(submitter_folder, exist_ok=True)
        with zipfile.ZipFile(file_path, 'r') as zip_ref:
            zip_ref.extractall(submitter_folder)
        os.remove(file_path)
        print(f"Extracted zip to folder: {submitter_folder}")
    else:
        print(f"Skipping unsupported file: {file_path}")

def process_zip_file(main_zip_path, output_dir):
    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    with zipfile.ZipFile(main_zip_path, 'r') as main_zip:
        top_level_folders = get_top_level_folders(main_zip)

        for folder_name in top_level_folders:
            submitter_name = extract_submitter_name(folder_name)
            if not submitter_name:
                print(f"Skipping {folder_name}: Unable to extract submitter's name")
                continue
            
            temp_dir = os.path.join(output_dir, 'temp_extraction')
            os.makedirs(temp_dir, exist_ok=True)
            
            folder_contents = [f for f in main_zip.namelist() if f.startswith(f"{folder_name}/")]
            for file in folder_contents:
                main_zip.extract(file, temp_dir)
            
            extracted_folder_path = os.path.join(temp_dir, folder_name)
            for root, _, files in os.walk(extracted_folder_path):
                for file in files:
                    file_path = os.path.join(root, file)
                    process_file(file_path, submitter_name, output_dir)
            
            shutil.rmtree(temp_dir)

    print(f"Processing complete. Output directory: {output_dir}")

def main():
    parser = argparse.ArgumentParser(
        description="""
        Process a zip file containing folders with PDFs, HTML files, or ZIP files.
        
        This script takes a zip file as input, where the zip file contains multiple folders.
        Each folder is expected to be named with a submitter's name followed by an underscore and additional information.
        The script will process files from these folders as follows:
        - PDF and HTML files will be renamed based on the submitter's name and moved to the output directory.
        - ZIP files will be extracted into a folder named after the submitter in the output directory.
        
        The output directory is automatically named by appending 'out' to the input filename (without extension).
        For example, if the input is 'myarchive.zip', the output directory will be 'myarchiveout'.
        
        If no input file is specified, the script will look for a file named 'in.zip' in the current directory.
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("input_zip", nargs='?', default="in.zip", 
                        help="Path to the input zip file (default: in.zip)")
    
    args = parser.parse_args()

    input_name = os.path.splitext(args.input_zip)[0]
    output_dir = f"{input_name}out"

    if not os.path.exists(args.input_zip):
        print(f"Error: The file '{args.input_zip}' does not exist.")
        sys.exit(1)

    process_zip_file(args.input_zip, output_dir)

if __name__ == "__main__":
    main()