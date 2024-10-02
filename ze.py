import os
import zipfile
import argparse

def process_zip_file(main_zip_path, output_dir):
    # Create the output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    # Open the main zip file
    with zipfile.ZipFile(main_zip_path, 'r') as main_zip:
        # Iterate through all files in the main zip
        for file_name in main_zip.namelist():
            print(file_name)
            if file_name.endswith('.zip'):
                # Extract the person's name from the zip file name
                person_name = os.path.splitext(file_name)[0]
                print(person_name)
                
                # Extract the individual zip file
                main_zip.extract(file_name, output_dir)
                individual_zip_path = os.path.join(output_dir, file_name)
                
                # Open the individual zip file
                with zipfile.ZipFile(individual_zip_path, 'r') as individual_zip:
                    # Find the PDF file
                    pdf_files = [f for f in individual_zip.namelist() if f.lower().endswith('.pdf')]
                    
                    if pdf_files:
                        # Extract and rename the PDF file
                        pdf_file = pdf_files[0]
                        individual_zip.extract(pdf_file, output_dir)
                        old_pdf_path = os.path.join(output_dir, pdf_file)
                        new_pdf_path = os.path.join(output_dir, f"{person_name}.pdf")
                        os.rename(old_pdf_path, new_pdf_path)
                
                # Remove the individual zip file
                os.remove(individual_zip_path)

    print(f"Processing complete. Output directory: {output_dir}")

def main():
    # Set up argument parser
    parser = argparse.ArgumentParser(description="Process a zip file containing multiple zip files with PDFs.")
    parser.add_argument("input_zip", help="Path to the input zip file")
    parser.add_argument("output_dir", help="Path to the output directory")
    
    # Parse arguments
    args = parser.parse_args()

    # Call the process_zip_file function with the provided arguments
    process_zip_file(args.input_zip, args.output_dir)

if __name__ == "__main__":
    main()