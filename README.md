# Moodle Assignment Zip Exploder

**Written collaboratively with claude.ai. I would say that it wrote all the code and I wrote the specifications. We iterated about ten times.**

* Process the zip file structured like Moodle gives you when you "download all assigments". It generates a zip file containing folders with PDFs, HTML, Python, Markdown, or ZIP files.
        
* This script takes a zip file as input, where the zip file contains multiple folders.

* Each folder is expected to be named with a submitter's name followed by an underscore and additional information.

* The script will process files from these folders as follows:
    * PDF, HTML, Python (.py), and Markdown (.md) files will be renamed based on the submitter's name and moved to the output directory.
    * ZIP files will be extracted into a folder named after the submitter in the output directory.
    * Other file types will be skipped.
        
* The output directory is automatically named by appending 'out' to the input filename (without extension).
    * For example, if the input is 'myarchive.zip', the output directory will be 'myarchiveout'.
    * If no input file is specified, the script will look for a file named 'in.zip' in the current directory.
