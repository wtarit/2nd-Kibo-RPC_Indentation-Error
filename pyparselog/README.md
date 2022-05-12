# Python ADB log parsing program

This python is used to automatically unzip and read log file downloaded from JAXA simulation server. 
We save output in to tsv (Tab-separated values) format for importing in to Excel or Google Sheets for further analysis. 
The reason we used tsv instead of csv is to prevent confusion with comma in json output of QR-code reader (which we also logs).  

The main program is in parselog.py file. Others is for our testing purpose. We also include example input and output in log folder and result.tsv file respectively.